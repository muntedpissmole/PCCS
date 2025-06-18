from flask import Flask, render_template, request, jsonify
import board
import busio
from adafruit_pca9685 import PCA9685
import lgpio
import json
import time
import logging
import atexit
import serial
import pynmea2
from astral.sun import sun
from astral import LocationInfo
from datetime import datetime, date
import pytz
import subprocess
import threading
import os
from w1thermsensor import W1ThermSensor, NoSensorFoundError, SensorNotReadyError
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from logging.handlers import RotatingFileHandler
from threading import Lock

app = Flask(__name__)

# Setup logging with rotation
log_dir = 'log'
if not os.path.exists(log_dir):
    os.makedirs(log_dir, exist_ok=True)
file_handler = RotatingFileHandler(
    os.path.join(log_dir, 'app.log'),
    maxBytes=1_000_000,
    backupCount=5
)
file_handler.setLevel(logging.DEBUG)
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[logging.StreamHandler(), file_handler]
)

# Global GPIO handle
h = None
gps_lock = threading.Lock()
config_lock = Lock()

# GPS setup
try:
    ser = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=5)
    logging.info("Initialized GPS serial connection")
except Exception as e:
    logging.error(f"Error initializing GPS: {e}")
    ser = None

# Initialize I2C bus
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    logging.info("Initialized I2C bus")
except Exception as e:
    logging.error(f"Error initializing I2C bus: {e}")
    i2c = None

# Initialize ADS1115
try:
    if i2c:
        ads = ADS.ADS1115(i2c, address=0x49)
        ads.gain = 2/3
        chan = AnalogIn(ads, ADS.P0)
        chan_tank = AnalogIn(ads, ADS.P1)
        logging.info("Initialized ADS1115")
    else:
        ads = chan = chan_tank = None
except Exception as e:
    logging.error(f"Error initializing ADS1115: {e}")
    ads = chan = chan_tank = None

# Sensor caches
last_battery_voltage = last_battery_voltage_time = None
last_tank_level = last_tank_level_time = None
last_temperature = last_temperature_time = None
CACHE_DURATION = 10

# Initialize GPIO
def init_gpio():
    global h
    try:
        h = lgpio.gpiochip_open(0)  # Open GPIO chip
        logging.info("Initialized GPIO chip")
        
        # Configure reed switch pins as inputs
        reed_switch_pins = [int(pin) for pin in config['reed_switches'].keys()]
        for pin in reed_switch_pins:
            lgpio.gpio_claim_input(h, pin, lgpio.SET_PULL_UP)
            logging.debug(f"Configured GPIO pin {pin} as input with pull-up")
        
        # Configure relay pins as outputs
        relay_pins = [int(pin) for pin in config['channels']['relays'].keys()]
        for pin in relay_pins:
            lgpio.gpio_claim_output(h, pin, 1)  # Default to off (relays are active-low)
            logging.debug(f"Configured GPIO pin {pin} as output")
        
        # Apply saved relay states
        relay_states = load_relay_states()
        for pin, state in relay_states.items():
            if pin in config['channels']['relays']:
                lgpio.gpio_write(h, int(pin), 0 if state == 1 else 1)  # Active-low
                logging.debug(f"Restored relay state for pin {pin}: {'on' if state == 1 else 'off'}")
        
        atexit.register(cleanup_gpio)  # Register cleanup on exit
    except Exception as e:
        logging.error(f"Error initializing GPIO: {e}")
        h = None

def cleanup_gpio():
    global h
    if h is not None:
        try:
            # Release all claimed pins
            reed_switch_pins = [int(pin) for pin in config['reed_switches'].keys()]
            relay_pins = [int(pin) for pin in config['channels']['relays'].keys()]
            for pin in reed_switch_pins + relay_pins:
                lgpio.gpio_free(h, pin)
            lgpio.gpiochip_close(h)
            logging.info("Cleaned up GPIO resources")
        except Exception as e:
            logging.error(f"Error cleaning up GPIO: {e}")
        h = None

def get_battery_voltage():
    global last_battery_voltage, last_battery_voltage_time
    if chan is None:
        return None
    current_time = time.time()
    if last_battery_voltage is not None and (current_time - last_battery_voltage_time) < CACHE_DURATION:
        return last_battery_voltage
    try:
        battery_voltage = round(chan.voltage * 5.0, 2)
        if battery_voltage < 0.5 or battery_voltage > 15.0:
            logging.warning(f"Battery voltage out of range: {battery_voltage}V")
            last_battery_voltage = None
            last_battery_voltage_time = current_time
            return None
        last_battery_voltage = battery_voltage
        last_battery_voltage_time = current_time
        return battery_voltage
    except Exception as e:
        logging.error(f"Error reading battery voltage: {e}")
        return None

def get_tank_level():
    global last_tank_level, last_tank_level_time
    if chan_tank is None:
        return None
    current_time = time.time()
    if last_tank_level is not None and (current_time - last_tank_level_time) < CACHE_DURATION:
        return last_tank_level
    try:
        v_out = chan_tank.voltage
        v_in = 5.0
        if v_out < 0.5 or v_out > 3.0:
            logging.warning(f"Tank level sensor fault: V_out={v_out:.3f}V")
            last_tank_level = None
            last_tank_level_time = current_time
            return None
        r1 = 100.0
        r2 = r1 * (v_in - v_out) / v_out if v_out > 0 else float('inf')
        if r2 < 30 or r2 > 250:
            logging.warning(f"Tank level sensor fault: r2={r2:.1f}Ω")
            last_tank_level = None
            last_tank_level_time = current_time
            return None
        r2 = max(33, min(240, r2))
        percentage = round(((r2 - 33) / (240 - 33)) * 100)
        last_tank_level = percentage
        last_tank_level_time = current_time
        return percentage
    except Exception as e:
        logging.error(f"Error reading tank level: {e}")
        return None

def get_ds18b20_temperature():
    global last_temperature, last_temperature_time
    current_time = time.time()
    if last_temperature is not None and (current_time - last_temperature_time) < CACHE_DURATION:
        return last_temperature
    try:
        sensor = W1ThermSensor()
        temperature = round(sensor.get_temperature(), 1)
        if temperature < -20.0 or temperature > 80.0:
            logging.warning(f"Temperature out of range: {temperature}°C")
            last_temperature = None
            last_temperature_time = current_time
            return None
        last_temperature = temperature
        last_temperature_time = current_time
        return temperature
    except (NoSensorFoundError, SensorNotReadyError, Exception) as e:
        logging.error(f"Error reading DS18B20: {e}")
        return None

gps_data = {
    "fix": "No",
    "quality": "0 satellites",
    "satellites": 0,
    "latitude": None,
    "longitude": None
}

sun_times_cache = {
    "sunrise": "",
    "sunset": "",
    "last_calculated": None,
    "last_latitude": None,
    "last_longitude": None
}

current_time_cache = {
    "time": "",
    "last_updated": None,
    "fix_obtained": False,
    "using_gps": False
}

gps_timeout_start = None
GPS_TIMEOUT_MINUTES = 5

MELBOURNE_LOCATION = LocationInfo(
    name="Melbourne",
    region="Victoria",
    timezone="Australia/Melbourne",
    latitude=-37.8136,
    longitude=144.9631
)

SHUTDOWN_TOKEN = "kzqWazMQIO8YrefrqwEi4cFvM9pCrlCAYG05FLpjgpc"

def set_system_clock(dt):
    try:
        time_str = dt.strftime("%Y-%m-%d %H:%M:%S")
        subprocess.run(["sudo", "date", "-s", time_str], check=True)
        logging.info(f"Set system clock to {time_str}")
    except Exception as e:
        logging.error(f"Error setting system clock: {e}")

def save_relay_states(states):
    with config_lock:
        try:
            with open('config.json', 'r') as f:
                current_config = json.load(f)
            current_config['relay_states'] = states
            with open('config.json', 'w') as f:
                json.dump(current_config, f, indent=4)
            logging.debug(f"Saved relay states: {states}")
        except Exception as e:
            logging.error(f"Error saving relay states: {e}")

def load_relay_states():
    try:
        with open('config.json', 'r') as f:
            config = json.load(f)
        return config.get('relay_states', {})
    except Exception as e:
        logging.error(f"Error loading relay states: {e}")
        return {}

def update_gps_data():
    global gps_data, current_time_cache, gps_timeout_start, sun_times_cache
    if ser is None:
        return
    try:
        while True:
            with gps_lock:
                if not current_time_cache["time"]:
                    system_time = datetime.now(pytz.timezone("Australia/Melbourne"))
                    current_time_cache["time"] = system_time.strftime("%A %B %d %Y %I:%M %p %Z").lstrip("0").replace(" 0", " ") + "*"
                    current_time_cache["last_updated"] = datetime.now(pytz.UTC)
                    current_time_cache["using_gps"] = False
                if not sun_times_cache["sunrise"]:
                    local_tz = pytz.timezone("Australia/Melbourne")
                    today = date.today()
                    s = sun(MELBOURNE_LOCATION.observer, date=today, tzinfo=local_tz)
                    sun_times_cache.update({
                        "sunrise": s["sunrise"].strftime("%I:%M %p").lstrip("0") + "*",
                        "sunset": s["sunset"].strftime("%I:%M %p").lstrip("0") + "*",
                        "last_calculated": datetime.now(pytz.UTC),
                        "last_latitude": MELBOURNE_LOCATION.latitude,
                        "last_longitude": MELBOURNE_LOCATION.longitude
                    })
                ser.reset_input_buffer()
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line and ('$GPGGA' in line or '$GNGGA' in line):
                    try:
                        msg = pynmea2.parse(line)
                        fix_quality = int(msg.data[5])
                        num_sats = int(msg.data[6]) if msg.data[6].isdigit() else 0
                        latitude = msg.latitude
                        longitude = msg.longitude
                        gps_data.update({
                            "fix": "Yes" if fix_quality in (1, 2) else "No",
                            "quality": f"{num_sats} satellites" if fix_quality == 1 else f"Differential ({num_sats} satellites)" if fix_quality == 2 else "0 satellites",
                            "satellites": num_sats,
                            "latitude": latitude,
                            "longitude": longitude
                        })
                        if fix_quality == 0:
                            gps_timeout_start = datetime.now() if gps_timeout_start is None else gps_timeout_start
                        else:
                            gps_timeout_start = None
                            current_time_cache["using_gps"] = True
                        if hasattr(msg, 'timestamp') and msg.timestamp and gps_data["fix"] == "Yes":
                            today = date.today()
                            naive_dt = datetime(
                                year=today.year,
                                month=today.month,
                                day=today.day,
                                hour=msg.timestamp.hour,
                                minute=msg.timestamp.minute
                            )
                            utc_dt = pytz.utc.localize(naive_dt)
                            local_dt = utc_dt.astimezone(pytz.timezone("Australia/Melbourne"))
                            current_time_cache["time"] = local_dt.strftime("%A %B %d %Y %I:%M %p %Z").lstrip("0").replace(" 0", " ")
                            current_time_cache["last_updated"] = datetime.now(pytz.UTC)
                            if not current_time_cache["fix_obtained"]:
                                current_time_cache["fix_obtained"] = True
                                set_system_clock(local_dt)
                    except pynmea2.ParseError:
                        pass
            time.sleep(1)
    except Exception as e:
        logging.error(f"Error updating GPS data: {e}")

gps_thread = threading.Thread(target=update_gps_data, daemon=True)
gps_thread.start()

# Load config
config_path = 'config.json'
try:
    if not os.path.exists(config_path):
        default_config = {
            "theme": {
                "darkMode": "off",
                "autoTheme": "off",
                "defaultTheme": "light",
                "screenBrightness": "50"
            },
            "channels": {
                "pca9685": {},
                "relays": {}
            },
            "scenes": {},
            "reed_switches": {
                "17": {
                    "name": "Storage Panel",
                    "icon": "fa-box",
                    "display": true
                },
                "18": {
                    "name": "Rear Drawer",
                    "icon": "fa-box-open",
                    "display": true
                }
            },
            "relay_states": {}
        }
        with open(config_path, 'w') as f:
            json.dump(default_config, f, indent=4)
        logging.info(f"Created default config.json with reed switches")
    with open(config_path, 'r') as f:
        config = json.load(f)
    # Ensure required fields exist
    if 'theme' not in config:
        config['theme'] = {
            "darkMode": "off",
            "autoTheme": "off",
            "defaultTheme": "light",
            "screenBrightness": "50"
        }
    if 'channels' not in config:
        config['channels'] = {"pca9685": {}, "relays": {}}
    if 'relays' not in config['channels']:
        config['channels']['relays'] = {}
    if 'pca9685' not in config['channels']:
        config['channels']['pca9685'] = {}
    if 'scenes' not in config:
        config['scenes'] = {}
    if 'reed_switches' not in config:
        config['reed_switches'] = {
            "17": {
                "name": "Storage Panel",
                "icon": "fa-box",
                "display": true
            },
            "18": {
                "name": "Rear Drawer",
                "icon": "fa-box-open",
                "display": true
            }
        }
    if 'relay_states' not in config:
        config['relay_states'] = {}
    # Log reed_switches for debugging
    logging.info(f"Loaded reed_switches: {config['reed_switches']}")
    with config_lock:
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=4)
        logging.info(f"Updated config.json with required fields")
except Exception as e:
    logging.error(f"Error loading config.json: {e}")
    config = {
        "theme": {
            "darkMode": "off",
            "autoTheme": "off",
            "defaultTheme": "light",
            "screenBrightness": "50"
        },
        "channels": {
            "pca9685": {},
            "relays": {}
        },
        "scenes": {},
        "reed_switches": {
            "17": {
                "name": "Storage Panel",
                "icon": "fa-box",
                "display": true
            },
            "18": {
                "name": "Rear Drawer",
                "icon": "fa-box-open",
                "display": true
            }
        },
        "relay_states": {}
    }
    logging.info(f"Falling back to default config with reed switches: {config['reed_switches']}")

# Initialize GPIO after config is loaded
init_gpio()

# Global PCA9685 status
pca_available = False

# Initialize PCA9685
try:
    if i2c:
        pca = PCA9685(i2c, address=0x40)
        pca.frequency = 60
        pca_available = True
        logging.info("Initialized PCA9685")
    else:
        pca = None
        logging.error("I2C bus not initialized, PCA9685 unavailable")
except Exception as e:
    pca = None
    logging.error(f"Error initializing PCA9685: {e}")

@app.route('/')
def index():
    try:
        return render_template('index.html',
                              scenes=config['scenes'].keys(),
                              pca9685_channels=config['channels']['pca9685'],
                              relay_channels=config['channels']['relays'],
                              reed_switches=config['reed_switches'],
                              pca_available=pca_available)
    except Exception as e:
        logging.error(f"Error rendering index: {e}")
        return f"Error rendering page: {e}", 500

@app.route('/set_brightness', methods=['POST'])
def set_brightness():
    try:
        if not pca_available:
            logging.error("set_brightness called but PCA9685 not detected")
            return jsonify({"error": "PCA9685 not detected"}), 503
        data = request.json
        channel_type = data['type']
        channel = data['channel']
        brightness = int(data['brightness'])
        if channel_type == 'pca9685' and pca:
            target_pwm = int(brightness * 4095 / 100)
            channel_idx = int(channel) - 1
            current_pwm = pca.channels[channel_idx].duty_cycle
            steps = 10
            step_size = (target_pwm - current_pwm) / steps
            step_time = 0.05
            for i in range(steps):
                new_pwm = int(current_pwm + step_size * (i + 1))
                pca.channels[channel_idx].duty_cycle = max(0, min(4095, new_pwm))
                time.sleep(step_time)
            return jsonify({"message": f"Brightness set to {brightness}%"})
        return jsonify({"error": "Invalid channel type or PCA9685 not initialized"}), 400
    except Exception as e:
        logging.error(f"Error in set_brightness: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/toggle', methods=['POST'])
def toggle():
    try:
        data = request.json
        channel_type = data['type']
        channel = data['channel']
        state = data['state']
        if channel_type == 'pca9685':
            if not pca_available:
                logging.error("toggle called for PCA9685 but not detected")
                return jsonify({"error": "PCA9685 not detected"}), 503
            if pca:
                target_pwm = 4095 if state == 'on' else 0
                channel_idx = int(channel) - 1
                current_pwm = pca.channels[channel_idx].duty_cycle
                steps = 10
                step_size = (target_pwm - current_pwm) / steps
                step_time = 0.05
                for i in range(steps):
                    new_pwm = int(current_pwm + step_size * (i + 1))
                    pca.channels[channel_idx].duty_cycle = max(0, min(4095, new_pwm))
                    time.sleep(step_time)
                return jsonify({"message": f"Channel {channel} set to {state.upper()}"})
        elif channel_type == 'relays':
            if not h:
                logging.error("toggle called for relay but GPIO not initialized")
                return jsonify({"error": "GPIO not initialized"}), 503
            lgpio.gpio_write(h, int(channel), 0 if state == 'on' else 1)  # Active-low
            config['relay_states'][channel] = 1 if state == 'on' else 0
            save_relay_states(config['relay_states'])
            return jsonify({"message": f"Relay {channel} set to {state.upper()}"})
        return jsonify({"error": "Invalid channel type or hardware not initialized"}), 400
    except Exception as e:
        logging.error(f"Error in toggle: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/activate_scene', methods=['POST'])
def activate_scene():
    try:
        if not pca_available:
            logging.warning("activate_scene called but PCA9685 not detected, only applying relay actions")
        data = request.json
        scene_name = data['scene']
        scene = config['scenes'][scene_name]
        steps = 20
        step_time = 2.0 / steps
        current_pwm = {ch: pca.channels[int(ch) - 1].duty_cycle for ch in config['channels']['pca9685']} if pca else {}
        target_pwm = {ch: 0 for ch in config['channels']['pca9685']}
        for channel, brightness in scene.get('pca9685', {}).items():
            if channel in config['channels']['pca9685']:
                target_pwm[channel] = int(brightness * 4095 / 100)
        if pca_available and pca:
            for step in range(steps + 1):
                for channel in config['channels']['pca9685']:
                    channel_idx = int(channel) - 1
                    current = current_pwm[channel]
                    target = target_pwm[channel]
                    new_pwm = int(current + (target - current) * (step / steps))
                    pca.channels[channel_idx].duty_cycle = max(0, min(4095, new_pwm))
                time.sleep(step_time)
        config['relay_states'] = config.get('relay_states', {})
        if h:
            for pin, state in scene.get('relays', {}).items():
                if pin in config['channels']['relays']:
                    lgpio.gpio_write(h, int(pin), 1 if state == 0 else 0)  # Active-low
                    config['relay_states'][pin] = state
            for pin in config['channels']['relays']:
                if pin not in scene.get('relays', {}):
                    lgpio.gpio_write(h, int(pin), 1)  # Turn off relays not in scene
                    config['relay_states'][pin] = 0
            save_relay_states(config['relay_states'])
        else:
            logging.error("activate_scene called but GPIO not initialized")
            return jsonify({"error": "GPIO not initialized"}), 503
        return jsonify({"message": f"Scene {scene_name} activated"})
    except Exception as e:
        logging.error(f"Error in activate_scene: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/get_pca_states', methods=['GET'])
def get_pca_states():
    try:
        if not pca_available:
            logging.error("get_pca_states called but PCA9685 not detected")
            return jsonify({"error": "PCA9685 not detected"}), 503
        states = {}
        if pca:
            for channel in config['channels']['pca9685']:
                duty_cycle = pca.channels[int(channel) - 1].duty_cycle
                brightness = int(round(duty_cycle / 4095 * 100))
                states[channel] = brightness
        return jsonify(states)
    except Exception as e:
        logging.error(f"Error in get_pca_states: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/get_data', methods=['GET'])
def get_data():
    global sun_times_cache, current_time_cache
    try:
        temperature = get_ds18b20_temperature()
        battery_voltage = get_battery_voltage()
        tank_level = get_tank_level()
        current_time = datetime.now(pytz.UTC)
        recalculate = False
        if sun_times_cache["last_calculated"] is None or (current_time - sun_times_cache["last_calculated"]).total_seconds() > 3600:
            recalculate = True
        if gps_data["latitude"] and gps_data["longitude"]:
            if sun_times_cache["last_latitude"] is None or abs(gps_data["latitude"] - sun_times_cache["last_latitude"]) > 0.1 or abs(gps_data["longitude"] - sun_times_cache["last_longitude"]) > 0.1:
                recalculate = True
        if recalculate and gps_data["fix"] == "Yes" and gps_data["latitude"] and gps_data["longitude"]:
            local_tz = pytz.timezone("Australia/Melbourne")
            location = LocationInfo(
                name="Current Location",
                region="Unknown",
                timezone="UTC",
                latitude=gps_data["latitude"],
                longitude=gps_data["longitude"]
            )
            today = date.today()
            s = sun(location.observer, date=today, tzinfo=local_tz)
            sun_times_cache.update({
                "sunrise": s["sunrise"].astimezone(local_tz).strftime("%I:%M %p").lstrip("0"),
                "sunset": s["sunset"].astimezone(local_tz).strftime("%I:%M %p").lstrip("0"),
                "last_calculated": current_time,
                "last_latitude": gps_data["latitude"],
                "last_longitude": gps_data["longitude"]
            })
        data = {
            "temperature": str(temperature) if temperature is not None else "Error",
            "battery_level": f"{battery_voltage}V" if battery_voltage is not None else "Error",
            "tank_level": f"{tank_level}" if tank_level is not None else "Error",
            "sunrise": sun_times_cache["sunrise"] or "---",
            "sunset": sun_times_cache["sunset"] or "---",
            "current_datetime": current_time_cache["time"] or "---",
            "gps_fix": gps_data["fix"],
            "gps_quality": gps_data["quality"],
            "latitude": gps_data["latitude"],
            "longitude": gps_data["longitude"]
        }
        # Add reed switch states dynamically
        if h:
            for pin, switch_info in config['reed_switches'].items():
                name = switch_info['name'].lower().replace(" ", "_")
                state = "Open" if lgpio.gpio_read(h, int(pin)) else "Closed"
                data[name] = state
        else:
            for pin, switch_info in config['reed_switches'].items():
                name = switch_info['name'].lower().replace(" ", "_")
                data[name] = "Unknown"
        return jsonify(data)
    except Exception as e:
        logging.error(f"Error in get_data: {e}")
        return jsonify({}), 500

@app.route('/shutdown', methods=['POST'])
def shutdown():
    try:
        data = request.json or {}
        if data.get('token') != SHUTDOWN_TOKEN:
            return jsonify({"error": "Unauthorized"}), 403
        display_pi_user = "pi"  # Username for display-pi
        display_pi_ip = "10.10.10.20"  # IP for display-pi
        ssh_key = "/home/pi/.ssh/id_rsa_shutdown"  # SSH key for pi user
        try:
            subprocess.run([
                "ssh", "-i", ssh_key, f"{display_pi_user}@{display_pi_ip}",
                "sudo", "shutdown", "now"
            ], check=True, timeout=10)
            logging.info(f"Successfully sent shutdown command to display-pi at {display_pi_ip}")
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            logging.warning(f"Failed to shut down display-pi: {e}")
        cleanup_gpio()
        threading.Thread(target=lambda: [
            time.sleep(2),
            subprocess.run(['sudo', 'shutdown', 'now'], check=True)
        ], daemon=True).start()
        return jsonify({"message": "Both systems are shutting down"})
    except Exception as e:
        logging.error(f"Error in shutdown: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/save_config', methods=['POST'])
def save_config():
    try:
        config_data = request.json
        logging.debug(f"Received save_config request: {config_data}")
        with config_lock:
            config_path = 'config.json'
            if not os.access(config_path, os.W_OK):
                logging.error(f"No write permissions for {config_path}")
                return jsonify({"error": "Cannot write to config file"}), 500
            with open(config_path, 'r') as f:
                current_config = json.load(f)
            # Update theme
            if 'theme' in config_data:
                new_theme = {
                    "darkMode": config_data['theme'].get('darkMode', current_config['theme'].get('darkMode', 'off')),
                    "autoTheme": config_data['theme'].get('autoTheme', current_config['theme'].get('autoTheme', 'off')),
                    "defaultTheme": config_data['theme'].get('defaultTheme', current_config['theme'].get('defaultTheme', 'light')),
                    "screenBrightness": str(config_data['theme'].get('screenBrightness', current_config['theme'].get('screenBrightness', '50')))
                }
                logging.debug(f"Updating theme: {new_theme}")
                current_config['theme'] = new_theme
            else:
                logging.warning("No theme data in config_data, preserving existing theme")
            # Log the config before saving
            logging.debug(f"Config to be saved: {current_config}")
            # Save updated config
            with open(config_path, 'w') as f:
                json.dump(current_config, f, indent=4)
            logging.info(f"Saved theme config: {current_config['theme']}")
            return jsonify({"message": "Config saved"})
    except Exception as e:
        logging.error(f"Error saving config: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/load_config', methods=['GET'])
def load_config():
    try:
        with config_lock:
            config_path = 'config.json'
            if not os.access(config_path, os.R_OK):
                logging.error(f"No read permissions for {config_path}")
                return jsonify({
                    "theme": {
                        "darkMode": "off",
                        "autoTheme": "off",
                        "defaultTheme": "light",
                        "screenBrightness": "50"
                    },
                    "screenBrightness": 50
                }), 500
            with open(config_path, 'r') as f:
                config_data = json.load(f)
            theme = config_data.get('theme', {
                "darkMode": "off",
                "autoTheme": "off",
                "defaultTheme": "light",
                "screenBrightness": "50"
            })
            try:
                screen_brightness = int(theme.get('screenBrightness', '50'))
                screen_brightness = max(10, min(100, screen_brightness))
            except (ValueError, TypeError):
                screen_brightness = 50
            response = {
                "theme": {
                    "darkMode": theme.get("darkMode", "off"),
                    "autoTheme": theme.get("autoTheme", "off"),
                    "defaultTheme": theme.get("defaultTheme", "light"),
                    "screenBrightness": str(screen_brightness)
                },
                "screenBrightness": screen_brightness  # For backward compatibility
            }
            logging.debug(f"Returning config: {response}")
            return jsonify(response)
    except Exception as e:
        logging.error(f"Error in load_config: {e}")
        return jsonify({
            "theme": {
                "darkMode": "off",
                "autoTheme": "off",
                "defaultTheme": "light",
                "screenBrightness": "50"
            },
            "screenBrightness": 50
        }), 500

@app.route('/get_scenes', methods=['GET'])
def get_scenes():
    try:
        with config_lock:
            config_path = 'config.json'
            if not os.access(config_path, os.R_OK):
                logging.error(f"No read permissions for {config_path}")
                return jsonify({}), 500
            with open(config_path, 'r') as f:
                config_data = json.load(f)
            scenes = config_data.get('scenes', {})
            logging.debug(f"Returning scenes: {scenes}")
            return jsonify(scenes)
    except Exception as e:
        logging.error(f"Error in get_scenes: {e}")
        return jsonify({}), 500
        
@app.route('/set_screen_brightness', methods=['POST'])
def set_screen_brightness():
    try:
        data = request.json
        brightness_level = data.get('brightness')
        
        # Map brightness levels to values (0-255 range)
        brightness_map = {
            'low': 25,    # 10% of 255
            'medium': 127, # 50% of 255
            'high': 255   # 100% of 255
        }
        
        if brightness_level not in brightness_map:
            logging.error(f"Invalid brightness level: {brightness_level}")
            return jsonify({"error": "Invalid brightness level"}), 400
        
        brightness_value = brightness_map[brightness_level]
        display_pi_user = "pi"  # Username for display-pi
        display_pi_ip = "10.10.10.20"  # IP for display-pi
        ssh_key = "/home/pi/.ssh/id_rsa_shutdown"  # SSH key for pi user
        
        # Construct the SSH command
        ssh_command = (
            f"echo {brightness_value} | sudo tee /sys/class/backlight/*/brightness"
        )
        
        try:
            # Execute SSH command to set brightness on display-pi
            subprocess.run([
                "ssh", "-i", ssh_key, f"{display_pi_user}@{display_pi_ip}",
                ssh_command
            ], check=True, timeout=10, capture_output=True, text=True)
            logging.info(f"Successfully set display-pi brightness to {brightness_level} ({brightness_value})")
            return jsonify({"message": f"Brightness set to {brightness_level} ({brightness_value})"})
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            logging.error(f"Failed to set brightness on display-pi: {e}")
            return jsonify({"error": f"Failed to set brightness: {str(e)}"}), 500
            
    except Exception as e:
        logging.error(f"Error in set_screen_brightness: {e}")
        return jsonify({"error": str(e)}), 500
        
import time
import subprocess
from threading import Lock

def send_ssh_display_command(display_pi_user, display_pi_ip, ssh_key, command, action_desc, brightness_value=None):
    """
    Sends an SSH command to the display-pi, handling screen off (xset) or wake (xdotool) and optional brightness restore.
    Returns True on success, False on failure.
    """
    try:
        cmd = ["ssh", "-i", ssh_key, f"{display_pi_user}@{display_pi_ip}"]
        # Only prepend DISPLAY=:0 for xdotool, not for xset
        if command[0] == "xdotool":
            cmd.append("DISPLAY=:0 " + " ".join(command))
        else:
            cmd.append(" ".join(command))
        
        logging.debug(f"Executing SSH command: {' '.join(cmd)}")
        result = subprocess.run(
            cmd, check=True, timeout=20, capture_output=True, text=True
        )
        logging.info(f"Successfully sent {action_desc} command to display-pi: {result.stdout.strip()}")
        
        # If brightness_value is provided (for wake), restore brightness
        if brightness_value is not None:
            ssh_command = f"echo {brightness_value} | sudo tee /sys/class/backlight/*/brightness"
            logging.debug(f"Executing brightness SSH command: {ssh_command}")
            result = subprocess.run(
                ["ssh", "-i", ssh_key, f"{display_pi_user}@{display_pi_ip}", ssh_command],
                check=True, timeout=20, capture_output=True, text=True
            )
            logging.info(f"Restored brightness to {brightness_value}: {result.stdout.strip()}")
        
        return True
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
        error_msg = e.stderr.strip() if hasattr(e, 'stderr') and e.stderr else str(e)
        logging.error(f"Failed to send {action_desc} command to display-pi: {error_msg}, cmd: {' '.join(cmd)}")
        
        # Fallback to backlight control if xset or xdotool fails
        if command[0] in ["xset", "xdotool"]:
            fallback_value = 0 if command[0] == "xset" and command[-1] == "off" else (brightness_value if brightness_value else 127)
            logging.info(f"Attempting fallback: set backlight to {fallback_value}")
            try:
                ssh_command = f"echo {fallback_value} | sudo tee /sys/class/backlight/*/brightness"
                logging.debug(f"Executing fallback SSH command: {ssh_command}")
                result = subprocess.run(
                    ["ssh", "-i", ssh_key, f"{display_pi_user}@{display_pi_ip}", ssh_command],
                    check=True, timeout=20, capture_output=True, text=True
                )
                logging.info(f"Fallback succeeded: set backlight to {fallback_value}: {result.stdout.strip()}")
                return True
            except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
                error_msg = e.stderr.strip() if hasattr(e, 'stderr') and e.stderr else str(e)
                logging.error(f"Fallback failed: {error_msg}, cmd: {ssh_command}")
        return False

def check_initial_reed_state():
    """
    Checks the initial state of the Kitchen Panel reed switch after GPIO initialization
    and sends an SSH command to sync the display-pi screen state.
    """
    kitchen_pin = None
    for pin, switch_info in config['reed_switches'].items():
        if switch_info['name'].lower() == "kitchen panel":
            kitchen_pin = int(pin)
            break
    
    if kitchen_pin is None:
        logging.error("Kitchen Panel reed switch not found in config")
        return None
    
    if h is None:
        logging.error("GPIO not initialized, cannot check Kitchen Panel reed switch")
        return None
    
    display_pi_user = "pi"
    display_pi_ip = "10.10.10.20"
    ssh_key = "/home/pi/.ssh/id_rsa_shutdown"
    
    # Read initial state
    try:
        initial_state = lgpio.gpio_read(h, kitchen_pin)
        logging.info(f"Initial Kitchen Panel state: {'closed' if initial_state == 0 else 'open'} (GPIO {kitchen_pin})")
        
        with config_lock:
            # Prepare command based on state
            if initial_state == 0:  # Closed
                command = ["xset", "dpms", "force", "off"]
                action_desc = "screen off"
                brightness_value = None
                config['reed_switches'][str(kitchen_pin)]['state'] = "closed"
            else:  # Open
                command = ["xdotool", "key", "Shift"]
                action_desc = "screen wake"
                brightness_level = config['theme'].get('screenBrightness', 'medium')
                brightness_map = {'low': 25, 'medium': 127, 'high': 255}
                brightness_value = brightness_map.get(brightness_level, 127)
                config['reed_switches'][str(kitchen_pin)]['state'] = "open"
            
            # Try sending the command with retries
            max_attempts = 3
            retry_interval = 10  # Seconds between retries
            for attempt in range(max_attempts):
                if send_ssh_display_command(display_pi_user, display_pi_ip, ssh_key, command, action_desc, brightness_value):
                    break
                logging.warning(f"SSH attempt {attempt + 1}/{max_attempts} failed, retrying in {retry_interval} seconds")
                time.sleep(retry_interval)
            else:
                logging.error("All SSH attempts failed, could not sync initial screen state")
            
            # Save updated config
            try:
                with open('config.json', 'w') as f:
                    json.dump(config, f, indent=4)
                logging.debug(f"Updated config with initial Kitchen Panel state: {config['reed_switches'][str(kitchen_pin)]['state']}")
            except Exception as e:
                logging.error(f"Failed to save config.json: {e}")
        
        return initial_state
    except Exception as e:
        logging.error(f"Error reading initial Kitchen Panel reed switch state: {e}")
        return None

def monitor_kitchen_reed_switch(initial_state):
    """
    Monitors the Kitchen Panel reed switch (GPIO 23) and sends SSH commands to turn off the
    display-pi screen when closed and wake it with last brightness when opened.
    """
    kitchen_pin = None
    for pin, switch_info in config['reed_switches'].items():
        if switch_info['name'].lower() == "kitchen panel":
            kitchen_pin = int(pin)
            break
    
    if kitchen_pin is None:
        logging.error("Kitchen Panel reed switch not found in config")
        return
    
    if h is None:
        logging.error("GPIO not initialized, cannot monitor Kitchen Panel reed switch")
        return
    
    display_pi_user = "pi"
    display_pi_ip = "10.10.10.20"
    ssh_key = "/home/pi/.ssh/id_rsa_shutdown"
    last_state = initial_state  # Use initial state from check
    debounce_count = 0
    DEBOUNCE_THRESHOLD = 2  # Require 2 consistent readings
    last_read_state = None
    
    logging.info(f"Starting Kitchen Panel reed switch monitor on GPIO {kitchen_pin} with initial state: {last_state}")
    
    while True:
        try:
            current_read = lgpio.gpio_read(h, kitchen_pin)
            if current_read == last_read_state:
                debounce_count += 1
            else:
                debounce_count = 0
                last_read_state = current_read
            
            if debounce_count >= DEBOUNCE_THRESHOLD and current_read != last_state:
                with config_lock:
                    if current_read == 0:  # Reed switch closed
                        logging.info("Kitchen Panel reed switch closed, turning off display-pi screen")
                        command = ["xset", "dpms", "force", "off"]
                        action_desc = "screen off"
                        brightness_value = None
                        config['reed_switches'][str(kitchen_pin)]['state'] = "closed"
                    else:  # Reed switch opened
                        logging.info("Kitchen Panel reed switch opened, waking display-pi screen")
                        command = ["xdotool", "key", "Shift"]
                        action_desc = "screen wake"
                        brightness_level = config['theme'].get('screenBrightness', 'medium')
                        brightness_map = {'low': 25, 'medium': 127, 'high': 255}
                        brightness_value = brightness_map.get(brightness_level, 127)
                        config['reed_switches'][str(kitchen_pin)]['state'] = "open"
                    
                    if send_ssh_display_command(display_pi_user, display_pi_ip, ssh_key, command, action_desc, brightness_value):
                        last_state = current_read
                    else:
                        logging.error(f"Failed to update screen state, keeping last_state as {last_state}")
                    
                    # Save updated config
                    try:
                        with open('config.json', 'w') as f:
                            json.dump(config, f, indent=4)
                        logging.debug(f"Updated config with Kitchen Panel state: {config['reed_switches'][str(kitchen_pin)]['state']}")
                    except Exception as e:
                        logging.error(f"Failed to save config.json: {e}")
            
            time.sleep(0.5)  # Poll every 0.5 seconds
        except Exception as e:
            logging.error(f"Error monitoring Kitchen Panel reed switch: {e}")
            time.sleep(5)  # Wait before retrying
            
# Perform initial reed switch check and start monitoring
initial_state = None
try:
    initial_state = check_initial_reed_state()
except Exception as e:
    logging.error(f"Failed to check initial reed state: {e}")

# Start the reed switch monitoring thread with the initial state
reed_switch_thread = threading.Thread(target=monitor_kitchen_reed_switch, args=(initial_state,), daemon=True)
reed_switch_thread.start()

if __name__ == '__main__':
    try:
        logging.info("Starting Flask app on port 5000")
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
    except Exception as e:
        logging.error(f"Error starting Flask app: {e}")
        cleanup_gpio()
        raise  # Re-raise to see the error in console
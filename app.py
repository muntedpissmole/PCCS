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
from datetime import datetime, date, timedelta
import pytz
import subprocess
import threading
import os
from w1thermsensor import W1ThermSensor, NoSensorFoundError, SensorNotReadyError
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import math

app = Flask(__name__)

# Setup logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('app.log')
    ]
)

# Global GPIO handle
h = None

# Relay state file
RELAY_STATE_FILE = 'relay_states.json'

# GPS setup
try:
    ser = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=5)
    logging.info("Initialized GPS serial connection on /dev/ttyAMA0")
except Exception as e:
    logging.error(f"Error initializing GPS serial connection: {e}")
    ser = None

# Initialize I2C bus
try:
    i2c = busio.I2C(board.SCL, board.SDA)
except Exception as e:
    logging.error(f"Error initializing I2C bus: {e}")
    raise

# Initialize ADS1115
try:
    ads = ADS.ADS1115(i2c, address=0x49)
    ads.gain = 2/3  # ±6.144V range, suitable for 0–5V input
    chan = AnalogIn(ads, ADS.P0)  # Channel A0 for voltage
    chan_tank = AnalogIn(ads, ADS.P1)  # Channel A1 for tank level
    logging.info("Initialized ADS1115 at address 0x49")
except Exception as e:
    logging.error(f"Error initializing ADS1115: {e}")
    ads = None
    chan = None
    chan_tank = None

# Battery voltage cache
last_battery_voltage = None
last_battery_voltage_time = 0
BATTERY_VOLTAGE_CACHE_DURATION = 10  # seconds

# Tank level cache
last_tank_level = None
last_tank_level_time = 0
TANK_LEVEL_CACHE_DURATION = 10  # seconds

# Function to read battery voltage
def get_battery_voltage():
    global last_battery_voltage, last_battery_voltage_time
    current_time = time.time()
    if last_battery_voltage is not None and (current_time - last_battery_voltage_time) < BATTERY_VOLTAGE_CACHE_DURATION:
        logging.debug(f"Using cached battery voltage: {last_battery_voltage}V")
        return last_battery_voltage
    if chan is None:
        logging.error("ADS1115 not initialized, cannot read battery voltage")
        return None
    try:
        raw_voltage = chan.voltage
        battery_voltage = raw_voltage * 5.0  # 5:1 voltage divider
        last_battery_voltage = round(battery_voltage, 2)
        last_battery_voltage_time = current_time
        logging.debug(f"Battery voltage: {last_battery_voltage}V (raw: {raw_voltage}V)")
        return last_battery_voltage
    except Exception as e:
        logging.error(f"Error reading battery voltage: {e}")
        return None

# Function to read tank level
def get_tank_level():
    global last_tank_level, last_tank_level_time
    current_time = time.time()
    if last_tank_level is not None and (current_time - last_tank_level_time) < TANK_LEVEL_CACHE_DURATION:
        logging.debug(f"Using cached tank level: {last_tank_level}%")
        return last_tank_level
    if chan_tank is None:
        logging.error("ADS1115 not initialized, cannot read tank level")
        return None
    try:
        v_out = chan_tank.voltage
        v_in = 5.0  # 5V supply
        # Log raw voltage and ADC value for debugging
        logging.debug(f"Tank level raw V_out: {v_out:.3f}V, raw ADC value: {chan_tank.value}")
        # Check for invalid voltage (short or open circuit)
        if v_out < 0.7 or v_out > 3.3:
            logging.warning(f"Tank level sensor fault: Invalid voltage (V_out: {v_out:.3f}V)")
            last_tank_level = None
            last_tank_level_time = current_time
            return None
        r1 = 150.0  # Fixed resistor in ohms
        r2 = r1 * (v_in - v_out) / v_out if v_out > 0 else float('inf')  # Sensor resistance
        # Check for invalid resistance (out of expected range)
        if r2 < 30 or r2 > 250:
            logging.warning(f"Tank level sensor fault: Invalid resistance (r2: {r2:.1f}Ω)")
            last_tank_level = None
            last_tank_level_time = current_time
            return None
        r2 = max(33, min(240, r2))  # Clamp to 33–240Ω
        percentage = ((r2 - 33) / (240 - 33)) * 100  # 0% at 33Ω, 100% at 240Ω
        last_tank_level = round(percentage)  # Round to integer
        last_tank_level_time = current_time
        logging.debug(f"Tank level: {last_tank_level}% (resistance: {r2:.1f}Ω, V_out: {v_out:.3f}V)")
        return last_tank_level
    except Exception as e:
        logging.error(f"Error reading tank level: {e}")
        return None

# Global variable to store GPS data
gps_data = {
    "fix": "No",
    "quality": "0 satellites",
    "satellites": 0,
    "latitude": None,
    "longitude": None
}

# Cache for sunrise/sunset times
sun_times_cache = {
    "sunrise": "",
    "sunset": "",
    "last_calculated": None,
    "last_latitude": None,
    "last_longitude": None
}

# Cache for current time
current_time_cache = {
    "time": "",
    "last_updated": None,
    "fix_obtained": False,
    "using_gps": False
}

# Lock for thread-safe updates
gps_lock = threading.Lock()

# Timeout tracking
gps_timeout_start = None
GPS_TIMEOUT_MINUTES = 5
GPS_CHECK_INTERVAL_MINUTES = 5

# Melbourne fallback coordinates
MELBOURNE_LOCATION = LocationInfo(
    name="Melbourne",
    region="Victoria",
    timezone="Australia/Melbourne",
    latitude=-37.8136,
    longitude=144.9631
)

# Shutdown security token
SHUTDOWN_TOKEN = "your-secret-token"

# Temperature cache
last_temperature = None
last_temperature_time = 0
TEMPERATURE_CACHE_DURATION = 10

def set_system_clock(dt):
    try:
        time_str = dt.strftime("%Y-%m-%d %H:%M:%S")
        subprocess.run(["sudo", "date", "-s", time_str], check=True)
        logging.info(f"Set RPi system clock to {dt.strftime('%Y-%m-%d %H:%M:%S')}")
    except subprocess.CalledProcessError as e:
        logging.error(f"Error setting system clock with date: {e}")
    except Exception as e:
        logging.error(f"Unexpected error setting system clock: {e}")

def save_relay_states(states):
    try:
        with open(RELAY_STATE_FILE, 'w') as f:
            json.dump(states, f)
        logging.debug(f"Saved relay states: {states}")
    except Exception as e:
        logging.error(f"Error saving relay states: {e}")

def load_relay_states():
    try:
        if os.path.exists(RELAY_STATE_FILE):
            with open(RELAY_STATE_FILE, 'r') as f:
                states = json.load(f)
            logging.debug(f"Loaded relay states: {states}")
            return states
        else:
            logging.warning(f"No relay state file found at {RELAY_STATE_FILE}, using default OFF states")
            return None
    except Exception as e:
        logging.error(f"Error loading relay states: {e}")
        return None

def get_ds18b20_temperature():
    global last_temperature, last_temperature_time
    current_time = time.time()
    if last_temperature is not None and (current_time - last_temperature_time) < TEMPERATURE_CACHE_DURATION:
        logging.debug(f"Using cached DS18B20 temperature: {last_temperature}°C")
        return last_temperature
    try:
        sensor = W1ThermSensor()
        logging.info("Attempting to read DS18B20 sensor")
        temperature = sensor.get_temperature()
        logging.debug(f"DS18B20 temperature: {temperature}°C")
        last_temperature = round(temperature, 1)
        last_temperature_time = current_time
        return last_temperature
    except NoSensorFoundError:
        logging.error("No DS18B20 sensor found. Check wiring, GPIO configuration, or 1-Wire modules.")
        return None
    except SensorNotReadyError:
        logging.error("DS18B20 sensor not ready. Retrying may help.")
        return None
    except Exception as e:
        logging.error(f"Unexpected error reading DS18B20: {e}")
        return None

def update_gps_data():
    global gps_data, current_time_cache, gps_timeout_start, sun_times_cache
    if ser is None:
        logging.warning("Serial connection is None, skipping GPS update")
        return
    logging.debug("update_gps_data thread running")
    try:
        while True:
            with gps_lock:
                if not current_time_cache["time"]:
                    system_time = datetime.now(pytz.timezone("Australia/Melbourne"))
                    formatted_time = system_time.strftime("%A %B %d %Y %I:%M %p %Z").lstrip("0").replace(" 0", " ") + "*"
                    current_time_cache["time"] = formatted_time
                    current_time_cache["last_updated"] = datetime.now(pytz.UTC)
                    current_time_cache["using_gps"] = False
                    logging.info(f"Initial fallback to system time: {formatted_time}")

                if not sun_times_cache["sunrise"] or not sun_times_cache["sunset"]:
                    local_tz = pytz.timezone("Australia/Melbourne")
                    today = date.today()
                    s = sun(MELBOURNE_LOCATION.observer, date=today, tzinfo=local_tz)
                    sunrise = s["sunrise"].strftime("%I:%M %p").lstrip("0")
                    sunset = s["sunset"].strftime("%I:%M %p").lstrip("0")
                    sun_times_cache.update({
                        "sunrise": sunrise + "*",
                        "sunset": sunset + "*",
                        "last_calculated": datetime.now(pytz.UTC),
                        "last_latitude": MELBOURNE_LOCATION.latitude,
                        "last_longitude": MELBOURNE_LOCATION.longitude
                    })
                    logging.info(f"Initial fallback sunrise: {sunrise}*, sunset: {sunset}* using Melbourne coordinates")

                if gps_data["fix"] != "Yes" and gps_timeout_start:
                    elapsed_minutes = (datetime.now() - gps_timeout_start).total_seconds() / 60
                    if elapsed_minutes >= GPS_TIMEOUT_MINUTES:
                        system_time = datetime.now(pytz.timezone("Australia/Melbourne"))
                        formatted_time = system_time.strftime("%A %B %d %Y %I:%M %p %Z").lstrip("0").replace(" 0", " ") + "*"
                        current_time_cache["time"] = formatted_time
                        current_time_cache["last_updated"] = datetime.now(pytz.UTC)
                        current_time_cache["using_gps"] = False
                        logging.info(f"No GPS fix after {GPS_TIMEOUT_MINUTES} minutes, falling back to system time: {formatted_time}")

                ser.reset_input_buffer()
                line = ser.readline().decode('ascii', errors='ignore').strip()
                logging.debug(f"Raw GPS data: {line}")
                if line and ('$GPGGA' in line or '$GNGGA' in line or '$GPRMC' in line):
                    msg = pynmea2.parse(line)
                    logging.debug(f"Parsed message attributes: {vars(msg)}")
                    
                    if '$GPGGA' in line or '$GNGGA' in line:
                        if hasattr(msg, 'data') and len(msg.data) >= 7:
                            fix_quality = int(msg.data[5])
                            num_sats = int(msg.data[6]) if msg.data[6].isdigit() else 0
                            latitude = msg.latitude if hasattr(msg, 'latitude') else None
                            longitude = msg.longitude if hasattr(msg, 'longitude') else None
                            logging.debug(f"Fix quality: {fix_quality}, Satellites: {num_sats}, Lat: {latitude}, Lon: {longitude}")
                            if fix_quality == 0:
                                gps_data = {
                                    "fix": "No",
                                    "quality": "0 satellites",
                                    "satellites": 0,
                                    "latitude": None,
                                    "longitude": None
                                }
                                current_time_cache["time"] = current_time_cache["time"]
                                gps_timeout_start = datetime.now() if gps_timeout_start is None else gps_timeout_start
                            elif fix_quality in (1, 2):
                                gps_data = {
                                    "fix": "Yes",
                                    "quality": f"{num_sats} satellites" if fix_quality == 1 else f"Differential ({num_sats} satellites)",
                                    "satellites": num_sats,
                                    "latitude": latitude,
                                    "longitude": longitude
                                }
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
                        local_tz = pytz.timezone("Australia/Melbourne")
                        local_dt = utc_dt.astimezone(local_tz)
                        formatted_time = local_dt.strftime("%A %B %d %Y %I:%M %p %Z").lstrip("0").replace(" 0", " ")
                        current_time_cache["time"] = formatted_time
                        current_time_cache["last_updated"] = datetime.now(pytz.UTC)
                        if not current_time_cache["fix_obtained"]:
                            current_time_cache["fix_obtained"] = True
                            set_system_clock(local_dt)
                            logging.info(f"First GPS fix: Set current time to {formatted_time}")
                        logging.debug(f"Updated current time: {formatted_time}")
            time.sleep(1)
    except pynmea2.ParseError as pe:
        logging.debug(f"ParseError: {pe} for line: {line}")
    except Exception as e:
        logging.error(f"Error updating GPS data: {e}")

def check_gps_fix():
    while True:
        with gps_lock:
            if not gps_data["fix"] == "Yes" and gps_timeout_start:
                elapsed_minutes = (datetime.now() - gps_timeout_start).total_seconds() / 60
                if elapsed_minutes >= GPS_TIMEOUT_MINUTES:
                    local_tz = pytz.timezone("Australia/Melbourne")
                    today = date.today()
                    s = sun(MELBOURNE_LOCATION.observer, date=today, tzinfo=local_tz)
                    sunrise = s["sunrise"].strftime("%I:%M %p").lstrip("0")
                    sunset = s["sunset"].strftime("%I:%M %p").lstrip("0")
                    sun_times_cache.update({
                        "sunrise": sunrise + "*",
                        "sunset": sunset + "*",
                        "last_calculated": datetime.now(pytz.UTC),
                        "last_latitude": MELBOURNE_LOCATION.latitude,
                        "last_longitude": MELBOURNE_LOCATION.longitude
                    })
                    logging.info(f"Fallback sunrise: {sunrise}*, sunset: {sunset}* using Melbourne coordinates")
        time.sleep(GPS_CHECK_INTERVAL_MINUTES * 60)

gps_thread = threading.Thread(target=lambda: [time.sleep(1), update_gps_data()] * 1000, daemon=True)
gps_check_thread = threading.Thread(target=check_gps_fix, daemon=True)
gps_thread.start()
gps_check_thread.start()

try:
    with open('config.json', 'r') as f:
        config = json.load(f)
    logging.info("Loaded config.json successfully")
except Exception as e:
    logging.error(f"Error loading config.json: {e}")
    raise

try:
    pca = PCA9685(i2c, address=0x40)
    pca.frequency = 60
    logging.info("Initialized PCA9685 at address 0x40")
except Exception as e:
    logging.error(f"Error initializing PCA9685: {e}")
    raise

try:
    h = lgpio.gpiochip_open(0)
    logging.info("Opened GPIO chip")
except Exception as e:
    logging.error(f"Error opening GPIO chip: {e}")
    raise

relay_states = {}
for pin in config['channels']['relays']:
    try:
        saved_states = load_relay_states()
        if saved_states and str(pin) in saved_states:
            relay_state = saved_states[str(pin)]
            gpio_state = 0 if relay_state else 1
            logging.info(f"Restoring relay pin {pin} to saved state: {'ON' if relay_state == 1 else 'OFF'}")
        else:
            relay_state = 0
            gpio_state = 1
            logging.warning(f"No saved state for relay pin {pin}, defaulting to OFF")
        lgpio.gpio_claim_output(h, int(pin))
        lgpio.gpio_write(h, int(pin), gpio_state)
        relay_states[pin] = relay_state
        logging.info(f"Initialized relay pin {pin} with state: {'ON' if relay_state == 1 else 'OFF'}")
    except Exception as e:
        logging.error(f"Error initializing relay pin {pin}: {e}")
        lgpio.gpio_claim_output(h, int(pin))
        lgpio.gpio_write(h, int(pin), 1)
        relay_states[pin] = 0
        logging.warning(f"Defaulted relay pin {pin} to OFF due to error")
save_relay_states(relay_states)

for pin in config['reed_switches']:
    try:
        lgpio.gpio_claim_input(h, int(pin), lgpio.SET_PULL_UP)
        logging.info(f"Initialized reed switch pin {pin}")
    except Exception as e:
        logging.error(f"Error initializing reed switch pin {pin}: {e}")

def cleanup_gpio():
    global h
    if h is None:
        return
    try:
        lgpio.gpiochip_close(h)
        logging.info("Closed GPIO chip")
    except Exception as e:
        logging.error(f"Error closing GPIO chip: {e}")
    h = None

atexit.register(cleanup_gpio)

@app.route('/')
def index():
    try:
        logging.info("Rendering index.html")
        return render_template('index.html',
                              scenes=config['scenes'].keys(),
                              pca9685_channels=config['channels']['pca9685'],
                              relay_channels=config['channels']['relays'],
                              reed_switches=config['reed_switches'])
    except Exception as e:
        logging.error(f"Error rendering index: {str(e)}")
        return f"Error rendering page: {str(e)}", 500

@app.route('/set_brightness', methods=['POST'])
def set_brightness():
    try:
        data = request.json
        logging.debug(f"Received set_brightness request: {data}")
        channel_type = data['type']
        channel = data['channel']
        brightness = int(data['brightness'])
        
        if channel_type == 'pca9685':
            target_pwm = int(brightness * 4095 / 100)
            channel_idx = int(channel) - 1
            current_pwm = pca.channels[channel_idx].duty_cycle
            steps = 10
            step_size = (target_pwm - current_pwm) / steps
            step_time = 0.05
            logging.debug(f"Ramping channel {channel}: {current_pwm} to {target_pwm}, step_size={step_size}")
            start_time = time.perf_counter()
            for i in range(steps):
                new_pwm = int(current_pwm + step_size * (i + 1))
                pca.channels[channel_idx].duty_cycle = max(0, min(4095, new_pwm))
                logging.debug(f"Step {i+1}/{steps}: PWM={new_pwm}, duty_cycle={pca.channels[channel_idx].duty_cycle}")
                time.sleep(step_time)
            elapsed = time.perf_counter() - start_time
            final_pwm = pca.channels[channel_idx].duty_cycle
            logging.info(f"Ramped channel {channel} to {brightness}% (PWM: {final_pwm}, took {elapsed:.3f}s)")
            return jsonify({"message": f"Brightness set to {brightness}% on channel {channel}"})
        
        logging.warning(f"Invalid channel type: {channel_type}")
        return jsonify({"error": "Invalid channel type"}), 400
    except Exception as e:
        logging.error(f"Error in set_brightness: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/toggle', methods=['POST'])
def toggle():
    try:
        data = request.json
        logging.debug(f"Received toggle request: {data}")
        channel_type = data['type']
        channel = data['channel']
        state = data['state']
        
        if channel_type == 'pca9685':
            target_pwm = 4095 if state == 'on' else 0
            channel_idx = int(channel) - 1
            current_pwm = pca.channels[channel_idx].duty_cycle
            steps = 10
            step_size = (target_pwm - current_pwm) / steps
            step_time = 0.05
            logging.debug(f"Ramping channel {channel}: {current_pwm} to {target_pwm}, step_size={step_size}")
            start_time = time.perf_counter()
            for i in range(steps):
                new_pwm = int(current_pwm + step_size * (i + 1))
                pca.channels[channel_idx].duty_cycle = max(0, min(4095, new_pwm))
                logging.debug(f"Step {i+1}/{steps}: PWM={new_pwm}, duty_cycle={pca.channels[channel_idx].duty_cycle}")
                time.sleep(step_time)
            elapsed = time.perf_counter() - start_time
            final_pwm = pca.channels[channel_idx].duty_cycle
            logging.info(f"Ramped channel {channel} to {state} (PWM: {final_pwm}, took {elapsed:.3f}s)")
            return jsonify({"message": f"Channel {channel} set to {state.upper()}"})
        elif channel_type == 'relays':
            lgpio.gpio_write(h, int(channel), 0 if state == 'on' else 1)
            current_states = load_relay_states() or {}
            current_states[channel] = 1 if state == 'on' else 0
            save_relay_states(current_states)
            logging.info(f"Toggled relay {channel} to {state}")
            return jsonify({"message": f"Relay {channel} set to {state.upper()}"})
        
        logging.warning(f"Invalid channel type: {channel_type}")
        return jsonify({"error": "Invalid channel type"}), 400
    except Exception as e:
        logging.error(f"Error in toggle: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/test_ramp', methods=['POST'])
def test_ramp():
    try:
        data = request.json
        channel = data['channel']
        brightness = int(data['brightness'])
        channel_idx = int(channel) - 1
        target_pwm = int(brightness * 4095 / 100)
        current_pwm = pca.channels[channel_idx].duty_cycle
        steps = 10
        step_time = 0.05
        step_size = (target_pwm - current_pwm) / steps
        logging.debug(f"Test ramp for channel {channel}: {current_pwm} to {target_pwm}")
        start_time = time.perf_counter()
        for i in range(steps):
            new_pwm = int(current_pwm + step_size * (i + 1))
            pca.channels[channel_idx].duty_cycle = max(0, min(4095, new_pwm))
            logging.debug(f"Test step {i+1}/{steps}: PWM={new_pwm}, duty_cycle={pca.channels[channel_idx].duty_cycle}")
            time.sleep(step_time)
        elapsed = time.perf_counter() - start_time
        final_pwm = pca.channels[channel_idx].duty_cycle
        logging.info(f"Test ramped channel {channel} to {brightness}% (PWM: {final_pwm}, took {elapsed:.3f}s)")
        return jsonify({"message": f"Test ramped channel {channel} to {brightness}%"})
    except Exception as e:
        logging.error(f"Error in test_ramp: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/activate_scene', methods=['POST'])
def activate_scene():
    try:
        data = request.json
        logging.debug(f"Received activate_scene request: {data}")
        scene_name = data['scene']
        scene = config['scenes'][scene_name]
        
        for channel, brightness in scene.get('pca9685', {}).items():
            pwm_value = int(brightness * 4095 / 100)
            pca.channels[int(channel) - 1].duty_cycle = max(0, min(4095, pwm_value))
            logging.info(f"Scene {scene_name}: Set PCA9685 channel {channel} to {brightness}%")
        
        current_states = load_relay_states() or {}
        for pin, state in scene.get('relays', {}).items():
            lgpio.gpio_write(h, int(pin), 1 if state else 0)
            current_states[pin] = 1 if state else 0
            logging.info(f"Scene {scene_name}: Set relay {pin} to {'on' if state else 'off'}")
        save_relay_states(current_states)
        
        logging.info(f"Activated scene {scene_name}")
        return jsonify({"message": f"Scene {scene_name} activated"})
    except Exception as e:
        logging.error(f"Error in activate_scene: {e}")
        return jsonify({"error": str(e)}), 500

@app.route('/get_pca_states', methods=['GET'])
def get_pca_states():
    try:
        states = {}
        for channel in config['channels']['pca9685']:
            duty_cycle = pca.channels[int(channel) - 1].duty_cycle
            brightness = int(round(duty_cycle / 4095 * 100))
            states[channel] = brightness
            logging.debug(f"Channel {channel}: duty_cycle={duty_cycle}, brightness={brightness}%")
        logging.debug(f"Returning PCA states: {states}")
        return jsonify(states)
    except Exception as e:
        logging.error(f"Error in get_pca_states: {e}")
        return jsonify({}), 500

@app.route('/get_relay_states', methods=['GET'])
def get_relay_states():
    try:
        states = {}
        for pin in config['channels']['relays']:
            state = lgpio.gpio_read(h, int(pin))
            states[pin] = 0 if state else 1
        logging.debug(f"Returning relay states: {states}")
        return jsonify(states)
    except Exception as e:
        logging.error(f"Error in get_relay_states: {e}")
        return jsonify({}), 500

@app.route('/get_data', methods=['GET'])
def get_data():
    global sun_times_cache, current_time_cache
    try:
        # Read DS18B20 temperature
        temperature = get_ds18b20_temperature()
        temperature_str = str(temperature) if temperature is not None else "N/A"

        # Read battery voltage
        battery_voltage = get_battery_voltage()
        battery_voltage_str = f"{battery_voltage}V" if battery_voltage is not None else "N/A"

        # Read tank level
        tank_level = get_tank_level()
        tank_level_str = f"{tank_level}%" if tank_level is not None else "N/A"
        # Optional: Add fault counter for persistent sensor issues (not implemented)

        # Check if we need to recalculate sunrise/sunset
        current_time = datetime.now(pytz.UTC)
        recalculate = False
        if sun_times_cache["last_calculated"] is None:
            recalculate = True
        else:
            time_diff = (current_time - sun_times_cache["last_calculated"]).total_seconds()
            if time_diff > 3600:
                recalculate = True
            if gps_data["latitude"] is not None and gps_data["longitude"] is not None:
                if sun_times_cache["last_latitude"] is None or sun_times_cache["last_longitude"] is None:
                    recalculate = True
                else:
                    lat_diff = abs(gps_data["latitude"] - sun_times_cache["last_latitude"])
                    lon_diff = abs(gps_data["longitude"] - sun_times_cache["last_longitude"])
                    if lat_diff > 0.1 or lon_diff > 0.1:
                        recalculate = True

        if recalculate and gps_data["fix"] == "Yes" and gps_data["latitude"] is not None and gps_data["longitude"] is not None:
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
            sunrise = s["sunrise"].astimezone(local_tz).strftime("%I:%M %p").lstrip("0")
            sunset = s["sunset"].astimezone(local_tz).strftime("%I:%M %p").lstrip("0")
            sun_times_cache.update({
                "sunrise": sunrise,
                "sunset": sunset,
                "last_calculated": current_time,
                "last_latitude": gps_data["latitude"],
                "last_longitude": gps_data["longitude"]
            })
            logging.debug(f"Calculated sunrise: {sunrise}, sunset: {sunset} using GPS lat: {gps_data['latitude']}, lon: {gps_data['longitude']}")
        elif not current_time_cache["using_gps"] and gps_timeout_start:
            elapsed_minutes = (datetime.now() - gps_timeout_start).total_seconds() / 60
            if elapsed_minutes >= GPS_TIMEOUT_MINUTES and sun_times_cache["sunrise"] == "---":
                local_tz = pytz.timezone("Australia/Melbourne")
                today = date.today()
                s = sun(MELBOURNE_LOCATION.observer, date=today, tzinfo=local_tz)
                sunrise = s["sunrise"].astimezone(local_tz).strftime("%I:%M %p").lstrip("0")
                sunset = s["sunset"].astimezone(local_tz).strftime("%I:%M %p").lstrip("0")
                sun_times_cache.update({
                    "sunrise": sunrise + "*",
                    "sunset": sunset + "*",
                    "last_calculated": current_time,
                    "last_latitude": MELBOURNE_LOCATION.latitude,
                    "last_longitude": MELBOURNE_LOCATION.longitude
                })
                logging.info(f"Fallback sunrise: {sunrise}*, sunset: {sunset}* using Melbourne coordinates")

        data = {
            "temperature": temperature_str,
            "battery_level": battery_voltage_str,
            "tank_level": tank_level_str,
            "sunrise": sun_times_cache["sunrise"],
            "sunset": sun_times_cache["sunset"],
            "current_datetime": current_time_cache["time"],
            "kitchen_panel": lgpio.gpio_read(h, 23) and "Open" or "Closed",
            "storage_panel": lgpio.gpio_read(h, 24) and "Open" or "Closed",
            "rear_drawer": lgpio.gpio_read(h, 25) and "Open" or "Closed",
            "gps_fix": gps_data["fix"],
            "gps_quality": gps_data["quality"]
        }
        logging.debug(f"Returning data: {data}")
        return jsonify(data)
    except Exception as e:
        logging.error(f"Error in get_data: {e}")
        return jsonify({}), 500

@app.route('/shutdown', methods=['POST'])
def shutdown():
    try:
        data = request.json or {}
        token = data.get('token')
        if token != SHUTDOWN_TOKEN:
            logging.warning("Unauthorized shutdown attempt")
            return jsonify({"error": "Unauthorized"}), 403
        
        logging.info("Shutdown command received, initiating system shutdown")
        cleanup_gpio()
        threading.Thread(target=lambda: subprocess.run(['sudo', 'shutdown', 'now'], check=True), daemon=True).start()
        return jsonify({"message": "System is shutting down"}), 200
    except Exception as e:
        logging.error(f"Error in shutdown: {e}")
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    try:
        logging.info("Starting Flask app on port 5000")
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
    except KeyboardInterrupt:
        logging.info("Received KeyboardInterrupt, cleaning up")
        cleanup_gpio()
    except Exception as e:
        logging.error(f"Error starting Flask app: {e}")
        cleanup_gpio()
        raise
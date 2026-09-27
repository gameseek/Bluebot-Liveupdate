# I2C
SDA_PIN = 20
SCL_PIN = 21

# TB6612 Motor Driver
PWMA_PIN = 4
PWMB_PIN = 3

AIN1_PIN = 10
AIN2_PIN = 7

BIN1_PIN = 6
BIN2_PIN = 5

# WS2812B
NEOPIXEL_PIN = 1
NEOPIXEL_COUNT = 2

# Sharp GP2Y0E03 Analog
SHARP_PIN = 2

# Battery ADC
BATTERY_PIN = 0

FIRMWARE_VERSION = "1.0.15"
SETTINGS_VERSION = "1.0.2"

from machine import Pin, PWM, ADC, SoftI2C
import machine
import bluetooth, neopixel, time, math, json, os, struct
import ssd1306
from ota import OTAUpdater

try:
    from VL53L0X_V1 import VL53L0X
    VL53_AVAILABLE = True
except:
    VL53_AVAILABLE = False

DEVICE_NAME = "BluBot"
SETTINGS_FILE = "settings.json"

MODE_REMOTE = 1
MODE_TRACKING = 2
MODE_AUTO = 3

LED_OFF = 0
LED_SOLID = 1
LED_PULSE = 2
LED_RAINBOW = 3
LED_STROBE = 4
LED_CHASE = 5
LED_BREATHING = 6

SHARP_INTERVAL_MS = 10
TOF_INTERVAL_MS = 10
MPU_CONTROL_INTERVAL_MS = 10
FAST_TELEMETRY_INTERVAL_MS = 50
MPU_TELEMETRY_INTERVAL_MS = 100
BATTERY_INTERVAL_MS = 1000
OLED_INTERVAL_MS = 100
SETTINGS_SAVE_DELAY_MS = 1500

EMERGENCY_BRAKE_MS = 90

FRONT_HARD_STOP_CM = 15.0
FRONT_MAX_BRAKE_CM = 40.0
FRONT_CLEAR_CM = 22.0
FRONT_CLEAR_COUNT = 2
FRONT_CONFIRM_COUNT = 1

CLIFF_THRESHOLD_MM = 120
CLIFF_CONFIRM_COUNT = 1
TOF_INVALID_CLIFF_COUNT = 2
CLIFF_BRAKE_MS = 80
CLIFF_BACKOFF_MS = 520
CLIFF_BACKOFF_POWER = 0.38
CLIFF_HOLD_MS = 150

SAFETY_NORMAL = 0
SAFETY_CLIFF_BRAKE = 1
SAFETY_CLIFF_BACKOFF = 2
SAFETY_CLIFF_HOLD = 3

DRIVE_RAMP_PER_SEC = 3.0
GYRO_TURN_MAX_DPS = 160.0
GYRO_STEER_KP = 0.22
MAX_GYRO_CORRECTION = 0.22
GYRO_Z_SIGN = 1.0
STRAIGHT_DEADZONE = 0.08
TILT_STOP_DEG = 55.0
TILT_CONFIRM_COUNT = 5

AUTO_IDLE = 0
AUTO_FORWARD = 1
AUTO_WALL_BRAKE = 2
AUTO_WALL_REVERSE = 3
AUTO_SCAN_PREP = 4
AUTO_SCAN_LEFT = 5
AUTO_SCAN_LEFT_SETTLE = 6
AUTO_RETURN_CENTER_1 = 7
AUTO_SCAN_RIGHT = 8
AUTO_SCAN_RIGHT_SETTLE = 9
AUTO_RETURN_CENTER_2 = 10
AUTO_CHOOSE_PATH = 11
AUTO_TURN_CHOSEN = 12
AUTO_ESCAPE_REVERSE = 13
AUTO_ESCAPE_TURN = 14

AUTO_TURN_POWER = 0.38
AUTO_WALL_REVERSE_POWER = 0.34
AUTO_WALL_REVERSE_MS = 350
AUTO_SCAN_ANGLE_DEG = 50.0
AUTO_AVOID_TURN_DEG = 70.0
AUTO_ESCAPE_TURN_DEG = 150.0
AUTO_ESCAPE_REVERSE_POWER = 0.38
AUTO_ESCAPE_REVERSE_MS = 500
AUTO_SCAN_SETTLE_MS = 120
AUTO_MIN_CLEAR_CM = 20.0
AUTO_HEADING_KP = 0.018
AUTO_MAX_HEADING_CORRECTION = 0.18
AUTO_TURN_TIMEOUT_MS = 1400

SAFETY_FLASH_PERIOD_MS = 300

# Battery: measured divider ratio 8.41V / 1.94V ~= 4.335
BATTERY_ADC_FACTOR = 4.3333
BATTERY_EMPTY_V = 6.0
BATTERY_FULL_V = 8.4

DEFAULT_SETTINGS = {
    "settings_version": SETTINGS_VERSION, "mode": MODE_REMOTE, "speed": 75,
    "p": 0.0, "i": 0.0, "d": 0.0,
    "led_r": 255, "led_g": 255, "led_b": 255,
    "led_brightness": 100, "led_animation": LED_STROBE, "led_speed": 1000
}

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def angle_diff(a, b):
    d = a - b
    while d > 180: d -= 360
    while d < -180: d += 360
    return d

def write_settings(data):
    try:
        with open(SETTINGS_FILE, "w") as f: json.dump(data, f)
        return True
    except Exception as e:
        print("Settings write error:", e)
        return False

def create_default_settings():
    d = DEFAULT_SETTINGS.copy()
    write_settings(d)
    return d

def migrate_settings(d):
    changed = False
    for k in DEFAULT_SETTINGS:
        if k != "settings_version" and k not in d:
            d[k] = DEFAULT_SETTINGS[k]
            changed = True
    d["settings_version"] = SETTINGS_VERSION
    if changed: write_settings(d)
    return d

def load_settings():
    try:
        if SETTINGS_FILE not in os.listdir(): return create_default_settings()
        with open(SETTINGS_FILE, "r") as f: return migrate_settings(json.load(f))
    except Exception as e:
        print("Settings load error:", e)
        return create_default_settings()

settings = load_settings()
current_mode = int(settings["mode"])
max_speed = int(settings["speed"])
pid_p, pid_i, pid_d = float(settings["p"]), float(settings["i"]), float(settings["d"])
led_r, led_g, led_b = int(settings["led_r"]), int(settings["led_g"]), int(settings["led_b"])
led_brightness = int(settings["led_brightness"])
led_animation = int(settings["led_animation"])
led_speed = int(settings["led_speed"])
settings_dirty = False
settings_dirty_since = 0

def save_current_settings():
    global settings_dirty
    write_settings({
        "settings_version": SETTINGS_VERSION, "mode": int(current_mode), "speed": int(max_speed),
        "p": float(pid_p), "i": float(pid_i), "d": float(pid_d),
        "led_r": int(led_r), "led_g": int(led_g), "led_b": int(led_b),
        "led_brightness": int(led_brightness), "led_animation": int(led_animation), "led_speed": int(led_speed)
    })
    settings_dirty = False

def mark_settings_dirty():
    global settings_dirty, settings_dirty_since
    settings_dirty = True
    settings_dirty_since = time.ticks_ms()

requested_left = requested_right = 0.0
actual_left = actual_right = 0.0
drive_left_output = drive_right_output = 0.0
drive_last_time = time.ticks_ms()
app_stop_latched = False

motor_brake_active = False
motor_brake_start = 0

front_obstacle = False
front_danger_count = 0
front_clear_count = 0

safety_state = SAFETY_NORMAL
safety_state_start = time.ticks_ms()
cliff_danger_count = 0
tof_invalid_count = 0
cliff_backoff_allowed = False
tilt_danger_count = 0

auto_running = False
auto_state = AUTO_IDLE
auto_state_start = time.ticks_ms()
auto_left_target = auto_right_target = 0.0
auto_heading = 0.0
auto_left_distance = auto_right_distance = 0.0
auto_chosen_direction = 1
auto_turn_direction = 0
auto_turn_degrees = 0.0
auto_turn_start_yaw = 0.0
auto_turn_start_time = 0
auto_escape_direction = -1

ble_connected = False
connections = set()
rx_buffer = ""

sharp_raw = 0
sharp_voltage = 0.0
sharp_distance_cm = sharp_instant_cm = -1.0
sharp_filtered_cm = 0.0
sharp_filter_ready = False

tof_available = False
tof_distance_mm = -1
tof_valid = False
floor_cliff = 0

mpu_available = False
mpu_pitch = mpu_roll = mpu_yaw = 0.0
gyro_z_dps = gyro_z_bias = 0.0
mpu_last_time = time.ticks_ms()

battery_voltage = 0.0
battery_percent = 0
battery_voltage_filtered = None

safety_flash_start = time.ticks_ms()

ain1, ain2 = Pin(AIN1_PIN, Pin.OUT), Pin(AIN2_PIN, Pin.OUT)
bin1, bin2 = Pin(BIN1_PIN, Pin.OUT), Pin(BIN2_PIN, Pin.OUT)
pwma, pwmb = PWM(Pin(PWMA_PIN)), PWM(Pin(PWMB_PIN))
pwma.freq(20000); pwmb.freq(20000)
pwma.duty_u16(0); pwmb.duty_u16(0)

np = neopixel.NeoPixel(Pin(NEOPIXEL_PIN, Pin.OUT), NEOPIXEL_COUNT)

# IMPORTANT: read_u16()*3.3 is wrong on ESP32-C3 with attenuation.
# Use calibrated read_uv() instead.
sharp_adc = ADC(Pin(SHARP_PIN), atten=ADC.ATTN_11DB)
battery_adc = ADC(Pin(BATTERY_PIN), atten=ADC.ATTN_11DB)

i2c = SoftI2C(sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=400000)
try: i2c_devices = i2c.scan()
except: i2c_devices = []
print("I2C:", [hex(x) for x in i2c_devices])

oled = None
try:
    oled = ssd1306.SSD1306_I2C(128, 64, i2c)
except Exception as e:
    print("OLED not found:", e)

tof = None
if VL53_AVAILABLE:
    try:
        tof = VL53L0X(i2c)
        tof_available = True
        print("VL53L0X connected")
    except Exception as e:
        print("TOF not found:", e)

MPU_ADDR = 0x68
if MPU_ADDR in i2c_devices:
    try:
        i2c.writeto_mem(MPU_ADDR, 0x6B, b"\x00")
        time.sleep_ms(100)
        mpu_available = True
        print("MPU6050 connected")
    except Exception as e:
        print("MPU error:", e)

def calibrate_gyro():
    global gyro_z_bias
    if not mpu_available: return
    print("Keep robot still - gyro calibration")
    total = 0.0; count = 0
    for _ in range(100):
        try:
            _, _, gz = struct.unpack(">hhh", i2c.readfrom_mem(MPU_ADDR, 0x43, 6))
            total += gz / 131.0
            count += 1
        except: pass
        time.sleep_ms(5)
    if count: gyro_z_bias = total / count
    print("Gyro Z bias:", gyro_z_bias)

def drive_motor(in1, in2, pwm, value):
    value = clamp(value, -1.0, 1.0)
    if abs(value) < 0.03: value = 0.0
    if value == 0.0:
        in1.value(0); in2.value(0); pwm.duty_u16(0); return
    if value > 0:
        in1.value(1); in2.value(0)
    else:
        in1.value(0); in2.value(1)
    pwm.duty_u16(int(abs(value) * 65535))

def brake_motor(in1, in2, pwm):
    in1.value(1); in2.value(1); pwm.duty_u16(65535)

def release_motor_output():
    ain1.value(0); ain2.value(0); bin1.value(0); bin2.value(0)
    pwma.duty_u16(0); pwmb.duty_u16(0)

def start_emergency_brake():
    global motor_brake_active, motor_brake_start
    global actual_left, actual_right, drive_left_output, drive_right_output
    actual_left = actual_right = drive_left_output = drive_right_output = 0.0
    if not motor_brake_active:
        motor_brake_active = True
        motor_brake_start = time.ticks_ms()
        brake_motor(ain1, ain2, pwma)
        brake_motor(bin1, bin2, pwmb)

def update_emergency_brake():
    global motor_brake_active
    if motor_brake_active and time.ticks_diff(time.ticks_ms(), motor_brake_start) >= EMERGENCY_BRAKE_MS:
        motor_brake_active = False
        release_motor_output()

def coast_motor_stop():
    global actual_left, actual_right, drive_left_output, drive_right_output
    actual_left = actual_right = drive_left_output = drive_right_output = 0.0
    release_motor_output()

def absolute_stop():
    global app_stop_latched, requested_left, requested_right
    global auto_running, auto_state, auto_left_target, auto_right_target
    global safety_state, cliff_danger_count, tof_invalid_count, front_danger_count
    app_stop_latched = True
    requested_left = requested_right = 0.0
    auto_running = False; auto_state = AUTO_IDLE
    auto_left_target = auto_right_target = 0.0
    safety_state = SAFETY_NORMAL
    cliff_danger_count = tof_invalid_count = front_danger_count = 0
    start_emergency_brake()

def stop_motors():
    global requested_left, requested_right
    requested_left = requested_right = 0.0
    start_emergency_brake()

def set_motors(left, right):
    global requested_left, requested_right, app_stop_latched
    if current_mode != MODE_REMOTE: return
    app_stop_latched = False
    left, right = clamp(left, -1, 1), clamp(right, -1, 1)
    if abs(left) < 0.03: left = 0.0
    if abs(right) < 0.03: right = 0.0
    requested_left, requested_right = left, right

def robot_motion_commanded():
    if current_mode == MODE_REMOTE:
        return abs(requested_left) > .03 or abs(requested_right) > .03 or abs(actual_left) > .03 or abs(actual_right) > .03
    return current_mode == MODE_AUTO and auto_running

def forward_motion_commanded():
    if current_mode == MODE_REMOTE:
        return (requested_left + requested_right) / 2 > .02 or (actual_left + actual_right) / 2 > .02
    if current_mode == MODE_AUTO and auto_running:
        return (auto_left_target + auto_right_target) / 2 > .02 or auto_state == AUTO_FORWARD
    return False

def get_front_stop_distance():
    f = clamp((actual_left + actual_right) / 2, 0, 1)
    return clamp(FRONT_HARD_STOP_CM + (FRONT_MAX_BRAKE_CM - FRONT_HARD_STOP_CM) * f,
                 FRONT_HARD_STOP_CM, FRONT_MAX_BRAKE_CM)

def safety_red_now():
    global safety_flash_start
    safety_flash_start = time.ticks_ms()
    for i in range(NEOPIXEL_COUNT): np[i] = (255, 0, 0)
    np.write()

def update_front_safety():
    global front_obstacle, front_danger_count, front_clear_count
    global requested_left, requested_right, auto_left_target, auto_right_target
    if app_stop_latched or sharp_instant_cm < 0: return

    if front_obstacle:
        front_clear_count = front_clear_count + 1 if sharp_instant_cm >= FRONT_CLEAR_CM else 0
        if front_clear_count >= FRONT_CLEAR_COUNT:
            front_obstacle = False
            front_clear_count = front_danger_count = 0
            print("WALL CLEARED:", round(sharp_instant_cm, 1), "cm")
            reset_led_animation()
        return

    if not forward_motion_commanded():
        front_danger_count = 0
        return

    stop_distance = get_front_stop_distance()
    front_danger_count = front_danger_count + 1 if sharp_instant_cm <= stop_distance else 0
    if front_danger_count < FRONT_CONFIRM_COUNT: return

    front_danger_count = front_clear_count = 0
    front_obstacle = True
    if current_mode == MODE_REMOTE:
        requested_left = requested_right = 0.0
    auto_left_target = auto_right_target = 0.0
    start_emergency_brake()
    safety_red_now()
    print("WALL HARD BRAKE:", round(sharp_instant_cm, 1), "/", round(stop_distance, 1))

def start_cliff_safety(reason="CLIFF"):
    global safety_state, safety_state_start, cliff_danger_count, tof_invalid_count
    global cliff_backoff_allowed, requested_left, requested_right
    global auto_left_target, auto_right_target
    if app_stop_latched or safety_state != SAFETY_NORMAL: return
    cliff_backoff_allowed = robot_motion_commanded()
    requested_left = requested_right = 0.0
    auto_left_target = auto_right_target = 0.0
    cliff_danger_count = tof_invalid_count = 0
    safety_state = SAFETY_CLIFF_BRAKE
    safety_state_start = time.ticks_ms()
    start_emergency_brake()
    safety_red_now()
    print("!!!", reason, "CLIFF HARD BRAKE", tof_distance_mm, "backoff", cliff_backoff_allowed)

def update_cliff_safety():
    global safety_state, safety_state_start, cliff_danger_count, tof_invalid_count
    global requested_left, requested_right, auto_state
    if app_stop_latched: return
    now = time.ticks_ms()

    if safety_state == SAFETY_NORMAL:
        if tof_valid:
            tof_invalid_count = 0
            if floor_cliff:
                cliff_danger_count += 1
                if cliff_danger_count >= CLIFF_CONFIRM_COUNT: start_cliff_safety("VALID")
            else:
                cliff_danger_count = 0
        elif robot_motion_commanded():
            tof_invalid_count += 1
            if tof_invalid_count >= TOF_INVALID_CLIFF_COUNT:
                start_cliff_safety("NO FLOOR RETURN")
        else:
            tof_invalid_count = 0
        return

    if safety_state == SAFETY_CLIFF_BRAKE:
        if time.ticks_diff(now, safety_state_start) >= CLIFF_BRAKE_MS:
            safety_state = SAFETY_CLIFF_BACKOFF if cliff_backoff_allowed else SAFETY_CLIFF_HOLD
            safety_state_start = now
            print("CLIFF -> REVERSE NOW" if cliff_backoff_allowed else "CLIFF HOLD")
        return

    if safety_state == SAFETY_CLIFF_BACKOFF:
        if time.ticks_diff(now, safety_state_start) >= CLIFF_BACKOFF_MS:
            start_emergency_brake()
            safety_state = SAFETY_CLIFF_HOLD
            safety_state_start = now
            print("CLIFF REVERSE COMPLETE")
        return

    if safety_state == SAFETY_CLIFF_HOLD and time.ticks_diff(now, safety_state_start) >= CLIFF_HOLD_MS:
        if tof_valid and floor_cliff == 0:
            safety_state = SAFETY_NORMAL
            cliff_danger_count = tof_invalid_count = 0
            coast_motor_stop()
            print("CLIFF RECOVERY: FLOOR SAFE")
            reset_led_animation()
            if current_mode == MODE_AUTO and auto_running:
                auto_state = AUTO_SCAN_PREP
            else:
                requested_left = requested_right = 0.0

def set_auto_target(left, right):
    global auto_left_target, auto_right_target
    auto_left_target = clamp(left, -1, 1)
    auto_right_target = clamp(right, -1, 1)

def auto_brake():
    set_auto_target(0, 0)
    start_emergency_brake()

def begin_auto_turn(direction, degrees):
    global auto_turn_direction, auto_turn_degrees, auto_turn_start_yaw, auto_turn_start_time
    auto_turn_direction = -1 if direction < 0 else 1
    auto_turn_degrees = abs(degrees)
    auto_turn_start_yaw = mpu_yaw
    auto_turn_start_time = time.ticks_ms()

def update_auto_turn():
    if app_stop_latched:
        set_auto_target(0, 0)
        return False
    now = time.ticks_ms()
    if mpu_available and abs(angle_diff(mpu_yaw, auto_turn_start_yaw)) >= auto_turn_degrees:
        set_auto_target(0, 0)
        return True
    if time.ticks_diff(now, auto_turn_start_time) >= AUTO_TURN_TIMEOUT_MS:
        print("AUTO TURN TIMEOUT")
        set_auto_target(0, 0)
        return True
    if auto_turn_direction < 0:
        set_auto_target(-AUTO_TURN_POWER, AUTO_TURN_POWER)
    else:
        set_auto_target(AUTO_TURN_POWER, -AUTO_TURN_POWER)
    return False

def start_autonomous():
    global auto_running, auto_state, auto_heading, app_stop_latched
    if current_mode != MODE_AUTO:
        print("AUTOSTART ignored - use MODE,3 first")
        return
    app_stop_latched = False
    auto_running = True
    auto_state = AUTO_FORWARD
    auto_heading = mpu_yaw
    set_auto_target(0, 0)
    print("AUTONOMOUS START")

def stop_autonomous():
    absolute_stop()
    print("AUTONOMOUS STOP")

def update_autonomous():
    global auto_state, auto_state_start, auto_heading
    global auto_left_distance, auto_right_distance, auto_chosen_direction, auto_escape_direction

    if app_stop_latched:
        set_auto_target(0, 0)
        return
    if current_mode != MODE_AUTO or not auto_running:
        set_auto_target(0, 0)
        return
    if safety_state != SAFETY_NORMAL:
        set_auto_target(0, 0)
        return

    now = time.ticks_ms()

    if auto_state == AUTO_FORWARD:
        if front_obstacle:
            set_auto_target(0, 0)
            auto_state = AUTO_WALL_BRAKE
            auto_state_start = now
            print("AUTO WALL -> BRAKE")
            return
        speed = clamp(max_speed / 100.0, 0, 1)
        correction = 0.0
        if mpu_available:
            correction = clamp(angle_diff(auto_heading, mpu_yaw) * AUTO_HEADING_KP,
                               -AUTO_MAX_HEADING_CORRECTION, AUTO_MAX_HEADING_CORRECTION)
        set_auto_target(clamp(speed + correction, 0, 1), clamp(speed - correction, 0, 1))
        return

    if auto_state == AUTO_WALL_BRAKE:
        set_auto_target(0, 0)
        if not motor_brake_active:
            auto_state = AUTO_WALL_REVERSE
            auto_state_start = now
            print("AUTO WALL -> REVERSE")
        return

    if auto_state == AUTO_WALL_REVERSE:
        set_auto_target(-AUTO_WALL_REVERSE_POWER, -AUTO_WALL_REVERSE_POWER)
        if time.ticks_diff(now, auto_state_start) >= AUTO_WALL_REVERSE_MS:
            auto_brake()
            auto_state = AUTO_SCAN_PREP
            auto_state_start = now
            print("AUTO REVERSE COMPLETE -> SCAN")
        return

    if auto_state == AUTO_SCAN_PREP:
        set_auto_target(0, 0)
        if motor_brake_active: return
        begin_auto_turn(-1, AUTO_SCAN_ANGLE_DEG)
        auto_state = AUTO_SCAN_LEFT
        print("AUTO SCAN LEFT")
        return

    if auto_state == AUTO_SCAN_LEFT:
        if update_auto_turn():
            auto_brake()
            auto_state = AUTO_SCAN_LEFT_SETTLE
            auto_state_start = now
        return

    if auto_state == AUTO_SCAN_LEFT_SETTLE:
        set_auto_target(0, 0)
        if motor_brake_active or time.ticks_diff(now, auto_state_start) < AUTO_SCAN_SETTLE_MS: return
        auto_left_distance = sharp_instant_cm if sharp_instant_cm >= 0 else 0
        print("AUTO LEFT CLEARANCE:", round(auto_left_distance, 1))
        begin_auto_turn(1, AUTO_SCAN_ANGLE_DEG)
        auto_state = AUTO_RETURN_CENTER_1
        return

    if auto_state == AUTO_RETURN_CENTER_1:
        if update_auto_turn():
            auto_brake()
            begin_auto_turn(1, AUTO_SCAN_ANGLE_DEG)
            auto_state = AUTO_SCAN_RIGHT
            print("AUTO SCAN RIGHT")
        return

    if auto_state == AUTO_SCAN_RIGHT:
        if motor_brake_active: return
        if update_auto_turn():
            auto_brake()
            auto_state = AUTO_SCAN_RIGHT_SETTLE
            auto_state_start = now
        return

    if auto_state == AUTO_SCAN_RIGHT_SETTLE:
        set_auto_target(0, 0)
        if motor_brake_active or time.ticks_diff(now, auto_state_start) < AUTO_SCAN_SETTLE_MS: return
        auto_right_distance = sharp_instant_cm if sharp_instant_cm >= 0 else 0
        print("AUTO RIGHT CLEARANCE:", round(auto_right_distance, 1))
        begin_auto_turn(-1, AUTO_SCAN_ANGLE_DEG)
        auto_state = AUTO_RETURN_CENTER_2
        return

    if auto_state == AUTO_RETURN_CENTER_2:
        if update_auto_turn():
            auto_brake()
            auto_state = AUTO_CHOOSE_PATH
            auto_state_start = now
        return

    if auto_state == AUTO_CHOOSE_PATH:
        set_auto_target(0, 0)
        if motor_brake_active: return
        print("PATH L/R:", round(auto_left_distance, 1), "/", round(auto_right_distance, 1))
        if auto_left_distance < AUTO_MIN_CLEAR_CM and auto_right_distance < AUTO_MIN_CLEAR_CM:
            auto_state = AUTO_ESCAPE_REVERSE
            auto_state_start = now
            auto_escape_direction *= -1
            print("AUTO BOTH BLOCKED -> ESCAPE")
            return
        auto_chosen_direction = -1 if auto_left_distance >= auto_right_distance else 1
        print("AUTO CHOOSE", "LEFT" if auto_chosen_direction < 0 else "RIGHT")
        begin_auto_turn(auto_chosen_direction, AUTO_AVOID_TURN_DEG)
        auto_state = AUTO_TURN_CHOSEN
        return

    if auto_state == AUTO_TURN_CHOSEN:
        if update_auto_turn():
            auto_brake()
            auto_heading = mpu_yaw
            auto_state = AUTO_FORWARD
            print("AUTO TURN COMPLETE -> FORWARD")
        return

    if auto_state == AUTO_ESCAPE_REVERSE:
        set_auto_target(-AUTO_ESCAPE_REVERSE_POWER, -AUTO_ESCAPE_REVERSE_POWER)
        if time.ticks_diff(now, auto_state_start) >= AUTO_ESCAPE_REVERSE_MS:
            auto_brake()
            begin_auto_turn(auto_escape_direction, AUTO_ESCAPE_TURN_DEG)
            auto_state = AUTO_ESCAPE_TURN
            print("AUTO ESCAPE TURN")
        return

    if auto_state == AUTO_ESCAPE_TURN:
        if motor_brake_active: return
        if update_auto_turn():
            auto_brake()
            auto_heading = mpu_yaw
            auto_state = AUTO_FORWARD
            print("AUTO ESCAPE COMPLETE -> FORWARD")

def update_drive_control():
    global actual_left, actual_right, drive_left_output, drive_right_output, drive_last_time
    global requested_left, requested_right, tilt_danger_count

    now = time.ticks_ms()
    dt = time.ticks_diff(now, drive_last_time) / 1000.0
    drive_last_time = now
    if dt <= 0: return
    if dt > .1: dt = .1

    if app_stop_latched:
        if not motor_brake_active: release_motor_output()
        return
    if motor_brake_active: return
    if safety_state == SAFETY_CLIFF_BRAKE:
        coast_motor_stop()
        return

    if safety_state == SAFETY_CLIFF_BACKOFF:
        actual_left = drive_left_output = -CLIFF_BACKOFF_POWER
        actual_right = drive_right_output = -CLIFF_BACKOFF_POWER
        physical_left = actual_right
        physical_right = -actual_left
        drive_motor(ain1, ain2, pwma, physical_left)
        drive_motor(bin1, bin2, pwmb, physical_right)
        return

    if safety_state == SAFETY_CLIFF_HOLD:
        coast_motor_stop()
        return

    if current_mode == MODE_REMOTE:
        il, ir = requested_left, requested_right
        if front_obstacle and (il + ir) / 2 > .02:
            coast_motor_stop()
            return
        throttle = (il + ir) / 2
        turn = (il - ir) / 2
        corrected_turn = turn
        if mpu_available:
            scale = max_speed / 100.0
            desired = 0.0 if abs(turn) < STRAIGHT_DEADZONE else turn * GYRO_TURN_MAX_DPS * scale
            corr = clamp(((desired - gyro_z_dps) / GYRO_TURN_MAX_DPS) * GYRO_STEER_KP,
                         -MAX_GYRO_CORRECTION, MAX_GYRO_CORRECTION)
            corrected_turn = clamp(turn + corr, -1, 1)
        target_left = clamp(throttle + corrected_turn, -1, 1) * (max_speed / 100.0)
        target_right = clamp(throttle - corrected_turn, -1, 1) * (max_speed / 100.0)

    elif current_mode == MODE_AUTO and auto_running:
        target_left, target_right = auto_left_target, auto_right_target
    else:
        coast_motor_stop()
        return

    if mpu_available:
        tilt_danger_count = tilt_danger_count + 1 if abs(mpu_pitch) > TILT_STOP_DEG or abs(mpu_roll) > TILT_STOP_DEG else 0
        if tilt_danger_count >= TILT_CONFIRM_COUNT:
            tilt_danger_count = 0
            print("TILT SAFETY STOP")
            absolute_stop()
            safety_red_now()
            return

    step = DRIVE_RAMP_PER_SEC * dt
    drive_left_output += clamp(target_left - drive_left_output, -step, step)
    drive_right_output += clamp(target_right - drive_right_output, -step, step)
    actual_left, actual_right = drive_left_output, drive_right_output

    # Keep existing corrected physical mapping.
    physical_left = actual_right
    physical_right = -actual_left
    drive_motor(ain1, ain2, pwma, physical_left)
    drive_motor(bin1, bin2, pwmb, physical_right)

def scale_led_color(r, g, b, brightness=None):
    if brightness is None: brightness = led_brightness
    f = clamp(brightness, 0, 100) / 100.0
    return int(r*f), int(g*f), int(b*f)

def leds_off():
    for i in range(NEOPIXEL_COUNT): np[i] = (0, 0, 0)
    np.write()

def set_all_leds(r, g, b, brightness=None):
    c = scale_led_color(r, g, b, brightness)
    for i in range(NEOPIXEL_COUNT): np[i] = c
    np.write()

def color_wheel(p):
    p %= 256
    if p < 85: return 255-p*3, p*3, 0
    if p < 170:
        p -= 85
        return 0, 255-p*3, p*3
    p -= 170
    return p*3, 0, 255-p*3

animation_start = time.ticks_ms()

def reset_led_animation():
    global animation_start
    animation_start = time.ticks_ms()

def safety_led_required():
    return front_obstacle or safety_state != SAFETY_NORMAL

def update_safety_led():
    phase = time.ticks_diff(time.ticks_ms(), safety_flash_start) % SAFETY_FLASH_PERIOD_MS
    if phase < SAFETY_FLASH_PERIOD_MS // 2: set_all_leds(255, 0, 0, 100)
    else: leds_off()

def update_led_animation():
    now = time.ticks_ms()
    if safety_led_required():
        update_safety_led(); return
    if led_animation == LED_OFF:
        leds_off(); return
    if led_animation == LED_SOLID:
        set_all_leds(led_r, led_g, led_b); return

    period = max(50, led_speed)
    phase = (time.ticks_diff(now, animation_start) % period) / period

    if led_animation == LED_PULSE:
        if phase < .10: level = phase/.10
        elif phase < .18: level = 1.0 - ((phase-.10)/.08)*.65
        elif phase < .26: level = .35 + ((phase-.18)/.08)*.65
        elif phase < .42: level = 1.0 - ((phase-.26)/.16)
        else: level = 0
        set_all_leds(led_r, led_g, led_b, int(led_brightness*clamp(level,0,1)))
        return

    if led_animation == LED_RAINBOW:
        pos = int(phase*255)
        for i in range(NEOPIXEL_COUNT):
            np[i] = scale_led_color(*color_wheel(pos + int(i*256/NEOPIXEL_COUNT)))
        np.write()
        return

    if led_animation == LED_STROBE:
        period = max(400, led_speed)
        phase = (time.ticks_diff(now, animation_start) % period) / period
        flash = phase < .07 or .14 <= phase < .21 or .28 <= phase < .35
        if flash: set_all_leds(255,255,255,led_brightness)
        else: leds_off()
        return

    if led_animation == LED_CHASE:
        pos = min(int(phase*NEOPIXEL_COUNT), NEOPIXEL_COUNT-1)
        for i in range(NEOPIXEL_COUNT): np[i] = (0,0,0)
        np[pos] = scale_led_color(led_r,led_g,led_b)
        np.write()
        return

    if led_animation == LED_BREATHING:
        level = (math.sin(phase*2*math.pi - math.pi/2)+1)/2
        set_all_leds(led_r,led_g,led_b,int(led_brightness*level))

def read_battery():
    global battery_voltage, battery_percent, battery_voltage_filtered
    try:
        total_uv = 0
        for _ in range(16):
            total_uv += battery_adc.read_uv()
            time.sleep_us(150)

        adc_v = (total_uv / 16) / 1000000.0
        measured_v = adc_v * BATTERY_ADC_FACTOR

        if battery_voltage_filtered is None:
            battery_voltage_filtered = measured_v
        else:
            battery_voltage_filtered = battery_voltage_filtered * 0.85 + measured_v * 0.15

        battery_voltage = battery_voltage_filtered
        pct = ((battery_voltage - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V)) * 100.0
        battery_percent = int(clamp(pct, 0, 100))

        #print("BAT ADC:", round(adc_v,3), "BAT:", round(battery_voltage,2), "V", battery_percent, "%")
    except Exception as e:
        print("Battery error:", e)

def sharp_voltage_to_distance(v):
    if v <= .05: return 50.0
    raw = 13.0 / v
    if raw >= 40.0: return 50.0
    x = raw - 3.36
    if x <= .05: return 4.0
    return clamp(.56 + 15.99*math.log(x), 4.0, 50.0)

def read_sharp():
    global sharp_raw, sharp_voltage, sharp_distance_cm, sharp_instant_cm
    global sharp_filtered_cm, sharp_filter_ready
    try:
        total = 0
        for _ in range(4): total += sharp_adc.read_u16()
        sharp_raw = int(total/4)
        try: sharp_voltage = sharp_adc.read_uv() / 1000000.0
        except: sharp_voltage = (sharp_raw / 65535.0) * 3.3
        d = sharp_voltage_to_distance(sharp_voltage)
        sharp_instant_cm = d
        if not sharp_filter_ready:
            sharp_filtered_cm = d
            sharp_filter_ready = True
        else:
            sharp_filtered_cm = sharp_filtered_cm*.75 + d*.25
        sharp_distance_cm = sharp_filtered_cm
    except Exception as e:
        print("Sharp error:", e)
        sharp_distance_cm = sharp_instant_cm = -1

def get_tof_reading():
    if tof is None: return -1
    for name in ("read", "ping", "range"):
        try:
            if hasattr(tof, name):
                v = getattr(tof, name)
                return int(v() if callable(v) else v)
        except: pass
    return -1

def read_tof():
    global tof_distance_mm, tof_valid, floor_cliff
    if not tof_available:
        tof_distance_mm = -1
        tof_valid = False
        return
    v = get_tof_reading()
    if v <= 0 or v >= 8190:
        tof_distance_mm = -1
        tof_valid = False
        return
    tof_distance_mm = v
    tof_valid = True
    floor_cliff = 0 if v <= CLIFF_THRESHOLD_MM else 1

def read_mpu():
    global mpu_pitch, mpu_roll, mpu_yaw, gyro_z_dps, mpu_last_time
    if not mpu_available: return
    try:
        now = time.ticks_ms()
        dt = time.ticks_diff(now, mpu_last_time)/1000.0
        mpu_last_time = now
        if dt <= 0 or dt > 1: dt = .01
        axr, ayr, azr, _, _, _, gzr = struct.unpack(">hhhhhhh", i2c.readfrom_mem(MPU_ADDR,0x3B,14))
        ax, ay, az = axr/16384.0, ayr/16384.0, azr/16384.0
        gyro_z_dps = (gzr/131.0 - gyro_z_bias) * GYRO_Z_SIGN
        mpu_roll = math.atan2(ay,az)*57.2957795
        mpu_pitch = math.atan2(-ax, math.sqrt(ay*ay+az*az))*57.2957795
        mpu_yaw += gyro_z_dps*dt
        while mpu_yaw > 180: mpu_yaw -= 360
        while mpu_yaw < -180: mpu_yaw += 360
    except Exception as e:
        print("MPU error:", e)

def mode_name():
    if current_mode == MODE_REMOTE: return "REMOTE"
    if current_mode == MODE_TRACKING: return "TRACK"
    if current_mode == MODE_AUTO: return "AUTO" if auto_running else "AUTO WAIT"
    return "?"

def movement_name():
    if app_stop_latched: return "STOP"
    if safety_state == SAFETY_CLIFF_BRAKE: return "CLIFF BRAKE"
    if safety_state == SAFETY_CLIFF_BACKOFF: return "CLIFF BACK"
    if safety_state == SAFETY_CLIFF_HOLD: return "CLIFF HOLD"

    if current_mode == MODE_AUTO and auto_running:
        names = {
            AUTO_FORWARD:"AUTO FWD", AUTO_WALL_BRAKE:"WALL BRAKE", AUTO_WALL_REVERSE:"WALL BACK",
            AUTO_SCAN_PREP:"SCAN PREP", AUTO_SCAN_LEFT:"SCAN LEFT", AUTO_SCAN_LEFT_SETTLE:"SCAN LEFT",
            AUTO_RETURN_CENTER_1:"CENTER", AUTO_SCAN_RIGHT:"SCAN RIGHT", AUTO_SCAN_RIGHT_SETTLE:"SCAN RIGHT",
            AUTO_RETURN_CENTER_2:"CENTER", AUTO_CHOOSE_PATH:"CHOOSE", AUTO_TURN_CHOSEN:"PATH TURN",
            AUTO_ESCAPE_REVERSE:"ESC BACK", AUTO_ESCAPE_TURN:"ESC TURN"
        }
        return names.get(auto_state, "AUTO")

    if front_obstacle: return "WALL STOP"
    l, r = requested_left, requested_right
    if abs(l)<.03 and abs(r)<.03: return "STOP"
    if l>.03 and r>.03: return "FORWARD"
    if l<-.03 and r<-.03: return "REVERSE"
    if l>.03 and r<-.03: return "SPIN RIGHT"
    if l<-.03 and r>.03: return "SPIN LEFT"
    return "MOVE"

_IRQ_CENTRAL_CONNECT = 1
_IRQ_CENTRAL_DISCONNECT = 2
_IRQ_GATTS_WRITE = 3

UART_SERVICE_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
UART_TX_UUID = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
UART_RX_UUID = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
UART_SERVICE = (UART_SERVICE_UUID, (
    (UART_TX_UUID, bluetooth.FLAG_NOTIFY),
    (UART_RX_UUID, bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE)
))

ble = bluetooth.BLE()
ble.active(True)
((tx_handle, rx_handle),) = ble.gatts_register_services((UART_SERVICE,))
ble.gatts_set_buffer(rx_handle, 1024, True)

def ble_send(message):
    if not connections: return
    data = message.encode()
    for conn in list(connections):
        for pos in range(0, len(data), 20):
            try: ble.gatts_notify(conn, tx_handle, data[pos:pos+20])
            except Exception as e: print("BLE notify error:", e)

ota = OTAUpdater(ble_send, stop_motors)

def send_sensor_telemetry():
    ble_send("S,{:.1f},{},{},{},{}\n".format(sharp_distance_cm,floor_cliff,battery_percent,current_mode,max_speed))

def send_battery():
    ble_send("BAT:{},{:.2f}\n".format(battery_percent,battery_voltage))

def send_mpu():
    if mpu_available:
        ble_send("MPU:{:.1f},{:.1f},{:.1f}\n".format(mpu_pitch,mpu_roll,mpu_yaw))

def send_led_state():
    ble_send("LED,{},{},{},{},{},{}\n".format(led_r,led_g,led_b,led_brightness,led_animation,led_speed))

def send_config():
    ble_send("CFG,{:.5f},{:.5f},{:.5f},{},{}\n".format(pid_p,pid_i,pid_d,max_speed,current_mode))

def send_mode():
    ble_send("MODE,{}\n".format(current_mode))

def update_oled():
    if oled is None: return
    try:
        oled.fill(0)
        if ota.active:
            oled.text("BLE: CONNECTED" if ble_connected else "BLE: WAITING",0,0)
            oled.text("FIRMWARE UPDATE",0,16)
            oled.text("UPLOADING...",0,32)
            oled.text("{}%".format(ota.progress()),0,48)
            oled.show()
            return

        oled.text("BLE: CONNECTED" if ble_connected else "BLE: WAITING",0,0)
        oled.text("MODE:{} S:{}%".format(mode_name(),max_speed)[:16],0,8)
        oled.text("MOVE:{}".format(movement_name())[:16],0,16)
        oled.text(("SHARP:{:.1f}cm".format(sharp_distance_cm) if sharp_distance_cm>=0 else "SHARP: ERROR")[:16],0,24)
        oled.text("BAT:{}%".format(battery_percent),0,32)
        oled.text("MPU: CONNECTED" if mpu_available else "MPU: NOT FOUND",0,40)
        if not tof_available: t="TOF: NOT FOUND"
        elif not tof_valid: t="TOF: WAITING"
        elif floor_cliff == 0: t="TOF: FLOOR"
        else: t="TOF: CLIFF"
        oled.text(t[:16],0,48)
        oled.text("FW:{}".format(FIRMWARE_VERSION),0,56)
        oled.show()
    except Exception as e:
        print("OLED error:", e)

def process_command(command):
    global current_mode,max_speed,pid_p,pid_i,pid_d
    global led_r,led_g,led_b,led_brightness,led_animation,led_speed
    global app_stop_latched,auto_running,auto_state

    command = command.strip()
    if not command: return
    print("APP:", command)

    if command == "STOP":
        absolute_stop(); print("APP STOP"); return
    if command == "AUTOSTOP":
        stop_autonomous(); return
    if command == "FW?":
        ble_send("FW,{}\n".format(FIRMWARE_VERSION)); return

    if command.startswith("OTA_BEGIN,"):
        try:
            p=command.split(",")
            if len(p)!=4:
                ble_send("OTA_ERROR,BAD_BEGIN\n"); return
            absolute_stop()
            ota.begin(p[1],p[2],p[3])
        except Exception as e:
            print("OTA BEGIN error:",e)
            ble_send("OTA_ERROR,BAD_BEGIN\n")
        return

    if command.startswith("OTA_DATA,"):
        try:
            p=command.split(",",2)
            if len(p)!=3:
                ble_send("OTA_ERROR,BAD_DATA\n"); return
            ota.receive_chunk(p[1],p[2])
        except Exception as e:
            print("OTA DATA error:",e)
        return
    if command=="OTA_END": ota.finish(); return
    if command=="OTA_CANCEL": ota.cancel(); return
    if command=="OTA_STATUS": ota.send_status(); return
    if ota.active: return

    if command=="AUTOSTART": start_autonomous(); return

    if command.startswith("MOVE,"):
        try:
            p=command.split(",")
            if len(p)==3: set_motors(float(p[1]),float(p[2]))
        except Exception as e: print("MOVE error:",e)
        return

    if command.startswith("MODE,"):
        try:
            m=int(command.split(",")[1])
            if m not in (MODE_REMOTE,MODE_TRACKING,MODE_AUTO): return
            absolute_stop()
            current_mode=m
            app_stop_latched=False
            auto_running=False
            auto_state=AUTO_IDLE
            save_current_settings()
            send_mode()
            send_config()
            print("MODE:",current_mode)
        except Exception as e: print("MODE error:",e)
        return

    if command.startswith("SPEED,"):
        try:
            max_speed=int(clamp(int(command.split(",")[1]),0,100))
            mark_settings_dirty()
            send_config()
        except Exception as e: print("SPEED error:",e)
        return

    if command.startswith("PID,"):
        try:
            p=command.split(",")
            if len(p)==4:
                pid_p,pid_i,pid_d=float(p[1]),float(p[2]),float(p[3])
                save_current_settings()
                send_config()
        except Exception as e: print("PID error:",e)
        return

    if command.startswith("LED,"):
        try:
            p=command.split(",")
            if len(p)!=7: return
            led_r=int(clamp(int(p[1]),0,255))
            led_g=int(clamp(int(p[2]),0,255))
            led_b=int(clamp(int(p[3]),0,255))
            led_brightness=int(clamp(int(p[4]),0,100))
            led_animation=int(clamp(int(p[5]),0,6))
            led_speed=int(clamp(int(p[6]),50,5000))
            reset_led_animation()
            save_current_settings()
            send_led_state()
        except Exception as e: print("LED error:",e)
        return

    if command.startswith("LEDCOLOR,"):
        try:
            p=command.split(",")
            if len(p)!=4: return
            led_r=int(clamp(int(p[1]),0,255))
            led_g=int(clamp(int(p[2]),0,255))
            led_b=int(clamp(int(p[3]),0,255))
            reset_led_animation()
            save_current_settings()
            send_led_state()
        except Exception as e: print("LEDCOLOR error:",e)
        return

    if command.startswith("LEDBRIGHT,"):
        try:
            led_brightness=int(clamp(int(command.split(",")[1]),0,100))
            save_current_settings()
            send_led_state()
        except Exception as e: print("LEDBRIGHT error:",e)
        return

    if command.startswith("LEDANIM,"):
        try:
            led_animation=int(clamp(int(command.split(",")[1]),0,6))
            reset_led_animation()
            save_current_settings()
            send_led_state()
        except Exception as e: print("LEDANIM error:",e)
        return

    if command.startswith("LEDSPEED,"):
        try:
            led_speed=int(clamp(int(command.split(",")[1]),50,5000))
            reset_led_animation()
            save_current_settings()
            send_led_state()
        except Exception as e: print("LEDSPEED error:",e)
        return

    if command=="LED?": send_led_state(); return
    if command in ("CFG?","CONFIG?"): send_config(); return
    if command=="SAVECFG": save_current_settings(); send_config(); return
    if command in ("OBJECTSTART","OBJECTSTOP"): absolute_stop(); return

def advertising_payload(name):
    p=bytearray((2,0x01,0x06))
    n=name.encode()
    p += bytes((len(n)+1,0x09)) + n
    return p

def start_advertising():
    try: ble.gap_advertise(100000,adv_data=advertising_payload(DEVICE_NAME))
    except Exception as e: print("Advertising error:",e)

def ble_irq(event,data):
    global ble_connected,rx_buffer

    if event==_IRQ_CENTRAL_CONNECT:
        conn,_,_=data
        connections.add(conn)
        ble_connected=True
        update_oled()
        send_mode()
        send_config()
        send_led_state()
        send_sensor_telemetry()
        send_battery()
        send_mpu()

    elif event==_IRQ_CENTRAL_DISCONNECT:
        conn,_,_=data
        connections.discard(conn)
        ble_connected=len(connections)>0
        absolute_stop()
        update_oled()
        start_advertising()

    elif event==_IRQ_GATTS_WRITE:
        _,attr=data
        if attr!=rx_handle: return
        try:
            rx_buffer += ble.gatts_read(rx_handle).decode("utf-8")
            rx_buffer = rx_buffer.replace("\r\n","\n").replace("\r","\n")
            while "\n" in rx_buffer:
                line,rx_buffer=rx_buffer.split("\n",1)
                line=line.strip()
                if line: process_command(line)
        except Exception as e:
            print("BLE RX error:",e)

ble.irq(ble_irq)

print("BluBot Firmware",FIRMWARE_VERSION)
coast_motor_stop()
reset_led_animation()
read_battery()
read_sharp()
read_tof()
read_mpu()
calibrate_gyro()
read_mpu()
update_led_animation()
update_oled()
start_advertising()

now=time.ticks_ms()
sharp_last=tof_last=mpu_control_last=telemetry_last=mpu_telemetry_last=battery_last=oled_last=now

while True:
    now=time.ticks_ms()
    update_emergency_brake()

    if time.ticks_diff(now,tof_last)>=TOF_INTERVAL_MS:
        tof_last=now
        read_tof()
        update_cliff_safety()

    if time.ticks_diff(now,sharp_last)>=SHARP_INTERVAL_MS:
        sharp_last=now
        read_sharp()
        update_front_safety()

    if time.ticks_diff(now,mpu_control_last)>=MPU_CONTROL_INTERVAL_MS:
        mpu_control_last=now
        read_mpu()

    update_autonomous()
    update_drive_control()
    update_led_animation()

    if time.ticks_diff(now,battery_last)>=BATTERY_INTERVAL_MS:
        battery_last=now
        read_battery()
        if ble_connected and not ota.active: send_battery()

    if time.ticks_diff(now,telemetry_last)>=FAST_TELEMETRY_INTERVAL_MS:
        telemetry_last=now
        if ble_connected and not ota.active: send_sensor_telemetry()

    if time.ticks_diff(now,mpu_telemetry_last)>=MPU_TELEMETRY_INTERVAL_MS:
        mpu_telemetry_last=now
        if ble_connected and not ota.active: send_mpu()

    if time.ticks_diff(now,oled_last)>=OLED_INTERVAL_MS:
        oled_last=now
        update_oled()

    if settings_dirty and time.ticks_diff(now,settings_dirty_since)>=SETTINGS_SAVE_DELAY_MS:
        save_current_settings()

    time.sleep_ms(2)


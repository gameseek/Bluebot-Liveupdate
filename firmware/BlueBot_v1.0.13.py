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


# ============================================================
# BLUBOT FIRMWARE
# ============================================================

FIRMWARE_VERSION = "1.0.13"
SETTINGS_VERSION = "1.0.2"


from machine import Pin, PWM, ADC, SoftI2C
import bluetooth
import neopixel
import time
import math
import json
import os
import struct

import ssd1306
from ota import OTAUpdater


try:
    from VL53L0X_V1 import VL53L0X
    VL53_AVAILABLE = True
except:
    VL53_AVAILABLE = False


# ============================================================
# GENERAL
# ============================================================

DEVICE_NAME = "BluBot"
SETTINGS_FILE = "settings.json"


# ============================================================
# MODES
# ============================================================

MODE_REMOTE = 1
MODE_TRACKING = 2
MODE_AUTO = 3


# ============================================================
# LED MODES
# ============================================================

LED_OFF = 0
LED_SOLID = 1
LED_PULSE = 2
LED_RAINBOW = 3
LED_STROBE = 4
LED_CHASE = 5
LED_BREATHING = 6


# ============================================================
# LOOP / SENSOR TIMING
# ============================================================

SHARP_INTERVAL_MS = 10

# Critical downward sensor
TOF_INTERVAL_MS = 10

MPU_CONTROL_INTERVAL_MS = 10

FAST_TELEMETRY_INTERVAL_MS = 50
MPU_TELEMETRY_INTERVAL_MS = 100

BATTERY_INTERVAL_MS = 1000
OLED_INTERVAL_MS = 100

SETTINGS_SAVE_DELAY_MS = 1500


# ============================================================
# BRAKING
# ============================================================

EMERGENCY_BRAKE_MS = 90


# ============================================================
# FRONT OBJECT SAFETY
# ============================================================

FRONT_HARD_STOP_CM = 15.0

FRONT_MAX_BRAKE_CM = 40.0

# v1.0.13:
# clearing an obstacle does NOT depend on the original
# high-speed trigger distance anymore.
FRONT_CLEAR_CM = 22.0
FRONT_CLEAR_COUNT = 2

FRONT_CONFIRM_COUNT = 1


# ============================================================
# CLIFF SAFETY
# ============================================================

CLIFF_THRESHOLD_MM = 120

# A VALID cliff result stops immediately.
CLIFF_CONFIRM_COUNT = 1

# If ToF loses the floor completely while moving,
# repeated invalid measurements are also treated as danger.
TOF_INVALID_CLIFF_COUNT = 2

CLIFF_BRAKE_MS = 80

CLIFF_BACKOFF_MS = 520
CLIFF_BACKOFF_POWER = 0.38

CLIFF_HOLD_MS = 150


SAFETY_NORMAL = 0
SAFETY_CLIFF_BRAKE = 1
SAFETY_CLIFF_BACKOFF = 2
SAFETY_CLIFF_HOLD = 3


# ============================================================
# MPU
# ============================================================

DRIVE_RAMP_PER_SEC = 3.0

GYRO_TURN_MAX_DPS = 160.0
GYRO_STEER_KP = 0.22

MAX_GYRO_CORRECTION = 0.22

GYRO_Z_SIGN = 1.0

STRAIGHT_DEADZONE = 0.08

TILT_STOP_DEG = 55.0
TILT_CONFIRM_COUNT = 5


# ============================================================
# AUTONOMOUS NAVIGATION STATES
# ============================================================

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


# ============================================================
# AUTONOMOUS PARAMETERS
# ============================================================

# Configured app speed is used for normal forward travel.
#
# Turns deliberately use a slower fixed speed.
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

# Prevent a damaged/mis-signed gyro from trapping
# the robot forever inside a turn.
AUTO_TURN_TIMEOUT_MS = 1400


# ============================================================
# SAFETY LED
# ============================================================

SAFETY_FLASH_PERIOD_MS = 300


# ============================================================
# BATTERY
# ============================================================

ADC_REFERENCE_V = 3.30

BATTERY_ADC_FACTOR = 4.3333

BATTERY_EMPTY_V = 6.0
BATTERY_FULL_V = 8.4


# ============================================================
# DEFAULT SETTINGS
# ============================================================

DEFAULT_SETTINGS = {

    "settings_version": SETTINGS_VERSION,

    "mode": MODE_REMOTE,
    "speed": 75,

    "p": 0.0,
    "i": 0.0,
    "d": 0.0,

    "led_r": 255,
    "led_g": 255,
    "led_b": 255,

    "led_brightness": 100,

    "led_animation": LED_STROBE,

    "led_speed": 1000
}


# ============================================================
# GENERAL HELPERS
# ============================================================

def clamp(value, minimum, maximum):

    if value < minimum:
        return minimum

    if value > maximum:
        return maximum

    return value


def angle_diff(a, b):

    diff = a - b

    while diff > 180:
        diff -= 360

    while diff < -180:
        diff += 360

    return diff


# ============================================================
# SETTINGS
# ============================================================

def write_settings(data):

    try:

        with open(
            SETTINGS_FILE,
            "w"
        ) as f:

            json.dump(
                data,
                f
            )

        return True

    except Exception as e:

        print(
            "Settings write error:",
            e
        )

        return False


def create_default_settings():

    data = DEFAULT_SETTINGS.copy()

    write_settings(
        data
    )

    return data


def migrate_settings(data):

    changed = False


    for key in DEFAULT_SETTINGS:

        if key == "settings_version":
            continue

        if key not in data:

            data[key] = (
                DEFAULT_SETTINGS[key]
            )

            changed = True


    data["settings_version"] = (
        SETTINGS_VERSION
    )


    if changed:

        write_settings(
            data
        )


    return data


def load_settings():

    try:

        if SETTINGS_FILE not in os.listdir():

            return create_default_settings()


        with open(
            SETTINGS_FILE,
            "r"
        ) as f:

            data = json.load(
                f
            )


        return migrate_settings(
            data
        )


    except Exception as e:

        print(
            "Settings load error:",
            e
        )

        return create_default_settings()


settings = load_settings()


current_mode = int(
    settings["mode"]
)

max_speed = int(
    settings["speed"]
)


pid_p = float(
    settings["p"]
)

pid_i = float(
    settings["i"]
)

pid_d = float(
    settings["d"]
)


led_r = int(
    settings["led_r"]
)

led_g = int(
    settings["led_g"]
)

led_b = int(
    settings["led_b"]
)

led_brightness = int(
    settings["led_brightness"]
)

led_animation = int(
    settings["led_animation"]
)

led_speed = int(
    settings["led_speed"]
)


settings_dirty = False
settings_dirty_since = 0


def save_current_settings():

    global settings_dirty


    write_settings(
        {
            "settings_version": SETTINGS_VERSION,

            "mode": int(
                current_mode
            ),

            "speed": int(
                max_speed
            ),

            "p": float(pid_p),
            "i": float(pid_i),
            "d": float(pid_d),

            "led_r": int(led_r),
            "led_g": int(led_g),
            "led_b": int(led_b),

            "led_brightness": int(
                led_brightness
            ),

            "led_animation": int(
                led_animation
            ),

            "led_speed": int(
                led_speed
            )
        }
    )


    settings_dirty = False


def mark_settings_dirty():

    global settings_dirty
    global settings_dirty_since


    settings_dirty = True

    settings_dirty_since = (
        time.ticks_ms()
    )


# ============================================================
# DRIVE STATE
# ============================================================

requested_left = 0.0
requested_right = 0.0

actual_left = 0.0
actual_right = 0.0

drive_left_output = 0.0
drive_right_output = 0.0

drive_last_time = (
    time.ticks_ms()
)


# ============================================================
# APP STOP
# ============================================================

app_stop_latched = False


# ============================================================
# MOTOR BRAKE
# ============================================================

motor_brake_active = False
motor_brake_start = 0


# ============================================================
# WALL SAFETY
# ============================================================

front_obstacle = False

front_danger_count = 0
front_clear_count = 0


# ============================================================
# CLIFF SAFETY
# ============================================================

safety_state = SAFETY_NORMAL

safety_state_start = (
    time.ticks_ms()
)

cliff_danger_count = 0

tof_invalid_count = 0

cliff_backoff_allowed = False


# ============================================================
# TILT
# ============================================================

tilt_danger_count = 0


# ============================================================
# AUTONOMOUS
# ============================================================

auto_running = False

auto_state = AUTO_IDLE

auto_state_start = (
    time.ticks_ms()
)


auto_left_target = 0.0
auto_right_target = 0.0


auto_heading = 0.0


auto_left_distance = 0.0
auto_right_distance = 0.0


auto_chosen_direction = 1


# Relative turn system
#
# -1 = left
# +1 = right

auto_turn_direction = 0

auto_turn_degrees = 0.0

auto_turn_start_yaw = 0.0

auto_turn_start_time = 0


# Alternate escape direction if both paths
# repeatedly appear blocked.
auto_escape_direction = -1


# ============================================================
# BLE
# ============================================================

ble_connected = False

connections = set()

rx_buffer = ""


# ============================================================
# SHARP SENSOR
# ============================================================

sharp_raw = 0

sharp_voltage = 0.0

sharp_distance_cm = -1.0

sharp_instant_cm = -1.0

sharp_filtered_cm = 0.0

sharp_filter_ready = False


# ============================================================
# TOF
# ============================================================

tof_available = False

tof_distance_mm = -1

tof_valid = False

floor_cliff = 0


# ============================================================
# MPU
# ============================================================

mpu_available = False

mpu_pitch = 0.0
mpu_roll = 0.0
mpu_yaw = 0.0

gyro_z_dps = 0.0
gyro_z_bias = 0.0

mpu_last_time = (
    time.ticks_ms()
)


# ============================================================
# BATTERY
# ============================================================

battery_voltage = 0.0

battery_percent = 0

battery_filtered = None


# ============================================================
# SAFETY LED
# ============================================================

safety_flash_start = (
    time.ticks_ms()
)


# ============================================================
# MOTOR HARDWARE
# ============================================================

ain1 = Pin(
    AIN1_PIN,
    Pin.OUT
)

ain2 = Pin(
    AIN2_PIN,
    Pin.OUT
)

bin1 = Pin(
    BIN1_PIN,
    Pin.OUT
)

bin2 = Pin(
    BIN2_PIN,
    Pin.OUT
)


pwma = PWM(
    Pin(PWMA_PIN)
)

pwmb = PWM(
    Pin(PWMB_PIN)
)


pwma.freq(20000)
pwmb.freq(20000)

pwma.duty_u16(0)
pwmb.duty_u16(0)


# ============================================================
# NEOPIXEL
# ============================================================

np = neopixel.NeoPixel(

    Pin(
        NEOPIXEL_PIN,
        Pin.OUT
    ),

    NEOPIXEL_COUNT
)


# ============================================================
# ADC
# ============================================================

sharp_adc = ADC(
    Pin(SHARP_PIN)
)

battery_adc = ADC(
    Pin(BATTERY_PIN)
)


try:

    sharp_adc.atten(
        ADC.ATTN_11DB
    )

except:
    pass


try:

    battery_adc.atten(
        ADC.ATTN_11DB
    )

except:
    pass


# ============================================================
# I2C
# ============================================================

i2c = SoftI2C(

    sda=Pin(SDA_PIN),

    scl=Pin(SCL_PIN),

    freq=400000
)


try:

    i2c_devices = (
        i2c.scan()
    )

except:

    i2c_devices = []


print(
    "I2C:",
    [
        hex(x)
        for x in i2c_devices
    ]
)


# ============================================================
# OLED
# ============================================================

oled = None


try:

    oled = ssd1306.SSD1306_I2C(
        128,
        64,
        i2c
    )

except Exception as e:

    print(
        "OLED not found:",
        e
    )


# ============================================================
# VL53L0X
# ============================================================

tof = None


if VL53_AVAILABLE:

    try:

        tof = VL53L0X(
            i2c
        )

        tof_available = True

        print(
            "VL53L0X connected"
        )

    except Exception as e:

        print(
            "TOF not found:",
            e
        )


# ============================================================
# MPU6050
# ============================================================

MPU_ADDR = 0x68


if MPU_ADDR in i2c_devices:

    try:

        i2c.writeto_mem(
            MPU_ADDR,
            0x6B,
            b"\x00"
        )

        time.sleep_ms(
            100
        )

        mpu_available = True

        print(
            "MPU6050 connected"
        )

    except Exception as e:

        print(
            "MPU error:",
            e
        )


# ============================================================
# MPU CALIBRATION
# ============================================================

def calibrate_gyro():

    global gyro_z_bias


    if not mpu_available:

        return


    print(
        "Keep robot still - gyro calibration"
    )


    total = 0.0
    count = 0


    for _ in range(100):

        try:

            raw = i2c.readfrom_mem(
                MPU_ADDR,
                0x43,
                6
            )


            gx, gy, gz = struct.unpack(
                ">hhh",
                raw
            )


            total += (
                gz /
                131.0
            )

            count += 1

        except:
            pass


        time.sleep_ms(
            5
        )


    if count > 0:

        gyro_z_bias = (
            total /
            count
        )


    print(
        "Gyro Z bias:",
        gyro_z_bias
    )


# ============================================================
# LOW-LEVEL MOTOR
# ============================================================

def drive_motor(
    in1,
    in2,
    pwm,
    value
):

    value = clamp(
        value,
        -1.0,
        1.0
    )


    if abs(value) < 0.03:

        value = 0.0


    if value == 0:

        in1.value(0)
        in2.value(0)

        pwm.duty_u16(0)

        return


    duty = int(
        abs(value) *
        65535
    )


    if value > 0:

        in1.value(1)
        in2.value(0)

    else:

        in1.value(0)
        in2.value(1)


    pwm.duty_u16(
        duty
    )


# ============================================================
# TB6612 SHORT BRAKE
# ============================================================

def brake_motor(
    in1,
    in2,
    pwm
):

    in1.value(1)
    in2.value(1)

    pwm.duty_u16(
        65535
    )


def release_motor_output():

    ain1.value(0)
    ain2.value(0)

    bin1.value(0)
    bin2.value(0)

    pwma.duty_u16(0)
    pwmb.duty_u16(0)


# ============================================================
# EMERGENCY BRAKE
# ============================================================

def start_emergency_brake():

    global motor_brake_active
    global motor_brake_start

    global actual_left
    global actual_right

    global drive_left_output
    global drive_right_output


    actual_left = 0.0
    actual_right = 0.0

    drive_left_output = 0.0
    drive_right_output = 0.0


    if not motor_brake_active:

        motor_brake_active = True

        motor_brake_start = (
            time.ticks_ms()
        )


        brake_motor(
            ain1,
            ain2,
            pwma
        )

        brake_motor(
            bin1,
            bin2,
            pwmb
        )


def update_emergency_brake():

    global motor_brake_active


    if not motor_brake_active:

        return


    if time.ticks_diff(
        time.ticks_ms(),
        motor_brake_start
    ) >= EMERGENCY_BRAKE_MS:

        motor_brake_active = False

        release_motor_output()


def coast_motor_stop():

    global actual_left
    global actual_right

    global drive_left_output
    global drive_right_output


    actual_left = 0.0
    actual_right = 0.0

    drive_left_output = 0.0
    drive_right_output = 0.0


    release_motor_output()


# ============================================================
# ABSOLUTE APP STOP
# ============================================================

def absolute_stop():

    global app_stop_latched

    global requested_left
    global requested_right

    global auto_running
    global auto_state

    global auto_left_target
    global auto_right_target

    global safety_state

    global cliff_danger_count
    global tof_invalid_count

    global front_danger_count


    app_stop_latched = True


    requested_left = 0.0
    requested_right = 0.0


    auto_running = False

    auto_state = AUTO_IDLE

    auto_left_target = 0.0
    auto_right_target = 0.0


    safety_state = SAFETY_NORMAL

    cliff_danger_count = 0
    tof_invalid_count = 0

    front_danger_count = 0


    start_emergency_brake()


# ============================================================
# NORMAL STOP
# ============================================================

def stop_motors():

    global requested_left
    global requested_right


    requested_left = 0.0
    requested_right = 0.0


    start_emergency_brake()


# ============================================================
# REMOTE MOVEMENT
# ============================================================

def set_motors(
    left,
    right
):

    global requested_left
    global requested_right

    global app_stop_latched


    if current_mode != MODE_REMOTE:

        return


    app_stop_latched = False


    left = clamp(
        left,
        -1.0,
        1.0
    )

    right = clamp(
        right,
        -1.0,
        1.0
    )


    if abs(left) < 0.03:
        left = 0.0

    if abs(right) < 0.03:
        right = 0.0


    requested_left = left
    requested_right = right


# ============================================================
# CURRENT MOVEMENT INTENT
# ============================================================

def robot_motion_commanded():

    if current_mode == MODE_REMOTE:

        return (
            abs(requested_left) > 0.03
            or
            abs(requested_right) > 0.03
            or
            abs(actual_left) > 0.03
            or
            abs(actual_right) > 0.03
        )


    if (
        current_mode == MODE_AUTO
        and
        auto_running
    ):

        return True


    return False


def forward_motion_commanded():

    if current_mode == MODE_REMOTE:

        requested_forward = (
            requested_left +
            requested_right
        ) / 2.0


        actual_forward = (
            actual_left +
            actual_right
        ) / 2.0


        return (
            requested_forward > 0.02
            or
            actual_forward > 0.02
        )


    if (
        current_mode == MODE_AUTO
        and
        auto_running
    ):

        target_forward = (
            auto_left_target +
            auto_right_target
        ) / 2.0


        return (
            target_forward > 0.02
            or
            auto_state == AUTO_FORWARD
        )


    return False


# ============================================================
# DYNAMIC WALL DISTANCE
# ============================================================

def get_front_stop_distance():

    forward = (
        actual_left +
        actual_right
    ) / 2.0


    forward = clamp(
        forward,
        0.0,
        1.0
    )


    return clamp(

        FRONT_HARD_STOP_CM
        +
        (
            FRONT_MAX_BRAKE_CM -
            FRONT_HARD_STOP_CM
        )
        *
        forward,

        FRONT_HARD_STOP_CM,

        FRONT_MAX_BRAKE_CM
    )


# ============================================================
# SAFETY LED
# ============================================================

def safety_red_now():

    global safety_flash_start


    safety_flash_start = (
        time.ticks_ms()
    )


    for i in range(
        NEOPIXEL_COUNT
    ):

        np[i] = (
            255,
            0,
            0
        )


    np.write()


# ============================================================
# FRONT SAFETY
# ============================================================

def update_front_safety():

    global front_obstacle

    global front_danger_count
    global front_clear_count

    global requested_left
    global requested_right

    global auto_left_target
    global auto_right_target


    if app_stop_latched:

        return


    if sharp_instant_cm < 0:

        return


    # ========================================================
    # ALREADY STOPPED FOR OBJECT
    #
    # v1.0.13:
    # obstacle clearing is completely independent of
    # movement commands.
    # ========================================================

    if front_obstacle:

        if (
            sharp_instant_cm >=
            FRONT_CLEAR_CM
        ):

            front_clear_count += 1

        else:

            front_clear_count = 0


        if (
            front_clear_count >=
            FRONT_CLEAR_COUNT
        ):

            front_obstacle = False

            front_clear_count = 0
            front_danger_count = 0


            print(
                "WALL CLEARED:",
                round(
                    sharp_instant_cm,
                    1
                ),
                "cm"
            )


            # Immediately restore selected LED animation.
            reset_led_animation()

            update_led_animation()


        return


    # ========================================================
    # DON'T DETECT FRONT COLLISION WHILE REVERSING OR SCANNING
    # ========================================================

    if not forward_motion_commanded():

        front_danger_count = 0

        return


    stop_distance = (
        get_front_stop_distance()
    )


    if (
        sharp_instant_cm <=
        stop_distance
    ):

        front_danger_count += 1

    else:

        front_danger_count = 0


    if (
        front_danger_count <
        FRONT_CONFIRM_COUNT
    ):

        return


    front_danger_count = 0
    front_clear_count = 0

    front_obstacle = True


    # Remote command must be cleared.
    if current_mode == MODE_REMOTE:

        requested_left = 0.0
        requested_right = 0.0


    # Auto controller will move into its wall recovery state.
    auto_left_target = 0.0
    auto_right_target = 0.0


    start_emergency_brake()

    safety_red_now()


    print(
        "WALL HARD BRAKE:",
        round(
            sharp_instant_cm,
            1
        ),
        "cm /",
        round(
            stop_distance,
            1
        ),
        "cm"
    )


# ============================================================
# CLIFF START
# ============================================================

def start_cliff_safety(
    reason="CLIFF"
):

    global safety_state
    global safety_state_start

    global cliff_danger_count
    global tof_invalid_count

    global cliff_backoff_allowed

    global requested_left
    global requested_right

    global auto_left_target
    global auto_right_target


    if app_stop_latched:

        return


    if (
        safety_state !=
        SAFETY_NORMAL
    ):

        return


    # Capture movement state BEFORE clearing commands.
    cliff_backoff_allowed = (
        robot_motion_commanded()
    )


    requested_left = 0.0
    requested_right = 0.0

    auto_left_target = 0.0
    auto_right_target = 0.0


    cliff_danger_count = 0
    tof_invalid_count = 0


    safety_state = (
        SAFETY_CLIFF_BRAKE
    )

    safety_state_start = (
        time.ticks_ms()
    )


    # Hard brake now, not after another timer.
    start_emergency_brake()

    safety_red_now()


    print(
        "!!!",
        reason,
        "- CLIFF HARD BRAKE !!!",
        tof_distance_mm,
        "backoff:",
        cliff_backoff_allowed
    )


# ============================================================
# CLIFF SAFETY UPDATE
# ============================================================

def update_cliff_safety():

    global safety_state
    global safety_state_start

    global cliff_danger_count
    global tof_invalid_count

    global requested_left
    global requested_right

    global auto_state


    if app_stop_latched:

        return


    now = (
        time.ticks_ms()
    )


    # ========================================================
    # NORMAL
    # ========================================================

    if (
        safety_state ==
        SAFETY_NORMAL
    ):

        # ----------------------------------------------------
        # VALID SENSOR READING
        # ----------------------------------------------------

        if tof_valid:

            tof_invalid_count = 0


            if floor_cliff == 1:

                cliff_danger_count += 1


                if (
                    cliff_danger_count >=
                    CLIFF_CONFIRM_COUNT
                ):

                    start_cliff_safety(
                        "VALID"
                    )


            else:

                cliff_danger_count = 0


            return


        # ----------------------------------------------------
        # INVALID READING
        #
        # At the edge VL53L0X can lose the return completely.
        #
        # v1.0.12 ignored this, which could allow the robot
        # to fall before reporting CLIFF.
        #
        # Only treat repeated invalid readings as danger while
        # the robot is actually commanded to move.
        # ----------------------------------------------------

        if (
            robot_motion_commanded()
        ):

            tof_invalid_count += 1


            if (
                tof_invalid_count >=
                TOF_INVALID_CLIFF_COUNT
            ):

                start_cliff_safety(
                    "NO FLOOR RETURN"
                )


        else:

            tof_invalid_count = 0


        return


    # ========================================================
    # CLIFF HARD-BRAKE PHASE
    # ========================================================

    if (
        safety_state ==
        SAFETY_CLIFF_BRAKE
    ):

        if time.ticks_diff(
            now,
            safety_state_start
        ) < CLIFF_BRAKE_MS:

            return


        # User explicitly pressed STOP?
        if app_stop_latched:

            return


        if cliff_backoff_allowed:

            safety_state = (
                SAFETY_CLIFF_BACKOFF
            )


            safety_state_start = (
                now
            )


            print(
                "CLIFF -> REVERSE NOW"
            )


        else:

            safety_state = (
                SAFETY_CLIFF_HOLD
            )


            safety_state_start = (
                now
            )


        return


    # ========================================================
    # BACKOFF
    # ========================================================

    if (
        safety_state ==
        SAFETY_CLIFF_BACKOFF
    ):

        if time.ticks_diff(
            now,
            safety_state_start
        ) >= CLIFF_BACKOFF_MS:

            # Hard brake after reverse.
            start_emergency_brake()


            safety_state = (
                SAFETY_CLIFF_HOLD
            )


            safety_state_start = (
                now
            )


            print(
                "CLIFF REVERSE COMPLETE"
            )


        return


    # ========================================================
    # HOLD UNTIL FLOOR RETURNS
    # ========================================================

    if (
        safety_state ==
        SAFETY_CLIFF_HOLD
    ):

        if time.ticks_diff(
            now,
            safety_state_start
        ) < CLIFF_HOLD_MS:

            return


        if (
            tof_valid
            and
            floor_cliff == 0
        ):

            safety_state = (
                SAFETY_NORMAL
            )


            cliff_danger_count = 0
            tof_invalid_count = 0


            coast_motor_stop()


            print(
                "CLIFF RECOVERY: FLOOR SAFE"
            )


            reset_led_animation()


            if (
                current_mode == MODE_AUTO
                and
                auto_running
            ):

                # Do not simply drive toward the same edge.
                auto_state = (
                    AUTO_SCAN_PREP
                )

            else:

                requested_left = 0.0
                requested_right = 0.0


# ============================================================
# AUTONOMOUS TARGET
# ============================================================

def set_auto_target(
    left,
    right
):

    global auto_left_target
    global auto_right_target


    auto_left_target = clamp(
        left,
        -1.0,
        1.0
    )


    auto_right_target = clamp(
        right,
        -1.0,
        1.0
    )


# ============================================================
# AUTONOMOUS BRAKE
# ============================================================

def auto_brake():

    set_auto_target(
        0.0,
        0.0
    )

    start_emergency_brake()


# ============================================================
# RELATIVE AUTONOMOUS TURN
#
# direction:
# -1 = LEFT
# +1 = RIGHT
# ============================================================

def begin_auto_turn(
    direction,
    degrees
):

    global auto_turn_direction
    global auto_turn_degrees

    global auto_turn_start_yaw
    global auto_turn_start_time


    auto_turn_direction = (
        -1
        if direction < 0
        else 1
    )


    auto_turn_degrees = abs(
        degrees
    )


    auto_turn_start_yaw = (
        mpu_yaw
    )


    auto_turn_start_time = (
        time.ticks_ms()
    )


def update_auto_turn():

    if app_stop_latched:

        set_auto_target(
            0.0,
            0.0
        )

        return False


    now = (
        time.ticks_ms()
    )


    # --------------------------------------------------------
    # MPU angle check
    #
    # We intentionally use ABSOLUTE angle travelled.
    #
    # Therefore the code does not depend on whether your
    # particular MPU installation reports CW as + or -.
    # --------------------------------------------------------

    if mpu_available:

        turned = abs(
            angle_diff(
                mpu_yaw,
                auto_turn_start_yaw
            )
        )


        if (
            turned >=
            auto_turn_degrees
        ):

            set_auto_target(
                0.0,
                0.0
            )

            return True


    # --------------------------------------------------------
    # Safety timeout
    # --------------------------------------------------------

    if time.ticks_diff(
        now,
        auto_turn_start_time
    ) >= AUTO_TURN_TIMEOUT_MS:

        print(
            "AUTO TURN TIMEOUT"
        )


        set_auto_target(
            0.0,
            0.0
        )

        return True


    # --------------------------------------------------------
    # Physical turn command
    #
    # Existing robot mapping:
    #
    # LEFT:
    #   logical left negative,
    #   logical right positive
    #
    # RIGHT:
    #   logical left positive,
    #   logical right negative
    # --------------------------------------------------------

    if auto_turn_direction < 0:

        set_auto_target(
            -AUTO_TURN_POWER,
            AUTO_TURN_POWER
        )

    else:

        set_auto_target(
            AUTO_TURN_POWER,
            -AUTO_TURN_POWER
        )


    return False


# ============================================================
# START AUTONOMOUS
# ============================================================

def start_autonomous():

    global auto_running
    global auto_state

    global auto_heading

    global app_stop_latched


    if current_mode != MODE_AUTO:

        print(
            "AUTOSTART ignored - use MODE,3 first"
        )

        return


    app_stop_latched = False


    auto_running = True

    auto_state = (
        AUTO_FORWARD
    )


    auto_heading = (
        mpu_yaw
    )


    set_auto_target(
        0.0,
        0.0
    )


    print(
        "AUTONOMOUS START"
    )


# ============================================================
# STOP AUTONOMOUS
# ============================================================

def stop_autonomous():

    absolute_stop()


    print(
        "AUTONOMOUS STOP"
    )


# ============================================================
# AUTONOMOUS STATE MACHINE
# ============================================================

def update_autonomous():

    global auto_state
    global auto_state_start

    global auto_heading

    global auto_left_distance
    global auto_right_distance

    global auto_chosen_direction

    global auto_escape_direction


    if app_stop_latched:

        set_auto_target(
            0.0,
            0.0
        )

        return


    if (
        current_mode !=
        MODE_AUTO
    ):

        return


    if not auto_running:

        set_auto_target(
            0.0,
            0.0
        )

        return


    # Cliff has complete priority over autonomous state.
    if (
        safety_state !=
        SAFETY_NORMAL
    ):

        set_auto_target(
            0.0,
            0.0
        )

        return


    now = (
        time.ticks_ms()
    )


    # ========================================================
    # AUTO FORWARD
    # ========================================================

    if (
        auto_state ==
        AUTO_FORWARD
    ):

        if front_obstacle:

            set_auto_target(
                0.0,
                0.0
            )


            auto_state = (
                AUTO_WALL_BRAKE
            )


            auto_state_start = (
                now
            )


            print(
                "AUTO WALL -> BRAKE"
            )


            return


        speed = clamp(
            max_speed /
            100.0,
            0.0,
            1.0
        )


        if mpu_available:

            heading_error = (
                angle_diff(
                    auto_heading,
                    mpu_yaw
                )
            )


            correction = clamp(

                heading_error *
                AUTO_HEADING_KP,

                -AUTO_MAX_HEADING_CORRECTION,

                AUTO_MAX_HEADING_CORRECTION
            )

        else:

            correction = 0.0


        set_auto_target(

            clamp(
                speed +
                correction,
                0.0,
                1.0
            ),

            clamp(
                speed -
                correction,
                0.0,
                1.0
            )
        )


        return


    # ========================================================
    # WALL BRAKE
    # ========================================================

    if (
        auto_state ==
        AUTO_WALL_BRAKE
    ):

        set_auto_target(
            0.0,
            0.0
        )


        if motor_brake_active:

            return


        auto_state = (
            AUTO_WALL_REVERSE
        )


        auto_state_start = (
            now
        )


        print(
            "AUTO WALL -> REVERSE"
        )


        return


    # ========================================================
    # WALL REVERSE
    # ========================================================

    if (
        auto_state ==
        AUTO_WALL_REVERSE
    ):

        set_auto_target(
            -AUTO_WALL_REVERSE_POWER,
            -AUTO_WALL_REVERSE_POWER
        )


        if time.ticks_diff(
            now,
            auto_state_start
        ) >= AUTO_WALL_REVERSE_MS:

            auto_brake()


            auto_state = (
                AUTO_SCAN_PREP
            )


            auto_state_start = (
                now
            )


            print(
                "AUTO REVERSE COMPLETE -> SCAN"
            )


        return


    # ========================================================
    # PREPARE LEFT/RIGHT SCAN
    # ========================================================

    if (
        auto_state ==
        AUTO_SCAN_PREP
    ):

        set_auto_target(
            0.0,
            0.0
        )


        if motor_brake_active:

            return


        begin_auto_turn(
            -1,
            AUTO_SCAN_ANGLE_DEG
        )


        auto_state = (
            AUTO_SCAN_LEFT
        )


        print(
            "AUTO SCAN LEFT"
        )


        return


    # ========================================================
    # TURN LEFT FOR SCAN
    # ========================================================

    if (
        auto_state ==
        AUTO_SCAN_LEFT
    ):

        if update_auto_turn():

            auto_brake()


            auto_state = (
                AUTO_SCAN_LEFT_SETTLE
            )


            auto_state_start = (
                now
            )


        return


    # ========================================================
    # MEASURE LEFT
    # ========================================================

    if (
        auto_state ==
        AUTO_SCAN_LEFT_SETTLE
    ):

        set_auto_target(
            0.0,
            0.0
        )


        if motor_brake_active:

            return


        if time.ticks_diff(
            now,
            auto_state_start
        ) < AUTO_SCAN_SETTLE_MS:

            return


        auto_left_distance = (
            sharp_instant_cm
            if sharp_instant_cm >= 0
            else 0
        )


        print(
            "AUTO LEFT CLEARANCE:",
            round(
                auto_left_distance,
                1
            )
        )


        # Return the same relative angle.
        begin_auto_turn(
            1,
            AUTO_SCAN_ANGLE_DEG
        )


        auto_state = (
            AUTO_RETURN_CENTER_1
        )


        return


    # ========================================================
    # RETURN TO CENTER AFTER LEFT SCAN
    # ========================================================

    if (
        auto_state ==
        AUTO_RETURN_CENTER_1
    ):

        if update_auto_turn():

            auto_brake()


            # Then rotate right from center.
            begin_auto_turn(
                1,
                AUTO_SCAN_ANGLE_DEG
            )


            auto_state = (
                AUTO_SCAN_RIGHT
            )


            print(
                "AUTO SCAN RIGHT"
            )


        return


    # ========================================================
    # RIGHT SCAN
    # ========================================================

    if (
        auto_state ==
        AUTO_SCAN_RIGHT
    ):

        if motor_brake_active:

            return


        if update_auto_turn():

            auto_brake()


            auto_state = (
                AUTO_SCAN_RIGHT_SETTLE
            )


            auto_state_start = (
                now
            )


        return


    # ========================================================
    # MEASURE RIGHT
    # ========================================================

    if (
        auto_state ==
        AUTO_SCAN_RIGHT_SETTLE
    ):

        set_auto_target(
            0.0,
            0.0
        )


        if motor_brake_active:

            return


        if time.ticks_diff(
            now,
            auto_state_start
        ) < AUTO_SCAN_SETTLE_MS:

            return


        auto_right_distance = (
            sharp_instant_cm
            if sharp_instant_cm >= 0
            else 0
        )


        print(
            "AUTO RIGHT CLEARANCE:",
            round(
                auto_right_distance,
                1
            )
        )


        # Always return to center BEFORE choosing.
        #
        # This prevents the left/right oscillation
        # problem from v1.0.12.
        begin_auto_turn(
            -1,
            AUTO_SCAN_ANGLE_DEG
        )


        auto_state = (
            AUTO_RETURN_CENTER_2
        )


        return


    # ========================================================
    # RETURN TO CENTER AFTER RIGHT SCAN
    # ========================================================

    if (
        auto_state ==
        AUTO_RETURN_CENTER_2
    ):

        if update_auto_turn():

            auto_brake()


            auto_state = (
                AUTO_CHOOSE_PATH
            )


            auto_state_start = (
                now
            )


        return


    # ========================================================
    # CHOOSE PATH
    # ========================================================

    if (
        auto_state ==
        AUTO_CHOOSE_PATH
    ):

        set_auto_target(
            0.0,
            0.0
        )


        if motor_brake_active:

            return


        print(
            "PATH L/R:",
            round(
                auto_left_distance,
                1
            ),
            "/",
            round(
                auto_right_distance,
                1
            )
        )


        # ----------------------------------------------------
        # BOTH SIDES BLOCKED
        # ----------------------------------------------------

        if (
            auto_left_distance <
            AUTO_MIN_CLEAR_CM

            and

            auto_right_distance <
            AUTO_MIN_CLEAR_CM
        ):

            auto_state = (
                AUTO_ESCAPE_REVERSE
            )


            auto_state_start = (
                now
            )


            # Alternate escape direction.
            auto_escape_direction *= -1


            print(
                "AUTO BOTH BLOCKED -> ESCAPE"
            )


            return


        # ----------------------------------------------------
        # CHOOSE BIGGER CLEARANCE
        # ----------------------------------------------------

        if (
            auto_left_distance >=
            auto_right_distance
        ):

            auto_chosen_direction = -1

            print(
                "AUTO CHOOSE LEFT"
            )

        else:

            auto_chosen_direction = 1

            print(
                "AUTO CHOOSE RIGHT"
            )


        begin_auto_turn(
            auto_chosen_direction,
            AUTO_AVOID_TURN_DEG
        )


        auto_state = (
            AUTO_TURN_CHOSEN
        )


        return


    # ========================================================
    # TURN INTO CLEAR PATH
    # ========================================================

    if (
        auto_state ==
        AUTO_TURN_CHOSEN
    ):

        if update_auto_turn():

            auto_brake()


            auto_heading = (
                mpu_yaw
            )


            auto_state = (
                AUTO_FORWARD
            )


            print(
                "AUTO TURN COMPLETE -> FORWARD"
            )


        return


    # ========================================================
    # ESCAPE REVERSE
    # ========================================================

    if (
        auto_state ==
        AUTO_ESCAPE_REVERSE
    ):

        set_auto_target(
            -AUTO_ESCAPE_REVERSE_POWER,
            -AUTO_ESCAPE_REVERSE_POWER
        )


        if time.ticks_diff(
            now,
            auto_state_start
        ) >= AUTO_ESCAPE_REVERSE_MS:

            auto_brake()


            begin_auto_turn(
                auto_escape_direction,
                AUTO_ESCAPE_TURN_DEG
            )


            auto_state = (
                AUTO_ESCAPE_TURN
            )


            print(
                "AUTO ESCAPE TURN"
            )


        return


    # ========================================================
    # ESCAPE TURN
    # ========================================================

    if (
        auto_state ==
        AUTO_ESCAPE_TURN
    ):

        if motor_brake_active:

            return


        if update_auto_turn():

            auto_brake()


            auto_heading = (
                mpu_yaw
            )


            auto_state = (
                AUTO_FORWARD
            )


            print(
                "AUTO ESCAPE COMPLETE -> FORWARD"
            )


# ============================================================
# DRIVE CONTROL
# ============================================================

def update_drive_control():

    global actual_left
    global actual_right

    global drive_left_output
    global drive_right_output

    global drive_last_time

    global requested_left
    global requested_right

    global tilt_danger_count


    now = (
        time.ticks_ms()
    )


    dt = (
        time.ticks_diff(
            now,
            drive_last_time
        )
        /
        1000.0
    )


    drive_last_time = (
        now
    )


    if dt <= 0:

        return


    if dt > 0.1:

        dt = 0.1


    # ========================================================
    # PRIORITY 1: APP STOP
    # ========================================================

    if app_stop_latched:

        if not motor_brake_active:

            release_motor_output()

        return


    # ========================================================
    # HARD BRAKE
    # ========================================================

    if motor_brake_active:

        return


    # ========================================================
    # CLIFF BRAKE
    # ========================================================

    if (
        safety_state ==
        SAFETY_CLIFF_BRAKE
    ):

        coast_motor_stop()

        return


    # ========================================================
    # CLIFF BACKOFF
    #
    # v1.0.13:
    # bypass normal smooth ramp.
    #
    # We need reverse movement immediately after
    # the hard-brake period.
    # ========================================================

    if (
        safety_state ==
        SAFETY_CLIFF_BACKOFF
    ):

        actual_left = (
            -CLIFF_BACKOFF_POWER
        )

        actual_right = (
            -CLIFF_BACKOFF_POWER
        )


        drive_left_output = (
            actual_left
        )

        drive_right_output = (
            actual_right
        )


        physical_left = (
            actual_right
        )

        physical_right = (
            -actual_left
        )


        drive_motor(
            ain1,
            ain2,
            pwma,
            physical_left
        )


        drive_motor(
            bin1,
            bin2,
            pwmb,
            physical_right
        )


        return


    # ========================================================
    # CLIFF HOLD
    # ========================================================

    if (
        safety_state ==
        SAFETY_CLIFF_HOLD
    ):

        coast_motor_stop()

        return


    # ========================================================
    # REMOTE MODE
    # ========================================================

    if (
        current_mode ==
        MODE_REMOTE
    ):

        target_input_left = (
            requested_left
        )

        target_input_right = (
            requested_right
        )


        if front_obstacle:

            forward = (
                target_input_left +
                target_input_right
            ) / 2.0


            if forward > 0.02:

                coast_motor_stop()

                return


        throttle = (
            target_input_left +
            target_input_right
        ) / 2.0


        turn = (
            target_input_left -
            target_input_right
        ) / 2.0


        corrected_turn = turn


        if mpu_available:

            speed_scale = (
                max_speed /
                100.0
            )


            if (
                abs(turn) <
                STRAIGHT_DEADZONE
            ):

                desired_yaw_rate = 0.0

            else:

                desired_yaw_rate = (
                    turn *
                    GYRO_TURN_MAX_DPS *
                    speed_scale
                )


            yaw_error = (
                desired_yaw_rate -
                gyro_z_dps
            )


            correction = (
                yaw_error /
                GYRO_TURN_MAX_DPS
            ) * GYRO_STEER_KP


            correction = clamp(
                correction,
                -MAX_GYRO_CORRECTION,
                MAX_GYRO_CORRECTION
            )


            corrected_turn = clamp(
                turn +
                correction,
                -1.0,
                1.0
            )


        target_left = clamp(
            throttle +
            corrected_turn,
            -1.0,
            1.0
        )


        target_right = clamp(
            throttle -
            corrected_turn,
            -1.0,
            1.0
        )


        speed_factor = (
            max_speed /
            100.0
        )


        target_left *= (
            speed_factor
        )

        target_right *= (
            speed_factor
        )


    # ========================================================
    # AUTO MODE
    # ========================================================

    elif (
        current_mode ==
        MODE_AUTO

        and

        auto_running
    ):

        target_left = (
            auto_left_target
        )

        target_right = (
            auto_right_target
        )


    else:

        coast_motor_stop()

        return


    # ========================================================
    # MPU TILT SAFETY
    # ========================================================

    if mpu_available:

        if (
            abs(mpu_pitch) >
            TILT_STOP_DEG

            or

            abs(mpu_roll) >
            TILT_STOP_DEG
        ):

            tilt_danger_count += 1

        else:

            tilt_danger_count = 0


        if (
            tilt_danger_count >=
            TILT_CONFIRM_COUNT
        ):

            tilt_danger_count = 0


            print(
                "TILT SAFETY STOP"
            )


            absolute_stop()

            safety_red_now()

            return


    # ========================================================
    # NORMAL SMOOTH RAMP
    # ========================================================

    max_change = (
        DRIVE_RAMP_PER_SEC *
        dt
    )


    if (
        target_left >
        drive_left_output
    ):

        drive_left_output = min(

            target_left,

            drive_left_output +
            max_change
        )

    else:

        drive_left_output = max(

            target_left,

            drive_left_output -
            max_change
        )


    if (
        target_right >
        drive_right_output
    ):

        drive_right_output = min(

            target_right,

            drive_right_output +
            max_change
        )

    else:

        drive_right_output = max(

            target_right,

            drive_right_output -
            max_change
        )


    actual_left = (
        drive_left_output
    )

    actual_right = (
        drive_right_output
    )


    # ========================================================
    # EXISTING PHYSICAL MOTOR MAPPING
    # ========================================================

    physical_left = (
        actual_right
    )

    physical_right = (
        -actual_left
    )


    drive_motor(
        ain1,
        ain2,
        pwma,
        physical_left
    )


    drive_motor(
        bin1,
        bin2,
        pwmb,
        physical_right
    )


# ============================================================
# LED FUNCTIONS
# ============================================================

def scale_led_color(
    r,
    g,
    b,
    brightness=None
):

    if brightness is None:

        brightness = (
            led_brightness
        )


    factor = clamp(
        brightness,
        0,
        100
    ) / 100.0


    return (
        int(r * factor),
        int(g * factor),
        int(b * factor)
    )


def leds_off():

    for i in range(
        NEOPIXEL_COUNT
    ):

        np[i] = (
            0,
            0,
            0
        )


    np.write()


def set_all_leds(
    r,
    g,
    b,
    brightness=None
):

    color = scale_led_color(
        r,
        g,
        b,
        brightness
    )


    for i in range(
        NEOPIXEL_COUNT
    ):

        np[i] = (
            color
        )


    np.write()


def color_wheel(position):

    position %= 256


    if position < 85:

        return (
            255-position*3,
            position*3,
            0
        )


    if position < 170:

        position -= 85

        return (
            0,
            255-position*3,
            position*3
        )


    position -= 170


    return (
        position*3,
        0,
        255-position*3
    )


animation_start = (
    time.ticks_ms()
)


def reset_led_animation():

    global animation_start


    animation_start = (
        time.ticks_ms()
    )


def safety_led_required():

    return (
        front_obstacle

        or

        safety_state !=
        SAFETY_NORMAL
    )


def update_safety_led():

    phase = (
        time.ticks_diff(
            time.ticks_ms(),
            safety_flash_start
        )
        %
        SAFETY_FLASH_PERIOD_MS
    )


    if phase < (
        SAFETY_FLASH_PERIOD_MS //
        2
    ):

        set_all_leds(
            255,
            0,
            0,
            100
        )

    else:

        leds_off()


def update_led_animation():

    now = (
        time.ticks_ms()
    )


    if safety_led_required():

        update_safety_led()

        return


    if led_animation == LED_OFF:

        leds_off()

        return


    if led_animation == LED_SOLID:

        set_all_leds(
            led_r,
            led_g,
            led_b
        )

        return


    period = max(
        50,
        led_speed
    )


    phase = (
        time.ticks_diff(
            now,
            animation_start
        )
        %
        period
    ) / period


    # ========================================================
    # HEARTBEAT
    # ========================================================

    if led_animation == LED_PULSE:

        if phase < 0.10:

            level = (
                phase /
                0.10
            )

        elif phase < 0.18:

            level = (
                1.0 -
                ((phase-0.10)/0.08)
                *
                0.65
            )

        elif phase < 0.26:

            level = (
                0.35 +
                ((phase-0.18)/0.08)
                *
                0.65
            )

        elif phase < 0.42:

            level = (
                1.0 -
                ((phase-0.26)/0.16)
            )

        else:

            level = 0


        level = clamp(
            level,
            0,
            1
        )


        set_all_leds(
            led_r,
            led_g,
            led_b,
            int(
                led_brightness *
                level
            )
        )


        return


    # ========================================================
    # RAINBOW
    # ========================================================

    if led_animation == LED_RAINBOW:

        position = int(
            phase *
            255
        )


        for i in range(
            NEOPIXEL_COUNT
        ):

            offset = int(
                i *
                256 /
                NEOPIXEL_COUNT
            )


            r, g, b = (
                color_wheel(
                    position +
                    offset
                )
            )


            np[i] = (
                scale_led_color(
                    r,
                    g,
                    b
                )
            )


        np.write()

        return


    # ========================================================
    # TRIPLE WHITE STROBE
    # ========================================================

    if led_animation == LED_STROBE:

        period = max(
            400,
            led_speed
        )


        phase = (
            time.ticks_diff(
                now,
                animation_start
            )
            %
            period
        ) / period


        flash = (
            phase < 0.07
            or
            0.14 <= phase < 0.21
            or
            0.28 <= phase < 0.35
        )


        if flash:

            set_all_leds(
                255,
                255,
                255,
                led_brightness
            )

        else:

            leds_off()


        return


    # ========================================================
    # CHASE
    # ========================================================

    if led_animation == LED_CHASE:

        position = int(
            phase *
            NEOPIXEL_COUNT
        )


        if (
            position >=
            NEOPIXEL_COUNT
        ):

            position = (
                NEOPIXEL_COUNT -
                1
            )


        for i in range(
            NEOPIXEL_COUNT
        ):

            np[i] = (
                0,
                0,
                0
            )


        np[position] = (
            scale_led_color(
                led_r,
                led_g,
                led_b
            )
        )


        np.write()

        return


    # ========================================================
    # BREATHING
    # ========================================================

    if (
        led_animation ==
        LED_BREATHING
    ):

        level = (
            math.sin(
                phase *
                2 *
                math.pi -
                math.pi/2
            ) + 1
        ) / 2


        set_all_leds(
            led_r,
            led_g,
            led_b,
            int(
                led_brightness *
                level
            )
        )


# ============================================================
# BATTERY
# ============================================================

def read_battery():

    global battery_voltage
    global battery_percent
    global battery_filtered


    try:

        time.sleep_ms(5)


        total = 0


        for _ in range(16):

            total += (
                battery_adc.read_u16()
            )

            time.sleep_us(
                200
            )


        raw = (
            total /
            16
        )


        adc_voltage = (
            raw *
            ADC_REFERENCE_V /
            65535
        )


        battery_voltage = (
            adc_voltage *
            BATTERY_ADC_FACTOR
        )


        pct = int(
            (
                battery_voltage -
                BATTERY_EMPTY_V
            )
            /
            (
                BATTERY_FULL_V -
                BATTERY_EMPTY_V
            )
            *
            100
        )


        pct = max(
            0,
            min(
                100,
                pct
            )
        )


        if battery_filtered is None:

            battery_filtered = (
                pct
            )

        else:

            battery_filtered = int(
                battery_filtered *
                0.9
                +
                pct *
                0.1
            )


        battery_percent = (
            battery_filtered
        )


        print(
            "BAT RAW:",
            int(raw),
            "ADC:",
            round(
                adc_voltage,
                3
            ),
            "BAT:",
            round(
                battery_voltage,
                2
            ),
            "V",
            battery_percent,
            "%"
        )


    except Exception as e:

        print(
            "Battery error:",
            e
        )


# ============================================================
# SHARP GP2Y0E03
# ============================================================

def sharp_voltage_to_distance(
    voltage
):

    if voltage <= 0.05:

        return 50.0


    raw_distance = (
        13.0 /
        voltage
    )


    if raw_distance >= 40.0:

        return 50.0


    x = (
        raw_distance -
        3.36
    )


    if x <= 0.05:

        return 4.0


    distance = (
        0.56 +
        15.99 *
        math.log(x)
    )


    return clamp(
        distance,
        4.0,
        50.0
    )


def read_sharp():

    global sharp_raw
    global sharp_voltage

    global sharp_distance_cm
    global sharp_instant_cm

    global sharp_filtered_cm
    global sharp_filter_ready


    try:

        total = 0


        for _ in range(4):

            total += (
                sharp_adc.read_u16()
            )


        sharp_raw = int(
            total /
            4
        )


        sharp_voltage = (
            sharp_raw /
            65535.0
        ) * ADC_REFERENCE_V


        distance = (
            sharp_voltage_to_distance(
                sharp_voltage
            )
        )


        sharp_instant_cm = (
            distance
        )


        if not sharp_filter_ready:

            sharp_filtered_cm = (
                distance
            )

            sharp_filter_ready = True

        else:

            sharp_filtered_cm = (
                sharp_filtered_cm *
                0.75
                +
                distance *
                0.25
            )


        sharp_distance_cm = (
            sharp_filtered_cm
        )


    except Exception as e:

        print(
            "Sharp error:",
            e
        )


        sharp_distance_cm = -1
        sharp_instant_cm = -1


# ============================================================
# TOF
# ============================================================

def get_tof_reading():

    if tof is None:

        return -1


    try:

        if hasattr(
            tof,
            "read"
        ):

            return int(
                tof.read()
            )

    except:
        pass


    try:

        if hasattr(
            tof,
            "ping"
        ):

            return int(
                tof.ping()
            )

    except:
        pass


    try:

        if hasattr(
            tof,
            "range"
        ):

            value = (
                tof.range
            )


            if callable(value):

                return int(
                    value()
                )


            return int(
                value
            )

    except:
        pass


    return -1


def read_tof():

    global tof_distance_mm
    global tof_valid
    global floor_cliff


    if not tof_available:

        tof_distance_mm = -1
        tof_valid = False

        return


    value = (
        get_tof_reading()
    )


    if (
        value <= 0
        or
        value >= 8190
    ):

        tof_distance_mm = -1

        tof_valid = False

        return


    tof_distance_mm = (
        value
    )

    tof_valid = True


    if (
        value <=
        CLIFF_THRESHOLD_MM
    ):

        floor_cliff = 0

    else:

        floor_cliff = 1


# ============================================================
# MPU6050
# ============================================================

def read_mpu():

    global mpu_pitch
    global mpu_roll
    global mpu_yaw

    global gyro_z_dps
    global mpu_last_time


    if not mpu_available:

        return


    try:

        now = (
            time.ticks_ms()
        )


        dt = (
            time.ticks_diff(
                now,
                mpu_last_time
            )
            /
            1000.0
        )


        mpu_last_time = (
            now
        )


        if (
            dt <= 0
            or
            dt > 1
        ):

            dt = 0.01


        raw = i2c.readfrom_mem(
            MPU_ADDR,
            0x3B,
            14
        )


        (
            ax_raw,
            ay_raw,
            az_raw,
            temp_raw,
            gx_raw,
            gy_raw,
            gz_raw

        ) = struct.unpack(
            ">hhhhhhh",
            raw
        )


        ax = (
            ax_raw /
            16384.0
        )

        ay = (
            ay_raw /
            16384.0
        )

        az = (
            az_raw /
            16384.0
        )


        gyro_z_dps = (
            (
                gz_raw /
                131.0
            )
            -
            gyro_z_bias
        ) * GYRO_Z_SIGN


        mpu_roll = (
            math.atan2(
                ay,
                az
            )
            *
            57.2957795
        )


        mpu_pitch = (
            math.atan2(

                -ax,

                math.sqrt(
                    ay*ay +
                    az*az
                )

            )
            *
            57.2957795
        )


        mpu_yaw += (
            gyro_z_dps *
            dt
        )


        while mpu_yaw > 180:

            mpu_yaw -= 360


        while mpu_yaw < -180:

            mpu_yaw += 360


    except Exception as e:

        print(
            "MPU error:",
            e
        )


# ============================================================
# MODE NAME
# ============================================================

def mode_name():

    if current_mode == MODE_REMOTE:

        return "REMOTE"


    if current_mode == MODE_TRACKING:

        return "TRACK"


    if current_mode == MODE_AUTO:

        if auto_running:

            return "AUTO"

        return "AUTO WAIT"


    return "?"


# ============================================================
# MOVEMENT NAME
# ============================================================

def movement_name():

    if app_stop_latched:

        return "STOP"


    if (
        safety_state ==
        SAFETY_CLIFF_BRAKE
    ):

        return "CLIFF BRAKE"


    if (
        safety_state ==
        SAFETY_CLIFF_BACKOFF
    ):

        return "CLIFF BACK"


    if (
        safety_state ==
        SAFETY_CLIFF_HOLD
    ):

        return "CLIFF HOLD"


    if (
        current_mode == MODE_AUTO
        and
        auto_running
    ):

        if auto_state == AUTO_FORWARD:
            return "AUTO FWD"

        if auto_state == AUTO_WALL_BRAKE:
            return "WALL BRAKE"

        if auto_state == AUTO_WALL_REVERSE:
            return "WALL BACK"

        if auto_state == AUTO_SCAN_PREP:
            return "SCAN PREP"

        if auto_state in (
            AUTO_SCAN_LEFT,
            AUTO_SCAN_LEFT_SETTLE
        ):
            return "SCAN LEFT"

        if auto_state == AUTO_RETURN_CENTER_1:
            return "CENTER"

        if auto_state in (
            AUTO_SCAN_RIGHT,
            AUTO_SCAN_RIGHT_SETTLE
        ):
            return "SCAN RIGHT"

        if auto_state == AUTO_RETURN_CENTER_2:
            return "CENTER"

        if auto_state == AUTO_CHOOSE_PATH:
            return "CHOOSE"

        if auto_state == AUTO_TURN_CHOSEN:
            return "PATH TURN"

        if auto_state == AUTO_ESCAPE_REVERSE:
            return "ESC BACK"

        if auto_state == AUTO_ESCAPE_TURN:
            return "ESC TURN"


    if front_obstacle:

        return "WALL STOP"


    left = requested_left
    right = requested_right


    if (
        abs(left) < 0.03
        and
        abs(right) < 0.03
    ):

        return "STOP"


    if (
        left > 0.03
        and
        right > 0.03
    ):

        return "FORWARD"


    if (
        left < -0.03
        and
        right < -0.03
    ):

        return "REVERSE"


    if (
        left > 0.03
        and
        right < -0.03
    ):

        return "SPIN RIGHT"


    if (
        left < -0.03
        and
        right > 0.03
    ):

        return "SPIN LEFT"


    return "MOVE"


# ============================================================
# BLE
# ============================================================

_IRQ_CENTRAL_CONNECT = 1
_IRQ_CENTRAL_DISCONNECT = 2
_IRQ_GATTS_WRITE = 3


UART_SERVICE_UUID = bluetooth.UUID(
    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
)


UART_TX_UUID = bluetooth.UUID(
    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
)


UART_RX_UUID = bluetooth.UUID(
    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
)


UART_SERVICE = (

    UART_SERVICE_UUID,

    (
        (
            UART_TX_UUID,
            bluetooth.FLAG_NOTIFY
        ),

        (
            UART_RX_UUID,

            bluetooth.FLAG_WRITE |
            bluetooth.FLAG_WRITE_NO_RESPONSE
        )
    )
)


ble = bluetooth.BLE()

ble.active(
    True
)


(
    (
        tx_handle,
        rx_handle
    ),
) = ble.gatts_register_services(
    (
        UART_SERVICE,
    )
)


ble.gatts_set_buffer(
    rx_handle,
    1024,
    True
)


def ble_send(message):

    if not connections:

        return


    data = (
        message.encode()
    )


    for connection in list(
        connections
    ):

        for pos in range(
            0,
            len(data),
            20
        ):

            try:

                ble.gatts_notify(
                    connection,
                    tx_handle,

                    data[
                        pos:
                        pos+20
                    ]
                )

            except Exception as e:

                print(
                    "BLE notify error:",
                    e
                )


# ============================================================
# OTA
# ============================================================

ota = OTAUpdater(
    ble_send,
    stop_motors
)


# ============================================================
# TELEMETRY
# ============================================================

def send_sensor_telemetry():

    ble_send(
        "S,{:.1f},{},{},{},{}\n".format(
            sharp_distance_cm,
            floor_cliff,
            battery_percent,
            current_mode,
            max_speed
        )
    )


def send_battery():

    ble_send(
        "BAT:{},{:.2f}\n".format(
            battery_percent,
            battery_voltage
        )
    )


def send_mpu():

    if not mpu_available:

        return


    ble_send(
        "MPU:{:.1f},{:.1f},{:.1f}\n".format(
            mpu_pitch,
            mpu_roll,
            mpu_yaw
        )
    )


def send_led_state():

    ble_send(
        "LED,{},{},{},{},{},{}\n".format(
            led_r,
            led_g,
            led_b,
            led_brightness,
            led_animation,
            led_speed
        )
    )


def send_config():

    ble_send(
        "CFG,{:.5f},{:.5f},{:.5f},{},{}\n".format(
            pid_p,
            pid_i,
            pid_d,
            max_speed,
            current_mode
        )
    )


def send_mode():

    ble_send(
        "MODE,{}\n".format(
            current_mode
        )
    )


# ============================================================
# OLED
# ============================================================

def update_oled():

    if oled is None:

        return


    try:

        oled.fill(0)


        if ota.active:

            oled.text(
                "BLE: CONNECTED"
                if ble_connected
                else
                "BLE: WAITING",
                0,
                0
            )


            oled.text(
                "FIRMWARE UPDATE",
                0,
                16
            )


            oled.text(
                "UPLOADING...",
                0,
                32
            )


            oled.text(
                "{}%".format(
                    ota.progress()
                ),
                0,
                48
            )


            oled.show()

            return


        oled.text(
            "BLE: CONNECTED"
            if ble_connected
            else
            "BLE: WAITING",
            0,
            0
        )


        oled.text(
            "MODE:{} S:{}%".format(
                mode_name(),
                max_speed
            )[:16],
            0,
            8
        )


        oled.text(
            "MOVE:{}".format(
                movement_name()
            )[:16],
            0,
            16
        )


        if sharp_distance_cm >= 0:

            oled.text(
                "SHARP:{:.1f}cm".format(
                    sharp_distance_cm
                )[:16],
                0,
                24
            )

        else:

            oled.text(
                "SHARP: ERROR",
                0,
                24
            )


        oled.text(
            "BAT:{}%".format(
                battery_percent
            ),
            0,
            32
        )


        oled.text(
            "MPU: CONNECTED"
            if mpu_available
            else
            "MPU: NOT FOUND",
            0,
            40
        )


        if not tof_available:

            tof_text = (
                "TOF: NOT FOUND"
            )

        elif not tof_valid:

            tof_text = (
                "TOF: WAITING"
            )

        elif floor_cliff == 0:

            tof_text = (
                "TOF: FLOOR"
            )

        else:

            tof_text = (
                "TOF: CLIFF"
            )


        oled.text(
            tof_text[:16],
            0,
            48
        )


        oled.text(
            "FW:{}".format(
                FIRMWARE_VERSION
            ),
            0,
            56
        )


        oled.show()


    except Exception as e:

        print(
            "OLED error:",
            e
        )


# ============================================================
# COMMANDS
# ============================================================

def process_command(command):

    global current_mode
    global max_speed

    global pid_p
    global pid_i
    global pid_d

    global led_r
    global led_g
    global led_b

    global led_brightness
    global led_animation
    global led_speed

    global app_stop_latched

    global auto_running
    global auto_state


    command = (
        command.strip()
    )


    if not command:

        return


    print(
        "APP:",
        command
    )


    # ========================================================
    # ABSOLUTE HIGHEST PRIORITY
    # ========================================================

    if command == "STOP":

        absolute_stop()

        print(
            "APP STOP"
        )

        return


    if command == "AUTOSTOP":

        stop_autonomous()

        return


    # ========================================================
    # VERSION
    # ========================================================

    if command == "FW?":

        ble_send(
            "FW,{}\n".format(
                FIRMWARE_VERSION
            )
        )

        return


    # ========================================================
    # OTA
    # ========================================================

    if command.startswith(
        "OTA_BEGIN,"
    ):

        try:

            parts = (
                command.split(",")
            )


            if len(parts) != 4:

                ble_send(
                    "OTA_ERROR,BAD_BEGIN\n"
                )

                return


            absolute_stop()


            ota.begin(
                parts[1],
                parts[2],
                parts[3]
            )

        except Exception as e:

            print(
                "OTA BEGIN error:",
                e
            )


            ble_send(
                "OTA_ERROR,BAD_BEGIN\n"
            )


        return


    if command.startswith(
        "OTA_DATA,"
    ):

        try:

            parts = command.split(
                ",",
                2
            )


            if len(parts) != 3:

                ble_send(
                    "OTA_ERROR,BAD_DATA\n"
                )

                return


            ota.receive_chunk(
                parts[1],
                parts[2]
            )

        except Exception as e:

            print(
                "OTA DATA error:",
                e
            )


        return


    if command == "OTA_END":

        ota.finish()

        return


    if command == "OTA_CANCEL":

        ota.cancel()

        return


    if command == "OTA_STATUS":

        ota.send_status()

        return


    if ota.active:

        return


    # ========================================================
    # AUTOSTART
    # ========================================================

    if command == "AUTOSTART":

        start_autonomous()

        return


    # ========================================================
    # REMOTE MOVE
    # ========================================================

    if command.startswith(
        "MOVE,"
    ):

        try:

            parts = (
                command.split(",")
            )


            if len(parts) == 3:

                set_motors(
                    float(parts[1]),
                    float(parts[2])
                )

        except Exception as e:

            print(
                "MOVE error:",
                e
            )


        return


    # ========================================================
    # MODE
    # ========================================================

    if command.startswith(
        "MODE,"
    ):

        try:

            new_mode = int(
                command.split(",")[1]
            )


            if new_mode not in (
                MODE_REMOTE,
                MODE_TRACKING,
                MODE_AUTO
            ):

                return


            absolute_stop()


            current_mode = (
                new_mode
            )


            # Mode change itself releases the stop latch.
            app_stop_latched = False


            auto_running = False
            auto_state = AUTO_IDLE


            save_current_settings()


            send_mode()
            send_config()


            print(
                "MODE:",
                current_mode
            )

        except Exception as e:

            print(
                "MODE error:",
                e
            )


        return


    # ========================================================
    # SPEED
    # ========================================================

    if command.startswith(
        "SPEED,"
    ):

        try:

            max_speed = int(
                clamp(
                    int(
                        command.split(",")[1]
                    ),
                    0,
                    100
                )
            )


            mark_settings_dirty()

            send_config()

        except Exception as e:

            print(
                "SPEED error:",
                e
            )


        return


    # ========================================================
    # PID
    # ========================================================

    if command.startswith(
        "PID,"
    ):

        try:

            parts = (
                command.split(",")
            )


            if len(parts) == 4:

                pid_p = float(
                    parts[1]
                )

                pid_i = float(
                    parts[2]
                )

                pid_d = float(
                    parts[3]
                )


                save_current_settings()

                send_config()

        except Exception as e:

            print(
                "PID error:",
                e
            )


        return


    # ========================================================
    # LED
    # ========================================================

    if command.startswith(
        "LED,"
    ):

        try:

            parts = (
                command.split(",")
            )


            if len(parts) != 7:

                return


            led_r = int(
                clamp(
                    int(parts[1]),
                    0,
                    255
                )
            )

            led_g = int(
                clamp(
                    int(parts[2]),
                    0,
                    255
                )
            )

            led_b = int(
                clamp(
                    int(parts[3]),
                    0,
                    255
                )
            )

            led_brightness = int(
                clamp(
                    int(parts[4]),
                    0,
                    100
                )
            )

            led_animation = int(
                clamp(
                    int(parts[5]),
                    0,
                    6
                )
            )

            led_speed = int(
                clamp(
                    int(parts[6]),
                    50,
                    5000
                )
            )


            reset_led_animation()

            save_current_settings()

            send_led_state()

        except Exception as e:

            print(
                "LED error:",
                e
            )


        return


    if command.startswith(
        "LEDCOLOR,"
    ):

        try:

            parts = command.split(",")


            if len(parts) != 4:

                return


            led_r = int(
                clamp(
                    int(parts[1]),
                    0,
                    255
                )
            )

            led_g = int(
                clamp(
                    int(parts[2]),
                    0,
                    255
                )
            )

            led_b = int(
                clamp(
                    int(parts[3]),
                    0,
                    255
                )
            )


            reset_led_animation()

            save_current_settings()

            send_led_state()

        except Exception as e:

            print(
                "LEDCOLOR error:",
                e
            )


        return


    if command.startswith(
        "LEDBRIGHT,"
    ):

        try:

            led_brightness = int(
                clamp(
                    int(
                        command.split(",")[1]
                    ),
                    0,
                    100
                )
            )


            save_current_settings()

            send_led_state()

        except Exception as e:

            print(
                "LEDBRIGHT error:",
                e
            )


        return


    if command.startswith(
        "LEDANIM,"
    ):

        try:

            led_animation = int(
                clamp(
                    int(
                        command.split(",")[1]
                    ),
                    0,
                    6
                )
            )


            reset_led_animation()

            save_current_settings()

            send_led_state()

        except Exception as e:

            print(
                "LEDANIM error:",
                e
            )


        return


    if command.startswith(
        "LEDSPEED,"
    ):

        try:

            led_speed = int(
                clamp(
                    int(
                        command.split(",")[1]
                    ),
                    50,
                    5000
                )
            )


            reset_led_animation()

            save_current_settings()

            send_led_state()

        except Exception as e:

            print(
                "LEDSPEED error:",
                e
            )


        return


    if command == "LED?":

        send_led_state()

        return


    if command in (
        "CFG?",
        "CONFIG?"
    ):

        send_config()

        return


    if command == "SAVECFG":

        save_current_settings()

        send_config()

        return


    if command in (
        "OBJECTSTART",
        "OBJECTSTOP"
    ):

        absolute_stop()

        return


# ============================================================
# BLE ADVERTISING
# ============================================================

def advertising_payload(name):

    payload = bytearray()


    payload += bytes(
        (
            2,
            0x01,
            0x06
        )
    )


    name_bytes = (
        name.encode()
    )


    payload += bytes(
        (
            len(name_bytes)+1,
            0x09
        )
    )


    payload += (
        name_bytes
    )


    return payload


def start_advertising():

    try:

        ble.gap_advertise(
            100000,

            adv_data=advertising_payload(
                DEVICE_NAME
            )
        )

    except Exception as e:

        print(
            "Advertising error:",
            e
        )


# ============================================================
# BLE IRQ
# ============================================================

def ble_irq(
    event,
    data
):

    global ble_connected
    global rx_buffer


    if event == _IRQ_CENTRAL_CONNECT:

        conn_handle, _, _ = (
            data
        )


        connections.add(
            conn_handle
        )


        ble_connected = True


        update_oled()


        send_mode()
        send_config()
        send_led_state()
        send_sensor_telemetry()
        send_battery()
        send_mpu()


    elif (
        event ==
        _IRQ_CENTRAL_DISCONNECT
    ):

        conn_handle, _, _ = (
            data
        )


        connections.discard(
            conn_handle
        )


        ble_connected = (
            len(connections) >
            0
        )


        absolute_stop()


        update_oled()

        start_advertising()


    elif (
        event ==
        _IRQ_GATTS_WRITE
    ):

        _, attr_handle = (
            data
        )


        if (
            attr_handle !=
            rx_handle
        ):

            return


        try:

            incoming = (
                ble.gatts_read(
                    rx_handle
                ).decode(
                    "utf-8"
                )
            )


            rx_buffer += (
                incoming
            )


            rx_buffer = (
                rx_buffer
                .replace(
                    "\r\n",
                    "\n"
                )
                .replace(
                    "\r",
                    "\n"
                )
            )


            while "\n" in rx_buffer:

                line, rx_buffer = (
                    rx_buffer.split(
                        "\n",
                        1
                    )
                )


                line = (
                    line.strip()
                )


                if line:

                    process_command(
                        line
                    )

        except Exception as e:

            print(
                "BLE RX error:",
                e
            )


ble.irq(
    ble_irq
)


# ============================================================
# STARTUP
# ============================================================

print(
    "BluBot Firmware",
    FIRMWARE_VERSION
)


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


# ============================================================
# TIMERS
# ============================================================

now = (
    time.ticks_ms()
)


sharp_last = now

tof_last = now

mpu_control_last = now

telemetry_last = now

mpu_telemetry_last = now

battery_last = now

oled_last = now


# ============================================================
# MAIN LOOP
# ============================================================

while True:

    now = (
        time.ticks_ms()
    )


    # ========================================================
    # ACTIVE BRAKE
    # ========================================================

    update_emergency_brake()


    # ========================================================
    # TOF / CLIFF
    #
    # Highest-frequency critical physical safety sensor.
    # ========================================================

    if time.ticks_diff(
        now,
        tof_last
    ) >= TOF_INTERVAL_MS:

        tof_last = now


        read_tof()

        update_cliff_safety()


    # ========================================================
    # SHARP / FRONT OBJECT
    # ========================================================

    if time.ticks_diff(
        now,
        sharp_last
    ) >= SHARP_INTERVAL_MS:

        sharp_last = now


        read_sharp()

        update_front_safety()


    # ========================================================
    # MPU
    # ========================================================

    if time.ticks_diff(
        now,
        mpu_control_last
    ) >= MPU_CONTROL_INTERVAL_MS:

        mpu_control_last = now


        read_mpu()


    # ========================================================
    # AUTONOMOUS NAVIGATION
    # ========================================================

    update_autonomous()


    # ========================================================
    # MOTOR CONTROL
    # ========================================================

    update_drive_control()


    # ========================================================
    # LED
    # ========================================================

    update_led_animation()


    # ========================================================
    # BATTERY
    # ========================================================

    if time.ticks_diff(
        now,
        battery_last
    ) >= BATTERY_INTERVAL_MS:

        battery_last = now


        read_battery()


        if (
            ble_connected
            and
            not ota.active
        ):

            send_battery()


    # ========================================================
    # SENSOR TELEMETRY
    # ========================================================

    if time.ticks_diff(
        now,
        telemetry_last
    ) >= FAST_TELEMETRY_INTERVAL_MS:

        telemetry_last = now


        if (
            ble_connected
            and
            not ota.active
        ):

            send_sensor_telemetry()


    # ========================================================
    # MPU TELEMETRY
    # ========================================================

    if time.ticks_diff(
        now,
        mpu_telemetry_last
    ) >= MPU_TELEMETRY_INTERVAL_MS:

        mpu_telemetry_last = now


        if (
            ble_connected
            and
            not ota.active
        ):

            send_mpu()


    # ========================================================
    # OLED
    # ========================================================

    if time.ticks_diff(
        now,
        oled_last
    ) >= OLED_INTERVAL_MS:

        oled_last = now


        update_oled()


    # ========================================================
    # SETTINGS SAVE
    # ========================================================

    if settings_dirty:

        if time.ticks_diff(
            now,
            settings_dirty_since
        ) >= SETTINGS_SAVE_DELAY_MS:

            save_current_settings()


    time.sleep_ms(
        2
    )
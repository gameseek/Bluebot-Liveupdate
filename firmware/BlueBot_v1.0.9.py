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
# FIRMWARE
# ============================================================

FIRMWARE_VERSION = "1.0.9"
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
# SENSOR / TELEMETRY TIMING
# ============================================================

# Front collision sensor
SHARP_INTERVAL_MS = 10

# Down-facing cliff sensor
TOF_INTERVAL_MS = 30

# App telemetry
FAST_TELEMETRY_INTERVAL_MS = 50

# MPU telemetry to phone.
# Local MPU control runs much faster than this.
MPU_TELEMETRY_INTERVAL_MS = 100

BATTERY_INTERVAL_MS = 1000

OLED_INTERVAL_MS = 100


# ============================================================
# COMMAND WATCHDOG
# ============================================================

DRIVE_COMMAND_TIMEOUT_MS = 500


# ============================================================
# SETTINGS SAVE
# ============================================================

# Prevent continuous flash writes while speed slider moves.
SETTINGS_SAVE_DELAY_MS = 1500


# ============================================================
# FRONT COLLISION SAFETY
# ============================================================

# Even at very low speed, never intentionally get closer
# than approximately this.
FRONT_HARD_STOP_CM = 15.0

# At maximum driving speed begin emergency stopping earlier.
FRONT_MAX_BRAKE_CM = 30.0

# Minimum release point.
FRONT_RELEASE_MIN_CM = 20.0

# Additional clearance beyond the distance at which
# the emergency stop occurred.
FRONT_RELEASE_MARGIN_CM = 3.0

# Require two consecutive dangerous readings.
# At 10 ms sampling this introduces only ~10 ms confirmation.
FRONT_CONFIRM_COUNT = 2


# ============================================================
# CLIFF SAFETY
# ============================================================

CLIFF_THRESHOLD_MM = 120

CLIFF_BRAKE_MS = 120

# Approximate 10 cm retreat.
# No encoder exists, so this remains time based.
CLIFF_BACKOFF_MS = 500

CLIFF_BACKOFF_POWER = 0.30

CLIFF_HOLD_MS = 250


SAFETY_NORMAL = 0
SAFETY_CLIFF_BRAKE = 1
SAFETY_CLIFF_BACKOFF = 2
SAFETY_CLIFF_HOLD = 3


# ============================================================
# SAFETY LED
# ============================================================

SAFETY_FLASH_PERIOD_MS = 300


# ============================================================
# BATTERY
# ============================================================

ADC_REFERENCE_V = 3.30

BATTERY_ADC_FACTOR = 4.3333

BATTERY_FULL_V = 8.40
BATTERY_EMPTY_V = 6.40


# ============================================================
# MPU DRIVE CONTROLLER
# ============================================================

# Smoother normal acceleration
DRIVE_RAMP_PER_SEC = 3.0

GYRO_TURN_MAX_DPS = 160.0

GYRO_STEER_KP = 0.22

MAX_GYRO_CORRECTION = 0.22

GYRO_Z_SIGN = 1.0

STRAIGHT_DEADZONE = 0.08

TILT_STOP_DEG = 45.0


# ============================================================
# SETTINGS
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
# HELPERS
# ============================================================

def clamp(value, minimum, maximum):

    if value < minimum:
        return minimum

    if value > maximum:
        return maximum

    return value


# ============================================================
# SETTINGS FILE
# ============================================================

def write_settings(data):

    try:

        with open(
            SETTINGS_FILE,
            "w"
        ) as file:

            json.dump(
                data,
                file
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

    old_version = str(
        data.get(
            "settings_version",
            "1.0.1"
        )
    )


    for key in DEFAULT_SETTINGS:

        if key == "settings_version":
            continue

        if key not in data:

            data[key] = DEFAULT_SETTINGS[key]
            changed = True


    # Migration only from the older LED settings format.

    if old_version == "1.0.1":

        data["led_r"] = 255
        data["led_g"] = 255
        data["led_b"] = 255

        data["led_brightness"] = 100

        data["led_animation"] = LED_STROBE

        data["led_speed"] = 1000

        changed = True


    data["settings_version"] = SETTINGS_VERSION


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
        ) as file:

            data = json.load(
                file
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

            "mode": int(current_mode),

            "speed": int(max_speed),

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

    settings_dirty_since = time.ticks_ms()


# ============================================================
# DRIVE STATE
# ============================================================

requested_left = 0.0
requested_right = 0.0

actual_left = 0.0
actual_right = 0.0

drive_left_output = 0.0
drive_right_output = 0.0

drive_last_time = time.ticks_ms()

last_move_command_time = time.ticks_ms()


# ============================================================
# FRONT SAFETY STATE
# ============================================================

front_obstacle = False

front_danger_count = 0

front_trigger_distance = (
    FRONT_HARD_STOP_CM
)


# ============================================================
# CLIFF SAFETY STATE
# ============================================================

safety_state = SAFETY_NORMAL

safety_state_start = time.ticks_ms()


# ============================================================
# SAFETY LED STATE
# ============================================================

safety_flash_start = time.ticks_ms()


# ============================================================
# BLE
# ============================================================

ble_connected = False

connections = set()

rx_buffer = ""


# ============================================================
# SHARP
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

floor_cliff = 1

tof_valid = False


# ============================================================
# BATTERY
# ============================================================

battery_voltage = 0.0

battery_percent = 0

battery_filtered_voltage = 0.0

battery_filter_ready = False


# ============================================================
# MPU
# ============================================================

mpu_available = False

mpu_pitch = 0.0
mpu_roll = 0.0
mpu_yaw = 0.0

gyro_z_dps = 0.0
gyro_z_bias = 0.0

mpu_last_time = time.ticks_ms()


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
# NEOPIXELS
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

    i2c_devices = i2c.scan()

except:

    i2c_devices = []


print(
    "I2C:",
    [
        hex(device)
        for device
        in i2c_devices
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

        time.sleep_ms(100)

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

    good_samples = 0


    for _ in range(100):

        try:

            raw = i2c.readfrom_mem(
                MPU_ADDR,
                0x43,
                6
            )


            gx_raw, gy_raw, gz_raw = struct.unpack(
                ">hhh",
                raw
            )


            total += (
                gz_raw /
                131.0
            )


            good_samples += 1


        except:
            pass


        time.sleep_ms(5)


    if good_samples > 0:

        gyro_z_bias = (
            total /
            good_samples
        )


    print(
        "Gyro Z bias:",
        gyro_z_bias
    )


# ============================================================
# LOW LEVEL MOTOR
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
# EMERGENCY STOP
# ============================================================

def emergency_motor_stop():

    global actual_left
    global actual_right

    global drive_left_output
    global drive_right_output


    actual_left = 0.0
    actual_right = 0.0

    drive_left_output = 0.0
    drive_right_output = 0.0


    # No ramp.
    # Cut both PWM outputs immediately.

    pwma.duty_u16(0)
    pwmb.duty_u16(0)


    ain1.value(0)
    ain2.value(0)

    bin1.value(0)
    bin2.value(0)


# ============================================================
# NORMAL STOP
# ============================================================

def stop_motors():

    global requested_left
    global requested_right


    requested_left = 0.0
    requested_right = 0.0


    emergency_motor_stop()


# ============================================================
# SET MOTOR TARGET
# ============================================================

def set_motors(
    left,
    right
):

    global requested_left
    global requested_right

    global last_move_command_time


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


    if current_mode != MODE_REMOTE:

        stop_motors()

        return


    requested_left = left

    requested_right = right


    last_move_command_time = (
        time.ticks_ms()
    )


# ============================================================
# FRONT DYNAMIC STOP DISTANCE
# ============================================================

def get_front_stop_distance():

    # actual_left/right are already scaled by max_speed.
    # Therefore they represent approximate current commanded
    # physical motor output.

    current_forward = (
        actual_left +
        actual_right
    ) / 2.0


    current_forward = clamp(
        current_forward,
        0.0,
        1.0
    )


    stop_distance = (
        FRONT_HARD_STOP_CM
        +
        (
            FRONT_MAX_BRAKE_CM -
            FRONT_HARD_STOP_CM
        )
        *
        current_forward
    )


    return clamp(
        stop_distance,
        FRONT_HARD_STOP_CM,
        FRONT_MAX_BRAKE_CM
    )


# ============================================================
# RED SAFETY LED IMMEDIATE
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
# FRONT COLLISION SAFETY
# ============================================================

def update_front_safety():

    global front_obstacle

    global front_danger_count

    global front_trigger_distance

    global requested_left
    global requested_right


    if sharp_instant_cm < 0:

        return


    stop_distance = (
        get_front_stop_distance()
    )


    # --------------------------------------------------------
    # NOT CURRENTLY BLOCKED
    # --------------------------------------------------------

    if not front_obstacle:

        if (
            sharp_instant_cm <=
            stop_distance
        ):

            front_danger_count += 1

        else:

            front_danger_count = 0


        if (
            front_danger_count >=
            FRONT_CONFIRM_COUNT
        ):

            front_obstacle = True

            front_danger_count = 0


            front_trigger_distance = (
                stop_distance
            )


            # Clear old forward request.

            requested_left = 0.0
            requested_right = 0.0


            # CRITICAL:
            # Do not use normal motor ramp.
            # Stop physically now.

            emergency_motor_stop()


            # Red immediately, not after next animation tick.

            safety_red_now()


            print(
                "SAFETY WALL STOP",
                round(
                    sharp_instant_cm,
                    1
                ),
                "cm threshold",
                round(
                    stop_distance,
                    1
                )
            )


    # --------------------------------------------------------
    # CURRENTLY BLOCKED
    # --------------------------------------------------------

    else:

        release_distance = max(
            FRONT_RELEASE_MIN_CM,

            front_trigger_distance +
            FRONT_RELEASE_MARGIN_CM
        )


        if (
            sharp_instant_cm >=
            release_distance
        ):

            front_obstacle = False

            front_danger_count = 0


            print(
                "SAFETY WALL CLEAR",
                round(
                    sharp_instant_cm,
                    1
                ),
                "cm"
            )


            reset_led_animation()


# ============================================================
# CLIFF
# ============================================================

def cliff_detected():

    if not tof_available:

        return False


    return (
        floor_cliff == 1
    )


def start_cliff_safety():

    global safety_state

    global safety_state_start

    global requested_left
    global requested_right


    if (
        safety_state !=
        SAFETY_NORMAL
    ):

        return


    print(
        "SAFETY CLIFF DETECTED"
    )


    requested_left = 0.0
    requested_right = 0.0


    # Immediate physical motor cut.

    emergency_motor_stop()


    # Immediate red LEDs.

    safety_red_now()


    safety_state = (
        SAFETY_CLIFF_BRAKE
    )


    safety_state_start = (
        time.ticks_ms()
    )


def update_cliff_safety():

    global safety_state

    global safety_state_start

    global requested_left
    global requested_right


    now = time.ticks_ms()


    cliff = cliff_detected()


    # --------------------------------------------------------
    # NEW CLIFF
    # --------------------------------------------------------

    if (
        safety_state ==
        SAFETY_NORMAL
    ):

        if cliff:

            start_cliff_safety()

        return


    # --------------------------------------------------------
    # BRAKE
    # --------------------------------------------------------

    if (
        safety_state ==
        SAFETY_CLIFF_BRAKE
    ):

        if time.ticks_diff(
            now,
            safety_state_start
        ) >= CLIFF_BRAKE_MS:

            safety_state = (
                SAFETY_CLIFF_BACKOFF
            )


            safety_state_start = now


            print(
                "SAFETY CLIFF BACKOFF"
            )


        return


    # --------------------------------------------------------
    # BACKOFF
    # --------------------------------------------------------

    if (
        safety_state ==
        SAFETY_CLIFF_BACKOFF
    ):

        if time.ticks_diff(
            now,
            safety_state_start
        ) >= CLIFF_BACKOFF_MS:

            emergency_motor_stop()


            safety_state = (
                SAFETY_CLIFF_HOLD
            )


            safety_state_start = now


            print(
                "SAFETY CLIFF BACKOFF COMPLETE"
            )


        return


    # --------------------------------------------------------
    # HOLD
    # --------------------------------------------------------

    if (
        safety_state ==
        SAFETY_CLIFF_HOLD
    ):

        emergency_motor_stop()


        if time.ticks_diff(
            now,
            safety_state_start
        ) >= CLIFF_HOLD_MS:

            # Do not re-enable movement while
            # floor sensor still says cliff.

            if not cliff:

                safety_state = (
                    SAFETY_NORMAL
                )


                requested_left = 0.0
                requested_right = 0.0


                print(
                    "SAFETY FLOOR SAFE"
                )


                reset_led_animation()


# ============================================================
# MPU ASSISTED DRIVE
# ============================================================

def update_drive_control():

    global actual_left
    global actual_right

    global drive_left_output
    global drive_right_output

    global drive_last_time

    global requested_left
    global requested_right


    now = time.ticks_ms()


    dt = (
        time.ticks_diff(
            now,
            drive_last_time
        )
        /
        1000.0
    )


    drive_last_time = now


    if dt <= 0:

        return


    if dt > 0.1:

        dt = 0.1


    # ========================================================
    # MODE
    # ========================================================

    if current_mode != MODE_REMOTE:

        emergency_motor_stop()

        return


    # ========================================================
    # TILT SAFETY
    # ========================================================

    if (
        mpu_available
        and
        (
            abs(mpu_pitch) >
            TILT_STOP_DEG

            or

            abs(mpu_roll) >
            TILT_STOP_DEG
        )
    ):

        requested_left = 0.0
        requested_right = 0.0


        emergency_motor_stop()

        return


    # ========================================================
    # CLIFF SAFETY OVERRIDE
    # ========================================================

    if (
        safety_state ==
        SAFETY_CLIFF_BRAKE
    ):

        # Keep motors physically off during
        # the brake interval.

        emergency_motor_stop()

        return


    elif (
        safety_state ==
        SAFETY_CLIFF_BACKOFF
    ):

        target_input_left = (
            -CLIFF_BACKOFF_POWER
        )

        target_input_right = (
            -CLIFF_BACKOFF_POWER
        )


    elif (
        safety_state ==
        SAFETY_CLIFF_HOLD
    ):

        emergency_motor_stop()

        return


    else:

        # ====================================================
        # BLE COMMAND WATCHDOG
        # ====================================================

        if (
            abs(requested_left) >
            0.03

            or

            abs(requested_right) >
            0.03
        ):

            if time.ticks_diff(
                now,
                last_move_command_time
            ) > DRIVE_COMMAND_TIMEOUT_MS:

                requested_left = 0.0
                requested_right = 0.0


        target_input_left = (
            requested_left
        )

        target_input_right = (
            requested_right
        )


        # ====================================================
        # FRONT OBJECT BLOCKING
        # ====================================================

        if front_obstacle:

            throttle_requested = (
                target_input_left +
                target_input_right
            ) / 2.0


            # Positive means forward.
            # Forward is completely prohibited.
            #
            # Reverse remains available.
            # Spin/turn commands remain available.

            if throttle_requested > 0.02:

                emergency_motor_stop()

                return


    # ========================================================
    # LEFT/RIGHT -> THROTTLE/TURN
    # ========================================================

    throttle = (
        target_input_left +
        target_input_right
    ) / 2.0


    turn = (
        target_input_left -
        target_input_right
    ) / 2.0


    corrected_turn = turn


    # ========================================================
    # MPU YAW RATE CONTROL
    # ========================================================

    if mpu_available:

        if (
            safety_state ==
            SAFETY_CLIFF_BACKOFF
        ):

            speed_scale = (
                CLIFF_BACKOFF_POWER
            )

        else:

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
                turn
                *
                GYRO_TURN_MAX_DPS
                *
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


    # ========================================================
    # REBUILD LEFT/RIGHT
    # ========================================================

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


    # ========================================================
    # SPEED SCALE
    # ========================================================

    if (
        safety_state ==
        SAFETY_CLIFF_BACKOFF
    ):

        # The automatic reverse power was
        # already specified directly.

        speed_factor = 1.0

    else:

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
    # SMOOTH NORMAL DRIVE
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
    # PHYSICAL MOTOR MAPPING
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
# MOVEMENT NAME
# ============================================================

def movement_name():

    if (
        safety_state ==
        SAFETY_CLIFF_BRAKE
    ):

        return "CLIFF STOP"


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

        if abs(
            left-right
        ) < 0.10:

            return "FORWARD"


        if left > right:

            return "RIGHT"


        return "LEFT"


    if (
        left < -0.03
        and
        right < -0.03
    ):

        if abs(
            abs(left) -
            abs(right)
        ) < 0.10:

            return "REVERSE"


        return "REV TURN"


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


    if left > 0.03:

        return "LEFT FWD"


    if right > 0.03:

        return "RIGHT FWD"


    if left < -0.03:

        return "LEFT REV"


    if right < -0.03:

        return "RIGHT REV"


    return "MOVE"


# ============================================================
# LED HELPERS
# ============================================================

def scale_led_color(
    r,
    g,
    b,
    brightness=None
):

    if brightness is None:

        brightness = led_brightness


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

        np[i] = color


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

    if front_obstacle:

        return True


    if (
        safety_state !=
        SAFETY_NORMAL
    ):

        return True


    return False


def update_safety_led():

    now = time.ticks_ms()


    phase = (
        time.ticks_diff(
            now,
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

    now = time.ticks_ms()


    # Safety overrides every normal LED animation.

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


            r, g, b = color_wheel(
                position +
                offset
            )


            np[i] = scale_led_color(
                r,
                g,
                b
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


        flash_on = (
            phase < 0.07
            or
            0.14 <= phase < 0.21
            or
            0.28 <= phase < 0.35
        )


        if flash_on:

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

        if NEOPIXEL_COUNT <= 1:

            set_all_leds(
                led_r,
                led_g,
                led_b
            )

            return


        position = int(
            phase *
            NEOPIXEL_COUNT
        )


        if position >= NEOPIXEL_COUNT:

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

    global battery_filtered_voltage
    global battery_filter_ready


    try:

        total = 0

        samples = 32


        for _ in range(samples):

            total += (
                battery_adc.read_u16()
            )


        raw = (
            total /
            samples
        )


        adc_voltage = (
            raw /
            65535.0
        ) * ADC_REFERENCE_V


        measured_voltage = (
            adc_voltage *
            BATTERY_ADC_FACTOR
        )


        if not battery_filter_ready:

            battery_filtered_voltage = (
                measured_voltage
            )

            battery_filter_ready = True

        else:

            battery_filtered_voltage = (
                battery_filtered_voltage *
                0.85

                +

                measured_voltage *
                0.15
            )


        battery_voltage = (
            battery_filtered_voltage
        )


        percent = (
            (
                battery_voltage -
                BATTERY_EMPTY_V
            )
            /
            (
                BATTERY_FULL_V -
                BATTERY_EMPTY_V
            )
        ) * 100.0


        battery_percent = int(
            clamp(
                percent,
                0,
                100
            )
        )


    except Exception as e:

        print(
            "Battery error:",
            e
        )


# ============================================================
# SHARP CONVERSION
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


# ============================================================
# SHARP READ
# ============================================================

def read_sharp():

    global sharp_raw
    global sharp_voltage

    global sharp_distance_cm
    global sharp_instant_cm

    global sharp_filtered_cm
    global sharp_filter_ready


    try:

        total = 0


        # Few samples because this sensor is now checked
        # every 10 ms.

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


        new_distance = (
            sharp_voltage_to_distance(
                sharp_voltage
            )
        )


        # Immediate value for safety.

        sharp_instant_cm = (
            new_distance
        )


        # Filtered value for UI/telemetry.

        if not sharp_filter_ready:

            sharp_filtered_cm = (
                new_distance
            )

            sharp_filter_ready = True

        else:

            sharp_filtered_cm = (
                sharp_filtered_cm *
                0.75

                +

                new_distance *
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

            value = tof.range


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

    global floor_cliff

    global tof_valid


    if not tof_available:

        tof_distance_mm = -1

        floor_cliff = 1

        tof_valid = False

        return


    value = get_tof_reading()


    if (
        value <= 0
        or
        value >= 8190
    ):

        tof_distance_mm = -1

        tof_valid = False


        # Fail safe.
        # No valid floor measurement means unsafe.

        floor_cliff = 1

        return


    tof_valid = True

    tof_distance_mm = value


    if (
        value <=
        CLIFF_THRESHOLD_MM
    ):

        floor_cliff = 0

    else:

        floor_cliff = 1


# ============================================================
# MPU
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

        now = time.ticks_ms()


        dt = (
            time.ticks_diff(
                now,
                mpu_last_time
            )
            /
            1000.0
        )


        mpu_last_time = now


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
# MODE
# ============================================================

def mode_name():

    if (
        current_mode ==
        MODE_REMOTE
    ):

        return "REMOTE"


    if (
        current_mode ==
        MODE_TRACKING
    ):

        return "TRACK"


    if (
        current_mode ==
        MODE_AUTO
    ):

        return "AUTO"


    return "?"


# ============================================================
# BLE DEFINITIONS
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

ble.active(True)


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


    data = message.encode()


    for connection in list(
        connections
    ):

        for position in range(
            0,
            len(data),
            20
        ):

            try:

                ble.gatts_notify(
                    connection,
                    tx_handle,

                    data[
                        position:
                        position+20
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

    if mpu_available:

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


        oled.text(
            "TOF: FLOOR"
            if (
                tof_available
                and
                floor_cliff == 0
            )
            else
            "TOF: CLIFF",
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
# MOVEMENT TEXT
# ============================================================

def movement_name():

    if (
        safety_state ==
        SAFETY_CLIFF_BRAKE
    ):

        return "CLIFF STOP"


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

        if abs(
            left-right
        ) < 0.10:

            return "FORWARD"


        if left > right:

            return "RIGHT"


        return "LEFT"


    if (
        left < -0.03
        and
        right < -0.03
    ):

        if abs(
            abs(left) -
            abs(right)
        ) < 0.10:

            return "REVERSE"


        return "REV TURN"


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


    if left > 0.03:

        return "LEFT FWD"


    if right > 0.03:

        return "RIGHT FWD"


    if left < -0.03:

        return "LEFT REV"


    if right < -0.03:

        return "RIGHT REV"


    return "MOVE"


# ============================================================
# COMMAND PROCESSING
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


    command = command.strip()


    if not command:

        return


    print(
        "APP:",
        command
    )


    # ========================================================
    # FIRMWARE VERSION
    # ========================================================

    if command == "FW?":

        ble_send(
            "FW,{}\n".format(
                FIRMWARE_VERSION
            )
        )

        return


    # ========================================================
    # OTA BEGIN
    # ========================================================

    if command.startswith(
        "OTA_BEGIN,"
    ):

        try:

            parts = command.split(",")


            if len(parts) != 4:

                ble_send(
                    "OTA_ERROR,BAD_BEGIN\n"
                )

                return


            stop_motors()


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


    # ========================================================
    # OTA DATA
    # ========================================================

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
    # MOVE
    # ========================================================

    if command.startswith(
        "MOVE,"
    ):

        try:

            parts = command.split(",")


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
    # STOP
    # ========================================================

    if command == "STOP":

        stop_motors()

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


            stop_motors()


            current_mode = (
                new_mode
            )


            save_current_settings()


            send_mode()

            send_config()


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


            # Do not continuously write to flash.

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

            parts = command.split(",")


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
    # LED COMPLETE
    # ========================================================

    if command.startswith(
        "LED,"
    ):

        try:

            parts = command.split(",")


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


    # ========================================================
    # LED COLOR
    # ========================================================

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


    # ========================================================
    # LED BRIGHTNESS
    # ========================================================

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


    # ========================================================
    # LED ANIMATION
    # ========================================================

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


    # ========================================================
    # LED SPEED
    # ========================================================

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
        "OBJECTSTOP",
        "AUTOSTART",
        "AUTOSTOP"
    ):

        stop_motors()

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


    payload += name_bytes


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

        conn_handle, _, _ = data


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

        conn_handle, _, _ = data


        connections.discard(
            conn_handle
        )


        ble_connected = (
            len(connections) >
            0
        )


        stop_motors()


        update_oled()


        start_advertising()


    elif (
        event ==
        _IRQ_GATTS_WRITE
    ):

        _, attr_handle = data


        if (
            attr_handle !=
            rx_handle
        ):

            return


        try:

            incoming = ble.gatts_read(
                rx_handle
            ).decode(
                "utf-8"
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


            while (
                "\n" in rx_buffer
            ):

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


stop_motors()


reset_led_animation()


read_battery()


read_sharp()

update_front_safety()


read_tof()

update_cliff_safety()


read_mpu()


calibrate_gyro()


read_mpu()


update_led_animation()


update_oled()


start_advertising()


# ============================================================
# TIMERS
# ============================================================

now = time.ticks_ms()


sharp_last = now

tof_last = now

telemetry_last = now

mpu_telemetry_last = now

battery_last = now

oled_last = now


# ============================================================
# MAIN LOOP
# ============================================================

while True:

    now = time.ticks_ms()


    # ========================================================
    # MPU LOCAL FEEDBACK
    # ========================================================

    read_mpu()


    # ========================================================
    # FRONT COLLISION SENSOR
    # ========================================================

    if time.ticks_diff(
        now,
        sharp_last
    ) >= SHARP_INTERVAL_MS:

        sharp_last = now


        read_sharp()


        update_front_safety()


    # ========================================================
    # CLIFF SENSOR
    # ========================================================

    if time.ticks_diff(
        now,
        tof_last
    ) >= TOF_INTERVAL_MS:

        tof_last = now


        read_tof()


        update_cliff_safety()


    # ========================================================
    # DRIVE CONTROL
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
    # FAST SENSOR TELEMETRY
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
    # DELAYED SETTINGS SAVE
    # ========================================================

    if settings_dirty:

        if time.ticks_diff(
            now,
            settings_dirty_since
        ) >= SETTINGS_SAVE_DELAY_MS:

            save_current_settings()


    time.sleep_ms(2)
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


# ================================================================
# FIRMWARE
# ================================================================

FIRMWARE_VERSION = "1.0.1"


# ================================================================
# IMPORTS
# ================================================================

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


# ================================================================
# GENERAL CONFIG
# ================================================================

DEVICE_NAME = "BluBot"

SETTINGS_FILE = "settings.json"

OLED_WIDTH = 128
OLED_HEIGHT = 64

MOTOR_PWM_FREQ = 20000

SENSOR_INTERVAL_MS = 100
OLED_INTERVAL_MS = 200

# Whole robot state sent every quarter second
TELEMETRY_INTERVAL_MS = 250


# ================================================================
# MODES
# ================================================================

MODE_REMOTE = 1
MODE_TRACKING = 2
MODE_AUTO = 3


# ================================================================
# LED MODES
#
# EXACT APP MAPPING
#
# 0 = OFF
# 1 = SOLID
# 2 = PULSE
# 3 = RAINBOW
# 4 = STROBE
# 5 = CHASE
# 6 = BREATHING
# ================================================================

LED_OFF = 0
LED_SOLID = 1
LED_PULSE = 2
LED_RAINBOW = 3
LED_STROBE = 4
LED_CHASE = 5
LED_BREATHING = 6


# ================================================================
# DEFAULT SETTINGS
#
# Created on first boot only.
# After that settings.json is loaded.
# ================================================================

DEFAULT_SETTINGS = {

    "mode": MODE_REMOTE,

    "speed": 75,

    "p": 0.0,
    "i": 0.0,
    "d": 0.0,

    "led_r": 255,
    "led_g": 255,
    "led_b": 255,

    "led_brightness": 100,

    # Default = STROBE
    "led_animation": LED_STROBE,

    "led_speed": 500
}


# ================================================================
# BATTERY
#
# PCB divider:
#
# Battery+
#    |
#   100K
#    |
# GPIO0
#    |
#   33K
#    |
#   GND
#
# You measured approximately:
#
# Battery = 7.8V
# ADC node = 1.8V
#
# Initial calibration = 1.075
# ================================================================

BATTERY_R1 = 100000.0
BATTERY_R2 = 33000.0

BATTERY_DIVIDER_RATIO = (
    BATTERY_R1 + BATTERY_R2
) / BATTERY_R2

BATTERY_CALIBRATION = 1.075

ADC_REFERENCE_V = 3.30


# ================================================================
# BATTERY PERCENT LOOKUP
# ================================================================

BATTERY_TABLE = (

    (8.40, 100),
    (8.30, 95),
    (8.20, 90),
    (8.10, 85),
    (8.00, 80),
    (7.90, 75),
    (7.80, 70),
    (7.70, 60),
    (7.60, 50),
    (7.50, 40),
    (7.40, 30),
    (7.30, 20),
    (7.20, 15),
    (7.00, 10),
    (6.80, 5),
    (6.40, 0)
)


# ================================================================
# SENSOR SETTINGS
# ================================================================

CLIFF_THRESHOLD_MM = 120

SHARP_MIN_CM = 4.0
SHARP_MAX_CM = 50.0


# ================================================================
# MOTOR DIRECTION
#
# Change to True if the corresponding side runs backwards.
# ================================================================

LEFT_MOTOR_INVERT = False
RIGHT_MOTOR_INVERT = False


# ================================================================
# HELPERS
# ================================================================

def clamp(value, minimum, maximum):

    if value < minimum:
        return minimum

    if value > maximum:
        return maximum

    return value


# ================================================================
# SETTINGS FILE
# ================================================================

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


def load_settings():

    try:

        if SETTINGS_FILE not in os.listdir():

            print(
                "Creating default settings.json"
            )

            data = (
                DEFAULT_SETTINGS.copy()
            )

            write_settings(
                data
            )

            return data


        with open(
            SETTINGS_FILE,
            "r"
        ) as file:

            data = json.load(
                file
            )


        changed = False


        # Automatically add settings introduced
        # by a later firmware version.

        for key in DEFAULT_SETTINGS:

            if key not in data:

                data[key] = (
                    DEFAULT_SETTINGS[key]
                )

                changed = True


        if changed:

            write_settings(
                data
            )


        return data


    except Exception as e:

        print(
            "Settings load error:",
            e
        )


        data = (
            DEFAULT_SETTINGS.copy()
        )

        write_settings(
            data
        )

        return data


settings = load_settings()


# ================================================================
# CURRENT SETTINGS
# ================================================================

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


# ================================================================
# SAVE CURRENT SETTINGS
# ================================================================

def save_current_settings():

    data = {

        "mode": int(
            current_mode
        ),

        "speed": int(
            max_speed
        ),

        "p": float(
            pid_p
        ),

        "i": float(
            pid_i
        ),

        "d": float(
            pid_d
        ),

        "led_r": int(
            led_r
        ),

        "led_g": int(
            led_g
        ),

        "led_b": int(
            led_b
        ),

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


    write_settings(
        data
    )


# ================================================================
# RUNTIME STATE
# ================================================================

requested_left = 0.0
requested_right = 0.0

actual_left = 0.0
actual_right = 0.0


ble_connected = False

connections = set()

rx_buffer = ""


# ================================================================
# SENSOR STATE
# ================================================================

sharp_raw = 0
sharp_voltage = 0.0
sharp_distance_cm = 0.0


tof_available = False
tof_distance_mm = -1

floor_cliff = 0


battery_raw = 0
battery_adc_voltage = 0.0

battery_voltage = 0.0
battery_percent = 0


mpu_available = False

mpu_pitch = 0.0
mpu_roll = 0.0
mpu_yaw = 0.0

mpu_last_time = (
    time.ticks_ms()
)


# ================================================================
# MOTOR INITIALIZATION
# ================================================================

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


pwma.freq(
    MOTOR_PWM_FREQ
)

pwmb.freq(
    MOTOR_PWM_FREQ
)


pwma.duty_u16(0)
pwmb.duty_u16(0)


# ================================================================
# NEOPIXEL
# ================================================================

np = neopixel.NeoPixel(

    Pin(
        NEOPIXEL_PIN,
        Pin.OUT
    ),

    NEOPIXEL_COUNT
)


# ================================================================
# ADC
# ================================================================

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


# ================================================================
# I2C
# ================================================================

i2c = SoftI2C(

    sda=Pin(
        SDA_PIN
    ),

    scl=Pin(
        SCL_PIN
    ),

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
        for x
        in i2c_devices
    ]
)


# ================================================================
# OLED
# ================================================================

oled = None


try:

    oled = ssd1306.SSD1306_I2C(

        OLED_WIDTH,
        OLED_HEIGHT,
        i2c
    )


    oled.fill(0)

    oled.text(
        "BLE: WAITING",
        0,
        0
    )

    oled.text(
        "BLUBOT STARTING",
        0,
        16
    )

    oled.show()


    print(
        "OLED ready"
    )


except Exception as e:

    print(
        "OLED not found:",
        e
    )

    oled = None


# ================================================================
# VL53L0X
# ================================================================

tof = None


if VL53_AVAILABLE:

    try:

        tof = VL53L0X(
            i2c
        )

        tof_available = True

        print(
            "VL53L0X ready"
        )


    except Exception as e:

        print(
            "VL53L0X not found:",
            e
        )


# ================================================================
# MPU6050
# ================================================================

MPU_ADDR = 0x68


if MPU_ADDR in i2c_devices:

    try:

        # Wake MPU6050

        i2c.writeto_mem(

            MPU_ADDR,
            0x6B,
            b"\x00"
        )


        time.sleep_ms(100)


        mpu_available = True


        print(
            "MPU6050 ready"
        )


    except Exception as e:

        print(
            "MPU error:",
            e
        )


# ================================================================
# MOTOR FUNCTIONS
# ================================================================

def drive_motor(
    in1,
    in2,
    pwm,
    value,
    invert=False
):

    value = clamp(
        value,
        -1.0,
        1.0
    )


    # App-defined dead zone

    if abs(value) < 0.03:

        value = 0.0


    if invert:

        value = -value


    if value == 0:

        in1.value(0)

        in2.value(0)

        pwm.duty_u16(0)

        return


    duty = int(
        abs(value)
        *
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


def stop_motors():

    global actual_left
    global actual_right


    actual_left = 0.0
    actual_right = 0.0


    drive_motor(

        ain1,
        ain2,
        pwma,
        0
    )


    drive_motor(

        bin1,
        bin2,
        pwmb,
        0
    )


def set_motors(
    left,
    right
):

    global requested_left
    global requested_right

    global actual_left
    global actual_right


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
    # ONLY REMOTE MODE DRIVES MOTORS FOR NOW
    # ============================================================

    if current_mode != MODE_REMOTE:

        stop_motors()

        return


    speed_multiplier = (
        max_speed /
        100.0
    )


    actual_left = (
        left *
        speed_multiplier
    )

    actual_right = (
        right *
        speed_multiplier
    )


    drive_motor(

        ain1,
        ain2,
        pwma,
        actual_left,
        LEFT_MOTOR_INVERT
    )


    drive_motor(

        bin1,
        bin2,
        pwmb,
        actual_right,
        RIGHT_MOTOR_INVERT
    )


# ================================================================
# MOVEMENT DESCRIPTION
# ================================================================

def movement_name():

    left = actual_left
    right = actual_right


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
            left -
            right
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
            abs(left)
            -
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


# ================================================================
# LED HELPERS
# ================================================================

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


    brightness = clamp(
        brightness,
        0,
        100
    )


    factor = (
        brightness /
        100.0
    )


    return (

        int(
            r *
            factor
        ),

        int(
            g *
            factor
        ),

        int(
            b *
            factor
        )
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


def color_wheel(
    position
):

    position %= 256


    if position < 85:

        return (

            255 -
            position * 3,

            position * 3,

            0
        )


    if position < 170:

        position -= 85


        return (

            0,

            255 -
            position * 3,

            position * 3
        )


    position -= 170


    return (

        position * 3,

        0,

        255 -
        position * 3
    )


def led_animation_name():

    names = (

        "OFF",
        "SOLID",
        "PULSE",
        "RAINBOW",
        "STROBE",
        "CHASE",
        "BREATHE"
    )


    if (
        0 <=
        led_animation <=
        6
    ):

        return names[
            led_animation
        ]


    return "UNKNOWN"


# ================================================================
# LED ANIMATION
# ================================================================

animation_start = (
    time.ticks_ms()
)


def reset_led_animation():

    global animation_start


    animation_start = (
        time.ticks_ms()
    )


def update_led_animation():

    now = (
        time.ticks_ms()
    )


    # ============================================================
    # OFF
    # ============================================================

    if led_animation == LED_OFF:

        leds_off()

        return


    # ============================================================
    # SOLID
    # ============================================================

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


    elapsed = (
        time.ticks_diff(
            now,
            animation_start
        )
        %
        period
    )


    phase = (
        elapsed /
        period
    )


    # ============================================================
    # PULSE
    # ============================================================

    if led_animation == LED_PULSE:

        # Heartbeat style

        if phase < 0.10:

            level = (
                phase /
                0.10
            )


        elif phase < 0.18:

            level = (

                1.0
                -
                (
                    (
                        phase -
                        0.10
                    )
                    /
                    0.08
                )
                *
                0.65
            )


        elif phase < 0.26:

            level = (

                0.35
                +
                (
                    (
                        phase -
                        0.18
                    )
                    /
                    0.08
                )
                *
                0.65
            )


        elif phase < 0.42:

            level = (

                1.0
                -
                (
                    (
                        phase -
                        0.26
                    )
                    /
                    0.16
                )
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


    # ============================================================
    # RAINBOW
    # ============================================================

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


            np[i] = scale_led_color(
                r,
                g,
                b
            )


        np.write()

        return


    # ============================================================
    # STROBE
    # ============================================================

    if led_animation == LED_STROBE:

        # Short flash followed by longer OFF period

        if phase < 0.25:

            set_all_leds(

                led_r,
                led_g,
                led_b
            )

        else:

            leds_off()


        return


    # ============================================================
    # CHASE
    # ============================================================

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


    # ============================================================
    # BREATHING
    # ============================================================

    if led_animation == LED_BREATHING:

        level = (

            math.sin(

                (
                    phase *
                    2 *
                    math.pi
                )
                -
                (
                    math.pi /
                    2
                )
            )

            +
            1.0

        ) / 2.0


        set_all_leds(

            led_r,
            led_g,
            led_b,

            int(
                led_brightness *
                level
            )
        )


# ================================================================
# BATTERY PERCENT
# ================================================================

def voltage_to_percent(
    voltage
):

    if (
        voltage >=
        BATTERY_TABLE[0][0]
    ):

        return 100


    if (
        voltage <=
        BATTERY_TABLE[-1][0]
    ):

        return 0


    for index in range(
        len(BATTERY_TABLE) - 1
    ):

        high_v, high_p = (
            BATTERY_TABLE[index]
        )

        low_v, low_p = (
            BATTERY_TABLE[
                index + 1
            ]
        )


        if (
            voltage <= high_v
            and
            voltage >= low_v
        ):

            fraction = (

                (
                    voltage -
                    low_v
                )

                /

                (
                    high_v -
                    low_v
                )
            )


            result = (

                low_p
                +
                fraction
                *
                (
                    high_p -
                    low_p
                )
            )


            return int(

                clamp(
                    result,
                    0,
                    100
                )
            )


    return 0


# ================================================================
# BATTERY READING
# ================================================================

def read_battery():

    global battery_raw
    global battery_adc_voltage

    global battery_voltage
    global battery_percent


    try:

        total = 0

        samples = 16


        for _ in range(
            samples
        ):

            total += (
                battery_adc.read_u16()
            )


        battery_raw = int(

            total /
            samples
        )


        battery_adc_voltage = (

            battery_raw /
            65535.0

        ) * ADC_REFERENCE_V


        battery_voltage = (

            battery_adc_voltage
            *
            BATTERY_DIVIDER_RATIO
            *
            BATTERY_CALIBRATION
        )


        battery_voltage = clamp(

            battery_voltage,

            0.0,
            9.5
        )


        battery_percent = (
            voltage_to_percent(
                battery_voltage
            )
        )


    except Exception as e:

        print(
            "Battery error:",
            e
        )


# ================================================================
# SHARP GP2Y0E03
#
# This conversion is intentionally isolated because your Sharp
# sensor needs physical calibration.
# ================================================================

def sharp_voltage_to_distance(
    voltage
):

    if voltage <= 0.05:

        return SHARP_MAX_CM


    # Temporary approximation.
    #
    # Replace this function after collecting your
    # actual Sharp ADC readings.

    distance = (
        13.0 /
        voltage
    )


    return clamp(

        distance,

        SHARP_MIN_CM,
        SHARP_MAX_CM
    )


def read_sharp():

    global sharp_raw
    global sharp_voltage
    global sharp_distance_cm


    try:

        total = 0

        samples = 8


        for _ in range(
            samples
        ):

            total += (
                sharp_adc.read_u16()
            )


        sharp_raw = int(

            total /
            samples
        )


        sharp_voltage = (

            sharp_raw /
            65535.0

        ) * ADC_REFERENCE_V


        sharp_distance_cm = (
            sharp_voltage_to_distance(
                sharp_voltage
            )
        )


    except Exception as e:

        print(
            "Sharp error:",
            e
        )

        sharp_distance_cm = -1


# ================================================================
# VL53L0X
# ================================================================

def get_tof_reading():

    if tof is None:

        return -1


    # Support common methods used by MicroPython VL53L0X drivers.

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
    global floor_cliff


    if not tof_available:

        tof_distance_mm = -1
        floor_cliff = 0

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
        floor_cliff = 0

        return


    tof_distance_mm = (
        value
    )


    if (
        tof_distance_mm >
        CLIFF_THRESHOLD_MM
    ):

        floor_cliff = 1

    else:

        floor_cliff = 0


# ================================================================
# MPU6050
# ================================================================

def read_mpu():

    global mpu_pitch
    global mpu_roll
    global mpu_yaw

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


        mpu_last_time = now


        if (
            dt <= 0
            or
            dt > 1
        ):

            dt = (
                SENSOR_INTERVAL_MS /
                1000.0
            )


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


        gz = (
            gz_raw /
            131.0
        )


        mpu_roll = (

            math.atan2(
                ay,
                az
            )

            *
            57.2957795
        )


        denominator = math.sqrt(

            ay * ay
            +
            az * az
        )


        mpu_pitch = (

            math.atan2(
                -ax,
                denominator
            )

            *
            57.2957795
        )


        # Relative gyro yaw.
        # MPU6050 has no magnetometer, so yaw will drift.

        mpu_yaw += (
            gz *
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


# ================================================================
# UPDATE ALL SENSORS
# ================================================================

def update_sensors():

    read_battery()

    read_sharp()

    read_tof()

    read_mpu()


# ================================================================
# MODE NAME
# ================================================================

def mode_name():

    if current_mode == MODE_REMOTE:

        return "REMOTE"


    if current_mode == MODE_TRACKING:

        return "TRACK"


    if current_mode == MODE_AUTO:

        return "AUTO"


    return "?"


# ================================================================
# BLE
# ================================================================

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

            bluetooth.FLAG_WRITE
            |
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


# ================================================================
# BLE SEND
# ================================================================

def ble_send(
    message
):

    if not connections:

        return


    data = (
        message.encode()
    )


    # Send in safe BLE pieces.
    #
    # The phone app joins received chunks until newline.

    chunk_size = 20


    for connection in list(
        connections
    ):

        for position in range(

            0,
            len(data),
            chunk_size
        ):

            try:

                ble.gatts_notify(

                    connection,
                    tx_handle,

                    data[
                        position:
                        position +
                        chunk_size
                    ]
                )


            except Exception as e:

                print(
                    "BLE notify error:",
                    e
                )


# ================================================================
# OTA OBJECT
#
# Must be created after:
#
# ble_send()
# stop_motors()
# ================================================================

ota = OTAUpdater(

    ble_send,
    stop_motors
)


# ================================================================
# TELEMETRY
# ================================================================

def send_primary_telemetry():

    # ------------------------------------------------------------
    # S,<distance>,<floor>,<battery>,<mode>,<speed>
    # ------------------------------------------------------------

    ble_send(

        "S,{:.1f},{},{},{},{}\n"
        .format(

            sharp_distance_cm,

            floor_cliff,

            battery_percent,

            current_mode,

            max_speed
        )
    )


def send_led_state():

    ble_send(

        "LED,{},{},{},{},{},{}\n"
        .format(

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

        "CFG,{:.5f},{:.5f},{:.5f},{},{}\n"
        .format(

            pid_p,
            pid_i,
            pid_d,

            max_speed,

            current_mode
        )
    )


def send_mode():

    ble_send(

        "MODE,{}\n"
        .format(
            current_mode
        )
    )


def send_battery():

    ble_send(

        "BAT:{},{:.2f}\n"
        .format(

            battery_percent,
            battery_voltage
        )
    )


def send_distance():

    ble_send(

        "DIST:{:.1f}\n"
        .format(
            sharp_distance_cm
        )
    )


def send_floor():

    ble_send(

        "FLOOR:{}\n"
        .format(
            floor_cliff
        )
    )


def send_mpu():

    if not mpu_available:

        return


    ble_send(

        "MPU:{:.1f},{:.1f},{:.1f}\n"
        .format(

            mpu_pitch,
            mpu_roll,
            mpu_yaw
        )
    )


def send_all_telemetry():

    send_primary_telemetry()

    send_led_state()

    send_config()

    send_battery()

    send_distance()

    send_floor()

    send_mpu()


# ================================================================
# OLED
#
# LINE 1 ALWAYS BLE STATUS
# ================================================================

def update_oled():

    if oled is None:

        return


    try:

        # ========================================================
        # OTA SCREEN
        # ========================================================

        if ota.active:

            oled.fill(0)


            if ble_connected:

                oled.text(
                    "BLE: CONNECTED",
                    0,
                    0
                )

            else:

                oled.text(
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
                "Uploading...",
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


        # ========================================================
        # NORMAL SCREEN
        # ========================================================

        oled.fill(0)


        # BLE

        if ble_connected:

            oled.text(
                "BLE: CONNECTED",
                0,
                0
            )

        else:

            oled.text(
                "BLE: WAITING",
                0,
                0
            )


        # Mode and max speed

        oled.text(

            "{} SPD:{}%"
            .format(

                mode_name(),
                max_speed
            )[:16],

            0,
            8
        )


        # Movement

        oled.text(

            movement_name()[:16],

            0,
            16
        )


        # Motor outputs

        oled.text(

            "L:{:+3d} R:{:+3d}"
            .format(

                int(
                    actual_left *
                    100
                ),

                int(
                    actual_right *
                    100
                )
            )[:16],

            0,
            24
        )


        # Sharp

        if sharp_distance_cm >= 0:

            text = (
                "SHARP:{:.1f}cm"
                .format(
                    sharp_distance_cm
                )
            )

        else:

            text = (
                "SHARP: ERROR"
            )


        oled.text(

            text[:16],

            0,
            32
        )


        # ToF / cliff

        if tof_distance_mm >= 0:

            text = (
                "TOF:{} C:{}"
                .format(

                    tof_distance_mm,
                    floor_cliff
                )
            )

        else:

            text = (
                "TOF: NOT FOUND"
            )


        oled.text(

            text[:16],

            0,
            40
        )


        # Battery

        oled.text(

            "BAT:{:.2f}V {}%"
            .format(

                battery_voltage,
                battery_percent
            )[:16],

            0,
            48
        )


        # LED animation

        oled.text(

            "{} {}%"
            .format(

                led_animation_name(),
                led_brightness
            )[:16],

            0,
            56
        )


        oled.show()


    except Exception as e:

        print(
            "OLED error:",
            e
        )


# ================================================================
# COMMAND PROCESSOR
# ================================================================

def process_command(
    command
):

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


    command = (
        command.strip()
    )


    if not command:

        return


    print(
        "APP:",
        command
    )


    # ============================================================
    # FIRMWARE VERSION
    # ============================================================

    if command == "FW?":

        ble_send(

            "FW,{}\n"
            .format(
                FIRMWARE_VERSION
            )
        )

        return


    # ============================================================
    # OTA BEGIN
    #
    # OTA_BEGIN,<version>,<size>,<sha256>
    # ============================================================

    if command.startswith(
        "OTA_BEGIN,"
    ):

        try:

            parts = command.split(
                ","
            )


            if len(parts) != 4:

                ble_send(
                    "OTA_ERROR,BAD_BEGIN\n"
                )

                return


            ota.begin(

                parts[1],
                parts[2],
                parts[3]
            )


        except Exception as e:

            print(
                "OTA begin error:",
                e
            )


            ble_send(
                "OTA_ERROR,BAD_BEGIN\n"
            )


        return


    # ============================================================
    # OTA DATA
    #
    # OTA_DATA,<sequence>,<base64>
    # ============================================================

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
                "OTA data error:",
                e
            )


        return


    # ============================================================
    # OTA END
    # ============================================================

    if command == "OTA_END":

        ota.finish()

        return


    # ============================================================
    # OTA CANCEL
    # ============================================================

    if command == "OTA_CANCEL":

        ota.cancel()

        return


    # ============================================================
    # OTA STATUS
    # ============================================================

    if command == "OTA_STATUS":

        ota.send_status()

        return


    # ============================================================
    # BLOCK NORMAL COMMANDS DURING OTA
    # ============================================================

    if ota.active:

        return


    # ============================================================
    # MOVE
    #
    # MOVE,<left>,<right>
    # ============================================================

    if command.startswith(
        "MOVE,"
    ):

        try:

            parts = command.split(
                ","
            )


            if len(parts) != 3:

                return


            left = float(
                parts[1]
            )

            right = float(
                parts[2]
            )


            if current_mode == MODE_REMOTE:

                set_motors(
                    left,
                    right
                )

            else:

                stop_motors()


        except Exception as e:

            print(
                "MOVE error:",
                e
            )


        return


    # ============================================================
    # MODE
    # ============================================================

    if command.startswith(
        "MODE,"
    ):

        try:

            new_mode = int(

                command.split(
                    ","
                )[1]
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


    # ============================================================
    # SPEED
    # ============================================================

    if command.startswith(
        "SPEED,"
    ):

        try:

            max_speed = int(

                clamp(

                    int(
                        command.split(
                            ","
                        )[1]
                    ),

                    0,
                    100
                )
            )


            # Reapply joystick position with new limit.

            if current_mode == MODE_REMOTE:

                set_motors(

                    requested_left,
                    requested_right
                )


            save_current_settings()

            send_config()


        except Exception as e:

            print(
                "SPEED error:",
                e
            )


        return


    # ============================================================
    # PID
    #
    # PID,p,i,d
    # ============================================================

    if command.startswith(
        "PID,"
    ):

        try:

            parts = command.split(
                ","
            )


            if len(parts) != 4:

                return


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


    # ============================================================
    # FULL LED
    #
    # LED,r,g,b,brightness,animation,speed
    # ============================================================

    if command.startswith(
        "LED,"
    ):

        try:

            parts = command.split(
                ","
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


    # ============================================================
    # LED COLOR
    # ============================================================

    if command.startswith(
        "LEDCOLOR,"
    ):

        try:

            parts = command.split(
                ","
            )


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


    # ============================================================
    # LED BRIGHTNESS
    # ============================================================

    if command.startswith(
        "LEDBRIGHT,"
    ):

        try:

            led_brightness = int(

                clamp(

                    int(
                        command.split(
                            ","
                        )[1]
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


    # ============================================================
    # LED ANIMATION
    # ============================================================

    if command.startswith(
        "LEDANIM,"
    ):

        try:

            led_animation = int(

                clamp(

                    int(
                        command.split(
                            ","
                        )[1]
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


    # ============================================================
    # LED SPEED
    # ============================================================

    if command.startswith(
        "LEDSPEED,"
    ):

        try:

            led_speed = int(

                clamp(

                    int(
                        command.split(
                            ","
                        )[1]
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


    # ============================================================
    # LED SYNC
    # ============================================================

    if command == "LED?":

        send_led_state()

        return


    # ============================================================
    # CONFIG SYNC
    # ============================================================

    if command in (

        "CFG?",
        "CONFIG?"
    ):

        send_config()

        return


    # ============================================================
    # SAVE CONFIG
    # ============================================================

    if command == "SAVECFG":

        save_current_settings()

        send_config()

        return


    # ============================================================
    # TRACKING
    #
    # Logic intentionally not implemented yet.
    # ============================================================

    if command == "OBJECTSTART":

        stop_motors()

        return


    if command == "OBJECTSTOP":

        stop_motors()

        return


    # ============================================================
    # AUTO
    #
    # Logic intentionally not implemented yet.
    # ============================================================

    if command == "AUTOSTART":

        stop_motors()

        return


    if command == "AUTOSTOP":

        stop_motors()

        return


    print(
        "UNKNOWN:",
        command
    )


# ================================================================
# BLE IRQ
# ================================================================

def ble_irq(
    event,
    data
):

    global ble_connected
    global rx_buffer


    # ============================================================
    # CONNECT
    # ============================================================

    if event == _IRQ_CENTRAL_CONNECT:

        conn_handle, addr_type, addr = (
            data
        )


        connections.add(
            conn_handle
        )


        ble_connected = True


        print(
            "BLE CONNECTED"
        )


        update_oled()


        # Immediate sync with app

        send_mode()

        send_all_telemetry()


    # ============================================================
    # DISCONNECT
    # ============================================================

    elif event == _IRQ_CENTRAL_DISCONNECT:

        conn_handle, addr_type, addr = (
            data
        )


        if conn_handle in connections:

            connections.remove(
                conn_handle
            )


        ble_connected = (
            len(connections) > 0
        )


        print(
            "BLE DISCONNECTED"
        )


        # Always stop robot if phone/controller disappears.

        stop_motors()


        # If OTA connection dropped before installation,
        # keep OTA temp data in ota.py state.
        #
        # App can reconnect and use OTA_STATUS / OTA_CANCEL.


        update_oled()

        start_advertising()


    # ============================================================
    # RECEIVE
    # ============================================================

    elif event == _IRQ_GATTS_WRITE:

        conn_handle, attr_handle = (
            data
        )


        if attr_handle != rx_handle:

            return


        try:

            raw = ble.gatts_read(
                rx_handle
            )


            incoming = raw.decode(
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


            # Commands may arrive across several BLE writes.
            #
            # Do nothing until newline is received.

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


# ================================================================
# BLE ADVERTISING
# ================================================================

def advertising_payload(
    name
):

    payload = bytearray()


    # Standard flags

    payload += bytes(
        (
            2,
            0x01,
            0x06
        )
    )


    # Device name

    name_bytes = (
        name.encode()
    )


    payload += bytes(
        (
            len(name_bytes) + 1,
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

            adv_data=(
                advertising_payload(
                    DEVICE_NAME
                )
            )
        )


        print(
            "BLE WAITING..."
        )


    except Exception as e:

        print(
            "Advertising error:",
            e
        )


# ================================================================
# STARTUP
# ================================================================

print()
print(
    "================================"
)

print(
    "            BLUBOT"
)

print(
    "================================"
)


print(
    "Firmware:",
    FIRMWARE_VERSION
)

print(
    "Mode:",
    mode_name()
)

print(
    "Speed:",
    max_speed,
    "%"
)

print(
    "PID:",
    pid_p,
    pid_i,
    pid_d
)

print(
    "LED:",
    led_animation_name()
)


# Safety

stop_motors()


# Apply stored LED configuration

reset_led_animation()

update_led_animation()


# Initial sensor readings

update_sensors()


# Initial screen shows BLE waiting

update_oled()


# Start Bluetooth advertising

start_advertising()


# ================================================================
# MAIN LOOP TIMERS
# ================================================================

now = (
    time.ticks_ms()
)

sensor_last = now
oled_last = now
telemetry_last = now


# ================================================================
# MAIN LOOP
# ================================================================

while True:

    now = (
        time.ticks_ms()
    )


    # ============================================================
    # LED
    # ============================================================

    update_led_animation()


    # ============================================================
    # SENSORS
    #
    # Read every 100ms.
    # ============================================================

    if time.ticks_diff(
        now,
        sensor_last
    ) >= SENSOR_INTERVAL_MS:

        sensor_last = now


        # Sensors can still be read while OTA is active,
        # but telemetry isn't transmitted.

        update_sensors()


    # ============================================================
    # OLED
    # ============================================================

    if time.ticks_diff(
        now,
        oled_last
    ) >= OLED_INTERVAL_MS:

        oled_last = now

        update_oled()


    # ============================================================
    # TELEMETRY
    #
    # Every 250ms.
    #
    # Suppressed while firmware is being uploaded.
    # ============================================================

    if time.ticks_diff(
        now,
        telemetry_last
    ) >= TELEMETRY_INTERVAL_MS:

        telemetry_last = now


        if (
            ble_connected
            and
            not ota.active
        ):

            send_all_telemetry()


    # ============================================================
    # CPU YIELD
    # ============================================================

    time.sleep_ms(10)

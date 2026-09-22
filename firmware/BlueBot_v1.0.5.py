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

FIRMWARE_VERSION = "1.0.5"
SETTINGS_VERSION = "1.0.2"

from machine import Pin, PWM, ADC, SoftI2C
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

MODE_REMOTE, MODE_TRACKING, MODE_AUTO = 1, 2, 3
LED_OFF, LED_SOLID, LED_PULSE, LED_RAINBOW, LED_STROBE, LED_CHASE, LED_BREATHING = range(7)

SENSOR_INTERVAL_MS = 100
OLED_INTERVAL_MS = 200
TELEMETRY_INTERVAL_MS = 250
CLIFF_THRESHOLD_MM = 120
ADC_REFERENCE_V = 3.30
BATTERY_DIVIDER_RATIO = 133000 / 33000
BATTERY_CALIBRATION = 1.075

DEFAULT_SETTINGS = {
    "settings_version": SETTINGS_VERSION,
    "mode": 1, "speed": 75,
    "p": 0.0, "i": 0.0, "d": 0.0,
    "led_r": 255, "led_g": 255, "led_b": 255,
    "led_brightness": 100, "led_animation": 4, "led_speed": 1000
}

BATTERY_TABLE = (
    (8.40,100),(8.30,95),(8.20,90),(8.10,85),(8.00,80),
    (7.90,75),(7.80,70),(7.70,60),(7.60,50),(7.50,40),
    (7.40,30),(7.30,20),(7.20,15),(7.00,10),(6.80,5),(6.40,0)
)

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def write_settings(d):
    try:
        with open(SETTINGS_FILE, "w") as f: json.dump(d, f)
        return True
    except Exception as e:
        print("Settings error:", e)
        return False

def load_settings():
    try:
        if SETTINGS_FILE not in os.listdir():
            write_settings(DEFAULT_SETTINGS.copy())
            return DEFAULT_SETTINGS.copy()

        with open(SETTINGS_FILE) as f: d = json.load(f)

        old_version = str(d.get("settings_version", "1.0.1"))

        for k, v in DEFAULT_SETTINGS.items():
            if k != "settings_version" and k not in d: d[k] = v

        if old_version != SETTINGS_VERSION:
            d.update({
                "led_r":255, "led_g":255, "led_b":255,
                "led_brightness":100, "led_animation":4,
                "led_speed":1000
            })

        d["settings_version"] = SETTINGS_VERSION
        write_settings(d)
        return d

    except Exception as e:
        print("Settings load:", e)
        write_settings(DEFAULT_SETTINGS.copy())
        return DEFAULT_SETTINGS.copy()

settings = load_settings()

current_mode = int(settings["mode"])
max_speed = int(settings["speed"])
pid_p, pid_i, pid_d = float(settings["p"]), float(settings["i"]), float(settings["d"])
led_r, led_g, led_b = int(settings["led_r"]), int(settings["led_g"]), int(settings["led_b"])
led_brightness = int(settings["led_brightness"])
led_animation = int(settings["led_animation"])
led_speed = int(settings["led_speed"])

def save_settings():
    write_settings({
        "settings_version": SETTINGS_VERSION,
        "mode":current_mode, "speed":max_speed,
        "p":pid_p, "i":pid_i, "d":pid_d,
        "led_r":led_r, "led_g":led_g, "led_b":led_b,
        "led_brightness":led_brightness,
        "led_animation":led_animation, "led_speed":led_speed
    })

requested_left = requested_right = 0.0
actual_left = actual_right = 0.0
ble_connected = False
connections = set()
rx_buffer = ""

sharp_raw = 0
sharp_voltage = 0.0
sharp_distance_cm = -1.0
sharp_filtered_cm = 0.0
sharp_filter_ready = False

tof_available = False
tof_distance_mm = -1
floor_cliff = 1

battery_voltage = 0.0
battery_percent = 0

mpu_available = False
mpu_pitch = mpu_roll = mpu_yaw = 0.0
mpu_last_time = time.ticks_ms()

ain1, ain2 = Pin(AIN1_PIN, Pin.OUT), Pin(AIN2_PIN, Pin.OUT)
bin1, bin2 = Pin(BIN1_PIN, Pin.OUT), Pin(BIN2_PIN, Pin.OUT)
pwma, pwmb = PWM(Pin(PWMA_PIN)), PWM(Pin(PWMB_PIN))
pwma.freq(20000)
pwmb.freq(20000)
pwma.duty_u16(0)
pwmb.duty_u16(0)

np = neopixel.NeoPixel(Pin(NEOPIXEL_PIN, Pin.OUT), NEOPIXEL_COUNT)
sharp_adc, battery_adc = ADC(Pin(SHARP_PIN)), ADC(Pin(BATTERY_PIN))

try: sharp_adc.atten(ADC.ATTN_11DB)
except: pass
try: battery_adc.atten(ADC.ATTN_11DB)
except: pass

i2c = SoftI2C(sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=400000)
try: i2c_devices = i2c.scan()
except: i2c_devices = []

print("I2C:", [hex(x) for x in i2c_devices])

try:
    oled = ssd1306.SSD1306_I2C(128, 64, i2c)
except Exception as e:
    print("OLED:", e)
    oled = None

tof = None
if VL53_AVAILABLE:
    try:
        tof = VL53L0X(i2c)
        tof_available = True
    except Exception as e:
        print("TOF:", e)

MPU_ADDR = 0x68
if MPU_ADDR in i2c_devices:
    try:
        i2c.writeto_mem(MPU_ADDR, 0x6B, b"\x00")
        time.sleep_ms(100)
        mpu_available = True
    except Exception as e:
        print("MPU:", e)

def drive_motor(a, b, pwm, v):
    v = clamp(v, -1, 1)
    if abs(v) < .03: v = 0
    if v == 0:
        a.value(0); b.value(0); pwm.duty_u16(0)
    elif v > 0:
        a.value(1); b.value(0); pwm.duty_u16(int(v * 65535))
    else:
        a.value(0); b.value(1); pwm.duty_u16(int(-v * 65535))

def stop_motors():
    global actual_left, actual_right
    actual_left = actual_right = 0
    drive_motor(ain1, ain2, pwma, 0)
    drive_motor(bin1, bin2, pwmb, 0)

def set_motors(left, right):
    global requested_left, requested_right, actual_left, actual_right

    left, right = clamp(left,-1,1), clamp(right,-1,1)
    if abs(left) < .03: left = 0
    if abs(right) < .03: right = 0

    requested_left, requested_right = left, right

    if current_mode != MODE_REMOTE:
        stop_motors()
        return

    f = max_speed / 100
    actual_left, actual_right = left * f, right * f

    drive_motor(ain1, ain2, pwma, actual_right)
    drive_motor(bin1, bin2, pwmb, -actual_left)

def movement_name():
    l, r = actual_left, actual_right
    if abs(l) < .03 and abs(r) < .03: return "STOP"
    if l > .03 and r > .03: return "FORWARD" if abs(l-r)<.1 else ("RIGHT" if l>r else "LEFT")
    if l < -.03 and r < -.03: return "REVERSE"
    if l > .03 and r < -.03: return "RIGHT"
    if l < -.03 and r > .03: return "LEFT"
    return "MOVE"

def set_led(r,g,b,brightness=None):
    br = led_brightness if brightness is None else brightness
    f = clamp(br,0,100)/100
    c = (int(r*f),int(g*f),int(b*f))
    for i in range(NEOPIXEL_COUNT): np[i] = c
    np.write()

def leds_off():
    for i in range(NEOPIXEL_COUNT): np[i] = (0,0,0)
    np.write()

def wheel(p):
    p %= 256
    if p < 85: return 255-p*3,p*3,0
    if p < 170:
        p -= 85
        return 0,255-p*3,p*3
    p -= 170
    return p*3,0,255-p*3

animation_start = time.ticks_ms()

def reset_led():
    global animation_start
    animation_start = time.ticks_ms()

def update_led():
    now = time.ticks_ms()

    if led_animation == LED_OFF:
        leds_off(); return

    if led_animation == LED_SOLID:
        set_led(led_r,led_g,led_b); return

    period = max(50,led_speed)
    phase = (time.ticks_diff(now,animation_start) % period) / period

    if led_animation == LED_PULSE:
        level = (math.sin(phase*2*math.pi-math.pi/2)+1)/2
        set_led(led_r,led_g,led_b,int(led_brightness*level))

    elif led_animation == LED_RAINBOW:
        for i in range(NEOPIXEL_COUNT):
            r,g,b = wheel(int(phase*255)+int(i*256/NEOPIXEL_COUNT))
            f = led_brightness/100
            np[i]=(int(r*f),int(g*f),int(b*f))
        np.write()

    elif led_animation == LED_STROBE:
        flash = phase<.07 or .14<=phase<.21 or .28<=phase<.35
        set_led(255,255,255) if flash else leds_off()

    elif led_animation == LED_CHASE:
        pos = min(NEOPIXEL_COUNT-1,int(phase*NEOPIXEL_COUNT))
        for i in range(NEOPIXEL_COUNT): np[i]=(0,0,0)
        f=led_brightness/100
        np[pos]=(int(led_r*f),int(led_g*f),int(led_b*f))
        np.write()

    elif led_animation == LED_BREATHING:
        level=(math.sin(phase*2*math.pi-math.pi/2)+1)/2
        set_led(led_r,led_g,led_b,int(led_brightness*level))

def voltage_to_percent(v):
    if v >= BATTERY_TABLE[0][0]: return 100
    if v <= BATTERY_TABLE[-1][0]: return 0

    for i in range(len(BATTERY_TABLE)-1):
        hv,hp = BATTERY_TABLE[i]
        lv,lp = BATTERY_TABLE[i+1]
        if lv <= v <= hv:
            return int(lp + (v-lv)/(hv-lv)*(hp-lp))
    return 0

def read_battery():
    global battery_voltage, battery_percent
    try:
        raw = sum(battery_adc.read_u16() for _ in range(16))/16
        adc_v = raw/65535*ADC_REFERENCE_V
        battery_voltage = adc_v*BATTERY_DIVIDER_RATIO*BATTERY_CALIBRATION
        battery_percent = voltage_to_percent(battery_voltage)
    except Exception as e:
        print("Battery:",e)

def sharp_voltage_to_distance(voltage):
    if voltage <= .05: return 50.0

    raw = 13.0 / voltage

    if raw >= 40:
        return 50.0

    x = raw - 3.36

    if x <= .05:
        return 4.0

    distance = 0.56 + 15.99 * math.log(x)
    return clamp(distance,4.0,50.0)

def read_sharp():
    global sharp_raw, sharp_voltage, sharp_distance_cm
    global sharp_filtered_cm, sharp_filter_ready

    try:
        sharp_raw = int(sum(sharp_adc.read_u16() for _ in range(8))/8)
        sharp_voltage = sharp_raw/65535*ADC_REFERENCE_V
        new_distance = sharp_voltage_to_distance(sharp_voltage)

        if not sharp_filter_ready:
            sharp_filtered_cm = new_distance
            sharp_filter_ready = True
        else:
            sharp_filtered_cm = sharp_filtered_cm*.75 + new_distance*.25

        sharp_distance_cm = sharp_filtered_cm

    except Exception as e:
        print("Sharp:",e)
        sharp_distance_cm = -1

def get_tof():
    if tof is None: return -1

    try:
        if hasattr(tof,"read"): return int(tof.read())
    except: pass
    try:
        if hasattr(tof,"ping"): return int(tof.ping())
    except: pass
    try:
        if hasattr(tof,"range"):
            v=tof.range
            return int(v() if callable(v) else v)
    except: pass
    return -1

def read_tof():
    global tof_distance_mm, floor_cliff

    if not tof_available:
        tof_distance_mm, floor_cliff = -1,1
        return

    v=get_tof()

    if v<=0 or v>=8190:
        tof_distance_mm,floor_cliff=-1,1
    else:
        tof_distance_mm=v
        floor_cliff=0 if v<=CLIFF_THRESHOLD_MM else 1

def read_mpu():
    global mpu_pitch,mpu_roll,mpu_yaw,mpu_last_time

    if not mpu_available: return

    try:
        now=time.ticks_ms()
        dt=time.ticks_diff(now,mpu_last_time)/1000
        mpu_last_time=now
        if dt<=0 or dt>1: dt=.1

        raw=i2c.readfrom_mem(MPU_ADDR,0x3B,14)
        ax,ay,az,t,gx,gy,gz=struct.unpack(">hhhhhhh",raw)
        ax,ay,az=ax/16384,ay/16384,az/16384
        mpu_roll=math.atan2(ay,az)*57.2957795
        mpu_pitch=math.atan2(-ax,math.sqrt(ay*ay+az*az))*57.2957795
        mpu_yaw += (gz/131)*dt

        if mpu_yaw>180: mpu_yaw-=360
        elif mpu_yaw<-180: mpu_yaw+=360

    except Exception as e:
        print("MPU:",e)

def update_sensors():
    read_battery()
    read_sharp()
    read_tof()
    read_mpu()

def mode_name():
    return "REMOTE" if current_mode==1 else "TRACK" if current_mode==2 else "AUTO"

_IRQ_CENTRAL_CONNECT=1
_IRQ_CENTRAL_DISCONNECT=2
_IRQ_GATTS_WRITE=3

SERVICE_UUID=bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
TX_UUID=bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
RX_UUID=bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")

ble=bluetooth.BLE()
ble.active(True)

((tx_handle,rx_handle),)=ble.gatts_register_services((
    (SERVICE_UUID,(
        (TX_UUID,bluetooth.FLAG_NOTIFY),
        (RX_UUID,bluetooth.FLAG_WRITE|bluetooth.FLAG_WRITE_NO_RESPONSE)
    )),
))

ble.gatts_set_buffer(rx_handle,1024,True)

def ble_send(msg):
    data=msg.encode()
    for conn in list(connections):
        for i in range(0,len(data),20):
            try: ble.gatts_notify(conn,tx_handle,data[i:i+20])
            except: pass

ota=OTAUpdater(ble_send,stop_motors)

def send_led():
    ble_send("LED,{},{},{},{},{},{}\n".format(
        led_r,led_g,led_b,led_brightness,led_animation,led_speed))

def send_cfg():
    ble_send("CFG,{:.5f},{:.5f},{:.5f},{},{}\n".format(
        pid_p,pid_i,pid_d,max_speed,current_mode))

def telemetry():
    ble_send("S,{:.1f},{},{},{},{}\n".format(
        sharp_distance_cm,floor_cliff,battery_percent,current_mode,max_speed))
    ble_send("BAT:{},{:.2f}\n".format(battery_percent,battery_voltage))
    ble_send("DIST:{:.1f}\n".format(sharp_distance_cm))
    ble_send("FLOOR:{}\n".format(floor_cliff))
    if mpu_available:
        ble_send("MPU:{:.1f},{:.1f},{:.1f}\n".format(
            mpu_pitch,mpu_roll,mpu_yaw))

def update_oled():
    if oled is None: return

    try:
        oled.fill(0)

        if ota.active:
            oled.text("BLE: CONNECTED" if ble_connected else "BLE: WAITING",0,0)
            oled.text("FIRMWARE UPDATE",0,16)
            oled.text("UPLOADING...",0,32)
            oled.text("{}%".format(ota.progress()),0,48)
        else:
            oled.text("BLE: CONNECTED" if ble_connected else "BLE: WAITING",0,0)
            oled.text(("MODE:{} S:{}%".format(mode_name(),max_speed))[:16],0,8)
            oled.text(("MOVE:"+movement_name())[:16],0,16)
            oled.text(("SHARP:{:.1f}cm".format(sharp_distance_cm))[:16],0,24)
            oled.text("BAT:{}%".format(battery_percent),0,32)
            oled.text("MPU: CONNECTED" if mpu_available else "MPU: NOT FOUND",0,40)
            oled.text("TOF: FLOOR" if tof_available and floor_cliff==0 else "TOF: CLIFF",0,48)
            oled.text("FW:"+FIRMWARE_VERSION,0,56)

        oled.show()

    except Exception as e:
        print("OLED:",e)

def process_command(cmd):
    global current_mode,max_speed,pid_p,pid_i,pid_d
    global led_r,led_g,led_b,led_brightness,led_animation,led_speed

    cmd=cmd.strip()
    if not cmd: return
    print("APP:",cmd)

    if cmd=="FW?":
        ble_send("FW,{}\n".format(FIRMWARE_VERSION)); return

    if cmd.startswith("OTA_BEGIN,"):
        p=cmd.split(",")
        if len(p)==4: ota.begin(p[1],p[2],p[3])
        else: ble_send("OTA_ERROR,BAD_BEGIN\n")
        return

    if cmd.startswith("OTA_DATA,"):
        p=cmd.split(",",2)
        if len(p)==3: ota.receive_chunk(p[1],p[2])
        else: ble_send("OTA_ERROR,BAD_DATA\n")
        return

    if cmd=="OTA_END": ota.finish(); return
    if cmd=="OTA_CANCEL": ota.cancel(); return
    if cmd=="OTA_STATUS": ota.send_status(); return
    if ota.active: return

    try:
        if cmd.startswith("MOVE,"):
            p=cmd.split(",")
            if len(p)==3: set_motors(float(p[1]),float(p[2]))
            return

        if cmd=="STOP":
            stop_motors(); return

        if cmd.startswith("MODE,"):
            m=int(cmd.split(",")[1])
            if m in (1,2,3):
                stop_motors()
                current_mode=m
                save_settings()
                ble_send("MODE,{}\n".format(m))
                send_cfg()
            return

        if cmd.startswith("SPEED,"):
            max_speed=int(clamp(int(cmd.split(",")[1]),0,100))
            if current_mode==MODE_REMOTE:
                set_motors(requested_left,requested_right)
            save_settings(); send_cfg(); return

        if cmd.startswith("PID,"):
            p=cmd.split(",")
            pid_p,pid_i,pid_d=float(p[1]),float(p[2]),float(p[3])
            save_settings(); send_cfg(); return

        if cmd.startswith("LED,"):
            p=cmd.split(",")
            led_r=int(clamp(int(p[1]),0,255))
            led_g=int(clamp(int(p[2]),0,255))
            led_b=int(clamp(int(p[3]),0,255))
            led_brightness=int(clamp(int(p[4]),0,100))
            led_animation=int(clamp(int(p[5]),0,6))
            led_speed=int(clamp(int(p[6]),50,5000))
            reset_led(); save_settings(); send_led(); return

        if cmd.startswith("LEDCOLOR,"):
            p=cmd.split(",")
            led_r=int(clamp(int(p[1]),0,255))
            led_g=int(clamp(int(p[2]),0,255))
            led_b=int(clamp(int(p[3]),0,255))
            save_settings(); send_led(); return

        if cmd.startswith("LEDBRIGHT,"):
            led_brightness=int(clamp(int(cmd.split(",")[1]),0,100))
            save_settings(); send_led(); return

        if cmd.startswith("LEDANIM,"):
            led_animation=int(clamp(int(cmd.split(",")[1]),0,6))
            reset_led(); save_settings(); send_led(); return

        if cmd.startswith("LEDSPEED,"):
            led_speed=int(clamp(int(cmd.split(",")[1]),50,5000))
            reset_led(); save_settings(); send_led(); return

        if cmd=="LED?":
            send_led(); return

        if cmd in ("CFG?","CONFIG?"):
            send_cfg(); return

        if cmd=="SAVECFG":
            save_settings(); send_cfg(); return

        if cmd in ("OBJECTSTART","OBJECTSTOP","AUTOSTART","AUTOSTOP"):
            stop_motors(); return

    except Exception as e:
        print("Command error:",e)

def start_advertising():
    name=DEVICE_NAME.encode()
    payload=bytes((2,1,6,len(name)+1,9))+name
    ble.gap_advertise(100000,adv_data=payload)

def ble_irq(event,data):
    global ble_connected,rx_buffer

    if event==_IRQ_CENTRAL_CONNECT:
        conn,_,_=data
        connections.add(conn)
        ble_connected=True
        update_oled()
        ble_send("MODE,{}\n".format(current_mode))
        send_cfg()
        send_led()
        telemetry()

    elif event==_IRQ_CENTRAL_DISCONNECT:
        conn,_,_=data
        connections.discard(conn)
        ble_connected=bool(connections)
        stop_motors()
        update_oled()
        start_advertising()

    elif event==_IRQ_GATTS_WRITE:
        _,handle=data
        if handle!=rx_handle: return

        try:
            rx_buffer += ble.gatts_read(rx_handle).decode()
            rx_buffer=rx_buffer.replace("\r\n","\n").replace("\r","\n")

            while "\n" in rx_buffer:
                line,rx_buffer=rx_buffer.split("\n",1)
                if line.strip(): process_command(line)

        except Exception as e:
            print("BLE RX:",e)

ble.irq(ble_irq)

print("BluBot FW",FIRMWARE_VERSION)

stop_motors()
reset_led()
update_sensors()
update_led()
update_oled()
start_advertising()

sensor_last=oled_last=telemetry_last=time.ticks_ms()

while True:
    now=time.ticks_ms()
    update_led()

    if time.ticks_diff(now,sensor_last)>=SENSOR_INTERVAL_MS:
        sensor_last=now
        update_sensors()

    if time.ticks_diff(now,oled_last)>=OLED_INTERVAL_MS:
        oled_last=now
        update_oled()

    if time.ticks_diff(now,telemetry_last)>=TELEMETRY_INTERVAL_MS:
        telemetry_last=now
        if ble_connected and not ota.active:
            telemetry()

    time.sleep_ms(10)
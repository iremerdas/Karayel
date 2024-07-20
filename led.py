import time
import RPi.GPIO as GPIO
from pymavlink import mavutil

# GPIO ayarları
GPIO.setmode(GPIO.BCM)
yesil_led = 17  # Yeşil LED için GPIO pin
sari_led = 27  # Sarı LED için GPIO pin
kirmizi_led = 22  # Kırmızı LED için GPIO pin

GPIO.setup(yesil_led, GPIO.OUT)
GPIO.setup(sari_led, GPIO.OUT)
GPIO.setup(kirmizi_led, GPIO.OUT)


def ledleri_ayarla(manuel=False, otonom=False, dur=False):
    GPIO.output(yesil_led, GPIO.HIGH if manuel else GPIO.LOW)
    GPIO.output(sari_led, GPIO.HIGH if otonom else GPIO.LOW)
    GPIO.output(kirmizi_led, GPIO.HIGH if dur else GPIO.LOW)


def ledleri_kapat():
    GPIO.output(yesil_led, GPIO.LOW)
    GPIO.output(sari_led, GPIO.LOW)
    GPIO.output(kirmizi_led, GPIO.LOW)


def arac_modunu_oku():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

    while True:
        mesaj = master.recv_match(type='HEARTBEAT', blocking=True)
        if not mesaj:
            continue

        if mesaj.base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED:
            return 'manuel'
        elif mesaj.base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED:
            return 'otonom'
        elif mesaj.base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED:
            return 'dur'
        else:
            return 'bilinmeyen'

def ana():
    try:
        while True:
            mod = arac_modunu_oku()
            if mod == 'manuel':
                ledleri_ayarla(manuel=True)
            elif mod == 'otonom':
                ledleri_ayarla(otonom=True)
            elif mod == 'dur':
                ledleri_ayarla(dur=True)
            else:
                ledleri_kapat()

            time.sleep(1)

    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__ == "__main__":
    ana()

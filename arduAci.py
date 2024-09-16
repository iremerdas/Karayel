from pymavlink import mavutil
# gpsteki veriler alınınp hesaba katılmalı ama önce iha haberleşmesi yapılmalı
import arduGPS as gps
import math


def ara_aci(hedef_x, hedef_y):  # alfayı alıyor
    x, y, _ = gps.gps_verileri_oku()
    # İlk noktadan ikinci noktaya giden vektörün bileşenleri
    dx = hedef_x - x
    dy = hedef_y - y

    # Açıyı hesapla
    aci_radyan = math.atan2(dy, dx)

    # Radyanı dereceye çevir
    aci_derece = math.degrees(aci_radyan)

    # Negatif açıları pozitif yap
    if aci_derece < 0:
        aci_derece += 360

    return aci_derece


def yaw_al(arduino):  #
    while True:
        if arduino.in_waiting > 0:
            # Veriyi okuyun
            line = arduino.readline().decode('utf-8').strip()
            return line

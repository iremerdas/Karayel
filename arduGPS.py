def gps_verileri_oku(arduino):
    while True:
        if arduino.in_waiting > 0:
            gps_data = arduino.readline().decode('utf-8').strip()
            return gps_data
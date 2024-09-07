from pymavlink import mavutil
import time

def connect_vehicle(connection_string, baud, max_try):
    attempt = 0
    while attempt < max_try:
        try:
            print(f"Attempt {attempt + 1} to connect...")
            master = mavutil.mavlink_connection(connection_string, baud)
            master.wait_heartbeat()
            print("Bağlantı kuruldu.")
            return master
        except Exception as e:
            print(f"Bağlantı kurulamadı. Hata: {e}")
            attempt += 1
            time.sleep(5)  # 5 saniye bekle ve tekrar dene
    raise ConnectionError("Bağlantı kurulamadı, deneme limitine ulaşıldı.")

def arm_vehicle(master, arm, max_try):
    print(f"Attempting to {'arm' if arm == 1 else 'disarm'} the vehicle...")
    for _ in range(max_try):
        if arm == 1:
            master.arducopter_arm()
            print("Arming...")
            master.motors_armed_wait()
            print("Arm successful")
            break
        elif arm == 0:
            master.arducopter_disarm()
            print("Disarming...")
            master.motors_disarmed_wait()
            print("Disarm successful")
            break
        else:
            print("Geçersiz parametre! 1 veya 0 olmalı.")

def armanddisarm(master, i):
    print(f"Sending arm/disarm command: {'arm' if i == 1 else 'disarm'}")
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 0,
                                 i, 0, 0, 0, 0, 0, 0)
    if i == 0:
        master.motors_disarmed_wait()
        print("Disarm")
    else:
        master.motors_armed_wait()
        print("Arm")

# Bağlantı kur
master = connect_vehicle('/dev/ttyS0', baud=115200, max_try=5)

# Arming veya disarming
arm_vehicle(master, 1, 5)  # Aracı arm etmek için 1, disarm etmek için 0 gönderin.
# veya
armanddisarm(master, 1)  # Aracı arm etmek için 1, disarm etmek için 0 gönderin.

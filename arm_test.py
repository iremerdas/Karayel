from pymavlink import mavutil
import time


def connect_vehicle(connection_string, baud):
    master = mavutil.mavlink_connection(connection_string, baud)
    master.wait_heartbeat()
    print("Bağlantı kuruldu.")
    return master


def arm_vehicle(master, arm, max_try):
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
#connection_string = 'udp:127.0.0.1:14550'
# UDP üzerinden 14550 portunu kullanarak bağlantı kurma
# veya , /dev/ttyUSB0, baud=57600
## Telemetri bağlantısı
# connection_string = '/dev/ttyUSB0'  # USB portunuza göre değiştirebilirsiniz
# baud_rate = 57600  # Telemetri modülünün baud rate'i
# MAVLink bağlantısı
# master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
master = connect_vehicle('COM7', baud=9600)

# Arming veya disarming
arm_vehicle(master, 1, 5)  # Aracı arm etmek için 1, disarm etmek için 0 gönderin.
# veya
armanddisarm(master, 1, 5)  # Aracı arm etmek için 1, disarm etmek için 0 gönderin.
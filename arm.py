from pymavlink import mavutil
import time
"""
def connect_vehicle(connection_string):
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print("Bağlantı kuruldu.")
    return master
"""


def arm_vehicle(master, arm):
    if arm == 1:
        master.arducopter_arm()
        print("Arming...")
        master.motors_armed_wait()
        print("Arm successful")
    elif arm == 0:
        master.arducopter_disarm()
        print("Disarming...")
        master.motors_disarmed_wait()
        print("Disarm successful")
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

"""
# Bağlantı kur
connection_string = 'udp:127.0.0.1:14550'
# UDP üzerinden 14550 portunu kullanarak bağlantı kurma
# veya , /dev/ttyUSB0, baud=57600
master = connect_vehicle(connection_string)
"""
# Arming veya disarming
#arm_vehicle(master, 1)  # Aracı arm etmek için 1, disarm etmek için 0 gönderin.
# veya
#armanddisarm(master, 1)  # Aracı arm etmek için 1, disarm etmek için 0 gönderin.





"""
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udpin:172.27.128.1:14550')
# 'udpin:172.27.128.1:14550'
# '/dev/ttyUSB0', baud=57600

# iki fonksiyon da arm ve disarm ediyor 0 olunca disarm ediyor
def arm_et(arm):
    # Arming veya disarming mesajı oluştur
    if arm == 1:
        master.arducopter_arm()
        print("Arming...")
    elif arm == 0:
        master.arducopter_disarm()
        print("Disarming...")
    else:
        print("Geçersiz parametre! 1 veya 0 olmalı.")




def armanddisarm(i):
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


# fonksiyon çağrılıyor 0=disarm









from pymavlink import mavutil
# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin: localhost: 14551')
# Wait for the first heartbeat
# This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat ( )
print("Heartbeat from system (system %u component %u)" %
(the_connection.target_system, the_connection.target_component))
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
mavutil.mavlink.MAV_CMD_COMPONENT_ARM DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print (msg)
"""
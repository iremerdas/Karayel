import arm_test as arm
import time


def sola_don(master, sol_guc, sag_guc, t):
    send_rc_channels_override(master, sol_guc, sag_guc)
    uyu(t)


def saga_don(master, sol_guc, sag_guc, t):
    send_rc_channels_override(master, sol_guc, sag_guc)
    uyu(t)


def zigzag1(master, sol_guc, sag_guc, t1, t2):
    sola_don(master, sol_guc, 1500, t1)
    saga_don(master, 1500, sag_guc, t2)


def zigzag2(master, sol_guc, sag_guc, t1, t2):
    saga_don(master, 1500, sag_guc, t1)
    sola_don(master, sol_guc, 1500, t2)


# RC_CHANNELS_OVERRIDE mesajını gönderme fonksiyonu
def send_rc_channels_override(master, left_speed, right_speed):
    master.mav.rc_channels_override_send(
        master.target_system,  # Hedef sistem
        master.target_component,  # Hedef bileşen
        0,  # Gaz kanalı
        0,  # Roll kanalı
        0,  # Pitch kanalı
        0,  # Yaw kanalı
        left_speed,  # Sol motor hızı (sola dönüş için)
        right_speed,  # Sağ motor hızı
        0, 0)  # Diğer kanallar (kullanılmıyor)


# Belirli bir süre sonra komutu iptal etme
def uyu(t):
    time.sleep(t)


connection_string = 'udp:127.0.0.1:14550'


def main():
    master = arm.connect_vehicle(connection_string)
    arm.arm_vehicle(master, 1)
    print("Arm edildi")
    send_rc_channels_override(master, 1500, 1500)
    print("İlk hareket edildi")
    uyu(2)
    sola_don(master, 1500, 500, 5)
    print("Sola döndü")
    arm.arm_vehicle(master, 0)
    print("Disarm edildi")


if __name__ == "__main__":
    main()

from pymavlink import mavutil

# MAVLink bağlantısını kur
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')


def get_target_coordinates():
    while True:
        msg = master.recv_match(type='MISSION_ITEM', blocking=True)
        if msg:
            lat = msg.x
            lon = msg.y
            alt = msg.z
            print(f"Received Target Coordinates: Lat={lat}, Lon={lon}, Alt={alt}")
            return lat, lon, alt








"""
# Raspberry Pi üzerinde kullandığınız COM portunu belirtin (örneğin, '/dev/ttyAMA0' veya '/dev/ttyUSB0')
connection_string = '/dev/ttyAMA0'  # veya '/dev/ttyUSB0'

# Bağlantıyı oluştur
vehicle = connect(connection_string, baud=57600)

print("Bağlandı!")
print("Araç Durumu:", vehicle.system_status)


# Bağlantı dizesini ayarlayın  
connection_string = 'udpin:0.0.0.0:14550'  # Telemetri bağlantısı (Mission Planner üzerinden gelen veri)  
vehicle = connect(connection_string, wait_ready=True)



# Hedef noktalarını dinamik olarak okuyacak bir döngü başlat
try:
    while True:
        # Hedef waypoint bilgilerini kontrol et
        if vehicle.armed:
            print("arm modunda")
        else:
            print("boku yedi")

        # Geçerli hedef waypoint'ini al
        current_wp = vehicle.commands.next
        print(f"Mevcut hedef waypoint: {current_wp}")

        # Eğer waypoint hedefi ulaşmak için gerekiyorsa, işlem yap
        # Örneğin, drone’unuus hedefe yönlendirin
        if current_wp != vehicle.commands.next:
            target_location = vehicle.commands[current_wp]
            print(f"Hedef Koordinatları: {target_location.lat}, {target_location.lon}")

        time.sleep(2)

except KeyboardInterrupt:
    print("Program durduruldu.")

# Uçuşu durdurun ve bağlantıyı kapatın
# vehicle.close()



1. Üstteki menüden "Flight Plan" sekmesine gir
2. Haritada, hedef koordinatın üzerine sağ tıkla açılan menüde "Add Waypoint" seçeneğini seç
3. 
4. 



from pymavlink import mavutil  
import time  

# Bağlantı dizesini ayarlayın  
connection_string = 'udpin:0.0.0.0:14550'  # Telemetri bağlantısı  
master = mavutil.mavlink_connection(connection_string)  

print("Bağlandı!")  

# Waypoint (hedef nokta) almak için  
try:  
    while True:  
        # Hedef waypoint sayısını al  
        master.mav.request_data_stream_send(master.target_system, master.target_component,   
                                             mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)  

        # Waypoint gönderimi  
        waypoints = []  
        master.mav.waypoint_request_send(master.target_system, master.target_component)  
        
        # Waypoint'leri al  
        while True:  
            msg = master.recv_match(type='MISSION_ITEM', blocking=True)  
            if msg is not None:  
                lat = msg.x / 1E7  # Dereceye dönüştürme  
                lon = msg.y / 1E7  # Dereceye dönüştürme  
                waypoints.append((lat, lon))  
                print(f"Waypoint Alındı: {lat}, {lon}")  
            if msg.seq == msg.count - 1:  
                break  

        time.sleep(1)  

except KeyboardInterrupt:  
    print("Program durduruldu.")  
finally:  
    master.close()
"""
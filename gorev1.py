import time
import cv2
import numpy as np
from collections import deque  # tespit edilen objenin merkezini depolamak için kullanılır
import hareket
import colorDetection as color
import mesafeSensor
import yonKontrol
import aci
import GPS as gps
from pymavlink import mavutil
import hedefKoordinat as hedef
import arm
import enKisaMesafe as mind


def get_heartbeat():
    print("heartbeat bekliyor")
    # MAVLink bağlantısı oluşturma
    connection_string = 'udp:127.0.0.1:14550'  # veya '/dev/ttyUSB0', baud=57600
    master_ = mavutil.mavlink_connection(connection_string)
    print("Bağlantı kuruldu.")
    master_.wait_heartbeat()
    return master_


master = get_heartbeat()

# nesne merkezini depolayacak veri tipi
buffer_size = 16  # deque nun boyutu
pts = deque(maxlen=buffer_size)  # nesnenin merkez noktaları

# capture
cap = cv2.VideoCapture(0)
cap.set(3, 960)
cap.set(4, 480)
cap.set(10, 100)


def en_kucuk_aci(aci_, yaw):
    # İki açı arasındaki farkı bulun
    fark = abs(aci_ - yaw)

    # Eğer fark 180 dereceden büyükse, farkı 360 dereceye çıkarın
    if fark > 180:
        fark = 360 - fark

    return fark


def yon_kontrol(master):
    hedef_x, hedef_y, _ = hedef.get_target_coordinates(master=master)

    aci_ = 450 - aci.ara_aci(hedef_x, hedef_y)

    yaw = aci.yaw_al(master)

    # iki fonksiyonda doğru arm edilecek deneyip iyi olan seçilecek şimdilik ilki seçildi
    arm.arm_vehicle(master, 1)
    # armanddisarm(1)

    o = en_kucuk_aci(aci_, yaw)

    if yaw >= 180:
        if o >= 180:
            hareket.sola_don(master, 1300, 1500, 5)  # (sol,sağ,zaman)
        else:
            hareket.saga_don(master, 1500, 1300, 5)
    else:
        if o >= 180:
            hareket.saga_don(master, 1500, 1300, 5)
        else:
            hareket.sola_don(master, 1300, 1500, 5)
        # gerekli denemelerler hangi açı için hangideğerler girilmei test edilerek öğrenilecek


def get_gps_data():
    return gps.gps_verileri_oku()


def mesafe_hesapla():
    x, y, _ = hedef.get_target_coordinates(master)
    min_dist = mind.mesafee(x, y)
    return min_dist


def get_yaw():
    return aci.yaw_al(master)


def get_sensor_data():
    return mesafeSensor.mesafe()


def draw_contour(frame, rect, colorr, label):
    ((x, y), (width, height), rotation) = rect
    box = cv2.boxPoints(rect)
    box = np.int64(box)
    cv2.drawContours(frame, [box], 0, colorr, 2)
    cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 0), -1)
    cv2.putText(frame, label, (int(x) - 50, int(y) - 50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255), 2)


def merge_contours(contours, threshold):
    merged_contours = []
    merged_flags = [False] * len(contours)
    for i, cnt1 in enumerate(contours):
        if merged_flags[i]:
            continue
        new_contour_group = [cnt1]
        merged_flags[i] = True
        for j, cnt2 in enumerate(contours):
            if i == j or merged_flags[j]:
                continue
            m1 = cv2.moments(cnt1)
            m2 = cv2.moments(cnt2)
            if m1["m00"] != 0 and m2["m00"] != 0:
                c_x1 = int(m1["m10"] / m1["m00"])
                c_y1 = int(m1["m01"] / m1["m00"])
                c_x2 = int(m2["m10"] / m2["m00"])
                c_y2 = int(m2["m01"] / m2["m00"])
                dist = np.sqrt((c_x1 - c_x2) ** 2 + (c_y1 - c_y2) ** 2)
                if dist < threshold:
                    new_contour_group.append(cnt2)
                    merged_flags[j] = True
        if len(new_contour_group) > 1:
            combined_contour = np.vstack(new_contour_group)
            hull = cv2.convexHull(combined_contour)
            merged_contours.append(hull)
        else:
            merged_contours.append(cnt1)
    return merged_contours


def find_largest_contour(contours):
    if contours:
        return max(contours, key=cv2.contourArea)
    return None


def main():
    global detected_objects
    if cap.isOpened():
        try:
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    print("Frame okunmuyor")
                    break
                detected_objects = []

                contours = color.color_detect(frame)

                for contour in contours:
                    contours_red, contours_yellow, contours_green = contour
                    merge_red = merge_contours(contours_red, 500)
                    for c in merge_red:
                        rect = cv2.minAreaRect(c)
                        ((x, y), (width, height), rotation) = rect
                        detected_objects.append(("Red", x, y, width, height, rotation))
                        draw_contour(frame, rect, (0, 0, 255), "Red")

                    merge_yellow = merge_contours(contours_yellow, 500)
                    for c in merge_yellow:
                        rect = cv2.minAreaRect(c)
                        ((x, y), (width, height), rotation) = rect
                        detected_objects.append(("Yellow", x, y, width, height, rotation))
                        draw_contour(frame, rect, (0, 255, 255), "Yellow")

                    merge_green = merge_contours(contours_green, 500)
                    for c in merge_green:
                        rect = cv2.minAreaRect(c)
                        ((x, y), (width, height), rotation) = rect
                        detected_objects.append(("Green", x, y, width, height, rotation))
                        draw_contour(frame, rect, (0, 255, 0), "Green")

                    largest_red = find_largest_contour(merge_red)
                    largest_yellow = find_largest_contour(merge_yellow)
                    largest_green = find_largest_contour(merge_green)

                    largest_contour = None
                    largest_area = 0
                    largest_color = None

                    if largest_red is not None and cv2.contourArea(largest_red) > largest_area:
                        largest_contour = largest_red
                        largest_area = cv2.contourArea(largest_red)
                        largest_color = "Red"
                    if largest_yellow is not None and cv2.contourArea(largest_yellow) > largest_area:
                        largest_contour = largest_yellow
                        largest_area = cv2.contourArea(largest_yellow)
                        largest_color = "Yellow"
                    if largest_green is not None and cv2.contourArea(largest_green) > largest_area:
                        largest_contour = largest_green
                        largest_area = cv2.contourArea(largest_green)
                        largest_color = "Green"

                    detected_objects.clear()

                    if largest_contour is not None:
                        rect = cv2.minAreaRect(largest_contour)
                        draw_contour(frame, rect, (0, 255, 255), largest_color)
                        m = cv2.moments(largest_contour)
                        if m["m00"] != 0:
                            c_x = int(m["m10"] / m["m00"])
                            c_y = int(m["m01"] / m["m00"])
                            detected_objects.append((largest_color, c_x, c_y, largest_area))

                    for obj in detected_objects:
                        label, x, y, width, height, rotation = obj
                        if label == "Red":
                            print(
                                f"{label}: Object Coordinates - x: {x}, y: {y}, width: {width}, height: {height}, "
                                f"rotation: {rotation}")
                            print("Sağa dön")
                            hareket.saga_don(master, 2000, 1000, 3)  # düzenle bunu
                            # motora ona göre güç ver
                        elif label == "Green":
                            print(
                                f"{label}: Object Coordinates - x: {x}, y: {y}, width: {width}, height: {height}, "
                                f"rotation: {rotation}")
                            print("Sola dön")
                            hareket.sola_don(master, 1000, 2000, 3)  # düzenle bunu
                        elif label == "Yellow":
                            print(
                                f"{label}: Object Coordinates - x: {x}, y: {y}, width: {width}, height: {height}, "
                                f"rotation: {rotation}")
                            if x < 320:
                                print("Sarı nesne kameranın solunda, sağa yönel")
                                hareket.saga_don(master, 2000, 1000, 3)  # düzenle bunu
                            elif x > 640:
                                print("Sarı nesne kameranın sağında, sola yönel")
                                hareket.sola_don(master, 1000, 2000, 3)  # düzenle bunu
                            else:
                                print("Sarı nesne kameranın ortasında, zikzak yönel")
                                hareket.zigzag1(master, 1000, 1000, 3, 3)  # düzenle bunu

                cv2.imshow('Original', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")
    return detected_objects


if __name__ == "__main__":
    # nesne tespit etme
    while True:
        distance = get_sensor_data()
        if distance == 1:  # engel varsa
            yonKontrol.yon_kontrol(master)
            detected_objects = main()
            if not detected_objects:  # Detected objects boş
                hareket.send_rc_channels_override(master, 1500, 1500)
            if not cap.isOpened():
                break
        else:  # engel yoksa
            print("2 metre uzaklıkta nesne tespit edilmedi")
            gps_data = get_gps_data()
            print(gps_data)
            min_distance = mesafe_hesapla()
            print(min_distance)
            if not min_distance == 0:  # hedefe gelmemişsek engelden kaçma görevi devam eder
                yonKontrol.yon_kontrol(master)
                hareket.send_rc_channels_override(master, 1500, 1500)  # degerleri değiştir teste göre
                # lat, lon, alt = get_target_coordinates(master)
                # goto_position_target_global_int(master, lat, lon, alt)
            else:  # hedefe ulaşıldı görev1 bitti
                print("engelden kacma görevi bitti!")
                break
        time.sleep(1)

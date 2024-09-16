import time
import cv2
import numpy as np
from collections import deque  # tespit edilen objenin merkezini depolamak için kullanılır
import colorDetection as color
import arduMesafeSensor
import arduAci as aci
import arduGPS as gps
import serial
import arduHedefKoordinat as hedef
import math

def get_ardu_connection():
    # Arduino ile bağlantıyı başlat
    arduino = serial.Serial('/dev/ttyUSB0', 9600)
    time.sleep(2)  # Arduino'nun hazır olması için bekleyin
    return arduino


arduino = get_ardu_connection()

# nesne merkezini depolayacak veri tipi
buffer_size = 16  # deque nun boyutu
pts = deque(maxlen=buffer_size)  # nesnenin merkez noktaları

# capture
cap = cv2.VideoCapture(0)
cap.set(3, 960)
cap.set(4, 480)
cap.set(10, 100)


def hareket_saga_don():
    arduino.write(b'saga_don\n')


def hareket_sola_don():
    arduino.write(b'sola_don\n')


def hareket_dur():
    arduino.write(b'dur\n')


def hareket_ileri():
    arduino.write(b'ileri\n')


# düzenlendi
def en_kucuk_aci(aci_, yaw):
    # İki açı arasındaki farkı bulun
    fark = abs(aci_ - yaw)

    # Eğer fark 180 dereceden büyükse, farkı 360 dereceye çıkarın
    if fark > 180:
        fark = 360 - fark

    return fark


def yonKontrol(arduino):

    hedef_tolerans = 5  # Hedefe yönelme toleransı, derece cinsinden

    while True:  # Sonsuz döngü
        hedef_x, hedef_y, _ = hedef.get_target_coordinates()  # Hedef koordinatlarını al

        aci_ = 450 - aci.ara_aci(hedef_x, hedef_y)  # Hedef açısını hesapla

        yaw = aci.yaw_al(arduino)  # Mevcut yaw açısını al

        o = en_kucuk_aci(aci_, yaw)  # En küçük açı farkını hesapla

        if yaw >= 180:
            if o >= 180:
                hareket_sola_don()  # Sola dön
            else:
                hareket_saga_don()  # Sağa dön
        else:
            if o >= 180:
                hareket_saga_don()  # Sağa dön
            else:
                hareket_sola_don()  # Sola dön

        # Hedefe yeterince yakın mıyız kontrolü
        if abs(o) < hedef_tolerans:  # Yaw açısının tolerans değerinin içinde olup olmadığını kontrol et
            break

        # Döngü arasında kısa bir gecikme
        time.sleep(0.1)  # Bu gecikme, işlemci yükünü azaltır ve sensör okuma hızını kontrol eder

def get_gps_data():   ## düzenlemek lazım ardunioya göre
    return gps.gps_verileri_oku(arduino)


def mesafee(hedef_a, hedef_b):
    a, b, c = gps.gps_verileri_oku(arduino)
    return math.sqrt(float((hedef_a - a)**2)+float((hedef_b - b)**2))


def mesafe_hesapla():
    x, y, _ = hedef.get_target_coordinates()
    min_dist = mesafee(x, y)
    return min_dist


def get_yaw():
    yaw = aci.yaw_al(arduino)  # Mevcut yaw açısını al
    return yaw


def get_distanceSensor_data():  # düzenle
    return arduMesafeSensor.mesafe()


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
                            hareket_saga_don()
                            yonKontrol()
                            hareket_ileri()
                        elif label == "Green":
                            print(
                                f"{label}: Object Coordinates - x: {x}, y: {y}, width: {width}, height: {height}, "
                                f"rotation: {rotation}")
                            print("Sola dön")
                            hareket_sola_don()
                            yonKontrol()
                            hareket_ileri()
                        elif label == "Yellow":
                            print(
                                f"{label}: Object Coordinates - x: {x}, y: {y}, width: {width}, height: {height}, "
                                f"rotation: {rotation}")
                            if x < 320:
                                print("Sarı nesne kameranın solunda, sağa yönel")
                                hareket_saga_don()
                                yonKontrol()
                                hareket_ileri()
                            elif x > 640:
                                print("Sarı nesne kameranın sağında, sola yönel")
                                hareket_sola_don()
                                yonKontrol()
                                hareket_ileri()
                            else:
                                print("Sarı nesne kameranın ortasında, zikzak yönel")
                                hareket_sola_don()
                                hareket_saga_don()
                                yonKontrol()
                                hareket_ileri()

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
        distance = get_distanceSensor_data()
        if distance < 2:  # engel varsa
            # Gorev 1!!!!!!!!!
            yonKontrol(arduino)
            if not cap.isOpened():
                print("kamera acık degil main name in içi mesafe sensorunden veri okunduktan sonraki kısım")
                break
            detected_objects = main()
            if not detected_objects:  # Detected objects boş
                print("BABA AKU YOK!! nesneler algılanmıyor GG")
            ### şamandıra renk kodlarını düzenle

        # şimdi önce hedef koordinatı şamadıranın ilerisine vereceksin. şamandırayı turuncu olarak görecev ve test edecek otomatik olarak yukarıda ki kısımda o kısmı halletmiş oluyorsun. görev 2ye geç
        # tekrar ve yeni bir hedef koordinat girmesi lazım. bu koordinate dümdüz git öndeki mesafe sensöründen veri al sonra bilmem kaç metre kala dur sonra da götü yanaştır
        # göt yanaştıktan sonra motorları durdur bi süre bekle mesela 10 saniye
        # sonra da onu çıkar işte bi şekilde başlangıçta başladığın konum bilgilerini tut ve hedef koordinatı güncelle oraya gitsin


        """
        else:  # engel yoksa
            print("2 metre uzaklıkta nesne tespit edilmedi")
            gps_data = get_gps_data()
            print(gps_data)
            min_distance = mesafe_hesapla()
            print(min_distance)
            if not min_distance == 0:  # hedefe gelmemişsek engelden kaçma görevi devam eder
                yonKontrol(arduino)
                hareket_ileri() # hedefe fogru düz gidecek o sırada engel çıkarsa 
            else:  # hedefe ulaşıldı görev1 bitti
                print("engelden kacma görevi bitti!")
                # Gorev 2!!!!!!!!!
                break
        """


        time.sleep(1)

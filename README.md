# Karayel - İnsansız Deniz Aracı Projesi

## 📌 Proje Açıklaması
Insansız Deniz Aracı (IDA), otonom navigasyon, nesne tespiti ve engelden kaçınma yeteneklerine sahip, yapay zeka destekli bir deniz aracıdır. Proje, deniz güvenliğini artırmak, çevresel gözetim sağlamak ve insansız operasyon yeteneklerini geliştirmek amacıyla tasarlanmıştır.İstenilen durumda MAVLink protokolü ile istenilen durumda Arduino ile kontrol edilebilir. 

## 🚀 Özellikler
- 🌊 **Otonom ve Uzaktan Kontrollü Navigasyon:** MAVLink üzerinden otonom kontrol veya manuel kullanım imkanı.
- 🛰️ **Gerçek Zamanlı Telemetri:** GPS ve diğer sensörlerden alınan veriler anlık olarak işlenir.
- 🔍 **Nesne Tespit ve Engel Algılama:** Kamera ve mesafe sensörleri kullanarak renk tabanlı nesne tespiti ve engel algılama.
- ⚙️ **Arduino Desteği:** Motorlar, servo kontrolleri ve sensörler Arduino ile yönetilir.
- 🔗 **MAVLink Entegrasyonu:** Araç, MAVLink protokolü üzerinden yönlendirilir.


### 🏗️ Elektronik Bileşenler
- **Kontrol Kartları**:
  - **Pixhawk 2.4.8** (Ana mikrodenetleyici)
  - **Raspberry Pi 4** (Görüntü işleme ve yüksek seviyeli işlemler için)
- **Sensörler**:
  - **GPS Modülü** (Holybro M9N)
  - **Ultrasonik Mesafe Sensörü** (JSN-SR04T)
  - **Sızdırmazlık Sensörü** (Su algılama ve güvenlik önlemleri için)
- **Batarya**: 12000 mAh Li-Po Batarya.
- **İletişim ve Kontrol**:
  - **Radyo Frekans (RF) Modülü**: Flysky FS-İ6X (Uzaktan kumanda)
  - **Telemetri Modülü**: Digi XBee 3 RF (3.2 km menzil)

### 💻  Yazılım Mimarisi
- **İşletim Sistemi**: Raspberry Pi OS
- **Gömülü Yazılım**: Python ile geliştirilmiş kontrol algoritmaları.
- **Haberleşme Protokolü**: MAVLink üzerinden Pixhawk ve Raspberry Pi haberleşmesi.
- **Haberleşme Protokolü 2**: Yedek plan olarak Ardunio ve Raspberry Pi haberleşmesi.
- **Görüntü İşleme**: OpenCV kullanılarak renk tespiti ve engel algılama.
- **Kontrol Yazılımı**: ArduPilot Mission Planner entegrasyonu.

## 📄 Algoritmalar
1. **Engel Algılama ve Kaçınma**
   
![image](https://github.com/user-attachments/assets/4fd128b2-868b-4c57-aed7-1e903439da35)

![image](https://github.com/user-attachments/assets/0099928e-5629-4631-b747-5e8c62afb14b)


2. **Rota Takibi ve Düzeltme**

![image](https://github.com/user-attachments/assets/3f5f9765-49df-4c41-aac9-354248976989)

3. **Renk Tespiti**

![image](https://github.com/user-attachments/assets/1bafe5bc-a71c-4cdb-9234-0b1b5b29c077)


4. **Limana Yanaşma**

![image](https://github.com/user-attachments/assets/56326f31-f7e9-47d3-b6c0-e6782fc8ad39)


![image](https://github.com/user-attachments/assets/aa24ace0-beae-4ff4-8fb8-9303eb17684b)

  
5. **Denge Durum Kontrolü**

![image](https://github.com/user-attachments/assets/7556f847-57fc-4942-8faa-2f4ac99ec970)

  
6. **Güvenlik Önlemleri** (Su sızdırmazlık ve acil stop mekanizmaları)
    
![image](https://github.com/user-attachments/assets/05d78af0-3e05-462e-a1b7-1b5dc42a5680)

   

## 🔧 Kurulum ve Kullanım
### Gereksinimler
- Raspberry Pi 4 ve Pixhawk/Ardunio
- Python 3.x ve ilgili kütüphaneler (`opencv`, `mavproxy`, `numpy`, `matplotlib`)
- Mission Planner yazılımı

### Kurulum Adımları
1. **Raspberry Pi Hazırlık**:
   ```bash
   sudo apt update && sudo apt upgrade -y
   sudo apt install python3-opencv python3-numpy python3-pip -y
   pip install pymavlink dronekit
   ```
2. **Mission Planner Yükleme**: Windows için [Mission Planner](http://ardupilot.org/planner/) yazılımını yükleyin.
3. **Pixhawk ve Sensör Kalibrasyonu**: Mission Planner üzerinden sensörlerin ve kumandanın kalibrasyonlarını yapın.
4. **Yazılımı Çalıştırma**:
   ```bash
   python3 main.py
   ```

## ⚠️ Test ve Simülasyon
- **Ardupilot SITL (Software In The Loop)** kullanılarak simülasyon ortamında algoritmalar test edilmiştir.


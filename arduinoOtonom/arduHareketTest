import serial
import time

# Seri portu ayarla (Arduino'nun bağlı olduğu portu belirtin)
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Port numarasını kendi sisteminize göre ayarlayın

def komut_gonder(komut):
    """
    Arduino'ya verilen komutu gönderir.
    """
    ser.write(komut.encode())

def ileri():
    """
    Motorları ileri hareket ettirir.
    """
    komut_gonder('F')
    print("Motorlar ileri gidiyor.")
    
def geri():
    """
    Motorları geri hareket ettirir.
    """
    komut_gonder('B')
    print("Motorlar geri gidiyor.")

def dur():
    """
    Motorları durdurur.
    """
    komut_gonder('S')
    print("Motorlar durduruldu.")

def saga_don():
    """
    Motorları sağa döndürür.
    """
    komut_gonder('R')
    print("Motorlar sağa dönüyor.")

def sola_don():
    """
    Motorları sola döndürür.
    """
    komut_gonder('L')
    print("Motorlar sola dönüyor.")

def main():
    try:
        # Örnek komutları gönder
        ileri()
        time.sleep(5)  # 5 saniye ileri hareket
        
        geri()
        time.sleep(5)  # 5 saniye geri hareket
        
        dur()
        time.sleep(5)  # 5 saniye dur
        
        saga_don()
        time.sleep(5)  # 5 saniye sağa dön
        
        sola_don()
        time.sleep(5)  # 5 saniye sola dön

    except KeyboardInterrupt:
        print("Program durduruldu.")
    finally:
        ser.close()  # Seri portu kapat

if __name__ == "__main__":
    main()

import aci
import hedefKoordinat as hedef
import arm
import hareket


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

    yaw = aci.yaw_al()

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

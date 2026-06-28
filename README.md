# ROS2 Autonomous Drive Controller

> ROS 2 üzerinde trafik işaretlerine ve sensör verilerine göre otonom araç manevra kontrolü.

---

## Proje Hakkında

Bu proje, **ZBEU OVAT** otonom araç yarışması için geliştirilmiş ROS 2 kontrol düğümüdür. Trafik işareti tanıma sisteminden gelen komutları, enkoder verisini ve şerit takip açısını birleştirerek araca yön, hız ve manevra komutları gönderir. Görev yönetimi bir durum makinesi üzerinde çalışır.

---

## Sistem Mimarisi

```
/traffic_sign  (Trafik işareti tanıma)   ─┐
/lane_angle    (Şerit takip açısı)        ├──► OvatKomutNode ──► Teensy (Serial)
/encoder       (Enkoder mesafe verisi)    ─┘         │
                                                      ├──► Direksiyon Komutu
                                                      ├──► Hız Komutu
                                                      └──► Fren Komutu
```

---

## Özellikler

- **Görev tabanlı durum makinesi** — aktif görevi dinamik olarak değiştirir
- Trafik işareti bazlı manevra yürütme
- Enkoder ile mesafe ölçümü ve görev tamamlama tespiti
- Şerit açısı ile sürekli direksiyon güncelleme
- Desteklenen manevra komutları:

| Komut | Açıklama |
|---|---|
| `sagadon` | Sağa dön |
| `soladon` | Sola dön |
| `dur` | Dur |
| `kirmizi_isik` | Kırmızı ışıkta bekle |
| `durak` | Durak manevrasına gir |
| `kavsak` | Kavşak geçişi |
| `ileri_ve_sol` | İleri gidip sola dön |
| `ileri_ve_sag` | İleri gidip sağa dön |
| `zduba` | Hız kesici geçişi |
| `tasitgiremez` | Araç giremez işareti |

---

## Gereksinimler

| Gereksinim | Sürüm |
|---|---|
| ROS 2 | Humble |
| C++ | 17 |
| Teensy | 4.x |

ROS 2 bağımlılıkları:
```
rclcpp, std_msgs
```

---

## Kurulum

```bash
git clone https://github.com/muhaliugur/Ros2-Autonomous-Drive-Controller.git
cd Ros2-Autonomous-Drive-Controller

rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## Kullanım

```bash
source install/setup.bash
ros2 run gpspackage ovat_komut_node
```

Sistem çalışmak için şu topic'lerin yayınlanmasını bekler:
- `/traffic_sign` — trafik işareti adı
- `/lane_angle` — şerit takip açısı (float)
- `/encoder` — enkoder değeri (int)

---

## Dosya Yapısı

```
├── src/
│   └── ovat_komut_node.cpp   # Ana kontrol düğümü
├── CMakeLists.txt
└── package.xml
```

---

## Katkıda Bulunanlar

**Muhammet Ali Uğur** — ZBEU Bilgisayar Mühendisliği / ZBEU OVAT Takımı

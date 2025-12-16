# STM32-IMU-Encoder-Fusion-ROS2
# STM32 ile IMU ve Encoder Sensör Füzyonu Tabanlı Robot Odometrisi

Bu proje, otonom mobil robotların navigasyon sistemleri için kritik olan **konum kestirimi (odometri)** problemini çözmek amacıyla geliştirilmiştir. STM32F4 mikrodenetleyicisi kullanılarak **BNO055 IMU** sensöründen alınan mutlak yönelim verisi ile **diferansiyel encoder** verileri birleştirilerek (Sensor Fusion) robotun anlık konumu (x, y) ve yönelimi (heading) hesaplanmıştır. Elde edilen veriler **micro-ROS** protokolü üzerinden **ROS 2** ortamına aktarılmış ve RViz üzerinde doğrulanmıştır.

## Proje Özellikleri

* **Sensör Füzyonu:** Enkoderlerden alınan doğrusal hız verisi ile IMU'dan alınan açısal yönelim (Yaw) verisinin kinematik model üzerinde birleştirilmesi.
* **Drift Önleme:** Tekerlek kaymasından kaynaklanan kümülatif açısal hataların, BNO055 sensörünün NDOF modu (Donanımsal Kalman Filtresi) ile elimine edilmesi.
* **micro-ROS Entegrasyonu:** Hesaplanan verilerin standart `nav_msgs/Odometry` mesaj tipi ile ROS 2 navigasyon yığınına uygun şekilde yayınlanması.
* **Gerçek Zamanlı Görselleştirme:** Robot hareketinin ve sensör tepkilerinin RViz ortamında eşzamanlı takibi.

---

## Donanım Konfigürasyonu

### Kullanılan Donanım

* **Mikrodenetleyici:** STM32F4 Discovery (STM32F407VGT6)
* **IMU Sensörü:** Bosch BNO055 (9-DOF, NDOF Modu)
* **Enkoderler:** KY-040 (Sol Motor) ve E38S6G5 (Sağ Motor)
* **Bağlantı:** USB Serial (micro-ROS Agent için)

### Pin Bağlantıları

| Bileşen | STM32 Pin | Protokol/Timer | Açıklama |
|---------|-----------|----------------|----------|
| **IMU (BNO055)** | PB6 (SCL) / PB7 (SDA) | I2C1 | 400 kHz Fast Mode |
| **Sol Enkoder** | PA15 (A) / PA1 (B) | TIM2 | 32-bit Sayaç |
| **Sağ Enkoder** | PA6 (A) / PA7 (B) | TIM3 | 16-bit Sayaç |

---

## ROS 2 Topic Yapısı

| Topic Adı | Mesaj Tipi | Açıklama |
|-----------|-----------|----------|
| `/robot_state` | `nav_msgs/Odometry` | Füzyonlanmış Konum (x,y) ve Yönelim (Quaternion) |
| `/fused_heading` | `geometry_msgs/Quaternion` | Sadece filtrelenmiş yönelim verisi |
| `/raw_imu` | `sensor_msgs/Imu` | Ham IMU verisi |
| `/wheel_speed` | `std_msgs/Float32` | Ham hız verisi (Debug) |

---

## Kurulum ve Çalıştırma

### 1. Micro-ROS Ajanını Başlatın

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1
```

### 2. Topic Listesini Kontrol Edin

```bash
ros2 topic list
```

### 3. Odometri Verisini İzleyin

```bash
ros2 topic echo /robot_state
```

### 4. RViz ile Görselleştirin

```bash
rviz2
```

RViz'de `Fixed Frame` ayarını `odom` olarak değiştirin ve `/robot_state` konusunu ekleyin.

---

## Test Sonuçları

### Test 1: Sistem Bağlantısı ve Topic Yapılandırması

Micro-ROS ajanının STM32 ile başarılı bir şekilde oturum açtığını gösteren terminal çıktısı.
<img width="733" height="488" alt="ROS2_Topic_Info_ve_Mesaj_Tipleri" src="https://github.com/user-attachments/assets/975bc1fc-6863-458d-879e-f818d69e44ab" />

**Gözlem:** Agent başarıyla başlatıldı, client (STM32) ile oturum kuruldu ve tüm publisher'lar oluşturuldu.

---

### Test 2: Canlı Odometri Verisi

`/robot_state` mesajının terminal çıktısı. Robotun pozisyonu (x, y) ve yönelimi (Quaternion) birleştirilmiş halde sunulur.

<img width="733" height="488" alt="Robot_State_Odometry_Data" src="https://github.com/user-attachments/assets/a4318700-7c8b-4856-82d9-319894a49f9c" />

**Gözlem:** Position, orientation ve covariance matrisleri düzgün şekilde yayınlanıyor.

---

### Test 3: Hız ve Yönelim Verilerinin Doğrulanması

Alt sistemlerin doğru çalıştığını gösteren debug verileri:
* `/wheel_speed` - Enkoderlerden okunan ham hız
* `/fused_heading` - IMU'dan okunan filtrelenmiş yönelim
<img width="733" height="488" alt="Wheel_Speed_Topic_Echo" src="https://github.com/user-attachments/assets/f31f8caa-6d85-43f8-a099-1c357371210d" />
<img width="733" height="488" alt="ROS2_Topic_Echo_Fused_Heading" src="https://github.com/user-attachments/assets/b7f1ebce-41d9-4691-b1a5-044aa0c33d35" />

---

### Test 4: Yerinde Dönüş Testi (Spot Turn)

Robot (x, y) konumu sabit tutularak sadece kendi ekseni etrafında döndürüldü. Enkoder hızı sıfır, sadece IMU açısal değişimi yakaladı.
<img width="1198" height="906" alt="Sabit_Konum_Yaw_Degisimi" src="https://github.com/user-attachments/assets/45a0a9ff-7ffe-46dd-b001-f4b229364a3e" />

**Gözlem:** Robot yer değiştirmeden 360° döndürüldü. Dönüş sonrası başlangıç yönelimine geri döndü.

---

### Test 5: Yörünge Takibi Testi (Füzyon)

Robot hem ileri sürülmüş hem de yönü değiştirildi. Kırmızı oklar robotun geçtiği yolu ve anlık yönelimini gösteriyor.
<img width="1198" height="906" alt="RViz_Robot_Hareket_Yorungesi" src="https://github.com/user-attachments/assets/630ab37f-9ed4-4ded-82ee-58685b73e372" />

**Gözlem:** Robot ileri-geri hareket + 90° dönüşler yaptı. Yörünge tutarlı.

---

### Test 6: Dinamik Tepki ve Drift Analizi

Sensöre ani ve sert dönüş hareketleri uygulandı. Hareket durduğunda veri 0.0 referansına döndü.
<img width="1920" height="1080" alt="IMU_Dinamik_Tepki_Grafigi" src="https://github.com/user-attachments/assets/b5addf37-09b0-4f01-9836-d2505e3b0756" />

**Gözlem:** Ani dönüşlere sistem hızlı tepki verdi. Kalıcı hata yok.

---

## Proje Test Videosu

**Video Linki:** https://youtu.be/fxYF3x8OX4I?si=GrGRt_Ip0hxMpd6Y

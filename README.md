# Duvardan Kaçan Kaplumbağa (`turtle_duvar_py`)

Bu paket, ROS 2 Humble ortamında `turtlesim` simülasyonundaki kaplumbağanın ekran sınırlarına (duvarlara) çarpmadan dönerek kaçmasını sağlar.

## İsterler ve Uygulama
1. **Paket:** `rclpy`, `geometry_msgs` ve `turtlesim` bağımlılıklarıyla oluşturuldu.
2. **Abone (Subscriber):** `/turtle1/pose` konusundan x ve y konum verileri okunmaktadır.
3. **Yayıncı (Publisher):** `/turtle1/cmd_vel` konusu üzerinden kaplumbağa ileri sürülmekte; x veya y değerleri 1.0'dan küçük ya da 10.0'dan büyük olduğunda açısal hız verilerek yön değiştirmesi sağlanmaktadır.
4. **Log:** Konum bilgisi saniyede bir `get_logger().info()` ile terminale basılmaktadır.
5. **Bonus:** Hız ve sınır değerleri (`lineer_hiz`, `aci_hiz`, `alt_sinir`, `ust_sinir`) dinamik parametre haline getirilmiştir.

## Kurulum ve Derleme

```bash
cd ~/ros2_ws
colcon build --packages-select turtle_duvar_py
source install/setup.bash
```

## Çalıştırma Adımları

1. **Simülasyonu Başlatın:**
   ```bash
   ros2 run turtlesim turtlesim_node
   ```

2. **Duvardan Kaçan Kaplumbağa Düğümünü Çalıştırın:**
   ```bash
   ros2 run turtle_duvar_py kacan_kaplumbaga
   ```

## Ekran Görüntüleri


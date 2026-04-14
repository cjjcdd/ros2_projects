# 🗺️ ROS2 Harita Alanı Hesaplama — Öğretici Proje

Bu proje, ROS2 ortamında bir **OccupancyGrid haritasının** taranmış alanını metre kare cinsinden hesaplamayı öğretmek amacıyla tasarlanmıştır. SLAM veya statik bir harita sunucusundan yayınlanan `/map` topic'ini dinleyerek haritanın bilinen (taranmış) alanını hesaplar ve terminale yazdırır.

---

## 📋 İçindekiler

- [Proje Amacı](#-proje-amacı)
- [Ön Koşullar](#-ön-koşullar)
- [OccupancyGrid Nedir?](#-occupancygrid-nedir)
- [Algoritma Mantığı](#-algoritma-mantığı)
- [Kurulum ve Derleme](#-kurulum-ve-derleme)
- [Çalıştırma](#-çalıştırma)
- [Beklenen Çıktı](#-beklenen-çıktı)
- [Kod Üzerinde Egzersiz](#-kod-üzerinde-egzersiz)
- [Sık Karşılaşılan Hatalar](#-sık-karşılaşılan-hatalar)

---

## 🎯 Proje Amacı

Bir otonom robot SLAM (Simultaneous Localization and Mapping) yaparken çevresini adım adım bir haritaya dönüştürür. Bu projede şu kavramları öğreneceksiniz:

- `nav_msgs/msg/OccupancyGrid` mesaj tipini okumak
- Hücre değerlerine göre bilinmeyeni bilinen alandan ayırt etmek
- Çözünürlük (resolution) bilgisini kullanarak piksel → metre² dönüşümü yapmak
- ROS2 subscriber yazımı ve lambda callback kullanımı

---

## 🔧 Ön Koşullar

| Gereksinim | Sürüm |
|---|---|
| Ubuntu | 22.04 (Jammy) |
| ROS2 | Humble veya Iron |
| C++ | 17 ve üzeri |
| `nav_msgs` paketi | ROS2 ile birlikte gelir |

Gerekli paketlerin yüklü olduğunu doğrulamak için:

```bash
ros2 pkg list | grep nav_msgs
```

---

## 🧩 OccupancyGrid Nedir?

`nav_msgs/msg/OccupancyGrid`, 2D bir haritayı **satır-sütun dizisi** olarak temsil eder. Her hücre (`cell`) aşağıdaki değerlerden birini taşır:

| Değer | Anlam |
|---|---|
| `0` | Boş alan (geçilebilir) |
| `1` – `99` | Kısmen dolu / belirsiz |
| `100` | Dolu alan (engel) |
| `-1` | Bilinmeyen alan (hiç taranmamış) |

### Mesajın Önemli Alanları

```
nav_msgs/msg/OccupancyGrid
├── header              → Zaman damgası ve frame bilgisi
├── info
│   ├── resolution      → Her hücrenin kenar uzunluğu (metre)
│   ├── width           → Haritanın sütun sayısı
│   ├── height          → Haritanın satır sayısı
│   └── origin          → Haritanın dünya koordinatındaki başlangıç noktası
└── data[]              → Tüm hücrelerin değeri (width × height boyutunda int8 dizisi)
```

> **Not:** `data` dizisi **satır-önce (row-major)** sırayla düzenlenir. Yani `data[y * width + x]` ifadesi (x, y) koordinatındaki hücreye erişir.

---

## 🧮 Algoritma Mantığı

Haritanın **taranmış (bilinen) alanını** bulmak için şu adımlar izlenir:

```
1. /map topic'inden OccupancyGrid mesajı al
2. data[] dizisindeki her hücreyi döngüyle gez
3. Değeri >= 0 olan hücreleri say  →  bunlar "bilinmekte" olan hücreler
4. Taranmış hücre sayısını çözünürlük karesiyle çarp
   Alan (m²) = Hücre Sayısı × (resolution × resolution)
5. Sonucu ekrana yazdır
```

### Neden `>= 0`?

Çünkü `data[]` dizisinde `-1` bilinmeyen alanı temsil eder. Değeri `0` veya daha büyük olan her hücre, robotun o bölgeyi **gördüğü ve işlediği** anlamına gelir; boş da olsa engelli de olsa orası taranmış bir alandır.

### Neden `resolution²`?

Her hücre, gerçek dünyada `resolution × resolution` metrekarelik bir kareyi temsil eder. Örneğin:

```
resolution = 0.05 m  →  her hücre = 0.05 × 0.05 = 0.0025 m²
10.000 hücre tarandıysa  →  Alan = 10.000 × 0.0025 = 25.0 m²
```

---

## 🚀 Kurulum ve Derleme

### 1. ROS2 Workspace Oluşturma

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Paketi Oluşturma

```bash
ros2 pkg create --build-type ament_cmake cal_map_size \
  --dependencies rclcpp nav_msgs visualization_msgs sensor_msgs tf2 tf2_geometry_msgs
```

### 3. Kaynak Dosyayı Yerleştirme

`cal_map_size_node.cpp` dosyasını `~/ros2_ws/src/cal_map_size/src/` dizinine kopyalayın.

### 4. `CMakeLists.txt` Güncelleme

`CMakeLists.txt` dosyasına aşağıdaki satırları ekleyin:

```cmake
add_executable(cal_map_size_node src/cal_map_size_node.cpp)
ament_target_dependencies(cal_map_size_node
  rclcpp
  nav_msgs
  visualization_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS cal_map_size_node
  DESTINATION lib/${PROJECT_NAME}
)
```

### 5. Derleme

```bash
cd ~/ros2_ws
colcon build --packages-select cal_map_size
source install/setup.bash
```

---

## ▶️ Çalıştırma

Düğümü çalıştırmadan önce bir harita kaynağına ihtiyaç vardır. Aşağıdaki iki yöntemden birini kullanabilirsiniz.

### Yöntem A — Gerçek SLAM ile (Turtlebot3 Örneği)

```bash
# Terminal 1: Simülasyonu başlat
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: SLAM çalıştır
ros2 launch turtlebot3_slam turtlebot3_slam.launch.py

# Terminal 3: Alan hesaplama düğümünü çalıştır
ros2 run cal_map_size cal_map_size_node
```

### Yöntem B — Kaydedilmiş Bir Haritayı Yükleyerek

```bash
# Terminal 1: Harita sunucusunu başlat
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=haritam.yaml

# Terminal 2: Lifecycle geçişini aktifleştir
ros2 lifecycle set /map_server activate

# Terminal 3: Alan hesaplama düğümünü çalıştır
ros2 run cal_map_size cal_map_size_node
```

---

## 📊 Beklenen Çıktı

Düğüm `/map` topic'inde bir mesaj aldığında terminalde şuna benzer bir çıktı görürsünüz:

```
[INFO] [1710000000.000000000] [CalculateMapSize]: Map Size -> 42.75 m2
```

Her yeni harita mesajı geldiğinde değer güncellenir. SLAM çalışıyorsa bu değerin zamanla büyüdüğünü gözlemleyebilirsiniz.

---

## 🏋️ Kod Üzerinde Egzersiz

Projeyi anladıktan sonra aşağıdaki geliştirmeleri kendiniz yapmayı deneyin:

1. **Yalnızca boş alanı hesapla:** `cell_value == 0` olan hücreleri sayarak geçilebilir alanı bulun.
2. **Yalnızca engel alanını hesapla:** `cell_value == 100` olan hücreleri sayın.
3. **Bilinmeyen oranı bul:** Toplam hücre sayısı `(width × height)` içinde `-1` olan hücrelerin yüzdesini hesaplayın.
4. **Farklı topic adını parametre yap:** `/map` yerine launch dosyasından farklı bir topic adı geçilebilsin.
5. **Marker ile görselleştir:** Hesaplanan değeri RViz2'de `visualization_msgs/msg/Marker` ile haritanın üstünde gösterin.

---

## ⚠️ Sık Karşılaşılan Hatalar

### `/map` topic'i yok, düğüm beklemeye alındı

```
# Hangi topic'lerin yayınlandığını kontrol edin:
ros2 topic list | grep map

# Mesajın gelip gelmediğini doğrulayın:
ros2 topic echo /map --once
```

### `area_m2` her zaman 0.0 çıkıyor

- `data` dizisinin boş olmadığından emin olun: `map_msg->data.size()` sıfırdan büyük olmalıdır.
- `map_msg->info.resolution` değerinin `0.0` olmadığını kontrol edin.

### Derleme hatası: `tf2_geometry_msgs` bulunamadı

```bash
sudo apt install ros-humble-tf2-geometry-msgs
```

---

## 📁 Proje Dosya Yapısı

```
cal_map_size/
├── CMakeLists.txt
├── package.xml
└── src/
    └── cal_map_size_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class CAL_MAP_SIZE : public rclcpp::Node{
public:
    CAL_MAP_SIZE() : Node("CalculateMapSize"){

        // /map konusuna abone oluyoruz
        auto mapCb = [this](nav_msgs::msg::OccupancyGrid::UniquePtr map_msg) -> void {
            float area_m2 = 0.0;
            int known_cell_count = 0;

            // Haritadaki her bir hücreyi kontrol et
            for (int8_t cell_value : map_msg->data) {
                // Bilinen hücreler: 0 (boş) veya 100 (dolu/duvar)
                // Bilinmeyenler (-1) hesaplamaya dahil edilmez
                if (cell_value >= 0) {
                    known_cell_count++;
                }
            }

            // Harita çözünürlüğünü al (metre/hücre)
            float res = map_msg->info.resolution;

            // Alan = Hücre Sayısı * (Çözünürlük * Çözünürlük)
            area_m2 = known_cell_count * (res * res);
            
            RCLCPP_INFO(this->get_logger(), "Güncel Harita Boyutu -> %.2f m2", area_m2);		
        };

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, mapCb);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CAL_MAP_SIZE>());
    rclcpp::shutdown();
    return 0;
}


add_executable(calc_map_size src/calc_map_size.cpp)
ament_target_dependencies(calc_map_size rclcpp nav_msgs sensor_msgs tf2 tf2_geometry_msgs visualization_msgs)

install(TARGETS
  calc_map_size
  DESTINATION lib/${PROJECT_NAME})


```

---

## 📚 Faydalı Kaynaklar

- [nav_msgs/OccupancyGrid — ROS2 Docs](https://docs.ros2.org/latest/api/nav_msgs/msg/OccupancyGrid.html)
- [ROS2 Humble — Resmi Dokümantasyon](https://docs.ros.org/en/humble/)
- [Turtlebot3 SLAM Rehberi](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/)

---

> Bu proje, ROS2 öğrenen öğrenciler için hazırlanmış öğretici bir alıştırmadır.  
> Sorularınız için kurs forum sayfasını kullanabilirsiniz.

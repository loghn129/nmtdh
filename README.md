# Mô phỏng ROS2 cho EEZYbotARM MK2

Workspace ROS2 (Jazzy, Ubuntu 24.04) mô phỏng cánh tay EEZYbotARM MK2 trong RViz2: chu trình gắp – thả sản phẩm
vào 3 khay và né vật cản.

- `src/eezybot_description` — URDF/xacro của tay, bàn, máng, 3 khay; cấu hình RViz.
- `src/eezybot_bringup` — các node điều khiển:
  - `slider_node` — kéo thanh trượt góc servo, xem tay chuyển động.
  - `sim_node` — chu trình gắp – thả, lập đường né vật cản (A* hoặc đường vòng cố định), tay giả lập hoặc tay thật.
  - `fake_esp32` — ESP32 ảo sau một cổng serial giả, để thử chế độ tay thật khi chưa có phần cứng.

## Cài và build

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
                 python3-colcon-common-extensions python3-numpy python3-serial python3-pytest
cd eezy_ws
colcon build --symlink-install
source install/setup.bash
```

## Chạy

Chỉ chạy một launch mỗi lúc. Đóng cửa sổ RViz là dừng.

```bash
ros2 launch eezybot_bringup sliders.launch.py     # kéo thanh trượt góc servo
ros2 launch eezybot_bringup sim.launch.py         # chu trình gắp - thả + né vật cản
#   tham số: speed:=3.0  planner:=co_dinh  auto_drop:=false  rviz:=false
ros2 launch eezybot_bringup real.launch.py        # RViz theo tay thật qua Serial
#   tham số: port:=/dev/ttyUSB0  baud:=115200  planner:=astar  rviz:=false
```

Thử chế độ tay thật khi chưa cắm tay:

```bash
ros2 run eezybot_bringup fake_esp32 --link /tmp/eezybot_tty
ros2 launch eezybot_bringup real.launch.py port:=/tmp/eezybot_tty
```

## Điều khiển

```bash
ros2 topic pub --once /eezybot/drop std_msgs/msg/Int32 '{data: 2}'                      # thả sản phẩm khay 2 (0 = ngẫu nhiên)
ros2 service call /eezybot/auto_drop std_srvs/srv/SetBool '{data: false}'               # bật / tắt tự thả
ros2 topic pub --once /eezybot/obstacle_size std_msgs/msg/String '{data: lon}'          # nho | vua | lon | rat_lon | số mm
ros2 topic pub --once /eezybot/add_obstacle geometry_msgs/msg/Point '{x: 0.04, y: 0.30}'  # vật cản tại (x, y) mét
ros2 service call /eezybot/clear_obstacles std_srvs/srv/Empty                           # xoá hết vật cản
ros2 topic echo /eezybot/status                                                         # trạng thái (JSON)
```

Trong RViz: chọn nút **Publish Point**, bấm một ô ở bảng "CO VAT CAN" để chọn cỡ, bấm lên mặt bàn để đặt vật cản,
bấm vào vật cản để xoá. Vật cản mới có hiệu lực từ chu trình kế tiếp.

## Test

```bash
cd src/eezybot_bringup && python3 -m pytest test -q
```

Mô phỏng không mô hình hóa rơ bánh răng. Kích thước máng, khay và khối sản phẩm là ước lượng để vẽ.

# Hệ Thống Điều Khiển UAV & Giám Sát Mạng Bằng Winsock (C/C++)

Hệ thống được thiết kế bằng ngôn ngữ **C (chuẩn C99)** sử dụng trực tiếp **Windows Sockets 2 (Winsock)** để giao tiếp với UAV/ArduPilot/Mission Planner qua giao thức **MAVLink v2** trên nền **UDP Port 14550**.

---

## 1. Cấu trúc thư mục dự án

```text
D:\Drone\
├── include/
│   ├── mavlink_lite.h           # Header MAVLink v2: đóng gói & kiểm tra CRC-16 X.25 (TX/RX), lệnh SET_MODE, ARM, TAKEOFF, WAYPOINT, LAND
│   └── network_metrics.h        # Cấu trúc lưu trữ chỉ số mạng, Jitter (RFC 3550/IPDV), Packet Loss, CRC errors, ARQ & Battery Failsafe
├── src/
│   ├── test_udp.c               # Kiểm tra kết nối UDP thô và nhận diện Magic Byte (0xFD/0xFE)
│   ├── telemetry_parser.c       # Giải mã tọa độ GPS, độ cao, vận tốc, pin, IMU từ gói MAVLink
│   ├── control_uav.c            # Gửi lệnh GUIDED, ARM và TAKEOFF lên 10m qua sendto() (Baseline không ARQ)
│   └── mission_with_network_metrics.c # Ứng dụng đa luồng hoàn chỉnh: đo đạc mạng, ARQ, CRC RX, Failsafe & điều khiển bay
├── bin/                         # Chứa các file thực thi đã biên dịch (.exe)
│   ├── test_udp.exe
│   ├── telemetry_parser.exe
│   ├── control_uav.exe
│   └── mission_with_network_metrics.exe
├── charts/                      # Ảnh biểu đồ PNG xuất tự động từ plot_reports.py
├── legacy_python/               # Lưu trữ các file code Python cũ để đối chiếu (Baseline Python)
├── build.bat                    # Script tự động biên dịch bằng MinGW GCC
├── run_clumsy.bat               # Script mở Clumsy 0.3 với quyền Administrator
├── plot_reports.py              # Script Python tự động vẽ 4 biểu đồ PNG từ network_report.csv & arq_report.csv
├── CMakeLists.txt               # File cấu hình CMake cho Visual Studio / CLion / VS Code
└── README.md                    # Tài liệu hướng dẫn
```

---

## 2. Hướng dẫn biên dịch

Chạy file script `build.bat` trong thư mục gốc:
```powershell
.\build.bat
```
Hoặc biên dịch thủ công từng file:
```powershell
gcc -O2 -Iinclude src/mission_with_network_metrics.c -o bin/mission_with_network_metrics.exe -lws2_32
```

---

## 3. Cách chạy từng chương trình

### A. Kiểm tra cổng UDP:
```powershell
.\bin\test_udp.exe
```

### B. Giám sát Telemetry UAV thời gian thực:
```powershell
.\bin\telemetry_parser.exe
```

### C. Ra lệnh cất cánh lên 10m (GUIDED):
```powershell
.\bin\control_uav.exe
```

### D. Trạm điều khiển tương tác (Interactive GCS CLI) & Đo đạc chỉ số mạng:
```powershell
.\bin\mission_with_network_metrics.exe
```
Sau khi kết nối tới UAV, chương trình cung cấp giao diện dòng lệnh tương tác trực tiếp (`[UAV-CLI] >`):
* `1` hoặc `arm`          : Mở khóa động cơ qua MAVLink ARQ
* `2` hoặc `takeoff 10`   : Cất cánh lên độ cao tùy chọn (mặc định 10m)
* `3` hoặc `goto 50`      : Bay tới Waypoint cách vị trí hiện tại 50m
* `4` hoặc `rtl`          : Tự động quay về điểm xuất phát và hạ cánh (Return to Launch)
* `5` hoặc `land`         : Hạ cánh khẩn cấp tại chỗ
* `6` hoặc `stats`        : Xem bảng hiệu năng mạng (Loss, Jitter RFC 3550, Throughput)
* `7` hoặc `monitor on`   : Bật in chỉ số mạng định kỳ 2 giây/lần
* `8` hoặc `auto`         : Tự động chạy toàn bộ kịch bản bay một mạch
* `0` hoặc `exit`         : Thoát an toàn

---

## 4. Các thông số kỹ thuật cốt lõi

* **UDP Port:** `14550`
* **Magic Byte:** `0xFD` (MAVLink v2)
* **CRC-16:** Thuật toán MCRF4XX / X.25 tích lũy `CRC_EXTRA` cho từng Message ID (kiểm tra toàn vẹn cả chiều gửi TX và chiều nhận RX).
* **Jitter:** Tính theo biến thiên thời gian đến giữa các gói liên tiếp (IPDV / chuẩn IETF RFC 3550) với hệ số $\alpha = 0.125$.
* **Packet Loss:** Theo dõi số thứ tự gói tin (Byte 4 trong frame MAVLink v2) theo modulo 256.

---

## 5. Các nâng cấp mạng chuyên sâu (Network Reliability & Synchronization)

1. **Cơ chế tin cậy tầng ứng dụng (Application-Layer Stop-and-Wait ARQ):**
   * Giải quyết nhược điểm mất gói tin của UDP đối với các lệnh điều khiển then chốt (`ARM`, `TAKEOFF`, `NAV_LAND`).
   * Tự động bắt gói tin `COMMAND_ACK` (Message ID 77). Nếu sau 800ms không nhận được phản hồi (mất gói/timeout), hệ thống tự động truyền lại (Retransmit) tối đa 4 lần.
   * Phân biệt mất mạng và từ chối an toàn: nếu UAV đã nhận lệnh nhưng trả về `MAV_RESULT != ACCEPTED` (`REJECTED`), ARQ dừng ngay lập tức mà không gửi lại.
2. **Xử lý Timeout & Failsafe 2 lớp (Link Heartbeat & Low Battery):**
   * **Link Failsafe:** Theo dõi thời gian giữa các gói `HEARTBEAT` (Message ID 0). Nếu mất tín hiệu vượt quá **3.0 giây**, hệ thống kích hoạt cảnh báo `[FAILSAFE CẢNH BÁO] MẤT KẾT NỐI MẠNG`.
   * **Battery Failsafe:** Khi nhận dữ liệu pin hợp lệ (`1%..100%` từ `SYS_STATUS`), nếu pin tụt dưới **20%** khi UAV đang bay (`alt > 1.0m`), hệ thống tự động gửi lệnh chuyển sang chế độ `LAND` để hạ cánh khẩn cấp.
3. **Đồng bộ hóa đa luồng an toàn (Thread Synchronization):**
   * Sử dụng `CRITICAL_SECTION` của Windows API để bảo vệ tài nguyên chia sẻ (`NetworkMetrics`) giữa Luồng nhận Socket (`NetworkMonitorThread`) và Luồng hiển thị/ghi log (`DisplayStatsThread`), loại bỏ hoàn toàn hiện tượng Race Condition.
4. **Tự động trích xuất dữ liệu hiệu năng ra file CSV (`network_report.csv` & `arq_report.csv`):**
   * `network_report.csv` (ghi mỗi 1 giây): `timestamp_ms`, `elapsed_sec`, `throughput_kbps`, `packet_rate_pps`, `total_packets`, `lost_packets`, `loss_rate_percent`, `jitter_ms`, `altitude_m`, `battery_percent`, `link_status`.
   * `arq_report.csv` (ghi mỗi lệnh ARQ): `timestamp_ms`, `command_name`, `mav_cmd`, `attempt_count`, `rtt_ms`, `result`, `loss_rate_percent`, `jitter_ms`.

---

## 6. Hướng dẫn thực nghiệm suy hao mạng (Network Impairment Test với Clumsy)

1. Tải công cụ giả lập mạng **Clumsy 0.3** (Windows): https://jagt.github.io/clumsy/ (hoặc chạy nhanh bằng `.\run_clumsy.bat`).
2. Mở Clumsy với quyền Administrator, tại ô **Filtering** nhập:
   ```text
   udp and (udp.DstPort == 14550 or udp.SrcPort == 14550)
   ```
3. Thiết lập các kịch bản thực nghiệm:
   * **Kịch bản 1 (Mạng lý tưởng):** Không bật Clumsy $\rightarrow$ Packet Loss ~0%, Jitter < 2ms, lệnh ARQ thành công ngay lần 1.
   * **Kịch bản 2 (Mất gói - Drop):** Bật checkbox `Drop`, đặt tỉ lệ `10%` đến `20%` $\rightarrow$ Quan sát số gói `lost_packets` tăng lên, cơ chế ARQ tự động kích hoạt gửi lại lần 2, lần 3 trên terminal.
   * **Kịch bản 3 (Độ trễ cao - Lag & Jitter):** Bật checkbox `Lag` (50–150ms) và `Throttle` $\rightarrow$ Quan sát chỉ số `Jitter` và `rtt_ms` trong `arq_report.csv` tăng lên.
   * **Kịch bản 4 (Đứt kết nối - Outage):** Đặt Drop 100% trong 5 giây $\rightarrow$ Quan sát hệ thống báo động `[FAILSAFE CẢNH BÁO] MẤT KẾT NỐI MẠNG (> 3.0s)`.
4. Xuất bộ ảnh biểu đồ tự động bằng 1 lệnh:
   ```powershell
   python plot_reports.py
   ```
   Bộ 4 ảnh PNG (`01_network_jitter_loss.png`, `02_network_throughput_pps.png`, `03_uav_altitude.png`, `04_arq_performance.png`) sẽ tự động lưu vào thư mục `charts/` (hoặc mở trực tiếp `network_report.csv` / `arq_report.csv` bằng Microsoft Excel).

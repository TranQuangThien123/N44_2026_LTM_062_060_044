import time
import math
import threading
from pymavlink import mavutil

UDP_IP = "127.0.0.1"
UDP_PORT = 14550

# --- CÁC BIẾN TOÀN CỤC THỐNG KÊ MẠNG ---
network_stats = {
    "total_bytes": 0,
    "interval_bytes": 0,
    "total_packets": 0,
    "interval_packets": 0,
    "lost_packets": 0,
    "last_seq": None,
    "last_packet_time": None,
    "jitter_ms": 0.0,
    "current_alt": 0.0,
    "current_lat": 0.0,
    "current_lon": 0.0,
    "battery_pct": 0,
    "running": True
}

# Khởi tạo kết nối UDP MAVLink hai chiều
print(f"[*] Dang ket noi toi UAV qua UDP {UDP_IP}:{UDP_PORT}...")
master = mavutil.mavlink_connection(f"udpin:{UDP_IP}:{UDP_PORT}", source_system=255)

print("[*] Dang cho phan hoi Heartbeat tu UAV...")
master.wait_heartbeat()
print(f"[+] Da ket noi toi UAV! (System ID: {master.target_system}, Component ID: {master.target_component})")

# --- LUỒNG 1: THU THẬP TELEMETRY & ĐO ĐẠC HIỆU NĂNG MẠNG ---
def network_monitor_thread():
    alpha = 0.125  # Hệ số làm mịn RFC 3550 để tính Jitter

    while network_stats["running"]:
        msg = master.recv_msg()
        current_time = time.time()

        if msg is not None:
            raw_buf = msg.get_msgbuf()
            packet_len = len(raw_buf)

            # Cập nhật thông số byte & gói
            network_stats["total_bytes"] += packet_len
            network_stats["interval_bytes"] += packet_len
            network_stats["total_packets"] += 1
            network_stats["interval_packets"] += 1

            # 1. Tính độ biến thiên trễ (Jitter)
            if network_stats["last_packet_time"] is not None:
                transit_time = (current_time - network_stats["last_packet_time"]) * 1000.0  # ms
                d = abs(transit_time - 20.0)  # Giả định tần số phát chuẩn ~50Hz (20ms/gói)
                network_stats["jitter_ms"] += alpha * (d - network_stats["jitter_ms"])
            network_stats["last_packet_time"] = current_time

            # 2. Phân tích Sequence Number để đo Packet Loss (Byte 4 trong frame MAVLink v2)
            if packet_len > 10 and raw_buf[0] == 0xFD:
                seq = raw_buf[4]
                if network_stats["last_seq"] is not None:
                    expected_seq = (network_stats["last_seq"] + 1) % 256
                    if seq != expected_seq:
                        diff = (seq - expected_seq) % 256
                        network_stats["lost_packets"] += diff
                network_stats["last_seq"] = seq

            # 3. Trích xuất Telemetry trạng thái drone
            msg_type = msg.get_type()
            if msg_type == "GLOBAL_POSITION_INT":
                network_stats["current_lat"] = msg.lat / 1e7
                network_stats["current_lon"] = msg.lon / 1e7
                network_stats["current_alt"] = msg.relative_alt / 1000.0
            elif msg_type == "SYS_STATUS":
                network_stats["battery_pct"] = msg.battery_remaining
        else:
            time.sleep(0.005)

# --- LUỒNG 2: IN BÁO CÁO CHỈ SỐ MẠNG ĐỊNH KỲ (MỖI 2 GIÂY) ---
def display_stats_thread():
    last_check_time = time.time()
    
    while network_stats["running"]:
        time.sleep(2.0)
        now = time.time()
        elapsed = now - last_check_time
        last_check_time = now

        # Tính băng thông tức thời
        throughput_kbps = (network_stats["interval_bytes"] * 8) / (elapsed * 1000.0)
        rate_pps = network_stats["interval_packets"] / elapsed
        
        # Reset bộ đếm khoảng thời gian
        network_stats["interval_bytes"] = 0
        network_stats["interval_packets"] = 0

        # Tính tỷ lệ rớt gói tin tích lũy
        total = network_stats["total_packets"] + network_stats["lost_packets"]
        loss_rate = (network_stats["lost_packets"] / total * 100.0) if total > 0 else 0.0

        print(f"\n[NETWORK STATS] Băng thông: {throughput_kbps:6.2f} Kbps | "
              f"Tốc độ: {rate_pps:5.1f} pkt/s | "
              f"Mất gói: {loss_rate:4.2f}% ({network_stats['lost_packets']} pkts) | "
              f"Jitter: {network_stats['jitter_ms']:4.2f} ms")
        print(f"[UAV STATUS]   Độ cao: {network_stats['current_alt']:5.2f}m | "
              f"Tọa độ: ({network_stats['current_lat']:.6f}, {network_stats['current_lon']:.6f}) | "
              f"Pin: {network_stats['battery_pct']}%")

# --- CÁC HÀM GỬI LỆNH ĐIỀU KHIỂN (UPLINK) ---
def set_mode(mode_name):
    mode_id = master.mode_mapping().get(mode_name)
    if mode_id is None:
        return
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"\n>>> [LỆNH MẠNG] Gửi Set Mode: {mode_name}")

def arm_vehicle():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("\n>>> [LỆNH MẠNG] Gửi ARM (Mở khóa động cơ)")

def takeoff(altitude=10):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    print(f"\n>>> [LỆNH MẠNG] Gửi TAKEOFF lên {altitude}m")

def send_waypoint(lat, lon, alt):
    """Gửi tọa độ mục tiêu để UAV tự bay tới trong chế độ GUIDED"""
    master.mav.mission_item_int_send(
        master.target_system,
        master.target_component,
        0,  # seq
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2,  # current = 2 (guided mode direct target)
        0,  # autocontinue
        0, 2, 0, 0,  # p1, p2 (accept radius), p3, p4
        int(lat * 1e7),
        int(lon * 1e7),
        alt
    )
    print(f"\n>>> [LỆNH MẠNG] Gửi WAYPOINT: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt}m")

def return_to_launch():
    """Gửi lệnh RTL - Quay về điểm xuất phát và tự hạ cánh"""
    set_mode("RTL")
    print("\n>>> [LỆNH MẠNG] Kích hoạt RTL (Return to Launch)")

# --- CHƯƠNG TRÌNH CHÍNH (FLIGHT MISSION) ---
if __name__ == "__main__":
    # Khởi động các thread mạng
    t1 = threading.Thread(target=network_monitor_thread, daemon=True)
    t2 = threading.Thread(target=display_stats_thread, daemon=True)
    t1.start()
    t2.start()

    try:
        # Bước 1: Chuẩn bị bay
        time.sleep(2)
        set_mode("GUIDED")
        time.sleep(2)
        arm_vehicle()
        time.sleep(2)

        # Bước 2: Cất cánh lên 10m
        takeoff(10)
        while network_stats["current_alt"] < 9.5:
            time.sleep(0.5)
        print("\n[+] Đã đạt độ cao 10m ổn định.")

        # Lấy tọa độ xuất phát làm mốc
        home_lat = network_stats["current_lat"]
        home_lon = network_stats["current_lon"]

        # Bước 3: Bay tới điểm Waypoint mới (cách điểm xuất phát ~50m về hướng Bắc)
        target_lat = home_lat + 0.0005
        target_lon = home_lon + 0.0005
        send_waypoint(target_lat, target_lon, 10.0)

        # Chờ drone di chuyển tới gần Waypoint
        print("[*] Đang di chuyển tới điểm Waypoint...")
        for _ in range(25):  # Theo dõi trong khoảng 25 giây
            dist = math.sqrt((network_stats["current_lat"] - target_lat)**2 + 
                             (network_stats["current_lon"] - target_lon)**2) * 111319.5
            if dist < 3.0:
                print(f"\n[+] Đã tới Waypoint mục tiêu (khoảng cách còn lại: {dist:.1f}m)!")
                break
            time.sleep(1)

        # Bước 4: Ra lệnh quay về vị trí ban đầu và hạ cánh (RTL)
        time.sleep(2)
        return_to_launch()

        # Theo dõi quá trình hạ cánh cho đến khi tiếp đất
        print("[*] Đang quay về và hạ cánh tự động...")
        while network_stats["current_alt"] > 0.3:
            time.sleep(1)
        print("\n[+] UAV đã tiếp đất an toàn hoàn tất nhiệm vụ.")

    except KeyboardInterrupt:
        print("\n[-] Dừng kịch bản bởi người dùng.")
    finally:
        network_stats["running"] = False
        time.sleep(1)
        master.close()
        print("[*] Đã đóng kết nối socket an toàn.")
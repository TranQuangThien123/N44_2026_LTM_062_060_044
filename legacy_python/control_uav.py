import time
from pymavlink import mavutil

# Địa chỉ và cổng UDP mà Mission Planner đang forward luồng MAVLink
UDP_IP = "127.0.0.1"
UDP_PORT = 14550

print(f"[*] Dang ket noi toi UAV qua UDP {UDP_IP}:{UDP_PORT}...")

# Khởi tạo kết nối UDP MAVLink hai chiều (vừa nhận vừa gửi)
# Thư viện mavutil quản lý socket UDP bên dưới và hỗ trợ đóng gói MAVLink v2
master = mavutil.mavlink_connection(f"udpin:{UDP_IP}:{UDP_PORT}", source_system=255)
# Đợi gói tin Heartbeat đầu tiên từ UAV để xác định mục tiêu (Target System & Component)
print("[*] Dang cho phan hoi Heartbeat tu UAV...")
master.wait_heartbeat()
print(f"[+] Da ket noi toi UAV! (System ID: {master.target_system}, Component ID: {master.target_component})")

def set_mode(mode_name):
    """Gui lenh chuyen che do bay qua mang"""
    mode_id = master.mode_mapping().get(mode_name)
    if mode_id is None:
        print(f"[-] Khong tim thay che do: {mode_name}")
        return
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"[>] Da gui lenh chuyen che do sang: {mode_name}")

def arm_vehicle():
    """Gui lenh mo khoa dong co (ARM)"""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        1,  # 1 = ARM, 0 = DISARM
        0, 0, 0, 0, 0, 0
    )
    print("[>] Da gui lenh ARM (Mo khoa dong co)...")

def takeoff(altitude=10):
    """Gui lenh cat canh toi do cao muc tieu (met)"""
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # Confirmation
        0, 0, 0, 0, 0, 0,
        altitude  # Tham so 7: Do cao dich tinh bang met
    )
    print(f"[>] Da gui lenh TAKEOFF toi do cao {altitude}m...")

# --- KỊCH BẢN THỰC THI QUA MẠNG ---
try:
    # 1. Chuyen sang che do GUIDED de nhan lenh dieu khien tu xa
    time.sleep(1)
    set_mode("GUIDED")
    time.sleep(2)

    # 2. Mo khoa dong co
    arm_vehicle()
    time.sleep(2)

    # 3. Ra lenh cat canh len 10m
    takeoff(altitude=10)

    # 4. Vong lap theo doi do cao thuc te tra ve qua UDP Telemetry
    print("\n[*] Dang theo doi qua trinh cat canh...")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # Chuyen sang met
            print(f"\r[Telemetry] Do cao hien tai: {current_alt:5.2f}m / 10.00m", end="")
            if current_alt >= 9.5:
                print("\n[+] Drone da dat do cao muc tieu on dinh!")
                break
        time.sleep(0.2)

except KeyboardInterrupt:
    print("\n[-] Dung kịch bản.")
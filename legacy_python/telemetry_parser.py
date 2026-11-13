import socket
from pymavlink import mavutil

UDP_IP = "127.0.0.1"
UDP_PORT = 14550

# 1. Khởi tạo UDP Socket thuần
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# 2. Khởi tạo parser MAVLink v2
mav = mavutil.mavlink.MAVLink(None)

print(f"[*] Dang phan tich goi tin Telemetry tren {UDP_IP}:{UDP_PORT}...\n")

try:
    last_seq = None
    lost_packets = 0
    total_packets = 0

    while True:
        data, addr = sock.recvfrom(2048)
        total_packets += 1

        # Trích xuất Header tầng mạng (Byte 4: Sequence Number)
        if len(data) > 10 and data[0] == 0xFD:
            seq = data[4]
            if last_seq is not None:
                # Kiểm tra mất gói tin (Packet Loss)
                expected_seq = (last_seq + 1) % 256
                if seq != expected_seq:
                    lost_packets += (seq - expected_seq) % 256
            last_seq = seq

        # Giải mã frame MAVLink
        msgs = mav.parse_buffer(data)
        if msgs:
            for msg in msgs:
                msg_type = msg.get_type()

                # Tọa độ GPS & Độ cao
                if msg_type == "GLOBAL_POSITION_INT":
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.relative_alt / 1000.0  # mét
                    vx = msg.vx / 100.0              # m/s
                    vy = msg.vy / 100.0              # m/s
                    print(f"[GPS] Vĩ độ: {lat:.6f} | Kinh độ: {lon:.6f} | Độ cao: {alt:.2f}m | Tốc độ: {vx:.1f} m/s")

                # Trạng thái IMU / Góc nghiêng
                elif msg_type == "ATTITUDE":
                    roll = round(msg.roll * 57.2958, 2)   # rad sang độ
                    pitch = round(msg.pitch * 57.2958, 2)
                    yaw = round(msg.yaw * 57.2958, 2)
                    # In định kỳ khi có góc nghiêng
                    # print(f"[IMU] Roll: {roll}° | Pitch: {pitch}° | Yaw: {yaw}°")

                # Trạng thái Pin & Điện áp
                elif msg_type == "SYS_STATUS":
                    voltage = msg.voltage_battery / 1000.0  # Volt
                    battery_remaining = msg.battery_remaining  # %
                    print(f"[PIN] Điện áp: {voltage:.2f}V | Dung lượng còn: {battery_remaining}%")

except KeyboardInterrupt:
    loss_rate = (lost_packets / (total_packets + lost_packets)) * 100 if (total_packets + lost_packets) > 0 else 0
    print(f"\n--- Thống kê Mạng UDP ---")
    print(f"Tổng số gói nhận: {total_packets}")
    print(f"Gói tin bị rớt (ước tính qua Seq): {lost_packets}")
    print(f"Tỷ lệ mất gói (Packet Loss Rate): {loss_rate:.2f}%")
    sock.close()
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 14550

# Khởi tạo Socket UDP thuần
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Gán socket vào IP và Port đã cấu hình
sock.bind((UDP_IP, UDP_PORT))

print(f"[*] Dang lang nghe goi tin UDP tren {UDP_IP}:{UDP_PORT}...")

try:
    packet_count = 0
    while True:
        # Nhan byte thô từ Mission Planner
        data, addr = sock.recvfrom(2048)
        packet_count += 1
        
        # Nhận diện phiên bản MAVLink qua Magic Byte đầu tiên
        magic_byte = data[0]
        if magic_byte == 0xFD:
            proto = "MAVLink v2"
        elif magic_byte == 0xFE:
            proto = "MAVLink v1"
        else:
            proto = "Raw Data"

        print(f"[{packet_count}] Nhan {len(data):3d} bytes tu {addr} | Dinh dang: {proto} | Byte dau: {hex(magic_byte)}")

except KeyboardInterrupt:
    print("\n[-] Da dung chuong trinh.")
    sock.close()
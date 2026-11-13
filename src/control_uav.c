#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "mavlink_lite.h"

#pragma comment(lib, "ws2_32.lib")

#define UDP_PORT 14550
#define BUFFER_SIZE 2048

int main() {
    SetConsoleOutputCP(CP_UTF8);

    WSADATA wsaData;
    SOCKET sock = INVALID_SOCKET;
    struct sockaddr_in server_addr, client_addr;
    int client_addr_len = sizeof(client_addr);
    uint8_t rx_buf[BUFFER_SIZE];
    uint8_t tx_buf[BUFFER_SIZE];
    uint8_t tx_seq = 0;

    printf("====================================================\n");
    printf("   CONTROL UAV WINSOCK: ĐIỀU KHIỂN CẤT CÁNH GUIDED  \n");
    printf("====================================================\n\n");

    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) return 1;

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        WSACleanup();
        return 1;
    }

    BOOL reuse = TRUE;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse));

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    server_addr.sin_port = htons(UDP_PORT);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        printf("[-] Khong the bind vao port %d. Ma loi: %d\n", UDP_PORT, WSAGetLastError());
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    printf("[*] Dang cho phan hoi Heartbeat tu UAV tren port %d...\n", UDP_PORT);

    uint8_t target_sys = 1;
    uint8_t target_comp = 1;
    bool connected = false;

    // 1. Chờ Heartbeat đầu tiên để định danh UAV
    while (!connected) {
        int bytes = recvfrom(sock, (char *)rx_buf, BUFFER_SIZE, 0,
                             (struct sockaddr *)&client_addr, &client_addr_len);
        if (bytes > 10 && rx_buf[0] == MAVLINK_V2_MAGIC) {
            uint32_t msg_id = (uint32_t)rx_buf[7] | ((uint32_t)rx_buf[8] << 8) | ((uint32_t)rx_buf[9] << 16);
            if (msg_id == MAVLINK_MSG_ID_HEARTBEAT) {
                target_sys = rx_buf[5];
                target_comp = rx_buf[6];
                connected = true;

                char ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &(client_addr.sin_addr), ip_str, INET_ADDRSTRLEN);
                printf("[+] Da ket noi toi UAV tai %s:%d! (System ID: %d, Component ID: %d)\n\n",
                       ip_str, ntohs(client_addr.sin_port), target_sys, target_comp);
            }
        }
    }

    // Thiết lập timeout nhận gói (1000ms)
    DWORD timeout = 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout));

    // Bước 1: Chuyển sang chế độ GUIDED
    Sleep(1000);
    int len = mavlink_build_set_mode(tx_buf, tx_seq++, target_sys, COPTER_MODE_GUIDED);
    sendto(sock, (const char *)tx_buf, len, 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
    printf("[>] Da gui lenh chuyen che do: GUIDED\n");
    Sleep(2000);

    // Bước 2: Mở khóa động cơ (ARM)
    len = mavlink_build_command_long(tx_buf, tx_seq++, target_sys, target_comp,
                                     MAV_CMD_COMPONENT_ARM_DISARM, 1.0f, 0, 0, 0, 0, 0, 0);
    sendto(sock, (const char *)tx_buf, len, 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
    printf("[>] Da gui lenh ARM (Mo khoa dong co)...\n");
    Sleep(2000);

    // Bước 3: Ra lệnh cất cánh lên 10m
    const float target_alt = 10.0f;
    len = mavlink_build_command_long(tx_buf, tx_seq++, target_sys, target_comp,
                                     MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, target_alt);
    sendto(sock, (const char *)tx_buf, len, 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
    printf("[>] Da gui lenh TAKEOFF len %.1fm...\n\n", target_alt);

    // Bước 4: Vòng lặp theo dõi độ cao phản hồi
    printf("[*] Dang theo doi do cao thuc te qua UDP Telemetry...\n");
    while (1) {
        int bytes = recvfrom(sock, (char *)rx_buf, BUFFER_SIZE, 0,
                             (struct sockaddr *)&client_addr, &client_addr_len);
        if (bytes > 10 && rx_buf[0] == MAVLINK_V2_MAGIC) {
            uint8_t payload_len = rx_buf[1];
            uint32_t msg_id = (uint32_t)rx_buf[7] | ((uint32_t)rx_buf[8] << 8) | ((uint32_t)rx_buf[9] << 16);

            if (msg_id == MAVLINK_MSG_ID_GLOBAL_POSITION_INT && payload_len >= 20) {
                const uint8_t *payload = rx_buf + 10;
                int32_t rel_alt_raw = *(const int32_t *)(payload + 16);
                double current_alt = (double)rel_alt_raw / 1000.0;

                printf("\r[Telemetry] Do cao hien tai: %5.2fm / %.2fm", current_alt, target_alt);
                fflush(stdout);

                if (current_alt >= (target_alt - 0.5)) {
                    printf("\n[+] Drone da dat do cao muc tieu on dinh!\n");
                    break;
                }
            }
        }
    }

    closesocket(sock);
    WSACleanup();
    return 0;
}

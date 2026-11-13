#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <stdint.h>
#include "mavlink_lite.h"
#include "network_metrics.h"

#pragma comment(lib, "ws2_32.lib")

#define UDP_PORT 14550
#define BUFFER_SIZE 2048

int main() {
    SetConsoleOutputCP(CP_UTF8);

    WSADATA wsaData;
    SOCKET sock = INVALID_SOCKET;
    struct sockaddr_in server_addr, client_addr;
    int client_addr_len = sizeof(client_addr);
    uint8_t buffer[BUFFER_SIZE];

    LARGE_INTEGER freq;
    QueryPerformanceFrequency(&freq);

    NetworkMetrics metrics = {0};
    metrics.last_seq = -1;

    printf("====================================================\n");
    printf("   TELEMETRY PARSER WINSOCK: GIẢI MÃ DỮ LIỆU UAV    \n");
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
        printf("[-] Khong the bind vao port %d\n", UDP_PORT);
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    printf("[*] Dang phan tich goi tin Telemetry tren port %d...\n\n", UDP_PORT);

    while (1) {
        int bytes = recvfrom(sock, (char *)buffer, BUFFER_SIZE, 0,
                             (struct sockaddr *)&client_addr, &client_addr_len);
        if (bytes <= 0) continue;

        metrics.total_packets++;

        // Kiểm tra frame MAVLink v2
        if (bytes > 10 && buffer[0] == MAVLINK_V2_MAGIC) {
            uint8_t payload_len = buffer[1];
            uint8_t seq = buffer[4];
            uint32_t msg_id = (uint32_t)buffer[7] | ((uint32_t)buffer[8] << 8) | ((uint32_t)buffer[9] << 16);

            update_sequence(&metrics, seq);

            const uint8_t *payload = buffer + 10;

            // 1. Tọa độ GPS & Độ cao: GLOBAL_POSITION_INT (ID 33)
            if (msg_id == MAVLINK_MSG_ID_GLOBAL_POSITION_INT && payload_len >= 20) {
                int32_t lat_raw = *(const int32_t *)(payload + 4);
                int32_t lon_raw = *(const int32_t *)(payload + 8);
                int32_t rel_alt_raw = *(const int32_t *)(payload + 16);
                int16_t vx_raw = *(const int16_t *)(payload + 20);
                int16_t vy_raw = *(const int16_t *)(payload + 22);

                double lat = (double)lat_raw / 1e7;
                double lon = (double)lon_raw / 1e7;
                double alt = (double)rel_alt_raw / 1000.0;
                float vx = (float)vx_raw / 100.0f;
                float vy = (float)vy_raw / 100.0f;

                printf("[GPS] Vi do: %10.6f | Kinh do: %10.6f | Do cao: %5.2fm | Toc do: %4.1f m/s\n",
                       lat, lon, alt, sqrt(vx * vx + vy * vy));
                fflush(stdout);
            }
            // 2. Trạng thái Pin & Điện áp: SYS_STATUS (ID 1)
            else if (msg_id == MAVLINK_MSG_ID_SYS_STATUS && payload_len >= 31) {
                uint16_t volt_raw = *(const uint16_t *)(payload + 14);
                int8_t battery = (int8_t)payload[30];
                float voltage = (float)volt_raw / 1000.0f;
                printf("[PIN] Dien ap: %5.2fV | Dung luong con: %3d%%\n", voltage, battery);
            }
            // 3. Góc nghiêng IMU: ATTITUDE (ID 30)
            else if (msg_id == MAVLINK_MSG_ID_ATTITUDE && payload_len >= 28) {
                float roll = *(const float *)(payload + 4) * 57.2958f;
                float pitch = *(const float *)(payload + 8) * 57.2958f;
                float yaw = *(const float *)(payload + 12) * 57.2958f;
                // printf("[IMU] Roll: %5.1f | Pitch: %5.1f | Yaw: %5.1f\n", roll, pitch, yaw);
            }
        }
    }

    closesocket(sock);
    WSACleanup();
    return 0;
}

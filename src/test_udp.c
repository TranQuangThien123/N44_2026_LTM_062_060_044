#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <stdint.h>
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
    uint8_t buffer[BUFFER_SIZE];

    printf("====================================================\n");
    printf("   TEST UDP WINSOCK: LẮNG NGHE GÓI TIN MAVLINK      \n");
    printf("====================================================\n\n");

    // 1. Khởi tạo Winsock
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        fprintf(stderr, "[-] WSAStartup that bai!\n");
        return 1;
    }

    // 2. Tạo UDP Socket
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        fprintf(stderr, "[-] Khong the tao socket. Ma loi: %d\n", WSAGetLastError());
        WSACleanup();
        return 1;
    }

    // Cho phép tái sử dụng địa chỉ (SO_REUSEADDR) để không bị đụng với Mission Planner
    BOOL reuse = TRUE;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse));

    // 3. Bind socket vào 127.0.0.1:14550
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    server_addr.sin_port = htons(UDP_PORT);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
        fprintf(stderr, "[-] Bind vao port %d that bai! Ma loi: %d\n", UDP_PORT, WSAGetLastError());
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    printf("[*] Dang lang nghe goi tin UDP tren 0.0.0.0:%d...\n", UDP_PORT);

    uint64_t packet_count = 0;
    while (1) {
        int bytes_received = recvfrom(sock, (char *)buffer, BUFFER_SIZE, 0,
                                      (struct sockaddr *)&client_addr, &client_addr_len);
        if (bytes_received > 0) {
            packet_count++;
            uint8_t magic = buffer[0];
            const char *proto = "Raw Data";
            if (magic == MAVLINK_V2_MAGIC) proto = "MAVLink v2";
            else if (magic == MAVLINK_V1_MAGIC) proto = "MAVLink v1";

            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(client_addr.sin_addr), client_ip, INET_ADDRSTRLEN);

            printf("[%llu] Nhan %3d bytes tu %s:%d | Dinh dang: %-10s | Byte dau: 0x%02X\n",
                   packet_count, bytes_received, client_ip, ntohs(client_addr.sin_port), proto, magic);
            fflush(stdout);
        }
    }

    closesocket(sock);
    WSACleanup();
    return 0;
}

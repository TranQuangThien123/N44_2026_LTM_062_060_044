#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "mavlink_lite.h"
#include "network_metrics.h"

#pragma comment(lib, "ws2_32.lib")

#define UDP_PORT 14550
#define BUFFER_SIZE 2048
#define CSV_FILE_PATH "D:\\Drone\\network_report.csv"
#define ARQ_CSV_FILE_PATH "D:\\Drone\\arq_report.csv"

// Biến toàn cục chia sẻ giữa các luồng
static SOCKET g_sock = INVALID_SOCKET;
static struct sockaddr_in g_uav_addr;
static int g_uav_addr_len = sizeof(g_uav_addr);
static NetworkMetrics g_metrics;
static LARGE_INTEGER g_perf_freq;
static uint8_t g_tx_seq = 0;

static double g_home_lat = 0.0;
static double g_home_lon = 0.0;
static bool g_home_set = false;
static bool g_show_periodic_stats = false; // Mặc định tắt in liên tục để không che mất dòng lệnh nhập

// Gửi một gói tin MAVLink tới UAV qua Winsock
static void send_uav_packet(const uint8_t *buf, int len) {
    if (g_sock != INVALID_SOCKET) {
        sendto(g_sock, (const char *)buf, len, 0, (struct sockaddr *)&g_uav_addr, g_uav_addr_len);
    }
}

// Hàm lấy nhanh độ cao an toàn luồng
static double get_safe_altitude(void) {
    metrics_lock(&g_metrics);
    double alt = g_metrics.current_alt;
    metrics_unlock(&g_metrics);
    return alt;
}

// Hàm kiểm tra trạng thái Battery Failsafe an toàn luồng
static bool is_battery_failsafe_active(void) {
    metrics_lock(&g_metrics);
    bool triggered = g_metrics.battery_failsafe_triggered;
    metrics_unlock(&g_metrics);
    return triggered;
}

// Hàm lấy nhanh tọa độ GPS an toàn luồng
static void get_safe_position(double *lat, double *lon) {
    metrics_lock(&g_metrics);
    *lat = g_metrics.current_lat;
    *lon = g_metrics.current_lon;
    metrics_unlock(&g_metrics);
}

// Khởi tạo file CSV log thống kê Stop-and-Wait ARQ
static void init_arq_csv(void) {
    FILE *fp = fopen(ARQ_CSV_FILE_PATH, "w");
    if (fp) {
        fprintf(fp, "timestamp_ms,command_name,mav_cmd,attempt_count,rtt_ms,result,loss_rate_percent,jitter_ms\n");
        fclose(fp);
    }
}

// Ghi một bản ghi kết quả thực thi ARQ ra file CSV
static void log_arq_event(double timestamp_ms, const char *cmd_name, uint16_t command,
                          int attempts, double rtt_ms, const char *result_str,
                          double loss_rate, double jitter_ms) {
    FILE *fp = fopen(ARQ_CSV_FILE_PATH, "a");
    if (fp) {
        fprintf(fp, "%.0f,%s,%u,%d,%.2f,%s,%.2f,%.2f\n",
                timestamp_ms, cmd_name, command, attempts, rtt_ms, result_str, loss_rate, jitter_ms);
        fclose(fp);
    }
}

// --- CƠ CHẾ TIN CẬY STOP-AND-WAIT ARQ VỚI COMMAND_ACK TRÊN UDP ---
static bool send_command_long_with_arq(uint16_t command, const char *cmd_name,
                                       float p1, float p2, float p3, float p4,
                                       float p5, float p6, float p7,
                                       int max_retries, int timeout_ms) {
    uint8_t tx_buf[BUFFER_SIZE];
    uint8_t target_sys = 1;
    uint8_t target_comp = 1;
    double arq_start_ms = get_high_res_time_ms(g_perf_freq);

    metrics_lock(&g_metrics);
    target_sys = g_metrics.target_system;
    target_comp = g_metrics.target_component;
    metrics_unlock(&g_metrics);

    printf("\n>>> [ARQ BẮT ĐẦU] Gửi lệnh '%s' (MAV_CMD=%d) qua UDP...\n", cmd_name, command);

    for (int attempt = 1; attempt <= max_retries; attempt++) {
        metrics_lock(&g_metrics);
        g_metrics.ack_updated = false;
        g_metrics.last_ack_command = 0;
        g_metrics.last_ack_result = 255;
        metrics_unlock(&g_metrics);

        int len = mavlink_build_command_long(tx_buf, g_tx_seq++, target_sys, target_comp,
                                             command, p1, p2, p3, p4, p5, p6, p7);
        send_uav_packet(tx_buf, len);

        int waited_ms = 0;
        bool got_ack = false;
        uint8_t ack_result = 255;

        while (waited_ms < timeout_ms) {
            Sleep(10);
            waited_ms += 10;

            metrics_lock(&g_metrics);
            if (g_metrics.ack_updated && g_metrics.last_ack_command == command) {
                got_ack = true;
                ack_result = g_metrics.last_ack_result;
                metrics_unlock(&g_metrics);
                break;
            }
            metrics_unlock(&g_metrics);
        }

        if (got_ack) {
            if (ack_result == MAV_RESULT_ACCEPTED) {
                double arq_end_ms = get_high_res_time_ms(g_perf_freq);
                double total_rtt_ms = arq_end_ms - arq_start_ms;

                metrics_lock(&g_metrics);
                g_metrics.arq_total_cmds++;
                g_metrics.arq_success_cmds++;
                g_metrics.arq_total_retries += (uint64_t)(attempt - 1);
                g_metrics.arq_total_rtt_ms += total_rtt_ms;
                uint64_t total_pkts = g_metrics.total_packets + g_metrics.lost_packets;
                double loss_rate = (total_pkts > 0) ? ((double)g_metrics.lost_packets / total_pkts * 100.0) : 0.0;
                double jitter = g_metrics.jitter_ms;
                metrics_unlock(&g_metrics);

                log_arq_event(arq_end_ms, cmd_name, command, attempt, total_rtt_ms, "ACCEPTED", loss_rate, jitter);
                printf("[ARQ SUCCESS] Đã nhận COMMAND_ACK cho lệnh '%s' (ACCEPTED) sau %.1f ms (Lần thử %d/%d)!\n",
                       cmd_name, total_rtt_ms, attempt, max_retries);
                return true;
            } else {
                // UAV đã nhận được gói tin (không mất mạng) nhưng từ chối vì lý do an toàn/trạng thái -> Dừng ngay, không retry
                double arq_end_ms = get_high_res_time_ms(g_perf_freq);
                double total_rtt_ms = arq_end_ms - arq_start_ms;

                metrics_lock(&g_metrics);
                g_metrics.arq_total_cmds++;
                g_metrics.arq_total_retries += (uint64_t)(attempt - 1);
                uint64_t total_pkts = g_metrics.total_packets + g_metrics.lost_packets;
                double loss_rate = (total_pkts > 0) ? ((double)g_metrics.lost_packets / total_pkts * 100.0) : 0.0;
                double jitter = g_metrics.jitter_ms;
                metrics_unlock(&g_metrics);

                log_arq_event(arq_end_ms, cmd_name, command, attempt, total_rtt_ms, "REJECTED", loss_rate, jitter);
                printf("[ARQ REJECTED] UAV đã nhận lệnh '%s' nhưng từ chối (MAV_RESULT=%d, lý do an toàn/trạng thái) sau %.1f ms -> Không gửi lại!\n",
                       cmd_name, ack_result, total_rtt_ms);
                return false;
            }
        } else {
            if (attempt < max_retries) {
                printf("[ARQ TIMEOUT] Mất gói / Timeout (%d ms) khi chờ ACK '%s'. Đang gửi lại lần %d/%d...\n",
                       timeout_ms, cmd_name, attempt + 1, max_retries);
            } else {
                printf("[ARQ TIMEOUT] Mất gói / Timeout (%d ms) ở lần thử cuối %d/%d cho '%s'.\n",
                       timeout_ms, attempt, max_retries, cmd_name);
            }
        }
    }

    double arq_end_ms = get_high_res_time_ms(g_perf_freq);
    double total_rtt_ms = arq_end_ms - arq_start_ms;

    metrics_lock(&g_metrics);
    g_metrics.arq_total_cmds++;
    g_metrics.arq_total_retries += (uint64_t)(max_retries - 1);
    uint64_t total_pkts = g_metrics.total_packets + g_metrics.lost_packets;
    double loss_rate = (total_pkts > 0) ? ((double)g_metrics.lost_packets / total_pkts * 100.0) : 0.0;
    double jitter = g_metrics.jitter_ms;
    metrics_unlock(&g_metrics);

    log_arq_event(arq_end_ms, cmd_name, command, max_retries, total_rtt_ms, "FAILED", loss_rate, jitter);
    printf("[-] [ARQ FAILED] Lệnh '%s' thất bại sau %d lần thử lại (Network Congestion / Packet Loss)!\n",
           cmd_name, max_retries);
    return false;
}

// Chuyển chế độ bay với thông báo
static void set_flight_mode(uint32_t mode, const char *mode_name) {
    uint8_t tx_buf[BUFFER_SIZE];
    metrics_lock(&g_metrics);
    uint8_t sys_id = g_metrics.target_system;
    metrics_unlock(&g_metrics);

    int len = mavlink_build_set_mode(tx_buf, g_tx_seq++, sys_id, mode);
    send_uav_packet(tx_buf, len);
    printf("[CHẾ ĐỘ] Đã gửi lệnh chuyển sang chế độ: %s (Mode %d)\n", mode_name, mode);
}

// In bảng hiệu năng mạng và trạng thái UAV tức thời
static void print_current_stats(void) {
    metrics_lock(&g_metrics);
    double throughput_kbps = (g_metrics.interval_bytes * 8.0) / 1000.0;
    uint64_t total = g_metrics.total_packets + g_metrics.lost_packets;
    double loss_rate = (total > 0) ? ((double)g_metrics.lost_packets / total * 100.0) : 0.0;
    double jitter = g_metrics.jitter_ms;
    double alt = g_metrics.current_alt;
    double lat = g_metrics.current_lat;
    double lon = g_metrics.current_lon;
    int battery = g_metrics.battery_remaining;
    uint64_t lost_pkts = g_metrics.lost_packets;
    uint64_t total_pkts = g_metrics.total_packets;
    uint64_t crc_errs = g_metrics.crc_errors;
    uint64_t arq_cmds = g_metrics.arq_total_cmds;
    uint64_t arq_ok = g_metrics.arq_success_cmds;
    uint64_t arq_retries = g_metrics.arq_total_retries;
    double arq_avg_rtt = (arq_ok > 0) ? (g_metrics.arq_total_rtt_ms / (double)arq_ok) : 0.0;
    double arq_rate = (arq_cmds > 0) ? ((double)arq_ok / (double)arq_cmds * 100.0) : 100.0;
    LinkState state = g_metrics.link_state;
    metrics_unlock(&g_metrics);

    const char *state_desc = (state == LINK_OK) ? "[OK] ỔN ĐỊNH" :
                             (state == LINK_TIMEOUT_LOST) ? "[!] MẤT KẾT NỐI (TIMEOUT)" : "[...] ĐANG CHỜ";

    printf("\n+--------------------------------------------------------------------------+\n");
    printf("|                    BÁO CÁO HIỆU NĂNG MẠNG & THÔNG SỐ UAV                 |\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("| Tốc độ mạng (Throughput): %7.2f Kbps  | Tổng số gói đã nhận: %9llu |\n", throughput_kbps, total_pkts);
    printf("| Tỷ lệ mất gói (Packet Loss): %6.2f%%      | Số gói tin bị mất:   %9llu |\n", loss_rate, lost_pkts);
    printf("| Jitter mạng (IETF RFC 3550): %6.2f ms    | Lỗi CRC-16 (RX):     %9llu |\n", jitter, crc_errs);
    printf("| Trạng thái mạng: %-25s | Cổng UDP: %-19d|\n", state_desc, UDP_PORT);
    printf("+--------------------------------------------------------------------------+\n");
    printf("| ARQ Lệnh: %4llu (Thành công: %5.1f%%)      | Tổng Retry: %5llu | RTT: %5.1fms |\n",
           arq_cmds, arq_rate, arq_retries, arq_avg_rtt);
    printf("| Độ cao hiện tại: %6.2f mét             | Pin: %3d%%                         |\n", alt, battery);
    printf("| Tọa độ GPS: (%10.6f, %10.6f)                                      |\n", lat, lon);
    printf("+--------------------------------------------------------------------------+\n\n");
}

// In menu trợ giúp
static void print_help_menu(void) {
    printf("\n========================================================================\n");
    printf("   DANH SÁCH LỆNH ĐIỀU KHIỂN UAV & GIÁM SÁT MẠNG (INTERACTIVE GCS)       \n");
    printf("========================================================================\n");
    printf("  1 | arm              : Mở khóa động cơ (ARM)\n");
    printf("  2 | takeoff [alt]    : Cất cánh (ví dụ: 'takeoff 10' hoặc chỉ cần gõ 2)\n");
    printf("  3 | goto [dist]      : Bay tới Waypoint (ví dụ: 'goto 50' hoặc gõ 3)\n");
    printf("  4 | rtl              : Quay về điểm xuất phát và hạ cánh (RTL)\n");
    printf("  5 | land             : Hạ cánh tại chỗ khẩn cấp (LAND)\n");
    printf("  6 | stats            : Xem bảng thống kê mạng (Loss, Jitter, Kbps) & UAV\n");
    printf("  7 | monitor [on/off] : Bật/tắt in chỉ số mạng tự động mỗi 2 giây\n");
    printf("  8 | auto             : Chạy toàn bộ hành trình tự động một mạch\n");
    printf("  ? | help             : Hiển thị lại bảng menu này\n");
    printf("  0 | exit             : Thoát chương trình\n");
    printf("========================================================================\n\n");
}

// --- LUỒNG 1: THU THẬP TELEMETRY & TÍNH HIỆU NĂNG MẠNG ---
DWORD WINAPI NetworkMonitorThread(LPVOID lpParam) {
    uint8_t rx_buf[BUFFER_SIZE];
    struct sockaddr_in from_addr;
    int from_len = sizeof(from_addr);

    while (g_metrics.running) {
        int bytes = recvfrom(g_sock, (char *)rx_buf, BUFFER_SIZE, 0,
                             (struct sockaddr *)&from_addr, &from_len);
        double current_time = get_high_res_time_ms(g_perf_freq);

        if (bytes > 0) {
            metrics_lock(&g_metrics);

            g_metrics.total_bytes += bytes;
            g_metrics.interval_bytes += bytes;
            g_metrics.total_packets++;
            g_metrics.interval_packets++;

            if (bytes > 10 && rx_buf[0] == MAVLINK_V2_MAGIC) {
                bool crc_checked = false;
                if (!mavlink_verify_crc_v2(rx_buf, bytes, &crc_checked)) {
                    g_metrics.crc_errors++;
                    metrics_unlock(&g_metrics);
                    continue;
                }

                uint8_t payload_len = rx_buf[1];
                uint8_t seq = rx_buf[4];
                uint8_t sys_id = rx_buf[5];
                uint8_t comp_id = rx_buf[6];
                uint32_t msg_id = (uint32_t)rx_buf[7] | ((uint32_t)rx_buf[8] << 8) | ((uint32_t)rx_buf[9] << 16);

                update_jitter(&g_metrics, current_time);
                update_sequence(&g_metrics, seq);

                const uint8_t *payload = rx_buf + 10;

                // Heartbeat
                if (msg_id == MAVLINK_MSG_ID_HEARTBEAT) {
                    g_metrics.last_heartbeat_time_ms = current_time;
                    if (!g_metrics.heartbeat_received) {
                        g_metrics.target_system = sys_id;
                        g_metrics.target_component = comp_id;
                        memcpy(&g_uav_addr, &from_addr, sizeof(from_addr));
                        g_metrics.heartbeat_received = true;
                        g_metrics.link_state = LINK_OK;
                    }
                }
                // COMMAND_ACK
                else if (msg_id == MAVLINK_MSG_ID_COMMAND_ACK && payload_len >= 3) {
                    uint16_t ack_cmd = *(const uint16_t *)payload;
                    uint8_t ack_res = payload[2];
                    g_metrics.last_ack_command = ack_cmd;
                    g_metrics.last_ack_result = ack_res;
                    g_metrics.ack_updated = true;
                }
                // GPS & Altitude
                else if (msg_id == MAVLINK_MSG_ID_GLOBAL_POSITION_INT && payload_len >= 20) {
                    int32_t lat_raw = *(const int32_t *)(payload + 4);
                    int32_t lon_raw = *(const int32_t *)(payload + 8);
                    int32_t rel_alt_raw = *(const int32_t *)(payload + 16);

                    g_metrics.current_lat = (double)lat_raw / 1e7;
                    g_metrics.current_lon = (double)lon_raw / 1e7;
                    g_metrics.current_alt = (double)rel_alt_raw / 1000.0;

                    if (!g_home_set && g_metrics.current_lat != 0.0) {
                        g_home_lat = g_metrics.current_lat;
                        g_home_lon = g_metrics.current_lon;
                        g_home_set = true;
                    }
                }
                // Battery
                else if (msg_id == MAVLINK_MSG_ID_SYS_STATUS && payload_len >= 31) {
                    int8_t batt_raw = (int8_t)payload[30];
                    if (batt_raw >= 1 && batt_raw <= 100) {
                        g_metrics.battery_valid = true;
                        g_metrics.battery_remaining = (uint8_t)batt_raw;
                    } else if (batt_raw == 0 && g_metrics.battery_valid) {
                        g_metrics.battery_remaining = 0;
                    }
                }
            }

            metrics_unlock(&g_metrics);
        } else {
            Sleep(5);
        }
    }
    return 0;
}

// --- LUỒNG 2: GIÁM SÁT MẠNG, PHÁT HIỆN TIMEOUT (FAILSAFE) & GHI LOG CSV ---
DWORD WINAPI DisplayStatsThread(LPVOID lpParam) {
    double start_time = get_high_res_time_ms(g_perf_freq);
    double last_check = start_time;
    int display_counter = 0;

    FILE *fp_csv = fopen(CSV_FILE_PATH, "w");
    if (fp_csv) {
        fprintf(fp_csv, "timestamp_ms,elapsed_sec,throughput_kbps,packet_rate_pps,total_packets,lost_packets,loss_rate_percent,jitter_ms,altitude_m,battery_percent,link_status\n");
        fflush(fp_csv);
    }

    while (g_metrics.running) {
        Sleep(1000);

        double now = get_high_res_time_ms(g_perf_freq);
        double elapsed_sec = (now - last_check) / 1000.0;
        double total_elapsed_sec = (now - start_time) / 1000.0;
        last_check = now;

        metrics_lock(&g_metrics);

        double throughput_kbps = (g_metrics.interval_bytes * 8.0) / (elapsed_sec * 1000.0);
        double pps = g_metrics.interval_packets / elapsed_sec;
        g_metrics.interval_bytes = 0;
        g_metrics.interval_packets = 0;

        uint64_t total = g_metrics.total_packets + g_metrics.lost_packets;
        double loss_rate = (total > 0) ? ((double)g_metrics.lost_packets / total * 100.0) : 0.0;
        double jitter = g_metrics.jitter_ms;
        double alt = g_metrics.current_alt;
        double lat = g_metrics.current_lat;
        double lon = g_metrics.current_lon;
        int battery = g_metrics.battery_remaining;
        uint64_t lost_pkts = g_metrics.lost_packets;
        uint64_t total_pkts = g_metrics.total_packets;

        char status_str[80];
        double time_since_start = (now - start_time) / 1000.0;

        if (!g_metrics.heartbeat_received) {
            if (time_since_start > 3.0) {
                snprintf(status_str, sizeof(status_str), "[!] KHÔNG CÓ TÍN HIỆU (Kiểm tra Drop/SITL)");
            } else {
                snprintf(status_str, sizeof(status_str), "[...] ĐANG CHỜ TÍN HIỆU BAN ĐẦU");
            }
        } else {
            double time_since_hb = (now - g_metrics.last_heartbeat_time_ms) / 1000.0;
            if (time_since_hb > 3.0) {
                if (g_metrics.link_state != LINK_TIMEOUT_LOST) {
                    g_metrics.link_state = LINK_TIMEOUT_LOST;
                    g_metrics.link_lost = true;
                    printf("\n\n========================================================================\n");
                    printf(" [FAILSAFE CẢNH BÁO] MẤT KẾT NỐI MẠNG! (Timeout Heartbeat: %.1fs > 3.0s)\n", time_since_hb);
                    printf(" Gói tin UDP bị mất hoàn toàn (Packet Loss 100%% hoặc đứt link)!\n");
                    printf(" Khuyến nghị: Kích hoạt chế độ an toàn Failsafe RTL hoặc Hover!\n");
                    printf("========================================================================\n");
                    printf("\n[UAV-CLI] > ");
                    fflush(stdout);
                }
                snprintf(status_str, sizeof(status_str), "[!] MẤT KẾT NỐI (TIMEOUT %.1fs)", time_since_hb);
            } else {
                if (g_metrics.link_state == LINK_TIMEOUT_LOST) {
                    printf("\n\n[LINK RESTORED] >>> ĐÃ KHÔI PHỤC KẾT NỐI TÍN HIỆU TỪ UAV! <<<\n");
                    printf("\n[UAV-CLI] > ");
                    fflush(stdout);
                }
                g_metrics.link_state = LINK_OK;
                g_metrics.link_lost = false;
                snprintf(status_str, sizeof(status_str), "[OK] ỔN ĐỊNH");
            }
        }

        bool trigger_batt_failsafe = false;
        if (g_metrics.battery_valid) {
            if (!g_metrics.battery_failsafe_triggered && battery < 20 && alt > 1.0) {
                g_metrics.battery_failsafe_triggered = true;
                trigger_batt_failsafe = true;
            } else if (battery >= 25) {
                g_metrics.battery_failsafe_triggered = false;
            }
        }

        metrics_unlock(&g_metrics);

        if (trigger_batt_failsafe) {
            printf("\n\n========================================================================\n");
            printf(" [FAILSAFE PIN YẾU] Pin chỉ còn %d%% (< 20%%) khi đang ở độ cao %.2fm!\n", battery, alt);
            printf(" Tự động gửi lệnh chuyển chế độ LAND để hạ cánh khẩn cấp!\n");
            printf("========================================================================\n");
            set_flight_mode(COPTER_MODE_LAND, "LAND (LOW BATTERY FAILSAFE)");
            printf("\n[UAV-CLI] > ");
            fflush(stdout);
        }

        // Ghi dữ liệu vào CSV
        if (fp_csv) {
            fprintf(fp_csv, "%.0f,%.2f,%.2f,%.1f,%llu,%llu,%.2f,%.2f,%.2f,%d,%s\n",
                    now, total_elapsed_sec, throughput_kbps, pps,
                    total_pkts, lost_pkts, loss_rate, jitter,
                    alt, battery, status_str);
            fflush(fp_csv);
        }

        // In ra màn hình định kỳ nếu người dùng bật chế độ monitor
        if (g_show_periodic_stats) {
            display_counter++;
            if (display_counter >= 2) {
                display_counter = 0;
                printf("\n[STREAM-STATS] Băng thông: %5.2f Kbps | Mất gói: %4.2f%% | Jitter: %4.2f ms | Độ cao: %5.2fm | Mạng: %s\n[UAV-CLI] > ",
                       throughput_kbps, loss_rate, jitter, alt, status_str);
                fflush(stdout);
            }
        }
    }

    if (fp_csv) {
        fclose(fp_csv);
    }
    return 0;
}

// Xử lý lệnh cất cánh
static void execute_takeoff(float target_alt) {
    double cur_alt = get_safe_altitude();
    if (cur_alt > 1.0) {
        printf("[-] [AN TOÀN] UAV đang bay ở độ cao %.2f m (> 1.0 m), không thể thực hiện TAKEOFF lại!\n", cur_alt);
        return;
    }

    if (target_alt <= 0.5f) target_alt = 10.0f;

    set_flight_mode(COPTER_MODE_GUIDED, "GUIDED");
    Sleep(1000);

    bool arm_ok = send_command_long_with_arq(MAV_CMD_COMPONENT_ARM_DISARM, "ARM MOTOR",
                                             1.0f, 0, 0, 0, 0, 0, 0, 4, 800);
    if (!arm_ok) {
        printf("[-] Cảnh báo: Lệnh ARM chưa nhận được ACK chấp thuận từ UAV!\n");
    }

    Sleep(1000);
    bool tk_ok = send_command_long_with_arq(MAV_CMD_NAV_TAKEOFF, "TAKEOFF",
                                            0, 0, 0, 0, 0, 0, target_alt, 4, 800);
    if (tk_ok) {
        printf("[*] Đang cất cánh lên độ cao %.1fm...\n", target_alt);
        int wait_ticks = 0;
        while (get_safe_altitude() < (target_alt - 0.5f)) {
            if (is_battery_failsafe_active()) {
                printf("\n[!] Hủy chờ cất cánh do kích hoạt Battery Failsafe (Pin yếu)!\n");
                return;
            }
            if (++wait_ticks >= 75) { // Tối đa 30 giây (75 * 400ms)
                printf("\n[-] Quá thời gian chờ cất cánh (30s). Vui lòng kiểm tra trạng thái UAV!\n");
                return;
            }
            printf("\r[*] Độ cao hiện tại: %5.2f m / mục tiêu %.1f m...", get_safe_altitude(), target_alt);
            fflush(stdout);
            Sleep(400);
        }
        printf("\n[+] ĐÃ ĐẠT ĐỘ CAO %.1f MÉT THÀNH CÔNG! UAV đang treo (hover) ổn định.\n", target_alt);
    }
}

// Xử lý lệnh bay tới Waypoint
static void execute_goto(double dist_meters) {
    if (dist_meters <= 0.0) dist_meters = 50.0;

    double cur_lat = 0.0, cur_lon = 0.0;
    get_safe_position(&cur_lat, &cur_lon);

    if (cur_lat == 0.0 && cur_lon == 0.0) {
        printf("[-] Chưa có tọa độ GPS hợp lệ từ UAV!\n");
        return;
    }

    // Tọa độ mục tiêu dịch chuyển về phía Bắc X mét
    double target_lat = cur_lat + (dist_meters / 111319.5);
    double target_lon = cur_lon;
    float current_alt = (float)get_safe_altitude();
    if (current_alt < 2.0f) current_alt = 10.0f;

    metrics_lock(&g_metrics);
    uint8_t sys_id = g_metrics.target_system;
    uint8_t comp_id = g_metrics.target_component;
    metrics_unlock(&g_metrics);

    uint8_t tx_buf[BUFFER_SIZE];
    int len = mavlink_build_mission_item_int(tx_buf, g_tx_seq++, sys_id, comp_id,
                                         target_lat, target_lon, current_alt);
    send_uav_packet(tx_buf, len);

    printf("\n>>> [LỆNH MẠNG] Gửi WAYPOINT: Bay %.1fm về hướng Bắc (Lat: %.6f, Lon: %.6f, Alt: %.1fm)\n",
           dist_meters, target_lat, target_lon, current_alt);

    for (int i = 0; i < 40; i++) {
        if (is_battery_failsafe_active()) {
            printf("\n[!] Hủy bay tới Waypoint do kích hoạt Battery Failsafe (Pin yếu)!\n");
            return;
        }
        double now_lat = 0.0, now_lon = 0.0;
        get_safe_position(&now_lat, &now_lon);

        double d_lat = now_lat - target_lat;
        double d_lon = now_lon - target_lon;
        double dist = sqrt(d_lat * d_lat + d_lon * d_lon) * 111319.5;

        printf("\r[*] Đang bay tới Waypoint: khoảng cách còn lại %5.1f m | Độ cao: %5.1f m...", dist, get_safe_altitude());
        fflush(stdout);

        if (dist < 3.0) {
            printf("\n[+] ĐÃ ĐẾN VỊ TRÍ MỤC TIÊU (Khoảng cách còn lại: %.1fm)! UAV đang dừng giữ vị trí.\n", dist);
            break;
        }
        Sleep(800);
    }
}

// Xử lý lệnh RTL
static void execute_rtl(void) {
    printf("\n[*] Đang kích hoạt RTL (Return to Launch)...\n");
    set_flight_mode(COPTER_MODE_RTL, "RTL (Return to Launch)");

    int stall_count = 0;
    double last_alt = get_safe_altitude();

    while (get_safe_altitude() > 0.3) {
        double cur_alt = get_safe_altitude();
        printf("\r[*] UAV đang quay về và hạ cánh: độ cao hiện tại %6.2f m...", cur_alt);
        fflush(stdout);

        if (fabs(cur_alt - last_alt) < 0.05) {
            stall_count++;
            if (stall_count >= 6) { // ~4.8s nếu UAV chưa phản hồi thì gửi lại
                stall_count = 0;
                set_flight_mode(COPTER_MODE_RTL, "RTL");
            }
        } else {
            stall_count = 0;
        }
        last_alt = cur_alt;
        Sleep(800);
    }
    printf("\n[+] UAV ĐÃ TIẾP ĐẤT AN TOÀN VÀ TỰ ĐỘNG KHÓA ĐỘNG CƠ (DISARMED)!\n");
}

// Xử lý lệnh LAND (Hạ cánh tại chỗ với Stop-and-Wait ARQ)
static void execute_land(void) {
    printf("\n[*] Đang gửi lệnh hạ cánh khẩn cấp tại chỗ (LAND)...\n");

    // 1. Gửi lệnh chuyển chế độ sang LAND
    set_flight_mode(COPTER_MODE_LAND, "LAND");
    Sleep(300);

    // 2. Gửi lệnh chuẩn MAV_CMD_NAV_LAND (Command 21) có xác nhận Stop-and-Wait ARQ
    bool land_ok = send_command_long_with_arq(MAV_CMD_NAV_LAND, "NAV_LAND",
                                              0, 0, 0, 0, 0, 0, 0, 4, 800);
    if (!land_ok) {
        printf("[-] Cảnh báo: Lệnh NAV_LAND chưa nhận ACK, tiếp tục gửi lại chế độ LAND...\n");
        set_flight_mode(COPTER_MODE_LAND, "LAND");
    }

    printf("[*] UAV đang bắt đầu giảm độ cao và hạ cánh...\n");

    int stall_count = 0;
    double last_alt = get_safe_altitude();

    while (get_safe_altitude() > 0.3) {
        double cur_alt = get_safe_altitude();
        printf("\r[*] UAV đang hạ cánh: độ cao hiện tại %6.2f m...", cur_alt);
        fflush(stdout);

        // Nếu sau 4.8s độ cao chưa suy giảm, tự động kích hoạt lại mode LAND
        if (fabs(cur_alt - last_alt) < 0.05) {
            stall_count++;
            if (stall_count >= 6) {
                stall_count = 0;
                set_flight_mode(COPTER_MODE_LAND, "LAND");
            }
        } else {
            stall_count = 0;
        }
        last_alt = cur_alt;
        Sleep(800);
    }
    printf("\n[+] UAV ĐÃ TIẾP ĐẤT TẠI CHỖ THÀNH CÔNG VÀ KHÓA ĐỘNG CƠ!\n");
}

// --- CHƯƠNG TRÌNH CHÍNH (INTERACTIVE GCS CLI) ---
int main(void) {
    SetConsoleOutputCP(CP_UTF8);

    WSADATA wsaData;
    struct sockaddr_in bind_addr;

    QueryPerformanceFrequency(&g_perf_freq);
    metrics_init(&g_metrics);
    init_arq_csv();

    printf("========================================================================\n");
    printf("  TRẠM ĐIỀU KHIỂN UAV & GIÁM SÁT MẠNG TƯƠNG TÁC (INTERACTIVE GCS CLI)  \n");
    printf("  Hỗ trợ: Giao thức MAVLink v2 | Stop-and-Wait ARQ | Đo hiệu năng mạng \n");
    printf("========================================================================\n\n");

    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        printf("[-] WSAStartup thất bại!\n");
        return 1;
    }

    g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_sock == INVALID_SOCKET) {
        WSACleanup();
        return 1;
    }

    DWORD timeout = 200;
    setsockopt(g_sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout));

    BOOL reuse = TRUE;
    setsockopt(g_sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse));

    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    bind_addr.sin_port = htons(UDP_PORT);

    if (bind(g_sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) == SOCKET_ERROR) {
        printf("[-] Không thể bind vào port %d. Mã lỗi: %d\n", UDP_PORT, WSAGetLastError());
        closesocket(g_sock);
        WSACleanup();
        return 1;
    }

    printf("[*] Khởi tạo luồng mạng thu thập Telemetry và luồng đo đạc hiệu năng...\n");
    HANDLE hThread1 = CreateThread(NULL, 0, NetworkMonitorThread, NULL, 0, NULL);
    HANDLE hThread2 = CreateThread(NULL, 0, DisplayStatsThread, NULL, 0, NULL);

    printf("[*] Đang lắng nghe Heartbeat từ UAV trên cổng UDP %d...\n", UDP_PORT);
    int wait_hb_ms = 0;
    while (true) {
        metrics_lock(&g_metrics);
        bool connected = g_metrics.heartbeat_received;
        metrics_unlock(&g_metrics);
        if (connected) break;
        Sleep(200);
        wait_hb_ms += 200;
        if (wait_hb_ms % 4000 == 0) {
            printf("[CHỜ KẾT NỐI] Chưa nhận được gói tin Heartbeat (%d giây)...\n", wait_hb_ms / 1000);
            printf("              * Nếu đang bật Clumsy Drop 100%%, hãy tạm bấm 'Stop' trên Clumsy để thiết lập kết nối ban đầu!\n");
        }
    }

    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(g_uav_addr.sin_addr), ip_str, INET_ADDRSTRLEN);

    metrics_lock(&g_metrics);
    uint8_t sys_id = g_metrics.target_system;
    uint8_t comp_id = g_metrics.target_component;
    metrics_unlock(&g_metrics);

    printf("\n[+] ĐÃ KẾT NỐI TỚI UAV TẠI %s:%d! (System ID: %d, Component ID: %d)\n",
           ip_str, ntohs(g_uav_addr.sin_port), sys_id, comp_id);
    printf("[*] Đã bắt đầu ghi log mạng vào file '%s'.\n", CSV_FILE_PATH);

    print_help_menu();

    // VÒNG LẶP DÒNG LỆNH TƯƠNG TÁC (INTERACTIVE SHELL)
    char cmd_line[128];
    while (g_metrics.running) {
        printf("[UAV-CLI] > ");
        fflush(stdout);

        if (!fgets(cmd_line, sizeof(cmd_line), stdin)) {
            break;
        }

        // Bỏ ký tự xuống dòng
        cmd_line[strcspn(cmd_line, "\r\n")] = 0;

        // Bỏ qua dòng trống
        if (strlen(cmd_line) == 0) continue;

        char command[64] = {0};
        char param[64] = {0};
        sscanf(cmd_line, "%s %s", command, param);

        // 1. ARM
        if (strcmp(command, "arm") == 0 || strcmp(command, "1") == 0) {
            set_flight_mode(COPTER_MODE_GUIDED, "GUIDED");
            Sleep(800);
            send_command_long_with_arq(MAV_CMD_COMPONENT_ARM_DISARM, "ARM MOTOR",
                                       1.0f, 0, 0, 0, 0, 0, 0, 4, 800);
        }
        // 2. TAKEOFF
        else if (strcmp(command, "takeoff") == 0 || strcmp(command, "2") == 0) {
            float alt = (float)atof(param);
            if (alt <= 0.0f) alt = 10.0f;
            execute_takeoff(alt);
        }
        // 3. GOTO WAYPOINT
        else if (strcmp(command, "goto") == 0 || strcmp(command, "3") == 0) {
            double dist = atof(param);
            if (dist <= 0.0) dist = 50.0;
            execute_goto(dist);
        }
        // 4. RTL
        else if (strcmp(command, "rtl") == 0 || strcmp(command, "4") == 0) {
            execute_rtl();
        }
        // 5. LAND
        else if (strcmp(command, "land") == 0 || strcmp(command, "5") == 0) {
            execute_land();
        }
        // 6. STATS
        else if (strcmp(command, "stats") == 0 || strcmp(command, "6") == 0) {
            print_current_stats();
        }
        // 7. MONITOR
        else if (strcmp(command, "monitor") == 0 || strcmp(command, "7") == 0) {
            if (strcmp(param, "off") == 0) {
                g_show_periodic_stats = false;
                printf("[*] Đã TẮT in thống kê tự động.\n");
            } else {
                g_show_periodic_stats = true;
                printf("[*] Đã BẬT in thống kê tự động mỗi 2 giây.\n");
            }
        }
        // 8. AUTO MISSION
        else if (strcmp(command, "auto") == 0 || strcmp(command, "8") == 0) {
            printf("\n[*] BẮT ĐẦU CHẠY TOÀN BỘ KỊCH BẢN TỰ ĐỘNG...\n");
            execute_takeoff(10.0f);
            if (is_battery_failsafe_active() || get_safe_altitude() < 2.0) {
                printf("[-] Dừng kịch bản tự động do UAV chưa đạt độ cao an toàn hoặc đang Failsafe!\n");
                continue;
            }
            Sleep(2000);
            execute_goto(50.0);
            if (is_battery_failsafe_active()) {
                printf("[-] Dừng kịch bản tự động do Battery Failsafe!\n");
                continue;
            }
            Sleep(2000);
            execute_rtl();
        }
        // HELP
        else if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
            print_help_menu();
        }
        // 0. EXIT
        else if (strcmp(command, "exit") == 0 || strcmp(command, "quit") == 0 || strcmp(command, "0") == 0) {
            if (get_safe_altitude() > 1.0) {
                printf("[CẢNH BÁO] UAV vẫn đang ở độ cao %.1fm! Bạn có chắc muốn thoát mà không hạ cánh? (y/n): ", get_safe_altitude());
                fflush(stdout);
                char confirm[10];
                if (fgets(confirm, sizeof(confirm), stdin) && (confirm[0] == 'y' || confirm[0] == 'Y')) {
                    break;
                } else {
                    printf("[*] Hủy lệnh thoát. Hãy gõ 'rtl' hoặc '4' để hạ cánh an toàn trước.\n");
                    continue;
                }
            }
            break;
        }
        else {
            printf("[-] Lệnh không hợp lệ: '%s'. Gõ 'help' hoặc '?' để xem danh sách lệnh.\n", command);
        }
    }

    // Dọn dẹp tài nguyên
    metrics_lock(&g_metrics);
    g_metrics.running = false;
    metrics_unlock(&g_metrics);

    WaitForSingleObject(hThread1, 2000);
    WaitForSingleObject(hThread2, 2000);
    CloseHandle(hThread1);
    CloseHandle(hThread2);

    metrics_cleanup(&g_metrics);
    closesocket(g_sock);
    WSACleanup();

    printf("\n[*] Đã kết thúc phiên làm việc GCS an toàn. Dữ liệu mạng đã lưu tại '%s'.\n", CSV_FILE_PATH);
    return 0;
}

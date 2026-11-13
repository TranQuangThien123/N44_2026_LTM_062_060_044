#ifndef NETWORK_METRICS_H
#define NETWORK_METRICS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <windows.h>

typedef enum {
    LINK_CONNECTING = 0,    // Đang chờ tín hiệu ban đầu từ UAV
    LINK_OK = 1,            // Kết nối tốt, nhận heartbeat đều đặn
    LINK_TIMEOUT_LOST = 2   // Bị đứt mạng, quá 3 giây không có heartbeat
} LinkState;

typedef struct {
    uint64_t total_bytes;
    uint64_t interval_bytes;
    uint64_t total_packets;
    uint64_t interval_packets;
    uint64_t lost_packets;
    uint64_t crc_errors;
    int last_seq;
    double last_packet_time_ms;
    double last_inter_arrival_ms;
    double jitter_ms;

    // Telemetry
    double current_lat;
    double current_lon;
    double current_alt;
    float current_vx;
    float current_vy;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float battery_voltage;
    uint8_t battery_remaining;
    bool battery_valid;
    bool battery_failsafe_triggered;

    // UAV Targets & Keep-Alive (Heartbeat)
    uint8_t target_system;
    uint8_t target_component;
    bool heartbeat_received;
    double last_heartbeat_time_ms;
    bool link_lost;
    LinkState link_state;

    // Stop-and-Wait ARQ Command Acknowledgment
    uint16_t last_ack_command;
    uint8_t last_ack_result;
    bool ack_updated;

    // Thống kê hiệu năng ARQ
    uint64_t arq_total_cmds;
    uint64_t arq_success_cmds;
    uint64_t arq_total_retries;
    double arq_total_rtt_ms;

    // Đồng bộ đa luồng (Thread Synchronization)
    CRITICAL_SECTION cs;

    volatile bool running;
} NetworkMetrics;

// Khởi tạo NetworkMetrics và Critical Section
static inline void metrics_init(NetworkMetrics *m) {
    memset(m, 0, sizeof(NetworkMetrics));
    m->last_seq = -1;
    m->running = true;
    m->link_state = LINK_CONNECTING;
    InitializeCriticalSection(&m->cs);
}

// Giải phóng tài nguyên đồng bộ
static inline void metrics_cleanup(NetworkMetrics *m) {
    DeleteCriticalSection(&m->cs);
}

// Khóa truy cập đa luồng an toàn
static inline void metrics_lock(NetworkMetrics *m) {
    EnterCriticalSection(&m->cs);
}

// Mở khóa truy cập đa luồng
static inline void metrics_unlock(NetworkMetrics *m) {
    LeaveCriticalSection(&m->cs);
}

// Lấy thời gian mili-giây độ chính xác cao
static inline double get_high_res_time_ms(LARGE_INTEGER freq) {
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    return ((double)now.QuadPart / (double)freq.QuadPart) * 1000.0;
}

// Cập nhật Jitter theo biến thiên thời gian đến giữa các gói liên tiếp (IPDV / RFC 3550)
static inline void update_jitter(NetworkMetrics *m, double current_time_ms) {
    const double alpha = 0.125;
    if (m->last_packet_time_ms > 0.0) {
        double inter_arrival = current_time_ms - m->last_packet_time_ms;
        if (m->last_inter_arrival_ms > 0.0) {
            double d = fabs(inter_arrival - m->last_inter_arrival_ms);
            m->jitter_ms += alpha * (d - m->jitter_ms);
        }
        m->last_inter_arrival_ms = inter_arrival;
    }
    m->last_packet_time_ms = current_time_ms;
}

// Cập nhật Sequence Number để đo Packet Loss
static inline void update_sequence(NetworkMetrics *m, uint8_t seq) {
    if (m->last_seq != -1) {
        int expected_seq = (m->last_seq + 1) % 256;
        if (seq != expected_seq) {
            int lost = (seq - expected_seq + 256) % 256;
            m->lost_packets += lost;
        }
    }
    m->last_seq = seq;
}

#endif // NETWORK_METRICS_H

#ifndef MAVLINK_LITE_H
#define MAVLINK_LITE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// --- MAVLINK V2 CONSTANTS ---
#define MAVLINK_V2_MAGIC 0xFD
#define MAVLINK_V1_MAGIC 0xFE

// Message IDs
#define MAVLINK_MSG_ID_HEARTBEAT            0
#define MAVLINK_MSG_ID_SYS_STATUS           1
#define MAVLINK_MSG_ID_SET_MODE             11
#define MAVLINK_MSG_ID_ATTITUDE             30
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT  33
#define MAVLINK_MSG_ID_MISSION_ITEM_INT     73
#define MAVLINK_MSG_ID_COMMAND_LONG         76
#define MAVLINK_MSG_ID_COMMAND_ACK          77

// CRC Extra (Specific to message ID in MAVLink v2)
#define CRC_EXTRA_HEARTBEAT                 50
#define CRC_EXTRA_SYS_STATUS                124
#define CRC_EXTRA_SET_MODE                  89
#define CRC_EXTRA_ATTITUDE                  39
#define CRC_EXTRA_GLOBAL_POSITION_INT       104
#define CRC_EXTRA_MISSION_ITEM_INT          38
#define CRC_EXTRA_COMMAND_LONG              152
#define CRC_EXTRA_COMMAND_ACK               143

// Commands (MAV_CMD)
#define MAV_CMD_NAV_WAYPOINT                16
#define MAV_CMD_NAV_LAND                    21
#define MAV_CMD_NAV_TAKEOFF                 22
#define MAV_CMD_DO_SET_MODE                 176
#define MAV_CMD_COMPONENT_ARM_DISARM        400

// Command ACK Results (MAV_RESULT)
#define MAV_RESULT_ACCEPTED                 0
#define MAV_RESULT_TEMPORARILY_REJECTED     1
#define MAV_RESULT_DENIED                   2
#define MAV_RESULT_UNSUPPORTED              3
#define MAV_RESULT_FAILED                   4
#define MAV_RESULT_IN_PROGRESS              5
#define MAV_RESULT_CANCELLED                6

// ArduCopter Modes
#define COPTER_MODE_STABILIZE               0
#define COPTER_MODE_GUIDED                  4
#define COPTER_MODE_RTL                     6
#define COPTER_MODE_LAND                    9

// Flags & Frames
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED   1
#define MAV_FRAME_GLOBAL_RELATIVE_ALT_INT   3

// --- CRC16-MCRF4XX CALCULATION ---
static inline void crc_accumulate(uint8_t data, uint16_t *crcAccum) {
    uint8_t tmp;
    tmp = data ^ (uint8_t)(*crcAccum & 0xff);
    tmp ^= (tmp << 4);
    *crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

static inline uint16_t crc_calculate(const uint8_t *buffer, size_t length, uint8_t crc_extra) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc_accumulate(buffer[i], &crc);
    }
    crc_accumulate(crc_extra, &crc);
    return crc;
}

// --- PACKET ENCODING FUNCTION ---
// Đóng gói frame MAVLink v2 hoàn chỉnh vào buffer xuất
static inline int mavlink_pack_v2(uint8_t *out_buf,
                                  uint8_t seq,
                                  uint8_t sys_id,
                                  uint8_t comp_id,
                                  uint32_t msg_id,
                                  uint8_t crc_extra,
                                  const uint8_t *payload,
                                  uint8_t payload_len) {
    out_buf[0] = MAVLINK_V2_MAGIC;
    out_buf[1] = payload_len;
    out_buf[2] = 0x00; // incompat_flags
    out_buf[3] = 0x00; // compat_flags
    out_buf[4] = seq;
    out_buf[5] = sys_id;
    out_buf[6] = comp_id;
    out_buf[7] = (uint8_t)(msg_id & 0xFF);
    out_buf[8] = (uint8_t)((msg_id >> 8) & 0xFF);
    out_buf[9] = (uint8_t)((msg_id >> 16) & 0xFF);

    if (payload_len > 0 && payload != NULL) {
        memcpy(&out_buf[10], payload, payload_len);
    }

    // Tính CRC qua Header (từ byte 1) + Payload + CRC_EXTRA
    uint16_t crc = crc_calculate(&out_buf[1], 9 + payload_len, crc_extra);
    out_buf[10 + payload_len] = (uint8_t)(crc & 0xFF);
    out_buf[10 + payload_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

    return 10 + payload_len + 2;
}

// Tra cứu CRC_EXTRA cho các Message ID đã biết
static inline bool mavlink_get_crc_extra(uint32_t msg_id, uint8_t *crc_extra_out) {
    switch (msg_id) {
        case MAVLINK_MSG_ID_HEARTBEAT:           *crc_extra_out = CRC_EXTRA_HEARTBEAT; return true;
        case MAVLINK_MSG_ID_SYS_STATUS:          *crc_extra_out = CRC_EXTRA_SYS_STATUS; return true;
        case MAVLINK_MSG_ID_SET_MODE:            *crc_extra_out = CRC_EXTRA_SET_MODE; return true;
        case MAVLINK_MSG_ID_ATTITUDE:            *crc_extra_out = CRC_EXTRA_ATTITUDE; return true;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: *crc_extra_out = CRC_EXTRA_GLOBAL_POSITION_INT; return true;
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:    *crc_extra_out = CRC_EXTRA_MISSION_ITEM_INT; return true;
        case MAVLINK_MSG_ID_COMMAND_LONG:        *crc_extra_out = CRC_EXTRA_COMMAND_LONG; return true;
        case MAVLINK_MSG_ID_COMMAND_ACK:         *crc_extra_out = CRC_EXTRA_COMMAND_ACK; return true;
        default: return false;
    }
}

// Kiểm tra độ dài frame và CRC-16 X.25 ở chiều nhận (RX)
static inline bool mavlink_verify_crc_v2(const uint8_t *buf, int buf_len, bool *crc_checked_out) {
    if (crc_checked_out) *crc_checked_out = false;
    if (buf == NULL || buf_len < 12 || buf[0] != MAVLINK_V2_MAGIC) {
        return false;
    }

    uint8_t payload_len = buf[1];
    int expected_len = 10 + payload_len + 2;
    if (buf_len < expected_len) {
        return false;
    }

    uint32_t msg_id = (uint32_t)buf[7] | ((uint32_t)buf[8] << 8) | ((uint32_t)buf[9] << 16);
    uint8_t crc_extra = 0;
    if (!mavlink_get_crc_extra(msg_id, &crc_extra)) {
        // Message ID chưa khai báo CRC_EXTRA: vẫn cho qua nếu đủ kích thước frame
        return true;
    }

    if (crc_checked_out) *crc_checked_out = true;
    uint16_t expected_crc = crc_calculate(&buf[1], 9 + payload_len, crc_extra);
    uint16_t actual_crc = (uint16_t)buf[10 + payload_len] | ((uint16_t)buf[11 + payload_len] << 8);
    return expected_crc == actual_crc;
}

// --- COMMAND BUILDERS ---

// Đóng gói lệnh SET_MODE (GUIDED, RTL, ...)
static inline int mavlink_build_set_mode(uint8_t *out_buf, uint8_t seq, uint8_t target_sys, uint32_t custom_mode) {
    #pragma pack(push, 1)
    struct {
        uint32_t custom_mode;
        uint8_t target_system;
        uint8_t base_mode;
    } payload;
    #pragma pack(pop)

    payload.custom_mode = custom_mode;
    payload.target_system = target_sys;
    payload.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    return mavlink_pack_v2(out_buf, seq, 255, 1, MAVLINK_MSG_ID_SET_MODE,
                           CRC_EXTRA_SET_MODE, (const uint8_t *)&payload, sizeof(payload));
}

// Đóng gói lệnh COMMAND_LONG (ARM / DISARM / TAKEOFF)
static inline int mavlink_build_command_long(uint8_t *out_buf, uint8_t seq, uint8_t target_sys, uint8_t target_comp,
                                            uint16_t command, float p1, float p2, float p3, float p4,
                                            float p5, float p6, float p7) {
    #pragma pack(push, 1)
    struct {
        float param1;
        float param2;
        float param3;
        float param4;
        float param5;
        float param6;
        float param7;
        uint16_t command;
        uint8_t target_system;
        uint8_t target_component;
        uint8_t confirmation;
    } payload;
    #pragma pack(pop)

    payload.param1 = p1;
    payload.param2 = p2;
    payload.param3 = p3;
    payload.param4 = p4;
    payload.param5 = p5;
    payload.param6 = p6;
    payload.param7 = p7;
    payload.command = command;
    payload.target_system = target_sys;
    payload.target_component = target_comp;
    payload.confirmation = 0;

    return mavlink_pack_v2(out_buf, seq, 255, 1, MAVLINK_MSG_ID_COMMAND_LONG,
                           CRC_EXTRA_COMMAND_LONG, (const uint8_t *)&payload, sizeof(payload));
}

// Đóng gói lệnh Waypoint MISSION_ITEM_INT
static inline int mavlink_build_mission_item_int(uint8_t *out_buf, uint8_t seq, uint8_t target_sys, uint8_t target_comp,
                                                double lat, double lon, float alt) {
    #pragma pack(push, 1)
    struct {
        float param1;
        float param2;
        float param3;
        float param4;
        int32_t x; // lat * 1e7
        int32_t y; // lon * 1e7
        float z;   // alt (m)
        uint16_t seq;
        uint16_t command;
        uint8_t target_system;
        uint8_t target_component;
        uint8_t frame;
        uint8_t current;
        uint8_t autocontinue;
        uint8_t mission_type;
    } payload;
    #pragma pack(pop)

    payload.param1 = 0;
    payload.param2 = 2.0f; // bán kính chấp nhận 2 mét
    payload.param3 = 0;
    payload.param4 = 0;
    payload.x = (int32_t)(lat * 1e7);
    payload.y = (int32_t)(lon * 1e7);
    payload.z = alt;
    payload.seq = 0;
    payload.command = MAV_CMD_NAV_WAYPOINT;
    payload.target_system = target_sys;
    payload.target_component = target_comp;
    payload.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    payload.current = 2; // Guided mode direct target
    payload.autocontinue = 0;
    payload.mission_type = 0;

    return mavlink_pack_v2(out_buf, seq, 255, 1, MAVLINK_MSG_ID_MISSION_ITEM_INT,
                           CRC_EXTRA_MISSION_ITEM_INT, (const uint8_t *)&payload, sizeof(payload));
}

// Đóng gói gói tin Heartbeat từ GCS
static inline int mavlink_build_heartbeat(uint8_t *out_buf, uint8_t seq) {
    #pragma pack(push, 1)
    struct {
        uint32_t custom_mode;
        uint8_t type;            // MAV_TYPE_GCS = 6
        uint8_t autopilot;       // MAV_AUTOPILOT_INVALID = 8
        uint8_t base_mode;
        uint8_t system_status;   // MAV_STATE_ACTIVE = 4
        uint8_t mavlink_version; // 3
    } payload;
    #pragma pack(pop)

    payload.custom_mode = 0;
    payload.type = 6;
    payload.autopilot = 8;
    payload.base_mode = 0;
    payload.system_status = 4;
    payload.mavlink_version = 3;

    return mavlink_pack_v2(out_buf, seq, 255, 1, MAVLINK_MSG_ID_HEARTBEAT,
                           CRC_EXTRA_HEARTBEAT, (const uint8_t *)&payload, sizeof(payload));
}

#endif // MAVLINK_LITE_H

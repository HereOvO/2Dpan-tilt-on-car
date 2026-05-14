#ifndef GIMBAL_APP_H
#define GIMBAL_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "tim.h"

#define GIMBAL_APP_PROTO_SOF0             0xAAU
#define GIMBAL_APP_PROTO_SOF1             0x55U
#define GIMBAL_APP_PROTO_TAIL             0x0DU
#define GIMBAL_APP_MAX_PAYLOAD_LEN        160U
#define GIMBAL_APP_FRAME_OVERHEAD         13U
#define GIMBAL_APP_MAX_FRAME_LEN          (GIMBAL_APP_FRAME_OVERHEAD + GIMBAL_APP_MAX_PAYLOAD_LEN)
#define GIMBAL_APP_RX_FIFO_LEN            512U

#define GIMBAL_APP_SERVO_PULSE_MIN_US     500U
#define GIMBAL_APP_SERVO_PULSE_MAX_US     2500U
#define GIMBAL_APP_SERVO_TIMER_PSC        167U
#define GIMBAL_APP_SERVO_TIMER_ARR        19999U

#define GIMBAL_APP_FLAG_TARGET_VALID      0x01U
#define GIMBAL_APP_FLAG_ACK_REQUEST       0x02U
#define GIMBAL_APP_FLAG_STATUS_REQUEST    0x04U

#define GIMBAL_APP_STATUS_FLAG_TARGET_VALID   0x01U
#define GIMBAL_APP_STATUS_FLAG_TARGET_SEEN    0x02U
#define GIMBAL_APP_STATUS_FLAG_ACK_PENDING    0x04U
#define GIMBAL_APP_STATUS_FLAG_STATUS_PENDING 0x08U
#define GIMBAL_APP_STATUS_FLAG_RX_OVERFLOW    0x10U
#define GIMBAL_APP_STATUS_FLAG_ACK_OVERFLOW   0x20U

typedef enum
{
    GIMBAL_APP_MSG_TRACK = 0x01,
    GIMBAL_APP_MSG_PARAM_SET = 0x02,
    GIMBAL_APP_MSG_PARAM_GET = 0x03,
    GIMBAL_APP_MSG_CONTROL_CMD = 0x04,
    GIMBAL_APP_MSG_STATUS = 0x81,
    GIMBAL_APP_MSG_ACK = 0x82
} GimbalAppMsgType;

typedef enum
{
    GIMBAL_APP_AXIS_PAN = 0,
    GIMBAL_APP_AXIS_TILT = 1,
    GIMBAL_APP_AXIS_COUNT = 2,
    GIMBAL_APP_AXIS_ALL = 0xFF
} GimbalAppAxis;

typedef enum
{
    GIMBAL_APP_STATE_BOOT_CENTERING = 0,
    GIMBAL_APP_STATE_STANDBY = 1,
    GIMBAL_APP_STATE_TRACKING = 2,
    GIMBAL_APP_STATE_HOLD_LAST = 3
} GimbalAppState;

typedef enum
{
    GIMBAL_APP_ACK_OK = 0,
    GIMBAL_APP_ACK_BAD_LENGTH = 1,
    GIMBAL_APP_ACK_BAD_PARAM = 2,
    GIMBAL_APP_ACK_BAD_CMD = 3,
    GIMBAL_APP_ACK_BAD_STATE = 4,
    GIMBAL_APP_ACK_BAD_MSG = 5,
    GIMBAL_APP_ACK_BUFFER_TOO_SMALL = 6
} GimbalAppAckCode;

typedef enum
{
    GIMBAL_APP_PARAM_DEADBAND = 0x01,
    GIMBAL_APP_PARAM_MAX_STEP_US = 0x02,
    GIMBAL_APP_PARAM_KP_NUM = 0x03,
    GIMBAL_APP_PARAM_KP_DEN = 0x04,
    GIMBAL_APP_PARAM_CENTER_US = 0x05,
    GIMBAL_APP_PARAM_HOME_US = 0x06,
    GIMBAL_APP_PARAM_MIN_US = 0x07,
    GIMBAL_APP_PARAM_MAX_US = 0x08,
    GIMBAL_APP_PARAM_INVERT = 0x09,
    GIMBAL_APP_PARAM_KALMAN_ENABLE = 0x0A,
    GIMBAL_APP_PARAM_STATUS_PERIOD_MS = 0x20,
    GIMBAL_APP_PARAM_TARGET_TIMEOUT_MS = 0x21,
    GIMBAL_APP_PARAM_BOOT_CENTER_MS = 0x22,
    GIMBAL_APP_PARAM_KALMAN_Q_MILLI = 0x23,
    GIMBAL_APP_PARAM_KALMAN_R_MILLI = 0x24
} GimbalAppParamId;

typedef enum
{
    GIMBAL_APP_CTRL_GO_HOME = 0x01,
    GIMBAL_APP_CTRL_SET_STANDBY = 0x02,
    GIMBAL_APP_CTRL_SET_HOLD_LAST = 0x03,
    GIMBAL_APP_CTRL_CLEAR_TARGET = 0x04,
    GIMBAL_APP_CTRL_FORCE_STATUS = 0x05,
    GIMBAL_APP_CTRL_SET_STATE = 0x06
} GimbalAppControlCmd;

typedef struct
{
    uint16_t deadband;
    uint16_t max_step_us;
    uint16_t kp_num;
    uint16_t kp_den;
    uint16_t center_us;
    uint16_t home_us;
    uint16_t min_us;
    uint16_t max_us;
    uint8_t invert;
    uint8_t kalman_enable;
} GimbalAppAxisParams;

typedef struct
{
    GimbalAppAxisParams axis[GIMBAL_APP_AXIS_COUNT];
    uint32_t status_period_ms;
    uint32_t target_timeout_ms;
    uint32_t boot_center_ms;
    uint32_t kalman_q_milli;
    uint32_t kalman_r_milli;
} GimbalAppParams;

typedef struct
{
    uint16_t frame_id;
    uint32_t capture_ts_ms;
    uint32_t rx_tick_ms;
    int16_t err_x;
    int16_t err_y;
    uint8_t flags;
    bool valid;
} GimbalAppTrackSample;

typedef struct
{
    GimbalAppState state;
    uint16_t pan_us;
    uint16_t tilt_us;
    uint16_t pan_compare;
    uint16_t tilt_compare;
    uint16_t last_frame_id;
    uint32_t last_capture_ts_ms;
    uint32_t last_track_rx_ms;
    uint32_t status_period_ms;
    uint32_t target_timeout_ms;
    int16_t last_err_x;
    int16_t last_err_y;
    uint8_t status_flags;
    bool pending_status;
    bool pending_ack;
} GimbalAppStatusSnapshot;

void GimbalApp_Init(void);
void GimbalApp_OnUsbBytes(const uint8_t *data, uint16_t len);
void GimbalApp_RxBytes(const uint8_t *data, uint16_t len);
void GimbalApp_ControlTick(uint32_t now_ms);
void GimbalApp_ControlTick100Hz(void);

GimbalAppState GimbalApp_GetState(void);
uint16_t GimbalApp_GetLatestFrameId(void);
void GimbalApp_GetLatestErrors(int16_t *err_x, int16_t *err_y);
void GimbalApp_GetCurrentOutputUs(uint16_t *pan_us, uint16_t *tilt_us);
void GimbalApp_GetCurrentOutputCompare(uint16_t *pan_compare, uint16_t *tilt_compare);
bool GimbalApp_GetLatestTrack(GimbalAppTrackSample *out_track);
void GimbalApp_GetStatusSnapshot(GimbalAppStatusSnapshot *out_status);
const GimbalAppParams *GimbalApp_GetParams(void);

bool GimbalApp_HasPendingStatusFrame(void);
bool GimbalApp_HasPendingAckFrame(void);
bool GimbalApp_PopTxFrame(uint8_t *out_buf, uint16_t buf_size, uint16_t *out_len);
uint16_t GimbalApp_BuildStatusFrame(uint8_t *out_buf, uint16_t buf_size);
uint16_t GimbalApp_BuildAckFrame(uint8_t *out_buf, uint16_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_APP_H */

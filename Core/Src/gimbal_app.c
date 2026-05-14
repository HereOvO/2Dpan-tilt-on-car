#include "gimbal_app.h"

#include <string.h>

#include "cmsis_os2.h"
#include "debug_uart.h"

typedef enum
{
    GIMBAL_PARSER_WAIT_SOF0 = 0,
    GIMBAL_PARSER_WAIT_SOF1,
    GIMBAL_PARSER_WAIT_MSG_TYPE,
    GIMBAL_PARSER_WAIT_FLAGS,
    GIMBAL_PARSER_WAIT_PAYLOAD_LEN,
    GIMBAL_PARSER_WAIT_FRAME_ID0,
    GIMBAL_PARSER_WAIT_FRAME_ID1,
    GIMBAL_PARSER_WAIT_TS0,
    GIMBAL_PARSER_WAIT_TS1,
    GIMBAL_PARSER_WAIT_TS2,
    GIMBAL_PARSER_WAIT_TS3,
    GIMBAL_PARSER_WAIT_PAYLOAD,
    GIMBAL_PARSER_WAIT_CHECKSUM,
    GIMBAL_PARSER_WAIT_TAIL
} GimbalParserState;

typedef struct
{
    GimbalParserState state;
    uint8_t msg_type;
    uint8_t flags;
    uint8_t payload_len;
    uint16_t frame_id;
    uint32_t capture_ts_ms;
    uint8_t payload[GIMBAL_APP_MAX_PAYLOAD_LEN];
    uint8_t payload_index;
    uint8_t checksum;
} GimbalParserContext;

typedef struct
{
    bool valid;
    uint8_t acked_msg_type;
    uint16_t frame_id;
    GimbalAppAckCode ack_code;
    uint8_t detail_len;
    uint8_t detail[GIMBAL_APP_MAX_PAYLOAD_LEN - 2U];
} GimbalPendingAck;

#define GIMBAL_APP_ACK_QUEUE_LEN 8U
#define GIMBAL_APP_PREDICT_DT_MAX_MS 500U
#define GIMBAL_APP_PREDICT_HORIZON_MAX_MS 250U
#define GIMBAL_APP_PREDICT_VEL_ABS_MAX_ERR_PER_S 12000.0f

typedef struct
{
    GimbalPendingAck items[GIMBAL_APP_ACK_QUEUE_LEN];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} GimbalAckQueue;

typedef struct
{
    GimbalAppParams params;
    GimbalParserContext parser;
    GimbalAckQueue ack_queue;
    GimbalAppTrackSample latest_track;
    GimbalAppState state;
    int16_t control_err[GIMBAL_APP_AXIS_COUNT];
    uint16_t output_us[GIMBAL_APP_AXIS_COUNT];
    uint16_t output_compare[GIMBAL_APP_AXIS_COUNT];
    uint32_t boot_center_until_ms;
    uint32_t last_status_mark_ms;
    uint16_t tx_status_frame_id;
    bool pending_status;
    bool target_was_seen;
    bool rx_overflow;
    bool ack_overflow;
    uint16_t rx_fifo_head;
    uint16_t rx_fifo_tail;
    uint8_t rx_fifo[GIMBAL_APP_RX_FIFO_LEN];
} GimbalAppContext;

typedef struct
{
    bool initialized;
    float estimate;
    float covariance;
} GimbalAxisKalmanState;

typedef struct
{
    bool initialized;
    float last_measurement;
    float velocity_err_per_s;
    uint32_t last_capture_ts_ms;
    uint32_t last_rx_tick_ms;
} GimbalAxisPredictState;

extern TIM_HandleTypeDef htim1;

static GimbalAppContext g_gimbal;
static GimbalAxisKalmanState g_kalman[GIMBAL_APP_AXIS_COUNT];
static GimbalAxisPredictState g_predictor[GIMBAL_APP_AXIS_COUNT];

static uint32_t GimbalApp_GetNowMs(void);
static void GimbalApp_LoadDefaultParams(GimbalAppParams *params);
static void GimbalApp_SanitizeParams(GimbalAppParams *params);
static uint16_t GimbalApp_ClampAxisUs(const GimbalAppAxisParams *axis, int32_t value_us);
static uint16_t GimbalApp_UsToCompare(uint16_t pulse_us);
static void GimbalApp_WriteOutputs(void);
static void GimbalApp_SetOutputsToHome(void);
static void GimbalApp_SetState(GimbalAppState new_state);
static uint8_t GimbalApp_MakeStatusFlags(void);
static void GimbalApp_RequestStatus(void);
static void GimbalApp_ClearTargetValidity(void);
static bool GimbalApp_HandleTrackFrame(uint16_t frame_id, uint32_t capture_ts_ms, uint8_t flags, const uint8_t *payload, uint8_t payload_len);
static void GimbalApp_HandleParamSet(uint8_t msg_type, uint16_t frame_id, const uint8_t *payload, uint8_t payload_len);
static void GimbalApp_HandleParamGet(uint8_t msg_type, uint16_t frame_id, const uint8_t *payload, uint8_t payload_len);
static void GimbalApp_HandleControlCmd(uint8_t msg_type, uint16_t frame_id, const uint8_t *payload, uint8_t payload_len);
static void GimbalApp_HandleParsedFrame(uint8_t msg_type, uint8_t flags, uint16_t frame_id, uint32_t capture_ts_ms, const uint8_t *payload, uint8_t payload_len);
static void GimbalApp_QueueAck(uint8_t acked_msg_type, uint16_t frame_id, GimbalAppAckCode ack_code, const uint8_t *detail, uint8_t detail_len);
static uint16_t GimbalApp_PackFrame(uint8_t msg_type, uint8_t flags, uint16_t frame_id, uint32_t capture_ts_ms, const uint8_t *payload, uint8_t payload_len, uint8_t *out_buf, uint16_t buf_size);
static bool GimbalApp_TrySetParam(GimbalAppParams *params, uint8_t param_id, uint8_t axis_id, int32_t value);
static bool GimbalApp_TryReadParam(const GimbalAppParams *params, uint8_t param_id, uint8_t axis_id, int32_t *value_out);
static uint8_t GimbalApp_AppendParamRecord(uint8_t *dst, uint8_t offset, uint8_t param_id, uint8_t axis_id, int32_t value);
static int32_t GimbalApp_ReadLe32(const uint8_t *src);
static uint16_t GimbalApp_ReadLe16(const uint8_t *src);
static void GimbalApp_WriteLe16(uint8_t *dst, uint16_t value);
static void GimbalApp_WriteLe32(uint8_t *dst, uint32_t value);
static int16_t GimbalApp_ReadLe16s(const uint8_t *src);
static uint16_t GimbalApp_AbsI16(int16_t value);
static float GimbalApp_ParamMilliToFloat(uint32_t value_milli);
static int16_t GimbalApp_FloatToI16(float value);
static float GimbalApp_ClampFloat(float value, float min_value, float max_value);
static uint32_t GimbalApp_ResolveTrackDtMs(const GimbalAxisPredictState *axis_state, uint32_t capture_ts_ms, uint32_t rx_tick_ms);
static void GimbalApp_ResetKalmanFilters(void);
static void GimbalApp_ResetPredictorStates(void);
static bool GimbalApp_DidObserverConfigChange(const GimbalAppParams *lhs, const GimbalAppParams *rhs);
static int16_t GimbalApp_ApplyKalman(uint8_t axis_index, int16_t measurement);
static void GimbalApp_UpdatePredictor(uint8_t axis_index, int16_t measurement, uint32_t capture_ts_ms, uint32_t rx_tick_ms);
static int16_t GimbalApp_GetControlErrorForAxis(uint8_t axis_index, uint32_t now_ms);
static void GimbalApp_ConfigureServoTimer(void);
static void GimbalApp_ProcessQueuedRx(void);
static bool GimbalApp_RxFifoPush(uint8_t byte);
static bool GimbalApp_RxFifoPop(uint8_t *byte);
static bool GimbalApp_CanAppendParamRecord(uint8_t offset, uint8_t capacity);
static bool GimbalApp_IsAxisParam(uint8_t param_id);

void GimbalApp_Init(void)
{
    memset(&g_gimbal, 0, sizeof(g_gimbal));
    memset(g_kalman, 0, sizeof(g_kalman));
    memset(g_predictor, 0, sizeof(g_predictor));

    GimbalApp_ConfigureServoTimer();
    GimbalApp_LoadDefaultParams(&g_gimbal.params);
    GimbalApp_SanitizeParams(&g_gimbal.params);

    g_gimbal.parser.state = GIMBAL_PARSER_WAIT_SOF0;
    g_gimbal.state = GIMBAL_APP_STATE_BOOT_CENTERING;
    g_gimbal.boot_center_until_ms = GimbalApp_GetNowMs() + g_gimbal.params.boot_center_ms;
    g_gimbal.tx_status_frame_id = 1U;
    g_gimbal.latest_track.valid = false;
    g_gimbal.latest_track.flags = 0U;

    GimbalApp_SetOutputsToHome();
    GimbalApp_RequestStatus();
}

void GimbalApp_OnUsbBytes(const uint8_t *data, uint16_t len)
{
    uint16_t index;

    if ((data == NULL) || (len == 0U))
    {
        return;
    }

    for (index = 0U; index < len; ++index)
    {
        if (!GimbalApp_RxFifoPush(data[index]))
        {
            if (!g_gimbal.rx_overflow)
            {
                DebugUart_WriteString("usb rx overflow\r\n");
            }
            g_gimbal.rx_overflow = true;
            GimbalApp_RequestStatus();
            break;
        }
    }
}

void GimbalApp_RxBytes(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if ((data == NULL) || (len == 0U))
    {
        return;
    }

    for (i = 0U; i < len; ++i)
    {
        uint8_t byte = data[i];

        switch (g_gimbal.parser.state)
        {
        case GIMBAL_PARSER_WAIT_SOF0:
            if (byte == GIMBAL_APP_PROTO_SOF0)
            {
                g_gimbal.parser.state = GIMBAL_PARSER_WAIT_SOF1;
            }
            break;

        case GIMBAL_PARSER_WAIT_SOF1:
            if (byte == GIMBAL_APP_PROTO_SOF1)
            {
                g_gimbal.parser.state = GIMBAL_PARSER_WAIT_MSG_TYPE;
                g_gimbal.parser.checksum = 0U;
                g_gimbal.parser.payload_index = 0U;
                g_gimbal.parser.payload_len = 0U;
                g_gimbal.parser.frame_id = 0U;
                g_gimbal.parser.capture_ts_ms = 0U;
                g_gimbal.parser.msg_type = 0U;
                g_gimbal.parser.flags = 0U;
            }
            else if (byte != GIMBAL_APP_PROTO_SOF0)
            {
                g_gimbal.parser.state = GIMBAL_PARSER_WAIT_SOF0;
            }
            break;

        case GIMBAL_PARSER_WAIT_MSG_TYPE:
            g_gimbal.parser.msg_type = byte;
            g_gimbal.parser.checksum ^= byte;
            g_gimbal.parser.state = GIMBAL_PARSER_WAIT_FLAGS;
            break;

        case GIMBAL_PARSER_WAIT_FLAGS:
            g_gimbal.parser.flags = byte;
            g_gimbal.parser.checksum ^= byte;
            g_gimbal.parser.state = GIMBAL_PARSER_WAIT_PAYLOAD_LEN;
            break;

        case GIMBAL_PARSER_WAIT_PAYLOAD_LEN:
            g_gimbal.parser.payload_len = byte;
            g_gimbal.parser.checksum ^= byte;
            if (g_gimbal.parser.payload_len > GIMBAL_APP_MAX_PAYLOAD_LEN)
            {
                g_gimbal.parser.state = GIMBAL_PARSER_WAIT_SOF0;
            }
            else
            {
                g_gimbal.parser.state = GIMBAL_PARSER_WAIT_FRAME_ID0;
            }
            break;

        case GIMBAL_PARSER_WAIT_FRAME_ID0:
            g_gimbal.parser.frame_id = byte;
            g_gimbal.parser.checksum ^= byte;
            g_gimbal.parser.state = GIMBAL_PARSER_WAIT_FRAME_ID1;
            break;

        case GIMBAL_PARSER_WAIT_FRAME_ID1:
            g_gimbal.parser.frame_id |= ((uint16_t)byte << 8);
            g_gimbal.parser.checksum ^= byte;
            g_gimbal.parser.state = GIMBAL_PARSER_WAIT_TS0;
            break;

        case GIMBAL_PARSER_WAIT_TS0:
            g_gimbal.parser.capture_ts_ms = byte;
            g_gimbal.parser.checksum ^= byte;
            g_gimbal.parser.state = GIMBAL_PARSER_WAIT_TS1;
            break;

        case GIMBAL_PARSER_WAIT_TS1:
            g_gimbal.parser.capture_ts_ms |= ((uint32_t)byte << 8);
            g_gimbal.parser.checksum ^= byte;
            g_gimbal.parser.state = GIMBAL_PARSER_WAIT_TS2;
            break;

        case GIMBAL_PARSER_WAIT_TS2:
            g_gimbal.parser.capture_ts_ms |= ((uint32_t)byte << 16);
            g_gimbal.parser.checksum ^= byte;
            g_gimbal.parser.state = GIMBAL_PARSER_WAIT_TS3;
            break;

        case GIMBAL_PARSER_WAIT_TS3:
            g_gimbal.parser.capture_ts_ms |= ((uint32_t)byte << 24);
            g_gimbal.parser.checksum ^= byte;
            if (g_gimbal.parser.payload_len == 0U)
            {
                g_gimbal.parser.state = GIMBAL_PARSER_WAIT_CHECKSUM;
            }
            else
            {
                g_gimbal.parser.state = GIMBAL_PARSER_WAIT_PAYLOAD;
            }
            break;

        case GIMBAL_PARSER_WAIT_PAYLOAD:
            g_gimbal.parser.payload[g_gimbal.parser.payload_index] = byte;
            g_gimbal.parser.payload_index++;
            g_gimbal.parser.checksum ^= byte;
            if (g_gimbal.parser.payload_index >= g_gimbal.parser.payload_len)
            {
                g_gimbal.parser.state = GIMBAL_PARSER_WAIT_CHECKSUM;
            }
            break;

        case GIMBAL_PARSER_WAIT_CHECKSUM:
            if (byte == g_gimbal.parser.checksum)
            {
                g_gimbal.parser.state = GIMBAL_PARSER_WAIT_TAIL;
            }
            else
            {
                g_gimbal.parser.state = GIMBAL_PARSER_WAIT_SOF0;
            }
            break;

        case GIMBAL_PARSER_WAIT_TAIL:
            if (byte == GIMBAL_APP_PROTO_TAIL)
            {
                GimbalApp_HandleParsedFrame(
                    g_gimbal.parser.msg_type,
                    g_gimbal.parser.flags,
                    g_gimbal.parser.frame_id,
                    g_gimbal.parser.capture_ts_ms,
                    g_gimbal.parser.payload,
                    g_gimbal.parser.payload_len);
            }
            g_gimbal.parser.state = GIMBAL_PARSER_WAIT_SOF0;
            break;

        default:
            g_gimbal.parser.state = GIMBAL_PARSER_WAIT_SOF0;
            break;
        }
    }
}

void GimbalApp_ControlTick(uint32_t now_ms)
{
    (void)now_ms;

    GimbalApp_ProcessQueuedRx();
    GimbalApp_ControlTick100Hz();
}

void GimbalApp_ControlTick100Hz(void)
{
    uint32_t now_ms = GimbalApp_GetNowMs();

    if ((g_gimbal.state == GIMBAL_APP_STATE_BOOT_CENTERING) &&
        ((uint32_t)(now_ms - g_gimbal.boot_center_until_ms) < 0x80000000UL))
    {
        if ((int32_t)(now_ms - g_gimbal.boot_center_until_ms) >= 0)
        {
            GimbalApp_SetState(GIMBAL_APP_STATE_STANDBY);
            GimbalApp_SetOutputsToHome();
        }
    }

    if (g_gimbal.latest_track.valid)
    {
        uint32_t age_ms = now_ms - g_gimbal.latest_track.rx_tick_ms;

        if (age_ms > g_gimbal.params.target_timeout_ms)
        {
            GimbalApp_ClearTargetValidity();
            if (g_gimbal.target_was_seen)
            {
                GimbalApp_SetState(GIMBAL_APP_STATE_HOLD_LAST);
            }
        }
    }

    if ((g_gimbal.state == GIMBAL_APP_STATE_TRACKING) && g_gimbal.latest_track.valid)
    {
        uint16_t axis_index;

        for (axis_index = 0U; axis_index < GIMBAL_APP_AXIS_COUNT; ++axis_index)
        {
            const GimbalAppAxisParams *axis = &g_gimbal.params.axis[axis_index];
            int16_t raw_err = GimbalApp_GetControlErrorForAxis(axis_index, now_ms);
            int32_t effective_err = raw_err;
            int32_t delta_us;

            g_gimbal.control_err[axis_index] = raw_err;

            if (GimbalApp_AbsI16(raw_err) <= axis->deadband)
            {
                effective_err = 0;
            }

            delta_us = (effective_err * (int32_t)axis->kp_num) / (int32_t)axis->kp_den;

            if (delta_us > (int32_t)axis->max_step_us)
            {
                delta_us = (int32_t)axis->max_step_us;
            }
            else if (delta_us < -(int32_t)axis->max_step_us)
            {
                delta_us = -(int32_t)axis->max_step_us;
            }

            if (axis->invert != 0U)
            {
                delta_us = -delta_us;
            }

            g_gimbal.output_us[axis_index] = GimbalApp_ClampAxisUs(
                axis,
                (int32_t)g_gimbal.output_us[axis_index] + delta_us);
        }

        GimbalApp_WriteOutputs();
    }
    else if (g_gimbal.state == GIMBAL_APP_STATE_STANDBY)
    {
        g_gimbal.control_err[GIMBAL_APP_AXIS_PAN] = 0;
        g_gimbal.control_err[GIMBAL_APP_AXIS_TILT] = 0;
        GimbalApp_SetOutputsToHome();
    }
    else
    {
        g_gimbal.control_err[GIMBAL_APP_AXIS_PAN] = 0;
        g_gimbal.control_err[GIMBAL_APP_AXIS_TILT] = 0;
    }

    if ((g_gimbal.params.status_period_ms > 0U) &&
        ((now_ms - g_gimbal.last_status_mark_ms) >= g_gimbal.params.status_period_ms))
    {
        g_gimbal.last_status_mark_ms = now_ms;
        g_gimbal.pending_status = true;
    }
}

bool GimbalApp_PopTxFrame(uint8_t *out_buf, uint16_t buf_size, uint16_t *out_len)
{
    uint16_t frame_len = 0U;

    if (out_len == NULL)
    {
        return false;
    }

    *out_len = 0U;

    if ((out_buf == NULL) || (buf_size == 0U))
    {
        return false;
    }

    if (g_gimbal.ack_queue.count > 0U)
    {
        frame_len = GimbalApp_BuildAckFrame(out_buf, buf_size);
    }
    else if (g_gimbal.pending_status)
    {
        frame_len = GimbalApp_BuildStatusFrame(out_buf, buf_size);
    }

    if (frame_len == 0U)
    {
        return false;
    }

    *out_len = frame_len;
    return true;
}

GimbalAppState GimbalApp_GetState(void)
{
    return g_gimbal.state;
}

uint16_t GimbalApp_GetLatestFrameId(void)
{
    return g_gimbal.latest_track.frame_id;
}

void GimbalApp_GetLatestErrors(int16_t *err_x, int16_t *err_y)
{
    if (err_x != NULL)
    {
        *err_x = g_gimbal.control_err[GIMBAL_APP_AXIS_PAN];
    }

    if (err_y != NULL)
    {
        *err_y = g_gimbal.control_err[GIMBAL_APP_AXIS_TILT];
    }
}

void GimbalApp_GetCurrentOutputUs(uint16_t *pan_us, uint16_t *tilt_us)
{
    if (pan_us != NULL)
    {
        *pan_us = g_gimbal.output_us[GIMBAL_APP_AXIS_PAN];
    }

    if (tilt_us != NULL)
    {
        *tilt_us = g_gimbal.output_us[GIMBAL_APP_AXIS_TILT];
    }
}

void GimbalApp_GetCurrentOutputCompare(uint16_t *pan_compare, uint16_t *tilt_compare)
{
    if (pan_compare != NULL)
    {
        *pan_compare = g_gimbal.output_compare[GIMBAL_APP_AXIS_PAN];
    }

    if (tilt_compare != NULL)
    {
        *tilt_compare = g_gimbal.output_compare[GIMBAL_APP_AXIS_TILT];
    }
}

bool GimbalApp_GetLatestTrack(GimbalAppTrackSample *out_track)
{
    if (out_track == NULL)
    {
        return false;
    }

    *out_track = g_gimbal.latest_track;
    return true;
}

void GimbalApp_GetStatusSnapshot(GimbalAppStatusSnapshot *out_status)
{
    if (out_status == NULL)
    {
        return;
    }

    out_status->state = g_gimbal.state;
    out_status->pan_us = g_gimbal.output_us[GIMBAL_APP_AXIS_PAN];
    out_status->tilt_us = g_gimbal.output_us[GIMBAL_APP_AXIS_TILT];
    out_status->pan_compare = g_gimbal.output_compare[GIMBAL_APP_AXIS_PAN];
    out_status->tilt_compare = g_gimbal.output_compare[GIMBAL_APP_AXIS_TILT];
    out_status->last_frame_id = g_gimbal.latest_track.frame_id;
    out_status->last_capture_ts_ms = g_gimbal.latest_track.capture_ts_ms;
    out_status->last_track_rx_ms = g_gimbal.latest_track.rx_tick_ms;
    out_status->status_period_ms = g_gimbal.params.status_period_ms;
    out_status->target_timeout_ms = g_gimbal.params.target_timeout_ms;
    out_status->last_err_x = g_gimbal.control_err[GIMBAL_APP_AXIS_PAN];
    out_status->last_err_y = g_gimbal.control_err[GIMBAL_APP_AXIS_TILT];
    out_status->status_flags = GimbalApp_MakeStatusFlags();
    out_status->pending_status = g_gimbal.pending_status;
    out_status->pending_ack = (g_gimbal.ack_queue.count > 0U);
}

const GimbalAppParams *GimbalApp_GetParams(void)
{
    return &g_gimbal.params;
}

bool GimbalApp_HasPendingStatusFrame(void)
{
    return g_gimbal.pending_status;
}

bool GimbalApp_HasPendingAckFrame(void)
{
    return (g_gimbal.ack_queue.count > 0U);
}

uint16_t GimbalApp_BuildStatusFrame(uint8_t *out_buf, uint16_t buf_size)
{
    uint8_t payload[32];
    uint16_t length;

    if (out_buf == NULL)
    {
        return 0U;
    }

    payload[0] = (uint8_t)g_gimbal.state;
    payload[1] = GimbalApp_MakeStatusFlags();
    GimbalApp_WriteLe16(&payload[2], g_gimbal.latest_track.frame_id);
    GimbalApp_WriteLe16(&payload[4], (uint16_t)g_gimbal.control_err[GIMBAL_APP_AXIS_PAN]);
    GimbalApp_WriteLe16(&payload[6], (uint16_t)g_gimbal.control_err[GIMBAL_APP_AXIS_TILT]);
    GimbalApp_WriteLe16(&payload[8], g_gimbal.output_us[GIMBAL_APP_AXIS_PAN]);
    GimbalApp_WriteLe16(&payload[10], g_gimbal.output_us[GIMBAL_APP_AXIS_TILT]);
    GimbalApp_WriteLe16(&payload[12], g_gimbal.output_compare[GIMBAL_APP_AXIS_PAN]);
    GimbalApp_WriteLe16(&payload[14], g_gimbal.output_compare[GIMBAL_APP_AXIS_TILT]);
    GimbalApp_WriteLe32(&payload[16], g_gimbal.latest_track.capture_ts_ms);
    GimbalApp_WriteLe32(&payload[20], g_gimbal.latest_track.rx_tick_ms);
    GimbalApp_WriteLe32(&payload[24], g_gimbal.params.status_period_ms);
    GimbalApp_WriteLe32(&payload[28], g_gimbal.params.target_timeout_ms);

    length = GimbalApp_PackFrame(
        GIMBAL_APP_MSG_STATUS,
        0U,
        g_gimbal.tx_status_frame_id++,
        GimbalApp_GetNowMs(),
        payload,
        (uint8_t)sizeof(payload),
        out_buf,
        buf_size);

    if (length > 0U)
    {
        g_gimbal.pending_status = false;
    }

    return length;
}

uint16_t GimbalApp_BuildAckFrame(uint8_t *out_buf, uint16_t buf_size)
{
    uint8_t payload[GIMBAL_APP_MAX_PAYLOAD_LEN];
    uint16_t length;

    GimbalPendingAck *pending_ack;

    if ((out_buf == NULL) || (g_gimbal.ack_queue.count == 0U))
    {
        return 0U;
    }

    pending_ack = &g_gimbal.ack_queue.items[g_gimbal.ack_queue.tail];

    payload[0] = pending_ack->acked_msg_type;
    payload[1] = (uint8_t)pending_ack->ack_code;

    if (pending_ack->detail_len > 0U)
    {
        memcpy(&payload[2], pending_ack->detail, pending_ack->detail_len);
    }

    length = GimbalApp_PackFrame(
        GIMBAL_APP_MSG_ACK,
        0U,
        pending_ack->frame_id,
        GimbalApp_GetNowMs(),
        payload,
        (uint8_t)(2U + pending_ack->detail_len),
        out_buf,
        buf_size);

    if (length > 0U)
    {
        g_gimbal.ack_queue.tail = (uint8_t)((g_gimbal.ack_queue.tail + 1U) % GIMBAL_APP_ACK_QUEUE_LEN);
        g_gimbal.ack_queue.count--;
    }

    return length;
}

static uint32_t GimbalApp_GetNowMs(void)
{
    if (osKernelGetState() == osKernelRunning)
    {
        return osKernelGetTickCount();
    }

    return HAL_GetTick();
}

static void GimbalApp_LoadDefaultParams(GimbalAppParams *params)
{
    if (params == NULL)
    {
        return;
    }

    memset(params, 0, sizeof(*params));

    params->axis[GIMBAL_APP_AXIS_PAN].deadband = 4U;
    params->axis[GIMBAL_APP_AXIS_PAN].max_step_us = 20U;
    params->axis[GIMBAL_APP_AXIS_PAN].kp_num = 1U;
    params->axis[GIMBAL_APP_AXIS_PAN].kp_den = 8U;
    params->axis[GIMBAL_APP_AXIS_PAN].center_us = 1500U;
    params->axis[GIMBAL_APP_AXIS_PAN].home_us = 1500U;
    params->axis[GIMBAL_APP_AXIS_PAN].min_us = 500U;
    params->axis[GIMBAL_APP_AXIS_PAN].max_us = 2500U;
    params->axis[GIMBAL_APP_AXIS_PAN].invert = 0U;
    params->axis[GIMBAL_APP_AXIS_PAN].kalman_enable = 1U;

    params->axis[GIMBAL_APP_AXIS_TILT].deadband = 4U;
    params->axis[GIMBAL_APP_AXIS_TILT].max_step_us = 20U;
    params->axis[GIMBAL_APP_AXIS_TILT].kp_num = 1U;
    params->axis[GIMBAL_APP_AXIS_TILT].kp_den = 8U;
    params->axis[GIMBAL_APP_AXIS_TILT].center_us = 1500U;
    params->axis[GIMBAL_APP_AXIS_TILT].home_us = 501U;
    params->axis[GIMBAL_APP_AXIS_TILT].min_us = 500U;
    params->axis[GIMBAL_APP_AXIS_TILT].max_us = 2500U;
    params->axis[GIMBAL_APP_AXIS_TILT].invert = 0U;
    params->axis[GIMBAL_APP_AXIS_TILT].kalman_enable = 1U;

    params->status_period_ms = 100U;
    params->target_timeout_ms = 250U;
    params->boot_center_ms = 300U;
    params->kalman_q_milli = 16000U;
    params->kalman_r_milli = 64000U;
    params->predict_enable = 0U;
    params->predict_lead_ms = 80U;
    params->predict_vel_tc_ms = 120U;
}

static void GimbalApp_SanitizeParams(GimbalAppParams *params)
{
    uint16_t axis_index;

    if (params == NULL)
    {
        return;
    }

    for (axis_index = 0U; axis_index < GIMBAL_APP_AXIS_COUNT; ++axis_index)
    {
        GimbalAppAxisParams *axis = &params->axis[axis_index];

        if (axis->min_us < GIMBAL_APP_SERVO_PULSE_MIN_US)
        {
            axis->min_us = GIMBAL_APP_SERVO_PULSE_MIN_US;
        }

        if (axis->max_us > GIMBAL_APP_SERVO_PULSE_MAX_US)
        {
            axis->max_us = GIMBAL_APP_SERVO_PULSE_MAX_US;
        }

        if (axis->min_us > axis->max_us)
        {
            uint16_t tmp = axis->min_us;
            axis->min_us = axis->max_us;
            axis->max_us = tmp;
        }

        if (axis->kp_den == 0U)
        {
            axis->kp_den = 1U;
        }

        if (axis->max_step_us == 0U)
        {
            axis->max_step_us = 1U;
        }

        axis->center_us = GimbalApp_ClampAxisUs(axis, axis->center_us);
        axis->home_us = GimbalApp_ClampAxisUs(axis, axis->home_us);
        axis->invert = (axis->invert != 0U) ? 1U : 0U;
        axis->kalman_enable = (axis->kalman_enable != 0U) ? 1U : 0U;
    }

    if (params->target_timeout_ms == 0U)
    {
        params->target_timeout_ms = 1U;
    }

    if (params->kalman_r_milli == 0U)
    {
        params->kalman_r_milli = 1U;
    }

    params->predict_enable = (params->predict_enable != 0U) ? 1U : 0U;
}

static uint16_t GimbalApp_ClampAxisUs(const GimbalAppAxisParams *axis, int32_t value_us)
{
    if (value_us < (int32_t)axis->min_us)
    {
        return axis->min_us;
    }

    if (value_us > (int32_t)axis->max_us)
    {
        return axis->max_us;
    }

    return (uint16_t)value_us;
}

static void GimbalApp_ConfigureServoTimer(void)
{
    if (htim1.Instance != TIM1)
    {
        return;
    }

    __HAL_TIM_DISABLE(&htim1);
    htim1.Init.Prescaler = GIMBAL_APP_SERVO_TIMER_PSC;
    htim1.Init.Period = GIMBAL_APP_SERVO_TIMER_ARR;
    __HAL_TIM_SET_PRESCALER(&htim1, htim1.Init.Prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim1, htim1.Init.Period);
    htim1.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COUNTER(&htim1, 0U);

    DebugUart_Printf(
        "tim1 servo pwm psc=%lu arr=%lu\r\n",
        (unsigned long)htim1.Init.Prescaler,
        (unsigned long)htim1.Init.Period);
}

static uint16_t GimbalApp_UsToCompare(uint16_t pulse_us)
{
    uint32_t timer_clk_hz;
    uint32_t pclk_hz = HAL_RCC_GetPCLK2Freq();
    uint32_t counter_hz;
    uint32_t compare;
    uint32_t timer_period;

    if (htim1.Instance != TIM1)
    {
        return 0U;
    }

    if ((RCC->CFGR & RCC_CFGR_PPRE2) == RCC_HCLK_DIV1)
    {
        timer_clk_hz = pclk_hz;
    }
    else
    {
        timer_clk_hz = pclk_hz * 2U;
    }

    counter_hz = timer_clk_hz / (htim1.Init.Prescaler + 1U);
    compare = ((uint32_t)pulse_us * counter_hz) / 1000000U;
    timer_period = htim1.Init.Period;

    if (compare > timer_period)
    {
        compare = timer_period;
    }

    if (compare > 0xFFFFU)
    {
        compare = 0xFFFFU;
    }

    return (uint16_t)compare;
}

static void GimbalApp_WriteOutputs(void)
{
    g_gimbal.output_compare[GIMBAL_APP_AXIS_PAN] = GimbalApp_UsToCompare(g_gimbal.output_us[GIMBAL_APP_AXIS_PAN]);
    g_gimbal.output_compare[GIMBAL_APP_AXIS_TILT] = GimbalApp_UsToCompare(g_gimbal.output_us[GIMBAL_APP_AXIS_TILT]);

    if (htim1.Instance == TIM1)
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, g_gimbal.output_compare[GIMBAL_APP_AXIS_PAN]);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, g_gimbal.output_compare[GIMBAL_APP_AXIS_TILT]);
    }
}

static void GimbalApp_SetOutputsToHome(void)
{
    g_gimbal.output_us[GIMBAL_APP_AXIS_PAN] = g_gimbal.params.axis[GIMBAL_APP_AXIS_PAN].home_us;
    g_gimbal.output_us[GIMBAL_APP_AXIS_TILT] = g_gimbal.params.axis[GIMBAL_APP_AXIS_TILT].home_us;
    GimbalApp_WriteOutputs();
}

static void GimbalApp_SetState(GimbalAppState new_state)
{
    if (g_gimbal.state != new_state)
    {
        g_gimbal.state = new_state;
        DebugUart_Printf("gimbal state=%u\r\n", (unsigned)new_state);
        GimbalApp_RequestStatus();
    }
}

static uint8_t GimbalApp_MakeStatusFlags(void)
{
    uint8_t flags = 0U;

    if (g_gimbal.latest_track.valid)
    {
        flags |= GIMBAL_APP_STATUS_FLAG_TARGET_VALID;
    }

    if (g_gimbal.target_was_seen)
    {
        flags |= GIMBAL_APP_STATUS_FLAG_TARGET_SEEN;
    }

    if (g_gimbal.ack_queue.count > 0U)
    {
        flags |= GIMBAL_APP_STATUS_FLAG_ACK_PENDING;
    }

    if (g_gimbal.pending_status)
    {
        flags |= GIMBAL_APP_STATUS_FLAG_STATUS_PENDING;
    }

    if (g_gimbal.rx_overflow)
    {
        flags |= GIMBAL_APP_STATUS_FLAG_RX_OVERFLOW;
    }

    if (g_gimbal.ack_overflow)
    {
        flags |= GIMBAL_APP_STATUS_FLAG_ACK_OVERFLOW;
    }

    return flags;
}

static void GimbalApp_RequestStatus(void)
{
    g_gimbal.pending_status = true;
}

static void GimbalApp_ClearTargetValidity(void)
{
    g_gimbal.latest_track.valid = false;
    g_gimbal.control_err[GIMBAL_APP_AXIS_PAN] = 0;
    g_gimbal.control_err[GIMBAL_APP_AXIS_TILT] = 0;
    GimbalApp_ResetKalmanFilters();
    GimbalApp_ResetPredictorStates();
}

static bool GimbalApp_HandleTrackFrame(uint16_t frame_id, uint32_t capture_ts_ms, uint8_t flags, const uint8_t *payload, uint8_t payload_len)
{
    int16_t raw_err_x;
    int16_t raw_err_y;
    uint32_t now_ms;

    if (payload_len != 4U)
    {
        return false;
    }

    now_ms = GimbalApp_GetNowMs();
    raw_err_x = GimbalApp_ReadLe16s(&payload[0]);
    raw_err_y = GimbalApp_ReadLe16s(&payload[2]);

    g_gimbal.latest_track.frame_id = frame_id;
    g_gimbal.latest_track.capture_ts_ms = capture_ts_ms;
    g_gimbal.latest_track.rx_tick_ms = now_ms;
    g_gimbal.latest_track.flags = flags;
    g_gimbal.target_was_seen = true;

    if ((flags & GIMBAL_APP_FLAG_TARGET_VALID) != 0U)
    {
        g_gimbal.latest_track.err_x = GimbalApp_ApplyKalman(GIMBAL_APP_AXIS_PAN, raw_err_x);
        g_gimbal.latest_track.err_y = GimbalApp_ApplyKalman(GIMBAL_APP_AXIS_TILT, raw_err_y);
        GimbalApp_UpdatePredictor(GIMBAL_APP_AXIS_PAN, g_gimbal.latest_track.err_x, capture_ts_ms, now_ms);
        GimbalApp_UpdatePredictor(GIMBAL_APP_AXIS_TILT, g_gimbal.latest_track.err_y, capture_ts_ms, now_ms);
        g_gimbal.latest_track.valid = true;
        if (g_gimbal.state != GIMBAL_APP_STATE_BOOT_CENTERING)
        {
            GimbalApp_SetState(GIMBAL_APP_STATE_TRACKING);
        }
    }
    else
    {
        g_gimbal.latest_track.err_x = raw_err_x;
        g_gimbal.latest_track.err_y = raw_err_y;
        g_gimbal.control_err[GIMBAL_APP_AXIS_PAN] = raw_err_x;
        g_gimbal.control_err[GIMBAL_APP_AXIS_TILT] = raw_err_y;
        GimbalApp_ResetKalmanFilters();
        GimbalApp_ResetPredictorStates();
        g_gimbal.latest_track.valid = false;
        if (g_gimbal.state != GIMBAL_APP_STATE_BOOT_CENTERING)
        {
            GimbalApp_SetState(GIMBAL_APP_STATE_HOLD_LAST);
        }
    }

    return true;
}

static void GimbalApp_HandleParamSet(uint8_t msg_type, uint16_t frame_id, const uint8_t *payload, uint8_t payload_len)
{
    GimbalAppParams new_params;
    bool observer_config_changed;
    uint8_t detail[GIMBAL_APP_MAX_PAYLOAD_LEN - 2U];
    uint8_t offset = 0U;
    uint8_t index;

    if ((payload_len == 0U) || ((payload_len % 6U) != 0U))
    {
        GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BAD_LENGTH, NULL, 0U);
        return;
    }

    new_params = g_gimbal.params;

    for (index = 0U; index < payload_len; index = (uint8_t)(index + 6U))
    {
        uint8_t param_id = payload[index];
        uint8_t axis_id = payload[index + 1U];
        int32_t value = GimbalApp_ReadLe32(&payload[index + 2U]);

        if (!GimbalApp_TrySetParam(&new_params, param_id, axis_id, value))
        {
            GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BAD_PARAM, NULL, 0U);
            return;
        }
    }

    GimbalApp_SanitizeParams(&new_params);
    observer_config_changed = GimbalApp_DidObserverConfigChange(&g_gimbal.params, &new_params);
    g_gimbal.params = new_params;

    if (observer_config_changed)
    {
        GimbalApp_ResetKalmanFilters();
        GimbalApp_ResetPredictorStates();
    }

    if ((g_gimbal.state == GIMBAL_APP_STATE_STANDBY) || (g_gimbal.state == GIMBAL_APP_STATE_BOOT_CENTERING))
    {
        GimbalApp_SetOutputsToHome();
    }
    else
    {
        g_gimbal.output_us[GIMBAL_APP_AXIS_PAN] = GimbalApp_ClampAxisUs(
            &g_gimbal.params.axis[GIMBAL_APP_AXIS_PAN],
            g_gimbal.output_us[GIMBAL_APP_AXIS_PAN]);
        g_gimbal.output_us[GIMBAL_APP_AXIS_TILT] = GimbalApp_ClampAxisUs(
            &g_gimbal.params.axis[GIMBAL_APP_AXIS_TILT],
            g_gimbal.output_us[GIMBAL_APP_AXIS_TILT]);
        GimbalApp_WriteOutputs();
    }

    for (index = 0U; index < payload_len; index = (uint8_t)(index + 6U))
    {
        uint8_t param_id = payload[index];
        uint8_t axis_id = payload[index + 1U];
        uint8_t axis_start;
        uint8_t axis_end;

        if ((axis_id == GIMBAL_APP_AXIS_ALL) && GimbalApp_IsAxisParam(param_id))
        {
            axis_start = 0U;
            axis_end = GIMBAL_APP_AXIS_COUNT;
        }
        else if ((!GimbalApp_IsAxisParam(param_id)) && (axis_id == GIMBAL_APP_AXIS_ALL))
        {
            axis_start = 0U;
            axis_end = 1U;
        }
        else
        {
            axis_start = axis_id;
            axis_end = (uint8_t)(axis_id + 1U);
        }

        for (; axis_start < axis_end; ++axis_start)
        {
            int32_t applied_value;
            uint8_t record_axis = GimbalApp_IsAxisParam(param_id) ? axis_start : GIMBAL_APP_AXIS_ALL;

            if (!GimbalApp_TryReadParam(&g_gimbal.params, param_id, record_axis, &applied_value))
            {
                GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BAD_PARAM, NULL, 0U);
                return;
            }

            if (!GimbalApp_CanAppendParamRecord(offset, (uint8_t)sizeof(detail)))
            {
                GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BUFFER_TOO_SMALL, NULL, 0U);
                return;
            }

            offset = GimbalApp_AppendParamRecord(detail, offset, param_id, record_axis, applied_value);
        }
    }

    GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_OK, detail, offset);
    GimbalApp_RequestStatus();
}

static void GimbalApp_HandleParamGet(uint8_t msg_type, uint16_t frame_id, const uint8_t *payload, uint8_t payload_len)
{
    uint8_t detail[GIMBAL_APP_MAX_PAYLOAD_LEN - 2U];
    uint8_t offset = 0U;
    int32_t value;
    uint8_t index;
    static const uint8_t full_param_list[][2] =
    {
        { GIMBAL_APP_PARAM_DEADBAND, GIMBAL_APP_AXIS_PAN },
        { GIMBAL_APP_PARAM_MAX_STEP_US, GIMBAL_APP_AXIS_PAN },
        { GIMBAL_APP_PARAM_KP_NUM, GIMBAL_APP_AXIS_PAN },
        { GIMBAL_APP_PARAM_KP_DEN, GIMBAL_APP_AXIS_PAN },
        { GIMBAL_APP_PARAM_CENTER_US, GIMBAL_APP_AXIS_PAN },
        { GIMBAL_APP_PARAM_HOME_US, GIMBAL_APP_AXIS_PAN },
        { GIMBAL_APP_PARAM_MIN_US, GIMBAL_APP_AXIS_PAN },
        { GIMBAL_APP_PARAM_MAX_US, GIMBAL_APP_AXIS_PAN },
        { GIMBAL_APP_PARAM_INVERT, GIMBAL_APP_AXIS_PAN },
        { GIMBAL_APP_PARAM_KALMAN_ENABLE, GIMBAL_APP_AXIS_PAN },
        { GIMBAL_APP_PARAM_DEADBAND, GIMBAL_APP_AXIS_TILT },
        { GIMBAL_APP_PARAM_MAX_STEP_US, GIMBAL_APP_AXIS_TILT },
        { GIMBAL_APP_PARAM_KP_NUM, GIMBAL_APP_AXIS_TILT },
        { GIMBAL_APP_PARAM_KP_DEN, GIMBAL_APP_AXIS_TILT },
        { GIMBAL_APP_PARAM_CENTER_US, GIMBAL_APP_AXIS_TILT },
        { GIMBAL_APP_PARAM_HOME_US, GIMBAL_APP_AXIS_TILT },
        { GIMBAL_APP_PARAM_MIN_US, GIMBAL_APP_AXIS_TILT },
        { GIMBAL_APP_PARAM_MAX_US, GIMBAL_APP_AXIS_TILT },
        { GIMBAL_APP_PARAM_INVERT, GIMBAL_APP_AXIS_TILT },
        { GIMBAL_APP_PARAM_KALMAN_ENABLE, GIMBAL_APP_AXIS_TILT },
        { GIMBAL_APP_PARAM_STATUS_PERIOD_MS, GIMBAL_APP_AXIS_ALL },
        { GIMBAL_APP_PARAM_TARGET_TIMEOUT_MS, GIMBAL_APP_AXIS_ALL },
        { GIMBAL_APP_PARAM_BOOT_CENTER_MS, GIMBAL_APP_AXIS_ALL },
        { GIMBAL_APP_PARAM_KALMAN_Q_MILLI, GIMBAL_APP_AXIS_ALL },
        { GIMBAL_APP_PARAM_KALMAN_R_MILLI, GIMBAL_APP_AXIS_ALL },
        { GIMBAL_APP_PARAM_PREDICT_ENABLE, GIMBAL_APP_AXIS_ALL },
        { GIMBAL_APP_PARAM_PREDICT_LEAD_MS, GIMBAL_APP_AXIS_ALL },
        { GIMBAL_APP_PARAM_PREDICT_VEL_TC_MS, GIMBAL_APP_AXIS_ALL }
    };

    if ((payload_len % 2U) != 0U)
    {
        GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BAD_LENGTH, NULL, 0U);
        return;
    }

    if (payload_len == 0U)
    {
        for (index = 0U; index < (uint8_t)(sizeof(full_param_list) / sizeof(full_param_list[0])); ++index)
        {
            if (!GimbalApp_TryReadParam(&g_gimbal.params, full_param_list[index][0], full_param_list[index][1], &value))
            {
                GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BAD_PARAM, NULL, 0U);
                return;
            }

            if (!GimbalApp_CanAppendParamRecord(offset, (uint8_t)sizeof(detail)))
            {
                GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BUFFER_TOO_SMALL, NULL, 0U);
                return;
            }

            offset = GimbalApp_AppendParamRecord(
                detail,
                offset,
                full_param_list[index][0],
                full_param_list[index][1],
                value);
        }
    }
    else
    {
        for (index = 0U; index < payload_len; index = (uint8_t)(index + 2U))
        {
            if (!GimbalApp_TryReadParam(&g_gimbal.params, payload[index], payload[index + 1U], &value))
            {
                GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BAD_PARAM, NULL, 0U);
                return;
            }

            if (!GimbalApp_CanAppendParamRecord(offset, (uint8_t)sizeof(detail)))
            {
                GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BUFFER_TOO_SMALL, NULL, 0U);
                return;
            }

            offset = GimbalApp_AppendParamRecord(detail, offset, payload[index], payload[index + 1U], value);
        }
    }

    GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_OK, detail, offset);
}

static void GimbalApp_HandleControlCmd(uint8_t msg_type, uint16_t frame_id, const uint8_t *payload, uint8_t payload_len)
{
    uint8_t detail[2];
    uint8_t cmd_id;
    uint8_t arg0 = 0U;

    if (payload_len == 0U)
    {
        GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BAD_LENGTH, NULL, 0U);
        return;
    }

    cmd_id = payload[0];
    if (payload_len >= 2U)
    {
        arg0 = payload[1];
    }

    switch (cmd_id)
    {
    case GIMBAL_APP_CTRL_GO_HOME:
        GimbalApp_ClearTargetValidity();
        GimbalApp_SetOutputsToHome();
        GimbalApp_SetState(GIMBAL_APP_STATE_STANDBY);
        break;

    case GIMBAL_APP_CTRL_SET_STANDBY:
        GimbalApp_ClearTargetValidity();
        GimbalApp_SetOutputsToHome();
        GimbalApp_SetState(GIMBAL_APP_STATE_STANDBY);
        break;

    case GIMBAL_APP_CTRL_SET_HOLD_LAST:
        GimbalApp_SetState(GIMBAL_APP_STATE_HOLD_LAST);
        break;

    case GIMBAL_APP_CTRL_CLEAR_TARGET:
        GimbalApp_ClearTargetValidity();
        if (g_gimbal.state != GIMBAL_APP_STATE_BOOT_CENTERING)
        {
            GimbalApp_SetState(GIMBAL_APP_STATE_HOLD_LAST);
        }
        break;

    case GIMBAL_APP_CTRL_FORCE_STATUS:
        GimbalApp_RequestStatus();
        break;

    case GIMBAL_APP_CTRL_SET_STATE:
        if (arg0 == (uint8_t)GIMBAL_APP_STATE_STANDBY)
        {
            GimbalApp_ClearTargetValidity();
            GimbalApp_SetOutputsToHome();
            GimbalApp_SetState(GIMBAL_APP_STATE_STANDBY);
        }
        else if (arg0 == (uint8_t)GIMBAL_APP_STATE_HOLD_LAST)
        {
            GimbalApp_SetState(GIMBAL_APP_STATE_HOLD_LAST);
        }
        else if (arg0 == (uint8_t)GIMBAL_APP_STATE_TRACKING)
        {
            if (g_gimbal.latest_track.valid)
            {
                GimbalApp_SetState(GIMBAL_APP_STATE_TRACKING);
            }
            else
            {
                GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BAD_STATE, NULL, 0U);
                return;
            }
        }
        else
        {
            GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BAD_STATE, NULL, 0U);
            return;
        }
        break;

    default:
        GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BAD_CMD, NULL, 0U);
        return;
    }

    detail[0] = cmd_id;
    detail[1] = (uint8_t)g_gimbal.state;
    GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_OK, detail, (uint8_t)sizeof(detail));
}

static void GimbalApp_HandleParsedFrame(uint8_t msg_type, uint8_t flags, uint16_t frame_id, uint32_t capture_ts_ms, const uint8_t *payload, uint8_t payload_len)
{
    switch (msg_type)
    {
    case GIMBAL_APP_MSG_TRACK:
        if (!GimbalApp_HandleTrackFrame(frame_id, capture_ts_ms, flags, payload, payload_len))
        {
            if ((flags & GIMBAL_APP_FLAG_ACK_REQUEST) != 0U)
            {
                GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BAD_LENGTH, NULL, 0U);
            }
            break;
        }
        if ((flags & GIMBAL_APP_FLAG_ACK_REQUEST) != 0U)
        {
            GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_OK, NULL, 0U);
        }
        if ((flags & GIMBAL_APP_FLAG_STATUS_REQUEST) != 0U)
        {
            GimbalApp_RequestStatus();
        }
        break;

    case GIMBAL_APP_MSG_PARAM_SET:
        GimbalApp_HandleParamSet(msg_type, frame_id, payload, payload_len);
        break;

    case GIMBAL_APP_MSG_PARAM_GET:
        GimbalApp_HandleParamGet(msg_type, frame_id, payload, payload_len);
        break;

    case GIMBAL_APP_MSG_CONTROL_CMD:
        GimbalApp_HandleControlCmd(msg_type, frame_id, payload, payload_len);
        break;

    case GIMBAL_APP_MSG_STATUS:
    case GIMBAL_APP_MSG_ACK:
        if ((flags & GIMBAL_APP_FLAG_ACK_REQUEST) != 0U)
        {
            GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BAD_MSG, NULL, 0U);
        }
        break;

    default:
        GimbalApp_QueueAck(msg_type, frame_id, GIMBAL_APP_ACK_BAD_MSG, NULL, 0U);
        break;
    }
}

static void GimbalApp_QueueAck(uint8_t acked_msg_type, uint16_t frame_id, GimbalAppAckCode ack_code, const uint8_t *detail, uint8_t detail_len)
{
    GimbalPendingAck *pending_ack;

    if (detail_len > (uint8_t)sizeof(g_gimbal.ack_queue.items[0].detail))
    {
        detail_len = (uint8_t)sizeof(g_gimbal.ack_queue.items[0].detail);
    }

    if (g_gimbal.ack_queue.count >= GIMBAL_APP_ACK_QUEUE_LEN)
    {
        g_gimbal.ack_overflow = true;
        g_gimbal.pending_status = true;
        DebugUart_WriteString("ack queue overflow\r\n");
        return;
    }

    pending_ack = &g_gimbal.ack_queue.items[g_gimbal.ack_queue.head];
    memset(pending_ack, 0, sizeof(*pending_ack));
    pending_ack->valid = true;
    pending_ack->acked_msg_type = acked_msg_type;
    pending_ack->frame_id = frame_id;
    pending_ack->ack_code = ack_code;
    pending_ack->detail_len = detail_len;

    if ((detail_len > 0U) && (detail != NULL))
    {
        memcpy(pending_ack->detail, detail, detail_len);
    }

    g_gimbal.ack_queue.head = (uint8_t)((g_gimbal.ack_queue.head + 1U) % GIMBAL_APP_ACK_QUEUE_LEN);
    g_gimbal.ack_queue.count++;
}

static uint16_t GimbalApp_PackFrame(uint8_t msg_type, uint8_t flags, uint16_t frame_id, uint32_t capture_ts_ms, const uint8_t *payload, uint8_t payload_len, uint8_t *out_buf, uint16_t buf_size)
{
    uint16_t total_len = (uint16_t)(GIMBAL_APP_FRAME_OVERHEAD + payload_len);
    uint16_t index = 0U;
    uint8_t checksum = 0U;
    uint8_t payload_index;

    if ((out_buf == NULL) || (payload_len > GIMBAL_APP_MAX_PAYLOAD_LEN) || (buf_size < total_len))
    {
        return 0U;
    }

    out_buf[index++] = GIMBAL_APP_PROTO_SOF0;
    out_buf[index++] = GIMBAL_APP_PROTO_SOF1;
    out_buf[index++] = msg_type;
    checksum ^= msg_type;
    out_buf[index++] = flags;
    checksum ^= flags;
    out_buf[index++] = payload_len;
    checksum ^= payload_len;
    out_buf[index++] = (uint8_t)(frame_id & 0xFFU);
    checksum ^= out_buf[index - 1U];
    out_buf[index++] = (uint8_t)((frame_id >> 8) & 0xFFU);
    checksum ^= out_buf[index - 1U];
    out_buf[index++] = (uint8_t)(capture_ts_ms & 0xFFU);
    checksum ^= out_buf[index - 1U];
    out_buf[index++] = (uint8_t)((capture_ts_ms >> 8) & 0xFFU);
    checksum ^= out_buf[index - 1U];
    out_buf[index++] = (uint8_t)((capture_ts_ms >> 16) & 0xFFU);
    checksum ^= out_buf[index - 1U];
    out_buf[index++] = (uint8_t)((capture_ts_ms >> 24) & 0xFFU);
    checksum ^= out_buf[index - 1U];

    for (payload_index = 0U; payload_index < payload_len; ++payload_index)
    {
        out_buf[index++] = payload[payload_index];
        checksum ^= payload[payload_index];
    }

    out_buf[index++] = checksum;
    out_buf[index++] = GIMBAL_APP_PROTO_TAIL;

    return index;
}

static bool GimbalApp_TrySetParam(GimbalAppParams *params, uint8_t param_id, uint8_t axis_id, int32_t value)
{
    uint8_t axis_start;
    uint8_t axis_end;

    if (params == NULL)
    {
        return false;
    }

    if ((axis_id == GIMBAL_APP_AXIS_ALL) &&
        (param_id != GIMBAL_APP_PARAM_STATUS_PERIOD_MS) &&
        (param_id != GIMBAL_APP_PARAM_TARGET_TIMEOUT_MS) &&
        (param_id != GIMBAL_APP_PARAM_BOOT_CENTER_MS) &&
        (param_id != GIMBAL_APP_PARAM_KALMAN_Q_MILLI) &&
        (param_id != GIMBAL_APP_PARAM_KALMAN_R_MILLI) &&
        (param_id != GIMBAL_APP_PARAM_PREDICT_ENABLE) &&
        (param_id != GIMBAL_APP_PARAM_PREDICT_LEAD_MS) &&
        (param_id != GIMBAL_APP_PARAM_PREDICT_VEL_TC_MS))
    {
        axis_start = 0U;
        axis_end = GIMBAL_APP_AXIS_COUNT;
    }
    else if (axis_id < GIMBAL_APP_AXIS_COUNT)
    {
        axis_start = axis_id;
        axis_end = (uint8_t)(axis_id + 1U);
    }
    else
    {
        axis_start = 0U;
        axis_end = 0U;
    }

    switch (param_id)
    {
    case GIMBAL_APP_PARAM_DEADBAND:
    case GIMBAL_APP_PARAM_MAX_STEP_US:
    case GIMBAL_APP_PARAM_KP_NUM:
    case GIMBAL_APP_PARAM_KP_DEN:
    case GIMBAL_APP_PARAM_CENTER_US:
    case GIMBAL_APP_PARAM_HOME_US:
    case GIMBAL_APP_PARAM_MIN_US:
    case GIMBAL_APP_PARAM_MAX_US:
    case GIMBAL_APP_PARAM_INVERT:
    case GIMBAL_APP_PARAM_KALMAN_ENABLE:
        if (axis_end == axis_start)
        {
            return false;
        }
        break;

    case GIMBAL_APP_PARAM_STATUS_PERIOD_MS:
        if (value < 0)
        {
            return false;
        }
        params->status_period_ms = (uint32_t)value;
        return true;

    case GIMBAL_APP_PARAM_TARGET_TIMEOUT_MS:
        if (value <= 0)
        {
            return false;
        }
        params->target_timeout_ms = (uint32_t)value;
        return true;

    case GIMBAL_APP_PARAM_BOOT_CENTER_MS:
        if (value < 0)
        {
            return false;
        }
        params->boot_center_ms = (uint32_t)value;
        return true;

    case GIMBAL_APP_PARAM_KALMAN_Q_MILLI:
        if (value < 0)
        {
            return false;
        }
        params->kalman_q_milli = (uint32_t)value;
        return true;

    case GIMBAL_APP_PARAM_KALMAN_R_MILLI:
        if (value <= 0)
        {
            return false;
        }
        params->kalman_r_milli = (uint32_t)value;
        return true;

    case GIMBAL_APP_PARAM_PREDICT_ENABLE:
        if ((value != 0) && (value != 1))
        {
            return false;
        }
        params->predict_enable = (uint8_t)value;
        return true;

    case GIMBAL_APP_PARAM_PREDICT_LEAD_MS:
        if (value < 0)
        {
            return false;
        }
        params->predict_lead_ms = (uint32_t)value;
        return true;

    case GIMBAL_APP_PARAM_PREDICT_VEL_TC_MS:
        if (value < 0)
        {
            return false;
        }
        params->predict_vel_tc_ms = (uint32_t)value;
        return true;

    default:
        return false;
    }

    for (; axis_start < axis_end; ++axis_start)
    {
        GimbalAppAxisParams *axis = &params->axis[axis_start];

        switch (param_id)
        {
        case GIMBAL_APP_PARAM_DEADBAND:
            if (value < 0)
            {
                return false;
            }
            axis->deadband = (uint16_t)value;
            break;

        case GIMBAL_APP_PARAM_MAX_STEP_US:
            if (value <= 0)
            {
                return false;
            }
            axis->max_step_us = (uint16_t)value;
            break;

        case GIMBAL_APP_PARAM_KP_NUM:
            if (value < 0)
            {
                return false;
            }
            axis->kp_num = (uint16_t)value;
            break;

        case GIMBAL_APP_PARAM_KP_DEN:
            if (value <= 0)
            {
                return false;
            }
            axis->kp_den = (uint16_t)value;
            break;

        case GIMBAL_APP_PARAM_CENTER_US:
            if ((value < (int32_t)GIMBAL_APP_SERVO_PULSE_MIN_US) ||
                (value > (int32_t)GIMBAL_APP_SERVO_PULSE_MAX_US))
            {
                return false;
            }
            axis->center_us = (uint16_t)value;
            break;

        case GIMBAL_APP_PARAM_HOME_US:
            if ((value < (int32_t)GIMBAL_APP_SERVO_PULSE_MIN_US) ||
                (value > (int32_t)GIMBAL_APP_SERVO_PULSE_MAX_US))
            {
                return false;
            }
            axis->home_us = (uint16_t)value;
            break;

        case GIMBAL_APP_PARAM_MIN_US:
            if ((value < (int32_t)GIMBAL_APP_SERVO_PULSE_MIN_US) ||
                (value > (int32_t)GIMBAL_APP_SERVO_PULSE_MAX_US))
            {
                return false;
            }
            axis->min_us = (uint16_t)value;
            break;

        case GIMBAL_APP_PARAM_MAX_US:
            if ((value < (int32_t)GIMBAL_APP_SERVO_PULSE_MIN_US) ||
                (value > (int32_t)GIMBAL_APP_SERVO_PULSE_MAX_US))
            {
                return false;
            }
            axis->max_us = (uint16_t)value;
            break;

        case GIMBAL_APP_PARAM_INVERT:
            if ((value != 0) && (value != 1))
            {
                return false;
            }
            axis->invert = (uint8_t)value;
            break;

        case GIMBAL_APP_PARAM_KALMAN_ENABLE:
            if ((value != 0) && (value != 1))
            {
                return false;
            }
            axis->kalman_enable = (uint8_t)value;
            break;

        default:
            return false;
        }
    }

    return true;
}

static bool GimbalApp_TryReadParam(const GimbalAppParams *params, uint8_t param_id, uint8_t axis_id, int32_t *value_out)
{
    const GimbalAppAxisParams *axis;

    if ((params == NULL) || (value_out == NULL))
    {
        return false;
    }

    switch (param_id)
    {
    case GIMBAL_APP_PARAM_STATUS_PERIOD_MS:
        *value_out = (int32_t)params->status_period_ms;
        return true;

    case GIMBAL_APP_PARAM_TARGET_TIMEOUT_MS:
        *value_out = (int32_t)params->target_timeout_ms;
        return true;

    case GIMBAL_APP_PARAM_BOOT_CENTER_MS:
        *value_out = (int32_t)params->boot_center_ms;
        return true;

    case GIMBAL_APP_PARAM_KALMAN_Q_MILLI:
        *value_out = (int32_t)params->kalman_q_milli;
        return true;

    case GIMBAL_APP_PARAM_KALMAN_R_MILLI:
        *value_out = (int32_t)params->kalman_r_milli;
        return true;

    case GIMBAL_APP_PARAM_PREDICT_ENABLE:
        *value_out = params->predict_enable;
        return true;

    case GIMBAL_APP_PARAM_PREDICT_LEAD_MS:
        *value_out = (int32_t)params->predict_lead_ms;
        return true;

    case GIMBAL_APP_PARAM_PREDICT_VEL_TC_MS:
        *value_out = (int32_t)params->predict_vel_tc_ms;
        return true;

    default:
        break;
    }

    if (axis_id >= GIMBAL_APP_AXIS_COUNT)
    {
        return false;
    }

    axis = &params->axis[axis_id];

    switch (param_id)
    {
    case GIMBAL_APP_PARAM_DEADBAND:
        *value_out = axis->deadband;
        return true;

    case GIMBAL_APP_PARAM_MAX_STEP_US:
        *value_out = axis->max_step_us;
        return true;

    case GIMBAL_APP_PARAM_KP_NUM:
        *value_out = axis->kp_num;
        return true;

    case GIMBAL_APP_PARAM_KP_DEN:
        *value_out = axis->kp_den;
        return true;

    case GIMBAL_APP_PARAM_CENTER_US:
        *value_out = axis->center_us;
        return true;

    case GIMBAL_APP_PARAM_HOME_US:
        *value_out = axis->home_us;
        return true;

    case GIMBAL_APP_PARAM_MIN_US:
        *value_out = axis->min_us;
        return true;

    case GIMBAL_APP_PARAM_MAX_US:
        *value_out = axis->max_us;
        return true;

    case GIMBAL_APP_PARAM_INVERT:
        *value_out = axis->invert;
        return true;

    case GIMBAL_APP_PARAM_KALMAN_ENABLE:
        *value_out = axis->kalman_enable;
        return true;

    default:
        return false;
    }
}

static uint8_t GimbalApp_AppendParamRecord(uint8_t *dst, uint8_t offset, uint8_t param_id, uint8_t axis_id, int32_t value)
{
    dst[offset] = param_id;
    dst[offset + 1U] = axis_id;
    GimbalApp_WriteLe32(&dst[offset + 2U], (uint32_t)value);
    return (uint8_t)(offset + 6U);
}

static int32_t GimbalApp_ReadLe32(const uint8_t *src)
{
    return (int32_t)(
        ((uint32_t)src[0]) |
        ((uint32_t)src[1] << 8) |
        ((uint32_t)src[2] << 16) |
        ((uint32_t)src[3] << 24));
}

static uint16_t GimbalApp_ReadLe16(const uint8_t *src)
{
    return (uint16_t)(((uint16_t)src[0]) | ((uint16_t)src[1] << 8));
}

static void GimbalApp_WriteLe16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void GimbalApp_WriteLe32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static int16_t GimbalApp_ReadLe16s(const uint8_t *src)
{
    return (int16_t)GimbalApp_ReadLe16(src);
}

static uint16_t GimbalApp_AbsI16(int16_t value)
{
    return (uint16_t)((value < 0) ? -value : value);
}

static float GimbalApp_ParamMilliToFloat(uint32_t value_milli)
{
    return ((float)value_milli) / 1000.0f;
}

static int16_t GimbalApp_FloatToI16(float value)
{
    int32_t rounded;

    if (value >= 0.0f)
    {
        rounded = (int32_t)(value + 0.5f);
    }
    else
    {
        rounded = (int32_t)(value - 0.5f);
    }

    if (rounded > 32767)
    {
        return 32767;
    }

    if (rounded < -32768)
    {
        return -32768;
    }

    return (int16_t)rounded;
}

static float GimbalApp_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static uint32_t GimbalApp_ResolveTrackDtMs(const GimbalAxisPredictState *axis_state, uint32_t capture_ts_ms, uint32_t rx_tick_ms)
{
    uint32_t capture_dt_ms;
    uint32_t rx_dt_ms;

    if ((axis_state == NULL) || (!axis_state->initialized))
    {
        return 0U;
    }

    capture_dt_ms = capture_ts_ms - axis_state->last_capture_ts_ms;
    if ((capture_dt_ms > 0U) && (capture_dt_ms <= GIMBAL_APP_PREDICT_DT_MAX_MS))
    {
        return capture_dt_ms;
    }

    rx_dt_ms = rx_tick_ms - axis_state->last_rx_tick_ms;
    if ((rx_dt_ms > 0U) && (rx_dt_ms <= GIMBAL_APP_PREDICT_DT_MAX_MS))
    {
        return rx_dt_ms;
    }

    return 0U;
}

static void GimbalApp_ResetKalmanFilters(void)
{
    memset(g_kalman, 0, sizeof(g_kalman));
}

static void GimbalApp_ResetPredictorStates(void)
{
    memset(g_predictor, 0, sizeof(g_predictor));
}

static bool GimbalApp_DidObserverConfigChange(const GimbalAppParams *lhs, const GimbalAppParams *rhs)
{
    uint8_t axis_index;

    if ((lhs == NULL) || (rhs == NULL))
    {
        return false;
    }

    if ((lhs->kalman_q_milli != rhs->kalman_q_milli) ||
        (lhs->kalman_r_milli != rhs->kalman_r_milli) ||
        (lhs->predict_enable != rhs->predict_enable) ||
        (lhs->predict_lead_ms != rhs->predict_lead_ms) ||
        (lhs->predict_vel_tc_ms != rhs->predict_vel_tc_ms))
    {
        return true;
    }

    for (axis_index = 0U; axis_index < GIMBAL_APP_AXIS_COUNT; ++axis_index)
    {
        if (lhs->axis[axis_index].kalman_enable != rhs->axis[axis_index].kalman_enable)
        {
            return true;
        }
    }

    return false;
}

static int16_t GimbalApp_ApplyKalman(uint8_t axis_index, int16_t measurement)
{
    GimbalAxisKalmanState *axis_state;
    float measurement_value;
    float predicted_covariance;
    float kalman_gain;
    float r_value;

    if (axis_index >= GIMBAL_APP_AXIS_COUNT)
    {
        return measurement;
    }

    if (g_gimbal.params.axis[axis_index].kalman_enable == 0U)
    {
        return measurement;
    }

    axis_state = &g_kalman[axis_index];
    measurement_value = (float)measurement;
    r_value = GimbalApp_ParamMilliToFloat(g_gimbal.params.kalman_r_milli);

    if (!axis_state->initialized)
    {
        axis_state->initialized = true;
        axis_state->estimate = measurement_value;
        axis_state->covariance = r_value;
        return measurement;
    }

    predicted_covariance = axis_state->covariance + GimbalApp_ParamMilliToFloat(g_gimbal.params.kalman_q_milli);
    kalman_gain = predicted_covariance / (predicted_covariance + r_value);
    axis_state->estimate += kalman_gain * (measurement_value - axis_state->estimate);
    axis_state->covariance = (1.0f - kalman_gain) * predicted_covariance;

    return GimbalApp_FloatToI16(axis_state->estimate);
}

static void GimbalApp_UpdatePredictor(uint8_t axis_index, int16_t measurement, uint32_t capture_ts_ms, uint32_t rx_tick_ms)
{
    GimbalAxisPredictState *axis_state;
    uint32_t dt_ms;
    float raw_velocity;
    float alpha;
    float vel_tc_ms;
    float measurement_value;

    if (axis_index >= GIMBAL_APP_AXIS_COUNT)
    {
        return;
    }

    axis_state = &g_predictor[axis_index];
    measurement_value = (float)measurement;

    if (!axis_state->initialized)
    {
        axis_state->initialized = true;
        axis_state->last_measurement = measurement_value;
        axis_state->velocity_err_per_s = 0.0f;
        axis_state->last_capture_ts_ms = capture_ts_ms;
        axis_state->last_rx_tick_ms = rx_tick_ms;
        return;
    }

    dt_ms = GimbalApp_ResolveTrackDtMs(axis_state, capture_ts_ms, rx_tick_ms);
    if (dt_ms > 0U)
    {
        raw_velocity = (measurement_value - axis_state->last_measurement) * 1000.0f / (float)dt_ms;
        raw_velocity = GimbalApp_ClampFloat(
            raw_velocity,
            -GIMBAL_APP_PREDICT_VEL_ABS_MAX_ERR_PER_S,
            GIMBAL_APP_PREDICT_VEL_ABS_MAX_ERR_PER_S);

        vel_tc_ms = (float)g_gimbal.params.predict_vel_tc_ms;
        if (vel_tc_ms <= 0.0f)
        {
            alpha = 1.0f;
        }
        else
        {
            alpha = ((float)dt_ms) / (vel_tc_ms + (float)dt_ms);
        }

        axis_state->velocity_err_per_s += alpha * (raw_velocity - axis_state->velocity_err_per_s);
    }

    axis_state->last_measurement = measurement_value;
    axis_state->last_capture_ts_ms = capture_ts_ms;
    axis_state->last_rx_tick_ms = rx_tick_ms;
}

static int16_t GimbalApp_GetControlErrorForAxis(uint8_t axis_index, uint32_t now_ms)
{
    const GimbalAxisPredictState *axis_state;
    int16_t base_err;
    uint32_t horizon_ms;
    float predicted_err;

    if (axis_index >= GIMBAL_APP_AXIS_COUNT)
    {
        return 0;
    }

    base_err = (axis_index == GIMBAL_APP_AXIS_PAN) ? g_gimbal.latest_track.err_x : g_gimbal.latest_track.err_y;

    if ((g_gimbal.params.predict_enable == 0U) || (!g_gimbal.latest_track.valid))
    {
        return base_err;
    }

    axis_state = &g_predictor[axis_index];
    if (!axis_state->initialized)
    {
        return base_err;
    }

    horizon_ms = now_ms - g_gimbal.latest_track.rx_tick_ms;
    horizon_ms += g_gimbal.params.predict_lead_ms;
    if (horizon_ms > GIMBAL_APP_PREDICT_HORIZON_MAX_MS)
    {
        horizon_ms = GIMBAL_APP_PREDICT_HORIZON_MAX_MS;
    }

    predicted_err = (float)base_err + (axis_state->velocity_err_per_s * ((float)horizon_ms / 1000.0f));
    return GimbalApp_FloatToI16(predicted_err);
}

static void GimbalApp_ProcessQueuedRx(void)
{
    uint8_t rx_chunk[64];
    uint16_t count = 0U;
    uint8_t byte;

    while (GimbalApp_RxFifoPop(&byte))
    {
        rx_chunk[count++] = byte;
        if (count >= (uint16_t)sizeof(rx_chunk))
        {
            GimbalApp_RxBytes(rx_chunk, count);
            count = 0U;
        }
    }

    if (count > 0U)
    {
        GimbalApp_RxBytes(rx_chunk, count);
    }
}

static bool GimbalApp_RxFifoPush(uint8_t byte)
{
    uint32_t primask = __get_PRIMASK();
    uint16_t next_head;
    bool pushed = false;

    __disable_irq();
    next_head = (uint16_t)((g_gimbal.rx_fifo_head + 1U) % GIMBAL_APP_RX_FIFO_LEN);
    if (next_head != g_gimbal.rx_fifo_tail)
    {
        g_gimbal.rx_fifo[g_gimbal.rx_fifo_head] = byte;
        g_gimbal.rx_fifo_head = next_head;
        pushed = true;
    }
    __set_PRIMASK(primask);

    return pushed;
}

static bool GimbalApp_RxFifoPop(uint8_t *byte)
{
    uint32_t primask = __get_PRIMASK();
    bool has_data = false;

    if (byte == NULL)
    {
        return false;
    }

    __disable_irq();
    if (g_gimbal.rx_fifo_head != g_gimbal.rx_fifo_tail)
    {
        *byte = g_gimbal.rx_fifo[g_gimbal.rx_fifo_tail];
        g_gimbal.rx_fifo_tail = (uint16_t)((g_gimbal.rx_fifo_tail + 1U) % GIMBAL_APP_RX_FIFO_LEN);
        has_data = true;
    }
    __set_PRIMASK(primask);

    return has_data;
}

static bool GimbalApp_CanAppendParamRecord(uint8_t offset, uint8_t capacity)
{
    return ((uint16_t)offset + 6U) <= capacity;
}

static bool GimbalApp_IsAxisParam(uint8_t param_id)
{
    switch (param_id)
    {
    case GIMBAL_APP_PARAM_DEADBAND:
    case GIMBAL_APP_PARAM_MAX_STEP_US:
    case GIMBAL_APP_PARAM_KP_NUM:
    case GIMBAL_APP_PARAM_KP_DEN:
    case GIMBAL_APP_PARAM_CENTER_US:
    case GIMBAL_APP_PARAM_HOME_US:
    case GIMBAL_APP_PARAM_MIN_US:
    case GIMBAL_APP_PARAM_MAX_US:
    case GIMBAL_APP_PARAM_INVERT:
    case GIMBAL_APP_PARAM_KALMAN_ENABLE:
        return true;

    default:
        return false;
    }
}

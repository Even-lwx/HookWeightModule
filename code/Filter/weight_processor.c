#include "weight_processor.h"
#include "weight_calibration.h"
#include <stddef.h>
#include <string.h>

#define MEDIAN_WINDOW_SIZE              3U
#define STABILITY_WINDOW_SIZE          10U
#define STABILITY_REQUIRED_SAMPLES     20U

/* 以下阈值按照传感器约538 counts/g的灵敏度设置。 */
#define FAST_ENTER_COUNTS            1076L  /* 约2.0 g，进入快速跟随模式 */
#define FAST_EXIT_COUNTS              161L  /* 约0.3 g，准备退出快速模式 */
#define FAST_EXIT_REQUIRED_SAMPLES      5U
#define STABLE_RANGE_COUNTS            108L  /* 稳定窗口极差约0.2 g */
#define STABLE_END_TO_END_COUNTS        24L  /* 0.9秒内变化约不超过0.05 g/s */
#define OVER_RANGE_MIN_X10                0L
#define OVER_RANGE_MAX_X10            10000L

typedef struct {
    int32_t median_values[MEDIAN_WINDOW_SIZE];
    int32_t stability_values[STABILITY_WINDOW_SIZE];
    int64_t filtered_q8;
    int32_t last_filtered_raw;
    uint32_t fallback_elapsed_ms;
    int32_t manual_tare_offset;
    int32_t current_profile_zero;
    uint8_t median_count;
    uint8_t median_index;
    uint8_t stability_count;
    uint8_t stability_index;
    uint8_t stable_qualified_count;
    uint8_t fast_exit_count;
    uint8_t filter_initialized;
    uint8_t fast_mode;
    uint8_t ready;
    uint8_t stable;
} WeightProcessor_State;

static WeightProcessor_State processor;

static int64_t Abs64(int64_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t Median3(int32_t a, int32_t b, int32_t c)
{
    int32_t temporary;

    /* 对连续3个采样值排序并返回中间值，用来抑制单点尖峰。 */
    if (a > b) {
        temporary = a;
        a = b;
        b = temporary;
    }
    if (b > c) {
        temporary = b;
        b = c;
        c = temporary;
    }
    if (a > b) {
        b = a;
    }
    return b;
}

static int32_t UpdateMedian(int32_t raw)
{
    processor.median_values[processor.median_index] = raw;
    processor.median_index = (uint8_t)((processor.median_index + 1U) %
                                       MEDIAN_WINDOW_SIZE);

    if (processor.median_count < MEDIAN_WINDOW_SIZE) {
        ++processor.median_count;
        return raw;
    }

    return Median3(processor.median_values[0],
                   processor.median_values[1],
                   processor.median_values[2]);
}

static int32_t UpdateAdaptiveIir(int32_t median_raw)
{
    int64_t target_q8;
    int64_t difference_q8;
    int64_t difference_counts;

    /* 使用Q8定点数保存滤波状态，避免引入浮点运算。 */
    target_q8 = (int64_t)median_raw * 256L;
    if (!processor.filter_initialized) {
        processor.filtered_q8 = target_q8;
        processor.filter_initialized = 1U;
    } else {
        difference_q8 = target_q8 - processor.filtered_q8;
        difference_counts = Abs64(difference_q8) / 256L;

        /* 变化较大时快速跟随真实重量阶跃；变化较小时加强平滑。 */
        if (difference_counts > FAST_ENTER_COUNTS) {
            processor.fast_mode = 1U;
            processor.fast_exit_count = 0U;
        } else if (processor.fast_mode) {
            if (difference_counts < FAST_EXIT_COUNTS) {
                if (processor.fast_exit_count < FAST_EXIT_REQUIRED_SAMPLES) {
                    ++processor.fast_exit_count;
                }
                if (processor.fast_exit_count >= FAST_EXIT_REQUIRED_SAMPLES) {
                    processor.fast_mode = 0U;
                    processor.fast_exit_count = 0U;
                }
            } else {
                processor.fast_exit_count = 0U;
            }
        }

        difference_q8 = target_q8 - processor.filtered_q8;
        if (processor.fast_mode) {
            processor.filtered_q8 += difference_q8 / 2L;
        } else {
            processor.filtered_q8 += difference_q8 / 4L;
        }
    }

    if (processor.filtered_q8 >= 0) {
        return (int32_t)((processor.filtered_q8 + 128L) / 256L);
    }
    return (int32_t)((processor.filtered_q8 - 128L) / 256L);
}

static void UpdateStability(int32_t filtered_raw)
{
    uint8_t i;
    uint8_t oldest_index;
    uint8_t newest_index;
    int32_t minimum;
    int32_t maximum;
    int32_t end_to_end;

    /* 检查最近10点的极差和首尾差，连续满足条件后判定为稳定。 */
    processor.stability_values[processor.stability_index] = filtered_raw;
    processor.stability_index = (uint8_t)((processor.stability_index + 1U) %
                                          STABILITY_WINDOW_SIZE);
    if (processor.stability_count < STABILITY_WINDOW_SIZE) {
        ++processor.stability_count;
        processor.stable = 0U;
        return;
    }

    minimum = processor.stability_values[0];
    maximum = processor.stability_values[0];
    for (i = 1U; i < STABILITY_WINDOW_SIZE; ++i) {
        if (processor.stability_values[i] < minimum) {
            minimum = processor.stability_values[i];
        }
        if (processor.stability_values[i] > maximum) {
            maximum = processor.stability_values[i];
        }
    }

    oldest_index = processor.stability_index;
    newest_index = (uint8_t)((processor.stability_index +
                              STABILITY_WINDOW_SIZE - 1U) %
                             STABILITY_WINDOW_SIZE);
    end_to_end = processor.stability_values[newest_index] -
                 processor.stability_values[oldest_index];
    if (end_to_end < 0) {
        end_to_end = -end_to_end;
    }

    if (((maximum - minimum) <= STABLE_RANGE_COUNTS) &&
        (end_to_end <= STABLE_END_TO_END_COUNTS) &&
        !processor.fast_mode) {
        if (processor.stable_qualified_count < STABILITY_REQUIRED_SAMPLES) {
            ++processor.stable_qualified_count;
        }
    } else {
        processor.stable_qualified_count = 0U;
    }
    processor.stable =
        (processor.stable_qualified_count >= STABILITY_REQUIRED_SAMPLES) ? 1U : 0U;
}

void WeightProcessor_Init(void)
{
    memset(&processor, 0, sizeof(processor));
    WeightCalibration_Init();
}

void WeightProcessor_Reset(void)
{
    WeightProcessor_Init();
}

uint8_t WeightProcessor_Update(int32_t raw, WeightProcessor_Result *result)
{
    uint8_t ready;

    ready = WeightProcessor_UpdateTimed(raw, processor.fallback_elapsed_ms,
                                        result);
    if (processor.fallback_elapsed_ms <= (0xFFFFFFFFUL - 100UL)) {
        processor.fallback_elapsed_ms += 100UL;
    }
    return ready;
}

uint8_t WeightProcessor_UpdateTimed(int32_t raw, uint32_t elapsed_ms,
                                    WeightProcessor_Result *result)
{
    int32_t median_raw;
    int32_t filtered_raw;
    int32_t weight_x10;

    if (result == NULL) {
        return 0U;
    }

    median_raw = UpdateMedian(raw);
    filtered_raw = UpdateAdaptiveIir(median_raw);
    processor.last_filtered_raw = filtered_raw;
    UpdateStability(filtered_raw);

    /* 只使用提前记录并写入 Flash 的空载零点。
     * 当前ADC值可能包含真实负载，因此不能反馈到零点计算中。 */
    processor.current_profile_zero =
        WeightCalibration_GetFlashZeroRaw(elapsed_ms);
    WeightCalibration_SetZeroRaw(processor.current_profile_zero +
                                 processor.manual_tare_offset);
    weight_x10 = WeightCalibration_Convert(filtered_raw);
    processor.ready = 1U;

    result->raw = raw;
    result->filtered_raw = filtered_raw;
    result->weight_x10 = weight_x10;
    result->ready = processor.ready;
    result->stable = processor.stable;
    result->over_range = ((weight_x10 < OVER_RANGE_MIN_X10) ||
                          (weight_x10 > OVER_RANGE_MAX_X10)) ? 1U : 0U;

    return processor.ready;
}

uint8_t WeightProcessor_Tare(void)
{
    if (!processor.ready || !processor.stable) {
        return 0U;
    }

    /* 手动去皮值作为固定偏移叠加在 Flash 动态零点曲线上。
     * 复位或重新上电后自动清除本次手动去皮值。 */
    processor.manual_tare_offset = processor.last_filtered_raw -
                                   processor.current_profile_zero;
    WeightCalibration_SetZeroRaw(processor.last_filtered_raw);
    return 1U;
}

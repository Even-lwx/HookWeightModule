#include "weight_calibration.h"

#define WEIGHT_ZERO_NODE_COUNT 21U

/* ARMCC 会将 const 常量表放入 STM32 程序 Flash。
 * 前90秒的数据来自三次竖挂空载上电日志在相同时间点的中值；
 * 90～300秒按照约29 counts/s的中值漂移速度外推，300秒后保持不变。
 * 设备正常工作时上电已经带载，因此禁止使用当前ADC值自动替换零点曲线。 */
typedef struct {
    uint32_t time_ms;
    int32_t zero_raw;
} WeightZeroNode;

static const WeightZeroNode flash_zero_profile[WEIGHT_ZERO_NODE_COUNT] = {
    /* 零点曲线使用已经滤波的 I1 通道，与重量换算函数的输入保持一致。
     * 正常工作时袋子始终挂在传感器上，因此在空钩曲线上叠加实测的
     * 袋子偏移量，约6200 counts。 */
    {     0UL, 16223L}, {  1000UL, 15286L}, {  2000UL, 15200L},
    {  3000UL, 15155L}, {  4000UL, 15099L}, {  5000UL, 15053L},
    {  6000UL, 15048L}, {  8000UL, 14953L}, { 10000UL, 14966L},
    { 15000UL, 15296L}, { 20000UL, 15424L}, { 30000UL, 15614L},
    { 45000UL, 16016L}, { 60000UL, 16319L}, { 75000UL, 16606L},
    { 90000UL, 16800L}, {120000UL, 17670L}, {150000UL, 18540L},
    {180000UL, 19410L}, {220000UL, 20570L}, {300000UL, 22890L}
};

/* 使用竖挂带袋子的0、50、100、200、500、700和1000克数据，
 * 通过最小二乘法拟合一条全局直线：
 *
 *     weight_x10 = (raw - zero) * 100000 / 5438909
 *
 * 传感器灵敏度为543.8909 counts/g。使用整数系数避免浮点运算，
 * 同时比直接使用544 counts/g保留更多计算精度。 */
#define LINEAR_WEIGHT_NUMERATOR      100000L
#define LINEAR_COUNTS_DENOMINATOR   5357109L

static int32_t zero_shift_counts;

/* 最终重量的全局截距，单位为克，可在其他文件中直接赋值修改。 */
float g_weight_output_offset_g = 110.0f;

static int32_t WeightOffsetToX10(void)
{
    float offset_x10;

    offset_x10 = g_weight_output_offset_g * 10.0f;
    if (offset_x10 >= 0.0f) {
        return (int32_t)(offset_x10 + 0.5f);
    }
    return (int32_t)(offset_x10 - 0.5f);
}

static int32_t DivideRoundSigned(int64_t numerator, int32_t denominator)
{
    if (numerator >= 0) {
        numerator += (int64_t)denominator / 2;
    } else {
        numerator -= (int64_t)denominator / 2;
    }
    return (int32_t)(numerator / denominator);
}

void WeightCalibration_Init(void)
{
    WeightCalibration_SetZeroRaw(flash_zero_profile[0].zero_raw);
}

void WeightCalibration_SetZeroShift(int32_t shift_counts)
{
    zero_shift_counts = shift_counts;
}

int32_t WeightCalibration_GetZeroShift(void)
{
    return zero_shift_counts;
}

int32_t WeightCalibration_GetZeroRaw(void)
{
    return zero_shift_counts;
}

void WeightCalibration_SetZeroRaw(int32_t zero_raw_counts)
{
    zero_shift_counts = zero_raw_counts;
}

int32_t WeightCalibration_GetFlashZeroRaw(uint32_t elapsed_ms)
{
    uint32_t i;
    uint32_t time_span;
    uint32_t time_delta;
    int32_t zero_span;
    int64_t scaled;

    if (elapsed_ms <= flash_zero_profile[0].time_ms) {
        return flash_zero_profile[0].zero_raw;
    }
    if (elapsed_ms >= flash_zero_profile[WEIGHT_ZERO_NODE_COUNT - 1U].time_ms) {
        return flash_zero_profile[WEIGHT_ZERO_NODE_COUNT - 1U].zero_raw;
    }

    i = 0U;
    while ((i < (WEIGHT_ZERO_NODE_COUNT - 2U)) &&
           (elapsed_ms > flash_zero_profile[i + 1U].time_ms)) {
        ++i;
    }

    time_span = flash_zero_profile[i + 1U].time_ms -
                flash_zero_profile[i].time_ms;
    time_delta = elapsed_ms - flash_zero_profile[i].time_ms;
    zero_span = flash_zero_profile[i + 1U].zero_raw -
                flash_zero_profile[i].zero_raw;
    scaled = (int64_t)zero_span * (int64_t)time_delta;
    if (scaled >= 0) {
        scaled += (int64_t)time_span / 2L;
    } else {
        scaled -= (int64_t)time_span / 2L;
    }
    return flash_zero_profile[i].zero_raw + (int32_t)(scaled / time_span);
}

int32_t WeightCalibration_Convert(int32_t raw_counts)
{
    int64_t scaled;

    scaled = (int64_t)(raw_counts - zero_shift_counts) *
             (int64_t)LINEAR_WEIGHT_NUMERATOR;
    return DivideRoundSigned(scaled, LINEAR_COUNTS_DENOMINATOR) +
           WeightOffsetToX10();
}

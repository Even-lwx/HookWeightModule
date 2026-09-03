#include "weight_calibration.h"
#include "stm32g0xx_hal.h"
#include <string.h>

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
#define CAL_FLASH_ADDR              0x0800F800UL
#define CAL_MAGIC                   0x5743414CUL
#define CAL_VERSION                 1U

static int32_t zero_shift_counts;
float g_weight_slope_g_per_count = (float)LINEAR_WEIGHT_NUMERATOR /
                                   (10.0f * (float)LINEAR_COUNTS_DENOMINATOR);
static uint8_t manual_valid;
static uint8_t flash_valid;

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
    g_weight_slope_g_per_count = (float)LINEAR_WEIGHT_NUMERATOR /
                                  (10.0f * (float)LINEAR_COUNTS_DENOMINATOR);
    g_weight_output_offset_g = 110.0f;
    manual_valid = 0U;
    flash_valid = WeightCalibration_Load();
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

void WeightCalibration_SetParameters(int32_t zero_raw, float slope, float offset)
{
    zero_shift_counts = zero_raw;
    g_weight_slope_g_per_count = slope;
    g_weight_output_offset_g = offset;
    manual_valid = 1U;
}

float WeightCalibration_GetSlope(void) { return g_weight_slope_g_per_count; }
float WeightCalibration_GetOffset(void) { return g_weight_output_offset_g; }
uint8_t WeightCalibration_IsManual(void) { return manual_valid; }
uint8_t WeightCalibration_FlashValid(void) { return flash_valid; }

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
    float value = ((float)(raw_counts - zero_shift_counts) * g_weight_slope_g_per_count +
                   g_weight_output_offset_g) * 10.0f;
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

/* Flash 参数镜像，末尾 CRC 用于检测掉电写入或数据损坏。 */
typedef struct { uint32_t magic; uint16_t version; uint16_t size; int32_t zero;
                float slope; float offset; uint32_t manual; uint32_t crc; } CalFlash;

static uint32_t CalCrc(const CalFlash *p)
{
    /* 使用轻量级滚动校验，避免引入额外 CRC 表和较大代码。 */
    const uint8_t *b = (const uint8_t *)p; uint32_t c = 0xA5A5A5A5UL; uint32_t i;
    for (i = 0U; i < (uint32_t)(sizeof(CalFlash) - sizeof(uint32_t)); ++i) c = (c << 5) ^ (c >> 27) ^ b[i];
    return c;
}

uint8_t WeightCalibration_Save(void)
{
    /* STM32G0 按 64 位 double-word 编程，先擦除最后一页再连续写入。 */
    CalFlash p; uint32_t i, page_error; HAL_StatusTypeDef st; FLASH_EraseInitTypeDef erase;
    p.magic = CAL_MAGIC; p.version = CAL_VERSION; p.size = sizeof(CalFlash);
    p.zero = zero_shift_counts; p.slope = g_weight_slope_g_per_count;
    p.offset = g_weight_output_offset_g; p.manual = manual_valid; p.crc = CalCrc(&p);
    erase.TypeErase = FLASH_TYPEERASE_PAGES; erase.Banks = FLASH_BANK_1; erase.Page = 31U; erase.NbPages = 1U;
    HAL_FLASH_Unlock();
    st = HAL_FLASHEx_Erase(&erase, &page_error);
    if (st == HAL_OK) for (i = 0U; i < sizeof(CalFlash); i += 8U)
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CAL_FLASH_ADDR + i, *(uint64_t *)((uint8_t *)&p + i)) != HAL_OK) st = HAL_ERROR;
    HAL_FLASH_Lock(); flash_valid = (st == HAL_OK); return (st == HAL_OK) ? 1U : 0U;
}

uint8_t WeightCalibration_Load(void)
{
    /* 启动时只接受 magic、版本、长度、CRC 和斜率都有效的参数。 */
    const CalFlash *p = (const CalFlash *)CAL_FLASH_ADDR;
    if (p->magic != CAL_MAGIC || p->version != CAL_VERSION || p->size != sizeof(CalFlash) || p->crc != CalCrc(p) || p->slope <= 0.0f) return 0U;
    zero_shift_counts = p->zero; g_weight_slope_g_per_count = p->slope;
    g_weight_output_offset_g = p->offset; manual_valid = p->manual ? 1U : 0U; return 1U;
}

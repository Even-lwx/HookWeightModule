#ifndef WEIGHT_PROCESSOR_H
#define WEIGHT_PROCESSOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t raw;
    int32_t filtered_raw;
    int32_t weight_x10;
    uint8_t ready;
    uint8_t stable;
    uint8_t over_range;
} WeightProcessor_Result;

void WeightProcessor_Init(void);
void WeightProcessor_Reset(void);

/* 结果中包含可用的三通道数据时返回1，否则返回0。 */
uint8_t WeightProcessor_Update(int32_t raw, WeightProcessor_Result *result);

/* 推荐使用此接口。elapsed_ms 为单片机上电后的毫秒数，
 * 仅用于选择 Flash 中保存的空载零点曲线。 */
uint8_t WeightProcessor_UpdateTimed(int32_t raw, uint32_t elapsed_ms,
                                    WeightProcessor_Result *result);

/* 只有数据已经准备好并且处于稳定状态时，才允许手动去皮。 */
uint8_t WeightProcessor_Tare(void);
const WeightProcessor_Result *WeightProcessor_GetLastResult(void);
/* 放宽后的校准取点稳定判定，仅供串口校准使用。 */
uint8_t WeightProcessor_IsCalibrationStable(void);

#ifdef __cplusplus
}
#endif

#endif

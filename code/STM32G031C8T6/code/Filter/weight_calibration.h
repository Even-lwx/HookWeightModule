#ifndef WEIGHT_CALIBRATION_H
#define WEIGHT_CALIBRATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 最终重量的全局截距，单位为克，可在运行时直接修改。
 * 例如：1.5f表示所有重量增加1.5克，-2.0f表示减少2.0克。 */
extern float g_weight_output_offset_g;
/* 当前运行时斜率，单位为克/ADC count。 */
extern float g_weight_slope_g_per_count;

/* 根据用户提供的10 Hz日志得到全局线性标定参数。
 * 输入单位为 ADS1232 counts，输出单位为0.1克。 */
void WeightCalibration_Init(void);
int32_t WeightCalibration_Convert(int32_t raw_counts);
void WeightCalibration_SetZeroShift(int32_t shift_counts);
int32_t WeightCalibration_GetZeroShift(void);
int32_t WeightCalibration_GetZeroRaw(void);

/* 竖挂并带袋子的空载零点以常量形式存入程序 Flash。
 * elapsed_ms 为单片机上电后的毫秒数，节点之间采用线性插值；
 * 超过曲线最后一个时间点后保持最后一个零点值。 */
int32_t WeightCalibration_GetFlashZeroRaw(uint32_t elapsed_ms);
void WeightCalibration_SetZeroRaw(int32_t zero_raw_counts);
void WeightCalibration_SetParameters(int32_t zero_raw, float slope_g_per_count, float offset_g);
float WeightCalibration_GetSlope(void);
float WeightCalibration_GetOffset(void);
uint8_t WeightCalibration_IsManual(void);
/* 保存/加载校准参数；校准修改默认只在 RAM 中，save 后才掉电保留。 */
uint8_t WeightCalibration_Save(void);
uint8_t WeightCalibration_Load(void);
uint8_t WeightCalibration_FlashValid(void);

#ifdef __cplusplus
}
#endif

#endif

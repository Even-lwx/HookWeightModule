#ifndef WEIGHT_STORAGE_H
#define WEIGHT_STORAGE_H
#include <stdint.h>
typedef struct { uint32_t sequence; uint32_t session_id; uint32_t minute; int32_t weight_x1000; } WeightStorage_Record;
/* 初始化存储区并建立本次上电会话。 */
void WeightStorage_Init(void);
/* 周期调用，到时间后写入一条重量记录。 */
uint8_t WeightStorage_Task(uint32_t now_ms, int32_t weight_x1000);
uint32_t WeightStorage_GetRecordCount(void);
uint32_t WeightStorage_GetCapacity(void);
uint8_t WeightStorage_ReadBegin(uint32_t latest_count);
uint8_t WeightStorage_ReadNext(WeightStorage_Record *record);
void WeightStorage_ReadEnd(void);
uint8_t WeightStorage_IsBusy(void);
uint8_t WeightStorage_Clear(void);
uint8_t WeightStorage_SetIntervalMs(uint32_t interval_ms);
uint32_t WeightStorage_GetIntervalMs(void);
#endif

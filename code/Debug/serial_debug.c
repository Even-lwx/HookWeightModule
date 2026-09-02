#include "serial_debug.h"
#include "weight_processor.h"
#include "weight_calibration.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

/* 串口调试协议：普通模式下保持重量输出，调试模式下只回复命令。 */
#define RX_SIZE 128U
#define MAX_POINTS 16U
typedef struct { int32_t raw; float grams; } CalPoint;
/* 接收缓冲区由中断逐字节填充，完整的一行在主循环中解析。 */
static UART_HandleTypeDef *uart; static uint8_t rx_byte, line[RX_SIZE]; static uint16_t line_len;
/* 斜率校准会话只修改 RAM 参数，必须执行 save 才写入 Flash。 */
static uint8_t debug_mode, slope_mode, point_count; static CalPoint points[MAX_POINTS];

static void Reply(const char *s) { HAL_UART_Transmit(uart,(uint8_t *)s,(uint16_t)strlen(s),50U); }
static void ReplyHelp(void)
{
    Reply("OK HELP\r\n");
    Reply("debug             enter debug mode\r\n");
    Reply("quick zero        zero now and auto-save (normal mode)\r\n");
    Reply("help              show this help\r\n");
    Reply("exit              leave debug mode and resume output\r\n");
    Reply("zero              calibrate current stable value as 0g\r\n");
    Reply("cal <grams>        calibrate current stable value\r\n");
    Reply("slope             start slope calibration\r\n");
    Reply("point <grams>      add a stable calibration point\r\n");
    Reply("end               fit slope and apply calibration\r\n");
    Reply("cancel            cancel slope calibration\r\n");
    Reply("save              save changed parameters to Flash\r\n");
    Reply("flash             read and print Flash parameters\r\n");
    Reply("NOTE: calibration changes are RAM-only until 'save'.\r\n");
}
static uint8_t Stable(int32_t *raw) { const WeightProcessor_Result *r=WeightProcessor_GetLastResult(); if(!r->ready)return 0U; if(!WeightProcessor_IsCalibrationStable())return 0U; *raw=r->filtered_raw; return 1U; }
/* 将命令参数转换为浮点数，并拒绝空参数、非法字符和无穷值。 */
static uint8_t ParseFloat(char *s, float *v) { char *e; while(*s==' ')s++; if(!*s)return 0U; *v=strtof(s,&e); while(*e==' ')e++; return (e!=s && *e=='\0' && isfinite(*v)); }
/* 解析并执行一条以换行结束的命令。每种情况都会给上位机返回结果。 */
static void Command(char *cmd)
{
    char *arg; int32_t raw; float g;
    while(*cmd==' ')cmd++; if(!*cmd){Reply("ERR BAD_CMD\r\n");return;}
    arg=strchr(cmd,' '); if(arg){*arg++='\0';} 
    if(!strcmp(cmd,"debug")){if(debug_mode)Reply("ERR ALREADY_DEBUG\r\n");else{debug_mode=1U;Reply("OK DEBUG ON; type help\r\n");}return;}
    /* quick zero 是普通模式下的快捷去零命令，执行后立即自动保存。 */
    if(!strcmp(cmd,"quick")) {
        if (debug_mode) { Reply("ERR QUICK_ZERO_ONLY_NORMAL\r\n"); return; }
        if (!arg || strcmp(arg, "zero")) { Reply("ERR BAD_PARAM\r\n"); return; }
        if (!Stable(&raw)) { Reply("ERR NOT_STABLE\r\n"); return; }
        WeightCalibration_SetParameters(raw, WeightCalibration_GetSlope(), 0.0f);
        if (WeightCalibration_Save()) {
            char b[96];
            snprintf(b, sizeof(b), "OK QUICK ZERO raw=%ld SAVED\r\n", (long)raw);
            Reply(b);
        } else {
            Reply("ERR QUICK ZERO FLASH_WRITE\r\n");
        }
        return;
    }
    if(!strcmp(cmd,"exit")){if(!debug_mode)Reply("ERR NOT_DEBUG\r\n");else{slope_mode=0U;point_count=0U;debug_mode=0U;Reply("OK DEBUG OFF\r\n");}return;}
    if(!debug_mode){Reply("ERR NOT_DEBUG\r\n");return;}
    if(!strcmp(cmd,"help")){if(arg){Reply("ERR BAD_PARAM\r\n");return;}ReplyHelp();return;}
    if(!strcmp(cmd,"cancel")){slope_mode=0U;point_count=0U;Reply("OK CANCEL\r\n");return;}
    if(!strcmp(cmd,"zero")){if(arg||!Stable(&raw)){Reply(arg?"ERR BAD_PARAM\r\n":"ERR NOT_STABLE\r\n");return;} WeightCalibration_SetParameters(raw,WeightCalibration_GetSlope(),0.0f); {char b[96];snprintf(b,sizeof(b),"OK ZERO raw=%ld; SAVE REQUIRED\r\n",(long)raw);Reply(b);}return;}
    if(!strcmp(cmd,"cal")){if(!arg||!ParseFloat(arg,&g)){Reply("ERR BAD_PARAM\r\n");return;}if(!Stable(&raw)){Reply("ERR NOT_STABLE\r\n");return;}WeightCalibration_SetParameters(WeightCalibration_GetZeroRaw(),WeightCalibration_GetSlope(),g-(raw-WeightCalibration_GetZeroRaw())*WeightCalibration_GetSlope());Reply("OK CAL; SAVE REQUIRED\r\n");return;}
    if(!strcmp(cmd,"slope")){if(arg){Reply("ERR BAD_PARAM\r\n");return;}if(slope_mode){Reply("ERR SLOPE_ACTIVE\r\n");return;}slope_mode=1U;point_count=0U;Reply("OK SLOPE\r\n");return;}
    if(!strcmp(cmd,"point")){if(!slope_mode){Reply("ERR SLOPE_NOT_ACTIVE\r\n");return;}if(!arg||!ParseFloat(arg,&g)){Reply("ERR BAD_PARAM\r\n");return;}if(point_count>=MAX_POINTS){Reply("ERR TOO_MANY_POINTS\r\n");return;}if(!Stable(&raw)){Reply("ERR NOT_STABLE\r\n");return;}points[point_count].raw=raw;points[point_count++].grams=g;Reply("OK POINT\r\n");return;}
    /* 最小二乘拟合 grams = slope * raw + intercept，至少需要两个不同原始值。 */
    if(!strcmp(cmd,"end")){uint8_t i;double sx=0,sy=0,sxx=0,sxy=0,d,a,b; if(!slope_mode){Reply("ERR SLOPE_NOT_ACTIVE\r\n");return;}if(point_count<2U){Reply("ERR TOO_FEW_POINTS\r\n");return;}for(i=0;i<point_count;i++){double x=points[i].raw,y=points[i].grams;sx+=x;sy+=y;sxx+=x*x;sxy+=x*y;}d=point_count*sxx-sx*sx;if(fabs(d)<1e-9){Reply("ERR BAD_SLOPE\r\n");return;}a=(point_count*sxy-sx*sy)/d;b=(sy-a*sx)/point_count;if(a<=0){Reply("ERR BAD_SLOPE\r\n");return;}WeightCalibration_SetParameters((int32_t)(-b/a), (float)a, 0.0f);slope_mode=0U;{char out[96];snprintf(out,sizeof(out),"OK SLOPE_END points=%u slope=%.8f; SAVE REQUIRED\r\n",point_count,a);Reply(out);}return;}
    if(!strcmp(cmd,"save")){if(arg){Reply("ERR BAD_PARAM\r\n");return;}Reply(WeightCalibration_Save()?"OK SAVE (parameters active and persistent)\r\n":"ERR FLASH_WRITE\r\n");return;}
    if(!strcmp(cmd,"flash")){const char *v=WeightCalibration_FlashValid()?"valid":"invalid";char out[128];if(arg){Reply("ERR BAD_PARAM\r\n");return;}snprintf(out,sizeof(out),"OK FLASH status=%s zero=%ld slope=%.8f offset=%.3f\r\n",v,(long)WeightCalibration_GetZeroRaw(),WeightCalibration_GetSlope(),WeightCalibration_GetOffset());Reply(out);return;}
    Reply("ERR BAD_CMD\r\n");
}
/* 初始化 UART 单字节接收中断。 */
void SerialDebug_Init(UART_HandleTypeDef *h){uart=h;debug_mode=0U;slope_mode=0U;line_len=0U;HAL_UART_Receive_IT(uart,&rx_byte,1U);}
/* UART 中断回调：遇到 CR/LF 形成完整命令，其余字符放入行缓冲区。 */
void SerialDebug_RxCpltCallback(UART_HandleTypeDef *h){if(h!=uart)return;if(rx_byte=='\r'||rx_byte=='\n'){if(line_len){line[line_len]=0;Command((char *)line);line_len=0U;}}else if(line_len<RX_SIZE-1U)line[line_len++]=rx_byte;HAL_UART_Receive_IT(uart,&rx_byte,1U);}
void SerialDebug_Process(void){}
uint8_t SerialDebug_OutputEnabled(void){return debug_mode?0U:1U;}

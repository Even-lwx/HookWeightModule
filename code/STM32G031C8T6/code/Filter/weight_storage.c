#include "weight_storage.h"
#include "stm32g0xx_hal.h"
#define BASE 0x08007800UL
#define PAGE_SIZE 2048UL
#define RECORD_SIZE 16UL
#define PAGE_COUNT 16UL
#define ACTIVE_PAGES 15UL
#define PER_PAGE (PAGE_SIZE/RECORD_SIZE)
#define CAPACITY (ACTIVE_PAGES*PER_PAGE)
#define EMPTY 0xFFFFFFFFUL
#define PERIOD_DEFAULT 60000UL
#define PERIOD_MIN 100UL
#define PERIOD_MAX 600000UL
static uint32_t wp,ws,oldp,sp,seq,sess,minc,tick,count,period=PERIOD_DEFAULT,rlimit,rdone,rstart,rlast;
static uint8_t init,busy;
static uint32_t Next(uint32_t v){++v;return(v==0UL||v==EMPTY)?1UL:v;}
static const WeightStorage_Record*At(uint32_t p,uint32_t s){return(const WeightStorage_Record*)(BASE+(p*PER_PAGE+s)*RECORD_SIZE);}
static uint8_t Valid(const WeightStorage_Record*r){return r->sequence!=EMPTY&&r->sequence!=0UL&&r->session_id!=EMPTY;}
static uint8_t Erase(uint32_t p){FLASH_EraseInitTypeDef e;uint32_t pe;HAL_StatusTypeDef st;e.TypeErase=FLASH_TYPEERASE_PAGES;e.Banks=FLASH_BANK_1;e.Page=15U+p;e.NbPages=1U;HAL_FLASH_Unlock();st=HAL_FLASHEx_Erase(&e,&pe);HAL_FLASH_Lock();return st==HAL_OK;}
static uint8_t Write(uint32_t p,uint32_t s,const WeightStorage_Record*r){uint32_t a=BASE+(p*PER_PAGE+s)*RECORD_SIZE;HAL_StatusTypeDef st;HAL_FLASH_Unlock();st=HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,a,*(const uint64_t*)r);if(st==HAL_OK)st=HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,a+8UL,*((const uint64_t*)r+1));HAL_FLASH_Lock();return st==HAL_OK;}
static uint8_t Compact(const WeightStorage_Record*n){uint32_t i;const WeightStorage_Record*r;if(!Erase(sp))return 0U;for(i=1UL;i<PER_PAGE;++i){r=At(oldp,i);if(!Write(sp,i-1UL,r))return 0U;}if(!Write(sp,PER_PAGE-1UL,n)||!Erase(oldp))return 0U;sp=oldp;oldp=(oldp+1UL)%PAGE_COUNT;wp=sp;ws=0UL;return 1U;}
void WeightStorage_Init(void){uint32_t p,s;uint32_t newest=0UL;const WeightStorage_Record*r;count=0UL;for(p=0UL;p<PAGE_COUNT;++p)for(s=0UL;s<PER_PAGE;++s){r=At(p,s);if(Valid(r)){++count;if(r->sequence>newest){newest=r->sequence;wp=p;ws=s+1UL;}}}if(count>CAPACITY)count=CAPACITY;if(newest==0UL){wp=0UL;ws=0UL;oldp=0UL;sp=ACTIVE_PAGES;seq=1UL;}else{seq=Next(newest);if(ws>=PER_PAGE){ws=0UL;wp=(wp+1UL)%PAGE_COUNT;}oldp=(wp+PAGE_COUNT-ACTIVE_PAGES+1UL)%PAGE_COUNT;sp=(wp+1UL)%PAGE_COUNT;}sess=1UL;minc=0UL;tick=HAL_GetTick()+period;init=1U;busy=0U;}
uint8_t WeightStorage_Task(uint32_t now,int32_t w){WeightStorage_Record r;if(!init||busy||(int32_t)(now-tick)<0)return 0U;tick+=period;if((int32_t)(now-tick)>=0)tick=now+period;r.sequence=seq;r.session_id=sess;r.minute=++minc;r.weight_x1000=w;if(count<CAPACITY){if(!Write(wp,ws,&r))return 0U;if(++ws>=PER_PAGE){ws=0UL;wp=(wp+1UL)%ACTIVE_PAGES;}++count;}else if(!Compact(&r))return 0U;seq=Next(seq);return 1U;}
uint32_t WeightStorage_GetRecordCount(void){return count;}uint32_t WeightStorage_GetCapacity(void){return CAPACITY;}uint8_t WeightStorage_IsBusy(void){return busy;}
uint8_t WeightStorage_ReadBegin(uint32_t n){
    uint32_t p,s,k,prev=EMPTY,b=0UL; const WeightStorage_Record*r;
    if(busy)return 0U;
    rlimit=(n==0UL||n>count)?count:n; rdone=0UL; rstart=0UL; rlast=0UL;
    /* 保留原有的“最新 N 条”行为，同时记录其中最旧的
       序号，使 ReadNext 能按升序输出这一组记录。 */
    for(k=0UL;k<rlimit;++k){
        b=0UL;
        for(p=0UL;p<PAGE_COUNT;++p)if(p!=sp)for(s=0UL;s<PER_PAGE;++s){
            r=At(p,s);
            if(Valid(r)&&(prev==EMPTY||r->sequence<prev)&&r->sequence>b)b=r->sequence;
        }
        if(b==0UL)break;
        prev=b;
    }
    rstart=b; busy=1U; return 1U;
}
uint8_t WeightStorage_ReadNext(WeightStorage_Record*out){
    uint32_t p,s,b=EMPTY; const WeightStorage_Record*r,*br=0;
    if(!busy||!out||rdone>=rlimit)return 0U;
    for(p=0UL;p<PAGE_COUNT;++p)if(p!=sp)for(s=0UL;s<PER_PAGE;++s){
        r=At(p,s);
        if(Valid(r)&&r->sequence>=rstart&&r->sequence>rlast&&(b==EMPTY||r->sequence<b)){b=r->sequence;br=r;}
    }
    if(!br){busy=0U;return 0U;}
    *out=*br; rlast=b; ++rdone; return 1U;
}
void WeightStorage_ReadEnd(void){busy=0U;rlimit=0UL;rdone=0UL;rstart=0UL;rlast=0UL;}
uint8_t WeightStorage_Clear(void){uint32_t p;if(busy)return 0U;for(p=0UL;p<PAGE_COUNT;++p)if(!Erase(p))return 0U;wp=0UL;ws=0UL;oldp=0UL;sp=ACTIVE_PAGES;seq=1UL;sess=1UL;minc=0UL;count=0UL;tick=HAL_GetTick()+period;return 1U;}
uint8_t WeightStorage_SetIntervalMs(uint32_t ms){if(ms<PERIOD_MIN||ms>PERIOD_MAX)return 0U;period=ms;tick=HAL_GetTick()+ms;return 1U;}uint32_t WeightStorage_GetIntervalMs(void){return period;}

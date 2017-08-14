#ifndef MAINSENSORHUB_H
#define MAINSENSORHUB_H

/*
 * =========== timer variable ===========
 */
#define LOOP_RATE_1000Hz_ 1000

/*
 * COUNT is defined for timer 1ms
 */
#define COUNT_50_HZ_	20
#define COUNT_100_HZ_	10
#define COUNT_125_HZ_   8
#define COUNT_25_HZ_  40
struct TimerCountType{
    unsigned char Hz_100;
    unsigned char Hz_50;
    unsigned char Hz_125;
    unsigned char Hz_25;
}Timer_Count;

struct FlagTimerType{
    unsigned char Hz_50:1;
    unsigned char Hz_100:1;
    unsigned char Hz_125:1;
    unsigned char Hz_30:1;
}FlagTimer;
//======timer handler run in 1000Hz=========
void Timer_handler(){
    Timer_Count.Hz_50++;
    Timer_Count.Hz_100++;
    Timer_Count.Hz_125++;
    if(Timer_Count.Hz_50==COUNT_50_HZ_){
        Timer_Count.Hz_50=0;
        FlagTimer.Hz_50=1;
    }


    if(Timer_Count.Hz_100==COUNT_100_HZ_)
    {
        Timer_Count.Hz_100=0;
        FlagTimer.Hz_100=1;
    }
    if(Timer_Count.Hz_125==COUNT_125_HZ_)
    {
        Timer_Count.Hz_125=0;
        FlagTimer.Hz_125=1;
    }
}


#define SAM_FB_TURNOFF 0
#define SAM_FB_LOWERLINK_POS12_50HZ 1
#define SAM_FB_LOWERLINK_POS12_125HZ 2
#define SAM_FB_FULLBODY_POS12_50HZ 3
#define SAM_FB_FULLBODY_POS12_125HZ 4

struct Sys_flag_struct{
    unsigned char getPos12LowerLink_50Hz:1;
    unsigned char getPos12FullBody_50Hz:1;
    unsigned char getPos12LowerLink_125Hz:1;
    unsigned char getPos12FullBody_125Hz:1;
}sys_flag;

unsigned int pos12LowerLink[30];
/*
 * =========== sensor variable ===========
 */

const unsigned char default_samP[25]={30,30,30,30,30,30,30,30,30,30,30,30,
                                     10,10,10,10,
                                      10,10,10,10,
                                      0,0,30,5,5};
const unsigned char default_samI[25]={0,0,0,0,0,0,0,0,0,0,0,0,
                                      0,0,0,0,0,0,0,0,0,0,0,0,0};

const unsigned char default_samD[25]={10,10,10,10,10,10,10,10,10,10,10,10,
                                     10,10,10,10,
                                      5,5,5,5,
                                      0,0,10,5,5};
const unsigned int default_averageTorq[25]={3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,
                                           2000,2000,2000,2000,
                                            3000,3000,3000,3000,
                                            0,0,3000,3000,3000};


#endif // MAINSENSORHUB_H

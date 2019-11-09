#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <sys/time.h>

//PRU INCLUDE
#include <soc/vmem.h>
#include <soc/pru.h>
#include "logging.h"
#include "am335x/pru.h"
#include "soc/pru_intc.h"
#include "soc/timer.h"
#include "am335x/dmtimer.h"
#include "gpio.h"
#include "spinlock.h"
#include "ringbuffer.h"

#define DELAY_MSEC 5
#define INIT 0
#define START 1 
#define END 2

volatile int running = 0;
void CleanUp (int signum);
int32_t uin16ToInt32(uint16_t reg);
struct pollfd pru_intc;
struct package_t recv;

int main (int argc, char **argv)
{
	//PRU var    
    uint8_t* sharedMemBase;
    struct SDT_t sdt;
    uint32_t package_count = 0;
    int ret = 0;
    int eleNum = 0;
    int prev_count = 0;
    int error_count = 0;
    int full_count = 0;
    int sdt_error = 0;
	int over_time_count  = 0;
	int under_time_count = 0;
    int i=0;
 	int state = INIT;
	int start_time = 0;
	int end_time = 0;
   
    //data log var
    char raw_data_name[64]={0};
    char raw_num_name[64]={0};
    //char matlab_data_name[32]={0};
    //char matlab_num_name[32]={0};
    FILE *raw_fp=NULL;
    FILE *raw_np=NULL;
    //FILE *matlab_fp;
    //FILE *matlab_np;

	struct timespec nt,pt;
	long period = 0;
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    
    sprintf(raw_data_name,"/root/IMUSDT/data_%d_%d_%d-%d-%d.txt\0", tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	sprintf(raw_num_name,"/root/IMUSDT/num_%d_%d_%d-%d-%d.txt\0", tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);    
    //sprintf(matlab_data_name,"./data_%d-%d-%d:%d:%d.txt\0", tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	//sprintf(matlab_num_name,"./num_%d-%d-%d:%d:%d.txt\0", tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec); 

    raw_fp = fopen(raw_data_name, "w");
    raw_np = fopen(raw_num_name, "w");
    //matlab_fp = fopen(matlab_data_name, "w");
    //matlab_np = fopen(matlab_num_name, "w");
    if(raw_fp == NULL) goto ERROR;
    if(raw_np == NULL) goto ERROR;

    double gyro_high_const  = 0.02;
    double gyro_low_const   = 0.01 * pow(2, -15);
    double accel_high_const = 0.8;
    double accel_low_const  = 0.4 * pow(2, -15);
    double temper_const     = 0.00565;    
    
    double gyro_X  = 0, gyro_Y = 0,  gyro_Z = 0;
    double accel_X = 0, accel_Y = 0, accel_Z = 0;
    
    double Vgyro_X  = 0, Vgyro_Y = 0,  Vgyro_Z = 0;
    double Vaccel_X = 0, Vaccel_Y = 0, Vaccel_Z = 0;

    double temper = 0.0;    
    int AX_hreg = 0, AY_hreg = 0, AZ_hreg = 0, GX_hreg = 0, GY_hreg = 0, GZ_hreg = 0;
    int AX_lreg = 0, AY_lreg = 0, AZ_lreg = 0, GX_lreg = 0, GY_lreg = 0, GZ_lreg = 0;
    
    lprintf(
        "******************** PRU IMUSDT Program ********************\r\n"
        "Build: " __DATE__ " " __TIME__"\r\n\r\n"
    );
    
    signal (SIGINT,     CleanUp);
    signal (SIGTERM,    CleanUp);
    signal (SIGQUIT,    CleanUp);
    signal (SIGHUP,     CleanUp);

    vmem_init();
    PRU_init();
    PRU_firmware_load(PRU0, "/lib/firmware/PRU_IMUSDT.elf");
    
    PRU_INTC_SysEvent_Enable(16);               /* Enable SysEvent 16 */
    PRU_INTC_SysEvent_SetChannel(16, 0);        /* Map SysEvent 16 to Channel 0 */
    PRU_INTC_Channel_SetHostInterrupt(0, 0);    /* Map Channel 0 to Host Interrupt 0 */
    PRU_INTC_HostInterrupt_Enable(0);           /* Enable Host Interrupt 0 */
    
    PRU_INTC_Open(&pru_intc);
    
    memset((void*) &recv, 0, sizeof(struct package_t));
    sharedMemBase = (uint8_t*) userSharedMemInit();
    sdtInit(&sdt, (void*) sharedMemBase, NUM_OF_CELL, 0);
    
	PRU_start(PRU0);
    
    Timer_Init(3);
    Timer_SetFreq(3, 200);
    Timer_Start(3);

    running = 1;
	usleep(700000);
	
    while (running)
    {
        
        ret = sdt_pop(&sdt, (void*) &recv);
        if(ret !=0) {

            eleNum = sdt_eleNum(&sdt);
            error_count = sdt_errorCnt(&sdt);
            full_count  = sdt_fullCnt(&sdt);
			   
            Vgyro_X  = 0; Vgyro_Y  = 0; Vgyro_Z  = 0;
            Vaccel_X = 0; Vaccel_Y = 0; Vaccel_Z = 0;

            
			
            for(i=0;i<10;i++) {
                 GX_hreg = uin16ToInt32(recv.gyroX_high[i]); 
                 GY_hreg = uin16ToInt32(recv.gyroY_high[i]);
                 GZ_hreg = uin16ToInt32(recv.gyroZ_high[i]);
                 GX_lreg = recv.gyroX_low[i]; 
                 GY_lreg = recv.gyroY_low[i];
                 GZ_lreg = recv.gyroZ_low[i];
                 AX_hreg = uin16ToInt32(recv.accelX_high[i]);
                 AY_hreg = uin16ToInt32(recv.accelY_high[i]);
                 AZ_hreg = uin16ToInt32(recv.accelZ_high[i]);
                 AX_lreg = recv.accelX_low[i];
                 AY_lreg = recv.accelY_low[i];
                 AZ_lreg = recv.accelZ_low[i];
                 
                 gyro_X  = gyro_high_const * GX_hreg + gyro_low_const * GX_lreg;
                 gyro_Y  = gyro_high_const * GY_hreg + gyro_low_const * GY_lreg;
                 gyro_Z  = gyro_high_const * GZ_hreg + gyro_low_const * GZ_lreg;
                 accel_X = accel_high_const * AX_hreg + accel_low_const * AX_lreg;
                 accel_Y = accel_high_const * AY_hreg + accel_low_const * AY_lreg;
                 accel_Z = accel_high_const * AZ_hreg + accel_low_const * AZ_lreg;                 
                 
                 Vgyro_X += gyro_X; 
                 Vgyro_Y += gyro_Y; 
                 Vgyro_Z += gyro_Z;
                 Vaccel_X += accel_X;
                 Vaccel_Y += accel_Y; 
                 Vaccel_Z += accel_Z;
			     temper = ((uint32_t) recv.temper) * temper_const +25.0;
				
                if(i==9) {
					periodCounter(&period, &nt, &pt);
                    fprintf(raw_fp,"],%6d,%4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x\n",
						period,                        
						recv.gyroX_high[i], 
                        recv.gyroY_high[i], 
                        recv.gyroZ_high[i], 
                        recv.gyroX_low[i],
                        recv.gyroY_low[i], 
                        recv.gyroZ_low[i],
                        recv.accelX_high[i],
                        recv.accelY_high[i],
                        recv.accelZ_high[i],
                        recv.accelX_low[i],  
                        recv.accelY_low[i],  
                        recv.accelZ_low[i],
                        recv.temper);
                } else {
                    fprintf(raw_fp,"[,%4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x,%4x\n",
                        recv.gyroX_high[i], 
                        recv.gyroY_high[i], 
                        recv.gyroZ_high[i], 
                        recv.gyroX_low[i],
                        recv.gyroY_low[i], 
                        recv.gyroZ_low[i],
                        recv.accelX_high[i],
                        recv.accelY_high[i],
                        recv.accelZ_high[i],
                        recv.accelX_low[i],  
                        recv.accelY_low[i],  
                        recv.accelZ_low[i]);
                }
            }
            
            Vgyro_X /= 10.0; 
            Vgyro_Y /= 10.0; 
            Vgyro_Z /= 10.0;
            Vaccel_X /= 10.0;
            Vaccel_Y /= 10.0; 
            Vaccel_Z /= 10.0;
            
			if(period > 55000) {
				over_time_count++;
			}
			
			if(period < 45000) {
				under_time_count++;
			}
            if(package_count%20 == 0) {
                printf("c:%5d e:%2d p:%5d o:%3d u:%3d gX:%4.4lf gY:%4.4lf gZ:%4.4lf aX:%4.4lf aY:%4.4lf aZ:%4.4lf tmp:%2.2lf\r\n",
                    package_count, sdt_error, period, over_time_count, under_time_count,              
                    Vgyro_X, Vgyro_Y, Vgyro_Z, Vaccel_X, Vaccel_Y, Vaccel_Z, temper);
            }
            if( (prev_count+1) != recv.package_count) {
                sdt_error++;
				printf("package_count error!\n");
            }
   			
			if(period > 100000) {
				sdt_error++;
			}

            prev_count = recv.package_count;
            memset((void*) &recv, 0, sizeof(struct package_t));
            package_count++;
					
        }
    }
	
    printf("%s\n",raw_data_name);
    printf("%s\n",raw_num_name);
    printf("num:%d error:%d over_time_count:%d under_time_count:%d\n", package_count, sdt_error, over_time_count,under_time_count);
    fprintf(raw_np,"num: %d error: %d \n", package_count, sdt_error);
	
    sdtDeinit(&sdt);
    userSharedMemDeInit(sharedMemBase);

ERROR:
	Timer_Stop(3);
	PRU_halt(PRU0);  
    fclose(raw_fp);
    fclose(raw_np);
    //fclose(matlab_fp);
    //fclose(matlab_np);	
    return 0;
}

void CleanUp (int signum) {
    running = 0;    
    lprintf("Exit.\r\n");
}


int32_t uin16ToInt32(uint16_t reg) {
    int32_t val=0;    
    if ( ( (reg>>15) & 1 ) ) { // negative
        reg = ((~reg)+1);
        val = -(int)reg;
    } else {
        val = (int)reg;
    }
    
    return val;
}

int periodCounter(long *period, struct timespec *nt , struct timespec *pt)
{
	long temp;
	clock_gettime(CLOCK_MONOTONIC, nt);
	if(pt->tv_nsec > nt->tv_nsec)
		temp = 1000000000 - pt->tv_nsec + nt->tv_nsec;
	else
		temp = nt->tv_nsec - pt->tv_nsec;
	pt->tv_nsec = nt->tv_nsec;
	*period = temp/1000;
	return 0;
}




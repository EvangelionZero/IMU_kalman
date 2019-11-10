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
	
	//kalman
	//angle(rad)
    float alpha = 0.0;
    float beta1 = 0.0;
    float gamma = 0.0;
	//angle(deg)
    float alpha_deg = 0.0;
    float beta1_deg = 0.0;
    float gamma_deg = 0.0;
	//angle kalman(rad)
    float alpha_kalman = 0.0;
    float beta1_kalman = 0.0;
    float gamma_kalman = 0.0;
	//angle kalman(deg)
    float alpha_kalman_deg = 0.0;
    float beta1_kalman_deg = 0.0;
    float gamma_kalman_deg = 0.0;

    float Q_beta1_angle=0.01;
    float Q_beta1_gyro=0.03;
    float R_beta1_angle=0.03;
    float beta1_bias=0.0;
    float P_beta1_00=0.0;
    float P_beta1_01=0.0;
    float P_beta1_10=0.0;
    float P_beta1_11=0.0;
    float y_beta1=0.0;
    float s_beta1=0.0;
    float K_beta1_0=0.0;
    float K_beta1_1=0.0;

    float Q_alpha_angle=0.01;
    float Q_alpha_gyro=0.04;
    float R_alpha_angle=0.01;
    float alpha_bias=0.0;
    float P_alpha_00=0.0;
    float P_alpha_01=0.0;
    float P_alpha_10=0.0;
    float P_alpha_11=0.0;
    float y_alpha=0.0;
    float s_alpha=0.0;
    float K_alpha_0=0.0;
    float K_alpha_1=0.0;

    float Q_gamma_angle=0.001;
    float Q_gamma_gyro=0.01;
    float R_gamma_angle=0.001;
    float gamma_bias=0;
    float P_gamma_00=0.0;
    float P_gamma_01=0.0;
    float P_gamma_10=0.0;
    float P_gamma_11=0.0;
    float y_gamma=0.0;
    float s_gamma=0.0;
    float K_gamma_0=0.0;
    float K_gamma_1=0.0;
    float gamma_measure=0.0;
	
   
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
				//16 bits資料合起來變32 bits
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
				 kalman_angle_filter();
                 
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
            //累計10次平均一次
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

int periodCounter(long *period, struct timespec *nt , struct timespec *pt){
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


void kalman_angle_filter(){
    float temp1=0.0, temp2=0.0;
    float sin_beta1 = sin(beta1_kalman);
    float cos_beta1 = cos(beta1_kalman);
    float sin_alpha = sin(alpha_kalman);
    float cos_alpha = cos(alpha_kalman);

//    //rad
    alpha = -atan(accel_X/accel_Z);// - 0.004;//負號代表方向?
    beta1 =  atan(accel_Y/accel_Z);// + 0.007;
	gamma =  atan(accel_Y/accel_X);
	alpha_deg = alpha*rad_to_deg;
    beta1_deg = beta1*rad_to_deg;
	gamma_deg = beta1*rad_to_deg;
    
	alpha_kalman_deg = alpha_kalman*rad_to_deg;
    beta1_kalman_deg = beta1_kalman*rad_to_deg;
    gamma_kalman_deg = gamma_kalman*rad_to_deg;

		
    //kalman beta1
    //beta1_kalman=beta1_kalman + sample_time*(beta1_dot - beta1_bias);    // project the state ahead
//    beta1_bias = 0;
    beta1_kalman=beta1_kalman + sample_time*(X_gyro - beta1_bias);
    P_beta1_00= P_beta1_00-sample_time*P_beta1_10-sample_time*P_beta1_01+sample_time*sample_time*P_beta1_11+Q_beta1_angle*sample_time;   //project the error covariance ahead
    P_beta1_01= P_beta1_01-P_beta1_11*sample_time;       //project the error covariance ahead
    P_beta1_10= P_beta1_10-P_beta1_11*sample_time;       //project the error covariance ahead
    P_beta1_11= P_beta1_11+Q_beta1_gyro*sample_time;     //project the error covariance ahead
    y_beta1= beta1-beta1_kalman;
    s_beta1= P_beta1_00+R_beta1_angle;
    K_beta1_0=P_beta1_00/s_beta1;                        // compute the kalman gain
    K_beta1_1=P_beta1_10/s_beta1;                        // compute the kalman gain
    beta1_kalman= beta1_kalman+K_beta1_0*y_beta1;         //update estimate with measurement
    beta1_bias= beta1_bias+K_beta1_1*y_beta1;             //update estimate with measurement
    P_beta1_00= P_beta1_00-K_beta1_0*P_beta1_00;          //update the error covariance
    P_beta1_01= P_beta1_01-K_beta1_0*P_beta1_01;          //update the error covariance
    P_beta1_10= P_beta1_10-K_beta1_1*P_beta1_00;          //update the error covariance
    P_beta1_11= P_beta1_11-K_beta1_1*P_beta1_01;          //update the error covariance

    //kalman alpha
    //alpha_kalman=alpha_kalman + sample_time*(alpha_dot - alpha_bias);    // project the state ahead

    alpha_kalman=alpha_kalman + sample_time*(Y_gyro - alpha_bias);    // project the state ahead
    P_alpha_00= P_alpha_00-sample_time*P_alpha_10-sample_time*P_alpha_01+sample_time*sample_time*P_alpha_11+Q_alpha_angle*sample_time;   //project the error covariance ahead
    P_alpha_01= P_alpha_01-P_alpha_11*sample_time;    //project the error covariance ahead
    P_alpha_10= P_alpha_10-P_alpha_11*sample_time;    //project the error covariance ahead
    P_alpha_11= P_alpha_11+Q_alpha_gyro*sample_time;  //project the error covariance ahead

    y_alpha= alpha-alpha_kalman;
    s_alpha= P_alpha_00+R_alpha_angle;
    K_alpha_0=P_alpha_00/s_alpha;                     // compute the kalman gain
    K_alpha_1=P_alpha_10/s_alpha;                     // compute the kalman gain

    alpha_kalman= alpha_kalman+K_alpha_0*y_alpha;     //update estimate with measurement
    alpha_bias= alpha_bias+K_alpha_1*y_alpha;         //update estimate with measurement
    P_alpha_00= P_alpha_00-K_alpha_0*P_alpha_00;      //update the error covariance
    P_alpha_01= P_alpha_01-K_alpha_0*P_alpha_01;      //update the error covariance
    P_alpha_10= P_alpha_10-K_alpha_1*P_alpha_00;      //update the error covariance
    P_alpha_11= P_alpha_11-K_alpha_1*P_alpha_01;      //update the error covariance


    //kalman gamma
    gamma_kalman=gamma_kalman + sample_time*(Y_gyro - gamma_bias);    // project the state ahead
    P_gamma_00= P_gamma_00-sample_time*P_gamma_10-sample_time*P_gamma_01+sample_time*sample_time*P_gamma_11+Q_gamma_angle*sample_time;   //project the error covariance ahead
    P_gamma_01= P_gamma_01-P_gamma_11*sample_time;    //project the error covariance ahead
    P_gamma_10= P_gamma_10-P_gamma_11*sample_time;    //project the error covariance ahead
    P_gamma_11= P_gamma_11+Q_gamma_gyro*sample_time;  //project the error covariance ahead

    y_gamma= gamma_measure-gamma_kalman;
    s_gamma= P_gamma_00+R_gamma_angle;
    K_gamma_0=P_gamma_00/s_gamma;                     // compute the kalman gain
    K_gamma_1=P_gamma_10/s_gamma;                     // compute the kalman gain

    gamma_kalman= gamma_kalman+K_gamma_0*y_gamma;     //update estimate with measurement
    gamma_bias= gamma_bias+K_gamma_1*y_gamma;         //update estimate with measurement
    P_gamma_00= P_gamma_00-K_gamma_0*P_gamma_00;      //update the error covariance
    P_gamma_01= P_gamma_01-K_gamma_0*P_gamma_01;      //update the error covariance
    P_gamma_10= P_gamma_10-K_gamma_1*P_gamma_00;      //update the error covariance
    P_gamma_11= P_gamma_11-K_gamma_1*P_gamma_01;      //update the error covariance

}




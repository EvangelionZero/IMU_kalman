
// --- Kalman Filter ---
  #define sample_time  0.001  // 1ms
  #define pi              3.14159
    //--- kalman ------------------------------------------------
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
	//noise parameter
    float gyro_noise_cut = 0.0001;
    float magn_noise_cut = 80;

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


void kalman_angle_filter()
{
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






/* void kalman_angle_filter()
{
    float temp1=0.0, temp2=0.0;
    float sin_beta1 = sin(beta1_kalman);
    float cos_beta1 = cos(beta1_kalman);
    float sin_alpha = sin(alpha_kalman);
    float cos_alpha = cos(alpha_kalman);

//    //rad
    alpha = -atan(X_accl/Z_accl) - 0.004;
    beta1  =  atan(Y_accl/Z_accl) + 0.007;
		alpha_deg = alpha*rad_to_deg;
    beta1_deg = beta1*rad_to_deg;
    
		alpha_kalman_deg = alpha_kalman*rad_to_deg;
    beta1_kalman_deg = beta1_kalman*rad_to_deg;
    gamma_kalman_deg = gamma_kalman*rad_to_deg;
	  
	
    // --- calculate bearing angle from compass --------------------
    temp1 = Z_magn*sin_beta1-Y_magn*cos_beta1;
    temp2 = X_magn*cos_alpha+Y_magn*sin_alpha*sin_beta1+Z_magn*sin_alpha*cos_beta1;
    gamma_measure = atan(temp1/temp2);
		gamma_deg = gamma_measure*rad_to_deg;
//    printf("________________________\n");
//    printf("temp1 = %f   ",temp1);
//    printf("temp2 = %f   ",temp2);
//    printf("gamma_measure = %f  ",gamma_measure);
    if(temp2>0)
        gamma = gamma_measure;
    else if(temp1>0)
        gamma = gamma_measure+pi;
    else
        gamma = gamma_measure-pi;
    
		
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


  
} */

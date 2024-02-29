#include "Teensy_IMU_Filter.h"


void IMU_Filter::initFilter(float freq, double gyro[3], double acc[3], double mag[3], double Beta){
    b[0] = -18.849226;
    b[1] = 68.742123;
    b[2] =-19.845744;
    A_inv[0][0] = 1.000057; A_inv[0][1] =  -0.006891; A_inv[0][2] =  0.014601;
    A_inv[1][0] = -0.006891; A_inv[1][1] =  0.943807; A_inv[1][2] =  -0.000186,
    A_inv[2][0] = 0.014601;  A_inv[2][1] =  -0.000186;A_inv[2][2] =  0.992156;
    
    double h_b[3];
    h_b[0] = mag[0] - b[0];
    h_b[1] = mag[1] - b[1];
    h_b[2] = mag[2] - b[2];

    mag[0] = A_inv[0][0]*h_b[0] + A_inv[0][1]*h_b[1] + A_inv[0][2]*h_b[2];
    mag[1] = A_inv[1][0]*h_b[0] + A_inv[1][1]*h_b[1] + A_inv[1][2]*h_b[2];
    mag[2] = A_inv[2][0]*h_b[0] + A_inv[2][1]*h_b[1] + A_inv[2][2]*h_b[2];

    double p = atan2(-acc[0], sqrt(acc[1]*acc[1] + acc[2]*acc[2]));
    double r = atan2(acc[1], acc[2]);
    
    double b[3];
    b[0] = cos(r)*mag[0] + sin(r)*sin(p)*mag[1] + sin(r)*cos(p)*mag[2];
    b[1] = 0*mag[0] + cos(p)*mag[1] - sin(p)*mag[2];
    b[2] = -sin(r)*mag[0] + cos(r)*sin(p)*mag[1] + cos(r)*cos(p)*mag[2]; 

    double y = atan2(-b[1], b[0]);
    double cr = cos(r * 0.5);
    double sr = sin(r * 0.5);
    double cp = cos(p * 0.5);
    double sp = sin(p * 0.5);
    double cy = cos(y * 0.5);
    double sy = sin(y * 0.5);
    dt = 1/freq;
    q_est[0] = cr * cp * cy + sr * sp * sy;
    q_est[1] = sr * cp * cy - cr * sp * sy;
    q_est[2] = cr * sp * cy + sr * cp * sy;
    q_est[3] = cr * cp * sy - sr * sp * cy;
    beta = Beta;

    Serial.print(q_est[0]);
    Serial.print(" , ");
    Serial.print(q_est[1]);
    Serial.print(" , ");
    Serial.print(q_est[2]);
    Serial.print(" , ");
    Serial.print(q_est[3]);
    Serial.println(" , ");
}

void IMU_Filter::setHardIronOffset(float b_HI[3]){

}
void IMU_Filter::setSoftIronOffset(float S_I[3][3]){

}
void IMU_Filter::bandStopFilter(float K, float center_freq, float Q){

}

void IMU_Filter::madgwickFilter6DOF(double gyro[3], double acc[3]){
    
    double acc_norm = sqrt((acc[0]*acc[0]) + (acc[1]*acc[1]) + (acc[2]*acc[2]));
    
    
    acc[0] /= acc_norm;
    acc[1] /= acc_norm;
    acc[2] /= acc_norm;

    double qt_w_dot[4];
    
    double q1, q2, q3, q0;
    
    q0 = q_est[0];
    q1 = q_est[1];
    q2 = q_est[2];
    q3 = q_est[3];
    double ax, ay, az;
    
    ax = acc[0];
    ay = acc[1];
    az = acc[2];
    
    
    qt_w_dot[0] = 0.5 * ((-q1*gyro[0])+(-q2*gyro[1])+(-q3*gyro[2]));
    qt_w_dot[1] = 0.5 * (( q0*gyro[0])+(-q3*gyro[1])+( q2*gyro[2]));
    qt_w_dot[2] = 0.5 * (( q3*gyro[0])+( q0*gyro[1])+(-q1*gyro[2]));
    qt_w_dot[3] = 0.5 * ((-q2*gyro[0])+( q1*gyro[1])+( q0*gyro[2]));

    double f[3];
    f[0] = 2*(q1*q3 - q0*q2) - ax;
    f[1] = 2*(q0*q1 + q2*q3) - ay;
    f[2] = 2*(0.5 - q1*q1 -q2*q2) - az;
    
    double j[3][4] = {{-2*q2, 2*q3, -2*q0, 2*q1}, {2*q1, 2*q0, 2*q3, 2*q2}, {0, -4*q1, -4*q2, 0}};
    
    double jt[4][3];

    for (int i = 0; i < 3; i++){
        for (int k = 0; k < 4; k++){
            jt[k][i] = j[i][k];
        }
    }
    double gradf[4];

    gradf[0] = (jt[0][0]*f[0]) + (jt[0][1]*f[1]) + (jt[0][2]*f[2]);
    gradf[1] = (jt[1][0]*f[0]) + (jt[1][1]*f[1]) + (jt[1][2]*f[2]);
    gradf[2] = (jt[2][0]*f[0]) + (jt[2][1]*f[1]) + (jt[2][2]*f[2]);
    gradf[3] = (jt[3][0]*f[0]) + (jt[3][1]*f[1]) + (jt[3][2]*f[2]);
    
    double gradf_norm = sqrt(gradf[0]*gradf[0] + gradf[1]*gradf[1] + gradf[2]*gradf[2] + gradf[3]*gradf[3]);

    gradf[0] /= gradf_norm;
    gradf[1] /= gradf_norm;
    gradf[2] /= gradf_norm;
    gradf[3] /= gradf_norm;

    double q_est_dot[4];
    q_est_dot[0] = qt_w_dot[0] - (beta*gradf[0]);
    q_est_dot[1] = qt_w_dot[1] - (beta*gradf[1]);
    q_est_dot[2] = qt_w_dot[2] - (beta*gradf[2]);
    q_est_dot[3] = qt_w_dot[3] - (beta*gradf[3]);

    q_est[0] += q_est_dot[0]*dt;
    q_est[1] += q_est_dot[1]*dt;
    q_est[2] += q_est_dot[2]*dt;
    q_est[3] += q_est_dot[3]*dt;

    double q_est_norm = sqrt(q_est[0]*q_est[0] + q_est[1]*q_est[1] + q_est[2]*q_est[2] + q_est[3]*q_est[3]);
    q_est[0] /= q_est_norm;
    q_est[1] /= q_est_norm;
    q_est[2] /= q_est_norm;
    q_est[3] /= q_est_norm;

    q0 = q_est[0];
    q1 = q_est[1];
    q2 = q_est[2];
    q3 = q_est[3];

    
    roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
    pitch = asin(2*(q0*q2-q3*q1));
    
    
}

void IMU_Filter::madgwickFilter9DOF(double gyro[3], double acc[3], double mag[3]){
    
    double acc_norm = sqrt((acc[0]*acc[0]) + (acc[1]*acc[1]) + (acc[2]*acc[2]));
    
    // Magnetometer Calibration
    double h_b[3];
    h_b[0] = mag[0] - b[0];
    h_b[1] = mag[1] - b[1];
    h_b[2] = mag[2] - b[2];

    
    mag[0] = A_inv[0][0]*h_b[0] + A_inv[0][1]*h_b[1] + A_inv[0][2]*h_b[2];
    mag[1] = A_inv[1][0]*h_b[0] + A_inv[1][1]*h_b[1] + A_inv[1][2]*h_b[2];
    mag[2] = A_inv[2][0]*h_b[0] + A_inv[2][1]*h_b[1] + A_inv[2][2]*h_b[2];

    acc[0] /= acc_norm;
    acc[1] /= acc_norm;
    acc[2] /= acc_norm;
    double mag_norm = sqrt((mag[0]*mag[0]) + (mag[1]*mag[1]) + (mag[2]*mag[2]));

    mag[0] /= mag_norm;
    mag[1] /= mag_norm;
    mag[2] /= mag_norm;
    
    double qt_w_dot[4];
    
    double q1, q2, q3, q0;
    
    q0 = q_est[0];
    q1 = q_est[1];
    q2 = q_est[2];
    q3 = q_est[3];
    /*
    Serial.print(q0, 7); Serial.print(",");
    Serial.print(q1, 7); Serial.print(",");
    Serial.print(q2, 7); Serial.print(",");
    Serial.print(q2, 7); Serial.print(",");
    */
    double ax, ay, az;
    
    ax = acc[0];
    ay = acc[1];
    az = acc[2];
    
    double mx, my, mz;
    mx = mag[0];
    my = mag[1];
    mz = mag[2];
    
    double hx, hy, hz;
    double bx, by, bz;

    hx = (mx*(2*q0*q0 + 2*q1*q1 -1)) + (my*2*(q1*q2 + q0*q3)) + (mz*2*(q1*q3 - q0*q2));
    hy = (mx*2*(q1*q2 - q0*q3)) + (my*(2*q0*q0 + 2*q2*q2 - 1)) + (mz*2*(q2*q3 + q0*q1));
    hz = (mx*2*(q1*q3 + q0*q2)) + (my*2*(q2*q3  - q0*q1)) + (mz*(2*q0*q0 + 2*q3*q3 - 1));
    
    bx = sqrt(hx*hx + hy*hy);
    by = 0;
    bz = hz;
    /*
    Serial.print(mx, 7); Serial.print(",");
    Serial.print(my, 7); Serial.print(",");
    Serial.print(mz, 7); Serial.print(",");
    */
    
    qt_w_dot[0] = 0.5 * ((-q1*gyro[0])+(-q2*gyro[1])+(-q3*gyro[2]));
    qt_w_dot[1] = 0.5 * (( q0*gyro[0])+(-q3*gyro[1])+( q2*gyro[2]));
    qt_w_dot[2] = 0.5 * (( q3*gyro[0])+( q0*gyro[1])+(-q1*gyro[2]));
    qt_w_dot[3] = 0.5 * ((-q2*gyro[0])+( q1*gyro[1])+( q0*gyro[2]));

    double f[6];
    f[0] = 2*(q1*q3 - q0*q2) - ax;
    f[1] = 2*(q0*q1 + q2*q3) - ay;
    f[2] = 2*(0.5 - q1*q1 -q2*q2) - az;
    f[3] = 2*bx*(0.5 - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2) - mx;
    f[4] = 2*bx*(q1*q2 - q0*q0) + 2*bz*(q0*q1 + q2*q3) - my;
    f[5] = 2*bx*(q0*q2 + q1*q3) + 2*bz*(0.5 - q1*q1 - q2*q2) - mz;
    /*
    Serial.print(f[0], 7); Serial.print(",");
    Serial.print(f[1], 7); Serial.print(",");
    Serial.print(f[2], 7); Serial.print(",");
    Serial.print(f[3], 7); Serial.print(",");
    Serial.print(f[4], 7); Serial.print(",");
    Serial.print(f[5], 7); Serial.println(",");
    */
    double j[6][4] = {{-2*q2, 2*q3, -2*q0, 2*q1},
                     {2*q1, 2*q0, 2*q3, 2*q2},
                     {0, -4*q1, -4*q2, 0},
                     {-2*bz*q2, 2*bz*q3, -4*bx*q2 - 2*bz*q0, -4*bx*q3 + 2*bz*q1},
                     {-2*bx*q3 + 2*bz*q1, 2*bx*q2 + 2*bz*q0, 2*bx*q1 + 2*bz*q3,  -2*bx*q0 + 2*bz*q2},
                     {2*bx*q2, 2*bx*q3-4*bz*q1, 2*bx*q0-4*bz*q2,  2*bx*q1}};

    double jt[4][6];

    for (int i = 0; i < 6; i++){
        for (int k = 0; k < 4; k++){
            jt[k][i] = j[i][k];
        }
    }
    double gradf[4];

    gradf[0] = (jt[0][0]*f[0]) + (jt[0][1]*f[1]) + (jt[0][2]*f[2]) + (jt[0][3]*f[3]) + (jt[0][4]*f[4]) + (jt[0][5]*f[5]);
    gradf[1] = (jt[1][0]*f[0]) + (jt[1][1]*f[1]) + (jt[1][2]*f[2]) + (jt[1][3]*f[3]) + (jt[1][4]*f[4]) + (jt[1][5]*f[5]);
    gradf[2] = (jt[2][0]*f[0]) + (jt[2][1]*f[1]) + (jt[2][2]*f[2]) + (jt[2][3]*f[3]) + (jt[2][4]*f[4]) + (jt[2][5]*f[5]);
    gradf[3] = (jt[3][0]*f[0]) + (jt[3][1]*f[1]) + (jt[3][2]*f[2]) + (jt[3][3]*f[3]) + (jt[3][4]*f[4]) + (jt[3][5]*f[5]);
    
    double gradf_norm = sqrt(gradf[0]*gradf[0] + gradf[1]*gradf[1] + gradf[2]*gradf[2] + gradf[3]*gradf[3]);
    
    gradf[0] /= gradf_norm;
    gradf[1] /= gradf_norm;
    gradf[2] /= gradf_norm;
    gradf[3] /= gradf_norm;

    double q_est_dot[4];
    q_est_dot[0] = qt_w_dot[0] - (beta*gradf[0]);
    q_est_dot[1] = qt_w_dot[1] - (beta*gradf[1]);
    q_est_dot[2] = qt_w_dot[2] - (beta*gradf[2]);
    q_est_dot[3] = qt_w_dot[3] - (beta*gradf[3]);

    q_est[0] += q_est_dot[0]*dt;
    q_est[1] += q_est_dot[1]*dt;    
    q_est[2] += q_est_dot[2]*dt;    
    
    double q_est_norm = sqrt(q_est[0]*q_est[0] + q_est[1]*q_est[1] + q_est[2]*q_est[2] + q_est[3]*q_est[3]);
    q_est[0] /= q_est_norm;
    q_est[1] /= q_est_norm;
    q_est[2] /= q_est_norm;
    q_est[3] /= q_est_norm;

    q0 = q_est[0];
    q1 = q_est[1];
    q2 = q_est[2];
    q3 = q_est[3];
    roll = atan2(2*(q0*q2 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
    pitch = (-3.14159265358979/2) + 2*atan2(sqrt(1 + 2*(q0*q2-q1*q3)), sqrt(1 - 2*(q0*q2-q1*q3)));
    yaw =  atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
    
}

void IMU_Filter::quat2rpy(){
    double q0, q1, q2, q3;
    roll = atan2(2*(q0*q2 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
    pitch = (-3.14159265358979/2) + 2*atan2(sqrt(1 + 2*(q0*q2-q1*q3)), sqrt(1 - 2*(q0*q2-q1*q3)));
    yaw =  atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
}
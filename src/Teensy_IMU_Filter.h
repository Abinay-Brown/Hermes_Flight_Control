#include "Arduino.h"
#ifndef Teensy_IMU_Filter_h
#define Teensy_IMU_Filter_h

class IMU_Filter{
        
    public:
        double q_est[4]; // Estimated Quaternion
        double roll;
        double pitch;
        double yaw;

        double beta;
        double dt;
        double b[3];
        double A_inv[3][3];
        void initFilter(float freq, double gyro[3], double acc[3], double mag[3], double beta);
        void setHardIronOffset(float b_HI[3]);
        void setSoftIronOffset(float S_I[3][3]);
        void bandStopFilter(float K, float center_freq, float Q);
        void madgwickFilter6DOF(double gyro[3], double acc[3]);
        void madgwickFilter9DOF(double gyro[3], double acc[3], double mag[3]);
        void quat2rpy();
            
    private:
         
};
#endif
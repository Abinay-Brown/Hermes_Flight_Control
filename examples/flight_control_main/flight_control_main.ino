#include "Teensy_OneShot125.h"
#include "Teensy_IMU_Filter.h"
#include <PulsePosition.h>

#include "ICM_20948.h" 
#define CS_PIN 10     


ICM_20948_SPI myICM; 

PulsePositionInput Receiver(RISING);
float channels[] = {0, 0, 0, 0, 0, 0};
int channelNum = 0;
void readReceiver();

  double tprev;
uint8_t motor_pins[] = {1, 2, 3, 4}; // FR, FL, RL, RR
OneShot125 motor_control(motor_pins, 4, 1000);
IMU_Filter ICM;

void setup(){
  Serial.begin(115200); 
  //motor_control.calibrateMotors();
  SPI.begin();

  myICM.begin(CS_PIN, SPI);
  delay(250);

  myICM.sleep(false);
  myICM.lowPower(false);
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok);

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 0;
  mySmplrt.a = 0;
  myFSS.a = gpm16; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  myFSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
   
  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  myICM.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt);
  if (myICM.status != ICM_20948_Stat_Ok);
  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok);
  
  if (myICM.dataReady()){
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
  }

  double gyro[3] = {myICM.gyrX(), myICM.gyrY(), myICM.gyrZ()};
  double acc[3] = {myICM.accX(), myICM.accY(), myICM.accZ()};
  double mag[3] = {myICM.magX()*1000, -myICM.magY()*1000, -myICM.magZ()*1000};  
  ICM.initFilter(1000, gyro, acc, mag, 0.866);
  Receiver.begin(14);  
}

void loop(){
  double t0 = micros();
  /*
  double t0 = micros();
  double throttle[] = {170, 127, 170, 127};
  motor_control.writeMotors(throttle);
  double t1 = micros();
  motor_control.loopDelay(t1-t0);
    */
    if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
  }
  double gyro[3] = {myICM.gyrX()*0.0174533, myICM.gyrY()*0.0174533, myICM.gyrZ()*0.0174533};
  double acc[3] = {myICM.accX(), myICM.accY(), myICM.accZ()};
  double mag[3] = {myICM.magX()*1000, -myICM.magY()*1000, -myICM.magZ()*1000};  
  ICM.madgwickFilter6DOF(gyro, acc);
  Serial.print(ICM.roll*180/3.14159265358979);
  Serial.print(" , ");
  Serial.print(ICM.pitch*180/3.14159265358979);
  Serial.print(" , ");
  Serial.print(ICM.yaw*180/3.14159265358979);
  Serial.print(" , ");
  
  readReceiver();
  Serial.print(channels[0]);
  Serial.print(" , ");
  Serial.print(channels[1]);
  Serial.print(" , ");
  Serial.print(channels[2]);
  Serial.print(" , ");
  Serial.print(channels[3]);
  Serial.print(" , ");
  Serial.print(channels[4]);
  Serial.print(" , ");
  Serial.print(channels[5]);
  Serial.print(" , ");
  Serial.print(tprev);
  Serial.println("  ");  
  double t1 = micros();
  tprev = t1-t0;
  delayMicroseconds(1000-(t1-t0));

}

void readReceiver(void){
    channelNum = Receiver.available();
    if (channelNum > 0){
      for (int i = 1; i<=channelNum; i++){
        channels[i-1] = Receiver.read(i);
      }
    }
}
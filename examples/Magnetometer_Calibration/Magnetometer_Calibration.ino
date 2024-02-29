#include "Teensy_IMU_Filter.h"
#include "ICM_20948.h" 

#define CS_PIN 10     

ICM_20948_SPI myICM; 
IMU_Filter ICM;

void setup(){
    Serial.begin(115200); 
    SPI.begin();

    bool initialized = false;
    while (!initialized)
    {
        myICM.begin(CS_PIN, SPI);

        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  delay(250);

  myICM.sleep(false);
  myICM.lowPower(false);

  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setSampleMode returned: "));
    Serial.println(myICM.statusString());
  }

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 0;
  mySmplrt.a = 0;
  myFSS.a = gpm16; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  myFSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
   
  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  myICM.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setFullScale returned: "));
    Serial.println(myICM.statusString());
  }

  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("startupMagnetometer returned: "));
    Serial.println(myICM.statusString());
  }
  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
  }

  double gyro[3] = {myICM.gyrX(), myICM.gyrY(), myICM.gyrZ()};
  double acc[3] = {myICM.accX(), myICM.accY(), myICM.accZ()};
  double mag[3] = {myICM.magX()*1000, -myICM.magY()*1000, -myICM.magZ()*1000};  
  ICM.initFilter(1000, gyro, acc, mag, 0.05);
    
}

void loop(){
   double t0 = micros();
  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
  }

  double gyro[3] = {myICM.gyrX()*0.0174533, myICM.gyrY()*0.0174533, myICM.gyrZ()*0.0174533};
  double acc[3] = {myICM.accX(), myICM.accY(), myICM.accZ()};
  double mag[3] = {myICM.magX()*1000, -myICM.magY()*1000, -myICM.magZ()*1000};  
  ICM.madgwickFilter6DOF(gyro, acc);
  Serial.print(mag[0], 6); Serial.print(",");
  Serial.print(mag[1], 6); Serial.print(",");
  Serial.println(mag[2], 6);   
  double t1 = micros();
  
  delayMicroseconds(1000-(t1-t0));

}
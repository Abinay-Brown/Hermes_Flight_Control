#ifndef Teensy_OneShot125_h
#define Teensy_OneShot125_h

#include "Arduino.h"

class OneShot125
{
    public:
        uint8_t output_pins[10];
        uint8_t output_num_pins;
        uint16_t output_freq;
        OneShot125(uint8_t pins[], uint8_t num_pins,  uint16_t freq);
        void calibrateMotors();
        void writeMotors(float microseconds[]);
        void loopDelay(float elapsedTimeMicros);

    private:        
    
        void delayNanos(uint32_t nsec);
        void Nanos();
    
};
#endif

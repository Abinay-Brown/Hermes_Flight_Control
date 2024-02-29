#include "Teensy_OneShot125.h"


OneShot125::OneShot125(uint8_t pins[], uint8_t num_pins, uint16_t freq){
    output_num_pins = num_pins;
    output_freq = freq;
    for (int i = 0; i < output_num_pins; i++){
        output_pins[i] = pins[i];
        pinMode(output_pins[i], OUTPUT);
    }
}

void OneShot125::calibrateMotors(){
    // Function sets the Max Throttle and Min Throttle for the ESCs
    delay(5000);  
    double in = micros();
    int i = 0;
    while (micros()-in < 1000000){
        for (i = 0; i < output_num_pins; i++){
            digitalWriteFast(output_pins[i], HIGH);
        }
        delayNanos(125000);
        for (i = 0; i < output_num_pins; i++){
            digitalWriteFast(output_pins[i], LOW);
        }
        delayNanos(875000);
    
    }

    in = micros();

    while (micros()-in < 10000000){
        for (i = 0; i < output_num_pins; i++){
            digitalWriteFast(output_pins[i], HIGH);
        }
        delayNanos(250000);
        for (i = 0; i < output_num_pins; i++){
            digitalWriteFast(output_pins[i], LOW);
        }
        delayNanos(750000);
    
    }

    in = micros();

    while (micros()-in < 10000000){
        for (i = 0; i < output_num_pins; i++){
            digitalWriteFast(output_pins[i], HIGH);
        }
        delayNanos(125000);
        for (i = 0; i < output_num_pins; i++){
            digitalWriteFast(output_pins[i], LOW);
        }
        delayNanos(875000);
    
    }

}

void OneShot125::writeMotors(float microseconds[]){
    float max = 125;
    int8_t max_ind = 0;
    int i = 0;
    for (i = 0; i < output_num_pins; i++){
        if (microseconds[i]>max){
            max = microseconds[i];
            max_ind = i;
        }
    }
    
    uint32_t output_cycles[output_num_pins];
    
    for (i = 0; i < output_num_pins; i++){
        output_cycles[i] = ((F_CPU_ACTUAL>>16) * microseconds[i] * 1000) / (1000000000UL>>16);
    }
    
    uint32_t begin = ARM_DWT_CYCCNT;

    for (i = 0; i < output_num_pins; i++){
            digitalWriteFast(output_pins[i], HIGH);
    }
    

    while (ARM_DWT_CYCCNT - begin < output_cycles[max_ind]){
        for (i = 0; i < output_num_pins; i++){
            if (ARM_DWT_CYCCNT - begin > output_cycles[i]){
                digitalWriteFast(output_pins[i], LOW);
            }
        }
    }
    for (i = 0; i < output_num_pins; i++){
            if (ARM_DWT_CYCCNT - begin > output_cycles[i]){
                digitalWriteFast(output_pins[i], LOW);
            }
        }
    
}

void OneShot125::loopDelay(float elapsedTimeMicros){
    float loop_time = 1/output_freq * 1000000;
    delayMicroseconds(loop_time-elapsedTimeMicros);
}

void OneShot125::delayNanos(uint32_t nsec){
    uint32_t begin = ARM_DWT_CYCCNT;
    uint32_t cycles =   ((F_CPU_ACTUAL>>16) * nsec) / (1000000000UL>>16);
    while (ARM_DWT_CYCCNT - begin < cycles) ; // wait
}

void OneShot125::Nanos(){

}

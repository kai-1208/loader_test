#include "mbed.h"
#include "key.hpp"
#include "serial_read.hpp"

#define PWM_OUTPUT 3000

BufferedSerial pc(USBTX, USBRX, 115200);

serial_unit serial(pc);

CAN can1(PA_11, PA_12, (int)1e6);

int16_t pwm[4] = {0};

void button_event() {
    while (1) {
        if (Triangle) {
            pwm[0] = PWM_OUTPUT;
        } else if (Cross) {
            pwm[0] = -PWM_OUTPUT;
        } else {
            pwm[0] = 0;
        }
        if (Up) {
            pwm[1] = PWM_OUTPUT;
        } else if (Down) {
            pwm[1] = -PWM_OUTPUT;
        } else {
            pwm[1] = 0;
        }
        printf("pwm: %d, %d\n", pwm[0], pwm[1]);
        ThisThread::sleep_for(10ms);
    }
}

void can_send() {
    while(1) {
        CANMessage msg(2, (const uint8_t*)pwm, 8);
        can1.write(msg);
        ThisThread::sleep_for(10ms);
    }
}


int main() {
    Thread serial_thread;
    serial_thread.start(serial_read);
    Thread button_thread;
    button_thread.start(button_event);
    Thread can_thread;
    can_thread.start(can_send);
    while (1) {
    }
}
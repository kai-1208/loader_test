#include "mbed.h"
#include "key.hpp"
#include "serial_read.hpp"
#include "c610.hpp"
#include "pid.hpp"

#define PWM_OUTPUT 7000

BufferedSerial pc(USBTX, USBRX, 115200);

serial_unit serial(pc);

CAN can1(PA_11, PA_12, (int)1e6);
C610 DJI(can1);

const float Kp = 0.7f;
const float Ki = 0.0f;
const float Kd = 0.0f;

PID roller_pid(Kp, Ki, Kd, PID::Mode::VELOCITY);

int16_t pwm[4] = {0};

void pid_control() {
    int wheel_min_driver_power = 100;
    int wheel_output_limits = 16000;
    roller_pid.set_min_drive_power(wheel_min_driver_power);
    roller_pid.set_output_limits(-wheel_output_limits, wheel_output_limits);
    roller_pid.enable_anti_windup(true);

    auto pre_time = HighResClock::now();
    while (1) {
        auto now_time = HighResClock::now();
        float dt = std::chrono::duration_cast<std::chrono::microseconds>(now_time - pre_time).count() / 1000000.0f;
        roller_pid.set_dt(dt);
        pre_time = now_time;

        ThisThread::sleep_for(10ms);
    }
}

void button_event() {
    while (1) {
        if (Triangle) {
            pwm[0] = PWM_OUTPUT;
            roller_pid.set_goal(10000);
        } else if (Cross) {
            pwm[0] = -PWM_OUTPUT;
            roller_pid.set_goal(-10000);
        } else {
            pwm[0] = 0;
            roller_pid.set_goal(0);
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
        // CANMessage msg(2, (const uint8_t*)pwm, 8);
        // can1.write(msg);
        DJI.set_power(1, roller_pid.do_pid(DJI.get_rpm(1)));
        if (DJI.send_message()) {
            printf("ID1: rpm=%d, power=%d\n", DJI.get_rpm(1), DJI.get_send_power(1));
        } else {
            printf("can failed\n");
        }
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
    Thread pid_thread;
    pid_thread.start(pid_control);
    while (1) {
    }
}
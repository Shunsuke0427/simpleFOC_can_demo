/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "./library/SimpleFOC_Mbed/SimpleFOC.h"
#include <cstdio>
#include "FastPWM.h"


// Blinking rate in milliseconds
#define BLINKING_RATE     500ms
#define PP 7

#define VEL_EXAMPLE

#ifndef VEL_EXAMPLE
#define ANG_EXAMPLE
#endif

//BufferedSerial pc(USBTX, USBRX);

DigitalOut led(LED1);

/* //F303
FastPWM pwm_A(D0);
FastPWM pwm_B(D1);
FastPWM pwm_C(D9);
DigitalOut en(D8);
InterruptIn in_A(D4);
InterruptIn in_B(D5);
InterruptIn in_C(D7);
*/
 FastPWM pwm_A(PC_6);
 FastPWM pwm_B(PC_7);
 FastPWM pwm_C(PB_8);
 DigitalOut en(PB_1);
 InterruptIn in_A(PA_4);
 InterruptIn in_B(PA_5);
 InterruptIn in_C(PA_6);



CAN can(PA_11, PA_12);
BufferedSerial serial(USBTX,USBRX, 115200);

BLDCMotor motor = BLDCMotor(PP);
BLDCDriver3PWM driver = BLDCDriver3PWM(&pwm_A, &pwm_B, &pwm_C, &en);
HallSensor sensor = HallSensor(in_A, in_B, in_C, PP);

Commander command = Commander(serial);

void doA() {sensor.handleA();}
void doB() {sensor.handleB();}
void doC() {sensor.handleC();}

#ifdef ANG_EXAMPLE
void hello(char *cmd);
void disableMotor(char* cmd);
void enableMotor(char* cmd);
void setTarget_ang(char* cmd);
void onPID_vel(char* cmd);
void onPID_ang(char* cmd);
void getMotorValue(char* cmd);

float target_angle = 0;

int main()
{
    Ticker ticker;
    command.add('a', hello, "hello world");
    command.add('d', disableMotor, "disable motor");
    command.add('e', enableMotor, "enable motor");
    command.add('t', setTarget_ang, "set target angle");
    command.add('p', onPID_vel, "velocoty PID setting");
    command.add('P', onPID_ang, "angle P setting");
    command.add('m', getMotorValue, "motor value");

    command.echo = true;

    
    driver.pwm_frequency = 20000;
    
    sensor.pullup = Pullup::USE_INTERN;
    sensor.init();
    sensor.enableInterrupts(doA, doB, doC);
    
    motor.linkSensor(&sensor);
    driver.voltage_power_supply = 12;
    
    driver.init();
    
    motor.linkDriver(&driver);
    
    motor.voltage_sensor_align = 0.2;
    motor.velocity_index_search = 5;

    motor.controller = MotionControlType::angle;

    motor.PID_velocity.P = 0.072;
    motor.PID_velocity.I = 0.06;
    motor.PID_velocity.D = 0;//.01;
    motor.PID_velocity.limit = 50;
    motor.voltage_limit = 10;
    motor.PID_velocity.output_ramp = 100;
    motor.LPF_velocity.Tf = 0.1;



    motor.P_angle.P = 10;
    motor.P_angle.I = 0;
    motor.P_angle.output_ramp = 50;
    motor.velocity_limit = 50;
    motor.useMonitoring(serial);
    
    motor.init();
    motor.initFOC();

    ThisThread::sleep_for(1s);
    printf("control start\n");
    printf("%d\n", pwm_A.read_period_us());
    

    led = 0;
    while (true) {
        
        motor.loopFOC();
        motor.move(target_angle);
        
        command.run();
        //led = 0;
        
    }
}

void hello(char* cmd) {
    /*if(led) led = 0;
    else led = 1;*/
}

void disableMotor(char* cmd) {
    motor.disable();
    printf("motor disabled\n");
}
void enableMotor(char* cmd) {
    motor.enable();
    printf("motor enabled\n");
}
void setTarget_ang(char* cmd) {
    command.scalar(&target_angle, cmd);
}
void getMotorValue(char* cmd) {
    command.motor(&motor, cmd);
}
void onPID_vel(char* cmd) {
    command.pid(&motor.PID_velocity, cmd);
}
void onPID_ang(char* cmd) {
    command.pid(&motor.P_angle, cmd);
}
#endif





#ifdef VEL_EXAMPLE

#define MAX_VEL 100
void hello(char* cmd);
void onPID_vel(char* cmd);
void setTarget_vel(char* cmd);
void getMotorValue(char* cmd);
void setTf_vel(char* cmd);
void setTf_ang(char* cmd);
void tickerLoop();
void canRead();

float target_velocity = 0;
int loop_count = 0;

int main()
{
    Ticker ticker;
    command.add('a', hello, "hello world");
    command.add('p', onPID_vel, "velocity PID setting");
    command.add('t', setTarget_vel, "set target velocity");
    command.add('g', getMotorValue, "get motor parameter");
    command.add('l', setTf_vel, "set velocity Tf");
    command.add('L', setTf_ang, "set angle Tf");
    command.echo = true;

    
    driver.pwm_frequency = 20000;
    
    sensor.pullup = Pullup::USE_INTERN;
    sensor.init();
    sensor.enableInterrupts(doA, doB, doC);
    
    motor.linkSensor(&sensor);
    driver.voltage_power_supply = 12;
    
    driver.init();
    
    motor.linkDriver(&driver);
    
    motor.voltage_sensor_align = 0.3;

    motor.controller = MotionControlType::velocity;
//PwmOut parameters
/* 
    motor.PID_velocity.P = 0.048;
    motor.PID_velocity.I = 0.04;
    motor.PID_velocity.D = 0;//.01;
    motor.LPF_velocity.Tf = 0.1;
    motor.LPF_angle.Tf = 0.00001;
    */

//FastPWM parameters
motor.PID_velocity.P = 0.072;
    motor.PID_velocity.I = 0.06;
    motor.PID_velocity.D = 0;//.01;
    motor.PID_velocity.limit = 50;
    motor.voltage_limit = 10;
    motor.PID_velocity.output_ramp = 100;
    motor.LPF_velocity.Tf = 0.1;

    motor.velocity_limit = 50;
    motor.useMonitoring(serial);
    
    motor.init();
    motor.initFOC();

    ThisThread::sleep_for(1s);
    printf("control start\n");
    printf("%d\n", pwm_A.read_period_us());
    NVIC_SetPriority(TIM2_IRQn,1); //F4393k8
    //ticker.attach_us(&ticker_loop,500);
    
    
    led = 0;
    while (true) {
        command.run();
        tickerLoop();
        canRead();
        
    }
}



void hello(char* cmd) {
    
}

void onPID_vel(char* cmd) {
    command.pid(&motor.PID_velocity, cmd);
}
void setTarget_vel(char* cmd) {
    command.scalar(&target_velocity, cmd);
}

void getMotorValue(char* cmd) {
    command.motor(&motor, cmd);
}

void setTf_vel(char* cmd) {
    command.scalar(&motor.LPF_velocity.Tf, cmd);
}

void setTf_ang(char* cmd) {
    command.scalar(&motor.LPF_angle.Tf, cmd);
}

void tickerLoop() {
    motor.loopFOC();
    motor.move(target_velocity);
    loop_count++;
    if(loop_count > 5000) {
        //led = !led;
        loop_count = 0;
    }
}

void canRead() {
    CANMessage msg;
    
    if (can.read(msg)) {
        char received_array[4];
        union {float f; int i;} dataUnion;
        int receiver_int;
        //printf("received\n");
        for(int i=0; i<4; i++) {
            received_array[i] = msg.data[i];
            //printf("Message received: %d\n", received_array[i]);
        }

        for(int i=3; i>-1; i--) {
            receiver_int = (receiver_int << 8) | received_array[i];
        }
        dataUnion.i = receiver_int;
        //printf("int: %d  float: %f\n", receiver_int, dataUnion.f);
        led = !led;
        float volume = dataUnion.f;
        if((volume >= 0) && (volume <= 1)) target_velocity = MAX_VEL * volume;
    }
}

#endif
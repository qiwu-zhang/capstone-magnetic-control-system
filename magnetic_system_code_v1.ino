// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>
#include <PID_v1.h>

#define ACTSTEP 7300
#define MAXLIMIT 6000

#define LS1GND 23
#define LS1VDD 25
#define LS1OUT 27

#define LS2GND 22
#define LS2VDD 24
#define LS2OUT 26

#define LF          0x0A

char cmd_from_pc[40];
int  cmd_idx = 0;

AccelStepper stepper2(1, 2, 3); // (Type of driver: with 2 pins, STEP, DIR)
AccelStepper stepper1(1, 4, 5); // (Type of driver: with 2 pins, STEP, DIR)

void Home();
void center();

//Define Variables we'll be connecting to
double x_Setpoint, x_Input, x_Output;
double y_Setpoint, y_Input, y_Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=40, aggKi=2, aggKd=10;
//double consKp=20, consKi=1, consKd=5;
double consKp=0.1, consKi=0.001, consKd=10;

//Specify the links and initial tuning parameters
PID x_PID(&x_Input, &x_Output, &x_Setpoint, consKp, consKi, consKd, DIRECT);
PID y_PID(&y_Input, &y_Output, &y_Setpoint, consKp, consKi, consKd, DIRECT);

void setup()
{ 
  x_Setpoint = 200;
  y_Setpoint = 200;
  stepper1.setMaxSpeed(2000);
  stepper1.setAcceleration(2000);
  stepper1.setCurrentPosition(0); // Set the current position to 0 steps

  stepper2.setMaxSpeed(2000);
  stepper2.setAcceleration(2000);
  stepper2.setCurrentPosition(0); // Set the current position to 0 steps

  pinMode(LS1VDD, OUTPUT);
  pinMode(LS1GND, OUTPUT);
  pinMode(LS1OUT, INPUT);

  pinMode(LS2VDD, OUTPUT);
  pinMode(LS2GND, OUTPUT);
  pinMode(LS2OUT, INPUT);

  digitalWrite(LS1VDD, HIGH);
  digitalWrite(LS1GND, LOW);

  digitalWrite(LS2VDD, HIGH);
  digitalWrite(LS2GND, LOW);
  x_PID.SetMode(AUTOMATIC);
  y_PID.SetMode(AUTOMATIC);
  Serial.begin(115200);
}

void loop()
{
  cmd();
}

void cmd() {
  while (Serial.available() > 0) {
    cmd_from_pc[cmd_idx] = Serial.read();
    if (cmd_from_pc[cmd_idx] == LF) {

      cmd_from_pc[cmd_idx] = 0;
      cmd_idx = -1;

      char cmd_prefix[3];
      strncpy(cmd_prefix, cmd_from_pc, 2);
      cmd_prefix[2] = 0;

      double argument1 = 0;
      double argument2 = 0;
      char * endptr;
      argument1 = strtod(cmd_from_pc + 2, &endptr);
      argument2 = strtod(endptr+1, &endptr);

      //Serial.print("Received new command: ");
      //Serial.print("Command ");
      //Serial.println(cmd_prefix);
      //Serial.print("Argument1 ");
      //Serial.println(argument1);
      //Serial.print("Argument2 ");
      //Serial.println(argument2);
      
      if (!strcmp(cmd_prefix, "hm")) {
        Serial.println("==Homie is Homing==");
        Home();
      } else if (!strcmp(cmd_prefix, "mv")) {
        Serial.println("==Moving to location==");
        move_blocking(argument1, argument2, 2000);}
      else if (!strcmp(cmd_prefix, "pi")) {

        x_Input = argument1;
        y_Input = argument2;
        double x_gap = abs(x_Setpoint-x_Input); //distance away from setpoint
        double y_gap = abs(y_Setpoint-y_Input); //distance away from setpoint

        if(x_gap<1.5){
        }
        else if(x_gap<5){  //we're close to setpoint, use conservative tuning parameters
          x_PID.SetTunings(aggKp, aggKi, aggKd);
          x_PID.Compute();
          stepper1.setSpeed((x_Output-128)*10);
          stepper1.runSpeed();
        }
        else if(x_gap<50){  //we're close to setpoint, use conservative tuning parameters
          x_PID.SetTunings(aggKp, aggKi, aggKd);
          x_PID.Compute();
          stepper1.setSpeed((x_Output-128)*100);
          for (int a =0; a<50;a++){
            stepper1.runSpeed();
          }
        }else{
          //we're far from setpoint, use aggressive tuning parameters
          x_PID.SetTunings(aggKp, aggKi, aggKd);
          x_PID.Compute();
          stepper1.setSpeed((x_Output-128)*500);
          for (int a =0; a<200;a++){
            stepper1.runSpeed();
          }

        }

        if(y_gap<1.5){
        }
        else if(y_gap<5){  //we're close to setpoint, use conservative tuning parameters
          y_PID.SetTunings(aggKp, aggKi, aggKd);
          y_PID.Compute();
          stepper2.setSpeed((y_Output-128)*10);
          stepper2.runSpeed();
        }
        else if(y_gap<50){  //we're close to setpoint, use conservative tuning parameters
          y_PID.SetTunings(aggKp, aggKi, aggKd);
          y_PID.Compute();
          stepper2.setSpeed((y_Output-128)*100);
          for (int a =0; a<50;a++){
            stepper2.runSpeed();
          }
        }else{
          //we're far from setpoint, use aggressive tuning parameters
          y_PID.SetTunings(aggKp, aggKi, aggKd);
          y_PID.Compute();
          stepper2.setSpeed((y_Output-128)*500);
          for (int a =0; a<200;a++){
            stepper2.runSpeed();
          }


        }




        //stepper1.moveTo(go_x);
        //stepper1.setSpeed(x_Output*10);
        //stepper1.runSpeedToPosition();
        //stepper2.moveTo(go_y);
        //stepper2.setSpeed(y_Output*10);
        //stepper2.runSpeedToPosition();
        //stepper1.run();
        //stepper2.run();
        //while (stepper1.currentPosition() != go_x || stepper2.currentPosition() != go_y) {
        //  stepper1.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
        //  stepper2.run();
        //}
        



      } else if (!strcmp(cmd_prefix, "ct")) {
        Serial.println("==Moving to location==");
        move_blocking(0, 0, 2000);
      }
    }
    cmd_idx++;
  }
}

void mv_blocking_dual_speed(int xstep, int ystep, int xspeed,int yspeed) {
  if (abs(xstep) > MAXLIMIT/2 || abs(ystep) > MAXLIMIT /2) {
    Serial.println("Error in move_blocking(): exceed set limit");
    return;
  }
  stepper1.setSpeed(xspeed);
  stepper2.setSpeed(yspeed);

  stepper1.moveTo(xstep);
  stepper2.moveTo(ystep);

  while (stepper1.currentPosition() != xstep || stepper2.currentPosition() != ystep) {
    stepper1.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepper2.run();
  }
}


void move_blocking(int xstep, int ystep, int run_speed) {
  if (abs(xstep) > MAXLIMIT/2 || abs(ystep) > MAXLIMIT /2) {
    Serial.println("Error in move_blocking(): exceed set limit");
    return;
  }
  stepper1.setSpeed(run_speed);
  stepper2.setSpeed(run_speed);

  stepper1.moveTo(xstep);
  stepper2.moveTo(ystep);

  while (stepper1.currentPosition() != xstep || stepper2.currentPosition() != ystep) {
    stepper1.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepper2.run();
  }
}

void Home() {
  stepper1.setSpeed(-1000);
  stepper2.setSpeed(1000);

  while (1) {
    stepper1.runSpeed();
    stepper2.runSpeed();
    if (digitalRead(LS1OUT) == 0) {
      stepper1.setSpeed(0);
    }

    if (digitalRead(LS2OUT) == 0) {
      stepper2.setSpeed(0);
    }

    if (digitalRead(LS1OUT) == 0 && digitalRead(LS2OUT) == 0) {
      break;
    }
  }

  stepper1.setCurrentPosition(-ACTSTEP / 2);
  stepper2.setCurrentPosition(ACTSTEP / 2);
  stepper1.setSpeed(2000);
  stepper2.setSpeed(-2000);

  stepper1.moveTo(0);
  stepper2.moveTo(0);

  while (stepper1.currentPosition() != 0 || stepper2.currentPosition() != 0) {
    stepper1.run();  // Move or step the motor implementing accelerations and decelerations to achieve the target position. Non-blocking function
    stepper2.run();
  }
}

void center(){
  stepper1.moveTo(0);
  stepper2.moveTo(0);
}

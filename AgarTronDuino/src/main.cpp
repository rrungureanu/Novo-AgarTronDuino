#include <Arduino.h>
#include <EnableInterrupt.h>
#include <AccelStepper.h>

//Step motor DIR pins
#define PIN_G1_DIR 30
#define PIN_G2_DIR 29
#define PIN_G3_DIR 28
#define PIN_G4_DIR 27
#define PIN_G5_DIR 26
#define PIN_G6_DIR 25
#define PIN_G7_DIR 24
#define PIN_G8_DIR 23
#define PIN_G9_DIR 22

//Step motor STEP pins
#define PIN_G1_STEP 2
#define PIN_G2_STEP 3
#define PIN_G3_STEP 4
#define PIN_G4_STEP 5
#define PIN_G5_STEP 6
#define PIN_G6_STEP 7
#define PIN_G7_STEP 8
#define PIN_G8_STEP 9
#define PIN_G9_STEP 11

//Sensor pins
#define PIN_S1 48
#define PIN_S2 44
#define PIN_S3 49
#define PIN_S4 45
#define PIN_S5 52
#define PIN_S6 46
#define PIN_S7 47
#define PIN_S8 50
#define PIN_S9 51

//Motor positions
#define POS_INF 0x7FFFFFFF
#define POS_TOP 0
#define POS_G1389_BOT 1
#define POS_G2467_BOT 1
#define POS_G5_BOT 1
#define POS_G5_MID 1
#define POS_G5_L6 1
#define POS_G5_L9 1
#define POS_G5_R6 1
#define POS_G5_R9 1

//Motor movement flags, while loop will step if true
bool flag_G1_step, flag_G2_step, flag_G3_step, flag_G4_step, flag_G5_step, flag_G6_step, flag_G7_step, flag_G8_step, flag_G9_step;

//AccelStepper objects, 
AccelStepper Stepper_G1(1,PIN_G1_STEP,PIN_G1_DIR), Stepper_G2(1,PIN_G2_STEP,PIN_G2_DIR), Stepper_G3(1,PIN_G3_STEP,PIN_G3_DIR), Stepper_G4(1,PIN_G4_STEP,PIN_G4_DIR), Stepper_G5(1,PIN_G5_STEP,PIN_G5_DIR),
             Stepper_G6(1,PIN_G6_STEP,PIN_G6_DIR), Stepper_G7(1,PIN_G7_STEP,PIN_G7_DIR), Stepper_G8(1,PIN_G8_STEP,PIN_G8_DIR), Stepper_G9(1,PIN_G9_STEP,PIN_G9_DIR);

//Hole stack count
unsigned int platecount_L6 = 0, platecount_L9 = 0, platecount_R6 = 0, platecount_R9 = 0; 

//Stack selection + phase enums and flags
bool flag_stack_left = true, flag_L6_active, flag_L6_flushing, flag_L9_active, flag_L9_flushing, flag_R6_active, flag_R6_flushing, flag_R9_active, flag_R9_flushing, flag_G5_ready;
enum phase_L6 {phase_L6_1, phase_L6_2, phase_L6_3, phase_L6_2F, phase_L6_3F, phase_L6_4F, phase_L6_5F, phase_L6_6F}; phase_L6 curr_phase_L6;
enum phase_L9 {phase_L9_1, phase_L9_2, phase_L9_3, phase_L9_2F, phase_L9_3F, phase_L9_4F, phase_L9_5F, phase_L9_6F}; phase_L9 curr_phase_L9;
enum phase_R6 {phase_R6_1, phase_R6_2, phase_R6_3, phase_R6_2F, phase_R6_3F, phase_R6_4F, phase_R6_5F, phase_R6_6F}; phase_R6 curr_phase_R6;
enum phase_R9 {phase_R9_1, phase_R9_2, phase_R9_3, phase_R9_2F, phase_R9_3F, phase_R9_4F, phase_R9_5F, phase_R9_6F}; phase_R9 curr_phase_R9;

//Elevator flush timers
unsigned int timer_L6, timer_L9, timer_R6, timer_R9;
bool timer_L6_wait, timer_L9_wait, timer_R6_wait, timer_R9_wait;

//Sensor is hit, turn step flag off for corresponding motor
void isr_G1(){
  flag_G1_step = false;

}

void isr_G2(){
  flag_G2_step = false;

}

void isr_G3(){
  flag_G3_step = false;

}

void isr_G4(){
  flag_G4_step = false;

}

void isr_G5(){
  flag_G5_step = false;
  Stepper_G5.stop();
  Stepper_G5.runToPosition();
  Serial.println("ISR5");
}

void isr_G6(){
  flag_G6_step = false;

}

void isr_G7(){
  flag_G7_step = false;

}

void isr_G8(){
  flag_G8_step = false;

}

void isr_G9(){
  flag_G9_step = false;

}

void pins_setup(){
  pinMode(PIN_G1_DIR,OUTPUT);
  pinMode(PIN_G2_DIR,OUTPUT);
  pinMode(PIN_G3_DIR,OUTPUT);
  pinMode(PIN_G4_DIR,OUTPUT);
  pinMode(PIN_G5_DIR,OUTPUT);
  pinMode(PIN_G6_DIR,OUTPUT);
  pinMode(PIN_G7_DIR,OUTPUT);
  pinMode(PIN_G8_DIR,OUTPUT);
  pinMode(PIN_G9_DIR,OUTPUT);

  pinMode(PIN_G1_STEP,OUTPUT);
  pinMode(PIN_G2_STEP,OUTPUT);
  pinMode(PIN_G3_STEP,OUTPUT);
  pinMode(PIN_G4_STEP,OUTPUT);
  pinMode(PIN_G5_STEP,OUTPUT);
  pinMode(PIN_G6_STEP,OUTPUT);
  pinMode(PIN_G7_STEP,OUTPUT);
  pinMode(PIN_G8_STEP,OUTPUT);
  pinMode(PIN_G9_STEP,OUTPUT);

  pinMode(PIN_S1,INPUT_PULLUP);
  pinMode(PIN_S2,INPUT_PULLUP);
  pinMode(PIN_S3,INPUT_PULLUP);
  pinMode(PIN_S4,INPUT_PULLUP);
  pinMode(PIN_S5,INPUT_PULLUP);
  pinMode(PIN_S6,INPUT_PULLUP);
  pinMode(PIN_S7,INPUT_PULLUP);
  pinMode(PIN_S8,INPUT_PULLUP);
  pinMode(PIN_S9,INPUT_PULLUP);

  enableInterrupt(PIN_S1,isr_G1,FALLING);
  enableInterrupt(PIN_S2,isr_G2,FALLING);
  enableInterrupt(PIN_S3,isr_G3,FALLING);
  enableInterrupt(PIN_S4,isr_G4,FALLING);
  enableInterrupt(PIN_S5,isr_G5,FALLING);
  enableInterrupt(PIN_S6,isr_G6,FALLING);
  enableInterrupt(PIN_S7,isr_G7,FALLING);
  enableInterrupt(PIN_S8,isr_G8,FALLING);
  enableInterrupt(PIN_S9,isr_G9,FALLING);
}

void steppers_setup(){
  Stepper_G1.setMaxSpeed(500);
  Stepper_G1.setAcceleration(100);
  Stepper_G1.setSpeed(100);
  Stepper_G1.setCurrentPosition(0);

  Stepper_G2.setMaxSpeed(500);
  Stepper_G2.setAcceleration(100);
  Stepper_G2.setSpeed(100);
  Stepper_G2.setCurrentPosition(0);

  Stepper_G3.setMaxSpeed(500);
  Stepper_G3.setAcceleration(100);
  Stepper_G3.setSpeed(100);
  Stepper_G3.setCurrentPosition(0);

  Stepper_G4.setMaxSpeed(500);
  Stepper_G4.setAcceleration(100);
  Stepper_G4.setSpeed(100);
  Stepper_G4.setCurrentPosition(0);

  Stepper_G5.setMaxSpeed(500);
  Stepper_G5.setAcceleration(100);
  Stepper_G5.setSpeed(100);
  Stepper_G5.setCurrentPosition(0);

  Stepper_G6.setMaxSpeed(500);
  Stepper_G6.setAcceleration(100);
  Stepper_G6.setSpeed(100);
  Stepper_G6.setCurrentPosition(0);

  Stepper_G7.setMaxSpeed(500);
  Stepper_G7.setAcceleration(100);
  Stepper_G7.setSpeed(100);
  Stepper_G7.setCurrentPosition(0);

  Stepper_G8.setMaxSpeed(500);
  Stepper_G8.setAcceleration(100);
  Stepper_G8.setSpeed(100);
  Stepper_G8.setCurrentPosition(0);

  Stepper_G9.setMaxSpeed(500);
  Stepper_G9.setAcceleration(100);
  Stepper_G9.setSpeed(100);
  Stepper_G9.setCurrentPosition(0);
}

void steppers_init(){ //Get flushers and top motors to 0 first, then get elevators to 0, then get G5 to middle and elevators to top
    //Put flushers and top motors to 0
  flag_G2_step = flag_G4_step = flag_G5_step = flag_G6_step = flag_G7_step = true;
  Stepper_G2.moveTo(POS_INF); Stepper_G4.moveTo(POS_INF); Stepper_G5.moveTo(POS_INF); Stepper_G6.moveTo(POS_INF); Stepper_G7.moveTo(POS_INF);
  while(flag_G2_step || flag_G4_step || flag_G5_step || flag_G6_step || flag_G7_step){
    if(flag_G2_step)
      Stepper_G2.run();
    if(flag_G4_step)
      Stepper_G4.run();
    if(flag_G5_step)
      Stepper_G5.run();
    if(flag_G6_step)
      Stepper_G6.run();
    if(flag_G7_step)
      Stepper_G7.run();
  }
  Stepper_G2.setCurrentPosition(POS_TOP); Stepper_G4.setCurrentPosition(POS_TOP); Stepper_G5.setCurrentPosition(POS_TOP); Stepper_G6.setCurrentPosition(POS_TOP); Stepper_G7.setCurrentPosition(POS_TOP);
  //Put all elevator motors at 0
  flag_G1_step = flag_G3_step = flag_G8_step = flag_G9_step = true;
  Stepper_G1.moveTo(POS_INF); Stepper_G3.moveTo(POS_INF); Stepper_G8.moveTo(POS_INF); Stepper_G9.moveTo(POS_INF);
  while(flag_G1_step || flag_G3_step || flag_G8_step || flag_G9_step){
    if(flag_G1_step)
      Stepper_G1.run();
    if(flag_G3_step)
      Stepper_G3.run();
    if(flag_G8_step)
      Stepper_G8.run();
    if(flag_G9_step)
      Stepper_G9.run();
  }
  Stepper_G1.setCurrentPosition(POS_TOP); Stepper_G3.setCurrentPosition(POS_TOP); Stepper_G8.setCurrentPosition(POS_TOP); Stepper_G9.setCurrentPosition(POS_TOP);
  //Put elevator motors to top and top motor to middle
  flag_G1_step = flag_G3_step = flag_G5_step = flag_G8_step = flag_G9_step = true;
  Stepper_G1.moveTo(POS_G1389_BOT); Stepper_G3.moveTo(POS_G1389_BOT); Stepper_G5.moveTo(POS_G5_MID); Stepper_G8.moveTo(POS_G1389_BOT); Stepper_G9.moveTo(POS_G1389_BOT);
  while(flag_G1_step || flag_G3_step || flag_G5_step || flag_G8_step || flag_G9_step){
    if(flag_G1_step)
      Stepper_G1.run();
    if(flag_G3_step)
      Stepper_G3.run();
    if(flag_G5_step)
      Stepper_G5.run();
    if(flag_G8_step)
      Stepper_G8.run();
    if(flag_G9_step)
      Stepper_G9.run();
  }
  Stepper_G1.setCurrentPosition(POS_TOP); Stepper_G3.setCurrentPosition(POS_TOP); Stepper_G8.setCurrentPosition(POS_TOP); Stepper_G9.setCurrentPosition(POS_TOP);
  flag_G5_ready = true;
}

void steppers_run(){
  if(flag_G1_step){     
    Stepper_G1.run();
  }
  if(flag_G2_step){     
    Stepper_G2.run();
  }
  if(flag_G3_step){     
    Stepper_G3.run();
  }
  if(flag_G4_step){     
    Stepper_G4.run();
  }
  if(flag_G5_step){     
    Stepper_G5.run();
  }
  if(flag_G6_step){     
    Stepper_G6.run();
  }
  if(flag_G7_step){     
    Stepper_G7.run();
  }
  if(flag_G8_step){     
    Stepper_G8.run();
  }
  if(flag_G9_step){     
    Stepper_G9.run();
  }
}

void msg_interpret(int msg){
  if(!flag_G5_ready)
    return;
  if(flag_stack_left && msg == 6){
    flag_L6_active = true;
    flag_L6_flushing = false;
    curr_phase_L6 = phase_L6::phase_L6_1;
    flag_G5_step = true;
    Stepper_G5.moveTo(POS_G5_L6);
  }
  else if(flag_stack_left && msg == 9){
    flag_L9_active = true;
    flag_L9_flushing = false;
    curr_phase_L9 = phase_L9::phase_L9_1;
    flag_G5_step = true;
    Stepper_G5.moveTo(POS_G5_L9);
  }
  else if(!flag_stack_left && msg == 6){
    flag_R6_active = true;
    flag_R6_flushing = false;
    curr_phase_R6 = phase_R6::phase_R6_1;
    flag_G5_step = true;
    Stepper_G5.moveTo(POS_G5_R6);
  }
  else if(!flag_stack_left && msg == 9){
    flag_R9_active = true;
    flag_R9_flushing = false;
    curr_phase_R9 = phase_R9::phase_R9_1;
    flag_G5_step = true;
    Stepper_G5.moveTo(POS_G5_R9);
  }
}

void phase_transition()
{
  if(flag_L6_active){
    switch (curr_phase_L6)
    {
    case phase_L6::phase_L6_1:{
      if(Stepper_G5.distanceToGo() == 0){
        if(++platecount_L6 == 5){ //If the stack count is 5, flush it (put it on phase 2F)
          curr_phase_L6 = phase_L6::phase_L6_2F; 
          flag_L6_flushing = true;
          timer_L6 = millis();
          flag_G5_step = false;
          flag_G3_step = true;
          Stepper_G3.moveTo(POS_TOP);
        }
        else{ //If the stack count is not 5, move it one step down (put it on phase 2)
          curr_phase_L6 = phase_L6::phase_L6_2;
          flag_G5_step = false;
          flag_G3_step = true;
          Stepper_G3.moveTo(POS_G1389_BOT+platecount_L6);
        }
      } 
      break;
    }
    case phase_L6::phase_L6_2:{ //Move slider to the middle
      if(Stepper_G3.distanceToGo() == 0 ){
        curr_phase_L6 = phase_L6::phase_L6_3;
        flag_G3_step = false;
        flag_G5_step = true;
        Stepper_G5.moveTo(POS_G5_MID);
      }
      break;
    }
    case phase_L6::phase_L6_3:{
      if(Stepper_G5.distanceToGo() == 0){
        flag_L6_active = false;
      }
      break;
    }
    case phase_L6::phase_L6_2F:{
      if(millis() - timer_L6 >= 1000){
        
      }
      break;
    }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pins_setup();
  steppers_setup();
  flag_G5_step = true;
  Stepper_G5.moveTo(99999999);
}

void loop() {
  //Read byte from serial if available, replace with modbus when ready
  if (Serial.available()) {
    int msg = Serial.read();
    msg_interpret(msg);
  }
  steppers_run();
}
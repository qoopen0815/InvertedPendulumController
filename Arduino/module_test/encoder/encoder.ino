#include <Wire.h>
#include <RotaryEncoder.h>
#include <Arduino_FreeRTOS.h>
#include "TB6612FNG.h"

#define PIN_IN1 2
#define PIN_IN2 3

#define TESTPINA 5
#define TESTPINB 11

int deg;
bool dir;

// define two tasks
void TaskGetState( void *pvParameters );
void TaskControl( void *pvParameters );

// Setup a RotaryEncoder with 4 steps per latch for the 2 signal input pins:
// RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

// Setup a RotaryEncoder with 2 steps per latch for the 2 signal input pins:
RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);
utils::TB6612FNG motor;

double past;
double last_pos;
double pos;
double rpm;

double past_w;

void setup()
{
  Serial.begin(115200);
  while (!Serial){}

  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskGetState
    ,  "TaskGetState"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskControl
    ,  "TaskControl"
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );

  dir = false;
  past = (double)micros();
  past_w = 0.0;
  pos = 0;
  deg = 0;
  
  pinMode(TESTPINA, OUTPUT);
  pinMode(TESTPINB, OUTPUT);

  Wire.begin();
  motor.init();

  delay(100);
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskGetState(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    encoder.tick();
    pos = (double)encoder.getPosition() * 3.14/600.;
    digitalWrite(TESTPINA, !(digitalRead(TESTPINA)));
    delayMicroseconds(50);
    // vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskControl(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    motor.move(255);
    double delta = (double)micros() - past;
    past += delta;
    double speed = (pos - past_w)/(delta*0.000001);
    past_w += delta*0.000001*speed;
    Serial.print(delta); Serial.print("\t"); Serial.print(pos*180./3.14); Serial.print("\t"); Serial.println(speed);
    digitalWrite(TESTPINB, !(digitalRead(TESTPINB)));
    // delayMicroseconds(300);
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}
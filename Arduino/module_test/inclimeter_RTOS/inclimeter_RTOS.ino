#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include "inclinometer.h"

#define TESTPINA 5
#define TESTPINB 11

// define two tasks
void TaskUpdate( void *pvParameters );
void TaskView( void *pvParameters );

const double rate = 400.0;
const double dt = 1.0/rate;

Inclinometer inclinometer;

double theta[2] = {0.0, 0.0};  // 0: new, 1: old

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  inclinometer.begin(rate);

  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskUpdate
    ,  "TaskUpdate"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1    // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskView
    ,  "TaskView"
    ,  128  // Stack size
    ,  NULL
    ,  2    // Priority
    ,  NULL );
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskUpdate(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    inclinometer.updateTheta();
    digitalWrite(TESTPINA, !(digitalRead(TESTPINA)));
    // delay(dt*1000);
    vTaskDelay(2);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskView(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    // Serial.println(inclinometer.getFilteredTheta());
    Serial.println("Do");
    digitalWrite(TESTPINB, !(digitalRead(TESTPINB)));
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}
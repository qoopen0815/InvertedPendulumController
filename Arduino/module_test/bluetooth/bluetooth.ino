#include <SoftwareSerial.h>
SoftwareSerial SoftSerial(2, 3); // RX | TX pins.  Can be reassigned to other pins if needed
 
const long BAUDRATE = 9600;    // This is the default communication baud rate of the HC-05 module
char c = ' ';                  // Character being transmitted 
boolean NL = true;             // Newline.  True indicates we should show a '>'
//===============================================================================
//  Initialization
//===============================================================================
void setup() 
{
    SoftSerial.begin(BAUDRATE);  // Init soft serial object
    Serial.begin(9600);          // Init hardware serial
    Serial.println("Test started - Enter something to send to computer or Android phone");
    SoftSerial.println("Test started - Enter something to send to Serial Monitor Window");
}
//===============================================================================
//  Main
//=============================================================================== 
void loop()
{
     // Read from the Bluetooth module and send to the Arduino Serial Monitor
    if (SoftSerial.available())
    {
        c = SoftSerial.read();
        // Serial.write(c);
    }
     // Read from the Serial Monitor and send to the Bluetooth module
    if (Serial.available())
    {
        c = Serial.read();
        SoftSerial.write(c);

        // Echo the user input to the main window. The ">" character indicates the user entered text.
        if (NL) { Serial.print(">");  NL = false; }
        Serial.write(c);
        if (c==10) NL = true;    // char '10' is the newline character
    }

}
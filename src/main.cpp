// /* Sweep
//  by BARRAGAN <http://barraganstudio.com>
//  This example code is in the public domain.

//  modified 8 Nov 2013
//  by Scott Fitzgerald

//  modified for the ESP32 on March 2017
//  by John Bennett

//  see http://www.arduino.cc/en/Tutorial/Sweep for a description of the original code

//  * Different servos require different pulse widths to vary servo angle, but the range is
//  * an approximately 500-2500 microsecond pulse every 20ms (50Hz). In general, hobbyist servos
//  * sweep 180 degrees, so the lowest number in the published range for a particular servo
//  * represents an angle of 0 degrees, the middle of the range represents 90 degrees, and the top
//  * of the range represents 180 degrees. So for example, if the range is 1000us to 2000us,
//  * 1000us would equal an angle of 0, 1500us would equal 90 degrees, and 2000us would equal 1800
//  * degrees.
//  *
//  * Circuit: (using an ESP32 Thing from Sparkfun)
//  * Servo motors have three wires: power, ground, and signal. The power wire is typically red,
//  * the ground wire is typically black or brown, and the signal wire is typically yellow,
//  * orange or white. Since the ESP32 can supply limited current at only 3.3V, and servos draw
//  * considerable power, we will connect servo power to the VBat pin of the ESP32 (located
//  * near the USB connector). THIS IS ONLY APPROPRIATE FOR SMALL SERVOS.
//  *
//  * We could also connect servo power to a separate external
//  * power source (as long as we connect all of the grounds (ESP32, servo, and external power).
//  * In this example, we just connect ESP32 ground to servo ground. The servo signal pins
//  * connect to any available GPIO pins on the ESP32 (in this example, we use pin 18.
//  *
//  * In this example, we assume a Tower Pro MG995 large servo connected to an external power source.
//  * The published min and max for this servo is 1000 and 2000, respectively, so the defaults are fine.
//  * These values actually drive the servos a little past 0 and 180, so
//  * if you are particular, adjust the min and max values to match your needs.
//  */

// #include <ESP32Servo.h>

// Servo myservo; // create servo object to control a servo
// // 16 servo objects can be created on the ESP32

// int pos = 0; // variable to store the servo position
// // Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33
// // Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
// // Possible PWM GPIO pins on the ESP32-S3: 0(used by on-board button),1-21,35-45,47,48(used by on-board LED)
// // Possible PWM GPIO pins on the ESP32-C3: 0(used by on-board button),1-7,8(used by on-board LED),9-10,18-21
// #if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
// int servoPin = 17;
// #elif defined(CONFIG_IDF_TARGET_ESP32C3)
// int servoPin = 7;
// #else
// int servoPin = 18;
// #endif

// unsigned long previousMillis = 0;
// const int interval = 1000; // Interval in milliseconds

// void setup()
// {
//   // Allow allocation of all timers
//   ESP32PWM::allocateTimer(0);
//   ESP32PWM::allocateTimer(1);
//   ESP32PWM::allocateTimer(2);
//   ESP32PWM::allocateTimer(3);
//   myservo.setPeriodHertz(50);          // standard 50 hz servo
//   myservo.attach(servoPin, 700, 2500); // attaches the servo on pin 18 to the servo object
//                                        // using default min/max of 1000us and 2000us
//                                        // different servos may require different min/max settings
//                                        // for an accurate 0 to 180 sweep
// }

// void loop()
// {
//   // unsigned long currentMillis = millis();
//   // if (currentMillis - previousMillis >= interval)
//   // {
//   //   // Save the last time you updated the servo position
//   //   previousMillis = currentMillis;

//   //   // Your servo control code here
//   //   myservo.write(0); // Set servo to 0 degrees
//   // }
//   // myservo.write(20);
//   // delay(2000);
//   // myservo.write(90);
//   // delay(2000);

//   myservo.write(0);
//   delay(2000);
//   myservo.write(90);
//   delay(2000);
// }

/////// Up is the code for Servo

// ---------------------------------------------------------------------------
// Example NewPing library sketch that does a ping about 20 times per second.
// ---------------------------------------------------------------------------
#include <ESP32Servo.h>
#include <NewPing.h>

#define TRIGGER_DISTANCE 21 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_DISTANCE 19    // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400    // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define TRIGGER_CAPACITY 22 // For capacity sensor trigger pin
#define ECHO_CAPACITY 23    // For capcity sensor echo pin

#define SERVO_PIN 18 //
Servo myservo;
NewPing sonar[2] = {                                        // Sensor object array.
    NewPing(TRIGGER_DISTANCE, ECHO_DISTANCE, MAX_DISTANCE), // NewPing setup of pins and maximum distance.
    NewPing(TRIGGER_CAPACITY, ECHO_CAPACITY, MAX_DISTANCE)};

void setup()
{
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);            // standard 50 hz servo
  myservo.attach(SERVO_PIN, 1000, 2000); // attaches the servo on pin 18 to the servo object
                                         // using default min/max of 1000us and 2000us
                                         // different servos may require different min/max settings
                                         // for an accurate 0 to 180 sweep
  Serial.begin(115200);                  // Open serial monitor at 115200 baud to see ping results.
}
int timePased = 0;
int global_angle = 20;

//TODO - Try this in operation for servo.
/**
 * Performs operation (opens or closes) on lid
 *
 *
 * @param desired_angle Final angle to achieve.
 * @return `void`
 */
void lid_operate(int desired_angle)
{
  for (int current_angle = global_angle; current_angle != desired_angle;)
  {
    // Updates the global angle
    // Updates the i variable

    // Finds the difference in angle
    float difference = (desired_angle - global_angle) * 0.1;
    if (difference > 0)
    {
      current_angle = current_angle + 20;
      Serial.println("current angle: " + String(current_angle));
      myservo.write(current_angle);
      global_angle = current_angle;
    }
    else
    {
      // Decrement is quite high because the hardware is not calibrated properly.
      current_angle = current_angle - 20;
      Serial.println("current angle: " + String(current_angle));
      myservo.write(current_angle);
      global_angle = current_angle;
    }

    delay(100);

    // Apply the difference to step
  }
}

unsigned long previousMillis = 0UL;
unsigned long interval = 5000UL;

void loop()
{

  // delay(50);
  // Serial.println(sonar[0].ping_cm());
  // Serial.println(sonar[1].ping_cm());

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > interval)
  {
    /* The ESP32 executes this code once every second
     *  (interval = 1000 (ms) = 1 second).
     */

    // Code for Blynk things.

    // Don't forget to update the previousMillis value
    previousMillis = currentMillis;
  }

  delay(50);

  int data = sonar[1].ping_cm();
  Serial.println(data);
  if (data < 30 && data > 0)
  {
    lid_operate(180);
    // myservo.write(180);
    delay(1500);
  }
  else
  {
    lid_operate(20);
    // myservo.write(20);
    delay(200);
  }
}
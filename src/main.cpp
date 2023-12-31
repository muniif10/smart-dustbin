
#include <ESP32Servo.h>
#include <NewPing.h>
#include <tokens.h>
#include <BlynkSimpleEsp32.h>

#define TRIGGER_DISTANCE 22 // Arduino pin tied to trigger pin on the ultrasonic sensor. 21

#define ECHO_DISTANCE 23 // Arduino pin tied to echo pin on the ultrasonic sensor. 19
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define TRIGGER_CAPACITY 21 // For capacity sensor trigger pin 22
#define ECHO_CAPACITY 19    // For capcity sensor echo pin 23

#define SERVO_PIN 18 //
Servo myservo;
NewPing sonar[2] = {                                        // Sensor object array.
    NewPing(TRIGGER_DISTANCE, ECHO_DISTANCE, MAX_DISTANCE), // NewPing setup of pins and maximum distance.
    NewPing(TRIGGER_CAPACITY, ECHO_CAPACITY, MAX_DISTANCE)};


/**
 * Converter from distance to percentage of the max height given.
 * Made for the Blynk percentage handling.
 *
 * @param data data to go through conversion.
 * @param container_height height of the container.
 * @return double
 */
double
converter(int data, int container_height)
{
  double converted = fabs(data * -(1.0 / container_height));
  return converted;
}

// Handles the data stream update to blynk
void updateDatastreamV0()
{
  int data_2 = (int)sonar[0].ping_cm();
  double percentage = converter(data_2, 30);
  Serial.println();
  Blynk.virtualWrite(V0, percentage);
}
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

  Blynk.begin(BLYNK_AUTH_TOKEN, "Muniif", "gawdemson");
}
int timePased = 0;
int global_angle = 20;

// TODO - Try this in operation for servo.
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

    
  }
}

unsigned long previousMillis = 0UL;
unsigned long interval = 10000UL;

void loop()
{

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > interval)
  {
    updateDatastreamV0();
    previousMillis = currentMillis;
  }
  unsigned int data = sonar[1].ping_cm();

  if (data < 30 && data > 0)
  {
    // lid_operate(180);
    myservo.write(180);
    delay(1500);
  }
  else
  {
    // lid_operate(20);
    myservo.write(20);
    delay(20);
  }
}

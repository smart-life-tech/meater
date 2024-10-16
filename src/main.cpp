#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_MCP23X08.h>
#include <Adafruit_MCP23X17.h>
#include "max6675.h"
#include <Nextion.h>

// Pin definitions for MCP23017
#define MCP23017_ADDR 0x20 // Address of MCP23017
#define NUM_MAX6675 5      // Number of MAX6675 modules

Adafruit_MCP23X17 mcp;
MAX6675 thermocouples[NUM_MAX6675]; // Array to hold MAX6675 instances

// Pin mappings for each MAX6675 module
const uint8_t THERMO_CS_PINS[NUM_MAX6675] = {0, 3, 6, 9, 12};
const uint8_t THERMO_DO_PINS[NUM_MAX6675] = {1, 4, 7, 10, 13};
const uint8_t THERMO_CLK_PINS[NUM_MAX6675] = {2, 5, 8, 11, 14};

// Fan control pin
const uint8_t FAN_PIN = 26; // Use a PWM-capable pin on ESP32

// Door switches
const uint8_t FIREBOX_DOOR_PIN = 27; // Pin connected to firebox door switch
const uint8_t OVEN_DOOR_PIN = 28;    // Pin connected to oven door switch

// PID constants
const double Kp = 1.0;
const double Ki = 0.1;
const double Kd = 0.1;

bool doorOpen = false;

class PIDController
{
public:
  PIDController(double kp, double ki, double kd) : Kp(kp), Ki(ki), Kd(kd) {}

  void setSetpoint(double setpoint)
  {
    this->setpoint = setpoint;
  }

  double compute(double input)
  {
    double error = setpoint - input;
    integral += error;
    double derivative = error - prevError;
    prevError = error;

    double output = Kp * error + Ki * integral + Kd * derivative;
    return output;
  }

private:
  double Kp, Ki, Kd;
  double setpoint = 0;
  double integral = 0;
  double prevError = 0;
};

PIDController pid(Kp, Ki, Kd);

NexText doorStatus = NexText(0, 1, "t1");

void rampFanUp()
{
  for (int i = 0; i <= 255; i++)
  {
    ledcWrite(0, i); // Stop the fan
    delay(20);       // Adjust ramp up speed here
  }
}
void setup()
{
  Serial.begin(9600);

  nexInit();

  // Initialize MCP23017
  // uncomment appropriate mcp.begin
  if (!mcp.begin_I2C())
  {
    // if (!mcp.begin_SPI(CS_PIN)) {
    Serial.println("Error.");
    while (1)
      ;
  }

  // Configure pins for MAX6675
  for (int i = 0; i < NUM_MAX6675; i++)
  {
    mcp.pinMode(THERMO_CS_PINS[i], OUTPUT);
    mcp.pinMode(THERMO_DO_PINS[i], INPUT);
    mcp.pinMode(THERMO_CLK_PINS[i], OUTPUT);
  }

  // Set initial states
  for (int i = 0; i < NUM_MAX6675; i++)
  {
    mcp.digitalWrite(THERMO_CS_PINS[i], HIGH); // Deselect MAX6675 initially
  }

  // Initialize MAX6675 instances
  for (int i = 0; i < NUM_MAX6675; i++)
  {
    thermocouples[i].begin(THERMO_CLK_PINS[i], THERMO_CS_PINS[i], THERMO_DO_PINS[i]);
  }

  // Set fan pin as output
  pinMode(FAN_PIN, OUTPUT);
  // Setup PWM for controlling the PWM-to-voltage module
  ledcSetup(0, 1000, 8);     // Use LEDC channel 0, 1 kHz frequency, 8-bit resolution
  ledcAttachPin(FAN_PIN, 0); // Attach PWM_PIN to LEDC channel 0
  // Set door switch pins as inputs with pullups enabled
  pinMode(FIREBOX_DOOR_PIN, INPUT_PULLUP);
  pinMode(OVEN_DOOR_PIN, INPUT_PULLUP);

  Serial.println("PID Fan Control");
  // Wait for MAX chip to stabilize
  delay(500);
}

void loop()
{
  // Check door status
  bool fireboxDoorOpen = digitalRead(FIREBOX_DOOR_PIN) == LOW; // LOW indicates door open
  bool ovenDoorOpen = digitalRead(OVEN_DOOR_PIN) == LOW;       // LOW indicates door open

  // If either door is open, pause fan/controller
  if (fireboxDoorOpen || ovenDoorOpen)
  {
    doorOpen = true;
    ledcWrite(0, 0);                 // Stop the fan        // Stop the fan
    doorStatus.setText("Door Open"); // Display door open message on Nextion display
    delay(1000);                     // Delay to prevent flickering
    return;                          // Exit loop iteration
  }
  else
  {
    // If both doors are closed and the fan/controller was paused, resume operation
    if (doorOpen)
    {
      doorOpen = false;
      rampFanUp(); // Ramp fan speed up slowly
    }
  }

  // Read temperatures from all thermocouples
  double temperatures[NUM_MAX6675];
  for (int i = 0; i < NUM_MAX6675; i++)
  {
    temperatures[i] = thermocouples[i].getTemperature();
    Serial.print("Temperature ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(temperatures[i]);
  }

  // Compute average temperature
  double avgTemperature = 0;
  for (int i = 0; i < NUM_MAX6675; i++)
  {
    avgTemperature += temperatures[i];
  }
  avgTemperature /= NUM_MAX6675;

  // Setpoint temperature
  double setpoint = 40.0; // Example setpoint

  // Compute PID output
  double output = pid.compute(avgTemperature);

  // Map PID output to PWM duty cycle (0-255)
  int pwmValue = map(output, 0, 10, 0, 255);

  // Convert PWM duty cycle to frequency within 0-1 kHz range
  int frequency = map(pwmValue, 0, 255, 0, 1000);

  // Control fan based on PWM frequency
  // analogWriteFreq(frequency); // Set PWM frequency
  ledcWrite(0, pwmValue); // Stop the fan

  // Debug output
  Serial.print("Average Temperature: ");
  Serial.print(avgTemperature);
  Serial.print(" Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" Output: ");
  Serial.println(output);

  // Delay between PID calculations
  delay(1000);
}

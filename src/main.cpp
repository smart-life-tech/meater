#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_MCP23X08.h>
#include <Adafruit_MCP23X17.h>
#include "max6675.h"
#include <Nextion.h>

// Nextion Page and Component Declarations (add these for new pages)
NexPage homePage(0, 0, "page0");     // Home/Dashboard
NexPage settingsPage(1, 0, "page1"); // Settings
NexPage monitorPage(2, 0, "page2");  // Monitoring
NexPage controlPage(3, 0, "page3");  // Control
NexPage alertsPage(4, 0, "page4");   // Alerts

// Home Page Components
NexText temp1(0, 1, "t1"); // Individual temps
NexText temp2(0, 2, "t2");
NexText temp3(0, 3, "t3");
NexText temp4(0, 4, "t4");
NexText temp5(0, 5, "t5");
NexText avgTemp(0, 6, "t6");         // Average temp
NexText setpointDisplay(0, 7, "t7"); // Setpoint
NexProgressBar fanBar(0, 8, "j0");   // Fan speed bar
NexText fanText(0, 9, "t8");         // Fan speed text
NexText doorText(0, 10, "t9");       // Door status
NexButton settingsBtn(0, 11, "b0");  // To Settings
NexButton monitorBtn(0, 12, "b1");   // To Monitor

// Settings Page Components
NexText settingsTitle(1, 1, "t0");
NexNumber setpointSlider(1, 2, "h0"); // Setpoint slider
NexText setpointLabel(1, 3, "t1");
NexNumber kpSlider(1, 4, "h1"); // PID sliders
NexText kpLabel(1, 5, "t2");
NexNumber kiSlider(1, 6, "h2");
NexText kiLabel(1, 7, "t3");
NexNumber kdSlider(1, 8, "h3");
NexText kdLabel(1, 9, "t4");
NexButton saveBtn(1, 10, "b0");  // Save
NexButton resetBtn(1, 11, "b1"); // Reset

// Monitoring Page Components
NexText monitorTitle(2, 1, "t0");
NexText logArea(2, 2, "ta0");     // Log text area
NexPicture graphPic(2, 3, "p0");  // Graph placeholder
NexButton refreshBtn(2, 4, "b0"); // Refresh
NexButton backBtn(2, 5, "b1");    // Back to Home

// Control Page Components
NexText controlTitle(3, 1, "t0");
NexNumber manualFanSlider(3, 2, "h0"); // Manual fan
NexText manualFanLabel(3, 3, "t1");
NexButton stopBtn(3, 4, "b0");     // Emergency stop
NexButton resumeBtn(3, 5, "b1");   // Resume
NexButton resetPidBtn(3, 6, "b2"); // Reset PID
NexText modeText(3, 7, "t2");      // Mode status

// Alerts Page Components
NexText alertsTitle(4, 1, "t0");
NexText alert1(4, 2, "t1"); // Alert messages
NexText alert2(4, 3, "t2");
NexText alert3(4, 4, "t3");
NexButton ackBtn(4, 5, "b0"); // Acknowledge

// Event Handlers (declare these for button presses)
void settingsBtnCallback(void *ptr);
void monitorBtnCallback(void *ptr);
void saveBtnCallback(void *ptr);
void resetBtnCallback(void *ptr);
void refreshBtnCallback(void *ptr);
void backBtnCallback(void *ptr);
void stopBtnCallback(void *ptr);
void resumeBtnCallback(void *ptr);
void resetPidBtnCallback(void *ptr);
void ackBtnCallback(void *ptr);

// Pin definitions for MCP23017
#define MCP23017_ADDR 0x20 // Address of MCP23017 to read the temperature from the ther mocouples
#define NUM_MAX6675 5      // Number of MAX6675 modules

Adafruit_MCP23X17 mcp;
MAX6675 thermocouples[NUM_MAX6675]; // Array to hold MAX6675 instances

// Pin mappings for each MAX6675 module
const uint8_t THERMO_CS_PINS[NUM_MAX6675] = {0, 3, 6, 9, 12}; // Pin connected to CS pin of MAX6675
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
// PID controller
/**
 * @class PIDController
 * @brief Implements a PID controller for process control.
 *
 * This class provides methods to set a desired setpoint and compute the control
 * output based on the current input using proportional, integral, and derivative
 * gains.
 *
 * @param Kp Proportional gain.
 * @param Ki Integral gain.
 * @param Kd Derivative gain.
 *
 * @method setSetpoint Sets the desired setpoint for the PID controller.
 * @param setpoint The target value the controller aims to achieve.
 *
 * @method compute Calculates the control output based on the current input.
 * @param input The current process variable.
 * @return The computed control output.
 */

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

// Global variables for new features
double setpoint = 40.0;  // Adjustable setpoint
bool manualMode = false; // For manual fan control
int manualFanSpeed = 0;  // Manual fan PWM value
String tempLog = "";     // For monitoring log

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
  // ledcSetup(0, 1000, 8);     // Use LEDC channel 0, 1 kHz frequency, 8-bit resolutio
  ledcAttach(FAN_PIN, 1000, 8); // Attach PWM_PIN to LEDC channel 0
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
    doorText.setText("Door Opened"); // Display door open message on Nextion display
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

  // Compute and average temperature
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

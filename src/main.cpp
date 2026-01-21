#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Nextion.h>

// ========================================
// LTC2983 Chip Configuration
// ========================================
#define LTC2983_CS_PIN 5        // Chip Select pin for LTC2983
#define LTC2983_INTERRUPT_PIN 4 // Interrupt pin (optional, can poll status instead)

// LTC2983 Register Addresses
#define LTC2983_COMMAND_STATUS_REG 0x0000
#define LTC2983_CHANNEL_ASSIGN_START 0x0200
#define LTC2983_CONVERSION_RESULT 0x0010
#define LTC2983_GLOBAL_CONFIG 0x00F0

// LTC2983 Commands
#define LTC2983_CMD_SLEEP 0x97
#define LTC2983_CMD_CONVERT_ALL 0x80
#define LTC2983_CMD_CONVERT_CH 0x81

// Thermocouple Configuration
#define NUM_THERMOCOUPLES 5

// Thermocouple Type (K-Type = 0x01, J-Type = 0x02, etc.)
#define THERMOCOUPLE_TYPE_K 0x01

// Channel assignments (using channels 2, 4, 6, 8, 10 for thermocouples)
// Channel 1, 3, 5, 7, 9 will be used as sense resistors/cold junction sensors
const uint8_t thermocoupleChannels[NUM_THERMOCOUPLES] = {2, 4, 6, 8, 10};

// Global variables
float temperatures[NUM_THERMOCOUPLES];
bool conversionComplete = false;

// Nextion Page and Component Declarations
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

// Event Handlers
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

// Fan control pin
const uint8_t FAN_PIN = 26; // Use a PWM-capable pin on ESP32

// Door switches
const uint8_t FIREBOX_DOOR_PIN = 27; // Pin connected to firebox door switch
const uint8_t OVEN_DOOR_PIN = 28;    // Pin connected to oven door switch

// PID constants
double Kp = 1.0;
double Ki = 0.1;
double Kd = 0.1;

// Nextion touch event list
NexTouch *nex_listen_list[] = {
    &settingsBtn,
    &monitorBtn,
    &saveBtn,
    &resetBtn,
    &refreshBtn,
    &backBtn,
    &stopBtn,
    &resumeBtn,
    &resetPidBtn,
    &ackBtn,
    NULL};

// ========================================
// LTC2983 SPI Communication Functions
// ========================================

void ltc2983_write(uint16_t address, uint8_t *data, uint8_t length)
{
  digitalWrite(LTC2983_CS_PIN, LOW);
  SPI.transfer(0x02); // Write command
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  for (uint8_t i = 0; i < length; i++)
  {
    SPI.transfer(data[i]);
  }
  digitalWrite(LTC2983_CS_PIN, HIGH);
}

void ltc2983_read(uint16_t address, uint8_t *data, uint8_t length)
{
  digitalWrite(LTC2983_CS_PIN, LOW);
  SPI.transfer(0x03); // Read command
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  for (uint8_t i = 0; i < length; i++)
  {
    data[i] = SPI.transfer(0x00);
  }
  digitalWrite(LTC2983_CS_PIN, HIGH);
}

uint8_t ltc2983_read_byte(uint16_t address)
{
  uint8_t data;
  ltc2983_read(address, &data, 1);
  return data;
}

void ltc2983_write_byte(uint16_t address, uint8_t data)
{
  ltc2983_write(address, &data, 1);
}

// ========================================
// LTC2983 Configuration Functions
// ========================================

void ltc2983_configure_thermocouple(uint8_t channel, uint8_t thermocoupleType)
{
  // Calculate channel assignment register address
  uint16_t channelAddress = LTC2983_CHANNEL_ASSIGN_START + ((channel - 1) * 4);

  // Configure thermocouple channel
  // Byte 0: Sensor type (0x01 = K-Type thermocouple)
  // Byte 1: Configuration bits (cold junction, single-ended)
  // Byte 2-3: Custom sensor configuration (if needed)
  uint8_t config[4];
  config[0] = thermocoupleType; // K-Type thermocouple
  config[1] = 0x01;             // Single-ended, use CH1 as cold junction
  config[2] = 0x00;
  config[3] = 0x00;

  ltc2983_write(channelAddress, config, 4);
}

void ltc2983_configure_cold_junction()
{
  // Configure channel 1 as a sense resistor for cold junction compensation
  uint16_t channelAddress = LTC2983_CHANNEL_ASSIGN_START;

  uint8_t config[4];
  config[0] = 0x1A; // Sense resistor type
  config[1] = 0x00;
  config[2] = 0x00;
  config[3] = 0x00;

  ltc2983_write(channelAddress, config, 4);
}

void ltc2983_init()
{
  // Set up SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16); // 1MHz for LTC2983

  pinMode(LTC2983_CS_PIN, OUTPUT);
  digitalWrite(LTC2983_CS_PIN, HIGH);

  delay(100); // Wait for chip to stabilize

  // Configure cold junction (Channel 1)
  ltc2983_configure_cold_junction();

  // Configure 5 thermocouple channels
  for (int i = 0; i < NUM_THERMOCOUPLES; i++)
  {
    ltc2983_configure_thermocouple(thermocoupleChannels[i], THERMOCOUPLE_TYPE_K);
  }

  Serial.println("LTC2983 initialized with 5 K-Type thermocouples");
}

// ========================================
// Temperature Reading Functions
// ========================================

void ltc2983_start_conversion()
{
  ltc2983_write_byte(LTC2983_COMMAND_STATUS_REG, LTC2983_CMD_CONVERT_ALL);
  conversionComplete = false;
}

bool ltc2983_check_conversion_complete()
{
  uint8_t status = ltc2983_read_byte(LTC2983_COMMAND_STATUS_REG);
  // Bit 6 indicates conversion complete
  return (status & 0x40) != 0;
}

float ltc2983_read_temperature(uint8_t channel)
{
  // Calculate result register address for the channel
  uint16_t resultAddress = LTC2983_CONVERSION_RESULT + ((channel - 1) * 4);

  uint8_t result[4];
  ltc2983_read(resultAddress, result, 4);

  // Combine bytes into 32-bit result
  int32_t rawValue = ((int32_t)result[0] << 24) |
                     ((int32_t)result[1] << 16) |
                     ((int32_t)result[2] << 8) |
                     result[3];

  // Check for fault condition (bit 24 set)
  if (rawValue & 0x01000000)
  {
    Serial.print("Fault detected on channel ");
    Serial.println(channel);
    return NAN;
  }

  // Convert to temperature (resolution is 1024 counts per degree C)
  float temperature = (float)rawValue / 1024.0;

  return temperature;
}

void ltc2983_read_all_temperatures()
{
  // Start conversion
  ltc2983_start_conversion();

  // Wait for conversion to complete (typically 150-200ms per channel)
  delay(1000);

  while (!ltc2983_check_conversion_complete())
  {
    delay(10);
  }

  // Read all temperatures
  for (int i = 0; i < NUM_THERMOCOUPLES; i++)
  {
    temperatures[i] = ltc2983_read_temperature(thermocoupleChannels[i]);

    Serial.print("Thermocouple ");
    Serial.print(i + 1);
    Serial.print(": ");
    if (isnan(temperatures[i]))
    {
      Serial.println("ERROR/FAULT");
    }
    else
    {
      Serial.print(temperatures[i]);
      Serial.println(" °C");
    }
  }
}

// ========================================
// PID Controller Class
// ========================================

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

  void reset()
  {
    integral = 0;
    prevError = 0;
  }

private:
  double Kp, Ki, Kd;
  double setpoint = 0;
  double integral = 0;
  double prevError = 0;
};

PIDController pid(Kp, Ki, Kd);

// ========================================
// Global Variables
// ========================================

bool doorOpen = false;
double setpoint = 40.0;  // Adjustable setpoint
bool manualMode = false; // For manual fan control
int manualFanSpeed = 0;  // Manual fan PWM value
String tempLog = "";     // For monitoring log

// ========================================
// Utility Functions
// ========================================

void rampFanUp()
{
  for (int i = 0; i <= 255; i++)
  {
    ledcWrite(0, i);
    delay(20);
  }
}

void updateNextionDisplay()
{
  // Update individual temperature displays
  char tempStr[10];

  if (!isnan(temperatures[0]))
  {
    sprintf(tempStr, "%.1f", temperatures[0]);
    temp1.setText(tempStr);
  }
  else
  {
    temp1.setText("ERR");
  }

  if (!isnan(temperatures[1]))
  {
    sprintf(tempStr, "%.1f", temperatures[1]);
    temp2.setText(tempStr);
  }
  else
  {
    temp2.setText("ERR");
  }

  if (!isnan(temperatures[2]))
  {
    sprintf(tempStr, "%.1f", temperatures[2]);
    temp3.setText(tempStr);
  }
  else
  {
    temp3.setText("ERR");
  }

  if (!isnan(temperatures[3]))
  {
    sprintf(tempStr, "%.1f", temperatures[3]);
    temp4.setText(tempStr);
  }
  else
  {
    temp4.setText("ERR");
  }

  if (!isnan(temperatures[4]))
  {
    sprintf(tempStr, "%.1f", temperatures[4]);
    temp5.setText(tempStr);
  }
  else
  {
    temp5.setText("ERR");
  }

  // Calculate and display average temperature
  float avgTempValue = 0;
  int validCount = 0;
  for (int i = 0; i < NUM_THERMOCOUPLES; i++)
  {
    if (!isnan(temperatures[i]))
    {
      avgTempValue += temperatures[i];
      validCount++;
    }
  }

  if (validCount > 0)
  {
    avgTempValue /= validCount;
    sprintf(tempStr, "%.1f", avgTempValue);
    avgTemp.setText(tempStr);
  }
  else
  {
    avgTemp.setText("ERR");
  }

  // Update setpoint display
  sprintf(tempStr, "%.1f", setpoint);
  setpointDisplay.setText(tempStr);
}

// ========================================
// Event Handler Callbacks
// ========================================

// Event callback implementations
void settingsBtnCallback(void *ptr)
{
  settingsPage.show();
}

void monitorBtnCallback(void *ptr)
{
  monitorPage.show();
}

void saveBtnCallback(void *ptr)
{
  // Save PID settings (implement EEPROM storage if needed)
  Serial.println("Settings saved");
}

void resetBtnCallback(void *ptr)
{
  // Reset PID controller
  pid.reset();
  Serial.println("PID reset");
}

void refreshBtnCallback(void *ptr)
{
  // Refresh monitoring data
  Serial.println("Data refreshed");
}

void backBtnCallback(void *ptr)
{
  homePage.show();
}

void stopBtnCallback(void *ptr)
{
  manualMode = true;
  ledcWrite(0, 0);
  Serial.println("Emergency stop");
}

void resumeBtnCallback(void *ptr)
{
  manualMode = false;
  Serial.println("Resumed automatic control");
}

void resetPidBtnCallback(void *ptr)
{
  pid.reset();
  Serial.println("PID reset from control page");
}

void ackBtnCallback(void *ptr)
{
  Serial.println("Alerts acknowledged");
}

// ========================================
// Setup Function
// ========================================

void setup()
{
  Serial.begin(115200);
  Serial.println("LTC2983 Thermocouple Reader");
  Serial.println("============================");

  // Initialize Nextion display
  nexInit();

  // Attach event handlers to buttons
  settingsBtn.attachPop(settingsBtnCallback);
  monitorBtn.attachPop(monitorBtnCallback);
  saveBtn.attachPop(saveBtnCallback);
  resetBtn.attachPop(resetBtnCallback);
  refreshBtn.attachPop(refreshBtnCallback);
  backBtn.attachPop(backBtnCallback);
  stopBtn.attachPop(stopBtnCallback);
  resumeBtn.attachPop(resumeBtnCallback);
  resetPidBtn.attachPop(resetPidBtnCallback);
  ackBtn.attachPop(ackBtnCallback);

  // Initialize LTC2983
  ltc2983_init();

  // Set fan pin as output and configure PWM
  pinMode(FAN_PIN, OUTPUT);
  ledcSetup(0, 1000, 8);        // Configure LEDC channel 0, 1 kHz frequency, 8-bit resolution
  ledcAttachPin(FAN_PIN, 0);    // Attach FAN_PIN to LEDC channel 0

  // Set door switch pins as inputs with pullups enabled
  pinMode(FIREBOX_DOOR_PIN, INPUT_PULLUP);
  pinMode(OVEN_DOOR_PIN, INPUT_PULLUP);

  // Set PID setpoint
  pid.setSetpoint(setpoint);

  Serial.println("Setup complete!");
  Serial.println();

  delay(500);
}

// ========================================
// Main Loop
// ========================================

void loop()
{
  // Process Nextion events
  nexLoop(nex_listen_list);

  // Check door status
  bool fireboxDoorOpen = digitalRead(FIREBOX_DOOR_PIN) == LOW;
  bool ovenDoorOpen = digitalRead(OVEN_DOOR_PIN) == LOW;

  // If either door is open, pause fan/controller
  if (fireboxDoorOpen || ovenDoorOpen)
  {
    if (!doorOpen)
    {
      doorOpen = true;
      ledcWrite(0, 0); // Stop the fan
      doorText.setText("Door Open");
      Serial.println("Door opened - fan stopped");
    }
    delay(1000);
    return;
  }
  else
  {
    // If both doors are closed and the fan was paused, resume operation
    if (doorOpen)
    {
      doorOpen = false;
      doorText.setText("Door Closed");
      rampFanUp(); // Ramp fan speed up slowly
      Serial.println("Door closed - resuming operation");
    }
  }

  // Read all thermocouple temperatures
  ltc2983_read_all_temperatures();

  // Update Nextion display with current temperatures
  updateNextionDisplay();

  // Calculate average temperature from valid readings
  float avgTemperature = 0;
  int validCount = 0;
  for (int i = 0; i < NUM_THERMOCOUPLES; i++)
  {
    if (!isnan(temperatures[i]))
    {
      avgTemperature += temperatures[i];
      validCount++;
    }
  }

  if (validCount > 0)
  {
    avgTemperature /= validCount;

    Serial.print("Average Temperature: ");
    Serial.print(avgTemperature);
    Serial.println(" °C");

    // Compute PID output if not in manual mode
    if (!manualMode)
    {
      double pidOutput = pid.compute(avgTemperature);

      // Map PID output to PWM duty cycle (0-255)
      // Adjust the mapping based on your system requirements
      int pwmValue = constrain((int)pidOutput, 0, 255);

      ledcWrite(0, pwmValue);

      Serial.print("PID Output: ");
      Serial.print(pidOutput);
      Serial.print(" -> PWM: ");
      Serial.println(pwmValue);

      // Update fan display
      fanBar.setValue(map(pwmValue, 0, 255, 0, 100));
      char fanStr[10];
      sprintf(fanStr, "%d%%", map(pwmValue, 0, 255, 0, 100));
      fanText.setText(fanStr);
    }
    else
    {
      // Manual mode - use manual fan speed
      ledcWrite(0, manualFanSpeed);
      Serial.print("Manual Mode - Fan Speed: ");
      Serial.println(manualFanSpeed);
    }
  }
  else
  {
    Serial.println("ERROR: No valid temperature readings!");
    ledcWrite(0, 0); // Stop fan if no valid readings
  }

  Serial.println("---");
  delay(2000); // Read every 2 seconds
}

#include "Arduino.h"
#include <Wire.h>
#include <SimpleFOC.h>
#include <Math.h>
#include "encoders/as5047/MagneticSensorAS5047.h"
#include "drivers/drv8316/drv8316.h"

void printDRV8316Status();
void printSensorStatus();

#define SENSOR_nCS 30
#define DRIVER_nCS 31
#define DRIVER_UH 5
#define DRIVER_VH 6
#define DRIVER_WH 9
#define DRIVER_OFF 11
#define nFAULT SDA
#define LED 13

MagneticSensorSPIConfig_s AS5047 = {
    .spi_mode = SPI_MODE1,
    .clock_speed = 1000000,
    .bit_resolution = 14,
    .angle_register = 0x3FFF,
    .data_start_bit = 13,
    .command_rw_bit = 14,
    .command_parity_bit = 15};

MagneticSensorAS5047 sensor(SENSOR_nCS, false, SPISettings(4000000, BitOrder::MSBFIRST, SPI_MODE1));
BLDCMotor motor = BLDCMotor(11);
DRV8316Driver3PWM driver = DRV8316Driver3PWM(DRIVER_UH, DRIVER_VH, DRIVER_WH, DRIVER_nCS, false);

Commander command = Commander(Serial);
void onMotor(char *cmd) { command.motor(&motor, cmd); }

void setup()
{
  pinMode(nFAULT, INPUT);
  pinMode(LED, OUTPUT);

  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();

  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.01;
  motor.P_angle.P = 20;

  motor.voltage_limit = 3;
  // motor.velocity_limit = 5;

  Serial.begin(115200);
  motor.useMonitoring(Serial);

  while (!Serial)
    ;

  motor.init();
  motor.initFOC(0.77, CW);

  command.add('M', onMotor, "Motor control");

  Serial.println(F("Motor ready."));
  printDRV8316Status();
  printSensorStatus();

  _delay(1000);
}

void loop()
{
  // digitalWrite(LED, digitalRead(nFAULT));
  motor.loopFOC();
  motor.move(motor.target);
  // motor.monitor();
  command.run();
}

void printSensorStatus()
{
  AS5047Diagnostics diag = sensor.readDiagnostics();
  Serial.println("AS5047 Status:");
  Serial.print("Error: ");
  Serial.println(sensor.isErrorFlag());
  // Serial.print("Level High: ");
  // Serial.println(diag.compHigh);
  // Serial.print("Level Low: ");
  // Serial.println(diag.compLow);
  // Serial.print("AGC: ");
  // Serial.println(diag.agc);
  // Serial.print("OCF: ");
  // Serial.println(diag.ocf);
  Serial.print("COF: ");
  Serial.println(diag.cof);
  float currentAngle = sensor.getCurrentAngle();
  Serial.print("Angle: ");
  Serial.println(currentAngle);
  uint16_t magnitude = sensor.readMagnitude();
  Serial.print("Magnitude: ");
  Serial.println(magnitude);
  Serial.print("Error: ");
  Serial.println(sensor.isErrorFlag());
  if (sensor.isErrorFlag())
  {
    AS5047Error err = sensor.clearErrorFlag();
    Serial.print("Command invalid: ");
    Serial.println(err.commandInvalid);
    Serial.print("Framing error: ");
    Serial.println(err.framingError);
    Serial.print("Parity error: ");
    Serial.println(err.parityError);
  }
  Serial.println();
}

void printDRV8316Status()
{
  DRV8316Status status = driver.getStatus();
  Serial.println("DRV8316 Status:");
  Serial.print("Fault: ");
  Serial.println(status.isFault());
  Serial.print("Buck Error: ");
  Serial.print(status.isBuckError());
  Serial.print("  Undervoltage: ");
  Serial.print(status.isBuckUnderVoltage());
  Serial.print("  OverCurrent: ");
  Serial.println(status.isBuckOverCurrent());
  Serial.print("Charge Pump UnderVoltage: ");
  Serial.println(status.isChargePumpUnderVoltage());
  Serial.print("OTP Error: ");
  Serial.println(status.isOneTimeProgrammingError());
  Serial.print("OverCurrent: ");
  Serial.print(status.isOverCurrent());
  Serial.print("  Ah: ");
  Serial.print(status.isOverCurrent_Ah());
  Serial.print("  Al: ");
  Serial.print(status.isOverCurrent_Al());
  Serial.print("  Bh: ");
  Serial.print(status.isOverCurrent_Bh());
  Serial.print("  Bl: ");
  Serial.print(status.isOverCurrent_Bl());
  Serial.print("  Ch: ");
  Serial.print(status.isOverCurrent_Ch());
  Serial.print("  Cl: ");
  Serial.println(status.isOverCurrent_Cl());
  Serial.print("OverTemperature: ");
  Serial.print(status.isOverTemperature());
  Serial.print("  Shutdown: ");
  Serial.print(status.isOverTemperatureShutdown());
  Serial.print("  Warning: ");
  Serial.println(status.isOverTemperatureWarning());
  Serial.print("OverVoltage: ");
  Serial.println(status.isOverVoltage());
  Serial.print("PowerOnReset: ");
  Serial.println(status.isPowerOnReset());
  Serial.print("SPI Error: ");
  Serial.print(status.isSPIError());
  Serial.print("  Address: ");
  Serial.print(status.isSPIAddressError());
  Serial.print("  Clock: ");
  Serial.print(status.isSPIClockFramingError());
  Serial.print("  Parity: ");
  Serial.println(status.isSPIParityError());
  if (status.isFault())
    driver.clearFault();
  delayMicroseconds(1); // ensure 400ns delay
  DRV8316_PWMMode val = driver.getPWMMode();
  Serial.print("PWM Mode: ");
  Serial.println(val);
  delayMicroseconds(1); // ensure 400ns delay
  bool lock = driver.isRegistersLocked();
  Serial.print("Lock: ");
  Serial.println(lock);
  Serial.println();
}
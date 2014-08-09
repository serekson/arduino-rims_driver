#ifndef _RIMS_DRIVER_H_INCLUDED
#define _RIMS_DRIVER_H_INCLUDED

#include <Arduino.h>
#include <math.h>
#include <LiquidCrystal.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <EEPROM.h>

#define SAMPLETIME 1000
#define RELAYWINDOWSIZE 5000

#define TEMPERATURE_PRECISION 9

#define EEPROM_STATUS 0x0F
#define EEPROM_TEMP0 0x10
#define EEPROM_TEMP1 0x11
#define EEPROM_TEMP2 0x12
#define EEPROM_TEMP3 0x13
#define EEPROM_KP0 0x14
#define EEPROM_KP1 0x15
#define EEPROM_KP2 0x16
#define EEPROM_KP3 0x17
#define EEPROM_KI0 0x18
#define EEPROM_KI1 0x19
#define EEPROM_KI2 0x1A
#define EEPROM_KI3 0x1B
#define EEPROM_KD0 0x1C
#define EEPROM_KD1 0x1D
#define EEPROM_KD2 0x1E
#define EEPROM_KD3 0x1F
#define EEPROM_DUTY0 0x20
#define EEPROM_DUTY1 0x21
#define EEPROM_DUTY2 0x22
#define EEPROM_DUTY3 0x23
#define EEPROM_LOG0 0x24
#define EEPROM_LOG1 0x25
#define EEPROM_LOG2 0x26
#define EEPROM_LOG3 0x27

#define LCD_WIDTH 20
#define LCD_HEIGHT 4

typedef uint8_t DeviceAddress[8];

class RimsClass {
  private:
  int _pin_temp;
  int _pin_element;
  int _pin_pump;
  
  uint32_t _logid;
  
  OneWire oneWire;
  DallasTemperature sensors;
  DeviceAddress sensor_addr;
  LiquidCrystal lcd;
  
  uint32_t _relayStartTime;

  double _PIDTempCur;
  double _PIDTempSet;
  double _PIDRelayCtrl;
  PID pid;
  
  void detTempSensors(void);

  void refreshRelay(void);
  void refreshDisplay(void);

  public:
  bool _pump_status;
  bool _element_status;
  bool _rims_mode;
  double _man_duty;
  
  RimsClass(int pin_temp, int pin_element, int pin_pump);

  void init(void);
  void run(void);
  
  void enablePump(void);
  void disablePump(void);
  void enableElement(void);
  void disableElement(void);
  void setPIDTuning(double Kp, double Ki, double Kd);
  void setMashTemp(double temp);
  void getRimsStatus(double* mash_temp, double* Kp, double* Ki, double* Kd);
  double getTemp(void);
  double getSetTemp(void);
  double getDutyCycle(void);
  
  void setLogID(uint32_t log_id);
  uint32_t getLogID(void);
  
  void updateMRFstat(uint8_t stat);
};
#endif
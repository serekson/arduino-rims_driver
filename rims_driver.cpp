#include <Arduino.h>
#include <rims_driver.h>

RimsClass::RimsClass(int pin_temp, int pin_element, int pin_pump) :
oneWire(pin_temp), sensors(&oneWire),
lcd(7, 6, 5, 4, A2, 2),
pid(&_PIDTempCur, &_PIDRelayCtrl, &_PIDTempSet, 0, 0, 0, DIRECT) {
  //setup pins
  _pin_temp = pin_temp;
  _pin_element = pin_element;
  _pin_pump = pin_pump;

  pinMode(_pin_element, OUTPUT);
  pinMode(_pin_pump, OUTPUT);
}

void RimsClass::init(void) {
  double Kp;
  double Ki;
  double Kd;
  
  union {
    byte asBytes[4];
    double asDouble;
    uint32_t asUINT32;
  } byte2var;

  disableElement();
  disablePump();
  
  byte2var.asBytes[0] = EEPROM.read(EEPROM_TEMP0);
  byte2var.asBytes[1] = EEPROM.read(EEPROM_TEMP1);
  byte2var.asBytes[2] = EEPROM.read(EEPROM_TEMP2);
  byte2var.asBytes[3] = EEPROM.read(EEPROM_TEMP3);
  _PIDTempSet = byte2var.asDouble;
  
  byte2var.asBytes[0] = EEPROM.read(EEPROM_KP0);
  byte2var.asBytes[1] = EEPROM.read(EEPROM_KP1);
  byte2var.asBytes[2] = EEPROM.read(EEPROM_KP2);
  byte2var.asBytes[3] = EEPROM.read(EEPROM_KP3);
  Kp=byte2var.asDouble;
  byte2var.asBytes[0] = EEPROM.read(EEPROM_KI0);
  byte2var.asBytes[1] = EEPROM.read(EEPROM_KI1);
  byte2var.asBytes[2] = EEPROM.read(EEPROM_KI2);
  byte2var.asBytes[3] = EEPROM.read(EEPROM_KI3);
  Ki=byte2var.asDouble;
  byte2var.asBytes[0] = EEPROM.read(EEPROM_KD0);
  byte2var.asBytes[1] = EEPROM.read(EEPROM_KD1);
  byte2var.asBytes[2] = EEPROM.read(EEPROM_KD2);
  byte2var.asBytes[3] = EEPROM.read(EEPROM_KD3);
  Kd=byte2var.asDouble;
  
  pid.SetTunings(Kp,Ki,Kd);
  
  byte2var.asBytes[0] = EEPROM.read(EEPROM_LOG0);
  byte2var.asBytes[1] = EEPROM.read(EEPROM_LOG1);
  byte2var.asBytes[2] = EEPROM.read(EEPROM_LOG2);
  byte2var.asBytes[3] = EEPROM.read(EEPROM_LOG3);
  _logid = byte2var.asUINT32;
  
  //start temp sensor
  sensors.begin();
  detTempSensors();

  //setup pid
  pid.SetOutputLimits(0,RELAYWINDOWSIZE);
  pid.SetSampleTime(SAMPLETIME);
  
  //init lcd
  lcd.begin(LCD_WIDTH, LCD_HEIGHT);
  lcd.setCursor(0,0);
  lcd.print("TEMP  DUT P E LOG  M");
  lcd.setCursor(0,2);
  lcd.print("temp  KP   KI  KD  W");
  
  lcd.setCursor(0,3);
  if(_PIDTempSet<100) lcd.print(" ");
  lcd.print(_PIDTempSet,1);
  
  lcd.setCursor(6,3);
  lcd.print(Kp,0);
  lcd.setCursor(11,3);
  lcd.print(Ki,0);
  lcd.setCursor(15,3);
  lcd.print(Kd,0);
  
  //log id
  //lcd.setCursor(17,1);
  //if(_logid<100) lcd.print(" ");
  //if(_logid<10) lcd.print(" ");
  //lcd.print(_logid,0);
  
  Serial.print("_logid: ");
  Serial.println(_logid);
}

void RimsClass::detTempSensors(void) {
  if(sensors.getDeviceCount()==1) {
    if(sensors.getAddress(sensor_addr, 0)) {
      //Serial.print("Device ");
      //Serial.print(i, DEC);
      //Serial.print(" Address: ");
      //mrf.printAddress(sensor_addr[i]);
      //Serial.println();
      sensors.setResolution(sensor_addr, TEMPERATURE_PRECISION);
    } else {
      Serial.println("Unable to find device ");
    }
  } else {
    Serial.println("Wrong number of temp sensors");
  }
}

void RimsClass::run(void) {
  sensors.requestTemperatures();
  _PIDTempCur=sensors.getTempF(sensor_addr);
  
  pid.Compute();
  
  refreshRelay();
  refreshDisplay();
}

void RimsClass::setPIDTuning(double Kp, double Ki, double Kd) {
  union {
    byte asBytes[4];
    double asDouble;
    uint32_t asUINT32;
  } byte2var;
  
  pid.SetTunings(Kp,Ki,Kd);
	
  lcd.setCursor(6,3);
  lcd.print(Kp,0);
  lcd.setCursor(11,3);
  lcd.print(Ki,0);
  lcd.setCursor(15,3);
  lcd.print(Kd,0);
	
  byte2var.asDouble = Kp;
  EEPROM.write(EEPROM_KP0,byte2var.asBytes[0]);
  EEPROM.write(EEPROM_KP1,byte2var.asBytes[1]);
  EEPROM.write(EEPROM_KP2,byte2var.asBytes[2]);
  EEPROM.write(EEPROM_KP3,byte2var.asBytes[3]);
  byte2var.asDouble = Ki;
  EEPROM.write(EEPROM_KI0,byte2var.asBytes[0]);
  EEPROM.write(EEPROM_KI1,byte2var.asBytes[1]);
  EEPROM.write(EEPROM_KI2,byte2var.asBytes[2]);
  EEPROM.write(EEPROM_KI3,byte2var.asBytes[3]);
  byte2var.asDouble = Kd;
  EEPROM.write(EEPROM_KD0,byte2var.asBytes[0]);
  EEPROM.write(EEPROM_KD1,byte2var.asBytes[1]);
  EEPROM.write(EEPROM_KD2,byte2var.asBytes[2]);
  EEPROM.write(EEPROM_KD3,byte2var.asBytes[3]);
}

void RimsClass::setMashTemp(double temp) {
  union {
    byte asBytes[4];
    double asDouble;
    uint32_t asUINT32;
  } byte2var;

  _PIDTempSet = temp;
  
  lcd.setCursor(0,3);
  if(_PIDTempSet<100) lcd.print(" ");
  lcd.print(_PIDTempSet,1);
  
  byte2var.asDouble = temp;
  EEPROM.write(EEPROM_TEMP0,byte2var.asBytes[0]);
  EEPROM.write(EEPROM_TEMP1,byte2var.asBytes[1]);
  EEPROM.write(EEPROM_TEMP2,byte2var.asBytes[2]);
  EEPROM.write(EEPROM_TEMP3,byte2var.asBytes[3]);
}

void RimsClass::setLogID(uint32_t log_id) {
  union {
    byte asBytes[4];
    double asDouble;
    uint32_t asUINT32;
  } byte2var;
  
  _logid = log_id;

  Serial.print("_logid: ");
  Serial.println(_logid);
  
  //lcd.setCursor(17,1);
  //if(_logid<100) lcd.print(" ");
  //if(_logid<10) lcd.print(" ");
  //lcd.print(_logid,0);
  
  byte2var.asUINT32 = _logid;
  EEPROM.write(EEPROM_LOG0,byte2var.asBytes[0]);
  EEPROM.write(EEPROM_LOG1,byte2var.asBytes[1]);
  EEPROM.write(EEPROM_LOG2,byte2var.asBytes[2]);
  EEPROM.write(EEPROM_LOG3,byte2var.asBytes[3]);
}

uint32_t RimsClass::getLogID(void) {
  return _logid;
}

void RimsClass::refreshRelay(void) {
  uint32_t currentTime = millis();

  if(currentTime - _relayStartTime > RELAYWINDOWSIZE)	{
    _relayStartTime += RELAYWINDOWSIZE;
  }

  if(currentTime - _relayStartTime <= _PIDRelayCtrl) {
    digitalWrite(_pin_element,HIGH);
  }	else {
    digitalWrite(_pin_element,LOW);
  }
}

void RimsClass::refreshDisplay(void) {
  uint8_t temp;

  lcd.setCursor(0,1);
  if(_PIDTempCur<100) lcd.print(" ");
  if(_PIDTempCur<10) lcd.print(" ");
  lcd.print(_PIDTempCur,1);
  
  lcd.setCursor(6,1);
  temp = _PIDRelayCtrl/RELAYWINDOWSIZE;
  if(temp<100) lcd.print(" ");
  if(temp<10) lcd.print(" ");
  lcd.print(temp);
  
  lcd.setCursor(10,1);
  lcd.print(_pump_status);
  lcd.setCursor(12,1);
  lcd.print(_element_status);
  lcd.setCursor(14,1);
  temp = _logid % 1000;
  if(temp<100) lcd.print("0");
  lcd.print(temp);
  
  lcd.setCursor(19,1);
  if(_rims_mode == 1) {
    lcd.print("A");
  } else {
    lcd.print("M");
  }
}

void RimsClass::enablePump(void) {
  _pump_status=1;
  digitalWrite(_pin_pump,HIGH);
  Serial.println("pump enabled");
}

void RimsClass::disablePump(void) {
  if(_element_status == 0) {
    _pump_status=0;
    digitalWrite(_pin_pump,LOW);
    Serial.println("pump disabled");
  } else {
    Serial.println("pump not disabled");
  }
}

void RimsClass::enableElement(void) {
  if(_pump_status == 1) {
    _element_status=1;
	
	if(_rims_mode == 1) {
      pid.SetMode(AUTOMATIC);
    } else {
      pid.SetMode(MANUAL);
      _PIDRelayCtrl=_man_duty*RELAYWINDOWSIZE;
    }
	
    Serial.println("element enabled");
  } else {
    Serial.println("element not enabled");
  }
}

void RimsClass::disableElement(void) {
  _element_status=0;
  pid.SetMode(MANUAL);
  _PIDRelayCtrl=0;
  refreshRelay();

  Serial.println("element disabled");
}

double RimsClass::getDutyCycle(void) {
  double duty_cycle;
  
  duty_cycle=_PIDRelayCtrl/RELAYWINDOWSIZE;
  
  //Serial.print("duty cycle: ");
  //Serial.println(duty_cycle);
  
  return duty_cycle;
}

double RimsClass::getTemp(void) {
  return _PIDTempCur;
}

double RimsClass::getSetTemp(void) {
  return _PIDTempSet;
}

void RimsClass::updateMRFstat(uint8_t stat) {
  lcd.setCursor(19,3);
  lcd.print(stat);
}

void RimsClass::getRimsStatus(double* mash_temp, double* Kp, double* Ki, double* Kd) {
  *mash_temp = _PIDTempSet;
  *Kp = pid.GetKp();
  *Ki = pid.GetKi();
  *Kd = pid.GetKd();
}
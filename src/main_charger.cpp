#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <EEPROM.h>

/*
Todo:
- COMM_STAT vom PSU hat immer Bit1 gesetzt, sollte aber nicht gesetzt sein. Evtl. CRC8-Fehler?

*/

// Action-Test
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);

#define ROTA  2
#define ROTB  3
#define ROTSW 4

#define PSU_WRITE_ADDR  0x00
#define PSU_READ_ADDR   0x0F

// PSU RAM
// PSU Status, Offset 0x00
#define PSU_STATUS          0x00  // Read Only
#define PSU_STAT_DC_ON          0x0001  // 1 = DC/DC enabled
#define PSU_STAT_BOOST_OK       0x0002  // 1 = Boost Voltage OK
#define PSU_STAT_AC_OK          0x0004  // 1 = AC input voltage OK
#define PSU_STAT_HVSD           0x0008  // 1 = Output shut down due to hi-voltage
#define PSU_STAT_FAN_FAIL       0x0010  // 1 = Fan failure
#define PSU_STAT_INT_TEMP       0x0040  // 1 = Heatsink over temperature alarm
#define PSU_STAT_ILIM           0x0080  // 1 = Current limit
#define PSU_STAT_UV_ALARM       0x0100  // 1 = Output Voltage below UV alarm threshold
#define PSU_STAT_UVSD           0x0200  // 1 = Output shut down due to lo-voltage
#define PSU_STAT_DC_ENABLE      0x0800  // 1 = DC enable asserted
#define PSU_STAT_REMOTE_OFF     0x1000  // 1 = Shutdown due to I2C remote off command     
#define PSU_STAT_MOD_DISABLE    0x2000  // 1 = Shutdown due to MOD_DISABLE input  
#define PSU_STAT_SHORT_PIN      0x4000  // 1 = Shutdown due to short pin transition    

// PSU Command, Offset 0x02
#define PSU_COMMAND         0x02  // Write
#define PSU_CMD_ON              0x02  // 1 = Turn on Rectifier
#define PSU_CMD_OFF             0x04  // 1 = Turn off Rectifier
#define PSU_CMD_ALARM_ON        0x08  // 1 = Enable Latent DC Failure Alarm
#define PSU_CMD_ALARM_OFF       0x10  // 1 = Disable Latent DC Failure Alarm

#define PSU_LOCATION        0x03  // RO Number that identifies device location in a shelf. (Hi-Nibble = Group type Lo-Nibble = Lo-Nibble of I2C Address)
#define PSU_TEMP_SEC_OR     0x04  // RO Output OR-ing FET temperature in degrees C Range 0C to +116C, 255=short, +127=open
#define PSU_TEMP_SEC_DIODES 0x05  // RO DC/DC output diode temperature in degrees C Range 0C to +116C, 255=short, 127=open
#define PSU_MEASURE_VOUT    0x06  // RO Output Voltage in Volts x 100
#define PSU_MEASURE_IOUT    0x08  // RO Output Current in Amps x 100
#define PSU_SETPOINT_ILIMIT 0x0A  // RW Current Limit Set Point in Amps x 100
#define PSU_SETPOINT_VOUT   0x0C  // RW Output Voltage Set Point in Volts x 100
#define PSU_SETPOINT_HVSD   0x0E  // RW Unit level High Voltage Shut Down Set Point in Volts x 100
#define PSU_SETPOINT_FAN    0x10  // RO Fan period, 38 - Low, 27 – Med, 1B – High, read only

// PSU EEPROM
#define PSU_SERIAL_NUMBER   0x42  // RO uchar[12] Device serial number as an array of ASCII characters
#define PSU_MODEL_NUMBER    0x4E  // RO uchar[8] Device model number as an array of ASCII characters
#define PSU_VSET_DEFAULT    0x72  // RW Power-up value for voltage setpoint
#define PSU_ISET_DEFAULT    0x74  // RW Power-up default for current limit setpoint
#define PSU_HVSD_DEFAULT    0x76  // RW Power-up default for HVSD setpoint

#define DISPLAY_TIMEOUT     60000  // OLED Protection Timeout (ms)
#define PSU_VSET_MAX        5800  // Maximum Voltage Setting (Volt * 100)
#define PSU_VSET_MIN        4320  // Minimum Voltage Setting (Volt * 100)
#define PSU_ISET_MAX        6250  // Maximum Current Setting (Ampere * 100)
#define PSU_ISET_MIN        0000  // Minimum Current Setting (Ampere * 100)
#define PSU_VSTART          3000  // Charge Start Voltage

// Prozessor-EEPROM
struct nonVolatileStruct
{
  uint16_t voltage;
  uint16_t current;
  uint16_t highvoltage;
};

nonVolatileStruct nonVolatile;

uint16_t actualReadVoltage = 0;
uint16_t actualReadCurrent = 0;
uint16_t actualSetVoltage = 0;
uint16_t actualSetCurrent = 0;

typedef enum {
  wait_for_battery,
  ramp_up,
  cc_charging,
  cv_charging,
  charging_done,
  wait_for_nobattery,
  oc_error,
  nc_error,
  setting_u,
  setting_i,
  save_dialog,
  save
} State_type;

State_type state = wait_for_battery;

bool psuDcOn = true;
bool displayOn = true;

// Setup a RotaryEncoder 
RotaryEncoder encoder(ROTB, ROTA, RotaryEncoder::LatchMode::FOUR3);

const unsigned long longPressThreshold = 1000; // Schwelle für einen langen Tastendruck in Millisekunden

unsigned long pressStartTime = 0;
bool isPressed = false;
bool wasLongPress = false;
bool wasPressed = false;

bool wasLongPressReported = false;

// a global variables to hold the last position
static int lastPos = 0;

long timeButtonPressed;

// Prototypes
void scan_i2c ();
uint8_t crc82(uint8_t *p, uint8_t len);
void write8Psu(uint8_t addr, uint8_t offs, uint8_t value);
void write16Psu(uint8_t addr, uint8_t offs, uint16_t value);
uint8_t readPsu(uint8_t addr, uint8_t len, uint8_t offs);
int leseDigitaleingang(int pin, int &vorherigerZustand);

uint8_t in_buffer[20];
uint8_t result;


// This interrupt routine will be called on any change of one of the input signals
void checkPosition()
{
  encoder.tick(); // just call tick() to check the state.
}

uint8_t readPsu(uint8_t addr, uint8_t len, uint8_t offs) 
{
  int i = 0;
  uint8_t comm_stat;

  uint8_t data[] = {addr, len, offs};
  uint8_t crc = crc82(data, sizeof(data));
  Wire.beginTransmission(addr);
  Wire.write(len);
  Wire.write(offs);
  Wire.write(crc);
  Wire.endTransmission(false);
  len += 3;
  Wire.requestFrom(addr, len);  // COMM_STAT + Daten + CRC16
  //Wire.requestFrom(addr, len+3, offs, len+3, true);
  comm_stat = Wire.read();  // COMM_STAT lesen 
  while (Wire.available())
  {
    in_buffer[i++] = Wire.read();
  }
  // CRC
  uint16_t crc16 = comm_stat;
  for (uint8_t i = 0; i < len; i++)
  {
    crc16 += in_buffer[i];
  }
  crc16 = ~crc16 + 1;

  if ((in_buffer[len] != (crc16 & 0xFF)) || (in_buffer[len+1] != (crc16 >> 8)))
  {
    comm_stat |= 0x10;  // CRC error
  }

  return comm_stat;
}

//void displayActual(String &sTitle)
void displayActual(const char* sTitle)
{
  float uValue, iValue;

  result = readPsu(PSU_READ_ADDR, 2, PSU_MEASURE_VOUT);
  uValue = (float)(in_buffer[0] + (in_buffer[1] << 8))/100.0;
  result = readPsu(PSU_READ_ADDR, 2, PSU_MEASURE_IOUT);
  iValue = (float)(in_buffer[0] + (in_buffer[1] << 8))/100.0;

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_profont22_mr);
//    
    u8g2.drawStr(0,14,sTitle);
    u8g2.drawHLine(0,20,128);

    u8g2.setCursor(0,41);
    u8g2.print("U: ");
    if (uValue < 10.0) u8g2.print(" ");  // Führendes Leerzeichen, falls erforderlich
    u8g2.print(uValue);
    u8g2.print(" V ");

    u8g2.setCursor(0,63);
    u8g2.print("I: ");
    if (iValue < 10.0) u8g2.print(" ");  // Führendes Leerzeichen, falls erforderlich
    u8g2.print(iValue);
    u8g2.print(" A ");
  } while ( u8g2.nextPage() );
}

// lineEdit: 1 / 2
void displaySetting(uint8_t lineEdit, uint16_t u, uint16_t i)
{
  float uValue, iValue;

  uValue = u / 100.0;
  iValue = i / 100.0;

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_profont22_mr);
    u8g2.drawStr(0,14,"Settings");
    u8g2.drawHLine(0,20,128);
    u8g2.setFontMode(0);
    
    u8g2.setCursor(0,41);
    u8g2.print("U: ");
    u8g2.setDrawColor((lineEdit == 1) ? 0 : 1);
    if (uValue < 10.0) u8g2.print(" ");  // Führendes Leerzeichen, falls erforderlich
    u8g2.print(uValue);
    u8g2.setDrawColor(1);
    u8g2.print(" V ");

    u8g2.setCursor(0,63);
    u8g2.print("I: ");
    u8g2.setDrawColor((lineEdit == 2) ? 0 : 1);
    if (iValue < 10.0) u8g2.print(" ");  // Führendes Leerzeichen, falls erforderlich
    u8g2.print(iValue);
    u8g2.setDrawColor(1);
    u8g2.print(" A ");
  } while ( u8g2.nextPage() );
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(ROTSW, INPUT_PULLUP);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(ROTB) , checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTA) , checkPosition, CHANGE);

//  scan_i2c ();

  u8g2.setBusClock(50000);
  u8g2.begin();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_profont22_mr);
    u8g2.drawStr(0,14,"FP2-Charger");
  } while ( u8g2.nextPage() );
  delay(1000);                          // anpassen an die Startup-Zeit des FP2

  EEPROM.get(0, nonVolatile);
  //nonVolatile.voltage = 20000;      // erneute Initialisierung der EEPROM-Struktur
  if (nonVolatile.voltage > 10000)    // Defaultwerte schreiben, wenn EEPROM neu
  {
    nonVolatile.voltage = 4800;
    nonVolatile.current = 6250;
    nonVolatile.highvoltage = 5900;
    EEPROM.put(0, nonVolatile);
  }

  // PSU auf Default-Werte einstellen
//  write16Psu(PSU_WRITE_ADDR, PSU_SETPOINT_VOUT, nonVolatile.voltage);
  write16Psu(PSU_WRITE_ADDR, PSU_SETPOINT_ILIMIT, nonVolatile.current);
  write16Psu(PSU_WRITE_ADDR, PSU_SETPOINT_HVSD, nonVolatile.highvoltage);
  // Spannung erst mal ausschalten
  write8Psu(PSU_WRITE_ADDR, PSU_COMMAND, PSU_CMD_OFF);

  timeButtonPressed = millis();
}


void loop() {
  int buttonState = digitalRead(ROTSW);

  if (buttonState == LOW) 
  {
    timeButtonPressed = millis();

    if (!isPressed) 
    {
      // Taste wurde neu gedrückt
      pressStartTime = millis(); // Startzeit messen
      isPressed = true; // Status aktualisieren
      wasLongPressReported = false; // Rücksetzen der langen Tastendruck-Meldung
    } 
    else if (!wasLongPressReported && (millis() - pressStartTime >= longPressThreshold)) 
    {
      // Langer Tastendruck wird gemeldet, während die Taste noch gedrückt ist
      Serial.println("Langer Tastendruck erkannt.");
      wasLongPressReported = true; // Verhindert weitere Meldungen, solange die Taste gedrückt bleibt
      wasPressed = true;
    }
  } 
  else if (buttonState == HIGH && isPressed) 
  {
    // Taste wurde losgelassen
    isPressed = false; // Status zurücksetzen

    if (!wasLongPressReported) 
    {
      // Es war ein kurzer Tastendruck, weil kein langer Tastendruck gemeldet wurde
      Serial.println("Kurzer Tastendruck erkannt.");
      wasPressed = true;
    }
  }

  encoder.tick(); // just call tick() to check the state.

  int newPos = encoder.getPosition();

  if (lastPos != newPos) 
  {
    if (encoder.getMillisBetweenRotations() < 50) 
    {
      newPos = lastPos + 10 * (newPos - lastPos);
      encoder.setPosition(newPos);
    }
    timeButtonPressed = millis();
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println((int)(encoder.getDirection()));
    lastPos = newPos;
  } // if


  switch (state)
  {
    case wait_for_battery:
      // Display nach gewisser Zeit ausschalten
      if ((millis() - timeButtonPressed) > DISPLAY_TIMEOUT)
      {
        if (displayOn)
        {
          // Display off
          u8g2.setPowerSave(1);
          displayOn = false;
          Serial.println("Display off");
        }
      }
      else
      {
        if (!displayOn)
        {
          u8g2.setPowerSave(0);
          displayOn = true;
          Serial.println("Display on");
        }

        // OLED aktualisieren
        displayActual("Connect");

        // Tastendruck
        if (wasPressed) 
        {
          if (wasLongPressReported)
          {
            encoder.setPosition(nonVolatile.voltage);
            lastPos = encoder.getPosition();
            state = setting_u;
          }
          // Bei kurzem Tastendruck in die Settings
          else
          {
            encoder.setPosition(nonVolatile.voltage);
            lastPos = encoder.getPosition();
            state = setting_u;
          }
          wasPressed = false;  
        }
      }

      // Spannung lesen
      result = readPsu(PSU_READ_ADDR, 2, PSU_MEASURE_VOUT);
      actualReadVoltage = in_buffer[0] + (in_buffer[1] << 8);
      // Akku angeschlossen?
      if (actualReadVoltage > PSU_VSTART)
      {
        // Stabile Spannung abwarten
        delay(250);
        result = readPsu(PSU_READ_ADDR, 2, PSU_MEASURE_VOUT);
        actualReadVoltage = in_buffer[0] + (in_buffer[1] << 8); 
        // Akku angeschlossen?
        if (actualReadVoltage > PSU_VSTART)
        { 
          actualSetVoltage = actualReadVoltage;
          actualSetCurrent = nonVolatile.current;
          write16Psu(PSU_WRITE_ADDR, PSU_SETPOINT_VOUT, actualSetVoltage); // Spannung setzen (RAM)
          write8Psu(PSU_WRITE_ADDR, PSU_COMMAND, PSU_CMD_ON);
          state = ramp_up;
        }
      }
      break;

    case ramp_up:
      // display
      displayActual("Ramp_up");
      // Strom lesen
      result = readPsu(PSU_READ_ADDR, 2, PSU_MEASURE_IOUT);
      actualReadCurrent = in_buffer[0] + (in_buffer[1] << 8);
      // Überstrom-Überwachung
      if (actualReadCurrent > actualSetCurrent * 2)
      {
        write8Psu(PSU_WRITE_ADDR, PSU_COMMAND, PSU_CMD_OFF);
        state = oc_error;
        break;
      }
      if (actualReadCurrent < actualSetCurrent)
      {
        // Spannung grossschrittig erhöhen (0,1V)
        actualSetVoltage += 10;
        write16Psu(PSU_WRITE_ADDR, PSU_SETPOINT_VOUT, actualSetVoltage); // Spannung setzen (RAM)   
        delay(250);     
      }
      else
      {
        state = cc_charging;
      }
      break;

    case cc_charging:
      // display
      displayActual("CC-Charg.");
      // Strom lesen
      result = readPsu(PSU_READ_ADDR, 2, PSU_MEASURE_IOUT);
      actualReadCurrent = in_buffer[0] + (in_buffer[1] << 8);
      // Überstrom-Überwachung
      if (actualReadCurrent > actualSetCurrent * 2)
      {
        write8Psu(PSU_WRITE_ADDR, PSU_COMMAND, PSU_CMD_OFF);
        state = oc_error;
        break;
      }
      // Überwachung auf Unterbrechung
      if (actualReadCurrent == 0)
      {
        write8Psu(PSU_WRITE_ADDR, PSU_COMMAND, PSU_CMD_OFF);
        state = nc_error;
        break;
      }
      if (actualReadCurrent < actualSetCurrent)
      {
        // Spannung kleinschrittig erhöhen (0,01V)
        actualSetVoltage++;
        write16Psu(PSU_WRITE_ADDR, PSU_SETPOINT_VOUT, actualSetVoltage); // Spannung setzen (RAM)   
        delay(250);     
      }
      // Ladeschlussspannung erreicht?
      if (actualSetVoltage >= nonVolatile.voltage)
      {
        state = cv_charging;
      }
      break;

    case cv_charging:
      // display
      displayActual("CV-Charg.");
      // Strom lesen
      result = readPsu(PSU_READ_ADDR, 2, PSU_MEASURE_IOUT);
      actualReadCurrent = in_buffer[0] + (in_buffer[1] << 8);
      // Überstrom-Überwachung
      if (actualReadCurrent > actualSetCurrent * 2)
      {
        write8Psu(PSU_WRITE_ADDR, PSU_COMMAND, PSU_CMD_OFF);
        state = oc_error;
        break;
      }
      // CV-Charging beenden, sobald nur noch 1/10 des Ladestroms fliesst
      if (actualReadCurrent < actualSetCurrent / 10)
      {
        // PSU ausschalten
        write8Psu(PSU_WRITE_ADDR, PSU_COMMAND, PSU_CMD_OFF);  
        state = wait_for_nobattery;
      }
      break;

    case wait_for_nobattery:
      // display
      displayActual("Ready");
      // Spannung lesen
      result = readPsu(PSU_READ_ADDR, 2, PSU_MEASURE_VOUT);
      actualReadVoltage = in_buffer[0] + (in_buffer[1] << 8);    
      if (actualReadVoltage < PSU_VSET_MIN)
      {
        delay(1000);
        state = wait_for_battery;
      }
      break;

    case oc_error:
      // display
      displayActual("OC-Error");
      break;

    case nc_error:
      // display
      displayActual("NC-Error");
      break;

    case setting_u:
      nonVolatile.voltage = encoder.getPosition();
      
      if (nonVolatile.voltage > PSU_VSET_MAX)
      {
        nonVolatile.voltage = PSU_VSET_MAX;
        encoder.setPosition(PSU_VSET_MAX);
        lastPos = encoder.getPosition();
      }
      else if (nonVolatile.voltage < PSU_VSET_MIN)
      {
        nonVolatile.voltage = PSU_VSET_MIN;
        encoder.setPosition(PSU_VSET_MIN);
        lastPos = encoder.getPosition();
      }

      displaySetting(1, nonVolatile.voltage, nonVolatile.current);
      write16Psu(PSU_WRITE_ADDR, PSU_SETPOINT_VOUT, nonVolatile.voltage); // Spannung setzen (RAM)

      if (wasPressed)
      {
        write16Psu(PSU_WRITE_ADDR, PSU_VSET_DEFAULT, nonVolatile.voltage); // Spannung setzen (EEPROM)
        EEPROM.put(0, nonVolatile);

        encoder.setPosition(nonVolatile.current);
        lastPos = encoder.getPosition();
        wasPressed = false;
        state = setting_i;
      }
      break;

    case setting_i:
      nonVolatile.current = encoder.getPosition();
      
      if (nonVolatile.current > PSU_ISET_MAX)
      {
        nonVolatile.current = PSU_ISET_MAX;
        encoder.setPosition(PSU_ISET_MAX);
        lastPos = encoder.getPosition();
      }
      else if (nonVolatile.current < PSU_ISET_MIN)
      {
        nonVolatile.current = PSU_ISET_MIN;
        encoder.setPosition(PSU_ISET_MIN);
        lastPos = encoder.getPosition();
      }

      displaySetting(2, nonVolatile.voltage, nonVolatile.current);
      write16Psu(PSU_WRITE_ADDR, PSU_SETPOINT_ILIMIT, nonVolatile.current); // Strom setzen (RAM)

      if (wasPressed)
      {
        write16Psu(PSU_WRITE_ADDR, PSU_ISET_DEFAULT, nonVolatile.current); // Strom setzen (EEPROM)
        EEPROM.put(0, nonVolatile);
        wasPressed = false;
        state = wait_for_battery;
      }
      break;

    default:
      break;
  }
}

void scan_i2c () {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning for power supplies");

  for (address = 0; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Power supply found at address 0x");
      if (address < 16){
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    }
    if (error == 4) {
      Serial.print("Power supply error at address 0x");
      if (address < 16){
        Serial.print("0");
      }
      Serial.println(address, HEX);      
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No power supplies found");
    Serial.println("Halting, fix i2c problem");
    while(1);
  } else {
    Serial.println("Setting default voltage");
  }
}

uint8_t crc82(uint8_t *p, uint8_t len) 
{
  uint8_t chk = 0;
  for (uint8_t i = 0; i < len; i++) 
  {
    chk += p[i];
  }
  chk = 256 - chk;
//  chk = ~chk + 1;
  return chk;
}

void write8Psu(uint8_t addr, uint8_t offs, uint8_t value)
{
  uint8_t len = 1;
  uint8_t data[] = {addr, len, offs, value};
  uint8_t crc = crc82(data, sizeof(data));
  Wire.beginTransmission(addr);
  Wire.write(len);
  Wire.write(offs);
  Wire.write(value);
  Wire.write(crc);
  Wire.endTransmission();
}

void write16Psu(uint8_t addr, uint8_t offs, uint16_t value)
{
  uint8_t len = 2;
  uint8_t h = value >> 8;
  uint8_t l = value & 0xFF;
  uint8_t data[] = {addr, len, offs, l, h};
  uint8_t crc = crc82(data, sizeof(data));
  Wire.beginTransmission(addr);
  Wire.write(len);
  Wire.write(offs);
  Wire.write(l);
  Wire.write(h);
  Wire.write(crc);
  Wire.endTransmission();
}

/*
void readPsu(uint8_t addr, uint8_t p, uint8_t offs) {
  uint8_t in_bufferer[5];

  uint8_t data[] = {addr, p, offs};
  uint8_t crc = crc82(data, sizeof(data));
  Wire.beginTransmission(addr);
  Wire.write(p);
  Wire.write(offs);
  Wire.write(crc);
  Wire.endTransmission();
  Wire.requestFrom(addr, 5, true);

  delayMicroseconds(10000);
  if (Wire.available())
  {
    Wire.readBytes(in_bufferer, 2);
    Serial.print("Receive: ");
    Serial.print(in_bufferer[0]);
    Serial.print(" ");
    Serial.println(in_bufferer[1]); 

  }
}
*/


int leseDigitaleingang(int pin, int &vorherigerZustand) {
  int aktuellerZustand = digitalRead(pin);
  if (aktuellerZustand != vorherigerZustand) {
    vorherigerZustand = aktuellerZustand;
    return aktuellerZustand;  // Zustand hat sich geändert
  } else {
    return -1;  // Keine Änderung, -1 signalisiert dies
  }
}

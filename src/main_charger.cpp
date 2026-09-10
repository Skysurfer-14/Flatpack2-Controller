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

#define DISPLAY_TIMEOUT     60000 // OLED Protection Timeout (ms)
#define PSU_VSET_MAX        5800  // Maximum Voltage Setting (Volt * 100)
#define PSU_VSET_MIN        4320  // Minimum Voltage Setting (Volt * 100)
#define PSU_V_RELOAD_DEFAULT 4800 // Default restart voltage (Volt * 100)
#define PSU_ISET_MAX        6250  // Maximum Current Setting (Ampere * 100)
#define PSU_ISET_MIN        0000  // Minimum Current Setting (Ampere * 100)
#define CV_CHARGING_TIMEOUT (30UL * 60UL * 1000UL)

// Charger defines
#define OC_CURRENT_FACTOR   2     // Threshold factor for OC-Error
#define OC_ERROR_ENABLED    0     // Enable OC-Error

// Prozessor-EEPROM
struct nonVolatileStruct
{
  uint16_t voltage;
  uint16_t current;
  uint16_t highvoltage;
  uint16_t reloadVoltage;
  uint8_t chargeMode;
};

nonVolatileStruct nonVolatile;

enum ChargeMode : uint8_t
{
  single_charge,
  cycle_charge
};

uint16_t actualReadVoltage = 0;
uint16_t actualReadCurrent = 0;
uint16_t actualSetVoltage = 0;
uint16_t actualSetCurrent = 0;

typedef enum {
  wait_for_battery,
  ramp_up,
  cc_charging,
  cv_charging,
  wait_for_reload,
  charging_done,
  wait_for_nobattery,
  oc_error,
  nc_error,
  setting_u,
  setting_i,
  setting_reload,
  setting_charge_mode,
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
unsigned long cvChargingStartTime = 0;

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

/**
 * @brief Reads data from a power supply unit (PSU) over I2C.
 * 
 * This function communicates with a PSU via I2C to read a specified number
 * of bytes from a given address and offset. It calculates and verifies a CRC
 * to ensure data integrity. If a CRC error is detected, the appropriate bit is set
 * in the communication status.
 * 
 * @param addr The I2C address of the PSU.
 * @param len The number of data bytes to read from the PSU.
 * @param offs The offset from which to start reading data.
 * @return The communication status, with a bit set if a CRC error is detected.
 */
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

/**
 * @brief displayActual: Display actual values of the power supply
 * @param sTitle The title string to be displayed on the first line of the display
 *
 * This function reads the actual values of the output voltage and current from the
 * power supply and displays them on the display. The title string is displayed on
 * the first line. The values are displayed with two decimal places on the second and
 * third line.
 */
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


/**
 * Displays the settings for voltage and current on the screen.
 * 
 * This function renders the voltage (U) and current (I) settings 
 * using the u8g2 library. The displayed values are converted from 
 * integer to float by dividing by 100.0. It highlights the field 
 * currently being edited, as indicated by the lineEdit parameter.
 *
 * @param lineEdit Indicates which line (1 for voltage, 2 for current) 
 *                 is being edited. The corresponding line will be 
 *                 highlighted.
 * @param u        The voltage value to be displayed, scaled by 100.
 * @param i        The current value to be displayed, scaled by 100.
 */
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

void displayReloadAndChargeMode(uint8_t lineEdit, uint16_t reloadVoltage, uint8_t chargeMode)
{
  float reloadValue = reloadVoltage / 100.0;

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_profont22_mr);
    u8g2.drawStr(0,14,"Settings");
    u8g2.drawHLine(0,20,128);
    u8g2.setFontMode(0);

    u8g2.setCursor(0,41);
    u8g2.print("R: ");
    u8g2.setDrawColor((lineEdit == 1) ? 0 : 1);
    if (reloadValue < 10.0) u8g2.print(" ");
    u8g2.print(reloadValue);
    u8g2.setDrawColor(1);
    u8g2.print(" V ");

    u8g2.setCursor(0,63);
    u8g2.print("C: ");
    u8g2.setDrawColor((lineEdit == 2) ? 0 : 1);
    u8g2.print(chargeMode == cycle_charge ? "Multi" : "Single");
    u8g2.setDrawColor(1);
  } while ( u8g2.nextPage() );
}

/**
 * @brief Setup function, called once at startup.
 *
 * Initialize I2C interface and U8G2 library, display the title string
 * and wait until the FP2-Charger has finished its startup sequence.
 * Then, read the non-volatile values from EEPROM and initialize the
 * power supply with these values.
 */
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
    u8g2.setFont(u8g2_font_profont17_mr);
    u8g2.drawStr(10,42,"Version 2.00");
  } while ( u8g2.nextPage() );
  delay(1000);                          // anpassen an die Startup-Zeit des FP2

  EEPROM.get(0, nonVolatile);
  //nonVolatile.voltage = 20000;      // erneute Initialisierung der EEPROM-Struktur
  if (nonVolatile.voltage > 10000)    // Defaultwerte schreiben, wenn EEPROM neu
  {
    nonVolatile.voltage = 4800;
    nonVolatile.current = 6250;
    nonVolatile.highvoltage = 5900;
    nonVolatile.reloadVoltage = PSU_V_RELOAD_DEFAULT;
    nonVolatile.chargeMode = single_charge;
    EEPROM.put(0, nonVolatile);
  }
  if (nonVolatile.reloadVoltage < PSU_VSET_MIN || nonVolatile.reloadVoltage > nonVolatile.voltage)
  {
    nonVolatile.reloadVoltage = min(PSU_V_RELOAD_DEFAULT, nonVolatile.voltage);
    EEPROM.put(0, nonVolatile);
  }
  if (nonVolatile.chargeMode > cycle_charge)
  {
    nonVolatile.chargeMode = single_charge;
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


/**
 * loop(): The main loop of the program. It checks the state of the rotary encoder
 * and the button. Depending on the state, it either displays the actual values
 * of the power supply or allows the user to enter new values. 
 * The states are:
 * - wait_for_battery: wait for the battery to be connected
 * - ramp_up: ramp up the voltage to the set value
 * - cc_charging: keep the voltage at the set value and regulate the current
 * - cv_charging: keep the voltage at the set value and regulate the current, but
 *                with a lower current limit
 * - wait_for_reload: wait until the battery voltage falls to the recharge threshold
 * - wait_for_nobattery: wait for the battery to be disconnected
 * - oc_error: display an overcurrent error
 * - nc_error: display a no connection error
 * - setting_u: allow the user to set the voltage
 * - setting_i: allow the user to set the current
 * - setting_reload: allow the user to set the reload voltage
 * - setting_charge_mode: allow the user to select single or cycle charge
 */
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
      if (actualReadVoltage > PSU_VSET_MIN)
      {
        // Stabile Spannung abwarten
        delay(250);
        result = readPsu(PSU_READ_ADDR, 2, PSU_MEASURE_VOUT);
        actualReadVoltage = in_buffer[0] + (in_buffer[1] << 8); 
        // Akku angeschlossen?
        if (actualReadVoltage > PSU_VSET_MIN)
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
      if (OC_ERROR_ENABLED && (actualReadCurrent > actualSetCurrent * OC_CURRENT_FACTOR))
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
      if (OC_ERROR_ENABLED && (actualReadCurrent > actualSetCurrent * 2))
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
      }
      else
      {
        // Spannung kleinschrittig verringern (0,01V)
        actualSetVoltage--;
      }
      write16Psu(PSU_WRITE_ADDR, PSU_SETPOINT_VOUT, actualSetVoltage); // Spannung setzen (RAM)   
      delay(250); 
      // Ladeschlussspannung erreicht?
      if (actualSetVoltage >= nonVolatile.voltage)
      {
        write16Psu(PSU_WRITE_ADDR, PSU_SETPOINT_VOUT, nonVolatile.voltage); // Spannung setzen (RAM)   
        cvChargingStartTime = millis();
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
      if (OC_ERROR_ENABLED && (actualReadCurrent > actualSetCurrent * OC_CURRENT_FACTOR))
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
        state = nonVolatile.chargeMode == cycle_charge ? wait_for_reload : wait_for_nobattery;
        break;
      }
      if (actualReadCurrent > uint16_t (actualSetCurrent * 1.5))
      {
        // zurück zur Stromregelung, wenn die Batteriespannung durch hohe Last wieder sinkt
        state = cc_charging;
      }
      if (millis() - cvChargingStartTime >= CV_CHARGING_TIMEOUT)
      {
        write8Psu(PSU_WRITE_ADDR, PSU_COMMAND, PSU_CMD_OFF);
        state = nonVolatile.chargeMode == cycle_charge ? wait_for_reload : wait_for_nobattery;
      }
      break;

    case wait_for_reload:
      // PSU ausgeschaltet lassen, bis die Batteriespannung wieder sinkt.
      displayActual("Wait Reload");
      result = readPsu(PSU_READ_ADDR, 2, PSU_MEASURE_VOUT);
      actualReadVoltage = in_buffer[0] + (in_buffer[1] << 8);
      if (actualReadVoltage <= nonVolatile.reloadVoltage)
      {
        actualSetVoltage = actualReadVoltage;
        actualSetCurrent = nonVolatile.current;
        write16Psu(PSU_WRITE_ADDR, PSU_SETPOINT_VOUT, actualSetVoltage);
        write8Psu(PSU_WRITE_ADDR, PSU_COMMAND, PSU_CMD_ON);
        state = ramp_up;
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
      if (nonVolatile.reloadVoltage > nonVolatile.voltage)
      {
        nonVolatile.reloadVoltage = nonVolatile.voltage;
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
        encoder.setPosition(nonVolatile.reloadVoltage);
        lastPos = encoder.getPosition();
        wasPressed = false;
        state = setting_reload;
      }
      break;

    case setting_reload:
      nonVolatile.reloadVoltage = encoder.getPosition();

      if (nonVolatile.reloadVoltage > nonVolatile.voltage)
      {
        nonVolatile.reloadVoltage = nonVolatile.voltage;
        encoder.setPosition(nonVolatile.voltage);
        lastPos = encoder.getPosition();
      }
      else if (nonVolatile.reloadVoltage < PSU_VSET_MIN)
      {
        nonVolatile.reloadVoltage = PSU_VSET_MIN;
        encoder.setPosition(PSU_VSET_MIN);
        lastPos = encoder.getPosition();
      }

      displayReloadAndChargeMode(1, nonVolatile.reloadVoltage, nonVolatile.chargeMode);

      if (wasPressed)
      {
        EEPROM.put(0, nonVolatile);
        wasPressed = false;
        encoder.setPosition(nonVolatile.chargeMode);
        lastPos = encoder.getPosition();
        state = setting_charge_mode;
      }
      break;

    case setting_charge_mode: {
      int chargeModePosition = encoder.getPosition();

      if (chargeModePosition > cycle_charge)
      {
        nonVolatile.chargeMode = cycle_charge;
        encoder.setPosition(cycle_charge);
        lastPos = encoder.getPosition();
      }
      else if (chargeModePosition < single_charge)
      {
        nonVolatile.chargeMode = single_charge;
        encoder.setPosition(single_charge);
        lastPos = encoder.getPosition();
      }
      else
      {
        nonVolatile.chargeMode = chargeModePosition;
      }

      displayReloadAndChargeMode(2, nonVolatile.reloadVoltage, nonVolatile.chargeMode);

      if (wasPressed)
      {
        EEPROM.put(0, nonVolatile);
        wasPressed = false;
        state = wait_for_battery;
      }
      break;
    }

    default:
      break;
  }
}

/**
 * \brief Scans the i2c bus for power supplies.
 *
 * Prints messages to the serial console indicating which power supplies are found.
 * If no power supplies are found, prints an error message and loops indefinitely.
 * If power supplies are found, prints a message indicating that the default voltage
 * will be set.
 */
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

/**
 * Calculates the CRC8 checksum for a given array of bytes.
 *
 * This function computes a simple checksum by summing all bytes in the input
 * array and then subtracting the result from 256. The checksum is used to
 * ensure data integrity when transmitting or storing data.
 *
 * @param p Pointer to the array of bytes for which the checksum is calculated.
 * @param len The number of bytes in the input array.
 * @return The calculated CRC8 checksum.
 */

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

/**
 * Write a byte to the specified PSU register at the given address.
 * Automatically calculates and appends the CRC8 checksum.
 *
 * @param addr The I2C address of the PSU.
 * @param offs The offset of the register to be written.
 * @param value The value to be written.
 */
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

/**
 * Write a 16-bit value to the specified PSU register at the given address.
 * Automatically calculates and appends the CRC8 checksum.
 *
 * @param addr The I2C address of the PSU.
 * @param offs The offset of the register to be written.
 * @param value The value to be written.
 */
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


/**
 * Liest den digitalen Zustand des angegebenen Pins und
 * vergleicht ihn mit dem vorherigen Zustand.
 * Wenn sich der Zustand geändert hat, wird der neue Zustand
 * zurückgegeben. Wenn sich der Zustand nicht geändert hat, wird -1
 * zurückgegeben.
 * @param pin Der Pin, dessen Zustand gelesen werden soll.
 * @param vorherigerZustand Referenz auf eine Variable, die den
 * vorherigen Zustand des Pins speichert.
 * @return Der neue Zustand des Pins, oder -1 wenn sich der Zustand
 * nicht geändert hat.
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

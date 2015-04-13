//
// 2015-04-12
// Checkout sketch for MOD-1016v6 lightning sensor
// Written by ductsoup, public domain
//

//
// Send output to serial or to the bridge
//
#define YUN_CONSOLE
#ifdef YUN_CONSOLE
#include <Console.h>
#define emit(s) Console.print(s);
#define emitb(s) Console.print("B"); Console.print(s,BIN);
#define emith(s) Console.print("0x"); Console.print(s,HEX);
#define emitln(s) Console.println(s);
#else
#define emit(s) Serial.print(s);
#define emitb(s) Serial.print(s,BIN);
#define emith(s) Serial.print("0x"); Serial.print(s,HEX);
#define emitln(s) Serial.println(s);
#endif

//
// Wiring
//
// SCL to analog 3
// SDA to analog 2
// VDD to 3.3V DC (there's a problem generating interrupts with 5V)
// GND to ground
// INT to digital 10
//

//
// Configure SoftI2C
// http://playground.arduino.cc/Main/SoftwareI2CLibrary (necessary for repeated start condition)
// https://github.com/felias-fogg/SoftI2CMaster
//
#define SCL_PORT PORTF  // YUN A3
#define SCL_PIN 4
#define SDA_PORT PORTF  // YUN A2
#define SDA_PIN 5
#define INT_PIN 10      // YUN D10
#include <SoftI2CMaster.h>

//
// AS3935
//
#define AS3935_ADDRESS          (0x03 << 1) // convert 7-bit address to 8-bit
#define AS3935_TUNE             (0x04)      // from the package label
//
#define AS3935_DIRECT_COMMAND_BYTE      (0x96)
#define AS3935_AFE_GB_INDOOR    (B10010)
#define AS3935_AFE_GB_OUTDOOR   (B01110)
// register and mask pairs
#define AS3935_AFE_GB           (0x00), (0x3e)
#define AS3935_PWD              (0x00), (0x01)
#define AS3935_NF_LEV           (0x01), (0x70)
#define AS3935_WDTH             (0x01), (0x0f)
#define AS3935_CL_STAT          (0x02), (0x40)
#define AS3935_MIN_NUM_LIGH     (0x02), (0x30)
#define AS3935_SREJ             (0x02), (0x0f)
#define AS3935_LCO_FDIV         (0x03), (0xc0)
#define AS3935_MASK_DIST        (0x03), (0x20)
#define AS3935_INT              (0x03), (0x0f)
#define AS3935_S_LIG_L          (0x04), (0xff)
#define AS3935_S_LIG_M          (0x05), (0xff)
#define AS3935_S_LIG_MM         (0x06), (0x1f)
#define AS3935_DISTANCE         (0x07), (0x3f)
#define AS3935_DISP_LCO         (0x08), (0x80)
#define AS3935_DISP_SRCO        (0x08), (0x40)
#define AS3935_DISP_TRCO        (0x08), (0x20)
#define AS3935_TUN_CAP          (0x08), (0x0f)
#define AS3935_LDLUT1           (0x09), (0xff)
#define AS3935_LDLUT2           (0x0A), (0xff)
#define AS3935_LDLUT3           (0x0B), (0xff)
#define AS3935_LDLUT4           (0x0C), (0xff)
#define AS3935_LDLUT5           (0x0D), (0xff)
#define AS3935_LDLUT6           (0x0E), (0xff)
#define AS3935_LDLUT7           (0x0F), (0xff)
#define AS3935_LDLUT8           (0x10), (0xff)
#define AS3935_LDLUT9           (0x11), (0xff)
#define AS3935_LDLUT10          (0x12), (0xff)
#define AS3935_LDLUT11          (0x13), (0xff)
#define AS3935_LDLUT12          (0x14), (0xff)
#define AS3935_LDLUT13          (0x15), (0xff)
#define AS3935_LDLUT14          (0x16), (0xff)
#define AS3935_LDLUT15          (0x17), (0xff)
#define AS3935_LDLUT16          (0x18), (0xff)
#define AS3935_LDLUT17          (0x19), (0xff)
#define AS3935_LDLUT18          (0x1A), (0xff)
#define AS3935_LDLUT19          (0x1B), (0xff)
#define AS3935_LDLUT20          (0x1C), (0xff)
#define AS3935_LDLUT21          (0x1D), (0xff)
#define AS3935_LDLUT22          (0x1E), (0xff)
#define AS3935_LDLUT23          (0x1F), (0xff)
#define AS3935_LDLUT24          (0x20), (0xff)
#define AS3935_LDLUT25          (0x21), (0xff)
#define AS3935_LDLUT26          (0x22), (0xff)
#define AS3935_LDLUT27          (0x23), (0xff)
#define AS3935_LDLUT28          (0x24), (0xff)
#define AS3935_LDLUT29          (0x25), (0xff)
#define AS3935_LDLUT30          (0x26), (0xff)
#define AS3935_LDLUT31          (0x27), (0xff)
#define AS3935_LDLUT32          (0x28), (0xff)
#define AS3935_LDLUT33          (0x29), (0xff)
#define AS3935_LDLUT34          (0x2A), (0xff)
#define AS3935_LDLUT35          (0x2B), (0xff)
#define AS3935_LDLUT36          (0x2C), (0xff)
#define AS3935_LDLUT37          (0x2D), (0xff)
#define AS3935_LDLUT38          (0x2E), (0xff)
#define AS3935_LDLUT39          (0x2F), (0xff)
#define AS3935_LDLUT40          (0x30), (0xff)
#define AS3935_LDLUT41          (0x31), (0xff)
#define AS3935_LDLUT42          (0x32), (0xff)
//
#define AS3935_PRESET_DEFAULT   (0x3C), (0xff)
#define AS3935_CALIB_RCO        (0X3D), (0xff)
//
#define AS3935_DISTANCE_OUT_OF_RANGE    (0x3f)

//
// Interrupt service routine
// http://playground.arduino.cc/Main/PinChangeInterrupt
//
volatile unsigned long int_event;
volatile bool event;
ISR (PCINT0_vect) {
  digitalWrite(13, HIGH);
  if (digitalRead(INT_PIN)) {
    event = true;
    delayMicroseconds(2000);
    int_event = read_bit(AS3935_INT);  // read the INT register
  }
  digitalWrite(13, LOW);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

// @RR, this is very clever
// http://developer.mbed.org/users/casper/code/AS3935/file/346c723cac97/AS3935.cpp
uint8_t _ffsz(uint8_t mask) {
  uint8_t i = 0;
  while (!(mask & 1)) {
    mask >>= 1;
    i++;
  }
  return i;
}

// Read a register from the device
uint8_t read_raw(uint8_t reg) {
  uint8_t val = 0;

  i2c_start(AS3935_ADDRESS | I2C_WRITE);
  i2c_write(reg);
  i2c_rep_start(AS3935_ADDRESS | I2C_READ);
  val = i2c_read(true);
  i2c_stop();

  return (val);
}

// Read bits from the device
uint8_t read_bit(uint8_t reg, uint8_t mask) {
  uint8_t val = 0;
  uint8_t i = _ffsz(mask);

  val = (read_raw(reg) & mask ) >> i;

  return (val);
}

// Write bits to the device
void write_bit(uint8_t reg, uint8_t mask, uint8_t x) {
  uint8_t val = 0;
  uint8_t i = _ffsz(mask);

  val = (read_raw(reg) & ~(mask)) | (x << i);
  i2c_start(AS3935_ADDRESS | I2C_WRITE);
  i2c_write(reg);
  i2c_write(val);
  i2c_stop();
}

// Read registers 0x00 through 0x08 for debugging purposes
void dump_regs(void) {
  int i, j, x;
  emitln(">Registers");
  emitln("reg 76543210");
  emitln("--- --------");
  for (i = 0 ; i <= 0x08 ; i++) {
    emith(i); emit(" ");
    if (i == 0) {
      i2c_start(AS3935_ADDRESS | I2C_WRITE);
      i2c_write(i);
      i2c_rep_start(AS3935_ADDRESS | I2C_READ);
    }
    x = i2c_read(i == 0x0f ? true : false);
    for (j = 0 ; j < 8 ; j++, x <<= 1)
      emit(x & 0x80 ? "1" : "0");
    emitln();
  }
  i2c_stop();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

// Set all registers in default mode
void presetDefault(void) {
  write_bit(AS3935_PRESET_DEFAULT, AS3935_DIRECT_COMMAND_BYTE);
}
// Calibrate the internal RC Oscillators
void calibrateRCO(void) {
  write_bit(AS3935_CALIB_RCO, AS3935_DIRECT_COMMAND_BYTE);
  write_bit(AS3935_DISP_TRCO, 0x1);
  delay(2);
  write_bit(AS3935_DISP_TRCO, 0x0);
}

void clearStatistics(void) {
  write_bit(AS3935_CL_STAT, 0x1);
  write_bit(AS3935_CL_STAT, 0x0);
  write_bit(AS3935_CL_STAT, 0x1);
}

bool isSleeping(void) {
  return ((bool) read_bit(AS3935_PWD));
}

void setSleep(bool val) {
  write_bit(AS3935_PWD, (val) ? 0x1 : 0x0);
  if (!val)
    calibrateRCO();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

// Energy of last lighting event, no uints
uint32_t getEnergy(void) {
  return ((uint32_t) read_bit(AS3935_S_LIG_MM) << 16) + ((uint32_t) read_bit(AS3935_S_LIG_M) << 8) + ((uint32_t) read_bit(AS3935_S_LIG_L));
}

// Return the storm distance in km or miles, 0 if out of range
float getDistance(bool miles = 0) {
  uint8_t val = float(read_bit(AS3935_DISTANCE));
  if (val == AS3935_DISTANCE_OUT_OF_RANGE)
    val = 0;
  if (miles)
    val *= 0.621371;
  return (val);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

// Set the AFE to indoors
void setIndoors(void) {
  write_bit(AS3935_AFE_GB, AS3935_AFE_GB_INDOOR);
}

// Set the AFE to outdoors
void setOutdoors(void) {
  write_bit(AS3935_AFE_GB, AS3935_AFE_GB_OUTDOOR);
}

// Return the AFE setting, 0 for indoor or 1 for outdoor
uint8_t getGain(void) {
  return (read_bit(AS3935_AFE_GB) == AS3935_AFE_GB_INDOOR) ? 0 : 1;
}

// Return watchdog threshold (WDTH)
uint8_t getWatchdog(void) {
  return read_bit(AS3935_WDTH);
}

// Set watchdog threshold (WDTH), range is 0 to 10 decimal
void setWatchdog(uint8_t val) {
  if (val >= 0 && val <= 10)
    write_bit(AS3935_WDTH, val);
}

// Get the noise floor limit threshold (NF_LEV)
uint8_t getNoiseFloor(void) {
  return read_bit(AS3935_NF_LEV);
}

// Set the noise floor limit threshold (NF_LEV), range is 0 to 7
void setNoiseFloor(uint8_t val) {
  if (val >= 0 && val <= 7)
    write_bit(AS3935_NF_LEV, val);
}

// Return the noise floor limit threshold in uVrms
uint16_t getNoiseThreshold(void) {
  uint16_t val[2][8] = {
    {28, 45, 62, 78, 95, 112, 130, 146},
    {390, 630, 860, 1100, 1140, 1570, 1800, 2000}
  };
  return val[getGain()][getNoiseFloor()];
}

// Return spike rejection (SREJ)
uint8_t getSpikeRejection(void) {
  return (read_bit(AS3935_SREJ));
}
// Set spike rejection (SREJ), range is 0 to 11
void setSpikeRejection(uint8_t val) {
  if (val >= 0 && val <= 11)
    write_bit(AS3935_SREJ, val);
}

// Return the disturber mask
uint8_t getMaskDisturberInt(void) {
  return read_bit(AS3935_MASK_DIST);
}

// Set the disturber mask
void setMaskDisturberInt(bool val) {
  write_bit(AS3935_MASK_DIST, (val) ? 0x1 : 0x0);
}

// Get minimum lighting events in last 15 minutes
uint8_t getMinLightning(void) {
  return read_bit(AS3935_MIN_NUM_LIGH);
}

// Set minimum lighting events, range 0 to 3 (1, 5, 9, 16)
void setMinLighting(uint8_t val) {
  if (val >= 0 && val <= 3)
    write_bit(AS3935_MIN_NUM_LIGH, val);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

// Get the antenna tuning capacitor value
uint8_t getTune(void) {
  return read_bit(AS3935_TUN_CAP);
}

// Set the annenna tuning capacitor value manually
void setTune(uint8_t val) {
  if (val >= 0 && val <= 0x0f)
    write_bit(AS3935_TUN_CAP, val);
}

// Set the annenna tuning capacitor value automatically
// https://github.com/raivisr/AS3935-Arduino-Library/blob/master/AS3935/AS3935.cpp
// Note: changed to 1000Ms sample for a more reliable result
bool tuneAntenna(void) {
  int target = 3125, currentcount = 0, bestdiff = 32000, currdiff = 0;
  byte bestTune = 0, currTune = 0;
  unsigned long setUpTime;
  int currIrq, prevIrq;
  emitln(">Auto Tune");
  // set lco_fdiv divider to 0, which translates to 16
  // so we are looking for 31250Hz on irq pin
  // and since we are counting for 100ms that translates to number 3125
  // each capacitor changes second least significant digit
  // using this timing so this is probably the best way to go
  write_bit(AS3935_LCO_FDIV, 0);
  write_bit(AS3935_DISP_LCO, 1);
  // tuning is not linear, can't do any shortcuts here
  // going over all built-in cap values and finding the best
  for (currTune = 0; currTune <= 0x0F; currTune++) {
    emith(currTune); emit(" ");
    write_bit(AS3935_TUN_CAP, currTune);
    // let it settle
    delay(5);
    currentcount = 0;
    prevIrq = digitalRead(INT_PIN);
    setUpTime = millis() + 1000;
    while ((long)(millis() - setUpTime) < 0) {
      currIrq = digitalRead(INT_PIN);
      if (currIrq > prevIrq) {
        currentcount++;
      }
      prevIrq = currIrq;
    }
    currdiff = target - currentcount / 10;
    // don't look at me, abs() misbehaves
    if (currdiff < 0) {
      currdiff = -currdiff;
    }
    emit(currdiff); emitln(" ");
    if (bestdiff > currdiff) {
      bestdiff = currdiff;
      bestTune = currTune;
    }
  }

  write_bit(AS3935_TUN_CAP, bestTune);
  delay(2);
  write_bit(AS3935_DISP_LCO, 0);
  // and now do RCO calibration
  write_bit(AS3935_PWD, 0);
  // if error is over 109, we are outside allowed tuning range of +/-3.5%
  return bestdiff > 109 ? false : true;
}

bool begin(uint8_t val = 1) {

  bool ret;

  i2c_init();
  ret = i2c_start(AS3935_ADDRESS | I2C_WRITE);
  i2c_stop();
  if (ret) {
    emitln(">Hello MOD-1016v6!");
    presetDefault();
    // Calibrate the AS3935
    delay(11);
    tuneAntenna();
    calibrateRCO();
    if (val)
      setIndoors();
    else
      setOutdoors();
  } else {
    emitln(F(">Device didn't respond"));
  }
  return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Initialize the terminal
#ifdef YUN_CONSOLE
  Bridge.begin();
  Console.begin();
  while (!Console);
#else
  Serial.begin(115200);
  while (!Serial);
#endif
  delay(1000);
  emitln(">Hello World!");

  // MOD-1016v6
  if (!begin())
    while (1);

  //
  // Enable pin change interrupt
  // http://playground.arduino.cc/Main/PinChangeInterrupt
  //
  pinMode(INT_PIN, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  *digitalPinToPCMSK(INT_PIN) |= bit (digitalPinToPCMSKbit(INT_PIN)); // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(INT_PIN)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(INT_PIN)); // enable interrupt for the group

  dump_regs();
  emitln(">Listening");
}

void loop() {
  if (event) {
    emit(millis());
    if (int_event == 0x00) {
      emit(" INT    Distance was updated to ");
      emit(getDistance(true));
      emitln(" miles");
    }
    if (int_event == 0x01)
      emitln(" INT_NH Noise level too high");
    if (int_event == 0x04)
      emitln(" INT_D  Disturber detected");
    if (int_event == 0x08) {
      emit(" INT_L  Lightning interrupt ");
      emitln(getEnergy());
    }
    event = false;
  }
  delay(50);
}

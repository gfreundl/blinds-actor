// Arduino blinds actor
//
// drives tilt angle of standard blinds (25mm case width or larger) by a gear motor
// power is provided by 2 AAA batteries (AA type possible for larger blind cases) 
// hardware used:
//    gear motor 3V 36 rpm
//    Arduino Pro Mini 8MHz
//    CC1101 module
//    Pololu DRV8838 module
//
// hardware changes for minimum idle power:
// remove 10K from DRV, Power LED and LDO from AVR, power up DRV8838 by EN pin only
// 
// shunt resistor not smaller thah 10 Ohms to avoid overtorque with fresh batteries
// 
// tested within 1.9 and 3.2 V range with 150 mA current limit protection
// works best between 2.0 and 3.0 V
// weak batteries still need to deliver 100mA to spin up motor
//
// 020921:  set AVR BOD down to 1.8V to fully use up batteries (extended Fuse 0xFE) 
//          changed pins for compatibility with AskSinPP project
//          Iq still at about 550uA (400uA from A0/A3 pins)
// 060921:  feed DRV8838 Vcc directly from EN pin, still keep SLEEP pin for minimum quiescent current
//          for lower Iq consider disabling ADC, BOD
// 070921:  reverse a bit more on end stop and pause
//          tried to reduce quiescent current by activation via power on -> boot takes too long (several seconds)
// 090921:  Iq = 130uA achieved with Low-Power library         
// 200921:  implemented BidCos remote control with CC1101, forked from jalousie.ino
//          Imax at end stop 50mA with 6V/36rpm gear motor
// 301021:  removing step-up converter reduces Iqsc from 100uA down to 25uA
//          measuring battery as reference done by this code: https://code.google.com/archive/p/tinkerit/wikis/SecretVoltmeter.wiki
//          report version in Serial init lines
//          set PH_PIN to LOW on idle: saves 20uA quiescent current, so we are at 4uA now
//          minimum Ucc = 2.1 V

// AskSin++ by 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER
#define USE_WOR
#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>

#include <Switch.h>


// we use a Pro Mini
// Arduino pin for the LED
// D4 == PIN 4 on Pro Mini
#define LED_PIN 4
// Arduino pin for the config button
// B0 == PIN 8 on Pro Mini
#define CONFIG_BUTTON_PIN 8

#define RELAY1_PIN 5

// number of available peers per channel
#define PEERS_PER_CHANNEL 8

#define MANUAL_PIN 3
#define EN_PIN 6
#define PH_PIN 7
#define SLEEP_PIN 9

// variables for actor
float Rmotor = 10;                  // [Ohm] shunt resistor
float Iref = 0.07;                  // [A] end stop detection by max current measured at Ucc = 3.0 V 
int Imaxcount = 0;                  //counter consecutive overcurrent 
bool dir = true;                    //direction reverse flag
bool awoken_by_btn = false;
int EndStopCount = 0;                //blind hits end stop, stop after second end stop

// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
    {0x42,0xA2,0xB1},       // Device ID
    "gfe_42a2b1",           // Device Serial
    {0x00,0x6c},            // Device Model
    0x10,                   // Firmware Version
    as::DeviceType::Switch, // Device Type
    {0x01,0x00}             // Info Bytes
};

/**
 * Configure the used hardware
 */
typedef AvrSPI<10,11,12,13> RadioSPI;
typedef AskSin<StatusLed<LED_PIN>,BatterySensor,Radio<RadioSPI,2> > Hal;

DEFREGISTER(Reg0,DREG_INTKEY,DREG_LEDMODE,MASTERID_REGS,DREG_LOWBATLIMIT)
class SwList0 : public RegList0<Reg0> {
public:
  SwList0(uint16_t addr) : RegList0<Reg0>(addr) {}
  void defaults () {
    clear();
    lowBatLimit(22);
  }
};

// setup the device with channel type and number of channels
class SwitchType : public MultiChannelDevice<Hal,SwitchChannel<Hal,PEERS_PER_CHANNEL,SwList0>,1,SwList0> {
public:
  typedef MultiChannelDevice<Hal,SwitchChannel<Hal,PEERS_PER_CHANNEL,SwList0>,1,SwList0> DevType;
  SwitchType (const DeviceInfo& i,uint16_t addr) : DevType(i,addr) {}
  virtual ~SwitchType () {}

  virtual void configChanged () {
    DevType::configChanged();
    uint8_t lowbat = getList0().lowBatLimit();
    DDECLN(lowbat);
    if( lowbat > 0 ) {
      battery().low(lowbat);
    }
  }
};

Hal hal;
SwitchType sdev(devinfo,0x20);
ConfigToggleButton<SwitchType> cfgBtn(sdev);
#ifndef USE_WOR
BurstDetector<Hal> bd(hal);
#endif

void initPeerings (bool first) {
  // create internal peerings - CCU2 needs this
  if( first == true ) {
    HMID devid;
    sdev.getDeviceID(devid);
    for( uint8_t i=1; i<=sdev.channels(); ++i ) {
      Peer ipeer(devid,i);
      sdev.channel(i).peer(ipeer);
    }
  }
}

//randomize direction to prevent wear
void random_dir() {
  randomSeed(analogRead(A1));
  if (random(0,2) == 0){
    dir = false;
  }
  else {
    dir = true;
  }
}

//ISR for wake up on interrupt pin 
void wakeUp() {
  awoken_by_btn = ! awoken_by_btn;
}

long readVcc() { 
  long result; 
  // Read 1.1V reference against AVcc 
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); 
  delay(2); // Wait for Vref to settle 
  ADCSRA |= _BV(ADSC); // Convert 
  while (bit_is_set(ADCSRA,ADSC)); 
  result = ADCL; 
  result |= ADCH<<8; 
  //result = 1126400L / result; // Back-calculate AVcc in mV (1126400 = 1024*1100mV)
  result = 1121280L / result;
  return result; 
}

// run Motor
// measure current after spin up phase
// if current to high, reverse for release and pause
// pause short to reduce speed
void runMotor() {
  float speed = 1.0;
  digitalWrite(SLEEP_PIN, HIGH);
  digitalWrite(EN_PIN, HIGH);
  digitalWrite(PH_PIN, dir);
  //analogReference(INTERNAL);
  // wait until spool up before taking current measurement
  delay(50); 
  float Ucc = (float)readVcc()/1000;
  float Umotor = analogRead(A3) * Ucc / 1023;
  float Imotor = ((Ucc - Umotor) / Rmotor);     
  //correct Imax for actual battery voltage (Imax reference is Ucc = 3.0 V)
  float Imax = Ucc * Iref / 3.0;
  
  Serial.print(dir);
  Serial.print("  Umotor [V]: ");
  Serial.print(Umotor, 2);
  Serial.print("  Ucc [V]: ");
  Serial.print(Ucc, 2);
  Serial.print("  Imotor [A]: ");
  Serial.print(Imotor, 3);
  Serial.print("  Imax [A]: ");
  Serial.print(Imax, 3);
  Serial.print("  P [W]: ");
  Serial.print(Imotor * Umotor, 3);
  Serial.print("  R [Ohm]: ");
  Serial.println(Umotor / Imotor, 0);

  //detachInterrupt(digitalPinToInterrupt(interruptPin));

  //sense motor current, reverse on end stop
  if (Imotor < Imax) {
    Imaxcount = 0;
    } 
  else {
    Imaxcount++;                        //option for longer end stop testing, not used
    if (Imaxcount == 1) {
      dir = !dir;                       //reverse direction
      digitalWrite(PH_PIN, dir);        //drive back a bit to release stringstension on strings
      Imaxcount = 0;
      delay(30);    
      digitalWrite(EN_PIN, LOW);
      EndStopCount++;
    }
  } 
  delay(50 / speed);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(SLEEP_PIN, LOW);
   
}

void setup () {

  DINIT(57600,ASKSIN_PLUS_PLUS_IDENTIFIER);
  bool first = sdev.init(hal);
  sdev.channel(1).init(RELAY1_PIN);
  buttonISR(cfgBtn,CONFIG_BUTTON_PIN);
  initPeerings(first);
#ifndef USE_WOR
  // start burst detection
  bd.enable(sysclock);
#endif
  // stay on for 15 seconds after start
  hal.activity.stayAwake(seconds2ticks(10));
  // measure battery every hour
  hal.battery.init(seconds2ticks(60UL*60),sysclock);
  sdev.initDone();

  //actor setup
  pinMode(EN_PIN, OUTPUT);
  pinMode(PH_PIN, OUTPUT);
  pinMode(SLEEP_PIN, OUTPUT);
  pinMode(MANUAL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MANUAL_PIN),wakeUp, FALLING); //LOW and long press hangs CPU (LED 13 flashing 11Hz)  
  //enableInterrupt(MANUAL_PIN, wakeUp, CHANGE);                  //not working ?
  //digitalWrite(SLEEPpin, HIGH);                                 //removed, stays HIGH otherwise
  random_dir();
  Serial.println("gfe Blinds actor based on HM-LC-SW1-BA-PCB");
  Serial.println("");
  Serial.println("Version 301021");
  Serial.println("");
     
}

//loop interval: 9sec in sleep, then 2ms up, 500msec sleep on pin interrupt, 150ms on WOR
void loop() {

  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
   if ((digitalRead(RELAY1_PIN)) && (EndStopCount < 2)){
    runMotor();
   }
   else if ((awoken_by_btn) && (EndStopCount < 2)) {
    runMotor();
     //awoken_by_btn = ! awoken_by_btn;               //useful for stepping   
   }
   else {
    //reset everything before sleep
    EndStopCount = 0;
    digitalWrite(PH_PIN, 0);                          //saves power
    awoken_by_btn = false;

    if( worked == false && poll == false ) {
      hal.activity.savePower<Sleep<> >(hal);
    }
   }
 
}

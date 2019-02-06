#include <OneWire.h> 
#include <DallasTemperature.h>
#include <TheThingsNetwork.h>
#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling
#include <avr/sleep.h>          // library for sleep
#include <avr/power.h>          // library for power control
const char *appEui = "70B3D57ED0016991";
const char *appKey = "ADC0FC64BD8E8960175B349BE5CFD3AA";
    
#define loraSerial Serial1
#define debugSerial Serial

// Replace REPLACE_ME with TTN_FP_EU868 or TTN_FP_US915   
#define freqPlan TTN_FP_EU868
    
TheThingsNetwork ttn(loraSerial, debugSerial, freqPlan);                
/********************************************************************/
//Definer hvilke pins som blir brukt
#define ONE_WIRE_BUS 2 
#define turbPin A0
volatile int nbr_remaining; 
/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices  
OneWire oneWire(ONE_WIRE_BUS); 
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
/********************************************************************/ 


//*******     HENT TURBIDITET   *************
float turbiditet(){
  float rawData;
  float data;
  rawData = analogRead(turbPin);  //Leser av data fra pinnen som er definert på toppen av koden
  data = abs(1-(rawData/320));  //Behandler dataen slik at rådata fra sensor gir en turbiditetsgrad mellom 0 og 1. 
  return data;
}

float getTemperature(){
  // call sensors.requestTemperatures() to issue a global temperature 
 // request to all devices on the bus 
  Serial.print(" Requesting temperatures..."); 
  sensors.requestTemperatures(); // Send the command to get temperature readings 
  Serial.println("DONE"); 
  /********************************************************************/
  Serial.print("Temperature is: "); 
  float temp = sensors.getTempCByIndex(0);
  Serial.print(temp); // Why "byIndex"?  
  // You can have more than one DS18B20 on the same bus.  
  // 0 refers to the first IC on the wire
  return temp;

}

void setup(void) { 
   loraSerial.begin(57600);
  debugSerial.begin(9600);
    
  // Wait a maximum of 10s for Serial Monitor
  while (!debugSerial && millis() < 10000);
    
  debugSerial.println("-- STATUS");
  ttn.showStatus();
  
  debugSerial.println("-- JOIN");
  ttn.join(appEui, appKey);
  
 // start serial port 
 // Serial.begin(9600); 
 Serial.println("Dallas Temperature IC Control Library Demo"); 
 // Start up the library 
 sensors.begin();

 configure_wdt();
} 
void loop(void) { 
  sleep(2);

  //Henter temperatur
  float temp = getTemperature();
  int iTemp = (int) (temp * 100);

  //Henter turbiditet
  float turb = turbiditet();
  int iTurb = (int) (turb*100);
   
  
  byte data[3];
  
  data[0] = highByte(iTemp);
  data[1] = lowByte(iTemp);
  data[2] = iTurb;
    
  // Send it off
  ttn.sendBytes(data, sizeof(data));
}

/**********************************************************************/
/*****************     SLEEP FUNCTION    ******************************/
/**********************************************************************/

void sleep(int ncycles)
{  
  nbr_remaining = ncycles; // defines how many cycles should sleep

  // Set sleep to full power down.  Only external interrupts or
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
  // Turn off the ADC while asleep.
  power_adc_disable();
 
  while (nbr_remaining > 0){ // while some cycles left, sleep!

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point if the
  // watchdog is configured for resume rather than restart
 
  // When awake, disable sleep mode
  sleep_disable();
  
  // we have slept one time more
  nbr_remaining = nbr_remaining - 1;
 
  }
 
  // put everything on again
  power_all_enable();
 
}


/******* Dette må bare være med, **********
 ******* så ikke interrupt kaller på en ***
 *******  uinitialisert funksjon      *****
 */
ISR(WDT_vect){
        // not hanging, just waiting
        // reset the watchdog
        wdt_reset();
}


/********* Configurer wdt *********/
void configure_wdt(void)
{
 
  cli();                           // disable interrupts for changing the registers

  MCUSR = 0;                       // reset status register flags

                                   // Put timer in interrupt-only mode:                                       
  WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                   // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b100001; // set WDIE: interrupt enabled
                                   // clr WDE: reset disabled
                                   // and set delay interval (right side of bar) to 8 seconds

  sei();                           // re-enable interrupts 
}

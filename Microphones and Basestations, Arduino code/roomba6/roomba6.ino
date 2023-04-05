/*************************************************************************************
  This program is roomba6.ino. 

  Circuit is the fall 2021 Physics 398DLP group 4 roomba tracker.

  I use the online tone generator at https://www.szynalski.com/tone-generator/
  to test the circuit with a 1 kHz tone.

  I do a single-frequency Fourier transform to get amplitude and phase at 1 kHz.
  After the first time through (in which I scan in frequency), I restrict the
  Fourier traansform to 1 kHz only,

  Recall the definition of a Fourier transform. For a continuous function f(t), 
  the transform g(omega) is an integral:
  g(omega) = integral (t from -infinity to +infinity) [f(t) * exp(i*omega*t) dt].
  Also recall that exp(i*omega*t) = cos(i*omega*t) + i*sin(i*omega*t)

  George Gollin
  University of Illinois
  October 2021
*************************************************************************************/

// We want to be able to have four identical devices that interleave their 
// radio communications withg a single base station. spicy the "board ID" here
// and the incremental delay (in milliseconds) relative to another board.

// uncomment exactly ONE of these.
// #define BOARD_ID 0
#define BOARD_ID 1
// #define BOARD_ID 2
// #define BOARD_ID 3

#define RADIO_DELAY_BUMP 250
const uint32_t radio_transmit_delay = BOARD_ID * RADIO_DELAY_BUMP;

uint32_t FFTloopcounter = 0;
uint32_t tstartLoRa;

#include <SPI.h>
#include <Adafruit_Sensor.h>  
#include "Wire.h"
#include <LiquidCrystal.h>
#include "RTClib.h"
#include <Adafruit_GPS.h>
#include <string.h>
#include <SD.h>
#include <RH_RF95.h>
#include "arduinoFFT.h"
#include "Adafruit_BME680.h"

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

//////////////////////////// BME680 stuff //////////////////////

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme;

float bme_temperature;
float bme_pressure;
float bme_humidity;
float bme_gas_resistance;

//////////////////////////// SD stuff //////////////////////

// SD.h lives in arduino.app/COntents/Java/libraries/SD/src.
File myFile;

// chip select line to Adalogger's SD:
#define SD_CS 4

//////////////////////////////// LoRa radio stuff /////////////////////////////////

// "chip select" pin for the radio, used by the Adalogger to control radio
// A5: 19
#define RFM95_CS 19
// "reset" pin for the radio. Reset when this pin is pulled low. I do not use it
// so pick a non-existent pin.
#define RFM95_RST 25
// An interrupt pin: A0, pin 14.
#define RFM95_INT 14

// If you change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
// #define RF95_FREQ 868.0

// Singleton instance of the radio driver. Arguments like so:
// RH_RF95::RH_RF95(uint8_t slaveSelectPin, uint8_t interruptPin, RHGenericSPI& spi)
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// packet counter
int16_t packetnum = 0;  

// here's what we'll send via LoRa
#define LASTRADIOPACKETCHARINDEX 28


// #define RADIOPACKETLENGTH 20
// #define RADIOPACKETLENGTH 50
#define RADIOPACKETLENGTH 60
char radiopacket[RADIOPACKETLENGTH + 1];

//////////////////////////// LCD stuff //////////////////////

// initialize the LCD library by associating any needed LCD interface pins
// with the M0 pin numbers to which they are connected
const int rs = 12, en = 11, d4 = 5, d5 = 6, d6 = 9, d7 = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//////////////////////////// LED stuff //////////////////////

// There is a single external LED lit by digital pin 16 going HIGH.
int LED_pin = 16;
#define LED_OFF LOW
#define LED_ON HIGH

//////////////////////////// GPS stuff //////////////////////

// I'll use the Serial1 UART (Universal Asynchronous Receiver/Transmitter)
// port to talk to the GPS through pins 0 (M0 RX) and 1
// (M0 TX).

// declare which M0 pin sees the GPS PPS signal. 
int GPS_PPS_pin = 13;

// set the name of the hardware serial port:
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// declare variables which we'll use to store the value (0 or 1). 
int GPS_PPS_value, GPS_PPS_value_old;

// define the synch-GPS-with-PPS command. NMEA is "National Marine Electronics 
// Association." 
#define PMTK_SET_SYNC_PPS_NMEA "$PMTK255,1*2D"

// command string to set GPS NMEA baud rate to 9,600:
#define PMTK_SET_NMEA_9600 "$PMTK251,9600*17"

// define a command to disable all NMEA outputs from the GPS except the date/time
#define PMTK_DATE_TIME_ONLY "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0*29"
 
// define a command to disable ALL NMEA outputs from the GPS
#define PMTK_ALL_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
 
// define a command to enable all NMEA outputs from the GPS
#define PMTK_ALL_ON "$PMTK314,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1*29"
 
// See https://blogs.fsfe.org/t.kandler/2013/11/17/ for additional GPS definitions.

// we don't expect a valid GPS "sentence" to be longer than this...
#define GPSMAXLENGTH 120
char GPS_sentence[GPSMAXLENGTH];
int GPS_command_string_index;

// we'll also want to convert the GPS sentence character array to a string for convenience
String GPS_sentence_string;

// pointers into parts of a GPZDA GPS data sentence whose format is
//    $GPZDA,hhmmss.sss,dd,mm,yyyy,xx,xx*CS 
//              111111111122222222223
//    0123456789012345678901234567890             
// where CS is a two-character checksum. Identify this sentence by the presence of a Z.

const int GPZDA_hour_index1 = 7;
const int GPZDA_hour_index2 = GPZDA_hour_index1 + 2;

const int GPZDA_minutes_index1 = GPZDA_hour_index2;
const int GPZDA_minutes_index2 = GPZDA_minutes_index1 + 2;
      
const int GPZDA_seconds_index1 = GPZDA_minutes_index2;
const int GPZDA_seconds_index2 = GPZDA_seconds_index1 + 2;
      
const int GPZDA_milliseconds_index1 = GPZDA_seconds_index2 + 1;   // skip the decimal point
const int GPZDA_milliseconds_index2 = GPZDA_milliseconds_index1 + 3;
      
const int GPZDA_day_index1 = GPZDA_milliseconds_index2 + 1;  // skip the comma
const int GPZDA_day_index2 = GPZDA_day_index1 + 2;
      
const int GPZDA_month_index1 = GPZDA_day_index2 + 1;
const int GPZDA_month_index2 = GPZDA_month_index1 + 2;

const int GPZDA_year_index1 = GPZDA_month_index2 + 1;
const int GPZDA_year_index2 = GPZDA_year_index1 + 4;

// here is how many non-GPZDA sentences carrying fix info to receive 
// before turning the GPZDA stuff back on.
const int maximum_fixes_to_demand = 5;

//; also define some GPGGA pointers.
const int GPGGA_command_index1 = 1;
const int GPGGA_command_index2 = GPGGA_command_index1 + 4;
const int GPGGA_commas_before_fix = 6;
const int GPGGA_crash_bumper = 50;
char GPGGA_fix;
bool we_see_satellites;
int number_of_good_fixes = 0;

// define some time variables.
unsigned long  time_ms_bumped_RTC_time_ready;

// system time from millis() at which the most recent GPS date/time
// sentence was first begun to be read
unsigned long t_GPS_read_start;

// system time from millis() at which the most recent GPS date/time
// sentence was completely parsed 
unsigned long t_GPS_read;

// system time from millis() at which the proposed bumped-by-1-second
// time is ready for downloading to the RTC
unsigned long t_bump_go; 
                
// system time from millis() at which the most recent 0 -> 1 
// transition on the GPS's PPS pin is detected
unsigned long t_GPS_PPS;  

// define some of the (self-explanatory) GPS data variables. Times/dates are UTC.
String GPS_hour_string;
String GPS_minutes_string;
String GPS_seconds_string;
String GPS_milliseconds_string;
int GPS_hour;
int GPS_minutes;
int GPS_seconds;
int GPS_milliseconds;

String GPS_day_string;
String GPS_month_string;
String GPS_year_string;
int GPS_day;
int GPS_month;
int GPS_year;

// the only kind of GPS sentence that can hold a Z, that I am allowing from the GPS,
// will carry date/time information.
bool sentence_has_a_Z;
  
// a GPGGA sentence has GPS fix data.
bool sentence_is_GPGGA;

// times for the arrival of a new data sentence and the receipt of its last character
unsigned long t_new_sentence;
unsigned long t_end_of_sentence;

// a counter
int i_am_so_bored;

//////////////////////////////////// RTC stuff //////////////////////////////

// instantiate an rtc (real time clock) object:
RTC_DS3231 rtc;

// system time from millis() at which the RTC time load is done 
unsigned long t_RTC_update;

// keep track of whether or not we have set the RTC using satellite-informed GPS data
bool good_RTC_time_from_GPS_and_satellites;

// RTC variables...
int RTC_hour;
int RTC_minutes;
int RTC_seconds;
int RTC_day;
int RTC_month;
int RTC_year;

// we will use the following to update the real time clock chip.
int RTC_hour_bumped;
int RTC_minutes_bumped;
int RTC_seconds_bumped;
int RTC_day_bumped;
int RTC_month_bumped;
int RTC_year_bumped;

// define a "DateTime" object:
DateTime now;

// limits for the timing data to be good:
const int t_RTC_update__t_GPS_PPS_min = -1;
const int t_GPS_PPS___t_bump_go_min = 200;
const int t_bump_go___t_GPS_read_min = -1;
const int t_RTC_update___t_GPS_read_min = 400;

const int t_RTC_update__t_GPS_PPS_max = 20;
const int t_GPS_PPS___t_bump_go_max = 800;
const int t_bump_go___t_GPS_read_max = 350;
const int t_RTC_update___t_GPS_read_max = 1000;

// more bookkeeping on clock setting... I will want to see several consecutive
// good reads/parses of GPS system time data to declare that all is good, and that
// we can wrap this up.
int consecutive_good_sets_so_far;
bool time_to_quit;
const int thats_enough = 5;

// it's obvious what these are:
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", 
  "Thursday", "Friday", "Saturday"};

// a "function prototype" so I can put the actual function at the end of the file:
void bump_by_1_sec(void);

//////////////////////////////// FFT stuff ////////////////////////////////

// Take multiple samples to calculate mean and RMS. This no longer 
// needs to be a power of 2.
// #define AUDIOSAMPLESIZE 1024
#define AUDIOSAMPLESIZE 2000

// known frequency (Hz) of tone generator
// #define TONEGENERATORFREQ 1000.0
#define TONEGENERATORFREQ 200.0
// #define TONEGENERATORFREQ 100.0

// These are the input and output vectors
// Input vectors receive computed results from FFT
double vReal[AUDIOSAMPLESIZE];
double vImag[AUDIOSAMPLESIZE];

double samplingFrequency;


////////////////////////// ADC & microphone stuff //////////////////////////

// select the analog input pin for the microphone
const int microphonePin = A1;

// ADC value each time we read the microphone
long int adc_value;

// use in calculating average and average of squares...
long sum_amplitudes, sum_amplitudes_squared;

// baseline ADC value we'll subtract from everything to avoid overflows
// when calculating sums
long int baseline;

// other stuff
long int how_many_samples;

#define BASELINESAMPLES 10000

// how long (ms) to collect ADC measurements per RMS calculation
#define MEASUREMENT_DURATION 500

// averages
float average, average_square, RMS;

// we'll want to keep track of how long we read the ADC before reporting a 
// average values..
unsigned long time_start;
unsigned long time_stop;

// arrays for microphone ADC values
uint16_t microphone_ADC[AUDIOSAMPLESIZE];

// quantities for calculating averages
double micsum, mic2sum;
double microphone_average;
double microphone_phase;

int32_t start_time;
int32_t start_time_ms;
int32_t stop_time;

int32_t start_time_FT;

uint32_t FT_calls = 0;

////////////////////////////////////////////////////////////////////////////

void setup() 
{
  // Open serial communications and wait for port to open:
  uint32_t t1234 = millis();
  Serial.begin(115200);
  while (!Serial && millis() - t1234 < 5000) {}

  Serial.println("\n\n*************** test the Roomba tracker ***************");
  Serial.print("This device has board ID = ");
  Serial.print(BOARD_ID);
  Serial.print(" and will delay FFT data radio transmission by ");
  Serial.print(radio_transmit_delay);
  Serial.println("ms");
  Serial.print("File name holding this program:\n   ");
  Serial.println(__FILE__);
  Serial.print("Date and (LOCAL) time of program compilation: ");
  Serial.print(__DATE__);
  Serial.print("  ");
  Serial.println(__TIME__);
  
  // set the LED-illuminating pin to be a digital output.
  pinMode(LED_pin, OUTPUT);

  // make sure the LED is off.
  digitalWrite(LED_pin, LED_OFF);

  ////////// setup LCD ////////// 
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  ////////// setup I2C ////////// 
  // light up the I2C comm lines
  Wire.begin(); 

  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  //         0123456789012345
  lcd.print("Roomba tracker  ");
  lcd.setCursor(0, 1);
  lcd.print("DAQ starting up!");

  ////////// setup GPS and RTC ////////// 
  int return_code = setup_GPS_RTC();
  // now set the RTC from the GPS.
  return_code = set_RTC_with_GPS();

  ////////// setup microSD memory breakout ////////// 
  return_code = setup_SD();

  ////////// setup BME680 ////////// 
  return_code = setup_BME680();

  if (return_code >= 0) 
  {
    // Serial.println(F("Just set up BME680"));
  } else {
    Serial.print(F("Trouble setting up BME680. Return code = "));
    Serial.println(return_code);
  }

  read_BME680();

  ////////// setup LoRa radio ////////// 
  return_code = setup_LoRa();

  ////////// setup microphone ////////// 
  setup_microphone();

  ////////// do a single-frequency Fourier transform ////////// 
  do_single_frequency_FT();

  ////////// all done with setup! ////////// 

}

////////////////////////////////////////////////////////////////////////////

int setup_GPS_RTC() 
{
  // set up the GPS and RTC.

  int return_code = 0;

  // fire up the GPS.
  Serial.println("Now setting up the GPS."); 

  // declare the GPS PPS pin to be an M0 input 
  pinMode(GPS_PPS_pin, INPUT);

/*
  // remove this later ///////////////////////////////////
  // declare the GPS PPS pin to be an M0 input 
  pinMode(GPS_PPS_pin, INPUT);
  int the_pin_val_old = 9;
  while(1)
  {
    int the_pin = digitalRead(GPS_PPS_pin);
    if(the_pin != the_pin_val_old)
    {
      the_pin_val_old = the_pin;
      Serial.println(the_pin);
    }
  }
 */   
 
  // initialize a flag and some counters
  good_RTC_time_from_GPS_and_satellites = false;
  consecutive_good_sets_so_far = 0;
  i_am_so_bored = 0;

  // 9600 NMEA is the default communication and baud rate for Adafruit MTK 3339 chipset GPS 
  // units. NMEA is "National Marine Electronics Association." 
  // Note that this serial communication path is different from the one driving the serial 
  // monitor window on your laptop.
  GPS.begin(9600);

  // initialize a flag holding the GPS PPS pin status: this pin pulses positive as soon as 
  // the seconds value rolls to the next second.
  GPS_PPS_value_old = 0;
    
  // Set the update rate to once per second. 
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 

  // Send a synch-with-PPS command to the GPS in hopes of having a deterministic
  // relationship between the PPS line lighting up and the GPS reporting data to us. According
  // to the manufacturer, the GPS will start sending us a date/time data sentence about 170
  // milliseconds after the PPS line transitions fom 0 to 1. 
  GPS.sendCommand(PMTK_SET_SYNC_PPS_NMEA);

  // turn on RMC (recommended minimum) and GGA (fix data) including altitude
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // turn on all data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  
  // turn off most GPS outputs to reduce the rate of stuff coming at us.
  // GPS.sendCommand(PMTK_DATE_TIME_ONLY);

  // this keeps track of where in the string of characters of a GPS data sentence we are.
  GPS_command_string_index = 0;

  // more initialization
  sentence_has_a_Z = false;
  sentence_is_GPGGA = false;
  we_see_satellites = false;

  time_to_quit = false;

  // fire up the RTC.
  Serial.println("Now set up the RTC."); 
  return_code = rtc.begin();
  // Serial.println(rtc.begin());

  // problems?
  if(!return_code) {
    Serial.println("RTC wouldn't respond so bail out.");
    while (1) {};
  }

  // now try read back the RTC to check.       
  delay(500);
  
  now = rtc.now();
  Serial.print("RTC time and date (UTC) before GPS synch are ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  if(now.minute() < 10)   Serial.print(0);
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  if(now.second() < 10)   Serial.print(0);
  Serial.print(now.second(), DEC);

  Serial.print("   ");
  if(int(now.day()) < 10) Serial.print("0");
  Serial.print(now.day(), DEC); Serial.print('/');
  if(int(now.month()) < 10) Serial.print("0");
  Serial.print(now.month(), DEC); Serial.print("/");
  Serial.print(now.year(), DEC);
  Serial.println(" (dd/mm/yyyy)");

  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("Now looking for ");
  lcd.setCursor(0, 1);
  lcd.print("GPS satellites  ");

/*
  // print information about the types of GPS sentences.
  Serial.println("Kinds of GPS sentences");
  Serial.println("   $GPBOD - Bearing, origin to destination");
  Serial.println("   $GPBWC - Bearing and distance to waypoint, great circle");
  Serial.println("   $GPGGA - Global Positioning System Fix Data");
  Serial.println("   $GPGLL - Geographic position, latitude / longitude");
  Serial.println("   $GPGSA - GPS DOP and active satellites ");
  Serial.println("   $GPGSV - GPS Satellites in view");
  Serial.println("   $GPHDT - Heading, True");
  Serial.println("   $GPR00 - List of waypoints in currently active route");
  Serial.println("   $GPRMA - Recommended minimum specific Loran-C data");
  Serial.println("   $GPRMB - Recommended minimum navigation info");
  Serial.println("   $GPRMC - Recommended minimum specific GPS/Transit data");
  Serial.println("   $GPRTE - Routes");
  Serial.println("   $GPTRF - Transit Fix Data");
  Serial.println("   $GPSTN - Multiple Data ID");
  Serial.println("   $GPVBW - Dual Ground / Water Speed");
  Serial.println("   $GPVTG - Track made good and ground speed");
  Serial.println("   $GPWPL - Waypoint location");
  Serial.println("   $GPXTE - Cross-track error, Measured");
  Serial.println("   $GPZDA - Date & Time\n");
*/
  // all done!
  return return_code;

}

////////////////////////////////////////////////////////////////////////////

int set_RTC_with_GPS()
{
  // set the RTC with the GPS.

  int return_code = 0;
  bool debug_printing = false;

  // ******************************************************************************
  /*

  First things first: check to see if we are we done setting the RTC. In order to 
  declare victory and exit, we'll need the following to happen. 

  Definitions:

    t_GPS_read    system time from millis() at which the most recent GPS date/time
                  sentence was completely parsed BEFORE the most recent PPS 0 -> 1 
                  transition was detected 
                      
    t_bump_go     system time from millis() at which the proposed bumped-by-1-second
                  time is ready for downloading to the RTC
    
    t_GPS_PPS     system time from millis() at which the most recent 0 -> 1 
                  transition on the GPS's PPS pin is detected

    t_RTC_update  system time from millis() at which the RTC time load is done 

  Typical timing for an event:   

    t_GPS_read    17,961    
    t_bump_go     17,971 (t_GPS_read +  10 ms)    
    t_GPS_PPS     18,597 (t_bump_go  + 626 ms)    
    t_RTC_update  18,598 (t_GPS_PPS  +   1 ms)

  Every once in a while we might miss the PPS 0 -> 1 transition, or the GPS might 
  not feed us a data sentence. So let's impose the following criteria.

  0 ms   <= t_RTC_update - t_GPS_PPS  <= 10 ms
  200 ms <= t_GPS_PPS - t_bump_go     <= 800 ms
  0 ms   <= t_bump_go - t_GPS_read    <= 50 ms
  400 ms <= t_RTC_update - t_GPS_read <= 1000 ms

  */
  Serial.println("Start synchronizing RTC with GPS. This may take 10 - 120 seconds.");

  // we need to loop here, so we can read GPS data, one character per pass through this loop.

  while(1)
  {
    if(time_to_quit) 
    {
      // print a message to the serial monitor, but only once.
      if (i_am_so_bored == 0) Serial.println("We are finished setting the RTC to GPS time.");

      // Print a message to the LCD each pass through, updating the time.
      lcd.setCursor(0, 0);
      //         0123456789012345
      lcd.print("RTC is now set  ");

      // blank the LCD's second line 
      lcd.setCursor(0, 1);
      lcd.print("                ");

      // print the time
      lcd.setCursor(0, 1);
      now = rtc.now();
      
      if(now.hour() < 10)   lcd.print(0);
      lcd.print(now.hour(), DEC);
      
      lcd.print(':');
      if(now.minute() < 10)   lcd.print(0);
      lcd.print(now.minute());
      
      lcd.print(':');
      if(now.second() < 10)   lcd.print(0);
      lcd.print(now.second());

      delay(50);

      // increment a counter
      i_am_so_bored++;

      return_code = 0;
      return return_code;
    }

    // *******************************************************************************

    // now check to see if we just got a PPS 0 -> 1 transition, indicating that the
    // GPS clock has just ticked over to the next second.

    GPS_PPS_value = digitalRead(GPS_PPS_pin);
    
    // did we just get a 0 -> 1 transition?
    if (GPS_PPS_value == 1 && GPS_PPS_value_old == 0) 
    {  
      // Serial.print("\nJust saw a PPS 0 -> 1 transition at time (ms) = ");
      t_GPS_PPS = millis();
      // Serial.println(t_GPS_PPS);

      // load the previously established time values into the RTC now.
      if (good_RTC_time_from_GPS_and_satellites) {

        // now set the real time clock to the bumped-by-one-second value that we have 
        // already calculated. To set the RTC with an explicit date & time, for example 
        // January 21, 2014 at 3am you would call
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
      
        rtc.adjust(DateTime(int(RTC_year_bumped), int(RTC_month_bumped), int(RTC_day_bumped), int(RTC_hour_bumped), 
          int(RTC_minutes_bumped), int(RTC_seconds_bumped)));

        // take note of when we're back from setting the real time clock:
        t_RTC_update = millis();

        // Serial.print("Just returned from updating RTC at system t = "); Serial.println(t_RTC_update);
        if(debug_printing)
        {
          Serial.print("Proposed new time fed to the RTC was ");
          Serial.print(RTC_hour_bumped, DEC); Serial.print(':');
          if(RTC_minutes_bumped < 10) Serial.print("0");
          Serial.print(RTC_minutes_bumped, DEC); Serial.print(':');
          if(RTC_seconds_bumped < 10) Serial.print("0");
          Serial.print(RTC_seconds_bumped, DEC); 
          Serial.print("   Date (dd/mm/yyyy): ");
          if(int(RTC_day_bumped) < 10) Serial.print("0");
          Serial.print(RTC_day_bumped, DEC); Serial.print('/');
          if(RTC_month_bumped < 10) Serial.print("0");
          Serial.print(RTC_month_bumped, DEC); Serial.print("/");
          Serial.println(RTC_year_bumped, DEC);  

          // now read back the RTC to check.       
          now = rtc.now();
          Serial.print("Now read back the RTC to check. ");
          Serial.print(now.hour(), DEC);
          Serial.print(':');
          if(now.minute() < 10)   Serial.print(0);
          Serial.print(now.minute(), DEC);
          Serial.print(':');
          if(now.second() < 10)   Serial.print(0);
          Serial.print(now.second(), DEC);

          Serial.print("   Date (dd/mm/yyyy): ");
          if(int(now.day()) < 10) Serial.print("0");      
          Serial.print(now.day(), DEC); Serial.print('/');
          if(int(now.month()) < 10) Serial.print("0");
          Serial.print(now.month(), DEC); Serial.print("/");
          Serial.println(now.year(), DEC);
        }
        
        // now that we've used this GPS value, set the following flag to false:
        good_RTC_time_from_GPS_and_satellites = false;
        consecutive_good_sets_so_far++;
        time_to_quit = consecutive_good_sets_so_far >= thats_enough;
      }
    }

    GPS_PPS_value_old = GPS_PPS_value;

    // *******************************************************************************
    // read data from the GPS; do this one character per pass through function loop.
    // when synched to the PPS pin, the GPS sentence will start arriving about 170 ms
    // after the PPS line goes high, according to the manufacturer of the MTK3339 GPS
    // chipset. So we need to start by seeing if there's been a PPS 0 -> 1 transition.
    // *******************************************************************************

    char c;

    // is there anything new to be read?

    if(GPSSerial.available()) {

      // read the character
      c = GPS.read();

      // a "$" indicates the start of a new sentence.
      if (c == '$') 
      {
        //reset the array index indicating where we put the characters as we build the GPS sentence.
        GPS_command_string_index = 0;
        t_new_sentence = millis();
        sentence_has_a_Z = false;
        sentence_is_GPGGA = false;

      } else {
      GPS_command_string_index++;
    }

      // build up the data sentence, one character at a time.
      GPS_sentence[GPS_command_string_index] = c;

      // are we reading a sentence from the GPS that carries date/time information? The
      // format is this: 
      //    $GPZDA,hhmmss.sss,dd,mm,yyyy,xx,xx*CS 
      // where CS is a checksum. Identify this kind of sentence by the presence of a Z.

      if (c == 'Z') 
      {
        sentence_has_a_Z = true;
      }
      
      // a "*" indicates the end of the sentence, except for the two-digit checksum and the CR/LF.
      if (c == '*') 
      {
        t_end_of_sentence = millis();
        t_GPS_read = t_end_of_sentence;
        // Serial.print("Beginning, end of reception of latest GPS sentence: "); Serial.print(t_new_sentence);
        // Serial.print(", "); Serial.println(t_end_of_sentence);

        // convert GPS data sentence from a character array to a string.
        GPS_sentence_string = String(GPS_sentence);

        // print the GPS sentence
        if(debug_printing)
        {
          Serial.print("New GPS_sentence_string is "); 
          Serial.println(GPS_sentence_string.substring(0, GPS_command_string_index+1));
        }
        // now parse the string if it corresponds to a date/time message.
        if (sentence_has_a_Z) 
        {
          GPS_hour_string = GPS_sentence_string.substring(GPZDA_hour_index1, GPZDA_hour_index2);
          GPS_minutes_string = GPS_sentence_string.substring(GPZDA_minutes_index1, GPZDA_minutes_index2);
          GPS_seconds_string = GPS_sentence_string.substring(GPZDA_seconds_index1, GPZDA_seconds_index2);
          GPS_milliseconds_string = GPS_sentence_string.substring(GPZDA_milliseconds_index1, GPZDA_milliseconds_index2);
          GPS_day_string = GPS_sentence_string.substring(GPZDA_day_index1, GPZDA_day_index2);
          GPS_month_string = GPS_sentence_string.substring(GPZDA_month_index1, GPZDA_month_index2);
          GPS_year_string = GPS_sentence_string.substring(GPZDA_year_index1, GPZDA_year_index2);
    
          if(debug_printing)
          {
            Serial.print("GPS time (UTC) in this sentence is " + GPS_hour_string + ":" + GPS_minutes_string + ":" + 
            GPS_seconds_string + "." + GPS_milliseconds_string);
            Serial.println("      dd/mm/yyyy = " + GPS_day_string + "/" + GPS_month_string + "/" + GPS_year_string);
          }
    
          // now convert to integers
          GPS_hour = GPS_hour_string.toInt();
          GPS_minutes = GPS_minutes_string.toInt();
          GPS_seconds = GPS_seconds_string.toInt();
          GPS_milliseconds = GPS_milliseconds_string.toInt();
          GPS_day = GPS_day_string.toInt();
          GPS_month = GPS_month_string.toInt();
          GPS_year = GPS_year_string.toInt();
    
          // now set the RTC variables.
          RTC_hour = GPS_hour;
          RTC_minutes = GPS_minutes;
          RTC_seconds = GPS_seconds;
          RTC_day = GPS_day;
          RTC_month = GPS_month;
          RTC_year = GPS_year;
    
          // now try bumping everything by 1 second.
          bump_by_1_sec();
    
          t_bump_go = millis();
    
          // set a flag saying that we have a good proposed time to load into the RTC. We
          // will load this the next time we see a PPS 0 -> 1 transition.
          good_RTC_time_from_GPS_and_satellites = true;
          
        }

        // now parse the string if it corresponds to a GPGGA "fix data" message.
        // here is a typical GPGGA sentence:
        //    $GPGGA,165838.000,4006.9608,N,08815.4365,W,1,07,0.99,252.1,M,-33.9,M,,*
        // note the fix quality immediately after the comma that follows the "W" or "E" in
        // the longitude data. For the fix data: 1 or 2 are both good.

        // skip a GPZDA sentence, if that's what we have.
        if (!sentence_has_a_Z) 
        {
          // now check to see if characters 1 - 5 are "GPGGA"
          sentence_is_GPGGA = true;
          char testit[] = "$GPGGA";

          // read one character at a time.
          for(int ijk = GPGGA_command_index1; ijk <= GPGGA_command_index2; ijk++)
          {
            if(GPS_sentence_string[ijk] != testit[ijk]) {sentence_is_GPGGA = false;}
          }

          // if sentence_is_GPGGA is still true, we are in luck.

          if(sentence_is_GPGGA)
          {
            // look for the "W" oir "E" now. set GPGGA_fix to 0 in case we don't find anything good.
            GPGGA_fix = '0';

            for(int ijk = 0; ijk <= GPGGA_crash_bumper; ijk++)
            {
              if(GPS_sentence_string[ijk] == 'W' || GPS_sentence_string[ijk] == 'E')
              {
                // now wwe know where the fix datum resides, so break out.
                GPGGA_fix = GPS_sentence_string[ijk + 2];
                break;
              }
            }

            // is there a 1 or 2 there? Note that they are characters, not integers.
            we_see_satellites = (GPGGA_fix == '1' || GPGGA_fix == '2');

            // Serial.print("  Fix parameter is ");
            // Serial.print(GPGGA_fix);

            // Serial.print(". we_see_satellites is ");
            // Serial.print(we_see_satellites);

            if(we_see_satellites) 
            {
              number_of_good_fixes++;

              // print the good news the first time we have good satellite data.
              if(number_of_good_fixes == 1)
              {
                Serial.println("We have acquired signals from several GPS satellites.");
              }
              // Serial.print(". number_of_good_fixes is now ");
              // Serial.print(number_of_good_fixes);

              // Print a message to the LCD.
              lcd.setCursor(0, 0);
              lcd.print("Now have a GPS  ");
              lcd.setCursor(0, 1);
              lcd.print("satellite fix!  ");

            }

            // Serial.println(" ");

            // is it time to turn back on the GPZDA sentences?
            if(number_of_good_fixes >= maximum_fixes_to_demand)
            {
              GPS.sendCommand(PMTK_DATE_TIME_ONLY);
              // Serial.println("\nNow turning on the GPZDA date/time records from GPS\n");
            }

          } 

        }
      }
    }  
  }
}
////////////////////////////////////////////////////////////////////////////

void loop() 
{
  // now select ports and read data.

  uint32_t tstart = millis();
  bool print_stuff = false;

  // turn on the LED.
  digitalWrite(LED_pin, LED_ON);

  if(print_stuff)
  {
    Serial.println(F("----====<<<< >>>>====----"));
    print_RTC();
     
    // Print a message to the LCD.
    lcd.setCursor(0, 0);
    //         0123456789012345
    lcd.print("inside loop now ");
    lcd.setCursor(0, 1);
    lcd.print("radio test...   ");
  }

  // turn the LED off.
  digitalWrite(LED_pin, LED_OFF);

  // send a radio message now, loading the radiopacket array before sending the
  // message. Note that the string must be 50 characters long, or less.

  //                             11111111112222222222333333333344444444445555555555
  //                   012345678901234567890123456789012345678901234567890123456789
  // strcpy(radiopacket, "Roomba wrangler test transmission                           ");
  // send_LoRa();

  if(print_stuff)
  {
    // also measure the microphone RMS. First turn the LED on.
    digitalWrite(LED_pin, LED_ON);

    float myRMS = measure_microphone_RMS();

    Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"); 
    Serial.print("%%%%%% ADC/microphone RMS = "); 
    Serial.print(myRMS); 
    Serial.println(" %%%%%%"); 
    Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"); 

    lcd.setCursor(0, 0);
    //         0123456789012345
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print("Samples: ");
    lcd.print(how_many_samples);
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("mic RMS: ");
    lcd.print(myRMS);
  }

  int the_pin_val_old = 9;
  bool FFT_is_armed = false;
  while(1)
  { 
    int the_pin = digitalRead(GPS_PPS_pin);
    if(the_pin != the_pin_val_old)
    {
      // here when PPS pin changes. do something when it just became 1.
      the_pin_val_old = the_pin;

      if((the_pin == 1) && FFT_is_armed)
      {
        Serial.println("In loop: PPS just did 0 => 1 with FFT trigger armed.");

        ////////// do a single-frequency Fourier transform ////////// 
        do_single_frequency_FT();

        FFT_is_armed= false;
        break;

      } else {

        // just got a 1 => 0 transition. Take note of the time, and only allow 
        // the next pps 0 => 1 transition to launch an FFT when the time (after PPS 
        // pulse drops) is an odd number of seconds. 
        now = rtc.now();
        if(now.second() % 2 == 1)
        {
        FFT_is_armed= true;
        // Serial.println("Just armed the FFT trigger.");
        }

      }

/*

      // Serial.println("Now wait for PPS pulse to go from low to high, then record (and display) // 100 samples."); 

      // read, then possibly display, 100 values, as soon as PPS lights up.
      long int ADC_vals[100];
      if(the_pin == 1)
      {
        for(int meep = 0; meep < 100; meep++)
        {
          ADC_vals[meep] = analogRead(microphonePin);
        }

        now = rtc.now();

        Serial.print("***** PPS just lit up, so we have read the ADC 100 times. Current time is ");
        Serial.print(now.hour(), DEC);
        Serial.print(':');
        if(now.minute() < 10)   Serial.print(0);
        Serial.print(now.minute(), DEC);
        Serial.print(':');
        if(now.second() < 10)   Serial.print(0);
        Serial.print(now.second(), DEC);
        Serial.println(" *****");

        uint32_t sum_ADC_vals = 0;
        float(ADC_vals_avg);
        for(int moop = 0; moop < 100; moop++)
        {
          Serial.print(moop); Serial.print(" = ");
          Serial.print(ADC_vals[moop]); Serial.print("  ");
          if((moop + 1) % 5 == 0) {Serial.println(" ");}

          sum_ADC_vals = sum_ADC_vals + ADC_vals[moop];
        }
        ADC_vals_avg = float(sum_ADC_vals) / 100;
        the_pin_val_old = the_pin;
        Serial.print("Average = "); Serial.println(ADC_vals_avg);

        Serial.println("Mean-subtracted ADC values ");
        for(int moop = 0; moop < 100; moop++)
        {
          Serial.print(moop); Serial.print(" = ");
          Serial.print(float(ADC_vals[moop]) - ADC_vals_avg); Serial.print("  ");
          if((moop + 1) % 5 == 0) {Serial.println(" ");}
        }
        Serial.println(" ");

        break;
      }

*/
    }
  }
}

////////////////////////////////////////////////////////////////////////////

void bump_by_1_sec(){

  // bump the RTC clock time by 1 second relative to the GPS value reported 
  // a few hundred milliseconds ago. I am using global variables for the ease
  // of doing this. Note that we're going to need to handle roll-overs from 59 
  // seconds to 0, and so forth.

    bool bump_flag;
    int place_holder;

    bool debug_echo = false;

    RTC_seconds_bumped = RTC_seconds + 1;

    // use "place_holder" this way so the timings through the two branches of the if blocks 
    // are the same
    place_holder = RTC_seconds + 1;
    
    if(int(RTC_seconds_bumped) >= 60) {
      bump_flag = true;
      RTC_seconds_bumped = 0;
      }else{
      bump_flag = false;
      RTC_seconds_bumped = place_holder;
      }
      
    place_holder = RTC_minutes + 1;
    
    // do we also need to bump the minutes?  
    if (bump_flag) {
      RTC_minutes_bumped = place_holder;
      }else{
      RTC_minutes_bumped = RTC_minutes;
      }

    // again, do this to equalize the time through the two branches of the if block
    place_holder = RTC_minutes_bumped;
    
    if(int(RTC_minutes_bumped) >= 60) {
      bump_flag = true;
      RTC_minutes_bumped = 0;
      }else{
      bump_flag = false;
      RTC_minutes_bumped = place_holder;
      }

    place_holder = RTC_hour + 1;
    
    // do we also need to bump the hours?  
    if (bump_flag) {
      RTC_hour_bumped = place_holder;
      }else{
      RTC_hour_bumped = RTC_hour;
      }

    place_holder = RTC_hour;

    if(int(RTC_hour_bumped) >= 24) {
      bump_flag = true;
      RTC_hour_bumped = 0;
      }else{
      bump_flag = false;
      RTC_hour_bumped = place_holder;
      }

    place_holder = RTC_day + 1;
    
    // do we also need to bump the days?  
    if (bump_flag) {
      RTC_day_bumped = place_holder;
      }else{
      RTC_day_bumped = RTC_day;
      }

    // do we need to bump the month too? Note the stuff I do to make both paths
    // through the if blocks take the same amount of execution time.
    
    int nobody_home;
    int days_in_month = 31;

    // 30 days hath September, April, June, and November...
    if (int(RTC_month) == 9 || int(RTC_month) == 4 || int(RTC_month) == 6 || int(RTC_month) == 11) {
      days_in_month = 30;
    }else{
      nobody_home = 99;
    }
      
    // ...all the rest have 31, except February...
    if (int(RTC_month) == 2 && (int(RTC_year) % 4)) {
      days_in_month = 28;
    }else{
      nobody_home = 99;
    }
    
    // ...leap year!
    if (int(RTC_month) == 2 && !(int(RTC_year) % 4)) {
      days_in_month = 29;
    }else{
      nobody_home = 99;
    }

    place_holder = RTC_day_bumped;
    
    if(int(RTC_day_bumped) > days_in_month) {
      bump_flag = true;
      RTC_day_bumped = 1;
      }else{
      bump_flag = false;
      RTC_day_bumped = place_holder;
      }

    if (bump_flag) {
      RTC_month_bumped = RTC_month + 1;
      }else{
      RTC_month_bumped = RTC_month;
      }

    place_holder = RTC_month_bumped;
              
    //... and also bump the year?
    
    if(int(RTC_month_bumped) > 12) {
      bump_flag = true;
      RTC_month_bumped = 1;
      }else{
      bump_flag = false;
      RTC_month_bumped = place_holder;
      }

    if (bump_flag) {
      RTC_year_bumped = RTC_year + 1;
      }else{
      RTC_year_bumped = RTC_year;
      }

    // keep track of when we have the proposed RTC time value ready for loading
    time_ms_bumped_RTC_time_ready = millis();

    if (debug_echo) {
      // now print the newly bumped time:
      Serial.print("Now have a proposed (1 second bumped) time ready at (ms) ");
      Serial.println(time_ms_bumped_RTC_time_ready, DEC);       
      Serial.print("Proposed (1 second bumped) time: ");
      Serial.print(RTC_hour_bumped, DEC); Serial.print(':');
      if(RTC_minutes_bumped < 10) Serial.print("0");
      Serial.print(RTC_minutes_bumped, DEC); Serial.print(':');
      if(RTC_seconds_bumped < 10) Serial.print("0");
      Serial.print(RTC_seconds_bumped, DEC); 
      Serial.print("   Date (dd/mm/yyyy): ");
      if(RTC_day_bumped < 10) Serial.print("0");      
      Serial.print(RTC_day_bumped, DEC); Serial.print('/');
      if(RTC_month_bumped < 10) Serial.print("0");
      Serial.print(RTC_month_bumped, DEC); Serial.print("/");
      Serial.println(RTC_year_bumped, DEC);
    }
  
}    

/////////////////////////////////////////////////////////////////////////

int setup_SD()
{
  int return_code = 0;

  // because the radio also uses SPI I need to pull
  // the radio's chip select line HIGH to disable the radio for the moment.
  Serial.print("Now set up (and write/read test) the SD... ");

  // RFM95_RST is used by the processor as a reset pin for the radio.
  // I think it's actually a reset-bar line: reset when pulled low.
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // make very sure the reset line has time to settle.
  delay(100);

  // manual reset, with 100 millisecond dwell times in each state
  digitalWrite(RFM95_RST, LOW);
  delay(100);

  digitalWrite(RFM95_RST, HIGH);
  delay(100);

  pinMode(RFM95_CS, OUTPUT);
  // disable the radio for the moment by jammming the chip select line high.
  digitalWrite(RFM95_CS, HIGH);

  // Serial.println("Initializing SD card...");

  if (!SD.begin(SD_CS)) {
    Serial.println("SD initialization failed!");
    return_code = -1;
    return return_code;
  }

  // Serial.println("SD initialization done. Now open test.txt");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);

  uint32_t millis_start = millis();

  // if the file opened okay, write to it:
  if (myFile) 
  {
    // Serial.print("Writing to test.txt...");

    myFile.println("************ testing 1, 2, 3 ************");
    for(int ijk = 0; ijk < 5; ijk++)
    {
      myFile.print("   current millis() and micros() values are  ");
      myFile.print(millis());
      myFile.print("  ");
      myFile.println(micros());
    }
    myFile.println("************ all done! ************");

    // close the file:
    myFile.close();
    // Serial.println("done: just closed file.\n");

  } else {

    // if the file didn't open, print an error:
    Serial.println("error opening test.txt\n");
    return_code = -2;
    return return_code;
  }

  // re-open the file for reading
  // Serial.println("Now open test.txt, read it, and echo it to the serial monitor.");
  // Serial.print("Initial millis()n value was ");
  // Serial.println(millis_start);

  myFile = SD.open("test.txt");

  if (myFile) 
  {
    // Serial.println("test.txt contents:");

/*
    // read from the file until there's nothing else in it:
    while (myFile.available()) 
    {
      Serial.write(myFile.read());
    }
*/
    // close the file:
    myFile.close();
    Serial.println(" We are finished testing the SD.");
  } else {

    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
    return_code = -3;
    return return_code;
  }

  return return_code;
}

/////////////////////////////////////////////////////////////////////////

int setup_LoRa()
{
  // set up the radio.

  int return_code = 0;

  // set the control pins' modes
  pinMode(RFM95_RST, OUTPUT);
  pinMode(RFM95_CS, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // let's make sure the SD isn't getting in our way.
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  // manual reset: pull LOW to reset, put HIGH to allow LoRa to  function. 
  // Since the group 1 PCB doesn't connect the RST imput, take this into account.

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    return_code = -1;
    return return_code;
  }

  Serial.print("LoRa radio init OK... ");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  // but we want to use 915 MHz...
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    return_code = -2;
    return return_code;
  }
  Serial.print("Just set Freq to "); Serial.print(RF95_FREQ);
  
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  Serial.println("; Just set LoRa power. Do a test transmission now.");

  // now load the radiopacket array before sending a LoRa message. Note that the 
  // string must be 50 characters long, or less.
  //                             1111111111222222222233333333334444444444
  //                   01234567890123456789012345678901234567890123456789
  strcpy(radiopacket, "Roomba tracker module, George Gollin, UIUC");
  return_code = send_LoRa();

  return return_code;
}

/////////////////////////////////////////////////////////////////////////

int send_LoRa()
{
  // send a LoRa radio message of up to 60 characters.

  // contents of the character array radiopacket (a global) will be transmitted,
  // which should be loaded before the call to send_LoRa.

  int return_code = 0;
  bool debug_print = true;

  // let's make sure the SD isn't getting in our way.
  digitalWrite(SD_CS, HIGH);

  if(debug_print) {Serial.println("\nRadio Feather M0 sending to rf95 listener");}

  // Send a message to rf95_server
  int ijk = RADIOPACKETLENGTH - 7;
  int lmn = strlen(radiopacket);
  if(lmn < ijk)
  {
    radiopacket[lmn] = ' ';
    itoa(packetnum++, radiopacket + lmn, 10);
  }

  if(debug_print) {  
    // itoa(packetnum++, radiopacket + ijk, 10);
    Serial.print("Sending the radio string  "); 
    Serial.print("'");
    Serial.print(radiopacket);
    Serial.println("'");
  }
  radiopacket[RADIOPACKETLENGTH - 1] = 0;

  lcd.setCursor(0, 0);
  lcd.print("Sending packet  ");
  lcd.setCursor(0, 1);
  lcd.print("to base station ");
  
  tstartLoRa = millis();

  // rf95.send((uint8_t *)radiopacket, RADIOPACKETLENGTH);
  // rf95.send((uint8_t *)radiopacket, LASTRADIOPACKETCHARINDEX + 1);
  rf95.send((uint8_t *)radiopacket, LASTRADIOPACKETCHARINDEX);
  
  if(debug_print) {Serial.println("Waiting for packet to complete...");}
  rf95.waitPacketSent();

  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  if(debug_print) {Serial.println("Packet sent. Wait for reply..."); }

  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  //         0123456789012345
  lcd.print("Packet sent     ");

  ////////////// receive a reply-to message //////////////

  // this is a workaround for an acknowledgement since waitAvailableTimeout
  // does not work reliably for me.

  uint32_t millis_now = millis();
  uint32_t time_out = 5000;

  while(!rf95.available() && (millis() - millis_now < time_out)){}

  if (rf95.available())
  {
    // Should be a reply message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED_pin, HIGH);
      if(debug_print) {
        RH_RF95::printBuffer("Roomba tracker has received... a reply: ", buf, len);
        Serial.print("That message, as characters, is '");
        Serial.print((char*)buf);
        Serial.println("'");
        Serial.print("RSSI (signal strength, in dB): ");
        Serial.println(rf95.lastRssi(), DEC);
      }
      Serial.print("Time spent waiting for LoRa acknowledgment: ");
      Serial.print(millis() - millis_now);
      Serial.println(" ms. Acknowledgement OK!");

      lcd.setCursor(0, 0);
      lcd.print("We have received");
      lcd.setCursor(0, 1);
      //         0123456789012345
      lcd.print("a reply.        ");
      lcd.setCursor(9, 1);
      lcd.print(packetnum);
    }
    else
    {
      Serial.println("Receive (as a reply-to) failed");
    }
  } else {
    Serial.println("Timed out waiting for reply.");
  }
  return return_code;
}

/////////////////////////////////////////////////////////////////////////

void print_RTC()
{
  // print the time, according to the DS3231 real time clock.

  now = rtc.now();
  Serial.print("Current time: ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  if(now.minute() < 10)   Serial.print(0);
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  if(now.second() < 10)   Serial.print(0);
  Serial.print(now.second(), DEC);

  Serial.print(" (UTC: CDT + 5, CST + 6).  Date (dd/mm/yyyy): ");
  if(int(now.day()) < 10) Serial.print("0");      
  Serial.print(now.day(), DEC); Serial.print('/');
  if(int(now.month()) < 10) Serial.print("0");
  Serial.print(now.month(), DEC); Serial.print("/");
  Serial.println(now.year(), DEC);
}

/////////////////////////////////////////////////////////////////////////

void setup_microphone()
{
  // set up the microphone and ADC.

  how_many_samples = 0;
  sum_amplitudes = 0;
  sum_amplitudes_squared = 0;

  // The ADC resolution defaults to 10 bits for backwards compatibility, but the native
  // resolution is actually 12 bits. So set it here:
  Serial.print("Now setting up ADC and microphone... ");
  Serial.println("Setting ADC resolution to 12 bits.");
  analogReadResolution(12);

  // read the ADC repeatedly to determine a baseline value that we'll
  // subtract from everything.

  long baseline_sum = 0;
  Serial.print("Now measure ADC baseline with ");
  Serial.print(BASELINESAMPLES);
  Serial.print(" samples. ");
  
  // take note of the time at which we are ready to start recording
  time_start = millis();
  for (int index = 0; index < BASELINESAMPLES; index++) 
    {baseline_sum = baseline_sum + analogRead(microphonePin);};
  time_stop = millis();

  Serial.print("Measurement took ");
  Serial.print(time_stop - time_start);
  Serial.print(" ms. ");

  baseline = int(baseline_sum / BASELINESAMPLES);
  Serial.print("Baseline: "); Serial.print(baseline);
  Serial.println(" counts.");

  lcd.setCursor(0, 0);
  //         0123456789012345
  lcd.print("ADC baseline    ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(baseline);

  float myRMS = measure_microphone_RMS(); 
  Serial.print("Now measure ADC/microphone RMS signal (counts): "); 
  Serial.println(myRMS); 

  Serial.print(">>>>> Pausing for 10 seconds. Please set the tone generator to ");
  Serial.print(TONEGENERATORFREQ);
  Serial.print("  Hz.");

  delay(10000);
  Serial.println("Ready... go!");

}

/////////////////////////////////////////////////////////////////////////

float measure_microphone_RMS()
{
  // measure the RMS signal and noise from the microphone and ADC.

  // flag for diagnostic printing
  bool print_stuff = false;

  // initialize stuff:
  sum_amplitudes = 0;
  sum_amplitudes_squared = 0;
  how_many_samples = 0;
  time_start = millis();

  while(millis() - time_start < MEASUREMENT_DURATION) 
  {  
    adc_value = analogRead(microphonePin);
    how_many_samples++;
  
    // now add this into the various sums, subtracting the baseline.
    sum_amplitudes = sum_amplitudes + adc_value - baseline;
    sum_amplitudes_squared = sum_amplitudes_squared + 
      (adc_value - baseline) * (adc_value - baseline);
  }

  // time to calculate.
  average = float(sum_amplitudes) / float(how_many_samples);
  average_square = float(sum_amplitudes_squared) / float(how_many_samples);

  float mean_sq_deviation = average_square - average*average;
  RMS = sqrt(mean_sq_deviation);

  if(print_stuff)
  {
    Serial.print("ADC/microphone average after subtracting baseline (counts) = "); 
    Serial.println(average); 
    Serial.print("average square= "); Serial.println(average_square); 
    Serial.print("sum_amplitudes = "); Serial.println(sum_amplitudes); 
    Serial.print("sum_amplitudes_squared = "); Serial.println(sum_amplitudes_squared); 
    Serial.print("ADC/microphone RMS = "); Serial.println(RMS); 
    Serial.print("baseline = "); Serial.println(baseline); 
    Serial.print("number of samples in RMS measurement = "); Serial.println(how_many_samples);
    Serial.println(" ");

    lcd.setCursor(0, 0);
    //         0123456789012345
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print("Samples: ");
    lcd.print(how_many_samples);
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("RMS: ");
    lcd.print(RMS);
  }
  return RMS;
}

//fn//////////////////////////////////////////////////////////////////////////////
// do_single_frequency_FT()
//////////////////////////////////////////////////////////////////////////////////

float do_single_frequency_FT()
{
  // calculate the Fourier transform at a single frequency. 

  start_time = micros();
  for (uint16_t i = 0; i < AUDIOSAMPLESIZE; i++)
  {
    microphone_ADC[i] = analogRead(microphonePin);
  }
  stop_time = micros();

  start_time_FT = start_time;

  // record RTC time.
  now = rtc.now();

  // bump a counter
  FT_calls++;

  // also initialize stuff for determination of maxes.
  double largest_magnitude = 0;
  double best_phase;
  double best_frequency;
  double the_magnitude;
  double the_phase;

  // now calculate the sampling frequency...
  samplingFrequency = 1000000. * float(AUDIOSAMPLESIZE) / float(stop_time - start_time);

  Serial.println("*************************************");

  bool print_lots_of_stuff = true;

  if(print_lots_of_stuff)
  {
    Serial.print("Now do single frequency FT(s). Time to fill the array is ");
    Serial.print(stop_time - start_time);
    Serial.print(" usec. Sample rate is ");
    Serial.println(samplingFrequency);
  }

  // now calculate mean.
  micsum = 0.;

  for (uint16_t i = 0; i < AUDIOSAMPLESIZE; i++)
  {
   micsum = micsum + microphone_ADC[i];
  }

  microphone_average = micsum / double(AUDIOSAMPLESIZE);

  // Serial.print("microphone mean ADC signal: ");
  // Serial.println(microphone_average);

  for (uint16_t i = 0; i < AUDIOSAMPLESIZE; i++)
  {
    vReal[i] = (double(microphone_ADC[i]) - microphone_average);
  }
  
  // now prepare to do a Fourier transform.

  // time between successive ADC reads:
  double dt = 1. / samplingFrequency;

  // will will start at t = 0; use a trig identity to calculate cosines and sines
  // at later times, namely
  //      cos(a + b) = cos(a)cos(b) - sin(a)sin(b)
  //      sin(a + b) = sin(a)cos(b) + cos(a)sin(b)
  // a will be omega * t, and b will be omega * dt. This way I do not have to ask
  // the processor to do time-consuming sine, cosine function calls.

  // some initial values:
  double cos_a_updated;
  double sin_a_updated;

  // Let's wrap a loop around this part to vary tone_generator_freq, but only
  // the first time through.

  int freq_span;
  
  if(FT_calls == 1)
  {
    freq_span = 10;
  } else {
    freq_span = 0;
  }

  for(int freq_bump = -freq_span; freq_bump <= freq_span; freq_bump = freq_bump + 1)
  {
    start_time = micros();

    now = rtc.now();
    Serial.print("RTC time: ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    if(now.minute() < 10)   Serial.print(0);
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    if(now.second() < 10)   Serial.print(0);
    Serial.println(now.second(), DEC);

    double tone_generator_freq = TONEGENERATORFREQ + freq_bump;
    double tone_generator_omega = tone_generator_freq * 6.2831853072;

    double t = 0;
    double cos_a = 1.;
    double sin_a = 0.;
    double sum_of_cosine_terms = 0;
    double sum_of_sine_terms = 0;

    // phase angle between successive ADC reads:
    double omega_dt = tone_generator_omega * dt;

    double cos_b = cos(omega_dt);
    double sin_b = sin(omega_dt);

    // now calculate the Fourier transform! I am using the vReal array
    // to hold mean-subtracted ADC values. I need to use it for both  the
    // cosine term and sine term... 
    // recall the definition of the Fourier transform.

    for (uint16_t i = 0; i < AUDIOSAMPLESIZE; i++)
    {
      // this is the real part of the FT...
      sum_of_cosine_terms = sum_of_cosine_terms + cos_a * vReal[i];
      // this is the imaginary part of the FT...
      sum_of_sine_terms = sum_of_sine_terms + sin_a * vReal[i];
    
      // now bump everything to have the terms ready for the next time bin.
      cos_a_updated = cos_a * cos_b - sin_a * sin_b;
      sin_a_updated = sin_a * cos_b + cos_a * sin_b;

      cos_a = cos_a_updated;
      sin_a = sin_a_updated;
      t = t + dt;
    }

    stop_time = micros();

    the_magnitude = sqrt(sum_of_cosine_terms * sum_of_cosine_terms + 
                         sum_of_sine_terms * sum_of_sine_terms);
    the_phase = 57.29578 * atan2(sum_of_sine_terms, sum_of_cosine_terms);

    if(the_magnitude > largest_magnitude)
    {
      largest_magnitude = the_magnitude;
      best_phase = the_phase;
      best_frequency = tone_generator_freq;
    }

    // now report what we got.
    Serial.print("Fourier transform of audio signal at ");
    Serial.print(tone_generator_freq);
    Serial.print(" Hz: real part = ");
    Serial.print(sum_of_cosine_terms);
    Serial.print(" and imaginary part = ");
    Serial.println(sum_of_sine_terms);

    Serial.print("Time required to calculate the transform (usec): ");
    Serial.println(stop_time - start_time);

    Serial.print(">>==-> Magnitude and phase are ");
    Serial.print(the_magnitude);
    Serial.print(" and ");
    Serial.print(the_phase);
    Serial.println(" degrees <-==<<");

  }

  if(FT_calls == 1)
  {
    Serial.println("*************************************");
    Serial.print("Largest magnitude, phase, and frequency are ");
    Serial.print(largest_magnitude);
    Serial.print(", ");
    Serial.print(best_phase);
    Serial.print(" degrees, and ");
    Serial.print(best_frequency);
    Serial.println(" Hz.");
    Serial.println("*************************************");
  }

  // now send this via LoRa to the base station. Load the radio 
  // packet, then call the routine to send. Format:
  //  [0]  - [1]  device ID (0 - 3) followed by a comma 
  //  [2]  - [8]  FT frequency estimate (Hz, f6.1) followed by a comma
  //  [9]  - [15] FT amplitude bin phase (degrees, f6.1) followed by a comma
  //  [16] - [25] FT magnitude (f9.1), followed by a comma
  //  [26] - [27] RTC seconds (UTC)
  //  [28]        '0'

  //                             11111111112222222222333333333344444444445555555555
  //                   012345678901234567890123456789012345678901234567890123456789
  sprintf(radiopacket, "%d,%6.1f,%6.1f,%9.1f,%02d", BOARD_ID, best_frequency,
    best_phase, largest_magnitude, now.second());

  radiopacket[LASTRADIOPACKETCHARINDEX] = {0};
  radiopacket[LASTRADIOPACKETCHARINDEX + 1] = {0};
  radiopacket[LASTRADIOPACKETCHARINDEX + 2] = {0};
  radiopacket[LASTRADIOPACKETCHARINDEX + 3] = {0};

  Serial.print("Now load the radiopacket... we will send   ");
  Serial.println(radiopacket);

  // Now do the interleaving delay so we don't have transmission from various trackers colliding
  delay(radio_transmit_delay);

  send_LoRa();

  Serial.print("Total time (ms) to perform and transmit FFT (including interleaving delay of ");
  Serial.print(radio_transmit_delay);
  Serial.print(" ms) is ");
  Serial.println(millis() - start_time_FT / 1000);
  Serial.print("Total time (ms) just for radio-transmission of FFT result: ");
  Serial.println(millis() - tstartLoRa);

  Serial.println("*************************************\n");

  return 0.;
}

//fn//////////////////////////////////////////////////////////////////////////////
// BME_T_P_RH()
//////////////////////////////////////////////////////////////////////////////////

int BME_T_P_RH()
{
  
  // read T, P, RH from the BME680.

  lcd.setCursor(0, 0);
  lcd.print(F("just entered    "));
  lcd.setCursor(0, 1);
  lcd.print(F("BME_T_P_RH"));

  read_BME680();

  lcd.setCursor(0, 0);
  lcd.print(F("BME_T_P_RH"));
  lcd.setCursor(0, 1);
  lcd.print(F("return =        "));
  lcd.setCursor(9, 1);
  lcd.print(0);

  return 0;
}

///////////////////////////////////////////////////////
// end of BME_T_P_RH()
///////////////////////////////////////////////////////

//fn/////////////////////////////////////////////////////////////////////////////////
// setup_BME680()
/////////////////////////////////////////////////////////////////////////////////////

int setup_BME680() {

  // Serial.println(F("inside setup_BME680"));

  int return_code = 0;

  // turn on the BME680, which measures temperature, pressure, etc.
  bme.begin();

  // if problems, tell the user.
  if (!bme.begin()) {

    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));

    //         0123456789012345
    lcd.setCursor(0, 0);
    lcd.print(F("BME680 is mute  "));
    lcd.setCursor(0, 1);
    lcd.print(F("so give up      "));

    return_code = -1;
    return return_code;
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // Serial.println(F("BME680 T/P/RH/VOC sensor is ready."));

  lcd.setCursor(0, 0);
  //         0123456789012345
  lcd.print(F("BME680 is alive "));
  lcd.setCursor(0, 1);
  lcd.print(F("so away we go!  "));

  return return_code;
}

//fn////////////////////////////////////////////////////////////////////////////////////
// read_BME680()
////////////////////////////////////////////////////////////////////////////////////////

int read_BME680()
{
  // read environmental data from BME680. return 0 if all is OK.

  if (! bme.performReading()) {
    //>>Serial.println(F("Failed to perform reading :("));
    return -1;

  } else {

    bme_temperature = bme.temperature;
    bme_pressure = bme.pressure / 100.0;
    bme_humidity = bme.humidity;
    bme_gas_resistance = bme.gas_resistance / 1000.0;

    Serial.print(F("BME680 all set. Data from it: "));
    Serial.print(F("T = "));
    Serial.print(bme_temperature);
    Serial.print(F(" *C;  P = "));
    Serial.print(bme_pressure);
    Serial.print(F(" hPa;  "));
    Serial.print(F("Humidity = "));
    Serial.print(bme_humidity);
    Serial.println(F(" %."));

    // Serial.print(F("Gas =         "));
    // Serial.print(bme_gas_resistance);
    // Serial.println(F(" kOhms"));

    return 0;
  }
}

// Roomba_basestation: file is in
// p398dlp_fa2021_tests/group4_roomba/Roomba_basestation.ino

#include <SPI.h>
#include <RH_RF95.h>
#include <LiquidCrystal.h>     
#include <SD.h>
#include <SD.h>
#include "RTClib.h"

////////////////////////// Roomba grid search data ///////////////////

// set this true to do stuff whether or not we've received all four trackers' signals.
// in this case, I'll just invent a Roomba position and calculate the expected phase 
// shifts, then do a grid search.
bool test_stuff = false;
// bool test_stuff = true;

// actual frequency (and wavelength) broadcast from the Roomba, Hz (and meters).
// float frequency = 1000.;
float frequency = 200.;
// float frequency = 100.;
float wavelength;

// speed of sound, meters per second
float v_sound = 340.;

// estimated phase precision and square of phase precision, in degrees^2
const float phase_precision = 5;
const float phase_precision_sq = phase_precision * phase_precision;

// initial x, y for trackers. I assume that tracker 0 rides around on the Roomba, 
// but 1, 2, and 3 are fixed. units are meters, of course.
// float xx0[4] = {2., 0., 0., 4.};
// float yy0[4] = {2., 4., 0., 0.};
float xx0[4] = {99., 0., 0., 2.};
float yy0[4] = {99., 2., 0., 0.};

// origin, and other geometrical paramaters for the search grid.
float gridx0 = 0.;
float gridy0 = 0.;
// grid cell size
float griddx = 0.05;
float griddy = 0.05;
// number of columns and rows in the grid
int numgridx = 40;
int numgridy = 40;

// expected amplitude of the Fourier transform, at a distance from the tone generator of 
// 0.25 meters. we expect power to drop like 1 / distance^2, but amplitudes to drop
// like 1 / distance^1.
float amplitude_at_1m = 5000.;
float chisq, best_chisq;

// for the phase differences, which can run from 0 to 360, instead 
// of -180 to +180.
float phase_difference[4];

int ibest, jbest;
float xbest, ybest;

// for each grid cell: where we are, and the predicted tracker results.
float xtrial, ytrial;
float trial_distance[4];
float trial_phase_difference[4];
float trial_amplitude[4];

float xroomba_simulated;
float yroomba_simulated;

//////////////////////////// RTC stuff //////////////////////

RTC_DS3231 rtc;

// RTC time, eg. 23:05:31
char RTC_time_string[9];

// RTC date, eg. 12/02/2020
char RTC_date_string[11];

DateTime nowis;

const char daysOfTheWeek[7][12] = 
  {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//////////////////////////// SD stuff //////////////////////

// SD.h lives in arduino.app/Contents/Java/libraries/SD/src.
// filenames should follow the 8.3 convention: up to 8 (upper case or numeric)
// characters in the file name, a period, then a three (upper case or numeric)
// character file extension. For example: ROOMBA21.TXT
File myFile;
// the very last character holds a null: a byte of all zeroes.
char myFilename[13] = 
  {'R', 'O', 'O', 'M', 'B', 'A', '0', '0', '.', 'T', 'X', 'T', {0}};
int filename_index = 0;
int filename_index_largest = 99;

// chip select line to Adalogger's SD:
#define SD_CS 4

// from Adafruit 
// https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/rfm9x-test

// define which pins do what! On the M0 and M4 pin 15 is also A1. pin 19 is A5.
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define HAS_LCD

#define LED_pin 12

// If you change to 434.0 or other frequency, must match TX's freq!
#define RF95_FREQ 915.0
//#define RF95_FREQ 868.0

// Singleton instance of the radio driver. Arguments like so:
// RH_RF95::RH_RF95(uint8_t slaveSelectPin, uint8_t interruptPin, RHGenericSPI& spi)
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// other SD card stuff
SdFile root;

//////////////////////////// LCD stuff //////////////////////

// initialize the LCD library by associating any needed LCD interface pins
// with the pin numbers to which they are connected
const int rs = 18, en = 11, d4 = 5, d5 = 6, d6 = 9, d7 = 19;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//////////////////////////// Roomba data //////////////////////
int board_id, now_second;
double best_frequency, best_phase, largest_magnitude;

// frequency, phase, magnitude, and RTC seconds for the most recent data from each of 
// the four trackers:
double last_frequency[4] = {1., 2., 3., 4.};
double last_phase[4] = {5., 6., 7., 8.};
double last_magnitude[4] = {9., 10.,11., 12.};
int last_second[4] = {-96, -97, -98, -99};


//////////////////////////////////////////////////////////////////////

void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
 
  // light up the serial monitor and wait for it to be ready.
  // include a time-out test to keep things from hanging if the 
  // serial momnitor isn't open.

  uint32_t t1234 = millis();
  Serial.begin(115200);
  while (!Serial && millis() - t1234 < 5000) {}

  Serial.println("\n\n***** Roomba base station DAQ *****");

  Serial.println(F("file holding this program is ")); Serial.println(__FILE__);

  // get the time at which the program was compiled just before upload.
  // this'll be several seconds behind the actual time, should we decide 
  // to use it to set the RTC.

  Serial.print(F("System date and time of program compilation: "));
  Serial.print(__DATE__);
  Serial.print(F("  ")); Serial.println(__TIME__);

  // wait for the RTC to be ready
  while (! rtc.begin()) {}

  // set the RTC to the date & time this sketch was compiled since we aren't
  // bothering to set the RTC to UTC using the GPS.
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // echo the RTC time.
  nowis = rtc.now();
  Serial.print(F("Initial DS3231 RTC reading: "));
  Serial.print(nowis.year(), DEC);
  Serial.print('/');
  if(nowis.month() < 10) {Serial.print(F("0"));}
  Serial.print(nowis.month(), DEC);
  Serial.print('/');
  if(nowis.day() < 10) {Serial.print(F("0"));}
  Serial.print(nowis.day(), DEC);
  Serial.print(F(" ("));
  Serial.print(daysOfTheWeek[nowis.dayOfTheWeek()]);
  Serial.print(F(") "));
  Serial.print(nowis.hour(), DEC);
  Serial.print(':');
  if(nowis.minute() < 10) {Serial.print(F("0"));}
  Serial.print(nowis.minute(), DEC);
  Serial.print(':');
  if(nowis.second() < 10) {Serial.print(F("0"));}
  Serial.print(nowis.second(), DEC);
  Serial.println();

  // setup microSD memory breakout 
  int return_code = setup_SD();

  // set the date/time for the SDfile system so we'll have a good creation 
  // date/time for our files.
  SdFile::dateTimeCallback(dateTime);  

  // also figure out what filename to use for the Roomba data. We want to 
  // avoid overwriting files that already exist.
  for(int ijk = 0; ijk <= filename_index_largest; ijk++)
  {
    // build the tentative filename, using a base 10 conversion (not hexadecimal!)
    char filenumber_char[3]; 
    // convert an integer to a character array...
    itoa(ijk, filenumber_char, 10);
    if(ijk < 10)
    {
      myFilename[6] = '0';
      myFilename[7] = filenumber_char[0];
    } else {      
      myFilename[6] = filenumber_char[0];
      myFilename[7] = filenumber_char[1];
    }
    // filenames will be things like ROOMBA02.TXT

    // Serial.print(myFilename);
    // Serial.print(" exists? ");
    // Serial.println(SD.exists(myFilename));

    // if the file doesn't already exist, we'll use that as the file name.
    if (!SD.exists(myFilename)) 
    {
      Serial.println("We will write data to the file ");
      Serial.println(myFilename);
      break;
    } 
  }

  // if all files from 00 to 99 already exist, delete 99 and write to it.
  if (SD.exists(myFilename)) 
  {
    Serial.println("No file names are available, so delete ");
    Serial.print(myFilename);
    Serial.println(" and write to that file.");
    SD.remove(myFilename);
  }

  // ask the real time clock what the time is...
  nowis = rtc.now();

  Serial.print("I am assuming the trackers are hearing a ");
  Serial.print(frequency);
  Serial.println(" Hz tone.");

  Serial.println("Tracker 0 rides along with the Roomba, as does the tone generator.");
  Serial.println("Other tracker x,y positions (in meters):");
  Serial.print("tracker 1 x, y = ");
  Serial.print(xx0[1], 2);
  Serial.print(", ");
  Serial.println(yy0[1], 2);
  Serial.print("tracker 2 x, y = ");
  Serial.print(xx0[2], 2);
  Serial.print(", ");
  Serial.println(yy0[2], 2);
  Serial.print("tracker 3 x, y = ");
  Serial.print(xx0[3], 2);
  Serial.print(", ");
  Serial.println(yy0[3], 2);

  Serial.println("Search grid geometry");
  Serial.print("x, y grid cell size: ");
  Serial.print(griddx, 3);
  Serial.print(", ");
  Serial.println(griddy, 3);
  Serial.print("grid lower left, upper right corners: ");
  Serial.print(gridx0, 3);
  Serial.print(", ");
  Serial.print(gridy0, 3);
  Serial.print("; ");
  
  Serial.print(gridx0 + numgridx * griddx, 3);
  Serial.print(", ");
  Serial.print(gridy0 + numgridy * griddy, 3);
  Serial.println(". ");

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  delay(100);

  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  //         0123456789012345
  lcd.print("Now do LoRa test");
  lcd.setCursor(0, 1);
  lcd.print("recption. Waitng");

  // now set up the radio
  return_code = setup_LoRa();

  // initialize Roomba grid search stuff
  wavelength = v_sound / frequency;

  // away we go!
}

//////////////////////////////////////////////////////////////////////

void loop()
{
  // Here is radio message format from Roomba tracker:
  //  [0]  - [1]  device ID (0 - 3) followed by a comma 
  //  [2]  - [8]  FT frequency estimate (Hz, f6.1) followed by a comma
  //  [9]  - [15] FT amplitude bin phase (degrees, f6.1) followed by a comma
  //  [16] - [25] FT magnitude (f9.1), followed by a comma
  //  [26] - [27] RTC seconds (UTC)
  //  [28]        '0'

  bool we_received_a_message = false;

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  // put some nulls into the end of the buffer.
  for(int ijklm = 0; ijklm < RH_RF95_MAX_MESSAGE_LEN; ijklm++) {buf[ijklm] = {0};}

  ////////////// receive a message //////////////
  if (rf95.available())
  {
    // Should be a message for us now.

    we_received_a_message = true;

    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED_pin, HIGH);

      ////////////// send a reply-to message //////////////

      char radiopacket[20] = "LoRa M0 RX ack";
      radiopacket[19] = 0;
      
      Serial.println("\nRoomba basestation acknowledging the packet"); 
      Serial.println((char*)buf); 

      delay(10);
      rf95.send((uint8_t *)radiopacket, 20);
      delay(10);
      rf95.waitPacketSent();

      lcd.setCursor(0, 0);
      lcd.print("ACK sent. All is");
      //         0123456789012345 
      lcd.setCursor(0, 1);
      lcd.print("fine, probably! ");

      digitalWrite(LED_pin, LOW);
    }
  }
  ////////////// end of receive-a-message code. //////////////

  // if we got a message, do something with it, otherwise return.

  if(!we_received_a_message) {return;}

  // Serial.print("base station frequency (MHz): ");
  // Serial.println(RF95_FREQ);
  // Serial.print("The message, as characters, is '");
  // Serial.print((char*)buf);
  // Serial.println("'");
  // Serial.print("RSSI (signal strength, in dB): ");
  // Serial.println(rf95.lastRssi(), DEC);
  
  // now convert character arrays in the message to numbers.

  char c_board_id[2] = {buf[0], {0}};
  char c_best_frequency[7] = {buf[2],  buf[3],  buf[4],  buf[5],  buf[6],  buf[7], {0}};
  char c_best_phase[7] = {buf[9],  buf[10], buf[11], buf[12], buf[13], buf[14], {0}};
  char c_largest_magnitude[10] = {buf[16],  buf[17], buf[18], buf[19], buf[20], buf[21],
                                  buf[22],  buf[23], buf[24], {0}};
  char c_now_second[3] = {buf[26],  buf[27], {0}};

  board_id = atoi(c_board_id);
  best_frequency = atof(c_best_frequency);
  best_phase = atof(c_best_phase);
  largest_magnitude = atof(c_largest_magnitude);
  now_second = atoi(c_now_second);


/*
  // now print what we got.

  Serial.print("Base has acknowledged the Roomba with");
  Serial.print(" ID = ");
  Serial.print(board_id);
  Serial.print(", frequency = ");
  Serial.print(best_frequency, 1);
  Serial.print(", phase = ");
  Serial.print(best_phase, 1);
  Serial.print(", magnitude = ");
  Serial.print(largest_magnitude, 1);
  Serial.print(", and RTC seconds = ");
  Serial.print(now_second);
  Serial.println(".");
*/

  lcd.setCursor(0, 0);
  //         0123456789012345
  lcd.print("                ");
  lcd.setCursor(0, 0);
  lcd.print("ID");
  lcd.print(board_id);
  lcd.print(" f=");
  lcd.print(best_frequency, 0);
  lcd.print(" s=");
  lcd.print(now_second);

  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("ph:");
  lcd.print(best_phase, 0);
  lcd.print(" m:");
  lcd.print(largest_magnitude, 0);

  // now put those values into global arrays.
  if(board_id < 0 || board_id > 3)
  {
    Serial.println(">>>>>>>>>>>>>>>>> Illegal board ID!!! <<<<<<<<<<<<<<<<<<<");
    return;
  } else {
    last_frequency[board_id] = best_frequency;
    last_phase[board_id] = best_phase;
    last_magnitude[board_id] = largest_magnitude;
    last_second[board_id] = now_second;
  }

  // now check that, upon receiving ID = 3 data, we have ID = 0, 1, 2 from the
  // same RTC second.

  bool OK_to_do_fits = 
    (last_second[0] == last_second[1] &&
     last_second[0] == last_second[2] &&
     last_second[0] == last_second[3]) || test_stuff;

  if(board_id == 3 && !OK_to_do_fits) 
  {
    Serial.println("Just received the 4th tracker's data, but we can't fit the Roomba location!");
    return;
  }

  if((board_id == 3 && OK_to_do_fits) || test_stuff) 
  {
    Serial.println("Now we can write to SD and fit for the Roomba location.");

    // disable the radio for the moment by jammming the chip select line high.
    digitalWrite(RFM95_CS, HIGH);
    delay(100);

    if (!SD.begin(SD_CS)) 
    {
      Serial.println("SD initialization failed!");
      return;
    }

    // set the date/time for the SDfile system so we'll have a good creation 
    // date/time for our files.
    SdFile::dateTimeCallback(dateTime);  

    // write the four trackers' data to SD. Open the file...
    myFile = SD.open(myFilename, FILE_WRITE);

    // if the file opened okay, write to it:
    if (myFile) 
    {
      // Serial.println("roomba data file opened successfully...");
      for(int ijk = 0; ijk < 4; ijk++)
      {
        myFile.print(ijk);
        myFile.print(",");
        myFile.print(last_frequency[ijk]);
        myFile.print(",");
        myFile.print(last_phase[ijk]);
        myFile.print(",");
        myFile.print(last_magnitude[ijk]);
        myFile.print(",");
        myFile.println(last_second[ijk]);
      }
      // close the file:
      myFile.close();

    } else {
    // if the file didn't open, print an error:
    Serial.println(">>>>> error opening Roomba data file\n");
    }

    //////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////
    // Sean's code goes here.
    //////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////

    // when we get to here, we either have four tracker phase values or 
    // else we're in test mode.

    // if we're in test mode, let's fake up the phase data. Put the Roomba at 
    // x, y = 2, 1.

    bool debug_grid = false;

    // phase jitters for testing; choose flat, centered on zero.
    double phase_jitter1 =  phase_precision * ((random(0, 1001) / 1000.) - 0.5);
    double phase_jitter2 =  phase_precision * ((random(0, 1001) / 1000.) - 0.5);
    double phase_jitter3 =  phase_precision * ((random(0, 1001) / 1000.) - 0.5);

    if(test_stuff)
    {
      Serial.println("Setting up to test our search algorithm.");

      // pick the Roomba position randomly. Arduino function random(min, max) gives
      // an integer between min and max-1 so...
      long randNumber;
      randNumber = random(0, 1001);
      xroomba_simulated = gridx0 + (randNumber * numgridx * griddx) / 1000.;
      randNumber = random(0, 1001);
      yroomba_simulated = gridy0 + (randNumber * numgridy * griddy) / 1000.;

      float d1 = sqrt(pow((xx0[1] - xroomba_simulated), 2) + pow((yy0[1] - yroomba_simulated), 2));
      float d2 = sqrt(pow((xx0[2] - xroomba_simulated), 2) + pow((yy0[2] - yroomba_simulated), 2));
      float d3 = sqrt(pow((xx0[3] - xroomba_simulated), 2) + pow((yy0[3] - yroomba_simulated), 2));

      // phase jitters; choose flat, centered on zero.
      double phase_jitter1 =  phase_precision * (random(0, 1001) / 1000.) - 0.5;
      double phase_jitter2 =  phase_precision * (random(0, 1001) / 1000.) - 0.5;
      double phase_jitter3 =  phase_precision * (random(0, 1001) / 1000.) - 0.5;

      double ratio_working;
      last_phase[0] = 0.;
      ratio_working = d1 / wavelength;
      last_phase[1] = 360. * (ratio_working - floor(ratio_working)) + phase_jitter1;
      ratio_working = d2 / wavelength;
      last_phase[2] = 360. * (ratio_working - floor(ratio_working)) + phase_jitter2;
      ratio_working = d3 / wavelength;
      last_phase[3] = 360. * (ratio_working - floor(ratio_working)) + phase_jitter3;
    }

    // load the phase differences now.
    for(int trackerID = 0; trackerID < 4; trackerID++)
    {
      phase_difference[trackerID] = last_phase[trackerID] - last_phase[0];
      if(phase_difference[trackerID] < 0.) 
      {
        phase_difference[trackerID] = phase_difference[trackerID] + 360.;
      }
      Serial.print("Phase differences for tracker ID ");
      Serial.print(trackerID);
      Serial.print(" in degrees: ");
      Serial.println(phase_difference[trackerID]);
    }

    // now loop over all cells in the grid.
    int i, j;
    best_chisq = 1.e9;

    uint32_t grid_search_start = millis();

    for(i = 0; i < numgridx; i++)
    {
      for(j = 0; j < numgridy; j++)
      {
        // position of our grid point
        xtrial = i * griddx;
        ytrial = j * griddy;

        // initialize the chisq sum...
        chisq = 0; 

        // distances to each of trackers 1, 2, 3 and also the predicted phase lags.
        for(int ijk = 1; ijk < 4; ijk++)
        {
          trial_distance[ijk] = sqrt( pow(xtrial - xx0[ijk], 2) + pow(ytrial - yy0[ijk], 2));
          // we want to get the fraction of wavelengths past an integer number of wavelengths
          // to calculate a phase difference
          double ratio_working = trial_distance[ijk] / wavelength;
          trial_phase_difference[ijk] = 360. * (ratio_working - floor(ratio_working));
          trial_amplitude[ijk] = amplitude_at_1m / trial_distance[ijk];

          // eventually consider include a look at the actual amplitudes in the chisq.
          chisq = chisq + pow(trial_phase_difference[ijk] - phase_difference[ijk], 2) / 
            phase_precision_sq;
        }

        if(debug_grid)
        {
          Serial.print(">>> trial x, y  and chisq = ");
          Serial.print(xtrial); 
          Serial.print(", "); 
          Serial.print(ytrial); 
          Serial.print(". "); 
          Serial.println(chisq); 
        }

        if(chisq < best_chisq)
        {
          best_chisq = chisq;
          ibest = i;
          jbest = j;
          xbest = xtrial;
          ybest = ytrial;
        }
      }
    }
    Serial.print("Grid search just finished. Time (ms) was ");
    Serial.println(millis() - grid_search_start);
    if(test_stuff)
    {
      Serial.print("Simulated x, y were ");
      Serial.print(xroomba_simulated, 3);
      Serial.print(", ");
      Serial.println(yroomba_simulated, 3);
      Serial.print("phase jitters injected into tracker 1, 2, 3: ");
      Serial.print(phase_jitter1, 2);
      Serial.print(" ");
      Serial.print(phase_jitter2, 2);
      Serial.print(" ");
      Serial.print(phase_jitter3, 2);
      Serial.println(" ");
    }

    Serial.print("Best x, y and chisq are ");
    Serial.print(xbest, 3);
    Serial.print(", ");
    Serial.print(ybest, 3);
    Serial.print(". chisq =  ");
    Serial.println(best_chisq, 3);

    // disable the radio for the moment by jammming the chip select line high.
    digitalWrite(RFM95_CS, HIGH);
    delay(100);

    if (!SD.begin(SD_CS)) 
    {
      Serial.println("SD initialization failed!");
      return;
    }

    // set the date/time for the SDfile system so we'll have a good creation 
    // date/time for our files.
    SdFile::dateTimeCallback(dateTime);  

    // now write the results to SD.
    myFile = SD.open(myFilename, FILE_WRITE);

    // Serial.print(". Now writing grid results to SD.");

    // if the file opened okay, write to it. keep format similar to 
    // what we used for Roomba tracker data, but use 99 for the ID number
    // and put x, y, and chisq into three of the other fields.
    if (myFile) 
    {
      // "ID"
      myFile.print(99);
      myFile.print(",");
      myFile.print(xbest);
      myFile.print(",");
      myFile.print(ybest);
      myFile.print(",");
      myFile.print(best_chisq);
      myFile.print(",");
      myFile.println(last_second[3]);
      // close the file:
      myFile.close();
      Serial.println("Grid search result has been written to SD.\n");

    } else {
      Serial.println("Couldn't open SD when trying to write grid search result\n");
    }

    // turn the radio back on and leave.
    setup_LoRa();
    return;
  }

  Serial.println("Waiting for all four trackers to report in...");
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

  // set the date/time for the SDfile system so we'll have a good creation 
  // date/time for our files.
  SdFile::dateTimeCallback(dateTime);  

  // open the file. 
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

  // set the date/time for the SDfile system so we'll have a good creation 
  // date/time for our files.
  SdFile::dateTimeCallback(dateTime);  

  myFile = SD.open("test.txt");

  if (myFile) 
  {
    // Serial.println("test.txt contents:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) 
    {
      // Serial.write(myFile.read());
      char meepzoorp = myFile.read();
    }

    // close the file:
    myFile.close();
    // Serial.println(" We are finished testing the SD.");
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
  // Since the group 1 PCB doesn't connect the RST input, take this into account.

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    return_code = -1;
    return return_code;
  }

  // Serial.print("LoRa radio init OK... ");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  // but we want to use 915 MHz...
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    return_code = -2;
    return return_code;
  }
  // Serial.print("Just set Freq to "); Serial.print(RF95_FREQ);
  
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  return return_code;
}

/////////////////////////////////////////////////////////////////////////

void dateTime(uint16_t* date, uint16_t* time) {

  // call back for file timestamps. NOTE THE CASE of the function name.
  
  // now get the current time.
  nowis = rtc.now();
 
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(nowis.year(), nowis.month(), nowis.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(nowis.hour(), nowis.minute(), nowis.second());
}

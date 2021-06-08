/*-----------------------------------------------------------------------------
 *  Author: Felipe G. Nievinski <fgnievinski@gmail.com>
 *  Author: Nelso G. Jost
 *  Author: Cristthian Marafigo Arpino
 *  Author: Lucas Doria de Carvalo
 *  Author: Iuri Mendonça Tinti
 *  License: GPLv2
 *  Purpose: Firmware to perform simple datalog of GPS raw data with the
 *           Adalogger Feather board and GPS Feathering module.
 *           Arduino IDE and PlatformIO compatible.
 *---------------------------------------------------------------------------*/
#include <SPI.h>
#include "SdFat.h"


// USER SETTINGS

// file duration:
const char fileDuration = 'H';     // use 'H' to save by Hour
//const char fileDuration = 'D';     // use 'D' to save by Day

// time interval between GPS readings (in milliseconds):
//#define GPS_UPDATE_INTERVAL  1000  //1Hz
//#define GPS_UPDATE_INTERVAL  500 //2Hz
//#define GPS_UPDATE_INTERVAL  200 //5Hz
#define GPS_UPDATE_INTERVAL  100 //10Hz 
// The possible interval values range between 100 and 10000 milliseconds.
// Examples: 10000 ms (10 s, 0.1 Hz), 1000 ms (1 s, 1 Hz), 100 ms (0.1 s, 10 Hz), 500 ms (0.5 s, 2 Hz), etc.
// Note: high sampling rates (greater than 1 Hz -- intervals smaller than 1000 ms) require the faster M0 board instead of 32u4.

// are you using the M0 board instead of 32u4?
#define USING_M0_BOARD

// battery update interval (in milliseconds; zero to disable)
#define BATTERY_UPDATE_INTERVAL  60000  // every minute
//#define BATTERY_UPDATE_INTERVAL  1000  // every second

// give visual indication that SD writing is okay?
#define BLINK_LED_IF_WRITING_OKAY

// print brief messages to USB serial?
#define DEBUG

// print detailed messages to USB serial?
//#define VERBOSE

// optimize settings for high-rate logging (>1Hz)?
//#define HIGH_RATE_OPTIMIZED
#ifdef HIGH_RATE_OPTIMIZED
    // disable features that would slow down the loop:
    #undef VERBOSE
    #undef DEBUG
    //#undef BLINK_LED_IF_WRITING_OKAY
    #define BATTERY_UPDATE_INTERVAL  0
#endif

// wait for valid GPS or write to DEFAULT.LOG file meanwhile?
//#define WAIT_FOR_VALID_GPS  // if not defined, we'll write to default file.


// wait for USB connection to PC?
// (useful for not missing first debug messages)
// Note: leave it undefined for field deployment!!!
//#define WAIT_FOR_USB


// INTERNAL SETTINGS

// GPS baud rates:
#define GPS_BAUD_RATE_DEFAULT    9600   // original GPS firmware
#define GPS_BAUD_RATE  115200

// time to wait for the GPS to finish writing data to buffer:
#define BUFFER_IDLE_THRESHOLD      10             // milliseconds
// It should be greater than 5 ms, which is the maximum time between GPS serial events (determined experimentally).

// time interval between SD card writings (in milliseconds) - EXPERIMENTAL:
#define  SDCARD_UPDATE_INTERVAL  GPS_UPDATE_INTERVAL
// Note: SDCARD_UPDATE_INTERVAL cannot be much greater than GPS_UPDATE_INTERVAL, otherwise the buffer will not fit in memory.
// Note: in any case, SDCARD_UPDATE_INTERVAL will only take effect when the GPS buffer is idle.
// Note: this is an experimental feature.

// typical GPS block size
#define  GPS_BLOCK_SIZE_TYPICAL 512

// typical buffer size, for pre-allocation:
const int GPS_BUFFER_SIZE_TYPICAL=GPS_BLOCK_SIZE_TYPICAL*
ceil((double) SDCARD_UPDATE_INTERVAL/GPS_UPDATE_INTERVAL);

// file name limits:
#define BASENAME_LEN_MAX 8+1  // FAT limit: 8.3
#define FILENAME_LEN_MAX BASENAME_LEN_MAX-1+1+3+1  // basename-null+dot+extension+null

// maximum NMEA sentence length 
// (as per <https://en.wikipedia.org/wiki/NMEA_0183>)
#define NMEA_SENTENCE_LEN_MAX 82

// define battery voltage pin:
#ifdef USING_M0_BOARD
    #define VBATPIN A7 // M0 battery voltage pin
#else
    #define VBATPIN A9  // 32u4 battery voltage pin
#endif

#ifdef DEBUG
    // print to USB serial (checking if port is open):
    #define serialPrint(x) if(Serial)Serial.print(x)
    #define serialPrintln(x) if(Serial)Serial.println(x)
#else
    // ignore status messages completely:
    #define serialPrint(x) 
    #define serialPrintln(x) 
#endif

#define gpsSerial Serial1
#define gpsSerialEvent serialEvent1
#ifdef USING_M0_BOARD
    void gpsSerialEvent();  
#endif


// global variables and constants:
const int SDCard = 4;  // SD card ID
const char basenameDefault[] = "DEFAULT";
char basenameOld[BASENAME_LEN_MAX];  // last used basename
String buffer = "";   // to hold the incoming GPS data
unsigned long bufferTimeEnd = millis();  // timer to indicate when buffer was last written
unsigned long sdcardTimeEnd = millis();  // timer to indicate when SD card was last written
unsigned long batteryTimeEnd = millis();  // timer to indicate when battery voltage was last written
SdFat sd;
// function prototypes:
bool initSD(void);
bool getRMCSentence(const char strInp[], char strOut[]);
bool getDateTime(const char strInp[], char dateTime[]);
void resetBuffer(void);
void configGPS(void);
void configGPSBaudRate(void);
void getBasename(char basename[], const char dateTime[], bool isGPSValid);
void datalog(const char basename[], const char dateTime[]);
void blinkLED(void);
void vbatlog(const char basename[], const char dateTime []);
void writeBatteryVoltage(const char basename[], const char dateTime[]);
float readBatteryVoltage (void);
const char* nth_strchr(const char* s, int c, int n);
void getNmeaUpdateCommand(char command[]);
uint8_t getCheckSum(char *string);


// main functions
void setup()
{
    Serial.begin(9600);
    //Serial.begin(115200);  // open USB serial

// 
#ifdef WAIT_FOR_USB
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
#endif
    
    initSD();
    configGPS();
    delay(1000);
    
    buffer.reserve(GPS_BUFFER_SIZE_TYPICAL);
    gpsSerial.flush();
}

/* Rationale: GPS keeps sending characters via a serial event;
 * We store the GPS characters in a string (buffer).
 * When the GPS stops talking, then we log the data to SD card.
 */

void loop()
{
    bool isGPSValid; // = gpsActive(?)
    char dateTime[6+6+1];  // yymmddHHMMSS\0
    char basename[BASENAME_LEN_MAX];
    unsigned long timeStart = millis();
    // time interval elapsed since last buffer write:
    unsigned long bufferTimeIdle = timeStart - bufferTimeEnd;

#ifdef USING_M0_BOARD
    if (gpsSerial.available()) {
        // M0 board doesn't support serial events natively
        // <https://forums.adafruit.com/viewtopic.php?f=52&t=140637&p=695321&hilit=m0+serialevent#p695321>
        gpsSerialEvent();
    }
#endif

    if ( bufferTimeIdle < BUFFER_IDLE_THRESHOLD || buffer.length() < 3) { return; }

    serialPrintln("to be written to SD:");
    serialPrintln(buffer);

    // GPS serial went silent, now try to log data to SD card.
#ifdef VERBOSE
    serialPrint(F("[VERBOSE] GPS serial idle for (ms): "));
    serialPrintln(bufferTimeIdle);
#endif

    isGPSValid = getDateTime(buffer.c_str(), dateTime);
    
    if (!isGPSValid)
    {
#ifdef WAIT_FOR_VALID_GPS
        serialPrintln(F("[DEBUG] waiting for GPS to be valid."));
        resetBuffer();
        return;
#endif
    }


    getBasename(basename, dateTime, isGPSValid);
    
    datalog(basename);

    vbatlog(basename, dateTime);

// clear the buffer and preallocate memory:    
    resetBuffer();
    
//#### pq essa mensagem? O tamanho é fixo e definido no cabeçalho do código.
#ifdef VERBOSE
    serialPrint(F("[DEBUG] GPS buffer size: "));
    serialPrintln(GPS_BUFFER_SIZE_TYPICAL);
#endif

    serialPrint(F("[DEBUG] done writing; time ellapsed on loop() function (ms): "));
    serialPrintln(millis() - timeStart);
    serialPrintln("");

    if (strcmp(basename, basenameOld) != 0)
    {
        strncpy(basenameOld, basename, strlen(basename)+1);
    }
}


// called in between loops, every time a new character appears at gpsSerial
void gpsSerialEvent()
{
    unsigned long bufferTimeStart = millis();
  
    // store the caracter in the buffer
    buffer += char(gpsSerial.read());

    // restart the idle timer count
    bufferTimeEnd = millis();

#ifdef VERBOSE
    serialPrint("[VERBOSE] gpsSerialRead time (ms): ");
    serialPrintln(bufferTimeEnd-bufferTimeStart);
#endif
}


// reset GPS buffer and serial:
void resetBuffer()
{
    buffer = "";
    buffer.reserve(GPS_BUFFER_SIZE_TYPICAL);

}


// configure the GPS settings
void configGPS(void)
{
    // configure the GPS baud rate:
    configGPSBaudRate();

    // configure the GPS update rate:
    // (e.g., PMTK_SET_NMEA_UPDATE_1HZ)
    char command[18+sizeof(int)*2];    
    getNmeaUpdateCommand(command);
    gpsSerial.println(command);
    
    // don't show antenna status messages
    // gpsSerial.println(F("$PGCMD,33,1*6C"));  // PGCMD_ANTENNA
    gpsSerial.println(F("$PGCMD,33,0*6D"));  // PGCMD_NOANTENNA

    // define which NMEA messages to print (PMTK_SET_NMEA_OUTPUT_RMCGGAGSV):
    gpsSerial.println(F("$PMTK314,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29"));

    // disable speed threshold, so that coordinates do change:
    gpsSerial.println(F("$PMTK397,0.0*3D")); 

    // disable Active Interference Cancellation (AIC)
    gpsSerial.println(F("$PMTK286,0*22"));

    serialPrintln(F("[DEBUG] GPS configured."));
}


// configure GPS baud rate
void configGPSBaudRate(void) 
{
    if (GPS_BAUD_RATE == GPS_BAUD_RATE_DEFAULT) {
        gpsSerial.begin(GPS_BAUD_RATE);
        return;
    }
    
    // open GPS at default baud rate (9600) then 
    // change it to the new rate (115200) and reconnect:
    // (if it was already at 115200, it will be ignored.)
    gpsSerial.begin(GPS_BAUD_RATE_DEFAULT);
    switch (GPS_BAUD_RATE)
    {
        case 57600:
            // PMTK_SET_BAUD_57600
            gpsSerial.println(F("$PMTK251,57600*2C"));
            break;
        case 115200:
            // PMTK_SET_BAUD_115200
            gpsSerial.println(F("$PMTK251,115200*1F"));
            break;
    }

    delay(1000);
    gpsSerial.begin(GPS_BAUD_RATE);
}


// initialize SD card
bool initSD(void)
{
    // WARNING: SD.begin() can only be called once due to a bug at the Arduino SD.h standard library

    if (!sd.begin(SDCard,SD_SCK_MHZ(10)))
    {
        serialPrintln(F("[ERROR] Unable to initialized SD card."));
        return false;
    }
    serialPrintln(F("[DEBUG] SD card initialized."));

    return true;
}

//NOT USED
// extract Recommended Minimum Common (RMC) sentence
/*bool getRMCSentence(const char strInp[], char strOut[])
{
    char* limInf=NULL;
    char* limSup=NULL;
    int len;
#ifdef VERBOSE
    serialPrintln(F("[VERBOSE] Starting getRMCSentence."));
#endif

    if(strlen(strInp)==0)
    {
#ifdef VERBOSE
        serialPrintln(F("[VERBOSE] Empty GPS string."));
#endif
        return false;
    }

    // find beginning of $GPRMC sentence:
    limInf=strstr(strInp, "$GPRMC");
    if(!limInf)
    {
#ifdef VERBOSE
        serialPrintln(F("[VERBOSE] RMC sentence not found."));
#endif
        return false;
    }

    // find end of $GPRMC sentence:
    limSup=strchr(limInf, '\n');
    if(!limSup)
    {
#ifdef VERBOSE
        serialPrintln(F("[VERBOSE] RMC incomplete."));
#endif
        return false;
    }

    // extract RMC sentence length:
    len=(limSup-limInf)+1;
#ifdef VERBOSE
    serialPrint(F("[VERBOSE] RMC length: "));  serialPrintln(len);
#endif

    //TODO: verify checksum (hint: use function getCheckSum)
    
    // extract full RMC sentence:
    strncpy(strOut, limInf, min(len, NMEA_SENTENCE_LEN_MAX));
    strOut[len]='\0';
#ifdef VERBOSE
    serialPrint(F("[VERBOSE] RMC sentence: "));  serialPrint(strOut);
#endif

    return true;
}*/

// fills a string with date and time from the GPS:
bool getDateTime(const char strInp[], char dateTimeOut[])
{ 
    char  strTemp[82];  // as per <https://en.wikipedia.org/wiki/NMEA_0183>
    char* limInf=NULL;
    char* limSup=NULL;
    const char *dateIn;
    const char *timeIn;
    int len;

     /* here we handle a string like this: 
     *    $GPRMC,144016.000,A,3001.2778,S,05113.2839,W,0.01,37.24,090318,,,A*59
     * then find the date (090318) and time (144016, discarding decimal seconds),
     * and finally reorder it as YYMMDDhhmmss: 180309124016.
    */

    
#ifdef VERBOSE
    serialPrintln(F("[VERBOSE] Starting getDateTime."));
#endif

    /* here we handle a string like this: 
     *    $GPRMC,144016.000,A,3001.2778,S,05113.2839,W,0.01,37.24,090318,,,A*59
     * then find the date (090318) and time (144016, discarding decimal seconds),
     * and finally reorder it as YYMMDDhhmmss: 180309124016.
    */


    if(strlen(strInp)==0)
    {
        serialPrintln("[DEBUG] Empty GPS string.");
        return false;
    }


    // find beginning of $GPRMC sentence:
    limInf=strstr(strInp, "$GPRMC");
    if(!limInf)
    {
        serialPrintln("[DEBUG] $GPRMC not found.");
        return false;
    }

    // verify if GPS is active:
    if((limInf[18]!='A'))
    {
#ifdef DEBUG
        serialPrintln(F("[VERBOSE] GPS not valid."));
#endif
        return false;
    }

    // find end of $GPRMC sentence:
    limSup=strchr(limInf, '\n');
    len=(limSup-limInf)+1;
    strncpy(strTemp, limInf, min(len, sizeof(strTemp)-1));
    strTemp[len]='\0';
    
    // extract date, reordering components (DDMMYY -> YYMMDD):
    dateIn = nth_strchr(strTemp, ',', 9) + 1;
    dateTimeOut[0] = dateIn[4];  // year
    dateTimeOut[1] = dateIn[5];  // year
    dateTimeOut[2] = dateIn[2];  // month
    dateTimeOut[3] = dateIn[3];  // month
    dateTimeOut[4] = dateIn[0];  // day
    dateTimeOut[5] = dateIn[1];  // day

    // extract time, keeping the order (hhmmss):
    timeIn = &strTemp[7];
    strncpy(&dateTimeOut[6], timeIn, 6);

    // terminate string:
    dateTimeOut[12]='\0';
#ifdef VERBOSE
    serialPrint(F("[VERBOSE] dateTime: "));
    serialPrintln(dateTimeOut);
#endif

    return true;
}


// obtain basename:
void getBasename(char basename[], const char dateTime[], bool isGPSValid)
{
    if(!isGPSValid)
    {
        // use default basename:
        strncpy(basename, basenameDefault, strlen(basenameDefault)+1);
        return;
    }

    // decide how many characters to copy:
    int num;
    switch (fileDuration)
    {
    case 'H':  // hour
        num = 8;
        break;
    case 'D':  // day
        num = 6;
        break;
    default:
        serialPrintln(F("[ERROR] Filename duration not recognized; assuming hourly."));
        num = 8;
    }

    // copy date/time (YYMMDDhh) for the basename:
    strncpy(basename, dateTime, num);
    
    // null-terminate string:
    basename[num]='\0';
    
#ifdef VERBOSE
    serialPrint(F("[VERBOSE] basename: "));  serialPrintln(basename);
#endif
}


// write data to SD card
void datalog(const char basename[])
{
    char filename[FILENAME_LEN_MAX];
    strncpy(filename, basename, min(BASENAME_LEN_MAX, strlen(basename)+1));
    strcat(filename, ".log");
    //char filename[] = "test.log";  // DEBUG
#ifdef VERBOSE
    serialPrint(F("[VERBOSE] filename (GPS): "));  serialPrintln(filename);
#endif
    
    // open the file
    File file = sd.open(filename, FILE_WRITE);
    if (!file)
    {
//        serialPrintln(F("[ERROR] Unable to open GPS log file."));
        serialPrintln("[ERROR] Unable to open file '" + String(filename) + "'.");
        serialPrintln(F("[ERROR] Check if SD card is connected"));
        return;
    }

    // get the size of string to check latter if it has been all written:
    byte len1 = buffer.length();
    
    // write whole GPS buffer on the SD file at once:
    byte len2 = file.print(buffer);
    
    // insert a blank line between blocks:
    file.println();

    // closing the file to guarantee integrity:
    // (it costs about 200 ms, same as flush())
    //return true;
    file.close();

    // give visual feedback to user:
    if (len2 == len1) { blinkLED(); }

    // give written feedback to user:
    String prefix = "[DEBUG]";
    if (len2 != len1) { prefix = "[ERROR]"; }
    serialPrintln(prefix + " " + (100*len2/len1) + "%" + " of " + String(len1) + " bytes written to SD file '" + String(filename) + "'");

    sdcardTimeEnd = millis();
}

// blink the light emitting diode (LED):
void blinkLED(void) {
#ifndef BLINK_LED_IF_WRITING_OKAY
    return;
#endif
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(5);              // wait a little
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
}


// manage battery voltage logging:
void vbatlog(const char basename[], const char dateTime [])
{
    if( BATTERY_UPDATE_INTERVAL == 0 ) { return; }
    unsigned long batteryTimeStart = millis();
    unsigned long batteryTimeIdle = batteryTimeStart - batteryTimeEnd;
    if( batteryTimeIdle < BATTERY_UPDATE_INTERVAL ) { return; }
    writeBatteryVoltage(basename, dateTime);
    batteryTimeEnd = millis();
}


// write battery voltage to SD card:
void writeBatteryVoltage (const char basename[], const char dateTime[])
{
    char filename[FILENAME_LEN_MAX];
    strncpy(filename, basename, strlen(basename)+1);
    strcat(filename, ".bat");
    serialPrint(F("[DEBUG] filename (battery): "));  serialPrintln(filename);

    File file = sd.open(filename, FILE_WRITE);
    //File file = SD.open("test.bat", FILE_WRITE);  // DEBUG
    if (!file)
    {
        serialPrintln(F("[ERROR] Unable to open battery log file."));
        return;
    }

    file.print(dateTime);
    file.print("\t");
    file.print(readBatteryVoltage());
    file.println("");

    file.close();
}


// read battery voltage:
float readBatteryVoltage (void)
{
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    return measuredvbat;
}


// returns a pointer to the nth character in a string:
const char* nth_strchr(const char* s, int c, int n)
{
    int c_count;
    char* nth_ptr;

    for (c_count=1,nth_ptr=strchr(s,c);
         nth_ptr != NULL && c_count < n && c!=0;
         c_count++)
    {
         nth_ptr = strchr(nth_ptr+1, c);
    }

    return nth_ptr;
}


// calculate NMEA check-sum given a string:
uint8_t getCheckSum(char *string)
{
  int XOR = 0;    
  for (int i = 0; i < strlen(string); i++) {
      XOR = XOR ^ string[i];
  }
  return XOR;
}


// prepare command to configure NMEA update rate
void getNmeaUpdateCommand(char command[]){
    // examples:
    // PMTK_SET_NMEA_UPDATE_10HZ: "$PMTK220,100*2F\r\n"
    // PMTK_SET_NMEA_UPDATE_1HZ: "$PMTK220,1000*1F\r\n"
    // PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ: "$PMTK220,10000*2F\r\n"
    char message[12]="PMTK220,";
    char updateInterval[6];
    char checksum [sizeof(int)*2+1];
    
    itoa(GPS_UPDATE_INTERVAL, updateInterval, 10);
    strcat(message, updateInterval);
    sprintf(checksum, "%X", getCheckSum(message));

    strcpy(command,"$");
    strcat(command,message);
    strcat(command,"*");
    strcat(command,checksum);
    strcat(command,"\r\n");
}

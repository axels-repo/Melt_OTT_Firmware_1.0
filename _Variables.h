
#define RELAYsampler_PIN  A2       //PIN linked to the relay to power of the sampler  (relay1 = pin1 & relay2 = pin2)
#define pulse_duration    300        //duration of the pulse to trigger one sample in MILLISECONDS!!

#define default_coeffb    1.2       //default value
#define default_coeffc    0.6       //default value
#define TEST_LOOP             (false)     //Run test loop instead of actual loop
#define SLEEP_ENABLED         (true)    //Disable to keep serial coms alive for testing
#define FIRMWARE_VERSION      (201)
#define BV_OFFSET             (0.01)
#define ALS_POWER_WAIT        (1000) //Depends on probe. 1000 is safe
#define RTC_OFFSET_S          (12)
#define WRAP_AROUND_S_LOWER   (11)
#define WRAP_AROUND_S_UPPER   (49)
#define WRAP_AROUND_BUFF      (1)
#define ALS_AVE_COUNT         (11)  //Number of ADC reads for ALS averaging
#define ALS_AVE_DELAY         (0) //Period (mS) between ADC reads for ALS averaging. ALS takes around 10mS to poll regardless. 
#define TEMP_INIT_TIME        (100)
#define MAX_ALS_CAL_POINTS    (12)
#define MAX_REPEATED_NODES    (10)
#define RAIN_CUMULATIVE       (true)  //true is much better for telemetry monitoring
//Hardware pins
#define ONE_WIRE_BUS          (5)        //One wire temp sensors
#define RAIN_GAUGE_INT        (9)
#define DATA_PIN              (3)          // The pin of the SDI-12 data bus
#define SD_SPI_CS             (A4)
#define USS_ECHO              (A2)
#define USS_TRIG              (A3)
#define USS_INIT_TIME         (50)
#define LED                   (13)
#define FET_POWER             (4)
#define BATT                  (A5)
//I2C Addressing
#define ADC_ADDR              (0x48)
#define RTC_ADDR              (0x68)  //Set automatically, here as a reminder
//Other
#define WATCHDOG_TIMER_MS     (11)   //Valid values: 0-11. 11 gives 16s timeout. 10 gives 8s timeout and so on  //resetWDT(); as necessary
#define LORA_FREQUENCY        (919.9)
#define LORA_BANDWIDTH        (125000)  //Increase to 250000 or 500000 for increased speed, less range. Linear change.
#define default_vpulse    100.00    //default cumulated volume that start sampling [m3]
#define default_wband     100.00    //default water level difference [m] for the neutral zone, if (wlevel<wpulse-wband), sampling is stopped
#define default_wpulse    100.00    //default water level that start sampling [m]
#define default_timsamp   60;       //default duration between time-based sampling [min]
#define default_headsam   10;       //default head level to start the time-based sampling [m]
#define default_vcumax    300000;   //default maximum cumulated volume [m3]
#define pulse_duration    300        //duration of the pulse to trigger one sample in MILLISECONDS!!
String nodeid = "DFLT";
uint8_t sendCount = 0;


//CONFIG.txt variables
float coeffb=default_coeffb;        //coefficient used in the flow formulae (FLOAT)
float coeffc=default_coeffc;        //coefficient used in the flow formulae (FLOAT)
float off1 = 0;                     //value to offset 1 (distance end of weir - UPSTREAM probe) [m]
float off2 = 0;                     //value to offset 2 (distance end of weir - 0 on rule) [m]
long int counter = 0;
String NodeID;
double ALSslope = 1;
double ALSoffset = 0;
int LoRaPollOffset = 23;
int pollPerLoRa = 6;
String Nodes_to_repeat[MAX_REPEATED_NODES] = {"NULL"};
String project = "TST";
String siteID = "AA";
float raingaugeTip_mm = 0.2;
bool LoRaEnabled = true;
bool SDEnabled = true;
// int lastSDSize = 1;

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

typedef struct {
  uint8_t sensorCount = 0;
  double measure[10];
  double measure_2[10];
  uint8_t pollPeriod = 240;
  uint8_t cyclesPerPoll;
  uint8_t cycleCount;
  DeviceAddress addr[4];
} value_t;
value_t value;

typedef struct {
  float temp[MAX_ALS_CAL_POINTS] = {15};
  float slope[MAX_ALS_CAL_POINTS] = {1.28783};
  float offset[MAX_ALS_CAL_POINTS] = { -1290.16};
} ALSCal_t;
ALSCal_t ALSCal;

typedef struct {
  value_t rainGauge;
  value_t temp;
  value_t RTCTemp;
  value_t ALS;
  value_t OTT;
  value_t USS;
} sensor_t;
sensor_t sensor;

typedef struct {
  String packet;
  int RSSI;
} LoRaReceive_t;
LoRaReceive_t LoRaReceive;

//Initialise libraries
RH_RF95 rf95(12, 6);
uint8_t rfbuf[RH_RF95_MAX_MESSAGE_LEN];//
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);
DeviceAddress Thermometer;
DeviceAddress hello;

RTC_DS3231 rtc;
ADS1115 ads(ADC_ADDR);
SDI12 mySDI12(DATA_PIN);
RTCZero internalrtc;

String stptbs = "";                 //timestamp of the last time-based sampling in UTC
int samnum = 0;                     //number of samples taken for an event
int samon = 0;                      //1 if sampling is in process, 0 if no event to sample now
float flowunsub = 0;                //will be used to calculate the flow NOT submerged (even if it is not necessary) [m3/s]
float headsam=default_headsam;      //head level to trigger to the time-based sampling [m]
float wlev=0;                       //UPSTREAM water level used to validate the mesure [m]
float hlev=0;                       //head level UPSTREAM used to calculate the flow [m]
float batlevel = 0;
float otttemp = 0;                  //will be used for the water temperature from the OTT [C]
float flow = 0;                     //will be used to calculate the flow not submerged or submerged, the "correct" flow [m3/s]
float vevent = 0;                   //cumulated volume already gone through the weir SINCE SAMPLING HAS STARTED (Q.time)
String underwater="11";               //condition "11" = underwater / "00" = above water based on the swith info / "01" or "10" : switch do not agree
float vcumul = 0;                   //cumulated volume already gone through the weir (Q.time)
float vcumax = default_vcumax;      //MAXIMUM cumulated volume (default 300'000 m3)
float vpulse = default_vpulse;      //cumulative volume that will start the sampling (compared to vevent)
float wband = default_wband;        //stop sampling if water level below (wpulse-wband)
float wpulse = default_wpulse;      //start sampling process if water level > wpulse
int timsamp = default_timsamp;                   //(min) duration between time-based samples in minutes
float wlevdown=0;                   //DOWNSTREAM water level used to validate the mesure [m]
float hlevdown=0;                   //head level DOWNSTREAM used to calculate the flow [m]
float ottlev;                    //Raw OTT Measurement
String logFile="datalog.csv";
int pollPeriod;
File myFile;
File configFile;
String UNIXtimestamp;
String stampold = "";               //old timestamp of the previous mesure (used to calculated the volume that went through the weir)
String normTimestamp;
String fileNameStr;
String datastring;
int sleep_now_time;
int sleep_remaining_s = 0;
uint8_t tx_count = 0;
char addr[5];
char hex_chars[] = "0123456789ABCDEF";
int Year;
bool setupLoop = true;
// float nodeDesyncRatio;
int tarSec;
int loraPollCount = pollPerLoRa;
String CSVHeader;
int logIncrement = 97;  //ASCII lowercase a
int lastUpTime = 0;
int WakeTime = 0;
int SensWakeTime = 0;
bool dailyReset = false;
float rainfall_mm = 0;
volatile uint16_t tip_count = 0;
volatile bool rain_interrupt = false;
String flowused ="";                //tell which flow equation is used "weir eq." or "Sub. eq."

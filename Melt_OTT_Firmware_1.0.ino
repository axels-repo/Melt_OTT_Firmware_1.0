/*
  Main branch
  Kai James kaicjames@outlook.com
*/

#include "_Libs.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////    SETUP           ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup () {
  sensor.RTCTemp.sensorCount = 1;
  hexstr(getChipId(), addr, sizeof(addr));  //Establish default node ID. Overwrite if specified in config
  NodeID = String(addr);
  pinMode(FET_POWER, OUTPUT);

  if (rf95.init() == false) {
    while (1) {
      delay(50);
      if (rf95.init()) {
        break;
      }
    }  //Watchdog will reset
  } else {
    rf95.setTxPower(23, false);
    rf95.setFrequency(LORA_FREQUENCY);
    rf95.setSignalBandwidth(LORA_BANDWIDTH); //500kHz
  }
  sendLoRaIgnore(String(NodeID) + ": sLoRa started");

  //SD card
  if (!SD.begin(SD_SPI_CS)) {
    //SerialUSB.println("SD initialization failed!");
    crashNflash(1);  //10*10ms high period
  }
  OneWireTempSetup(); //Must auto detect sensors BEFORE configRead()


  counter = readFile("counter").toInt(); //loggerln("'----counter="+String(counter));
  off1 = readFile("off1").toFloat(); //loggerln("'----off1="+String(off1,4));
  off2 = readFile("off2").toFloat(); //loggerln("'----off2="+String(off2,4));
  coeffb = readFile("coeffb").toFloat(); //loggerln("'----coeffb="+String(coeffb,4));
  coeffc = readFile("coeffc").toFloat(); //loggerln("'----coeffc="+String(coeffc,4));
  vcumul = readFile("vcumul").toFloat(); //loggerln("'----vcumul="+String(vcumul,1));
  vcumax = readFile("vcumax").toFloat(); //loggerln("'----vcumax="+String(vcumax,1));
  vevent = readFile("vevent").toFloat(); //loggerln("'----vevent="+String(vevent,4));
  vpulse = readFile("vpulse").toFloat(); //loggerln("'----vpulse="+String(vpulse,4));
  wband = readFile("wband").toFloat(); //loggerln("'----wband="+String(wband,4));
  wpulse = readFile("wpulse").toFloat(); //loggerln("'----wpulse="+String(wpulse,4));
  timsamp = readFile("timsamp").toInt();     //loggerln("'----timsamp="+String(timsamp));
  stptbs = readFile("stptbs"); //loggerln("'----stptbs="+stptbs);
  headsam = readFile("headsam").toFloat(); //loggerln("'----headsam="+String(headsam,4));


  configRead();
  sendLoRaIgnore("Config updated");
  logFile = siteID + NodeID + ".csv";



  Wire.begin(); //Might not need this
  setupWDT( WATCHDOG_TIMER_MS );

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  //Ultra sonic
  pinMode(USS_TRIG, OUTPUT);
  pinMode(USS_ECHO, INPUT);
  digitalWrite(USS_TRIG, LOW);

  //OTT probe
  sendLoRaIgnore("About to start OTT");
  if (sensor.OTT.sensorCount > 0) {
    mySDI12.begin();
    sendLoRaIgnore("OTT Started");
  }
  sendLoRaIgnore("About to start ADC");
  ads.begin();
  ads.setGain(0);  // 6.144 volt
  sendLoRaIgnore("ADC started");
  //ALS Sensor
  sendLoRaIgnore("About to start RTC");
  //RTC
  if (! rtc.begin()) {
    crashNflash(2);
  }
  sendLoRaIgnore("RTC started");

  DateTime now = rtc.now();
  int Hour;
  int Min;
  int Sec;
  int Day;
  char Month[12];
  uint8_t monthIndex;
  bool updateTime;
  sscanf(__TIME__, "%d:%d:%d", &Hour, &Min, &Sec);
  sscanf(__DATE__, "%s %d %d", Month, &Day, &Year);
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  int uploadTime = Min + Hour * 60 + Day * 24 * 60 + (monthIndex + 1) * 24 * 60 * 31;
  int RTCTime = now.minute() + now.hour() * 60 + now.day() * 24 * 60 + now.month() * 24 * 60 * 31;
  if (now.year() <= Year || RTCTime < uploadTime) {
    if (Sec >= 60 - RTC_OFFSET_S) {
      Min = Min + 1;
      Sec = Sec + RTC_OFFSET_S - 60;
    }
    else {
      Sec = Sec + RTC_OFFSET_S;
    }
    rtc.adjust(DateTime(Year, monthIndex + 1, Day, Hour, Min, Sec));
  }

  updatePollFreq(); //Calculate sensor poll frequencies
  tarSec = LoRaPollOffset % (pollPeriod * 60); //Gives us the target start second for polling

  internalrtc.begin(false);
  internalrtc.attachInterrupt(wake_from_sleep);

  analogReference(AR_INTERNAL2V23); //For internal battery level calculation


  sendLoRaIgnore("Going to sleep to sync time");
  if (!TEST_LOOP) {
    sleepTillSynced();
  }
  String tmpStr = String("Node ") + NodeID + " ready for work. Time offset " + String(LoRaPollOffset, DEC) + " seconds";
  sendLoRaIgnore(tmpStr); //Quick message to say we've woken up


  setupWDT( WATCHDOG_TIMER_MS ); // initialize and activate WDT with maximum period
  //Rain gauge. Add last to ensure the interrupt doesn't interfere with initial time sync
  if (sensor.rainGauge.sensorCount > 0) {
    pinMode(RAIN_GAUGE_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RAIN_GAUGE_INT), rain_isr, FALLING);
    // Configure EIC to use GCLK1 which uses XOSC32K. Allows rise/fall interupt modes to work.
    SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |
                        GCLK_CLKCTRL_GEN_GCLK1 |
                        GCLK_CLKCTRL_CLKEN;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////      LOOP           ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop () {
  while (TEST_LOOP) {
    disableWatchdog();
    digitalWrite(LED, HIGH);
  }

  resetWDT();
  wake_system();


  DateTime now;
  if (sensor.rainGauge.sensorCount > 0) {
    rainfall_mm = tip_count * raingaugeTip_mm;
    if (RAIN_CUMULATIVE) {
      now = rtc.now();
      if (now.hour() == 0) {
        if (dailyReset) {
          tip_count = 0;
          dailyReset = false;
        }
      }
      else if (!dailyReset) {
        dailyReset = true;
      }
    }
    else {
      tip_count = 0;
    }
  }

  if (sensor.temp.sensorCount > 0) {
    if (sensor.temp.cycleCount == 1) { //Only test if we have woken the sensor and reset counter
      if (!tempUpdate()) {  //This and subsequent blank if statements are used in problem solving. Can defs remove them
      }
    }
  }
  if (sensor.RTCTemp.sensorCount > 0) {
    if (sensor.RTCTemp.cycleCount == 1) { //Only test if we have woken the sensor and reset counter
      if (!RTCTempUpdate()) {  //This and subsequent blank if statements are used in problem solving. Can defs remove them
      }
    }
  }

  if (sensor.ALS.sensorCount > 0) {
    if (sensor.ALS.cycleCount == 1) { //Only test if we have woken the sensor and reset counter
      if (!ALSUpdate()) {  //This and subsequent blank if statements are used in problem solving. Can defs remove them
      }
    }
  }

  if (sensor.OTT.sensorCount > 0) {
    if (sensor.OTT.cycleCount == 1) { //Only test if we have woken the sensor and reset counter
      if (!OTTUpdate()) {
        sendLoRaRaw("OTT not reading correctly");//This and subsequent blank if statements are used in problem solving. Can defs remove them
      }
    }
  }
  if (sensor.USS.sensorCount > 0) {
    if (sensor.USS.cycleCount == 1) { //Only test if we have woken the sensor and reset counter
      if (!USSUpdate()) {  //This and subsequent blank if statements are used in problem solving. Can defs remove them
      }
    }
  }

  batlevel = battVoltUpdate();
  sleep_system();
  stampold = UNIXtimestamp; //keep in memory the last timestamp of the previous measure
  buildTimestamps();
  buildCSVDataString();

  if (SDEnabled) {
    logDataToSD();
  }

  if (LoRaEnabled) {
    if (loraPollCount >= pollPerLoRa) {
      loraPollCount = 0;
      LoRaUpdate();
    }
    loraPollCount++;
  }

  TestSampling();     //test if sampling is necessary

  rf95.sleep();
  delay(20);    //Delay 20ms to ensure the chips have gone to sleep before powering off the board
  disableWatchdog(); // disable watchdog
  rain_interrupt = false;
  bool first_loop = true;
  now = rtc.now();
  lastUpTime = millis() - WakeTime;
  while ((first_loop || rain_interrupt)) {
    if (first_loop) { //REMEMBER first loop is relative to this wake cycle considering rain interupt. NOT first_loop for the whole system.
      first_loop = false;
      if ((tarSec < WRAP_AROUND_S_LOWER) && (now.second() >= 30))  { //Wrap down case
        sleep_remaining_s = pollPeriod * 60 + tarSec - (-60 + now.second()) + 1 + WRAP_AROUND_BUFF;
      }
      else if ((tarSec > WRAP_AROUND_S_UPPER) && (now.second() <= 30)) { //Wrap up case
        sleep_remaining_s = pollPeriod * 60 + tarSec - (60 + now.second()) - 1 + WRAP_AROUND_BUFF;
      }
      else {  //Standard time correction
        sleep_remaining_s = pollPeriod * 60 + tarSec - now.second() + WRAP_AROUND_BUFF;
      }
    }
    rain_interrupt = false;
    sleep();
  }
  WakeTime = millis();  //For wake period calculation
  setupWDT( WATCHDOG_TIMER_MS ); // initialize and activate WDT with maximum period
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////    FUNCTIONS         ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void updatePollFreq() {
  pollPeriod = min(min(min(min(min(sensor.temp.pollPeriod, sensor.RTCTemp.pollPeriod), sensor.ALS.pollPeriod), sensor.OTT.pollPeriod), sensor.USS.pollPeriod), sensor.rainGauge.pollPeriod);
  //Calculate cycles per poll
  sensor.rainGauge.cyclesPerPoll = sensor.rainGauge.pollPeriod / pollPeriod;
  sensor.temp.cyclesPerPoll = sensor.temp.pollPeriod / pollPeriod;
  sensor.RTCTemp.cyclesPerPoll = sensor.RTCTemp.pollPeriod / pollPeriod;
  sensor.ALS.cyclesPerPoll = sensor.ALS.pollPeriod / pollPeriod;
  sensor.OTT.cyclesPerPoll = sensor.OTT.pollPeriod / pollPeriod;
  sensor.USS.cyclesPerPoll = sensor.USS.pollPeriod / pollPeriod;
  //Init counter
  sensor.rainGauge.cycleCount = sensor.rainGauge.cyclesPerPoll;
  sensor.temp.cycleCount = sensor.temp.cyclesPerPoll;
  sensor.RTCTemp.cycleCount = sensor.RTCTemp.cyclesPerPoll;
  sensor.ALS.cycleCount = sensor.ALS.cyclesPerPoll;
  sensor.USS.cycleCount = sensor.USS.cyclesPerPoll;
  sensor.OTT.cycleCount = sensor.OTT.cyclesPerPoll;
}

void sleep() {
  sleep_now_time = internalrtc.getEpoch();
  if (sleep_remaining_s > 0) {
    if (SLEEP_ENABLED) {
      internalrtc.setAlarmEpoch(sleep_now_time + (sleep_remaining_s - 1));
      internalrtc.enableAlarm(internalrtc.MATCH_HHMMSS);
      SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
      SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
      __DSB();
      __WFI();
      SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    }
    else if (!SLEEP_ENABLED) {
      delay(sleep_remaining_s * 1000);
    }
    sleep_remaining_s = sleep_remaining_s - (internalrtc.getEpoch() - sleep_now_time); // Restarts clock in case of wake due to rain interupt
  }
}

uint16_t getChipId() {
  volatile uint32_t *ptr = (volatile uint32_t *)0x0080A048;
  return *ptr;
}

void hexstr(uint16_t v, char *buf, size_t Size) {
  uint8_t i;
  if (Size > 4) {
    for (i = 0; i < 4; i++) {
      buf[3 - i] = hex_chars[v >> (i * 4) & 0x0f];
    }
    buf[4] = '\0';
  }
}

static void   WDTsync() {
  while (WDT->STATUS.bit.SYNCBUSY == 1); //Just wait till WDT is free
}

void setupWDT( uint8_t period) {
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(5) | GCLK_GENDIV_DIV(4);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(5) |
                      GCLK_GENCTRL_GENEN |
                      GCLK_GENCTRL_SRC_OSCULP32K |
                      GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY);  // Syncronize write to GENCTRL reg.
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT |
                      GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN_GCLK5;
  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync(); // sync is required
  WDT->CONFIG.reg = min(period, 11); // see Table 17-5 Timeout Period (valid values 0-11)
  WDT->CTRL.reg = WDT_CTRL_ENABLE; //enable watchdog
  WDTsync();
}

void systemReset() {  // use the WDT watchdog timer to force a system reset.
  WDT->CLEAR.reg = 0x00; // system reset via WDT
  WDTsync();
}

void resetWDT() {
  WDT->CLEAR.reg = 0xA5; // reset the WDT
  WDTsync();
}

void wake_from_sleep() {  //ISR runs whenever system wakes up from RTC
}

void rain_isr() {
  tip_count++;
  rain_interrupt = true;

  // static unsigned long last_interrupt_time = 0;
  // unsigned long interrupt_time = millis();
  // // If interrupts come faster than 200ms, assume it's a bounce and ignore
  // if (interrupt_time - last_interrupt_time > 200)
  // {
  //   ... do your thing
  // }
  // last_interrupt_time = interrupt_time;

}

void LoRaUpdate() {
  char *pmsg;
  datastring = "PKT:" + normTimestamp + siteID + "," + siteID + NodeID + "," + datastring;
  pmsg = (char*)datastring.c_str();
  rf95.send((uint8_t *)pmsg, strlen(pmsg) + 1);
  rf95.waitPacketSent();    //This takes 189ms
}

void sendLoRaRaw(String msg) {
  char *ppmsg;
  ppmsg = (char*)msg.c_str();
  rf95.send((uint8_t *)ppmsg, strlen(ppmsg) + 1);
  rf95.waitPacketSent();
}

void debug(String msg) {
  char *ppmsg;
  msg = "DBG:" + String(NodeID) + ":" + msg;
  ppmsg = (char*)msg.c_str();
  rf95.send((uint8_t *)ppmsg, strlen(ppmsg) + 1);
  rf95.waitPacketSent();
}

void sendLoRaIgnore(String msg) { //Thrown out at the gateway
  char *ppmsg;
  msg = "IGN:" + String(NodeID) + ":" + msg;
  ppmsg = (char*)msg.c_str();
  rf95.send((uint8_t *)ppmsg, strlen(ppmsg) + 1);
  rf95.waitPacketSent();
}

float battVoltUpdate() {
  float BATT_LVL = analogRead(BATT);
  BATT_LVL = BATT_LVL / 1024 * 2.23 * 2 + BV_OFFSET;
  return BATT_LVL;
}

bool tempUpdate() { //Make sure battery power is connected.
  tempSensors.requestTemperatures();
  for (int i = 0;  i < sensor.temp.sensorCount;  i++) {
    sensor.temp.measure[i] = tempSensors.getTempC(sensor.temp.addr[i]);
  }
  return true;
}

bool ALSUpdate() {
  double f = ads.toVoltage(1);
  int wait = ALS_POWER_WAIT - (millis() - SensWakeTime);
  delay(wait);

  for (int i = 0; i < sensor.ALS.sensorCount ; i++) {
    sensor.ALS.measure[i] = 0;
    for (int k = 0; k < ALS_AVE_COUNT; k++) {

      sensor.ALS.measure[i] += ads.readADC(i) * f * 1000;

      delay(ALS_AVE_DELAY);
    }
    sensor.ALS.measure[i] = sensor.ALS.measure[i] / ALS_AVE_COUNT; //true makes a call, only do it once per poll session
  }

  return true;
}

bool OTTUpdate() {
  String myComAdress = "?!";
  String address;
  String myComId = "0I!";
  String myComSend = "0D0!";
  String myComMeasure = "0M!";

  mySDI12.begin(); delay(1000); //start SDI communication and wait a little for the probe to be ready
  mySDI12.sendCommand(myComMeasure);
  delay(300); //loggerln("'----Start measure, "+readsensor());
  delay(1300); mySDI12.clearBuffer(); //time necessary to wait for the measure
  mySDI12.sendCommand(myComSend); delay(300);
  String rawdata = readSDI12();
  delay(500);
  mySDI12.clearBuffer();
  //Decoding string sent from probe
  int p = 0;
  int pos[] = {0, 0};
  for (int z = 0 ; z < rawdata.length() ; z++)  {
    char u = rawdata.charAt(z);
    if (u == '+' || u == '-') {
      pos[p] = z ;
      p++;
    }
    delay (50);
  }
  counter += 1;
  String level = rawdata.substring(pos[0], pos[1]);
  String temp = rawdata.substring(pos[1], rawdata.length());
  ottlev = level.toFloat();
  otttemp = temp.toFloat();
  wlev = ottlev + off1;      //new change in v19.04 to make off1 the difference between ottlev and staff gauge zero
  hlev = max(0, wlev - off2); //new change in v19.04 off2 is the weir invert level as read on the staff gauge
  if (hlev < 0.250) {
    flowunsub = coeffc * 0.53333 * tan(1.570796 / 2) * sqrt(2 * 9.8) * pow(hlev, 2.5); //weir coefficient Ce *(8/15)*(TAN(RADIANS(90)/2))*SQRT((2*9.8))*(hlev^(5/2))
  } else {
    flowunsub = (coeffc * 0.53333 * tan(1.570796 / 2) * sqrt(2 * 9.8) * pow(0.250, 2.5)) + (coeffc * 0.66666 * sqrt(2 * 9.8) * 2.600 * pow((hlev - 0.25), 1.5));
  };
  flow = flowunsub; // if not both condition, then "classical" flow from the weir (unsubmerged)
  flowused = "weir eq."; //to tell which flow equation is used "weir eq." or "sub. eq."

  if (isnan(flow)) {
    flow = 0; //if the flow is nan (probably because it is a power of a negative value because UP head < DOWN head), then force it to zero
  }
  if (stampold.toInt() != 0) { //calculation of the cumulated volume
    if (isnan(vcumul)) {
      vcumul = 0;
    }
    vcumul = vcumul + (UNIXtimestamp.toInt() - stampold.toInt()) * flow;
    if (vcumul > vcumax) {
      vcumul = 0; // to reset the cumulated volume when it becomes too big (every 20'000 m3)
    }
    if (isnan(vevent)) {
      vevent = 0; //if the event volume is nan, force it to zero
    }
    if (samon == 1 && hlev <= headsam) {
      vevent = vevent + (UNIXtimestamp.toInt() - stampold.toInt()) * flow; //increase cumulated volume for event if sampling in progress and head level is not too high
    }
  }
  return true;
}

String readSDI12() {
  String sdiResponse = "";
  delay(30);
  while (mySDI12.available()) {  // write the response to the screen
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(5);
    }
  }
  return sdiResponse;
}

bool RTCTempUpdate() {
  sensor.RTCTemp.measure[0] = rtc.getTemperature();
  return true;
}

bool USSUpdate() {
  digitalWrite(FET_POWER, LOW);
  delay(USS_INIT_TIME);
  digitalWrite(USS_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(USS_TRIG, HIGH);
  delayMicroseconds(20);
  digitalWrite(USS_TRIG, LOW);
  double duration = pulseIn(USS_ECHO, HIGH);
  float dist = duration / 2 * 0.000343;
  delay(50);
  sensor.USS.measure[0] = dist;   //Should return duration and have temp compensation
  digitalWrite(FET_POWER, LOW);
  return true;
}

void logDataToSD() {
  String msg = "Saving data, ";
  String csvObject = "";
  ChangeParameter ("counter" , String(counter));
  if (!SD.exists(logFile)) {
     //csvObject = "Site_name,DateTime [DDMMYY HH:MM:SS],Timestamp[s],Signal_strength[/31],Measure_number[-],Battery_level[V],UP_Head_level[m],UP_Water_level[m],Water_temp[C],Flow [m3/s],Vol_cumul_event[m3],Vol_cumul_total[m3],Weir_under_water [1=yes/0=no],DWN_water_level[m],DOWN_Head_level[m],Flow-eq_used[weir or submerged eq.]\n";
    csvObject = "Site_name,DateTime [DDMMYY HH:MM:SS],Timestamp[s],Signal_strength[/31],Measure_number[-],Battery_level[V],UP_Head_level[m],UP_Water_level[m],Water_temp[C],Flow [m3/s],Vol_cumul_event[m3],Vol_cumul_total[m3],Weir_under_water [1=yes/0=no],DWN_water_level[m],DOWN_Head_level[m],Flow-eq_used[weir or submerged eq.],Raw_OTT_Measurement\n";
  }
  myFile = SD.open(logFile, FILE_WRITE);
  csvObject = csvObject + siteID + NodeID + "," + normTimestamp + UNIXtimestamp + "," + String(sensor.USS.measure[0], 2) + "," + String(counter) + "," + String(batlevel) + "," + String(hlev, 3) + "," + String(wlev, 3) + "," + String(otttemp) + "," + String(flow, 4) + "," + String(vevent, 4) + "," + String(vcumul, 4) + "," + underwater + "," + String(wlevdown, 3) + "," + String(hlevdown, 3) + "," + flowused + "," + String(ottlev, 3);

  if (myFile) {
    myFile.println(csvObject);
    msg = msg + "done!";
    myFile.close();
  }
  else {
    msg = msg + "error opening file.";
  }
}

void crashNflash(int identifier) {
  while (1) {
    for (int k = 0; k < identifier; k++) {
      digitalWrite(LED, HIGH);
      delay(50);
      digitalWrite(LED, LOW);
      delay(300);
    }
    delay(2000);
  }
}

void buildTimestamps() {
  String day;
  String month;
  String hour;
  String minute;
  String second;

  DateTime now = rtc.now();
  if ((tarSec < WRAP_AROUND_S_LOWER) && (now.second() >= 30)) { //We will favor being forward in time, so this won't happen often
    delay((60 - int(now.second())) * 1000); //Wait until next second
    now = rtc.now();
    debug("Early wake, wasting power while waiting");//Wrap around occurring. Add 1 sec buffer to sleep time
    // logFile = SD.open((char*)fileNameStr.c_str(), FILE_WRITE);
    // logFile.println("Early wake, wasting power while waiting");
    // logFile.close();
  }
  if ((tarSec >= WRAP_AROUND_S_UPPER) && (now.second() <= 30)) { //We will favor being forward in time, so this won't happen often
    now = rtc.now();
    debug("Late wake, RTC time in next minute");
    // logFile = SD.open((char*)fileNameStr.c_str(), FILE_WRITE);
    // logFile.println("Late wake, RTC time in next minute");
    // logFile.close();
  }
  // if (tarSec < 10) {
  //   second = "0" + String(tarSec);
  // } else {
  //   second = String(tarSec);
  // }
  // if (now.second() < 10) {
  //   second = "0" + String(now.second(),DEC);
  // } else {
  //   second = String(now.second(),DEC);
  // }
  if (tarSec < 10) {
    second = "0" + String(tarSec, DEC);
  } else {
    second = String(tarSec, DEC);
  }
  if (now.day() < 10) {
    day = "0" + String(now.day(), DEC);
  } else {
    day = String(now.day(), DEC);
  }
  if (now.month() < 10) {
    month = "0" + String(now.month(), DEC);
  } else {
    month = String(now.month(), DEC);
  }
  if (now.hour() < 10) {
    hour = "0" + String(now.hour(), DEC);
  } else {
    hour = String(now.hour(), DEC);
  }
  if (now.minute() < 10) {
    minute = "0" + String(now.minute(), DEC);
  } else {
    minute = String(now.minute(), DEC);
  }
  normTimestamp = day + "/" + month + "/" + String(now.year(), DEC) + " " + hour + ":" + minute + ":" + second + ",";
  UNIXtimestamp = String(now.unixtime());
}



void configRead() {
  String configFileName = "CFG_" + String(FIRMWARE_VERSION) + ".txt";
  String key;
  String value;
  String line;
  char * pch;
  if (!SD.exists(configFileName)) {
    crashNflash(3);
  }
  configFile = SD.open((char*)configFileName.c_str());
  while (configFile.available()) {
    line = configFile.readStringUntil('\n');
    key = strtok((char*)line.c_str(), "=");
    value = strtok(NULL, ";");
    if (value != "DEFAULT" && value != NULL) {
      key.trim();
      value.trim();
      if (key == "Node_ID") {
        NodeID = value;
      } else if (key == "ALS_Slope") {
        ALSslope = value.toDouble();
      } else if (key == "ALS_Offset") {
        ALSoffset = value.toDouble();
      } else if (key == "LoRaPollOffset") {
        LoRaPollOffset = value.toInt();
      } else if (key == "PollPerLoRa") {
        pollPerLoRa = value.toInt();
      } else if (key == "Project_ID") {
        project = value;
      } else if (key == "Site_ID") {
        siteID = value;
      }
      else if (key == "Raingauge_mm") {
        raingaugeTip_mm = value.toFloat();
      } else if (key == "LoRa_EN") {
        LoRaEnabled = value.toInt();
      } else if (key == "SD_EN") {
        SDEnabled = value.toInt();
      } else if (key == "Nodes_to_repeat") {
        int i = 0;
        pch = strtok((char*)value.c_str(), "{},");
        while (pch != NULL) {
          Nodes_to_repeat[i] = pch;
          sendLoRaIgnore("Repeating: " + Nodes_to_repeat[i]);
          pch = strtok (NULL, ",{}");
          i++;
        }
      } else if (key == "ALS_Count") {
        sensor.ALS.sensorCount = value.toInt();
      } else if (key == "ALS_Period") {
        if (sensor.ALS.sensorCount != 0) {
          sensor.ALS.pollPeriod = value.toInt();
        }
      } else if (key == "Temp_Count") {
        // sensor.temp.sensorCount = value.toInt(); Auto detected
      } else if (key == "Temp_Period") {
        if (sensor.temp.sensorCount != 0) {
          sensor.temp.pollPeriod = value.toInt();
        }
      } else if (key == "USS_Count") {
        sensor.USS.sensorCount = value.toInt();
      } else if (key == "USS_Period") {
        if (sensor.USS.sensorCount != 0) {
          sensor.USS.pollPeriod = value.toInt();
        }
      } else if (key == "OTT_Count") {
        sensor.OTT.sensorCount = value.toInt();
      } else if (key == "OTT_Period") {
        if (sensor.OTT.sensorCount != 0) {
          sensor.OTT.pollPeriod = value.toInt();
        }
      } else if (key == "Raingauge_Count") {
        sensor.rainGauge.sensorCount = value.toInt();
      } else if (key == "Raingauge_Period") {
        if (sensor.rainGauge.sensorCount != 0) {
          sensor.rainGauge.pollPeriod = value.toInt();
        }
      } else if (key == "RTC_Temp_Count") {
        sensor.RTCTemp.sensorCount = value.toInt();
      } else if (key == "RTC_Temp_Period") {
        if (sensor.RTCTemp.sensorCount != 0) {
          sensor.RTCTemp.pollPeriod = value.toInt();
        }
      }
      else if (key == "ALS_cal_temps") {
        int i = 0;
        pch = strtok((char*)value.c_str(), "{},");
        while (pch != NULL) {
          ALSCal.temp[i] = atof(pch);
          sendLoRaIgnore("Temp " + String(i) + " = " + String(ALSCal.temp[i]));
          pch = strtok (NULL, ",{}");
          i++;
        }
      }
      else if (key == "ALS_cal_slopes") {
        int i = 0;
        pch = strtok((char*)value.c_str(), "{},");
        while (pch != NULL) {
          ALSCal.slope[i] = atof(pch);
          sendLoRaIgnore("slope " + String(i) + " = " + String(ALSCal.slope[i]));
          pch = strtok (NULL, ",{}");
          i++;
        }
      }
      else if (key == "ALS_cal_offsets") {
        int i = 0;
        pch = strtok((char*)value.c_str(), "{},");
        while (pch != NULL) {
          ALSCal.offset[i] = atof(pch);
          sendLoRaIgnore("Offset " + String(i) + " = " + String(ALSCal.offset[i]));
          pch = strtok (NULL, ",{}");
          i++;
        }
      }
    }

  }
  configFile.close();
}

void wake_system() {
  digitalWrite(FET_POWER, LOW);
  SensWakeTime = millis();
}

void sleep_system() {
  digitalWrite(FET_POWER, HIGH);
}

void disableWatchdog() {
  WDT->CTRL.reg = 0;
}

void sleepTillSynced() { //Initial clock sync
  rf95.sleep();
  delay(20);    //Delay 20ms to ensure the chips have gone to sleep before powering off the board
  WDT->CTRL.reg = 0; // disable watchdog
  DateTime now = rtc.now();
  // sleep_remaining_s = pollPeriod*60 - (now.minute()%pollPeriod)*60 - now.second();
  int relativeTimeS = (now.minute() * 60 + now.second()) % (pollPeriod * pollPerLoRa * 60);
  if (relativeTimeS > LoRaPollOffset) {
    sleep_remaining_s = pollPeriod * pollPerLoRa * 60 - relativeTimeS + LoRaPollOffset;
  }
  else if (relativeTimeS < LoRaPollOffset) {
    sleep_remaining_s = LoRaPollOffset - relativeTimeS;
  }
  sleep();
}

void convertALS() {
  double Slope;
  double Offset;
  float linearInterpRatio;

  for (int i = 0; i < sensor.ALS.sensorCount; i++) {    //Per each ALS sensor
    if (sensor.temp.measure[i] < ALSCal.temp[0]) { //Temperature below minimum
      Slope = ALSCal.slope[0];
      Offset = ALSCal.offset[0];
      // debug("Below min calibrated ALS temp");
    }
    else {
      for (int j = 1; j < MAX_ALS_CAL_POINTS - 1 ; j++) {
        if (ALSCal.temp[j] == 0) { //Temperature above maximum
          Slope = ALSCal.slope[j - 1];
          Offset = ALSCal.offset[j - 1];
          // debug("Above max calibrated ALS temp");
          break;
        }
        else if (sensor.temp.measure[i] < ALSCal.temp[j]) {
          linearInterpRatio = (sensor.temp.measure[i] - ALSCal.temp[j - 1]) / (ALSCal.temp[j] - ALSCal.temp[j - 1]);
          Slope = linearInterpRatio * (ALSCal.slope[j] - ALSCal.slope[j - 1]) + ALSCal.slope[j - 1];
          Offset = linearInterpRatio * (ALSCal.offset[j] - ALSCal.offset[j - 1]) + ALSCal.offset[j - 1];
          break;
        }
      }
    }
    sensor.ALS.measure_2[i] = sensor.ALS.measure[i] * Slope + Offset;
  }
}

bool LoRaListen(int timeout_S) {
  // Example use
  // if(LoRaListen(10)){;   //Enter 999 for no timeout
  // SerialUSB.println(LoRaReceive.RSSI);
  // SerialUSB.println(LoRaReceive.packet);}

  uint8_t len;
  int startTime = millis();
  while ((millis() - startTime) < (timeout_S * 1000) || timeout_S == 999) {
    while (rf95.available()) {
      len = sizeof(rfbuf);
      if (rf95.recv(rfbuf, &len)) {
        LoRaReceive.packet = (char*)rfbuf;
        LoRaReceive.RSSI = rf95.lastRssi();
      }
      return true;
    }
  }
  return false;
}

void bubbleSort(int a[], int arrayIndex[], int size) {
  for (int i = 0; i < (size - 1); i++) {
    for (int o = 0; o < (size - (i + 1)); o++) {
      if (a[o] > a[o + 1]) {
        int t = a[o];
        a[o] = a[o + 1];
        a[o + 1] = t;

        int tmp = arrayIndex[o];
        arrayIndex[o] = arrayIndex[o + 1];
        arrayIndex[o + 1] = tmp;
      }
    }
  }
}

void OneWireTempSetup() {
  int humanVal[4];
  digitalWrite(FET_POWER, LOW); //Turn on sensor
  delay(TEMP_INIT_TIME);  //Wait till we are warmed up
  tempSensors.begin();  // Start up the library
  sensor.temp.sensorCount = tempSensors.getDeviceCount(); //Check how many devices are attached
  sendLoRaIgnore("Temp sens count: " + String(sensor.temp.sensorCount));
  tempSensors.requestTemperatures();
  for (int i = 0;  i < sensor.temp.sensorCount;  i++) { //For each sensor, grab the value we will label with (Nibble 1 and 7 of full device address)
    tempSensors.getAddress(Thermometer, i);
    humanVal[i] = 256 * Thermometer[1] + Thermometer[2];
    sendLoRaIgnore("Temp_Address_" + String(i) + " = " + String(humanVal[i]) + ". Temp = " + String(tempSensors.getTempCByIndex(i)));
  }
  int arrayIndex[4] = {0, 1, 2, 3}; //This is how we will keep track of the order of the devices after sorting
  bubbleSort(humanVal, arrayIndex, sensor.temp.sensorCount);  //Sorts from smallest to largest. arrayIndex tells us where each value moved.
  for (int i = 0;  i < sensor.temp.sensorCount;  i++) { //For each sensor
    // tempSensors.getAddress(Thermometer, i);   //Read address
    tempSensors.getAddress(sensor.temp.addr[i], arrayIndex[i]);   //Assign addresses so that temp_1 will be the smallest address. Associate with relevant ALS probe.
    sendLoRaIgnore("HumanValOrdered_" + String(i) + " = " + String(humanVal[i]));

  }
  digitalWrite(FET_POWER, HIGH);  //Switch off temp sensor
}

String readFile(String nameFile) { //read the content of a file from the SD card
  sendLoRaIgnore("'--Reading file " + nameFile + ".txt: ");
  String result;
  nameFile = nameFile + ".txt";
  if (SD.exists(nameFile)) {
    //    msg=msg+"file exists! ";
    myFile = SD.open(nameFile);
    result = "";
    while (myFile.available()) {
      char c = myFile.read();
      if ((c != '\n') && (c != '\r')) {
        result += c;
        delay(5);
      }
    }
  }
  else {
    result = "0"; //default value to the parameter if there is no value in the SD card
    //    msg=msg+"no file? now good! ";
    myFile = SD.open(nameFile, FILE_WRITE);
    if (myFile) {
      if (nameFile == "coeffb.txt") {
        result = default_coeffb; //coefficient B by defaut
      }
      if (nameFile == "coeffc.txt") {
        result = default_coeffc; //coefficient C by defaut
      }
      if (nameFile == "headsam.txt") {
        result = default_headsam; //defaut head level trigger to start the time-based sample [m]
      }
      if (nameFile == "timsamp.txt") {
        result = default_timsamp; //default (min) duration between time-based samples in minutes
      }
      if (nameFile == "vpulse.txt") {
        result = default_vpulse; //defaut cumulative volume needed to start the sampling
      }
      if (nameFile == "vcumax.txt") {
        result = default_vcumax; //defaut MAX cumulative volume (for vcumul)
      }
      if (nameFile == "wband.txt")  {
        result = default_wband; //defaut height diff to stop the sampling
      }
      if (nameFile == "wpulse.txt") {
        result = default_wpulse; //defaut water level needed to start the sampling
      }
      myFile.println(result);
      myFile.close();
    }
  }
  //  loggerln(msg);
  return result;
}

void buildCSVDataString() {
  datastring = String(FIRMWARE_VERSION) + "," + String(counter) + "," + String(batlevel) + "," + String(flow, 4) + "," + String(vevent, 0) + "," + String(wlev, 3) + "," + String(hlev, 3) + "," + String(ottlev, 3) + "," + String(sensor.RTCTemp.measure[0]);
}


void ChangeParameter (String nameparameter , String value) { //all values should be given as string
  String name = nameparameter + ".txt";
  if (SD.exists(name)) {
    SD.remove(name);
  }
  myFile = SD.open(name, FILE_WRITE);
  if (myFile) {
    myFile.println(value);
    myFile.close();
  }
}

void StartSampling() { //start the sampling procedure
  //  To Be Implemented
  //  digitalWrite(RELAYsampler_PIN, HIGH); //start sampling with a pulse to the sampler
  //  delay(pulse_duration); //pulse in milliseconds
  //  digitalWrite(RELAYsampler_PIN, LOW); //stop sampling
  //  delay(1000); //wait a little, why not?
  //  samnum += 1;
  //  loggerln("Start sampling #" + String(samnum));
  //  lastaction = "StartSampling-2";
  //  //  String GSorder=String("\"TYP\":\"Order\",\"FRO\":\""+myphone+"\",\"MSG\":\""+sampletype+String(samnum)+"\"}");
  //  String GSorder = String(myphone + "," + sampletype + String(samnum));
  //  PubToGS(GSorder);
  //  lastaction = "StartSampling-3";
}

void TestSampling() { //test if sampling if required
  if (wlev > wpulse && samon == 0) { // if level reached + day ISNOT Friday (5) or Saturday (6) in UTC time (Melbourne -10 hours or -8 hours)
    StartSampling(); //start the sampler
    samon = 1; //sampling process started
    vevent = 0; //ready to monitor the cumulated volume of the event
    buildTimestamps();
    stptbs = UNIXtimestamp;
    ChangeParameter ("stptbs" , stptbs); //timestamp of the last sample (time-based sampling), to avoid another sampling just after (next condition)
  }
  if (samon == 1 && hlev > headsam && (UNIXtimestamp.toInt() - stptbs.toInt() >= (timsamp * 60))) { // if the head level is above the trigger head level and if the previous sample is too old, start the time-based sampling
    buildTimestamps();
    stptbs = UNIXtimestamp;
    ChangeParameter ("stptbs" , stptbs); //timestamp of the last sample (time-based sampling)
    StartSampling();
  } else {
    while (samon == 1 && (wlev > (wpulse - wband)) && vevent > vpulse) { //volume-based sampling
      StartSampling();
      vevent = vevent - vpulse;

      // Need to modify sending this data over LoRa

      // PublishData(); //publish the data online, same counter but new cumulated volume

      buildTimestamps();
      logDataToSD(); //save the data on the SD card, same counter but new cumulated volume
      stptbs = UNIXtimestamp;
      ChangeParameter ("stptbs" , stptbs); //timestamp of the last sample (time-based sampling), to avoid another sampling 6 mn later when the sampling change to time-based
      if (vevent > vpulse) {
        delay(5000); //wait 5 seconds because there will be another pulse for sampling
      }

    }
  }
  if (samon == 1 && (wlev <= (wpulse - wband))) {
    samon = 0;
    vevent = 0; //ready to monitor the cumulated volume of the next event
  }
}

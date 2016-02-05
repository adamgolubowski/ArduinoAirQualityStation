//================================================================================
// Station code - Air Quality Station
// This is code for an environmental station for handling data from sensors and
// sending the data to a web service.
//
// Work cycle:
//  start
//    synchronize time using NTP service
//    for each sensor:  
//      get values from sensor
//      send data to the web service
//    sleep
//  repeat the cycle
//    
// Author: Adam Gołubowski
// Date: 2016-01-13
//================================================================================

#include <EtherCard.h>  // https://github.com/jcw/ethercard
#include <Time.h>       // https://github.com/PaulStoffregen/Time
#include <Timezone.h>   // https://github.com/JChristensen/Timezone
#include "DHT.h"        //https://github.com/adafruit/DHT-sensor-library

//================================================================================
// Global settings
//================================================================================

// measurement cycle interval - time between measurement cycles start
#define cycleIntervalMin  15

// ip lease time in min. After that time it may be required to renew the ip lease
#define ipLeaseTime 10

// Sensor pins
#define DHT_pin     (5) // digital pin
#define MQ131       (0) // analog pin
#define MQ9         (1) // analog pin
#define dust_analog (2) // analog pin
#define dust_iled   (7) // digital pin, drives dust sensor led

// transmission buffer
byte Ethernet::buffer[350];
// stash represents access to memory on ENC28J60 chip
Stash stash;

// url of a web service
const char serviceUrl[] PROGMEM= "stationservice.azurewebsites.net";
// station API key - used for identification of a station - required for post method
const char stationKey[] = "KLHG6QKLGCEHTWKGNQJ2PDYL68GQWHBN";

// This is to indicate if there was a successfull synch at startup
boolean startupSynchDone = false;

// Dust sensor settings
#define COV_RATIO         0.2            //ug/mmm / mv
#define NO_DUST_VOLTAGE   400            //mv
#define SYS_VOLTAGE       5000           

// DHT21 sensor settings
#define DHTTYPE DHT21                    // DHT 21 (AM2301)
DHT dht(DHT_pin, DHTTYPE);

// Sensors identifiers for the service
const char dustSensorId[] = "21";
const char tempSensorId[] = "31";
const char humiSensorId[] = "41";
const char coSensorId[] = "1";
const char o3SensorId[] = "11";


void connectToInternet(){
  // ethernet interface mac address
  static byte mymac[]={0x00,0x04,0xA3,0x2D,0x30,0x31};
  if (ether.begin(sizeof Ethernet::buffer, mymac) == 0);
  ether.dhcpSetup();
  ether.dnsLookup(serviceUrl);
  ether.printIp("SRV: ", ether.hisip);
}

void setup () {
  Serial.begin(57600);
  connectToInternet();
}

//================================================================================
// This function gets current time using Network Time Protocol
// Forked from http://blog.cuyahoga.co.uk/2012/06/whats-the-time-mr-wolf/
// getNTPTime returns time value corrected to Central European time.
// The function will return 0 if it will not obtain time within a limited period
//================================================================================
uint8_t ntpServer[4] = { 130, 159, 196, 117 }; 
uint8_t ntpMyPort = 123; 
 
boolean getNTPTime() {
  // Time zone change rules. Taken from https://github.com/JChristensen/Timezone/blob/master/examples/WorldClock/WorldClock.pde
  // Central European Time (Frankfurt, Paris)
  TimeChangeRule CEST  = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
  TimeChangeRule CET  = {"CET ", Last, Sun, Oct, 3, 60};       //Central European Standard Time
  Timezone CE(CEST, CET);
  uint16_t n = 0;
  unsigned long timeFromNTP;   
  const unsigned long seventy_years = 2208988800UL;      
  ether.ntpRequest(ntpServer, ntpMyPort);   
  //limit time for waiting to synchronize. Break if waiting too long;
  while(n<65535) {      
    word length = ether.packetReceive();       
    ether.packetLoop(length);       
    if(length > 0 && ether.ntpProcessAnswer(&timeFromNTP, ntpMyPort)) {
      setTime(CE.toLocal(timeFromNTP - seventy_years));
      return true;
    }
    n++;
  }
  // In case of failed synchronization within a set period
  return false;
}


//================================================================================
// send senrsor data. The function takes sensor value and sends it to the
// web service using http post method. 
//  value       -  char value of the reading, e.g. "2.3";
//  equipmentID -  char identifier of a sensor, e.g. "1"; 
//================================================================================
void sendData (const char *value, const char *equipmentID) {
  Serial.println(equipmentID);
  // get timestamp in ISO format from current Arduino's time
  char timeStamp[19];
  sprintf(timeStamp,"%d-%02d-%02dT%02d:%02d:%02d",year(),month(),day(),hour(),minute(),second());
  // memory buffer for message body data
  char bodyBuffer[140];
  // build message body and place it in the buffer
  sprintf(bodyBuffer,"{\"TimeStamp\": \"%s\",\"Value\": %s,\"equipmentID\": %s,\"Key\": \"%s\"}",timeStamp,value,equipmentID,stationKey);
  Serial.println(bodyBuffer);
  // prepare memory space for data to post
  byte sd = stash.create();
  stash.print(bodyBuffer);
  stash.save();
  int stash_size=stash.size();

  // prepare post message
  Stash::prepare(PSTR("POST http://stationservice.azurewebsites.net/api/datapoints HTTP/1.0" "\r\n"
    "Host: stationservice.azurewebsites.net""\r\n"
    "Content-Type: application/json""\r\n"
    "Content-Length: $D""\r\n"
    "\r\n"
    "$H"),stash_size,sd);
  
  // send the packet - this also releases all stash buffers once done
  //ether.tcpSend();
  //ether.packetLoop(ether.packetReceive());

  byte session = ether.tcpSend();
  uint16_t n = 0;
  while(n<65535){
    ether.packetLoop(ether.packetReceive());
      const char* reply = ether.tcpReply(session);
      if (reply != 0) {
      //Serial.println("Reading a response:");
      Serial.println(reply);
      //return 1;
    }
    n++;
  }
  //Serial.println("No response received. Will not wait longer.");
}


//================================================================================
// Dust sensor functions
// Based on code from WaveShare: http://www.waveshare.com/w/upload/2/2a/Dust-Sensor-code.7z
// getDustReadings is the main function. Returns float number representing dust 
// concentration in an ambient air.
// Filter function is used for converting raw dust readings in the main function. 
//================================================================================

int Filter(int m)
{
  static int flag_first = 0, _buff[10], sum;
  const int _buff_max = 10;
  int i;
  if(flag_first == 0)
  {
    flag_first = 1;
    for(i = 0, sum = 0; i < _buff_max; i++)
    {
      _buff[i] = m;
      sum += _buff[i];
    }
    return m;
  }
  else
  {
    sum -= _buff[0];
    for(i = 0; i < (_buff_max - 1); i++)
    {
      _buff[i] = _buff[i + 1];
    }
    _buff[9] = m;
    sum += _buff[9];
    i = sum / 10.0;
    return i;
  }
}

float getDustReadings(){
  // I/O define
  float density, voltage;
  int   adcvalue;
  //setup for dust sensor
  pinMode(dust_iled, OUTPUT);
  digitalWrite(dust_iled, LOW);      //iled default closed
  /*  get adcvalue  */
  digitalWrite(dust_iled, HIGH);
  delayMicroseconds(280);
  adcvalue = analogRead(dust_analog);
  digitalWrite(dust_iled, LOW);
  adcvalue = Filter(adcvalue);
  /*  covert voltage (mv)  */
  voltage = (SYS_VOLTAGE / 1024.0) * adcvalue * 11;
  /*  voltage to density  */
  if(voltage >= NO_DUST_VOLTAGE)
  {
    voltage -= NO_DUST_VOLTAGE;
    density = voltage * COV_RATIO;
  }
  else density = 0;
  return density;
}

//================================================================================
// DHT 21 sensor reading functions: temperature and humidity
// Delay is required to get reliable measures.
//================================================================================
float getTemperature(){
  dht.begin();
  delay(2000);
  return dht.readTemperature();
}

float getHumidity(){
  dht.begin();
  delay(2000);
  return dht.readHumidity();
}

//================================================================================
// MQ sensor readings 
// Basic measurement function. Returns voltage value that corresponds to gas 
// concentration in the air.
// Based on http://www.seeedstudio.com/wiki/Grove_-_Gas_Sensor(MQ9)
//================================================================================
float getMqSensorReading(int sensorPin){
  float sensor_volt; 
  float sensorValue;
  sensorValue = analogRead(sensorPin);
  sensor_volt = sensorValue/1024*5.0;
  return sensor_volt;
}

//================================================================================
// Helper function for converting float values to chars
//================================================================================
char * convertToChar(float value){
  static char buf[10];
  dtostrf(value,7,2,buf);
  return buf;
}

//================================================================================
// Main program loop. 
// Handles main measurement cycle: synchronization with the NTP, getting readings 
// from sensors and sending data to the service.
//================================================================================

void loop () {
  // Do fist time connection or renew connection config
  if(startupSynchDone && cycleIntervalMin > ipLeaseTime){
    connectToInternet();
  }
  if(getNTPTime()) startupSynchDone=true;
  // If startup synchronization was done successfully then station is ready to work
  if(startupSynchDone){
      // Dust readings
      float dustReading = getDustReadings();
      if(dustReading!=0){
        sendData(convertToChar(dustReading),dustSensorId);
      }
      // Temperature
      sendData(convertToChar(getTemperature()),tempSensorId);
      // Humidity
      sendData(convertToChar(getHumidity()),humiSensorId);
      // CO
      sendData(convertToChar(getMqSensorReading(MQ9)),coSensorId);
      // O3
      sendData(convertToChar(getMqSensorReading(MQ131)),o3SensorId);
  }

  if(startupSynchDone) delay(60000*cycleIntervalMin);
  else delay(5000);

}


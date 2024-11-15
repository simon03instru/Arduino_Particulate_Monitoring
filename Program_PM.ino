/*
PM2.5 & PM10 Monitoring
Created 10 Dec 2021
Modified January, February, March 2022
This Code is in private domain
*/


#include <sps30.h>
#include <RTClib.h>
#include <TimeLib.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>


IPAddress timeServer(); // NTP
EthernetUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets
time_t prevDisplay = 0;

// Deklarasi Waktu
RTC_DS3231 rtc;
char t[32];
char d[32];
char w[32];

//Deklarasi File SD Card
File myFile;

//Deklarasi Tegangan Baterai (Unavailable yet)
int value = 0;
float voltage;
float R1 = 47000.0;
float R2 = 33000.0;

const int chipSelect = 4;

//Deklarasi Stasiun
String id = "";

//Deklarasi Ethernet, Pengiriman & Interval Kirim HTTP
    byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
    EthernetClient client;
    int    HTTP_PORT   = 80;
    String HTTP_METHOD = "GET"; // or POST
    char   HOST_NAME[] = "";
    String PATH_NAME   = "";
    int http_int = 10; // interval kirim HTTP
    String dataString;
    
//Deklarasi Pengiriman & Interval MQTT
    int interval = 1; //interval kirim MQTT
    String MQTT_PATH = "";
    char MQTT_HOST[] = "";
    String MQTT_TOPIC = "device/";
    String MQTT_METHOD = "POST";
    String mqttString;


// Program untuk setting waktu (panjang) hehehe

void ClockSet(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year()); 
  Serial.println(); 
  rtc.adjust(DateTime(year(), month(), day(), hour(), minute(), second()));
}

void printDigits(int digits){
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

float ten_PM10, ten_PM2p5;

    
void setup() {
              
              //Sensor's return value
                  int16_t ret;
                  uint8_t auto_clean_days = 1;
                  uint32_t auto_clean;

              ten_PM10 = 0;
              ten_PM2p5 = 0;
              
              //Init Logger
                  Serial.begin(57600);
                  Wire.begin();
                  rtc.begin();
                  delay(2000);
              
              //Init SD Card
//                  if (!SD.begin(chipSelect)) {
//                    Serial.println("Card failed, or not present");
//                    // don't do anything more:
//                    while (1);
//                  }
//                  Serial.println("card initialized.");
                  
                                        /*Jika Menggunakan LCD
                                          lcd.init(); 
                                          lcd.init();
                                          lcd.backlight();
                                          lcd.setCursor(2,0);
                                          lcd.print("D");delay(1000); lcd.print("U");delay(500);
                                          lcd.print("S");delay(500); lcd.print("T");delay(500);
                                          lcd.print(" M");delay(500); lcd.print("O");delay(500);
                                          lcd.print("N");delay(500); lcd.print("I");delay(500);
                                          lcd.print("T");delay(500); lcd.print("O");delay(500);lcd.print("R");delay(500);
                                        
                                          lcd.setCursor(3,1);
                                          lcd.print("P");delay(200); lcd.print("M");delay(200);
                                          lcd.print("10");delay(500); lcd.print(",");delay(200);
                                          lcd.print("P");delay(200); lcd.print("M");delay(200);
                                          lcd.print("2.5");delay(1000);
                                          lcd.clear();
                                        */
             
              //Init Ethernet Shield
                if (Ethernet.begin(mac) == 0) {
                  Serial.println("Failed to obtaining an IP address using DHCP");
                  while(true);
                }
              
              //Init Senor SPS30 IIC
                sensirion_i2c_init();
              
                while (sps30_probe() != 0) {
                  Serial.print("SPS sensor probing failed\n");
                            /*
                            lcd.setCursor(0, 0);
                            lcd.print("Sensor NOT OK");
                            lcd.clear();
                            */
                  delay(500);
                }
                
              /* PLOTTER_FORMAT 
              #ifndef PLOTTER_FORMAT
                Serial.print("SPS sensor probing successful\n");
                lcd.setCursor(0, 0);
                lcd.print("Sensor OK");delay(2000);
                lcd.clear();
              #endif 
              */
//                ret = sps30_get_fan_auto_cleaning_interval_days(auto_clean);
//                 if (ret) {
//                  Serial.print("error getting the auto-clean interval: ");
//                  Serial.println(ret);
//                }
//                Serial.println(auto_clean);
                
                
                ret = sps30_set_fan_auto_cleaning_interval_days(auto_clean_days);
                if (ret) {
                  Serial.print("error setting the auto-clean interval: ");
                  Serial.println(ret);
                }
              
              /* PLOTTER_FORMAT 
              #ifndef PLOTTER_FORMAT
                Serial.print("measurements started\n");
              #endif 
              */
              
              delay(1000);
}





void loop() {

      //Deklarasi Variable Looping
      int u;
      int count;
      float PM2p5, PM10;
      float av_PM2p5; 
      float av_PM10;
      float av10_PM2p5; 
      float av10_PM10;
      struct sps30_measurement m;
      char serial[SPS30_MAX_SERIAL_LEN];
      uint16_t data_ready;
      int16_t ret;
      value = analogRead(A0);
      voltage = value * (5.0/1024)*((R1 + R2)/R2);
  
      //Create Time String
      DateTime now = rtc.now();
      sprintf(t, "%02d%02d%02d%02d%02d%02d",  now.day(), now.month(), now.year(), now.hour(), now.minute(), now.second());  
      sprintf(d, "%02d/%02d/%02d",  now.day(), now.month(), now.year());
      sprintf(w, "%02d:%02d:%02d",  now.hour(), now.minute(), now.second());
      //sprintf(t_lcd, "%02d:%02d UTC", now.hour(),now.minute());

      Serial.println(now.second());
      
      if (now.minute()%interval == 0 && now.second()==0){

                      //          //Sinkron waktu dengan NTP
                                if (now.minute() == 0){
                                      Udp.begin(localPort);
                                      Serial.println("Waiting for time sync");
                                      setSyncProvider(getNtpTime);
                                      time_t prevDisplay = 0;
                                      ClockSet();
                                }
    
                                     ret = sps30_wake_up();
                                      if (ret < 0) {
                                        Serial.print("error waking up the sensor\n");
                                      }

                                      ret = sps30_start_measurement();
                                      if (ret < 0) {
                                        Serial.print("error starting measurement\n");
                                      }
                                      
                                      delay(10000);

                                      do {
                                        ret = sps30_read_data_ready(&data_ready);
                                        if (ret < 0) {
                                          Serial.print("error reading data-ready flag: ");
                                          Serial.println(ret);
                                        } else if (!data_ready)
                                          Serial.print("");
                                        else
                                          break;
                                        delay(100); /* retry in 100ms */
                                      } while (1);


                                      ret = sps30_read_measurement(&m);
                                      if (ret < 0) {
                                        Serial.print("error reading measurement\n");
                                      } 
     
                                      PM2p5 = 0;
                                      PM10 = 0;
                                      for ( int count =0; count < 10; count++){
                                         PM10 = PM10 + m.mc_10p0;
                                         PM2p5 = PM2p5 + m.mc_2p5;
                                         delay(1000);
                                       }
                                      av_PM10 = PM10/10;
                                      av_PM2p5 = PM2p5/10;
       
                                      ret = sps30_stop_measurement();
                                      if (ret < 0) {
                                        Serial.print("error stopping measurement\n");
                                      } 

                                      ret = sps30_sleep();
                                      if (ret < 0) {
                                        Serial.print("error sleeping sensor\n");
                                      } 
                            
                                      Serial.println(av_PM10);
                                      Serial.println(av_PM2p5);
                                      
                                      delay(1000);
      
          
        
            //Pengiriman MQTT (HTTP Post)
            mqttString = "{\"topic\":\"" + MQTT_TOPIC + "\",\"data\":" + "{\"pm2\":" +String(av_PM2p5)+",\"pm10\":" + String(av_PM10)+",\"date\":" +"\""+ String(d)+"\""+",\"time\":" +"\""+ String(w)+"\""+",\"id\":" +"\""+ String(id)+"\""+",\"batt\":" +"\""+ String(voltage)+"\""+ "}}";

           ten_PM10 = ten_PM10 + av_PM10;
           ten_PM2p5 = ten_PM2p5 + av_PM2p5;

           Serial.println(ten_PM10);
           Serial.println(ten_PM2p5);
           
           //Pengiriman via http (GET Request)
           if(now.minute()%http_int == 0){
              delay(3000);
              
              av10_PM10 = ten_PM10/10;
              av10_PM2p5 = ten_PM2p5/10;
              Serial.println(av10_PM2p5);
              
              //Susun datastring untuk http 
              dataString = String(id_stasiun)+String(";")+String(t)+String(";")+String(av10_PM2p5)+String(";")+String(av10_PM10)+String(";")+String(voltage);

//              
              if(client.connect(HOST_NAME, HTTP_PORT)) {
              // if connected:
                  Serial.println("Connected to server");
                  // make a HTTP request:
                  // send HTTP header
                  client.println(HTTP_METHOD + " " + PATH_NAME + String(dataString)+ " HTTP/1.0");
                  client.println("Host: " + String(HOST_NAME));
                  //client.println("Connection: close");
                  client.println(); // end HTTP header 
                          /*
                          lcd.clear();
                          lcd.setCursor(0,0);
                          lcd.print("Data Send");delay(2000);
                          */
                  while(client.connected()) {
                    if(client.available()){
                        // read an incoming byte from the server and print it to serial monitor:
                        char c = client.read();
                        Serial.print(c);
                        }
                  }
                  // the server's disconnected, stop the client:
                  client.stop();
                  Serial.println();
                  Serial.println("disconnected");
                  } else {// if not connected:
                    Serial.println("connection failed");
                      }
                      
//                  //Penyimpanan di microSD
//                        myFile = SD.open("Monitor.txt", FILE_WRITE);
//                        if (myFile) 
//                           {
//                              myFile.println(dataString);
//                              myFile.close();
//                              // print to the serial port too:
//                              Serial.println("Done Saving");
//                            }
//              
//                        else {
//                              Serial.println("error opening PM_Monitor.txt"); }

              ten_PM10 = 0;
              ten_PM2p5 = 0;
                            }
              
            delay(10000);

            
           
              if(client.connect(MQTT_HOST, HTTP_PORT)) {
                  client.println(MQTT_METHOD + " " + MQTT_PATH + " HTTP/1.0");
                  client.println("Host: " + String(MQTT_HOST));
                  client.println("Content-Type: application/json");
                  client.println("Connection: close");
                  client.print("Content-Length:");
                  client.println(mqttString.length());
                  client.println(); // end HTTP header
                  Serial.println(mqttString);
                  // send HTTP body
                  client.println(mqttString);
                  client.println();
                  while(client.connected()) {
                    if(client.available()){
                        // read an incoming byte from the server and print it to serial monitor:
                        char c = client.read();
                        Serial.print(c);
                    }
                  }

              // the server's disconnected, stop the client:
                  client.stop();
                  Serial.println();
                  Serial.println("disconnected");
            } else {// if not connected:
                Serial.println("connection failed");
            }

            
                }
                delay(1000);
                }
                

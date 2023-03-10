# E-bike
An e-bike made completely from scratch using off the shelf parts


## Tools, Parts, Design, and Skills

The core pieces of this project are the 750 Watt off the shelf 3 phase tesla motor generally used to power electric scooters and go-karts and the hall sensor that takes data from a magnet mounted to the rim of the wheel, feeding to a homemade speedometer that cuts power to the motor after 20 MPH, which is required to keep the bike street legal.

In completing this project, I utilized my existing soldering as well as analog circuit design and prototyping skills. Throughout the project, I learned to use Arduino's version of C++ and gained a thorough understanding of every piece of my bike and how to repair them. I sketched out a rough plan for the bike (below) but serious modifications needed to be made to that plan as clearance constraints were more of a factor than I had anticipated. I wanted every piece of the bike to be operating below spec, leading to the aggressive modification of the original mountain bike to a cruiser style riding posture while retaining all the strength and superior front suspension of the original mountain bike.



<picture>
 <source media="(prefers-color-scheme: dark)" srcset="https://github.com/Kove230/E-bike/blob/main/unnamed.png">
 <source media="(prefers-color-scheme: light)" srcset="https://github.com/Kove230/E-bike/blob/main/unnamed.png">
 <img alt="YOUR-ALT-TEXT" src="https://github.com/Kove230/E-bike/blob/main/unnamed.png">
</picture>





## Hardware

The main hardware challenges inherent in a completely DIY e-bike are building sufficiently sturdy mounts for the motor and electrical system, along with the execution of the electrical system. 

One of the primary motivations for pursuing this project was a lack of sufficient repairability and features in similar retail bikes available, which necessitated 12V power for automotive accessories such as turn signals and a headlight. This posed some challenges since it required splicing into the 48V battery and converting power down to 12V. In order to achieve this in a "weather-resistant" way, I housed the whole electrical system in a waterproof project box that was then bolted to the bike's frame with two U bolts. During testing, voltage ripple across the 12V line due to the 3 phase power draw for the motor was an issue for the esp32 based speedometer, but a 1MF capacitor smoothed the ripple enough that it became indiscernible. 

There is a system of 4 relays with their outputs shorting the off switch on the motor controller which provide motor control, 2 relays are dedicated to redundant speed controls limiting the bike to 20 MPH, a third sets the bike to "demo mode" further limiting the top speed to 15 MPH. This was included in anticipation of people wanting to ride the bike, but also functions as a low power mode. The last relay is currently detached, reserved for a possible lock upgrade in the future.



<picture>
 <source media="(prefers-color-scheme: dark)" srcset="https://github.com/Kove230/E-bike/blob/main/20230302_164749.jpg">
 <source media="(prefers-color-scheme: light)" srcset="https://github.com/Kove230/E-bike/blob/main/20230302_164749.jpg">
 <img alt="YOUR-ALT-TEXT" src="https://github.com/Kove230/E-bike/blob/main/20230302_164749.jpg">
</picture>




The motor was mounted similarly to the electrical box, it is slung underneath the bike's frame just behind the front wheel. I was very concerned about the motor wandering down the frame, resulting in slack in the chain, which I learned during testing resulted in the chain being thrown off of the motor's sprocket. To preemptively solve this wandering motor problem, I drilled two holes in the bottom of the bike frame and installed M4 threaded inserts, which align with holes drilled in the plates securing the U bolts. Threading an M4 bolt through the hole in the plate keeps the mount from wandering, and the motor covers the bolts, eliminating any possibility of them coming loose before the motor does.




## Software

Below is the code I wrote to run the speedometer that keeps the e-bike street legal. In addition to a speedometer, the display and code 
output an odometer, trip timer, and real time clock which automatically updates when powered on. It runs on an esp32 platform to allow for NTP updates of the real time clock as well as OTA updates.

```
#include <ArduinoOTA.h>
#include <TimeLib.h>
#include <LiquidCrystal_I2C.h>  //this library always triggers a warning for esp32 during compile, but works without any discernible flaws
#include <NTPClient.h>
#include <Ds1302.h>
#include <time.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

long unsigned int sec =(millis()/1000)%60;  //used for clock
long unsigned int mn=(millis()/60000)%60;   //used for clock
long unsigned int hr=(millis()/3600000);    //used for clock
const char* ssid       = "**************";   //Replace with your own SSID
const char* password   = "**************";  //Replace with your own password
unsigned long currenttime=millis();         //used for clock
unsigned long startrev=1;                   //used to calculate speed
unsigned long endrev=0;                     //used to calculate speed
unsigned long revtime=0;                    //used to calculate speed
int circMetric=4641; // wheel circumference, in ridiculous units to output miles/hour but intake milliseconds to cut down on calculations
float speedm=0;    // holds current speed in M/S (converted to MPH at the end for readability on the road)
int odometer=0;    //holds current odometer reading in miles
int rev = 0;
int ot = 0;
int nt = 0;
float thour=0;
int pst = 16;       //timezone offset for RTC, daylight savings makes this a mess

WiFiUDP ntpUDP; 
NTPClient timeClient(ntpUDP); //include NTP for real time clock
LiquidCrystal_I2C lcd(0x27,16,2);  // sets the LCD address to 0x27 for a 16 chars and 2 line display

Ds1302 rtc(14,27,16); //(PIN_ENA, PIN_CLK, PIN_DAT)     //set up RTC

void IRAM_ATTR isr() { //interrupt each time the magnet on the bike's wheel passes the hall sensor on the fork, 
                       //setting rev=1 functions as a debouncer, ensuring that multiple short contacts within a short time only count as one wheel revolution, 
                       //rev is set equal to 0 in the main loop
  if (rev==0){
  rev=1;
 endrev=startrev;
 startrev=millis(); //the interrupt debounces the input from the hall sensor, then stores the current and most recent wheel revolution times in system time, 
                    //discarding what is now the third most recent time as it is no longer relevant. 
                    //This is important for speed calculations which are executed in the main loop to save computational resources
  }
}

void setup()
{
lcd.init();            //initiate the LCD display
lcd.backlight();       //turn on the backlight
rtc.init();            //initiate real time clock
WiFi.mode(WIFI_STA);
WiFi.begin(ssid, password);
timeClient.begin();
timeClient.update();
pinMode(26,INPUT_PULLUP); //set pin 26 as an input, pin 26 connects to the hall sensor
pinMode(25,OUTPUT);       //sets pin 25 as an output, used for speed limiting
digitalWrite(25,LOW);     //set the speed control to off
pinMode(17,OUTPUT);       //second speed control with alternate limit for "demo mode"
digitalWrite(17,LOW);     //set speed control to off
attachInterrupt(digitalPinToInterrupt(26),isr,RISING);    //sets pin 26 as an interrupt

ArduinoOTA    //sets up OTA updating as long as the code takes up less than 50% of system memory, huge quality of life upgrade to be able to update an embedded device over WiFi
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";});

  ot=millis();
  
        while (WiFi.status() != WL_CONNECTED && millis()<7000) {    //crude loading screen while the system waits for WiFi connection. 
                                                                    //The delay is unnoticeable in use due to the delay from connecting the battery to starting the bike

nt=millis();

if (nt - ot >=0 && nt - ot<=400);
{ 
            lcd.setCursor(0,0);
  lcd.print("Connecting.  ");
}
   
if (nt - ot >400 && nt - ot<=800);
{
lcd.setCursor(0,0);
  lcd.print("Connecting.. ");
}
  
if (nt - ot >800);
{
lcd.setCursor(0,0);
  lcd.print("Connecting...");
  }
  if (nt-ot>1200){
ot=millis();
  }
  }
  
  ArduinoOTA.begin();

    if (WiFi.status()==WL_CONNECTED)
    {
     
      timeClient.update();

        Ds1302::DateTime dt = {
            .year = 17,
            .month = Ds1302::MONTH_OCT,
            .day = timeClient.getDay(),
            .hour = timeClient.getHours()+pst,
            .minute = timeClient.getMinutes(),
            .second = timeClient.getSeconds(),
            .dow = Ds1302::DOW_TUE    
                                      //Setting the clock readout
                                      //the code refuses to work without a month and year specified, even though they are unnecesarry,
                                      //the month and year are never called, and so are set aritrarily.
        };
        rtc.setDateTime(&dt);  
    }

      if(WiFi.status()==WL_CONNECTED) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Connected!");
  delay(1500);
  lcd.clear();
  }
        }

void loop()
{
   ArduinoOTA.handle();                  //needed for OTA updates

if (rev==1 && ((millis())-revtime)>75){ //reset counter for next input and increment odometer value, then calculate speed
  revtime = millis();
 rev=0;
 odometer++;
  t=(startrev-endrev); 
 int speedm=(circMetric/t);   //calculate speed, updates every input but is currently displayed at 1 HZ
}
   
if ((millis()-startrev)>500){ //if 500 ms passes since last input, set speed to 0
  speedm=0;
}
if (speedm>=20){ //setting safties
  digitalWrite(25,HIGH);
}
if (speedm<20){
  digitalWrite(25,LOW);
}
if (speedm>=15){ //setting second safety
  digitalWrite(17,HIGH);
}
if (speedm<15){
  digitalWrite(17,LOW);
}

   Ds1302::DateTime now; //retrieving RTC data
    rtc.getDateTime(&now);  

    static uint8_t last_second = 0;
    if (last_second != now.second)//limiting display refreshes to 1 HZ to conserve computational resources, refresh takes 250 milliseconds
    {
        last_second = now.second;

      lcd.clear(); //updating the speedometer and odometer display
        lcd.setCursor(0,0);
  lcd.print(int(speedm));
  lcd.print(" MPH");
  lcd.setCursor(10,1);
  lcd.print((odometer*0.00128916365));

  sec =(millis()/1000)%60; //timer code
  mn=(millis()/60000)%60;
  hr=(millis()/3600000);

  lcd.setCursor(0,1);
  lcd.print(hr);
  lcd.print(":");
  if (mn<10){
    lcd.print("0");
  }
  lcd.print(mn);
  lcd.print(":");
  if (sec<10){
    lcd.print("0");
  }
  lcd.print(sec);
  lcd.print(" "); 
  
  lcd.setCursor(7,0); //RTC display code
  lcd.print(" ");
        if (now.hour < 10) lcd.print('0');
        lcd.print(now.hour);    // 00-23
        lcd.print(':');
        if (now.minute < 10) lcd.print('0');
        lcd.print(now.minute);  // 00-59
        lcd.print(':');
        if (now.second < 10) lcd.print('0');
        lcd.print(now.second);  // 00-59 //end RTC display code
    }
  
}
```

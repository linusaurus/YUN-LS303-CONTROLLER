#include <Automaton.h>

/*
MQTT client LIFT_SLIDE Motor Controller
Master Controller
r.young 
8.29.2017 vB, v3[9.5.2017] v4[9.7.2017] v5[3.1.2018]
v6[6-11-2018]
LS301-DLC-ESP-A
*/

#include <Bridge.h>
#include <BridgeClient.h>
#include <Process.h>
#include <YunClient.h>
#include <BridgeServer.h>


#define DEBUG 1
//#define WRITE_MEM

#include <PubSubClient.h>
#include <EEPROM.h>

YunClient yunClient;
PubSubClient client(yunClient);

long lastMsg = 0;
char msg[50];
int value = 0;

byte DOOR_ONE_STATUS = 0;
byte DOOR_TWO_STATUS = 0;
byte DOOR_POSITION = 0;
byte OLD_DOOR_POSITION;
byte CLUTCH_LOCK = 0;
byte OLD_CLUTCH_LOCK;
bool DOOR_IN_MOTION = false;

//-------------Pins--------------
int OpenRelayPin =   2;
int CloseRelayPin =  3;
int StopRelayPin =   4;
int UnlockRelayPin = 5;
//int PowerPin =     6; // Not used
int ClutchPin      = 6; // changed to match new harness ry
int StatusPIN = 8;
//--------------------------------
byte STATE = 0;

// Relay Semaphores!
bool IN_OPEN_MODE = false;
bool IN_CLOSE_MODE = false;
bool IN_STOP_MODE = false;
bool IN_UNLOCKED_MODE = false;
bool DOORS_RISING = false;

Atm_led statusLED,updoorLED,downdoorLED;

unsigned long     TIME_START_VAL; 
volatile boolean  TIMED_OUT_FLAG = true; 
unsigned long     RELAY_DURATION = 850; 

const char* MQTT_CLIENT_ID = "LS301CTL";
const char* outTopic = "DLIN";
const char* inTopic = "LSCMD";
const char* statusTopic = "STATUS";
const char* mqtt_server = "192.168.10.62";



void callback(char* topic, byte* payload, unsigned int length) {
  #ifdef DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
    #endif
  // Filter for topics
    if(strcmp(topic,"LSCMD")==0){

      if ((char)payload[0] == '0') {
       STATE=0;
        statusLED.begin(StatusPIN).blink(20,2000).trigger(statusLED.EVT_START);
       ClearAllRelays();
      }
      
       // Raise Doors, confirm status, open doors 
        if ((char)payload[0] == '1') {  
          STATE=1;
          #ifdef DEBUG
          Serial.println("Open-> 1");
          Serial.print("State = ");
          Serial.println(STATE);
          statusLED.begin(StatusPIN).blink(200,200).trigger(statusLED.EVT_START);
          #endif
          if(DOOR_POSITION==2){
          client.publish(outTopic, "1" ); //Raise All Subscriberd Doors 
          }
          
        }
        // 
        else if ((char)payload[0] == '2') { 
          STATE=2;
          statusLED.begin(StatusPIN).blink(100,500).trigger(statusLED.EVT_START);
        #ifdef DEBUG  
          Serial.println("Close-> 2");
        #endif
        //Check that door are up /DLOUT=1
               
        } 
        else if ((char)payload[0] == '3') { 
          STATE=3;
          statusLED.begin(StatusPIN).blink(1000,1000).trigger(statusLED.EVT_START);
        #ifdef DEBUG
          Serial.print("Stop-> 3");
        #endif   
        }    
         else if ((char)payload[0] == '4') {
          STATE=4;
          statusLED.begin(StatusPIN).blink(100,1000).trigger(statusLED.EVT_START);
         #ifdef DEBUG  
          Serial.print("Unlock-> 4"); 
         #endif          
        }
    }//End Topic DMSTR--
    if(strcmp(topic,"STATUS")==0){

         if ((char)payload[0] == '1') {  
         #ifdef DEBUG
         Serial.println("Status = 1; DR1 UP POSITION");
         #endif
         DOOR_ONE_STATUS = 1;
       // EEPROM.write(1,DOOR_ONE_STATUS);
        // EEPROM.commit();
        }

        if ((char)payload[0] == '2') {
         #ifdef DEBUG  
         Serial.println("Status = 2; DR1 DN POSITION");
         #endif
         DOOR_ONE_STATUS = 2;
        // EEPROM.write(1,DOOR_ONE_STATUS);
        // EEPROM.commit();
        }     
         if ((char)payload[0] == '3') {  
         #ifdef DEBUG
         Serial.println("Status = 3; DR2 UP POSITION");
         #endif
         DOOR_TWO_STATUS = 3;
        // EEPROM.write(2,DOOR_TWO_STATUS);
        // EEPROM.commit();
        }

        if ((char)payload[0] == '4') { 
         #ifdef DEBUG 
         Serial.println("Status = 4; DR2 DN POSITION");
         #endif
         DOOR_TWO_STATUS = 4;
         //EEPROM.write(2,DOOR_TWO_STATUS);
         //EEPROM.commit();
        }
    }
}

void reconnect() {

  
  // Loop until we're reconnected
  while (!client.connected()) {
    #ifdef DEBUG
    Serial.print("Attempting MQTT connection...");
    #endif
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID)) {
      #ifdef DEBUG
      Serial.println("connected");
      ClearAllRelays();
      
      #endif
      // Once connected, publish an announcement...
      client.publish(outTopic, MQTT_CLIENT_ID );
      // ... and resubscribe
      client.subscribe(inTopic);
      client.subscribe(statusTopic);
      
    } else {
      #ifdef DEBUG
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      #endif
      // Wait 5 seconds before retrying
      for(int i = 0; i<5000; i++){ 
        delay(1);
      }
    }
  }
}

void setup() {
  Bridge.begin();
  //EEPROM.begin(512);              // Begin eeprom to store on/off state
  pinMode(OpenRelayPin,OUTPUT);
  pinMode(CloseRelayPin,OUTPUT);
  pinMode(StopRelayPin,OUTPUT);
  pinMode(UnlockRelayPin,OUTPUT);
  
  pinMode(ClutchPin,INPUT);
  statusLED.begin(StatusPIN).blink(20,2000);
  ClearAllRelays();
  
  #ifdef WRITE_MEM
  EEPROM.read(0);//Last Operation or State
  EEPROM.read(1);//D1-status
  EEPROM.read(2);//D2-status
  #endif
  delay(500);
  #ifdef DEBUG
  Serial.begin(115200);
  while (!Serial){
    ;
  }
  #endif
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  #ifdef DEBUG
 
  #endif
  statusLED.trigger(statusLED.EVT_BLINK);
 
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  automaton.run();  

   if ((!TIMED_OUT_FLAG) && ((millis() - TIME_START_VAL) >= RELAY_DURATION)) {
        // timed out
        TIMED_OUT_FLAG = true; // don't do this again
        if(IN_OPEN_MODE ==true){
          
          digitalWrite(OpenRelayPin, HIGH);
          #ifdef DEBUG
          Serial.println("Open Relay Released!");
          #endif
        }
        if(IN_CLOSE_MODE ==true){
          
          digitalWrite(CloseRelayPin, HIGH);
          #ifdef DEBUG
          Serial.println("Close Relay Released!");
          #endif
        }
        if(IN_STOP_MODE ==true){
          
          digitalWrite(StopRelayPin, HIGH);
         #ifdef DEBUG
          Serial.println("Stop Relay Released!");
          #endif
        }
        if(IN_UNLOCKED_MODE ==true){
          
          digitalWrite(UnlockRelayPin, HIGH);
          #ifdef DEBUG
          Serial.println("Unlock Relay Released!");
          #endif
        }    
          
   }
    
//-------------------------------STATE MACHINE----------------------------------
if(STATE==0){  
  
    ClearAllRelays();
    IN_OPEN_MODE=false;
    IN_CLOSE_MODE=false;
    IN_STOP_MODE = false;
    IN_UNLOCKED_MODE=false;
    
  }
if(STATE==1 && !IN_OPEN_MODE && !DOORS_RISING){ 
  Serial.println("Send DLIN=1");  
  client.publish(outTopic,"1" );
  DOORS_RISING=true;
  }
if(STATE==1 && DOOR_POSITION==1)
{
  OpenRelay();
}
if(STATE==2 && !IN_CLOSE_MODE){
  #ifdef DEBUG
  Serial.println("Close Relay Fired");
  #endif
  CloseRelay();
  }
if(STATE==3 && !IN_STOP_MODE){
  #ifdef DEBUG
  Serial.println("Stop Relay Fired");
  #endif
  StopRelay();
  }
if(STATE==4 && !IN_UNLOCKED_MODE){
  #ifdef DEBUG
  Serial.println("Unlock Relay Fired");
  #endif
  UnlockRelay();
  }

//------------------------------------------------------------------------------ 
//-------------------------------  POLLING EVENTS -----------------------------
//------------------------------------------------------------------------------
  CheckDoorPositions();
  OnDoorPositionChanged();
  ClutchStatus();
  OnClutchChanged();
//------------------------------------------------------------------------------
}

void ClutchStatus(){
  
 if( digitalRead(ClutchPin)==HIGH){
  CLUTCH_LOCK = 1;
  //Serial.println("HIGH");
 }
 else if(digitalRead(ClutchPin)==LOW){
  CLUTCH_LOCK = 0;
  //Serial.println("LOW");
 }
}

void OnClutchChanged(){
  if(OLD_CLUTCH_LOCK != CLUTCH_LOCK){
    #ifdef DEBUG
    Serial.print("Clutch LockedChanged = ");
    Serial.println(CLUTCH_LOCK);
    #endif
    OLD_CLUTCH_LOCK = CLUTCH_LOCK;
    if(STATE==2 && CLUTCH_LOCK==0){
      client.publish(outTopic,"2" );  // Lower the Doors after clutch releases
     }
  }
}
void OnDoorPositionChanged(){
  
  if(OLD_DOOR_POSITION != DOOR_POSITION){
    #ifdef DEBUG
    Serial.print("Door Position Changed; Position = ");
    Serial.println(DOOR_POSITION);
    #endif
    if(STATE==1){
      delay(200);
      OpenRelay();
      Serial.println("Open Relay");
      DOORS_RISING=false;
    }
   
    OLD_DOOR_POSITION = DOOR_POSITION;
  }
}

void CheckDoorPositions(){

//byte DOOR_ONE_STATUS = 0;  1 = UP-2=DN
//byte DOOR_TWO_STATUS = 0;  3 = UP-4=DN

  if(DOOR_ONE_STATUS==1 && DOOR_TWO_STATUS==3){
    DOOR_POSITION=1;   
    }
  if(DOOR_ONE_STATUS==2 && DOOR_TWO_STATUS==4){
    DOOR_POSITION=2;
    }     
   
}

void OpenDoors(){
  
  
}

void ClearAllRelays(){
  digitalWrite(OpenRelayPin,HIGH);
  digitalWrite(CloseRelayPin,HIGH);
  digitalWrite(StopRelayPin,HIGH);
  digitalWrite(UnlockRelayPin,HIGH);
}

void OpenRelay(){ 
    if(IN_OPEN_MODE==false){  
      TIMED_OUT_FLAG= false;
      digitalWrite(OpenRelayPin, LOW);
      TIME_START_VAL = millis();
      IN_OPEN_MODE=true;
      IN_CLOSE_MODE=false;
      IN_STOP_MODE=false;
      IN_UNLOCKED_MODE=false;
      
    }     
  }
  
void CloseRelay(){
      if(IN_CLOSE_MODE==false){
      TIMED_OUT_FLAG= false;
      digitalWrite(CloseRelayPin, LOW);     
      TIME_START_VAL = millis(); 
      IN_OPEN_MODE=false;
      IN_CLOSE_MODE=true;
      IN_STOP_MODE=false;
      IN_UNLOCKED_MODE=false;;
      }
}

void StopRelay(){
      if(IN_STOP_MODE==false){
      TIMED_OUT_FLAG= false;
      digitalWrite(StopRelayPin, LOW);     
      TIME_START_VAL = millis(); 
      IN_OPEN_MODE=false;
      IN_CLOSE_MODE=false;
      IN_STOP_MODE=true;
      IN_UNLOCKED_MODE=false;
      }
 }
 
 void UnlockRelay(){
      if(IN_UNLOCKED_MODE==false){
      TIMED_OUT_FLAG= false;
      digitalWrite(UnlockRelayPin, LOW);     
      TIME_START_VAL = millis(); 
      IN_OPEN_MODE=false;
      IN_CLOSE_MODE=false;
      IN_STOP_MODE=false;
      IN_UNLOCKED_MODE=true;
      }

   
}

  




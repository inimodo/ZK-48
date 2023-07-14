// Copyright: Markus K. 2023
// Github:    https://github.com/inimodo

#include <EEPROM.h>

#define MODUL_DATA 3
#define MODUL_CLK 4
#define TRIGGER_FIRE 2
#define TRIGGER_ARMED 0
#define TRIGGER_CONNECTED 1
#define TRIGADD_A0 8
#define TRIGADD_A1 9
#define TRIGADD_1E 10
#define TRIGADD_2E 11
#define STATUS_RED 12
#define STATUS_PIEP 5
#define STATUS_GREEN 13
#define THIS_ARMED 14
#define RX 7 
#define TX 6 

// pin mapping for selection of the module
//D11/A0 -> nA1
//D10/A1 -> nA0
//D9 /A2 -> 1E
//D8 /A3 -> 2E

// !! do not change !!
#define nA1 0
#define nA0 1
#define E1 2
#define E2 3

// !! do not change !!
#define ALL_OFF 0
#define CLEAR_REG 10
//nA1 nA0,1E 2E
const byte module_resolve[7][4]=
{
  {0,0,1,1},//ALL_OFF
  {0,0,0,1},
  {1,0,0,1},
  {0,1,0,1},
  {1,1,0,1},
  {1,1,1,0},
  {0,1,1,0}
};

typedef struct _section {
  byte module;
  byte port;
  unsigned short msdelay; 
} section;

// the program length (number of ports) higher values do not make sense
#define PG_LENGTH 48
byte pg_stop = 0;
byte pg_index = 0;

// chars for program mode operation
#define PGM_ARG_CHAR ':'
#define PGM_START_CHAR '#'
#define PGM_ANS_CHAR '!'
#define PGM_MAXARGS 6

const char* cmd_set = "set";
const char* cmd_list = "list";
const char* cmd_stop = "stop";
const char* cmd_pgm = "pgm";

// threshold for the number of impules before starting the wait sequence
#define FIRE_PULSE_SETOFF 1000
// delay before ignition of the program
#define FIRE_DELAY 100 // 1 = 100ms 100 = 10s
short fire_pulse_counter = 0;

// time each ignition pulse takes - depends on e-igniter
#define IGNITION_TIME 10 // ms
// clock speed of transmitData() !! do not change !!
#define TRANSMISSION_SPEED 5 //us

#define BAUDRATE 57600

#define STATE_END 8
#define STATE_WAIT 7
#define STATE_FIRE 6
#define STATE_ARMED 5
#define STATE_IDLE 4
#define STATE_NO_TRIGGER 3
#define STATE_INIT 2
#define STATE_PG_MODE 1

byte state = STATE_INIT;
byte wait_counter;

// ################# // Setup // ################# //

void setup() { //

  // this is not my code!
  // source: https://community.platformio.org/t/
  //         attiny-8mhz-wrong-timing-in-delay-fastled-and-neopixel/24992/3
  // start
  cli(); // Disable interrupts
  CLKPR = (1<<CLKPCE); // Prescaler enable
  CLKPR = 0x00; // Clock division factor
  sei(); // Enable interrupts
  // end
  
  Serial.begin(BAUDRATE);
  
  attachInterrupt(digitalPinToInterrupt(TRIGGER_FIRE),fire,RISING);
  
  pinMode(MODUL_DATA,OUTPUT);
  pinMode(MODUL_CLK,OUTPUT);
  pinMode(TRIGADD_A0,OUTPUT);
  pinMode(TRIGADD_A1,OUTPUT);
  pinMode(TRIGADD_1E,OUTPUT);
  pinMode(TRIGADD_2E,OUTPUT);
  pinMode(STATUS_PIEP,OUTPUT);
  pinMode(STATUS_RED,OUTPUT);
  pinMode(STATUS_GREEN,OUTPUT); 
  pinMode(TRIGGER_FIRE,INPUT);
  pinMode(TRIGGER_ARMED,INPUT);
  pinMode(TRIGGER_CONNECTED,INPUT);
  pinMode(THIS_ARMED,INPUT);

  transmitData(CLEAR_REG);
  setModule(ALL_OFF);
  pg_stop = readProgramEndpoint();
    
}

// ################# // EEPROM Functions // ################# //

void writeProgramEndpoint(byte endpoint)
{
  EEPROM.write(0,endpoint);
}

byte readProgramEndpoint()
{
  return EEPROM.read(0);
}

void writeProgramPoint(byte index,section pg_point)
{
  short real_address = index*sizeof(section) + sizeof(byte);
  EEPROM.put(real_address,pg_point);
  
}

section readProgramPoint(byte index)
{
  section pg_point;
  short real_address = index*sizeof(section) + sizeof(byte);
  EEPROM.get(real_address,pg_point);
  return pg_point;
}

// ################# // Program Mode // ################# //

// checks if program mode must be entered
char checkPGM()
{
  if (Serial.available() > 0) 
  {
    String str_buffer = Serial.readString();    
    char split_index[PGM_MAXARGS];
    split_index[0] = str_buffer.indexOf(PGM_START_CHAR);
    for(int index=1; index < PGM_MAXARGS; index++)
    {
      split_index[index] = str_buffer.indexOf(
        PGM_ARG_CHAR,
        split_index[index-1]+1);
    }
    
    str_buffer = str_buffer.substring(
      split_index[0]+1,
      split_index[1]);
      
    if(str_buffer.compareTo("pgm")==0)
    {
      return true;
    }
  }
  return false;
}

// programs the internal EEPROM via serial commands
byte programmMode()
{
  if (Serial.available() > 0) 
  {
    String str_buffer = Serial.readString();
    delay(1);

    char split_index[PGM_MAXARGS];
    split_index[0] = str_buffer.indexOf(PGM_START_CHAR);
    for(int index=1; index<PGM_MAXARGS; index++)
    {
      split_index[index] = str_buffer.indexOf(
        PGM_ARG_CHAR,
        split_index[index-1]+1);
    }
    
    String cmd = str_buffer.substring(split_index[0]+1,split_index[1]);

    if(cmd == cmd_list)
    {
      Serial.println("!list:");
      Serial.print(PGM_ANS_CHAR);
      Serial.print("stop:");
      Serial.print(pg_stop);
      Serial.println(PGM_ARG_CHAR);

      for(int index=0;index<PG_LENGTH;index++)
      {
        section pgpoint = readProgramPoint(index);
        Serial.print(PGM_ANS_CHAR);
        Serial.print("set:");
        Serial.print(index);
        Serial.print(PGM_ARG_CHAR);
        Serial.print(pgpoint.module);
        Serial.print(PGM_ARG_CHAR);
        Serial.print(pgpoint.port);
        Serial.print(PGM_ARG_CHAR);
        Serial.print(pgpoint.msdelay);
        Serial.println(PGM_ARG_CHAR);
      }
      return;
    }else if(cmd== cmd_stop)
    {
      pg_stop = str_buffer.substring(split_index[1]+1,split_index[2]).toInt();
        
      writeProgramEndpoint(pg_stop);
      Serial.println("!stop:");
      return;
    }else if(cmd== cmd_set)
    {
      byte index = str_buffer.substring(
        split_index[1]+1,
        split_index[2]).toInt();   
          
      section new_pgpoint;
      
      new_pgpoint.msdelay = (unsigned short)str_buffer.substring(
        split_index[4]+1,
        split_index[5]).toInt();
        
      new_pgpoint.port = (byte)str_buffer.substring(
        split_index[3]+1,
        split_index[4]).toInt();
        
      new_pgpoint.module = (byte)str_buffer.substring(
        split_index[2]+1,
        split_index[3]).toInt();

      writeProgramPoint(index,new_pgpoint);
      
      Serial.println("!set:");
      return;
    }
    Serial.println("!error:");
  }
}

// ################# // IO Functions // ################# //

// sets the port on ALL modules
void transmitData(byte port) //
{
  
  for(byte port_index = 0; port_index < 9; port_index ++ )
  {
    if(port_index == port)
    {
      digitalWrite(MODUL_DATA,HIGH);
    }else{
      digitalWrite(MODUL_DATA,LOW);  
    }
    delayMicroseconds(TRANSMISSION_SPEED);
    digitalWrite(MODUL_CLK,HIGH);
    delayMicroseconds(TRANSMISSION_SPEED);
    digitalWrite(MODUL_CLK,LOW);
    digitalWrite(MODUL_DATA,LOW);
  }
}

// selects what module is meant to ignite the set Port
void setModule(byte module) //
{
  digitalWrite(TRIGADD_A0,module_resolve[module][nA0]);
  digitalWrite(TRIGADD_A1,module_resolve[module][nA1]);
  digitalWrite(TRIGADD_1E,module_resolve[module][E1]);
  digitalWrite(TRIGADD_2E,module_resolve[module][E2]);
}

// ignites a port on a given module
void ignite(byte module, byte port) //
{
  transmitData(port);
  setModule(module+1); // +1 because 0 (ALL_OFF) is turning off all modules
  
  delay(IGNITION_TIME);
  
  setModule(ALL_OFF);
  transmitData(CLEAR_REG); // clear registers for safety reasons
}

// sets status led
void setLEDState(byte G,byte R,byte P)
{
  digitalWrite(STATUS_RED,R);
  digitalWrite(STATUS_GREEN,G);
  digitalWrite(STATUS_PIEP,P);
}

// ################# // State Machine // ################# //

// does what the name implies 
byte FSM(byte state)
{
  byte PG = checkPGM();
  byte TF = fire_pulse_counter >= FIRE_PULSE_SETOFF;
  byte DN = pg_index == pg_stop;
  byte WT = wait_counter < FIRE_DELAY;
  byte TA = digitalRead(TRIGGER_ARMED);
  byte TC = digitalRead(TRIGGER_CONNECTED);
  byte CA = digitalRead(THIS_ARMED);

  byte new_state = state;

  switch(state)
  {
    case STATE_PG_MODE:
      Serial.println("!pgm:");
      setLEDState(HIGH,HIGH,LOW);
      while(1)
      {
        programmMode();
      }
      break;
      
    case STATE_INIT:
      if(PG == 0)new_state = STATE_NO_TRIGGER;
      else if(CA == 0 && TC == 1 && TA == 0)new_state = STATE_IDLE;
            
      for(int i = 0;i<3;i++){
        setLEDState(HIGH,HIGH,HIGH);
        delay(100);
        setLEDState(LOW,LOW,LOW);
        delay(100);
      }
      
      break;
      
    case STATE_NO_TRIGGER:
      if(PG == 1)new_state = STATE_PG_MODE;
      else if(CA == 0 && TC == 1 && TA == 0)new_state = STATE_IDLE;
      
      setLEDState(HIGH,LOW,LOW);
      delay(250);
      setLEDState(LOW,HIGH,LOW); 
      delay(250);      
      break;
      
    case STATE_IDLE:
      if(PG == 1)new_state = STATE_PG_MODE;
      else if(TC == 0) new_state = STATE_NO_TRIGGER;
      else if(CA == 1 && TA == 1) new_state = STATE_ARMED;

      wait_counter = 0;
      
      setLEDState(HIGH,LOW,LOW);
      delay(100);
      setLEDState(LOW,LOW,LOW);
      delay(500);
      break;
    case STATE_ARMED: 
           
      if(TA == 0)new_state = STATE_IDLE;
      else if(CA == 0)new_state = STATE_IDLE;
      if(TF == 1)new_state = STATE_WAIT;
      fire_pulse_counter = 0;
      
      setLEDState(LOW,HIGH,HIGH);
      delay(100);
      setLEDState(LOW,LOW,LOW);
      delay(500); 

      break;
      
    case STATE_WAIT:
      if(TA == 0)new_state = STATE_IDLE;
      else if(TA == 0)new_state = STATE_IDLE;
      else if (WT == 0) new_state = STATE_FIRE;

      wait_counter++;
      
      setLEDState(LOW,HIGH,HIGH);
      delay(50);
      setLEDState(LOW,LOW,LOW);
      delay(50);
      break;
      
    case STATE_FIRE:
      if(TA == 0)new_state = STATE_IDLE;
      else if(TA == 0)new_state = STATE_IDLE;
      else if(DN == 1) new_state = STATE_END;
      
      section pgpoint = readProgramPoint(pg_index);
      
      setLEDState(HIGH,LOW,LOW);
      delay(pgpoint.msdelay);
      setLEDState(LOW,HIGH,HIGH);
      ignite(pgpoint.module,pgpoint.port); 
      
      pg_index++;  
      break;
  
    case STATE_END:
      setLEDState(LOW,HIGH,LOW);
      break;
      
  }
  return new_state;
}

// ################# // Main Loop // ################# //

void loop() {
  state = FSM(state);
}

void fire()
{
  fire_pulse_counter++;
}

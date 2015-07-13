/*
NHC Carriage Controller
by M. Emre Caliskan
2014

Controls and operates the instrumentation carriage.
Actuates carriage to move to the list of input coordinates (X, Y, Z).
*/

#include <AccelStepper.h>
#include <ByteBuffer.h>

#include "Arduino.h"
#include <WProgram.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#define USART_BAUDRATE 9600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

//Constants;
int dataPacketSize = 15;
int bottomLimitPin=2;
int topLimitPin=3;

long lowerLimitInSteps[3] = {0, 0, 0};
long upperLimitInSteps[3];    //Upper Limit in Inches * Pulses Per Inch

//Variables;
float userCommands[3];
float singleCommands[3];
boolean commandsReceived = false;
boolean motorsReachedDestination = false;
boolean atLimit = false;

long debouncing_time = 10; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;    //Counts the Time between the Last Interrupt for Debouncing
volatile int bottomLimitSwitchStatuses[3];    //0=X, 1=Y, 2=Z
volatile int topLimitSwitchStatuses[3];     //0=X, 1=Y, 2=Z

//Motor Variables
//Arrays with 0=X, 1=Y, 2=Z
int pulsesPerInch[3];
int pulsesPerRevolution[3];
int revolutionsPerInch[3];
int motorVelocity[3];
int motorAcceleration[3];
long nrOfPulsesMotor[3];

float userSpeedInput;
float userSteppingModeInput;
float userAccelerationInput;

long pausedTargetPosition[3];
long pausedCurrentPosition[3];
bool singleMoveCmd=false;
boolean systemPaused=false;

//Objects
AccelStepper StepperMotor[3]={AccelStepper(1, 5, 4), AccelStepper(1, 6, 7), AccelStepper(1, 9, 8)};
ByteBuffer rxBuffer(64);

// Initialize USART connection and baud rate
void USART0Init(void)
{
    // Set baud rate
    UBRR0H = (uint8_t)(UBRR_VALUE>>8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);                  // Set frame format to 8 data bits, no parity, 1 stop bit
    UCSR0B |= (1<<RXEN0)|(1<<RXCIE0)| (1 << TXEN0);     //enable reception and RC complete interrupt
}

//********Interrupt Service Routines*************

//RX Complete interrupt service routine
ISR(USART_RX_vect)
{
    uint8_t u8temp;
    u8temp=UDR0;
    rxBuffer.put(u8temp);
    processReceivedData();
}

// Function to execute when an Interrupt occurs due to limit switch
void LimitSwitchISR(void)
{
   if((long)(micros() - last_micros) >= debouncing_time * 1000)
   {

     GetLimitSwitchArray();

     for(int i=0; i<3; i++)
     {
       if(bottomLimitSwitchStatuses[i] == 1)
       {
         StepperMotor[i].setCurrentPosition(0);
         SendInt8UART(i+3);

       }
       if(topLimitSwitchStatuses[i] == 1)
       {
         StepperMotor[i].setCurrentPosition(upperLimitInSteps[i]);
         SendInt8UART(i+6);
       }
     }

     atLimit=true;
     PauseSystem();
     last_micros = micros();
    }
}


//***********************************************

void ConfigurePins(void)
{
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);

  //pinMode(13, OUTPUT);
}

void SetUpMotorParameters()
{
    pulsesPerInch[0]=400*0.127324;
    revolutionsPerInch[0] = 1;
    pulsesPerRevolution[0] = pulsesPerInch[0] * revolutionsPerInch[0];

    pulsesPerInch[1]=400;
    revolutionsPerInch[1] = 1;
    pulsesPerRevolution[1] = pulsesPerInch[1] * revolutionsPerInch[1];

    pulsesPerInch[2]=400;
    revolutionsPerInch[2] = 1;
    pulsesPerRevolution[2] = pulsesPerInch[2] * revolutionsPerInch[2];


    upperLimitInSteps[0]=40*pulsesPerInch[2];        //40 inches is the max limit for x-axis
    upperLimitInSteps[1]=40*pulsesPerInch[2];        //40 inches is the max limit for y-axis
    upperLimitInSteps[2]=40*pulsesPerInch[2];        //40 inches is the max limit for z-axis

    for(int i=0; i<3; i++)
    {
      motorVelocity[i] = 6;
      motorAcceleration[i] =15;

      StepperMotor[i].setMaxSpeed(motorVelocity[i] * pulsesPerInch[i]);
      StepperMotor[i].setAcceleration(motorAcceleration[i] * pulsesPerInch[i]);
      StepperMotor[i].setCurrentPosition(0);
    }
}

void SetUpLimitSwitchInterrupt(void)
{
  attachInterrupt(0, LimitSwitchISR, HIGH);

}

int main (void)
{
    init();
    ConfigurePins();
    SetUpLimitSwitchInterrupt();
    SetUpMotorParameters();
    USART0Init();            //Initialize USART0
    sei();                   //enable global interrupts

    while(1)
    {
      if(commandsReceived)
      {
        ActuateXMotor();
        ActuateYMotor();
        ActuateZMotor();
      }

      if(motorsReachedDestination)
      {
        commandsReceived = false;
        receiveNextCommands();
        motorsReachedDestination = false;
      }

      singleMoveCmd=false;
      delay(1000);

    }
}


// Process received coordinate data from PC with a buffer.
void processReceivedData(void)
{
  uint8_t cmdByte;
  uint8_t removedByte;
  uint8_t escByte;

  if(rxBuffer.getSize()>=dataPacketSize)
  {

    cmdByte = rxBuffer.peek(1);// - 48;      //Subtract 48 to turn from an ASCII character to INT
    switch(cmdByte)
    {
      case 0:
        removedByte = rxBuffer.get();      //Remove StartByte
        removedByte = rxBuffer.get();      //Remove CmdByte
        userCommands[0]= rxBuffer.getFloat();
        userCommands[1]= rxBuffer.getFloat();
        userCommands[2]= rxBuffer.getFloat();
        removedByte = rxBuffer.get();      //Remove EscapeByte;
        commandsReceived = true;
        break;
      case 1:
        //Clear Data Packet & Pause
        for(int i=0; i<dataPacketSize; i++)
        {
          removedByte = rxBuffer.get();
        }
        PauseSystem();
        break;
      case 2:
        for(int i=0; i<dataPacketSize; i++)
        {
          removedByte = rxBuffer.get();
        }
        ResumeSystem();
        break;
      case 3:
        for(int i=0; i<dataPacketSize; i++)
        {
          removedByte = rxBuffer.get();
        }
        StopSystem();
        break;
      case 4:
        for(int i=0; i<dataPacketSize; i++)
        {
          removedByte = rxBuffer.get();
        }
        ResetSystem();
        break;
      case 5:
        removedByte = rxBuffer.get();      //Remove StartByte
        removedByte = rxBuffer.get();      //Remove CmdByte
        userSteppingModeInput= rxBuffer.getFloat();
        userSpeedInput= rxBuffer.getFloat();
        userAccelerationInput= rxBuffer.getFloat();
        escByte = rxBuffer.get();      //Remove EscapeByte;

        pulsesPerInch[escByte]=userSteppingModeInput;
        pulsesPerRevolution[escByte] = pulsesPerInch[escByte] * revolutionsPerInch[escByte];
        motorVelocity[escByte] = userSpeedInput;
        motorAcceleration[escByte] = userAccelerationInput;

        StepperMotor[escByte].setMaxSpeed(motorVelocity[escByte] * pulsesPerInch[escByte]);
        StepperMotor[escByte].setAcceleration(motorAcceleration[escByte] * pulsesPerInch[escByte]);
        StepperMotor[escByte].setCurrentPosition(0);
        commandsReceived = true;
        break;

      case 6:                              //Case of Single Command
        singleMoveCmd=true;
        removedByte = rxBuffer.get();      //Remove StartByte
        removedByte = rxBuffer.get();      //Remove CmdByte
        singleCommands[0]= rxBuffer.getFloat();
        singleCommands[1]= rxBuffer.getFloat();
        singleCommands[2]= rxBuffer.getFloat();
        escByte = rxBuffer.get();      //Remove EscapeByte;
        commandsReceived = true;
        userCommands[0]=0;
        userCommands[1]=0;
        userCommands[2]=0;
        userCommands[escByte]= singleCommands[escByte];     //Only the specified motor (given by Escape byte) is moved
        break;
      case 7:
        for(int i=0; i<dataPacketSize-1; i++)
        {
          removedByte = rxBuffer.get();
        }
        escByte = rxBuffer.get();
        StepperMotor[escByte].setCurrentPosition(0);

    }
  }
}

// Send a "1" to PC to signal Arduino is ready to
// receive the next set of commands
void receiveNextCommands(void)
{
  SendInt8UART(1);        //Send signal to PC, so that it sends next coordinates
}

void ActuateXMotor(void)
{
  nrOfPulsesMotor[0] = long(userCommands[0] * pulsesPerInch[0]);
  StepperMotor[0].moveTo(nrOfPulsesMotor[0]);
  if(singleMoveCmd)
  {
    StepperMotor[0].move(nrOfPulsesMotor[0]);
    digitalWrite(12, HIGH);
  }


  while(abs(StepperMotor[0].distanceToGo())>0 && systemPaused == false && atLimit ==false)
  {
    StepperMotor[0].run();
  }

}

void ActuateYMotor(void)
{
  nrOfPulsesMotor[1] = long(userCommands[1] * pulsesPerInch[1]);
  StepperMotor[1].moveTo(nrOfPulsesMotor[1]);
  if(singleMoveCmd)
  {
    StepperMotor[1].move(nrOfPulsesMotor[1]);
  }

  while(abs(StepperMotor[1].distanceToGo())>0 && systemPaused == false && atLimit ==false)
  {
    StepperMotor[1].run();
  }
}

void ActuateZMotor(void)
{
  nrOfPulsesMotor[2] = long(userCommands[2] * pulsesPerInch[2]);

    StepperMotor[2].moveTo(nrOfPulsesMotor[2]);
  if(singleMoveCmd)
  {
    StepperMotor[2].move(nrOfPulsesMotor[2]);
  }

  while(abs(StepperMotor[2].distanceToGo())>0 && systemPaused == false && atLimit ==false)
  {
    StepperMotor[2].run();
  }

  if(systemPaused == false)
    motorsReachedDestination = true;
}

//************SYSTEM FUNCTIONS**************

// Pause carriage operation
void PauseSystem(void)
{
  for(int i=0; i<3; i++)
  {
    pausedTargetPosition[i] = StepperMotor[i].targetPosition();
    pausedCurrentPosition[i] = StepperMotor[i].currentPosition();
    StepperMotor[i].setCurrentPosition(pausedCurrentPosition[i]);     //This sets speed to 0, to allow for acceleration again
  }

  //StepperMotor[2].stop();
  //StepperMotor[2].runToPosition();
  systemPaused=true;
}

// Resume normal carriage operation after pause.
void ResumeSystem(void)
{
  if(systemPaused)
  {
    systemPaused=false;
    for(int i=0; i<3; i++)
    {
      StepperMotor[i].moveTo(pausedTargetPosition[i]);
    }
  }

  if(atLimit==true)
  {
    atLimit=false;
    for(int j=0; j<3; j++)
    {
      nrOfPulsesMotor[j] = long(userCommands[j] * pulsesPerInch[j]);
      StepperMotor[j].moveTo(nrOfPulsesMotor[j]);
    }
  }
}

void StopSystem(void)
{
}

void ResetSystem(void)
{
}

// Read status of limit switches
void GetLimitSwitchArray(void)
{
  bottomLimitSwitchStatuses[0]=digitalRead(14);
  bottomLimitSwitchStatuses[1]=digitalRead(15);
  bottomLimitSwitchStatuses[2]=digitalRead(16);
  topLimitSwitchStatuses[0]=digitalRead(17);
  topLimitSwitchStatuses[1]=digitalRead(18);
  topLimitSwitchStatuses[2]=digitalRead(19);
}

//**************HELPER FUNCTIONS *******************

// Sends 8 Bits of Data
void SendInt8UART(uint8_t data) {
    while ((UCSR0A & (1 << UDRE0)) == 0);  //make sure the data register is cleared
    UDR0 = data;                           //send the data
}

// Sends 16 bits of Data
void SendInt16UART(uint16_t data) {
    while ((UCSR0A & (1 << UDRE0)) == 0);  //make sure the data register is cleared
    UDR0 = data;                           //send the lower bits
    while ((UCSR0A & (1 << UDRE0)) == 0);  //make sure the data register is cleared
    UDR0 = (data >> 8);                    //send the higher bits
}

// Use this to send a string, it will split it up into individual parts
// send those parts, and then send the new line code
void SendStrUART(char *data) {
    while (*data) {
        while ((UCSR0A & (1 << UDRE0)) == 0);//make sure the data register is cleared
        UDR0 = *data; //goes through and splits the string into individual bits, sends them
        data += 1;//go to new bit in string
    }
}

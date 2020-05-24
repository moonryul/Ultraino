/*Please do not forget to add the following inline functions

inline bool _dataAvailable() {return _rx_buffer_head != _rx_buffer_tail; }
inline byte _peekData() { return _rx_buffer[_rx_buffer_tail]; }
inline void _discardByte() { _rx_buffer_tail = (rx_buffer_index_t)(_rx_buffer_tail + 1) % SERIAL_RX_BUFFER_SIZE; }

In the public interface of Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino\HardwareSerial.h

*/



#include <avr/sleep.h>
#include <avr/power.h>

//#include <SoftwareSerial.h>


#define N_PATTERNS 32

// Each drive board/arduiono can hangle 100 bits/channels of data per each pattern (period)
// by pattern by pattern, one pattern per iteration of the following LOOP:


#define N_PORTS 10
#define N_DIVS 10

//ports A C L B K F H D G J ; Port E is not used for data ports; 

//# The four types of byte message:
//#        MSB (command)    LSB (board number: 0 means all borads)     Data Command or Control Command

//# (1)     0000                0000 : Switch Patterns                    Control Command
//#         This means: Switch/swap the pattern buffers, 
//                  so that the new patterns (pattern 0, pattern 1,..., pattern 31) may be emitted
//        (i)  The ith pattern (received from Ultraino) specifies the one period (with 10 steps) for all of the 10 ports
//        (ii) The ith pattern (period) will be repeated by the number of times specifed in the ith duration, specifed by the 
//             "Add Duration" command, as described in the following. The unit of duration is one period.
//        (iii) One pattern with a given duration (number of repeating the period) represents a periodic singal
//              which lasts  for the given duration
//              
//# (2)     0001                0000 : Switch Durations                  Control  Command
//#         (2) means: Switch/swap the duration buffers, so that the new durations 
//#                                        (duration 0, duration 1,..., duration 31) may be emitted
//#         
//# (3)     XX11                0000 : Add Duration XX                   Data    Command
//#            (3) means: Add/Specify the duration of the current pattern in the duration buffer#          

//# (4)     XXXX                YYYY: Add Pattern [XXXX to Board YYYY]   Data    Command
//#            (4) means: Add/Specify four bits (XXXX) [half byte] of pattern (signals to the transducers) to the pattern buffer
//#                To fill one pattern (one period) for all the ports, we need 10 * 10 bytes (N_DIV * N_PORT).
//#                It means that we need 100 bytes / 0.5 byte = 200 Add Pattern messages from Ultraino. Be careful that
//#                the Ultraino paper and the Arduino code do not explain this in details. You need to read between the lines,
//#                which can be detected by those who understand message sending and receiving at low levels.

#define COMMAND_SWITCH 0b00000000  // This is the Switch Buffers
#define COMMAND_DURATION 0b00110000 
    // This is used to detect whther the received message is of the form  "XX11 0000", 
    // that is, Add Duration command
    
#define MASK_DURATION 0b00111111 
      // This is used to extract XX from "XX11 0000", which is  Add Duration Command

#define COMMAND_COMMITDURATIONS 0b00010000 // This is the Switch Durations

#define WAIT(a) __asm__ __volatile__ ("nop")

#define OUTPUT_WAVE(pointer, d)  PORTA = pointer[d*N_PORTS + 0]; PORTC = pointer[d*N_PORTS + 1]; PORTL = pointer[d*N_PORTS + 2]; PORTB = pointer[d*N_PORTS + 3]; PORTK = pointer[d*N_PORTS + 4]; PORTF = pointer[d*N_PORTS + 5]; PORTH = pointer[d*N_PORTS + 6];  PORTD = pointer[d*N_PORTS + 7]; PORTG = pointer[d*N_PORTS + 8]; PORTJ = pointer[d*N_PORTS + 9]

static byte bufferA[N_PATTERNS * N_DIVS * N_PORTS];
static byte bufferB[N_PATTERNS * N_DIVS * N_PORTS];

void setup()
{
  
// confer mega pin assingment:https://www.arduino.cc/en/uploads/Hacking/PinMap2560big_Rev2.png
// https://www.arduino.cc/en/Hacking/PinMapping2560

// The chip has several ports:  A C L B K F H D G J
// https://forum.arduino.cc/index.php?topic=572128.0
//Each port is controlled by three registers, 
//which are also defined variables in the arduino language. 
//https://www.arduino.cc/en/Reference/PortManipulation
//(1) The DDR register, determines whether the pin is an INPUT or OUTPUT. Read/Write
// (2) The PORT register controls whether the pins of the port are HIGH or LOW,
//                ie, bit patterns of the one byte port; Read/Write
// (3) the PIN register reads the state of INPUT pins set to input with pinMode(). ReadOnly

//set all bits of ports A C L B K F H D G J as output
// by setting the DDR register of each port to all 1 bits:
//PORTD is the register for the state of the outputs. For example;
//PORTD = B10101000; // sets digital pins 7,5,3 HIGH

// PORTB |= (1<<PB2); //set bit 2 of Port B, where PB2 is defined to be 2
// PORTB &= ~(1<<PB1); //clear bit 1 of Port B, where PB1 is defined to be 1
// In the setting below, do not do anything for pin 18 (D3) and 19 (D2), as they are used for Tx and Rx

 DDRA = DDRC = DDRL = DDRB = DDRK = DDRF = DDRH = DDRD = DDRG = DDRJ = 0xFF;
 // modified by MJ
 DDRD != (1 << PD3); // set D3 (18) = Tx to 1
 DDRD &= ~(1 << PD2); // set D2 (19) = Rx to 0
   
  // Set all of them to zero value
  // The PORT register controls whether the pin is HIGH or LOW
  PORTA = PORTC = PORTL = PORTB = PORTK = PORTF = PORTH = PORTD = PORTG = PORTJ = 0x00;

  //clear the buffers
  for (int i = 0; i < (N_PATTERNS * N_DIVS * N_PORTS); ++i) {
    bufferA[i] = bufferB[i] = 0;
  }

//initial pattern: Set the half of the first pattern to 0xFF
  for (int i = 0; i < (N_PORTS*N_DIVS/2); ++i){
     bufferA[i] = 0xFF;
  }
  

  //MJ: Pin Assignment Debugging:
  // Port Sequence: A C L B K F H D G J 
        //   => A has portIndex =0, C has portIndex =1, B has portIndex =2, etc. 
  // Set a patterns of 01s at Port A (portIndex =0), for debuging and adjusting times

  //int portIndex = 0; // MJ: change portIndex to 1,2,3,... 9 and repeat the experiment.
  
  //for(int i = 0; i < N_DIVS; ++i){ // for each step of  the first pattern (period)
   // if (i % 2 == 0){ // step index i = 0,2,4,6,
     // the 3rd bit is 1 in the port with portIndex 0 (port A) at the even steps.
   //   bufferA[i * N_PORTS + portIndex] |= 0b00001000; 
        
   // }else{ //  step index i = 1,3,5,7,
     // the 3rd bit is 0 in the port with portIndex 0 (port A) at the odd steps.
   //   bufferA[i * N_PORTS + portIndex] &= 0b11110111;
      
   // }
  //}
 
  


/* On the Arduino Mega we have 6 timers and 15 PWM outputs:

Pins 4 and 13: controlled by Timer0
Pins 11 and 12: controlled by Timer1
Pins 9 and10: controlled by Timer2
Pin 2, 3 and 5: controlled by timer 3
Pin 6, 7 and 8: controlled by timer 4
Pin 46, 45 and 44:: controlled by timer 5
*/
  // generate a sync signal of 40khz in pin 2 (using TCCR3A, TCCR3B, OCR3A, OCR3B registers)
  
//http://www.eprojectszone.com/40khz-square-signal-with-arduino-uno/

// Mega pin 2 = PORTE4 (PWM), Pin 3 = PORTE5 (PWM)

  pinMode (2, OUTPUT);

  noInterrupts();           // disable all interrupts

//http://www.righto.com/2009/07/secrets-of-arduino-pwm.html

// https://withinspecifications.30ohm.com/2014/02/20/Fast-PWM-on-AtMega328/

//http://www.hardcopyworld.com/gnuboard5/bbs/board.php?bo_table=lecture_pract&wr_id=12
//https://www.avrfreaks.net/forum/pwm-40khz-pulse-doesnt-seem-work-right-included-pic


//You know the output compare registers, like OCRA and OCRB?
// TCCRx (Timer/Counter Control Register); x = timer index

  TCCR3A = bit (WGM10) | bit (WGM11) | bit (COM1B1); // fast PWM, clear OC1B on compare
  TCCR3B = bit (WGM12) | bit (WGM13) | bit (CS10);   // fast PWM, no prescaler

// The ATmega328P has three timers known as Timer 0, Timer 1, and Timer 2.
// Each timer has two output compare registers that control the PWM width for the timer's two outputs: when the timer reaches the compare register value, the corresponding output is toggled. The two outputs for each timer will normally have the same frequency, but can have different duty cycles (depending on the respective output compare register).

// Each of the timers has a prescaler that generates the timer clock 
// by dividing the system clock by a prescale factor such as 1, 8, 64, 256, or 1024.
// The Arduino has a system clock of 16MHz and the timer clock frequency will be
// the system clock frequency divided by the prescale factor. 
// Note that Timer 2 has a different set of prescale values from the other timers.


//The main PWM modes are "Fast PWM" and "Phase-correct PWM"
// In the simplest PWM mode, the timer repeatedly counts from 0 to 255. 
// The output turns on when the timer is at 0, and turns off
// when the timer matches the output compare register. 
// The higher the value in the output compare register, the higher the duty cycle. 
// This mode is known as Fast PWM Mode. 


//The Output Compare Registers OCRnA and OCRnB set the levels 
//at which outputs A and B will be affected. 
//When the timer value matches the register value, 
//the corresponding output will be modified as specified by the mode.

  OCR3A =  (F_CPU / 40000L) - 5; 
     //should only be -1 but fine tunning with the scope determined that -5 gave 40kHz almost exactly
  OCR3B = (F_CPU / 40000L) / 2;

  interrupts();             // enable all interrupts

  //sync in signal at pin 3
// Mega pin 3 (PWM); Pin 3 = PORTE5 (PWM)
// At present, the mode of pin 3 is Input, and make it INPUT_PULLUP mode
//Pin 3  = PORTE5 (OC3C / INT5) (PWM)


//Pin Mode: INPUT_PULLUP: https://en.wikipedia.org/wiki/Pull-up_resistor
//In electronic logic circuits, a pull-up resistor or pull-down resistor is a resistor
//used to ensure a known state for a signal. It is typically used in combination with components 
//such as switches and transistors, which physically interrupt the connection of 
//subsequent components to ground or to VCC. 
//When the switch is closed, it creates a direct connection to ground or VCC,
//but when the switch is open, the rest of the circuit would be left floating
//(i.e., it would have an indeterminate voltage). For a switch that connects to ground, 
//a pull-up resistor ensures a well-defined voltage (i.e. VCC, or logical high) 
//across the remainder of the circuit when the switch is open.

//There are 20K pullup resistors built into the Atmega chip that can be accessed from software.
//These built-in pullup resistors are accessed by setting the pinMode() as INPUT_PULLUP.

//When connecting a sensor to a pin configured with INPUT_PULLUP, 
//the other end should be connected to ground. In the case of a simple switch, 
//this causes the pin to read HIGH when the switch is open, and LOW when the switch is pressed.
// https://electronics.stackexchange.com/questions/67007/whats-the-difference-between-input-and-input-pullup

  pinMode(3, INPUT_PULLUP); //please connect pin3 to pin 2 in the hardware; 
  // Pin 2 (clock signal)plays the role of switch for the pin 3 to which the pullup resistor is attached.
  // Pin 3 is on when the pin 2 signals goes LOW.
    //  A setting was done above to generate a sync signal of 40khz in pin 2 (clock signals)
  // Clock signals from pin 2 will go to pin 3, which will be checked within LOOP below.
   //**MJ: do you connect pin3 to pin 2 in the hardware??
   
  // disable everything that we do not need
  ADCSRA = 0;  // ADC
  power_adc_disable ();
  power_spi_disable();
  power_twi_disable();
  power_timer0_disable();
  power_usart1_disable();
  power_usart2_disable();
  power_usart3_disable();
  //power_usart0_disable();

  Serial.begin(115200);
  Serial1.begin(115200);

  // Serial Communication protocol:
  // https://www.deviceplus.com/arduino/arduino-communication-protocols-tutorial/
  //For TX, any pin can be used. For RX, only interrupt-enabled pins can:
// so only the following can be used for RX: 10, 11, 12, 13, 50, 51,
//52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69).

//https://books.google.co.kr/books?id=_IWHCAAAQBAJ&pg=PT336&lpg=PT336&dq=softwareserial+RX+TX+usable+pins&source=bl&ots=TJV-BWHcWr&sig=ACfU3U0SFYy25ekRk4ed8jGxeqo5vgJGdg&hl=en&sa=X&ved=2ahUKEwiS4MO9jL3pAhULfnAKHXL0BVkQ6AEwCXoECAoQAQ#v=onepage&q=softwareserial%20RX%20TX%20usable%20pins&f=false

  //int softRx = ;
  //int softTx = ;
  //SoftwareSerial softSerial(softRx, softTx); // RX, TX
   // set the data rate for the SoftwareSerial port
  //softSerial.begin(115200);

  // you can use softSerial.print()
  //SerialSoftware:
  // The SoftwareSerial library has been developed to allow serial communication
  //on other digital pins of the Arduino, using software to replicate the functionality 
  //(hence the name "SoftwareSerial").
  //It is possible to have multiple software serial ports with speeds up to 115200 bps.
  //A parameter enables inverted signaling for devices which require that protocol.

  byte byteReceived = 0;

  bool byteReady = false;
  bool isSwitch = false;
  bool isPatternForMe = false;
  bool isDuration = false;
  bool isCommitDurations = false;
  byte nextMsg = 0;
  int writtingIndex = 0;

// Emit data from the A buffer initially
  bool emittingA = true; 

// Why use two emitting pointers, emittingPointerH and emittingPointerL to different locations of the same bufferA?
  byte* emittingPointerH = & bufferA[0]; 
    // the initial value of emittingPointerH, which will change
  byte* emittingPointerL = & bufferA[N_PORTS * N_DIVS / 2]; 
  
    // the initial value of emittingPointerL, which will change
  byte* emittingPointerZeroH = & bufferA[0]; // the initial value of emittingPointerH
  byte* emittingPointerZeroL = & bufferA[N_PORTS * N_DIVS / 2]; // the initial value of emittingPointerL
  

// Read data to B the buffer initially
  byte* readingPointerH = & bufferB[0]; //// the initial value of readingPointerH, which will change
  byte* readingPointerL = & bufferB[N_PORTS * N_DIVS / 2];
   //// the initial value of readingPointerL, which will change

  byte durations[N_PATTERNS]; // the array of 32 durations; 
   // The ith duration is the duration of the ith  pattern
  byte durationsBuffer[N_PATTERNS]; // the buffer of durations

  for(int i = 0; i < N_PATTERNS; ++i){
    durations[i] = durationsBuffer[i] = 0;
  }

// set the first pattern's duration to be one period

  durations[0] = durationsBuffer[0] = 1;

  byte currentPattern = 0;
  byte currentPeriods = 0;
  byte durationsPointer = 0;
  byte currentDuration = 0;
  bool patternComplete = false;
  bool lastPattern = false;
  byte nextPattern = 0;
  byte nextDuration = 0;
  bool returnToFirstPattern = false;
 

LOOP:

// The loop reads the data from the sender (from the serial port) 
// and emits the sequence of message pattens to the transducer
// Both tasks are performed in a interleaving manner. 

// Every clock tick, send one period of PWM signal to each transduer

 while (PINE & 0b00100000); 
 //wait for pin 3 (the 5th on port E) = the clock signal with 40Khz  (PORTE5 Sync In) to go low;
 
// Pin 3  = PORTE5 (OC3C / INT5) (PWM)

// Emit data from the emitting buffer address pointing to the CURRENT pattern
// The SAME pattern will be sent by the number of times indicated 
// by the duration of the current pattern
// When the duration of the current pattern is used up, 
// the emitting buffer address will point to the
// next pattern

// NOTE: At the very beginning, the emittingBuffer will contain the 0 bits
// because it has not been filled  by the byte message from Ultraino. 

  OUTPUT_WAVE(emittingPointerH, 0); 
  // This  sets the bits of the all of the 10 ports corresponding to the 0th step (division)
  // of the current period of the current  pattern; 
  // The total number of patterns that can be stored in ArduinoMega is 32 patterns.
  //confer to section "C. Driver Board" in  Ultraino paper:
  // So, it is expected that every 32 patterns, the Ultraino sends the swap buffers command 
   //Note also that the maximum supported steps per period at 40 kHz was 10, 
  // giving a phase resolution of pi/5 (=2pi/10); 
  
  // To understand macro OUTPUT_WAVE(emittingPointerH, d=0), remember:
  // d=divison/step number;  emittingPointerH is an array of bytes:
  // static byte emittingPointerH[N_PATTERNS * N_DIVS * N_PORTS],
  // where N_PATTERNS =32, N_DIVS =10, N_PORTS  =10; 
  // N_DIVS =10 means that the one period T (cycle) of a periodic signal (wave)
  // is divided (discretized) into 10 steps/divisions. 
  // confer fig. 7 of Ultraino papper, which shows how ONE period of the signal
  // are discretized into 10 steps. 
  // The number of one's (ON) within the period T rerepresents the duty cycle of
  // PWM signal, whicn is then used to compute the RMS amplitude of the signal.
  // https://masteringelectronicsdesign.com/how-to-derive-the-rms-value-of-pulse-and-square-waveforms/
  
 
    
  // OUTPUT_WAVE(emittingPointerH, d=0) sets the 10 ports from the 0th step of the current pattern
  // as follows: 
  //  PORTA = emittingPointerH[0*N_PORTS + 0]; PORTC = emittingPointerH[0*N_PORTS + 1]; 
  // PORTL = emittingPointerH[0*N_PORTS + 2];   PORTB = emittingPointerH[0*N_PORTS + 3]; 
  // PORTK = emittingPointerH[0*N_PORTS + 4]; PORTF = emittingPointerHpointer[0*N_PORTS + 5];
  //  PORTH = emittingPointerH[0*N_PORTS + 6];  PORTD = emittingPointerH[0*N_PORTS + 7];
  // PORTG = emittingPointerH[d*N_PORTS + 8]; PORTJ = emittingPointerH[0*N_PORTS + 9]

byteReady = Serial._dataAvailable(); // read the data at the serial port
// When there is no data available, byteReady becomes false. 
// So, the part of the code  if (byteReady) { } is skipped, 
//and the current emitting buffer is sent to the transducers 
//continuously. This is used for debugging the messages and pin assignment.
// 

  OUTPUT_WAVE(emittingPointerH, 1);
   // This sets the bits of the all of the 10 ports corresponding to the 1th step (division) 
   // of the current message pattern, as follows:

  //  PORTA = emittingPointerH[1*N_PORTS + 0]; PORTC = emittingPointerH[1*N_PORTS + 1]; 
  // PORTL = emittingPointerH[1*N_PORTS + 2];   PORTB = emittingPointerH[1*N_PORTS + 3]; 
  // PORTK = emittingPointerH[1*N_PORTS + 4]; PORTF = emittingPointerHpointer[1*N_PORTS + 5];
  //  PORTH = emittingPointerH[1*N_PORTS + 6];  PORTD = emittingPointerH[1*N_PORTS + 7];
  // PORTG = emittingPointerH[1*N_PORTS + 8]; PORTJ = emittingPointerH[1*N_PORTS + 9]


 byteReceived = Serial._peekData(); 
 // get the byte which has arrived at the serial port from the PC; 

  OUTPUT_WAVE(emittingPointerH, 2); // Set the 10 ports of the 2th step (division) of the current pattern
  
//#define COMMAND_SWITCH 0b00000000  // This is the Switch Buffers
//#define COMMAND_DURATION 0b00110000 
    // This is used to detect whther the received message is of the form  "XX11 0000", Add Duration command
    
//#define MASK_DURATION 0b00111111 
    // This and COMMAND_DURATION are  used to check if the received byte is of the form  "XX11 0000", Add Duration Command

//#define COMMAND_COMMITDURATIONS 0b00010000 // This is the Switch Durations

// Is the received command COMMAND_SWITCH "0b00000000" ?

isSwitch = byteReceived == COMMAND_SWITCH;  // Switch Buffers

// Is the received comman == COMMAND_COMMITDURATIONS,Switch Durations command ?

isCommitDurations = byteReceived == COMMAND_COMMITDURATIONS; 

  OUTPUT_WAVE(emittingPointerH, 3); // set 3th step (division), 10 ports

// Does the received byte has LSB whose value is 1 (the current board = Me)?

 isPatternForMe = (byteReceived & 0b00001111) == 1; 

  OUTPUT_WAVE(emittingPointerH, 4); // set 4th step (division)

 nextMsg =  byteReceived - 1;  
 // Assuming that byteReceived is pattern data message of the form XXXX YYYY,
 // set nextMsg to  XXXX ZZZZ, where the value of ZZZ is the value of YYYY - 1
 // This byte message will be sent to the next arduino, if indeed byteReceived is the data message
 // for the next boards.

  OUTPUT_WAVE(emittingPointerL, 0); 
  // 5th step (division), 10 ports; the 0th step of emittingPointerL is the 5th of the whole buffer

//  is the received byte the ADD DURATION Command? 
isDuration = (byteReceived & MASK_DURATION) == COMMAND_DURATION; 
   //  MASK_DURATION == 0b00111111; COMMAND_DURATION = 0b00110000 

//nextPattern = currentPattern + 1;
   // Both the currentPattern and the nextPattern variables are maintained.
   // The initial value of currentPattern is 0 

OUTPUT_WAVE(emittingPointerL, 1); // 6th step (division), 10 ports

//nextDuration = durations[nextPattern];

 //nextDuration = durations[ currentPattern + 1 ];
 
// nextPattern = 1 at this point in the first iteration of the loop
// The initial value of durations[0] = 1; But durations[1]=0, durations[2]=0,..etc. 
// initially, that is, without getting data from Ultraino. 
 // 

  OUTPUT_WAVE(emittingPointerL, 2); // 7th step (division), 10 ports
  
 // When the currentPeriod of the CURRENT pattern has reached the duration of the pattern,
 // it is considered that the pattern has been completed.

 
++currentPeriods; 
// The currentPeriod is incremented each tick (period) of 40KHz clock;
// The initial value of currentPeriods is 0.
// Note period T = 1/f = 1/ 40KHz = 1 / (40K/s) = 1/40K seconds = 0.025 ms

// When the current pattern (period) has been sent by the number of times
// denoted by durations[currentPattern], the pattern will be considered completed

patternComplete = (currentPeriods == durations[currentPattern]); 
 // Initially, durations[currentPattern]= durations[0] = 1;
 // At the first iteration, currentPeriods =1; So, patternComplete is true.
// So, this condition is automatically true when the durations of patterns 
// have not been set by the received message.

  OUTPUT_WAVE(emittingPointerL, 3); // 8th step (division), 10 ports

 lastPattern = (currentPattern+1 == N_PATTERNS); //  All of N_PATTERNS patterns have arrived. 

//returnToFirstPattern = ( nextDuration == 0 );
returnToFirstPattern = ( durations[ currentPattern + 1] == 0 );
 
// If the duration of the next pattern is zero, it means the pattern is the "stop frame",
// we need to go back to the first pattern and repeat. But in the meanwhile, new patterns
// may have arrived from Ultraino, and the new emitting Buffer may be ready. In that case,
// the previous patterns are not repeated but the new patterns are used.

  OUTPUT_WAVE(emittingPointerL, 4); // 9th step (division), 10 ports

// Now ALL the steps of the current pattern are sent to the transducer
 
if (patternComplete)
{

 // the CURRENT pattern has been repeated by the number of times defined 
// by durations[currentPattern]: 
 // cf. patternComplete = (currentPeriods == durations[currentPattern]);  

  //currentPeriods = 0; // go to the first period of the next pattern

  //++currentPattern; // go to the next pattern

  // check if the completed pattern is also the end of the sequence of patterns or not
  if (lastPattern  || returnToFirstPattern)
  {

 // if the last pattern (pattern 32)  has been reached or the sequence of patterns has stoped, go to the first pattern. 
 // So, the current emitting buffer pointers are set to point to the beginning of the buffer which will contain
 // the first pattern.
 // This condition also holds immediately during the pin assignment debugging process 
 
    currentPattern = 0;
    currentPeriods = 0; // go to the first period of the current pattern
    emittingPointerH = emittingPointerZeroH;
    emittingPointerL = emittingPointerZeroL;
  } //  if (lastPattern  || returnToFirstPattern)
  
 else
 { // The completed current pattern is not the last pattern
  
// goto the next pattern:

    ++currentPattern;
    
    currentPeriods = 0; // go to the first period of the current pattern

// Let the emitting buffer point to the new location for the new pattern
    emittingPointerH += (N_DIVS * N_PORTS);
    emittingPointerL += (N_DIVS * N_PORTS);
    
 // The pattern pointed to by emittingPointerH and emittingPointerL will be
 // sent to the ports of Mega / transducers in the next iteration of the loop.
  } 
  
} // if (patternComplete)

// If the current pattern is not yet completed, Now process the received byte: Unless data is received, just skip the block

if (byteReady) 

{ // process the received byte if   available

    if ( isSwitch ) 
    { //  "switch  buffers" command arrived 
      //   => swap the emission buffer and the reading buffer, so that 
      //  the Arduino can emit data from the previously reading buffer.

      Serial.write( COMMAND_SWITCH );
      // The current board/arduino receives Switch Buffer command from the PC or another
      // Arduino, and sends it to the next arduino connected to it, via serial communication
      

// double buffering: swap the buffers ( emitting + reading )
      
      emittingA = !emittingA; // bool emittingA = true initially
     
      if (emittingA)
      { // Will be emitting from the A buffer, while reading to the B buffer
        emittingPointerH = & bufferA[0];
        emittingPointerL = & bufferA[N_PORTS * N_DIVS / 2];

        readingPointerH = & bufferB[0];
        readingPointerL = & bufferB[N_PORTS * N_DIVS / 2];

      } 

      else 
      { // Now  emit data from the B buffer ( which was the reading buffer), while reading to the A buffer

        emittingPointerH = & bufferB[0];
        emittingPointerL = & bufferB[N_PORTS * N_DIVS / 2];

        readingPointerH = & bufferA[0];
        readingPointerL = & bufferA[N_PORTS * N_DIVS / 2];
      }

      emittingPointerZeroH = emittingPointerH;
      emittingPointerZeroL = emittingPointerL;
      
      writtingIndex = 0; // index to the reading byte buffer
      durationsPointer = 0;

      // Whenever the switch/swap buffers command arrives, 
      //  Print the total number of patterns in the emittingPointerH; the number of patterns in 
      // the emitting buffer is less or equal to 32.

      // MJ: Print the "voltage" pattern for all of the 64 transducers:
      //

      for ( int i =0; i < N_PATTERNS; i++ ) {
        // if the duration of the ith pattern is 0, it means it is the "stop frame".
        // In that case, break out of the for loop because we reached the end of the
        // sequence of patterns before reaching N_PATTERNS.

        if ( durations[i] == 0 ) {
          break;
        }

        // print the ith pattern; 
        Serial1.print(i + "th pattern:\n");
        
        // The pattern has N_PORTS (10)  bytes for N_PORTS ports each of N_DIVS (10) steps.
        for (int s =0; s < N_DIVS; s++ ) {
          Serial1.print(s + "th step:\n");
          
          for (int p =0; p <N_PORTS; p++) {
            byte byteData = emittingPointerH[ i* (N_DIVS*N_PORTS) + s*N_PORTS + p];
            Serial1.print(byteData, BIN); // print byteData in bit stream
            // Serial.print(val, BIN): val = any datatype, BIN= print in binary form
            Serial1.print("|");
            
          } // innermost for
          Serial1.print("\n"); // new line
        } // inner for
      } // outermost for

    }//  if ( isSwitch ) // swap buffers

 else if  ( isPatternForMe )
 { 
      // receivedByte = XXXX 0001 = Add Pattern Command, where XXXX is a half byte for the half of one byte port

      // write the recieved  byte to the reading buffer (which is the B buffer initially)
 
        if (writtingIndex % 2 == 0) { // writtingIndex =0, 2, 4,6...
          
          readingPointerH[writtingIndex / 2] = byteReceived & 0xF0; 
           // assign the higher 4 bits of the received byte X1X1X1X1 (from  X1X1X1X1 0001)
           //    to the higher 4 bits at the location writtingIndex/2 =0,1,2,...

        } else { // writtingIndex =1, 3, 5,7...==> writtingIndex / 2 = 0,1,2,3....
          readingPointerH[writtingIndex / 2] |= (byteReceived >> 4); 
          // assign the higher 4 bits of the received byte (from X2X2X2X2 0001) 
          //      to the lower 4 bits at the location writtingIndex/2 =0,1,2,...
       // The above if else statements means that the arduino receives "X1X1X1X1 0001" and "X2X2X2X2 0001" in a pair
       // to form a byte "X1X1X1X1 X2X2X2X2" for a port. 
       // The port sequence in a single pattern is:  A C L B K F H D G J; 
       // So, a sequence of one byte messages of the form  "X1X1X1X1 X2X2X2X2" will be assigned to A, C, ...J
       //  from the reading buffer after it has become emitting buffer. 
        //  ...
        }

        ++writtingIndex; // this index is incremented for each pattern byte
        // writingIndex points to the four bits (XXX) in the readingPointerH buffer; 
        // writtingindex /2 goes from 0 to N_PATTERNS * N_DIV * N_PORT - 1 = 32 * 10 * 10 -1 = 3200 - 1

    }// if  ( isPatternForMe )

 else if (isDuration)
 
 { // received byte = "add duration": XX11 0000;

        Serial.write( byteReceived );
        // send the durations of patterns also to the next drive board, because the durations of
        // patterns are for all the boards.

      // write the recieved duration part (XX) in XX11 0000  to the corresponding bit location of  the duration buffer
      // XX11 0000; XX11 0000;  ;XX11 0000;  ;XX11 0000 forms one complete duration (XXXX XXXX)
        if (durationsPointer % 4 == 0) { // durationsPointer =0
       

          durationsBuffer[durationsPointer / 4] = byteReceived & 0b11000000;
          // byteReceived & 0b11000000 = the duration bits (two bits)

        }
        else 
        { // durationsPointer =1 => durationsPointer % 4 = 1 => durationsPointer % 4 * 2 = 2
          // durationsPointer =2 => durationsPointer % 4 = 2 => durationsPointer % 4 * 2 = 4
          // durationsPointer =3 => durationsPointer % 4 = 3 => durationsPointer % 4 * 2 = 6

          durationsBuffer[durationsPointer / 4]  // durationsPointer / 4 =0
                    |= (byteReceived & 0b11000000) >> (durationsPointer % 4 * 2);
        }

        ++durationsPointer; // this index increases for each duration byte
        // durationsPointer points to each 2 bit (XX) of the duration buffer; 
        // durationsPointer/4  goes from 0 to N_PATTERNS -1  

    }//if (isDuration)

else

 if (isCommitDurations) { // Command "Switch Durations"; The durations for all the patterns are set.

      Serial.write( byteReceived ); // send the same command to the next drive board/arduino

// mvoe the durations buffer to the duratonns array; 
//If the duration of nth pattern is zero, it means that the pattern sequence stops here.

      for(int i = 0; i<N_PATTERNS; ++i){
        durations[i] = durationsBuffer[i];
      }
      durationsPointer = 0;
    } // isCommitDurations

  else 
  { // The received byte must a patter data message of the form XXXX YYYY, 
    //  but is not for the current arduino. Send it to the next board
      Serial.write( nextMsg ); // to the next drive board
  } // nextMsg

   Serial._discardByte(); // discard the current bytes to get the next one.
    
  } // the received byte message is ready

  goto LOOP; // go to Loop to read a new period of data synchronized to PWM signal of each  period whether or not byte was ready

} // setup()

void loop() {}

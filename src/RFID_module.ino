/*------------------------------------------------------------------------------
- PJRC Teensy 4.0 pin mapping - Hardware Revision v7.1

D5  - RDY/CLK ready/clock
D6  - DMOD demodulation (out from EM4095)
D7  - SHD shutdown

D22 - Status LED
D23 - Read LED

D18 - SDA
D19 - SCL

*///----------------------------------------------------------------------------
#include <Wire.h>

//----- declaring variables ----------------------------------------------------
const char SOFTWARE_REV[] = "v1.0.0"; //Current Version of the program

//LEDs
const uint8_t statusLED = 22;
const uint8_t readLED = 23;

//used for interfacing EM4095
const uint8_t DMOD = 6;         //dmod pin
const uint8_t SHD = 7;          //shutdown pin
const uint8_t CLK = 5;          //RDY/CLK Pin
const uint8_t pulseTime = 181;  //181 uS , 8688 Systick ticks

volatile uint8_t headerDetect = 0;  //count zeros (and one 1) to detect header
volatile uint8_t findstart = 1;     //flag to toggle looking for header

volatile uint8_t bittic = 0;      //count bits from tag
volatile uint8_t bytetic = 0;     //count bytes from tag

//timing of the ISR to detect long/short pulses of the EM4095
volatile uint32_t tsnap0 = 0;
volatile uint32_t tsnap1 = 0;

volatile uint8_t toc = 0;         //to help translate two short pulses
volatile uint8_t tagfetched = 0;  //flag to indicate a complete tag was found

volatile uint32_t freq = 0;       //counter to determine resonant frequency
uint32_t freqgrab;                //buffer for sending

uint32_t tagfetched_time0 = 0;    //time the tag was fetched

volatile uint8_t tagbytes[12];    //array containing tag-data, crc check and data-block for temperature
uint8_t last_tagbytes[12];        //contains the tag from the last time we read it

uint8_t buffer[7];                //array containing a copy of the last tag that was detected, emptied once sent, with raw temp, without crc

volatile uint8_t sendmode = 0;          //which data to send on request
volatile uint8_t measure_frequency = 0; //flag to do one frequency measurement

//##############################################################################
//##### SETUP ##################################################################
//##############################################################################
void setup(){
  //I2C Setup
  Wire.begin(0x09);             //join I2C Bus at address 9 (0-7 is reserved)
  Wire.onRequest(sendData);     //what to do when being talked to
  Wire.onReceive(receiveEvent); //what to do with data received
  
  //set pins
  pinMode(SHD,OUTPUT);        //shutdown
  digitalWrite(SHD,HIGH);     //disable antenna on startup
  pinMode(DMOD,INPUT);        //demodulation input
  pinMode(CLK,INPUT);         //clock signal input
  pinMode(statusLED,OUTPUT);  //status LED
  pinMode(readLED,OUTPUT);    //read LED
  pinMode(2,OUTPUT);          //for timing/debugging purposes
 
  //while (!Serial); //wait until serial connection is enabled

  //to start ISR, last entry of setup
  attachInterrupt(digitalPinToInterrupt(DMOD), tag_watch, CHANGE);
} 

//##############################################################################
//##### LOOP ###################################################################
//##############################################################################
void loop(){
  //shutdown: 30us until amplitude <= 1%
  //startup: 1700us until amplitude >= 99%
  
  //--- manual antenna mode (for tuning/debugging)
  // delay(1000);
  // digitalWriteFast(SHD,LOW);
  // digitalWrite(statusLED,HIGH);
  // delay(1000);
  // digitalWriteFast(SHD,HIGH);
  // digitalWrite(statusLED,LOW);

  if(sendmode == 1){ //if in setup mode do various things
    digitalWrite(statusLED,HIGH); //to show reader is in setup mode
    if(measure_frequency){
      measureFreq();
      measure_frequency = 0;
    }
  }
  if(sendmode == 0){ //if in "read RFID mode"
    digitalWrite(statusLED,LOW);
  }

  //wait until ISR reports a complete tag
  if(tagfetched == 1){
    
    tagfetched = 0; //reset tagfetched flag
    tagfetched_time0 = millis();

    digitalWrite(readLED,HIGH); //light up LED on tag detection

    //reverse bits in bytes 11.5uS max
    for(uint8_t i = 0;i < 6;i++){
      if((tagbytes[i] & 1) != ((tagbytes[i] >> 7) & 1))        {tagbytes[i] ^=  0b10000001;}
      if(((tagbytes[i] >> 1) & 1) != ((tagbytes[i] >> 6) & 1)) {tagbytes[i] ^=  0b01000010;}
      if(((tagbytes[i] >> 2) & 1) != ((tagbytes[i] >> 5) & 1)) {tagbytes[i] ^=  0b00100100;}
      if(((tagbytes[i] >> 3) & 1) != ((tagbytes[i] >> 4) & 1)) {tagbytes[i] ^=  0b00011000;}
    }

    //compare current tag to last tag
    uint8_t sametag = 1;
    for(uint8_t i = 0;i < 12;i++){
      if(tagbytes[i] != last_tagbytes[i]){
        last_tagbytes[i] = tagbytes[i];
        sametag = 0;
      }
    }

//    if(!sametag) //option to do stuff if tag not the same
//    {
//      //use onboard red LED to indicate tag
//      digitalWrite(,HIGH);
//    }
    
    //copy tag we just read into buffer for sending (buffer is emptied once transmitted)
    for(uint8_t i = 0;i < 6;i++){ //first 6 bytes for tag
      buffer[i] = tagbytes[i]; 
    }
    buffer[7] = tagbytes[10]; //byte 10 for temperature (8bit value, 255 if faulty read)

    //reattach interrupt to watch for tag signal
    attachInterrupt(digitalPinToInterrupt(DMOD), tag_watch, CHANGE);
  }
  else{ //if time since last tag was fetched is longer than 46ms, turn off LED (means tag left read range)
    if((millis() - tagfetched_time0) > 46){
      digitalWrite(readLED,LOW);
    }
  }
} //end of loop

//##############################################################################
//#####   F U N C T I O N S   ##################################################
//##############################################################################

//must not take longer than 1ms or else millis function will start to report wrong values
void tag_watch(){ //Analyse the bitstream und check if data is a tag ~31uS max
  //store time when ISR started, and the last time it started 2.18uS
  tsnap1 = tsnap0;
  tsnap0 = micros();    //~1.25uS

  //record the bit-stream from the tag and wait until the tag-header is detected
  //the tag header is: 0000 0000 001
  //----------------------------------------------------------------------------
  if(findstart == 1){
    //----- header detect block ----- 0.96 uS
    //record the bitstream by analysing the pulse duration
    //two short pulses translate to one 0
    if((tsnap0 - tsnap1) < pulseTime){
      if(toc == 1){
        headerDetect += 1;
        toc = 0; //needed to detect two short pulses
      }
      else{
        toc = 1; //first short pulse sets flag to record on second short pulse. Needed to detect two short pulses
      }
    }
    else{
      toc = 0; //reset flag if we see a long pulse

      //if we receive a 1 (long pulse), check if we received 10 zeros before, otherwise reset
      if(headerDetect == 10){
        findstart = 0; //toggle flag, we found the start
        bittic = 0; //count bits to properly write into byte-array
        bytetic = 0; //count bytes
      }
      headerDetect = 0; //count zeros to detect the header (reset)
    }
  }
  else{ //do this if we have found a header -------------------------------------
    //----- bit reading block ----- 1.57uS
    if((tsnap0 - tsnap1) < pulseTime){ //duration uS since last level change
      if(toc == 1){
        if(bittic != 8){
          tagbytes[bytetic] = tagbytes[bytetic] << 1;
        }
        else{ //if the 9th bit is not a 1, control bit has failed, look for header again
          findstart = 1;
        }
        bittic += 1;
        toc = 0;
      }
      else{
        toc = 1;
      }
    }
    else{
      toc = 0;

      if(bittic != 8){
        tagbytes[bytetic] = (tagbytes[bytetic] << 1) | 1;
      }
      bittic += 1;
    }

    if(bittic == 9){
      bittic = 0;
      bytetic += 1;
    }

    //----- CRC check block ----- 26.7uS
    //reached end of transmission/array full -----------------------------------
    if(bytetic == 12){
      findstart = 1; //look for start again

      //perform CRC check on received data
      //this is done at the end (as opposed to parallel) since the control bit will catch a lot of
      //false reads and we can save the time by performing the crc only once (probably)
      uint16_t crc = 0x0; // initialization
      uint16_t polynomial = 0x1021; // polynomial to calculate crc, actually 0x11021 but first bit is always reversed

      for(uint8_t h = 0;h < 10;h++){ //iterate through all 8+2 bytes data+crc
        uint8_t tagbyte = tagbytes[h]; //copy first byte of tag-data

        for(uint8_t i = 0;i < 8;i++){ //iterate through 8 bits
          uint8_t bit = (tagbyte >> (7-i) & 1) == 1; //returns 1/0 for each bit starting @ msb for the current byte
          uint8_t crcmsb = ((crc >> 15) & 1) == 1;   //returns 1/0 for msb of the crc
          crc = crc << 1; //shift whole crc one to the left
          if(crcmsb ^ bit){ //flip bits in crc, where polynomial == 1 (done by xor whole polynomial)
            crc = crc ^ polynomial;
          }
        }
      }
      
      //while we're here check temperature parity bit as well
      uint8_t temp_parity = tagbytes[10]; //minimum value reported is 5, at lower temperature jumps to 0
      uint8_t parity_bit = tagbytes[11]; //parity_check is 1 if even number of "1" in temp. 0 if odd no. of "1"
      //calculate parity
      temp_parity ^= temp_parity >> 4;
      temp_parity ^= temp_parity >> 2;
      temp_parity ^= temp_parity >> 1;
      temp_parity = temp_parity & 1;    //0 = even, 1 = odd
      if(temp_parity == parity_bit){  //if parity bit indicates faulty temp reading, set temp to 255, otherwise carry on
        tagbytes[10] = 255;
      }
      
      if(crc == 0){ //if crc of tag is ok continue, temp isn't important enough to discard read
        tagfetched = 1; //multiple tags could be buffered or interrupt disabled until tag is processed
        detachInterrupt(digitalPinToInterrupt(DMOD)); //if crc checks out, disable interrupt to allow for communication
      }
    }
  }
}

//counter to measure resonant frequency for ISR
void freqCounter(){
  freq++; //count number of rising edges on CLK
}

//measure resonant frequency (tag reading will be disabled during measurement)
void measureFreq(){
  freqgrab = 0;
  digitalWriteFast(SHD,LOW); //enable antenna
  detachInterrupt(digitalPinToInterrupt(DMOD));
  delay(100); //allow antenna to power up
  attachInterrupt(digitalPinToInterrupt(CLK),freqCounter,RISING);

  uint32_t stoptime = millis() + 1000; //one second
  freq = 0;
  while(millis() < stoptime); //wait one second
  freqgrab = freq;
  detachInterrupt(digitalPinToInterrupt(CLK));
  attachInterrupt(digitalPinToInterrupt(DMOD),tag_watch, CHANGE);
  digitalWriteFast(SHD,HIGH); //disable antenna
}

// I2C functions
//send data on request
void sendData(){ //~12.6-22.6-uS
  if(sendmode == 0){
    //buffer contains last read tag id, or all 0 if no new tag since last send
    Wire.write(buffer,7);

    //clear buffer after send (sends last tag that was read, no matter how long ago)
    for(uint8_t i = 0;i < 7;i++){
      buffer[i] = 0;
    }
  }
  if(sendmode == 1){  //send frequency measurement
    uint8_t sendbuffer[4]; //array containing frequency
    sendbuffer[0] = (freqgrab >> 0  & 0xff);
    sendbuffer[1] = (freqgrab >> 8  & 0xff);
    sendbuffer[2] = (freqgrab >> 16 & 0xff);
    sendbuffer[3] = (freqgrab >> 24 & 0xff);

    Wire.write(sendbuffer,4); //send resonant frequency (or other info)
  }
}

//receive instructions (toggle antenna on/off)
void receiveEvent(int bytes_incoming){
  volatile uint8_t c = Wire.read();

  if(c == 0) digitalWriteFast(SHD,HIGH); //high is antenna off
  if(c == 1) digitalWriteFast(SHD,LOW);  //low is antenna on
  if(c == 2) sendmode = 0; //RFID mode (tag transmit)
  if(c == 3){
    sendmode = 1; //measure mode (frequency transmit)
    measure_frequency = 1;
  }
}



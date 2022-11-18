/*------------------------------------------------------------------------------
- PJRC Teensy LC pin mapping - Hardware Revision v7.0

D5  - RDY/CLK ready/clock
D6  - DMOD demodulation out
D7  - SHD shutdown

D22 - Status LED
D23 - Read LED

//Comms
D18 - SDA
D19 - SCL

*///----------------------------------------------------------------------------
#include <Wire.h>

//LEDs
const uint8_t statusLED = 22;
const uint8_t readLED = 23;
uint8_t ledset = 0;

//used for interfacing EM4095
const uint8_t DMOD = 6;         //dmod pin
const uint8_t SHD = 7;         //shutdown pin adjust port as well!
const uint8_t CLK = 5;          //RDY/CLK Pin
const uint8_t pulseTime = 181;  //181 uS , 8688 Systick ticks

volatile uint8_t headerDetect;  //count zeros (and one 1) to detect header
volatile uint8_t findstart = 1;  //flag to toggle looking for header

volatile uint8_t bittic = 0;    //count bits from tag
volatile uint8_t bytetic = 0;   //count bytes from tag

//timing of the ISR to detect long/short pulses of the EM4095
volatile uint32_t tsnap0 = 0;
volatile uint32_t tsnap1 = 0;

volatile uint8_t tick = 0;        //to help translate two short pulses
volatile uint8_t tagfetched = 0;  //flag to indicate a complete tag was found

volatile uint32_t freqgrab = 0;    //flag to determine resonant frequency

uint32_t tagfetched_time0 = 0;    //time the tag was fetched
uint32_t tagfetched_time1 = 0;

volatile uint8_t tagbytes[10];    //array containing tag-data and crc check (not data-block)
uint8_t last_tagbytes[10];        //contains the tag from the last time we read it

uint8_t buffer[6];                //array containing a copy of the last tag that was detected, emptied once sent, without datablock and crc



volatile uint8_t statled = 0;

//##############################################################################
//##### SETUP ##################################################################
//##############################################################################
void setup(){
  //I2C Setup
  // Wire.begin(0x08);             //join I2C Bus at address 8 (0-7 is reserved)
  // Wire.onRequest(sendData);     //what to do when being talked to
  // Wire.onReceive(receiveEvent); //what to do with data received
  
  //set pins
  pinMode(DMOD,INPUT);        //demodulation input
  pinMode(CLK,INPUT);         //clock signal input
  pinMode(SHD,OUTPUT);        //shutdown
  digitalWrite(SHD,HIGH);
  pinMode(statusLED,OUTPUT);  //status LED
  pinMode(readLED,OUTPUT);    //read LED
  pinMode(2,OUTPUT);          //for timing
  //only used for development
  //while (!Serial); //wait until serial connection is enabled

  //initialize variable for detecting the tag header
  headerDetect = 0;

  //measure resonant frequency
  Serial.println("Checking resonant frequency...");
  digitalWriteFast(SHD,LOW); //enable antenna
  delay(10); //allow antenna to power up
  attachInterrupt(digitalPinToInterrupt(CLK), freq_measure,RISING);
  delay(1000); //measure for 1 second
  detachInterrupt(digitalPinToInterrupt(CLK));
  Serial.print("Frequency: ");
  Serial.print(freqgrab);
  Serial.println(" Hz");
  delay(100);
  //digitalWriteFast(SHD,HIGH); //disable antenna

  //to start ISR, last entry of setup
  attachInterrupt(digitalPinToInterrupt(DMOD), tag_watch, CHANGE);
} 

//##############################################################################
//##### LOOP ###################################################################
//##############################################################################
void loop(){
  //shutdown: 30us until amplitude <= 1%
  //startup: 1700us until amplitude >= 99%

  //--- manual antenna mode (for tuning)
  // delay(1000);
  // digitalWriteFast(SHD,LOW);
  // digitalWrite(statusLED,HIGH);
  // delay(1000);
  // digitalWriteFast(SHD,HIGH);
  // digitalWrite(statusLED,LOW);

  //digitalWriteFast(2,LOW);
  if(statled == 1){
    digitalWrite(statusLED,HIGH);
  }

  //wait until ISR reports a complete tag
  if(tagfetched == 1){
    //digitalWriteFast(2,HIGH); //pin on for timing
    tagfetched = 0; //reset tagfetched flag
    tagfetched_time0 = millis();

    //light up LED on tag detection
    digitalWrite(readLED,HIGH);
    ledset = 1;

    //reverse bits in bytes 8.3uS
    for(int i = 0;i < 6;i++){
      if((tagbytes[i] & 1) != ((tagbytes[i] >> 7) & 1))        {tagbytes[i] ^=  0b10000001;}
      if(((tagbytes[i] >> 1) & 1) != ((tagbytes[i] >> 6) & 1)) {tagbytes[i] ^=  0b01000010;}
      if(((tagbytes[i] >> 2) & 1) != ((tagbytes[i] >> 5) & 1)) {tagbytes[i] ^=  0b00100100;}
      if(((tagbytes[i] >> 3) & 1) != ((tagbytes[i] >> 4) & 1)) {tagbytes[i] ^=  0b00011000;}
    }

    //compare current tag to last tag ~7.36uS
    uint8_t sametag = 1;
    for(int i=0;i<10;i++){ //check only first 6 bytes?
      if(tagbytes[i] != last_tagbytes[i]){
        last_tagbytes[i] = tagbytes[i];
        sametag = 0;
      }
    }

//    if(sametag != 1) //option to do stuff if tag not the same
//    {
//      //use onboard red LED to indicate tag
//      digitalWrite(,HIGH);
//      ledset = 1;
//    }

    for(int i = 0;i < 6;i++){
      buffer[i] = tagbytes[i]; //copy tag we just read into buffer (buffer is emptied once transmitted)
    }

    //reattach interrupt to watch for tag signal
    attachInterrupt(digitalPinToInterrupt(DMOD), tag_watch, CHANGE);
  }
  else{
    //if time since last tag was fetched is longer than 46ms, turn off LED (means tag left read range)
    if((millis() - tagfetched_time0) > 46){
      if(ledset == 1){ //turn off LED
        digitalWrite(readLED,LOW);
        ledset = 0;
      }
    }
  }
} //end of loop

//##############################################################################

//must not take longer than 1ms or else millis function will start to report wrong values
void tag_watch(){
  //**** speed test via port on/off toggle
  digitalWriteFast(2,HIGH); //pin on ~0.25 uS pulse duration
  statled = 1;
  //store time when ISR started, and the last time it started 2.18uS
  tsnap1 = tsnap0;
  tsnap0 = micros();

  //record the bit-stream from the tag and wait until the tag-header is detected
  //the tag header is: 0000 0000 001
  //----------------------------------------------------------------------------
  if(findstart == 1){
    //----- header detect block ----- 0.96 uS
    //record the bitstream by analysing the pulse duration
    //two short pulses translate to one 0
    if((tsnap0 - tsnap1) < pulseTime){
      if(tick == 1){
        headerDetect += 1;
        tick = 0; //needed to detect two short pulses
      }
      else{
        tick = 1; //first short pulse sets flag to record on second short pulse. Needed to detect two short pulses
      }
    }
    else{
      tick = 0; //reset flag if we see a long pulse

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
    if((tsnap0 - tsnap1) < pulseTime){ //duration us since last level change
      if(tick == 1){
        if(bittic != 8){
          tagbytes[bytetic] = tagbytes[bytetic] << 1;
        }
        else{
          //if the 9th bit is not a 1, control bit has failed, look for header again
          findstart = 1;
        }

        bittic += 1;
        tick = 0;
      }
      else{
        tick = 1;
      }
    }
    else{
      tick = 0;

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
    if(bytetic == 10){
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

      if(crc == 0){ //if crc is ok
        tagfetched = 1; //multiple tags could be buffered or interrupt disabled until tag is processed
        detachInterrupt(digitalPinToInterrupt(DMOD)); //if crc checks out, disable interrupt to allow for communication
      }
    }
  }
  digitalWriteFast(2,LOW); //pin off for timing
}

//measure resonant frequency
void freq_measure(){
  freqgrab++; //count number of rising edges on CLK
}

// I2C functions
//send data on request
void sendData(){ //~12.6-22.6-uS
  //buffer contains last read tag id, or all 0 if no new tag since last send
  Wire.write(buffer,6);

  //clear buffer after send (sends last tag is read, no matter how long ago)
  for(int i = 0;i < 6;i++){
    buffer[i] = 0;
  }
}

//receive instructions
void receiveEvent(int bytes_incoming){
  byte c = Wire.read();
  if(c == 1){
    digitalWriteFast(SHD,LOW);
  }
  else{
    digitalWriteFast(SHD,HIGH);
  }
}

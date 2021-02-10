/*------------------------------------------------------------------------------
--- Adafruit ItsyBitsy M0 pin mapping - Hardware Revision v6.5 ---

 A0-
 A1-
 A2-
 A3-
 A4-
 A5-

 D2-
 D3-
 D4-
 D5- ready/clock CLK
 D7-
 D9- demodulation DMOD
D10- shutdown SHD
D11-
D12-
D13-

*///----------------------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_DotStar.h>

//used for interfacing EM4095
const uint8_t DMOD = 9;         //dmod pin
const uint8_t SHD = 10;         //shutdown pin adjust port as well!
const uint8_t CLK = 7;          //RDY/CLK Pin
const uint8_t pulseTime = 181;  //181 uS , 8688 Systick ticks

volatile uint8_t headerDetect;  //count zeros (and one 1) to detect header
volatile uint8_t findstart = 1; //flag to toggle looking for header

volatile uint8_t bittic = 0;    //count bits from tag
volatile uint8_t bytetic = 0;   //count bytes from tag

//timing of the ISR to detect long/short pulses of the EM4095
volatile uint32_t tsnap0 = 0;
volatile uint32_t tsnap1 = 0;

volatile uint8_t tick = 0;        //to help translate two short pulses
volatile uint8_t tagfetched = 0;  //flag to indicate a complete tag was found

uint32_t tagfetched_time0 = 0;    //time the tag was fetched
uint32_t tagfetched_time1 = 0;

volatile uint8_t tagbytes[10];    //array containing tag-data and crc check (not data-block)
uint8_t last_tagbytes[10];        //contains the tag from the last time we read it

uint8_t buffer[6];                //array containing a copy of the last tag that was detected, emptied once sent, without datablock and crc

uint8_t ledset = 0;                             //flag to toggle internal LED
Adafruit_DotStar strip(1, 41, 40, DOTSTAR_BRG); //create dotstar object

//##############################################################################
//##### SETUP ##################################################################
//##############################################################################
void setup()
{
  //Set up RGB LED on board, and turn it off
  strip.begin(); //Initialize pins for output
  strip.show();  //Turn all LEDs off ASAP

  //I2C Setup
  Wire.begin(0x08); //join I2C Bus at address 8 (0-7 is reserved)
  Wire.onRequest(sendData); //what to do when being talked to
  Wire.onReceive(receiveEvent); //what to do with data received
  
  //Sets the priority of the systick interrupt to 0, in order to get correct micros() readings, as they are otherwise faulty
  //due to a change in SYsTick priority for implementing I2S
  NVIC_SetPriority (SysTick_IRQn, 0);

  //set pins
  pinMode(DMOD,INPUT);
  pinMode(SHD,OUTPUT); //for Shutdown
  pinMode(13,OUTPUT); //for LED

  //only used for development
  //Serial.begin(115200);
  //while (!Serial); //wait until serial connection is enabled

  //initialize variable for detecting the tag header
  headerDetect = 0;

  //to start ISR, last entry of setup
  attachInterrupt(digitalPinToInterrupt(DMOD), tag_watch, CHANGE);
}

//##############################################################################
//##### LOOP ###################################################################
//##############################################################################
void loop()
{
  //shutdown: 30us until amplitude <= 1%
  //startup: 1700us until amplitude >= 99%
  
  //wait until ISR reports a complete tag
  if(tagfetched == 1)
  {
    //REG_PORT_OUTSET0 = PORT_PA07; //pin on ffor timing
    tagfetched = 0; //reset tagfetched flag
    tagfetched_time0 = millis();

    //light up LED on tag detection
    digitalWrite(13,HIGH);
    ledset = 1;

    //reverse bits in bytes 8.3uS
    for(int i = 0;i < 6;i++)
    {
      if((tagbytes[i] & 1) != ((tagbytes[i] >> 7) & 1))
      {
        tagbytes[i] ^=  0b10000001;
      }
      if(((tagbytes[i] >> 1) & 1) != ((tagbytes[i] >> 6) & 1))
      {
        tagbytes[i] ^=  0b01000010;
      }
      if(((tagbytes[i] >> 2) & 1) != ((tagbytes[i] >> 5) & 1))
      {
        tagbytes[i] ^=  0b00100100;
      }
      if(((tagbytes[i] >> 3) & 1) != ((tagbytes[i] >> 4) & 1))
      {
        tagbytes[i] ^=  0b00011000;
      }
    }

    //compare current tag to last tag ~7.36uS
    uint8_t sametag = 1;
    for(int i=0;i<10;i++) //check only first 6 bytes?
    {
      if(tagbytes[i] != last_tagbytes[i])
      {
        last_tagbytes[i] = tagbytes[i];
        sametag = 0;
      }
    }

//    if(sametag != 1) //option to do stuff if tag not the same
//    {
//      //use onboard red LED to indicate tag (dotstar causes noise)
//      digitalWrite(13,HIGH);
//      ledset = 1;
//    }

    for(int i = 0;i < 6;i++)
    {
      buffer[i] = tagbytes[i]; //copy tag we just read into buffer (buffer is emptied once transmitted)
    }

    //reattach interrupt to watch for tag signal
    attachInterrupt(digitalPinToInterrupt(DMOD), tag_watch, CHANGE);
  }
  else
  {
    //if time since last tag was fetched is longer than 46ms, turn off LED (means tag left read range)
    if((millis() - tagfetched_time0) > 46)
    {
      //turn off LED
      if(ledset == 1)
      {
        digitalWrite(13,LOW);
        ledset = 0;
      }
    }
  }

}

//##############################################################################

//must not take longer than 1ms or else millis function will start to report wrong values
void tag_watch()
{
  //**** speed test via port on/off toggle
  //REG_PORT_OUTSET0 = PORT_PA07; //pin on ~0.25 uS pulse duration

  //store time when ISR started, and the last time it started 2.18uS
  tsnap1 = tsnap0;
  tsnap0 = micros();

  //record the bit-stream from the tag and wait until the tag-header is detected
  //the tag header is: 0000 0000 001
  //----------------------------------------------------------------------------
  if(findstart == 1)
  {
    //----- header detect block ----- 0.96 uS
    //record the bitstream by analysing the pulse duration
    //two short pulses translate to one 0
    if((tsnap0 - tsnap1) < pulseTime)
    {
      if(tick == 1)
      {
        headerDetect += 1;
        tick = 0; //needed to detect two short pulses
      }
      else
      {
        tick = 1; //first short pulse sets flag to record on second short pulse. Needed to detect two short pulses
      }
    }
    else
    {
      tick = 0; //reset flag if we see a long pulse

      //if we receive a 1 (long pulse), check if we received 10 zeros before, otherwise reset
      if(headerDetect == 10)
      {
        findstart = 0; //toggle flag, we found the start
        bittic = 0; //count bits to properly write into byte-array
        bytetic = 0; //count bytes
      }
      headerDetect = 0; //count zeros to detect the header (reset)
    }
  }
  else //do this if we have found a header -------------------------------------
  {
    //----- bit reading block ----- 1.57uS
    if((tsnap0 - tsnap1) < pulseTime) //duration us since last level change
    {
      if(tick == 1)
      {
        if(bittic != 8)
        {
          tagbytes[bytetic] = tagbytes[bytetic] << 1;
        }
        else
        {
          //if the 9th bit is not a 1, control bit has failed, look for header again
          findstart = 1;
        }

        bittic += 1;
        tick = 0;
      }
      else
      {
        tick = 1;
      }
    }
    else
    {
      tick = 0;

      if(bittic != 8)
      {
        tagbytes[bytetic] = (tagbytes[bytetic] << 1) | 1;
      }
      bittic += 1;
    }

    if(bittic == 9)
    {
      bittic = 0;
      bytetic += 1;
    }

    //----- CRC check block ----- 26.7uS
    //reached end of transmission/array full -----------------------------------
    if(bytetic == 10)
    {
      findstart = 1; //look for start again

      //perform CRC check on received data
      //this is done at the end (as opposed to parallel) since the control bit will catch a lot of
      //false reads and we can save the time by performing the crc only once (probably)
      uint16_t crc = 0x0; // initialization
      uint16_t polynomial = 0x1021; // polynomial to calculate crc, actually 0x11021 but first bit is always reversed

      for(uint8_t h = 0;h < 10;h++) //iterate through all 8+2 bytes data+crc
      {
        uint8_t tagbyte = tagbytes[h]; //copy first byte of tag-data

        for(uint8_t i = 0;i < 8;i++) //iterate through 8 bits
        {
          uint8_t bit = (tagbyte >> (7-i) & 1) == 1; //returns 1/0 for each bit starting @ msb for the current byte
          uint8_t crcmsb = ((crc >> 15) & 1) == 1;   //returns 1/0 for msb of the crc
          crc = crc << 1; //shift whole crc one to the left
          if(crcmsb ^ bit) //flip bits in crc, where polynomial == 1 (done by xor whole polynomial)
          {
            crc = crc ^ polynomial;
          }
        }
      }

      if(crc == 0) //if crc is ok
      {
        tagfetched = 1; //multiple tags could be buffered or interrupt disabled until tag is processed
        detachInterrupt(digitalPinToInterrupt(DMOD)); //if crc checks out, disable interrupt to allow for communication
      }
    }
  }

}

// I2C functions
//send data on request
void sendData() //~12.6-22.6-uS
{
  //buffer contains last read tag id, or all 0 if no new tag since last send
  Wire.write(buffer,6);

  //clear buffer after send (sends last tag it read, no matter how long ago)
  for(int i = 0;i < 6;i++)
  {
    buffer[i] = 0;
  }
}

//receive instructions
void receiveEvent(int bytes_incoming)
{
  byte c = Wire.read();
  if(c == 1)
  {
    //attachInterrupt(digitalPinToInterrupt(DMOD), tag_watch, CHANGE); //attach interrupt to watch for tag
    REG_PORT_OUTCLR0 = PORT_PA18; //pin off, Antenna on
  }
  else
  {
    REG_PORT_OUTSET0 = PORT_PA18; //pin on, Antenna off
    //detachInterrupt(digitalPinToInterrupt(DMOD)); //disable interrupt if antenna is off
  }
}

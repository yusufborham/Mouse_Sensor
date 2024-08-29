#define PROD_ID1 0x00
#define PROD_ID2 0x01
#define MOTION_STATUS 0X02
#define DEL_X 0x03
#define DEL_Y 0x04
#define OP_MODE 0x05
#define CONF 0x06
#define IMG_QUALITY 0x07
#define OP_STATE 0x08
#define W_PROTECT 0x09
#define SLEEP1_SET 0x0A
#define ENTER_TIME 0x0B
#define SLEEP2_SET 0x0C
#define IMG_THRESHOLD 0x0D
#define IMG_RECOG 0x0E

/// MOTION_STATUS register ///
#define DX0VF 0X08 
#define DYOVF 0X10
#define MOTION_STATUS_FLAG_BIT 0X80
//////////////////////////////

///OP_MODE register///
#define LED_CTR 0X80 
#define SLP_EN 0x10
#define SLP2_EN 0x08
#define SLP2MU 0x04
#define SLP1MU 0x02  
#define WAKEUP 0x01 
//////////////////////

byte clkPin , dioPin ;

int dataXY [2] = {0,0} ;
int* ptrData = dataXY ;

enum resolution {
        RES_400 = 0b000,
        RES_500 = 0b001,
        RES_600 = 0b010,
        RES_800 = 0b011,
        RES_1000 = 0b100,
        RES_1200 = 0b101,
        RES_1600 = 0b110
    };

enum sleepMode {
        NO_SLEEP = 0b00000 ,
        SLEEP1   = 0b10000 ,
        SLEEP2   = 0b11000 
    };

///////////////////////////////////////////////////////////////////////////////

void mouseInit(byte clk , byte dio) {
  clkPin = clk ;
  dio = dioPin ;
  pinMode(clkPin , OUTPUT); 

  //Re-Synchronous Serial Interface
  digitalWrite (clkPin , HIGH);
  digitalWrite (clkPin , LOW);
  delayMicroseconds(1);            //tRESYNC
  digitalWrite(clkPin , HIGH);
  delay(320) ;                     //tSWITT
  digitalWrite(clkPin , LOW) ;
  mouseWrite(OP_MODE,0xA0);       //disable sleep mode 
  mouseWrite(CONF,0x07);          //set resolution to max 1600       
}   

/////////////////////////////////////////////////////////////////////////////////

// to write a value to any register 
void mouseWrite(byte address , byte value ) {
  pinMode(dioPin , OUTPUT) ;
  byte address_to_send = address | 0x80 ;       // setting MSB to 1 to enable write 

  // sending the address MSB first
  for (byte i = 7 ; i >= 0 ; i--){
     // mouse sensor reads dio on rising edge 
     digitalWrite(clkPin , LOW);
     digitalWrite(dioPin, ( (address_to_send >> i) & 0x01) ? HIGH : LOW ) ;
     digitalWrite(clkPin , HIGH);
  }

  // sending the data on dio 
  for (byte i = 7 ; i >= 0 ; i--){
     // mouse sensor reads dio on rising edge 
     digitalWrite(clkPin , LOW);
     digitalWrite(dioPin, ( (value >> i) & 0x01) ? HIGH : LOW ) ;
     digitalWrite(clkPin , HIGH);
  }

  delayMicroseconds(50) ;
  
}

////////////////////////////////////////////////////////////////////////////////////

byte mouseRead(byte address) {
  // sending the address MSB first
  byte result = 0 ;
  
  for (byte i = 7 ; i >= 0 ; i--){
     // mouse sensor reads dio on rising edge 
     digitalWrite(clkPin , LOW);
     digitalWrite(dioPin, ( (address >> i) & 0x01) ? HIGH : LOW ) ;
     digitalWrite(clkPin , HIGH);
  }

  pinMode(dioPin ,INPUT);     // high impendence state 
  delayMicroseconds(3) ;      // tHOLD 

  for (byte i = 7 ; i >= 0 ; i--){
     // mouse sensor changes dio on clk rising edge 
     digitalWrite(clkPin , LOW);
     digitalWrite(clkPin , HIGH);
     result |= ((digitalRead(dioPin) ? 1 : 0) << i) ;
  }

  delayMicroseconds(3);      // tHOLD 

  return result ; 

}
/////////////////////////////////////////////////////////////////////////////////

void getData(){
  // must first read first bit in motion status register 
  int dx =0 ,dy = 0;

  byte MOTION_STATUS_FLAG = ( (mouseRead(MOTION_STATUS) & MOTION_STATUS_FLAG_BIT ) ? true : false ) ;

  if (MOTION_STATUS_FLAG) {
    
    byte DXOVF_FLAG = ( (mouseRead(MOTION_STATUS) & DX0VF ) ? true : false ) ;    
    byte DYOVF_FLAG = ( (mouseRead(MOTION_STATUS) & DYOVF ) ? true : false ) ;

    // if data is overflowed add 255 to the data first 
    
    if (DXOVF_FLAG) dx = 255 ;       
    if (DYOVF_FLAG) dy = 255 ;

    // read data from mouse 

    dx += mouseRead(DEL_X) ;
    dy += mouseRead(DEL_Y) ;

    // write the 2 values in the array dataXY 
    *(ptrData++) = dx ; 
    *(ptrData--) = dy ;
    
  }
    
///////////////////////////////////////////////////////////////////////////////////////

// change the resolution 
void setRes(resolution RES_VALUE)

  byte CONFIGURE_REG = mouseRead(CONF) ;
  CONFIGURE_REG |= RES_VALUE ;
  mouseWrite(CONF,CONFIGURE_REG) ;
  
}
//////////////////////////////////////////////////////////////////////////////////////

// to enable sleep mode

void enableSleepMode(sleepMode SLEEP_MODE_TYPE){

  byte OP_MODE_REG = mouseRead(OP_MODE) ;
  OP_MODE_REG = OP_MODE_REG | 0x80 | SLEEP_MODE_TYPE ;    // enable LED shutter and also enable the sleep mode 
  mouseWrite(OP_MODE , OP_MODE_REG) ;
}
/////////////////////////////////////////////////////////////////////////////////////


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600) ;
  mouseInit(3,4);   // clock pin = 3, data pin = 4

  byte PID = mouseRead(PROD_ID1) ;              // product id
  Serial.print("activated and the PID is ") ;
  Serial.println(PID);

}

void loop() {
  // put your main code here, to run repeatedly:
  getData() ;
  Serial.print(dataXY[0]) ;
  Serial.print("   " ) ;
  Serial.println(dataXY[1]);

}
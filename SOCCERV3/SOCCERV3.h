/*
 * SOCCER3.h
 *
 * Created: 2015-08-06 오후 1:59:58
 * Author: JEON HAKYEONG (전하경)
 *
 * History
 *
 * 2015-10-28
 *	Debugging & Optimizing Header File.
 *	Change Timer Interrupt 0 Period 20us -> 40us
 *
 * 2015-10-31
 *	Debugging & Optimizing Header File.
 *	Change Timer Interrupt 0 Period 40us -> 20us
 *  Remove All Arithmetic commands in Timer Interrupt Routine
 *
 * 2015-11-01
 *	PWM OFF/BREAK mode Change (Forward & Backward : Break Mode)
 *
 *
 * 2016-02-21
 *	Change Compass Sensor Measurement mode (continuous -> single MODE)
 *
 * 2016-06-07
 *	Change L_IR2 and S_IR2 (change position S_IR2 to front)
 *
 */ 

//  For CAMERA
#include <Pixy.h>
volatile Pixy pixy;

//Compass Sensor Address
#define HMC5883L 0x1E
#define QMC5883L 0x0D
unsigned char reg = 0x03; //HMC5883L : 0x03 , QMC5883L : 0x00
unsigned char addr = HMC5883L;
//


void Scan_ADC(void);


#define TRIGER1ON (PORTJ |= 0x80)
#define TRIGER1OFF (PORTJ &= 0x7F)
#define TRIGER2ON (PORTJ |= 0x40)
#define TRIGER2OFF (PORTJ &= 0xBF)
#define TRIGER3ON (PORTJ |= 0x20)
#define TRIGER3OFF (PORTJ &= 0xDF)
#define TRIGER4ON (PORTJ |= 0x10)
#define TRIGER4OFF (PORTJ &= 0xEF)

#define ECHO1 (PINJ & 0x08)
#define ECHO2 (PINJ & 0x04) 
#define ECHO3 (PINJ & 0x02)
#define ECHO4 (PINJ & 0x01)


volatile unsigned int TX1LED = 18;
volatile unsigned int RX1LED = 19;
volatile unsigned int TX2LED = 16;
volatile unsigned int RX2LED = 17;
volatile bool led = HIGH;


#include "lcd.h"
#include "MOTOR.h"

#include <Wire.h>

volatile unsigned int Pulse_Width_Count=0;
volatile unsigned int ECHO_CNT[4];
volatile unsigned char ECHO_REPLY[4]={0,0,0,0};
volatile unsigned char US_SEQ = 0;

volatile unsigned char ADC_SEQ = 0;
volatile unsigned char ADC_CNT = 0;
volatile unsigned int temp_dir=0;
volatile unsigned int temp_ir=0;

volatile int memComp=180;

volatile int xin, yin;
volatile int xoff, yoff;
volatile float xs, ys;
void echo_clear(void)
{
  for(int i=0; i<4;i++)
  {
    ECHO_CNT[i]=0;
    ECHO_REPLY[i]=0;
  }
}


void US_Check(void)
{
  if (Pulse_Width_Count==0)
  {
    switch(US_SEQ){
      case 0 :  TRIGER1ON;
                delayMicroseconds(10);
                TRIGER1OFF;
                break;
      case 1 :  TRIGER2ON;  
                delayMicroseconds(10);
                TRIGER2OFF;
                break;
      case 2 :  TRIGER3ON;  
                delayMicroseconds(10);
                TRIGER3OFF;
                break;
      case 3 :  TRIGER4ON;  
                delayMicroseconds(10);
                TRIGER4OFF;
                break;  
    }
  }
  else if (Pulse_Width_Count < 300)
    switch(US_SEQ){
      case 0 :
          if (ECHO_CNT[US_SEQ] && !ECHO_REPLY[US_SEQ] && !ECHO1)
          {
            ECHO_CNT[US_SEQ]=Pulse_Width_Count-ECHO_CNT[US_SEQ];
            ECHO_REPLY[US_SEQ]=1;
          }
          if (!ECHO_CNT[US_SEQ] && ECHO1) ECHO_CNT[US_SEQ]=Pulse_Width_Count;
          break;
      case 1 :
          if (ECHO_CNT[US_SEQ] && !ECHO_REPLY[US_SEQ] && !ECHO2)
          {
            ECHO_CNT[US_SEQ]=Pulse_Width_Count-ECHO_CNT[US_SEQ];
            ECHO_REPLY[US_SEQ]=1;
          }
          if (!ECHO_CNT[US_SEQ] && ECHO2) ECHO_CNT[US_SEQ]=Pulse_Width_Count;
          break;

      case 2 :
          if (ECHO_CNT[US_SEQ] && !ECHO_REPLY[US_SEQ] && !ECHO3)
          {
            ECHO_CNT[US_SEQ]=Pulse_Width_Count-ECHO_CNT[US_SEQ];
            ECHO_REPLY[US_SEQ]=1;
          }
          if (!ECHO_CNT[US_SEQ] && ECHO3) ECHO_CNT[US_SEQ]=Pulse_Width_Count;
          break;
           
      case 3 :
          if (ECHO_CNT[US_SEQ] && !ECHO_REPLY[US_SEQ] && !ECHO4)
          {
            ECHO_CNT[US_SEQ]=Pulse_Width_Count-ECHO_CNT[US_SEQ];
            ECHO_REPLY[US_SEQ]=1;
          }
          if (!ECHO_CNT[US_SEQ] && ECHO4) ECHO_CNT[US_SEQ]=Pulse_Width_Count;
          break;
    }
  else if (Pulse_Width_Count == 500)  // Stop Measurement after 16ms (Distance : 272cm)
  {
    if (!ECHO_REPLY[US_SEQ]) ECHO_CNT[US_SEQ]=Pulse_Width_Count;
    ultra[US_SEQ]=ECHO_CNT[US_SEQ];
  }
}

void Scan_Ultra()
{
//  digitalWrite(TX2LED, LOW);          // turn the LED on (voltage level - 0V - LOW)
  US_Check();
  Pulse_Width_Count++;
  if (Pulse_Width_Count > 500) // Wait till 15ms, Restart Ultra-Sonic Measurement
  {
    Pulse_Width_Count=0;

    ECHO_CNT[US_SEQ]=0;
    ECHO_REPLY[US_SEQ]=0;
    
    US_SEQ++;
    if (US_SEQ > 3) US_SEQ = 0;
  }
  ADC_CNT++;
  if (ADC_CNT == 2)   ADCSRA = ADCSRA | 0xc0; //ADC START & ADC Interrupt disable
  
  if(ADCSRA & 0x10)    Scan_ADC();

//   digitalWrite(TX2LED, HIGH);          // turn the LED OFF (voltage level - 5V - HIGH)      
}



void Scan_ADC()
{
  int tempseq = ADC_SEQ;
  int tmpSum =0;
  
//  digitalWrite(TX1LED, LOW);          // turn the LED on (voltage level - 0V - LOW)

  tmpSum = ADC_Raw[NumberOfSamples][tempseq] - ADC_Raw[0][tempseq];
  for(int i=0;i < NumberOfSamples-1 ; i++)
  {
    ADC_Raw[i][tempseq] = ADC_Raw[i+1][tempseq];
  }
  ADC_Raw[NumberOfSamples-1][tempseq]=ADCH;
  
  tmpSum += ADC_Raw[NumberOfSamples-1][tempseq];
  ADC_Raw[NumberOfSamples][tempseq]=tmpSum;
  
  ADC_Value[tempseq] = tmpSum >> 2;
//  ADC_Value[tempseq] = tmpSum >> 1;  

  
  ADC_CNT = 0;
  
  if (ADC_SEQ<12)
  {
    if(ADC_Value[tempseq] > temp_ir)
    {
      temp_ir=ADC_Value[tempseq];
      temp_dir=tempseq;
    }
  }
  else if(ADC_SEQ ==15)    ADMUX  = 0x65; //select adc CH 5
  else if(ADC_SEQ ==16)
  {
    ADMUX  = 0x66;                        //select adc CH 6
    PORTF &= 0x80;
  }
  else if(ADC_SEQ ==17)    ADMUX  = 0x64; //select adc CH 4

  ADC_SEQ++;

  if(ADC_SEQ>17)
  {
    unsigned char tmpLine =0;
    
    ADC_SEQ=0;

    if (temp_ir>1)
    {
      max_ir = temp_ir;
      ball_dir = temp_dir; 
    }
    else ball_dir = 15;
    
    if(ADC_Value[12] > Line0_White) tmpLine |= 0x01;
    if(ADC_Value[13] > Line1_White) tmpLine |= 0x02;
    if(ADC_Value[14] > Line2_White) tmpLine |= 0x04;
    if(ADC_Value[15] > Line3_White) tmpLine |= 0x08;

    tmpLine |= LineDetected;
    if (tmpLine != 0x0F)  LineDetected = tmpLine;
    
    tmpLine = LineDetected;
    tmpLine = ~(tmpLine<<4) & 0xF0;

//    if(LineDetected && !escape)  move(0,0,0);
    
    PORTL = tmpLine  | (ball_dir & 0x0F);

    temp_ir=0;
    temp_dir=0;
  }
  if (ADC_SEQ <16)  PORTF =(PORTF&0x80) | (ADC_SEQ & 0x0F);

  ADCSRA = ADCSRA & 0x3f; //ADC disable

//  digitalWrite(TX1LED, HIGH);          // turn the LED off (voltage level - 5V - HIGH)
}

void port_init(void)
{
  DDRA=0xFF;
  PORTA=0x00;

  DDRB=0x8F;
  PORTB=0xFE;

  DDRC=0xFF;
  PORTC=0xFF;

  DDRD=0xFF;
  PORTD=0x0F;

  DDRE=0xF0;
  PORTE=0x0F;

  DDRF=0x8F;
  PORTF=0x80;

  DDRG=0x27;
  PORTG=0x00;

  DDRH=0xFF;
  PORTH=0x0F;

  DDRJ=0xF0;
  PORTJ=0x0F;

  DDRK=0x00;
  PORTK=0xFF;

  DDRL=0xFF;
  PORTL=0xF0;
  
}

void adc_init(void)
{
  ADC_SEQ=0;
  PORTF=0x80;

  ADCSRA = 0x00;          //disable adc
  ADMUX  = 0x64;          //select adc CH 4
  ACSR   = 0x80;          //Analog Comparator Disable
  ADCSRA = 0x86;          //ADC CLOCK Pre-Scaler set to 6 (divide factor is 64) 
  ADCSRA = ADCSRA & 0x3f; //ADC disable
  ADCSRA = ADCSRA | 0xc0; //ADC START & ADC Interrupt disable
}

void read_compass()
{
  int x, y, z; //triple axis data

//  digitalWrite(RX1LED, LOW);          // turn the LED on (voltage level - 0V - LOW)

  Wire.beginTransmission(addr);
  Wire.write(reg); //select register 3, X MSB register
  Wire.endTransmission();
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(addr, 6);
  if(6<=Wire.available())
  {
    if (addr == HMC5883L)
    {
      x = Wire.read()<<8;   //X msb
      x |= Wire.read();     //X lsb
      z = Wire.read()<<8;      //Z msb
      z = Wire.read();      //Z lsb
      y = Wire.read()<<8;   //Y msb
      y |= Wire.read();     //Y lsb
    }
    else {
      x = Wire.read();      //X lsb
      x |= Wire.read()<<8;  //X msb
      y = Wire.read();      //y lsb
      y |= Wire.read()<<8;  //y msb
      z = Wire.read();      //z lsb
      z = Wire.read()<<8;      //z msb
    }
    xin = x;
    yin = y;
  }
 
  x=(int)((float)(x+xoff)*xs);
  y=(int)((float)(y+yoff)*ys);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x & y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  
  float heading = atan2(y, x);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.

  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  absCompass = heading * 180/M_PI; 

  compass = absCompass - memComp;
  compass += 180;
  if(compass<0) compass+=360;
  if(compass>359) compass-=360;

  digitalWrite(RX1LED, LOW);          // turn the LED on (voltage level - 5V - HIGH)

}

boolean Check_Compass()
{
  unsigned char id = 0x00;
  
  Wire.beginTransmission(HMC5883L);
  Wire.write(0x0A); //select register 10, Identification Register A 
  Wire.endTransmission();
 
 //Read Identification Register A 
  Wire.requestFrom(HMC5883L, 1);
  if(1<=Wire.available()){
    id = Wire.read();
  }

  if(id == 'H')
  {
    addr=HMC5883L;
    reg=0x03;

    Wire.beginTransmission(addr); //start talking
    Wire.write(0x02); // Tell the HMC5883 to Continuously Measure
    Wire.write(0x00); // Set the Register
    Wire.endTransmission();

    return true;
  }
  
  Wire.beginTransmission(QMC5883L);
  Wire.write(0x0D); //select register 13, Chip ID
  Wire.endTransmission();
 
 //Read Chip ID
  Wire.requestFrom(QMC5883L, 1);
  if(1<=Wire.available()){
    id = Wire.read();
  }

  if(id == 0xFF)
  {
    addr=QMC5883L;
    reg=0x00;

    Wire.beginTransmission(addr); //start talking
    Wire.write(0x0B); // Tell the QMC5883 to Continuously Measure
    Wire.write(0x01); // Set the Register
    Wire.endTransmission();
    Wire.beginTransmission(addr); //start talking
    Wire.write(0x09); // Tell the QMC5883 to Continuously Measure
    Wire.write(0x1D); // Set the Register 200HZ 
    Wire.endTransmission();

    return true;
  }

  return false;
}


void init_devices(void)
{
	MCUCR = 0x00;
	EIMSK = 0x00;

  PWM_Init();
  echo_clear();
  port_init();
  Lcd_Init();
  adc_init();

  for (int j=0 ;  j < 18 ; j++)
  {
    for (int i=0 ; i < NumberOfSamples+1 ; i++)
      ADC_Raw[i][j]=0;

   ADC_Value[j]=0;
  }

  sei();

}



volatile unsigned int OldDirA =0;
volatile unsigned int OldDirB =0;
volatile unsigned int OldDirC =0;
volatile unsigned int OldDirD =0;

#define madirF (PORTE &= 0xBF)
#define madirB (PORTE |= 0x40)
#define mbdirF (PORTE &= 0x7F)
#define mbdirB (PORTE |= 0x80)
#define mcdirF (PORTH &= 0xBF)
#define mcdirB (PORTH |= 0x40)
#define mddirF (PORTH &= 0x7F)
#define mddirB (PORTH |= 0x80)

#define SHOOTERON (PORTH &= 0xF7)
#define SHOOTEROFF (PORTH |= 0x08)

void Shooting(unsigned int ms)
{
  SHOOTERON;
  delay(ms);
  SHOOTEROFF;
}

void WarningDisplay()
{
  
	OCR1A = 0;
	madirB;
	
	OCR1B = 0;
	mbdirB;
	
	OCR2B = 0;
	mcdirB;
	
	OldDirA=0;
	OldDirB=0;
	OldDirC=0;
	
	Lcd_Init();
	Lcd_Write_String(LINE1,"-WARNING-");
	Lcd_Write_String(LINE2,"  LOW BATTERY");
	while(1)
	Volt_Display(Voltage);

}

int Sign(int value)
{
	if (value==0) return 0;
	else if (value<0) return -1;
	else return 1;
}

void MOTORA(int ma)
{
	int tmp = abs(ma);
	
	if(Voltage < 95)
	{
		WarningDisplay();
	}
	else
	{
		if(Sign(ma) != OldDirA)
		{
			OCR3B=0;
			madirF;
			delay(20);
		}
		
		tmp=tmp*255/100;
		if(tmp>255)	tmp=255;
		
		if(ma<0)	madirB;
		else		madirF;

		OCR3B = tmp;

		OldDirA=Sign(ma);
	}
}

void MOTORB(int mb)
{
	int tmp = abs(mb);
	
	tmp=tmp*255/100;
	if(tmp>255)	tmp=255;
	if(Voltage < 95)
	{
		WarningDisplay();
	}
	else
	{
		if(Sign(mb) != OldDirB)
		{
			OCR3C=0;
			mbdirF;
			delay(20);
		}
		
		if(mb<0)	mbdirB;
		else		mbdirF;

		OCR3C = tmp;

		OldDirB=Sign(mb);
	}
}
void MOTORC(int mc)
{
	int tmp = abs(mc);
	
	tmp=tmp*255/100;
	if(tmp>255)	tmp=255;
	if(Voltage < 95)
	{
		WarningDisplay();
	}
	else
	{
		if(Sign(mc) != OldDirC)
		{
			OCR4B=0;
			mcdirF;
			delay(20);
		}
		
		if(mc<0)	mcdirB;
		else		mcdirF;

		OCR4B = tmp;

		OldDirC=Sign(mc);
	}
}

void MOTORD(int md)
{
  int tmp = abs(md);
  
  tmp=tmp*255/100;
  if(tmp>255) tmp=255;
  if(Voltage < 95)
  {
    WarningDisplay();
  }
  else
  {
    if(Sign(md) != OldDirD)
    {
      OCR4C=0;
      mddirF;
      delay(20);
    }
    
    if(md<0)  mddirB;
    else    mddirF;

    OCR4C = tmp;

    OldDirD=Sign(md);
  }
}

void motor_stop(void)
{
   MOTORA(0);
   MOTORB(0);
   MOTORC(0);
   MOTORD(0);
}

void move(int ma, int mb, int mc)
{
   MOTORA(ma);
   MOTORB(mb);
   MOTORC(mc);
}



void PWM_Init(void)
{

//  TCCR3A = ((1 << COM3B1) | (1 << COM3C1)  | (1 << WGM30));                                 // 8 bit Phase correct PWM Mode , High Active
  TCCR3A = ((1 << COM3B1) | (1 << COM3B0) | (1 << COM3C1)  | (1 << COM3C0)  | (1 << WGM30));  // 8 bit Phase correct PWM Mode , Low Active
  TCCR3B = ((1 << CS31));                                                                     // Clock/8    3,921 HZ PWM GENERATION
    
//  TCCR4A = ((1 << COM4B1) | (1 << COM4C1)  | (1 << WGM40));                                 // 8 bit Phase correct PWM Mode , High Active
  TCCR4A = ((1 << COM4B1) | (1 << COM4B0) | (1 << COM4C1)  | (1 << COM4C0)  | (1 << WGM40));  // 8 bit Phase correct PWM Mode , Low Active
  TCCR4B = ((1 << CS41));                                                                     // Clock/8    3,921 HZ PWM GENERATION
    
}


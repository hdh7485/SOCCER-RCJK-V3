
#define LINE1 0x80
#define LINE2 0xC0

void Lcd_Cmd(char cmd)
{
  PORTD &= 0X7F;
  PORTG &= 0XDF;
  PORTH |= 0X04;

  PORTC=cmd;
  PORTH &= 0XFB;

  delay(1);
}

void Lcd_Clear()
{
	Lcd_Cmd(0x01);
	delay(1);
}

static void Lcd_Data(char ch)
{
	PORTD |= 0X80;
  PORTG &= 0XDF;
  PORTH |= 0X04;

  PORTC=ch;
  PORTH &= 0XFB;

  PORTD &= 0X7F;
  PORTG &= 0XDF;
  delay(1);
}

void Lcd_Init()
{
//	delay(30);

	Lcd_Cmd(0x3C);			//4bit mode set , 2 nibble writing.
  Lcd_Cmd(0x3C);      //4bit mode set , 2 nibble writing.
	Lcd_Cmd(0x0C);	
	Lcd_Clear();
	Lcd_Cmd(0x06);
}

void Lcd_Move(char line, char pos)
{
	pos=(line<<6)+pos;
	pos |= 0x80;
	Lcd_Cmd(pos);
}

void Lcd_String(char *lcd_str)
{
  while(*lcd_str != '\0')
  {
    Lcd_Data(*lcd_str);
    lcd_str++;
  }
}

void Lcd_Write_String(char d_line, char *lcd_str)
{
	Lcd_Cmd(d_line);
  Lcd_String(lcd_str);
}

void Hour_Display(unsigned int x)
{
  Lcd_Data((int)((x%10))+'0');
  Lcd_Data('h');
  Lcd_Data(':');
  Lcd_Data(' ');
}

void DigitDisplay(unsigned int x)
{
	Lcd_Data((int)((x%1000)/100)+'0');
	Lcd_Data((int)((x%100)/10)+'0');
	Lcd_Data((int)((x%10))+'0');
}

void HEX_Display(unsigned char x)
{
	unsigned char d=x;
	
	x=(x>>4)&0x0F;
	if(x>9)	Lcd_Data(x-10+'A');
	else Lcd_Data(x+'0');
	x=d & 0x0F;
	if(x>9)	Lcd_Data(x-10+'A');
	else Lcd_Data(x+'0');
}

void AngleDisplay(unsigned int x)
{
	Lcd_Data((int)((x%10000)/1000)+'0');
	Lcd_Data((int)((x%1000)/100)+'0');
	Lcd_Data((int)((x%100)/10)+'0');
	Lcd_Data('.');
	Lcd_Data((int)((x%10))+'0');
}

void AngleDisplay2(int x)
{
	if(x<0) Lcd_Data('-');
	x=abs(x);
	Lcd_Data((int)((x%10000)/1000)+'0');
	Lcd_Data((int)((x%1000)/100)+'0');
	Lcd_Data((int)((x%100)/10)+'0');
	Lcd_Data('.');
	Lcd_Data((int)((x%10))+'0');
}


void Volt_Display(int volt)
{
	Lcd_Move(0, 11);
	Lcd_Data((int)((volt%1000)/100)+'0');
	Lcd_Data((int)((volt%100)/10)+'0');
	Lcd_Data('.');
	Lcd_Data((int)((volt%10))+'0');
	Lcd_Data('V');
}

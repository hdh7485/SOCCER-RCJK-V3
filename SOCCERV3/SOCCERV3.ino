/*
   SOCCERV3.c

   Created: 2017-010-02
   Author: JEON HAKYEONG (전하경)

   History

   2017-10-02
    Migration From AtmelStudio.

*/

//카메라 유무 설정
//#define CameraExist ture     //카메라가 있음.
#define CameraExist false  //카메라가 없음.

//
// 키 입력 값
//
#define ENTER (~PINB & 0x40)  //ENTER KEY 입력 값
#define PREV (~PINB & 0x20)   //PREV KEY 입력 값
#define NEXT (~PINB & 0x10)   //NEXT KEY 입력 값
//
// 공격 수비 공 판단 기준 값
//
#define SHOOT 200  //공격수 슈팅 기준값, 먼곳에서 슈팅을 시도하면 이값을 키운다.
#define NEAR 50    //공격수 공 근처의 기준값,  공을 끼고 너무 크게 돌면 이값을 키운다.
#define SHOOT2 150 //수비수 슈팅 기준값, 먼곳에서 슈팅을 시도하면 이값을 키운다.
#define NEAR2 50   //수비수 공 감지 기준값, 가반응거리가 짧으면  이값을 줄인다.
//
// 아래 정의값들은 경기장 테두리의흰색 선의 기준값입니다.
// 경기장에서 직접 측정하여 변경하던지 가변저항을 조정하세요.
// 아래 설정값 보다 큰값이 입력되면 흰색 경계선으로 인식
//--------------------------------------------------------------------------
// 센서별로 흰색라인에 올려 놓고 가변저항을 조정하여 200정도에 맞춤.
// 경기장 바닥에서 입력되는 값과 흰색에서 입력되는 값의 중간 또는 3분의 2 정도로 설정
//---------------------------------------------------------------------------
//
unsigned char Line0_White = 30;     // 0시 방향 경계선 감지 기준 값
unsigned char Line1_White = 30;     // 3시 방향 경계선 감지 기준 값
unsigned char Line2_White = 30;     // 6시 방향 경계선 감지 기준 값
unsigned char Line3_White = 30;     // 9시 방향 경계선 감지 기준 값

//
// 위 기준값을 넘은 경우가 있으면 아래 변수에 기록이 되어짐.
// 선을 감지하고 그에따른 액션을 수행한후에는 아래값을 Clear 시켜주세요.
//
// 0 bit :  0h direction line detected
// 1 bit :  3h direction line detected
// 2 bit :  6h direction line detected
// 3 bit :  9h direction line detected

unsigned  char LineDetected = 0;  //경계라인 감지 상태 값
//0 Bit : 0시방향 감지
//1 Bit : 3시방향 감지
//2 Bit : 6시방향 감지
//3 Bit : 9시방향 감지

unsigned char Escape_Dir[16] = { 0  // 경계라인 감지 안됨
                                 , 6 // 0시 방향 감지 - 6시방향으로 탈출
                                 , 9 // 3시 방향 감지 - 9시 방향으로 탈출
                                 , 7 // 0시 3시 방향 감지 - 7시 방향으로 탈출
                                 , 0 // 6시 방향 감지 - 0시 방향으로 탈출
                                 , 3 // 0시 6시 방향 감지 - 3시 방향(또는 9시 방향)으로 탈출
                                 , 10 // 3시, 6시 방향 감지 - 10시 방향으로 탈출
                                 , 9 // 0,3,6시 방향 감지 - 9시 방향으로 탈출
                                 , 3 // 9시 방향 감지 - 3시 방향으로 탈출
                                 , 4 // 0,9시 방향 감지 - 4시 방향으로 탈출
                                 , 6 // 3,9시 방향 감지 - 6시 방향(또는 0시 방향)으로 탈출
                                 , 6 // 0,3,9시 방향 감지 - 6시 방향으로 탈출
                                 , 2 // 6,9시 방향 감지 - 2시 방향으로 탈출
                                 , 3 // 0,6,9 시 방향 감지 - 3시 방향으로 탈출
                                 , 0 // 3,6,9시 방향 감지 - 0시 방향으로 탈출
                                 , 0
                               };// 모든 방향 감지됨 - 존재 할 수 없음 (F/W에서 방지 함.)
//
//Sample Numbers for Average of Analog Value.
//
//#define NumberOfSamples 2   //가능한 변경하지 마세요.. 이값을 변경하고자 하면 Header File쪽도 수정해야 함. 2개
#define NumberOfSamples 4   //가능한 변경하지 마세요.. 이값을 변경하고자 하면 Header File쪽도 수정해야 함. 4개

//평균을 구하기 위한 배열
volatile unsigned int ADC_Raw[NumberOfSamples + 1][18];
//


//************************************************************************************
//------------- 여기부터의 변수는 읽기만 하세요. ------------------------
//************************************************************************************
//
// 아래 변수에 센서 값들이 들어옵니다.
// 읽기만 하시고 절대 쓰지 마세요.
//
volatile unsigned int ADC_Value[18];
// 0 ~ 11 : 12개의 IR Seeker 값
// 12 ~ 15 : 바닥감지 센서 값
// 16 : 전압 측정 값
// 17 : BALL Caputing 판단 기준 적외선 값 (부정확 함, 이값보다는 0시방향 초음파 거리 측정값( ultra[0] )을 사용하는게 좋음).

//
//  초음파 센서 거리 측정 값(0~3번 초음파)
//
volatile unsigned int ultra[4] = {0, 0, 0, 0};
//
// 적외선 공 판단 값
//
volatile unsigned int ball_dir;       //적외선 값이 가장 강하게 보이는 방향.
volatile unsigned int max_ir;         //가장 강한 적외선 값.
//
int last_pos = 0;      //공이 마지막으로 있던 좌우 방향, 왼쪽 = 1, 오른쪽 = 0;
//
//
// 컴파스 센서 값.(절대값이 아닌 상대적 방위 값)
// 처음 파워가 켜진 방향을 상대편 골대 방향으로 인식하며 그값은 180이 된다.
//

volatile int absCompass = 0;
volatile int compass;
volatile  int ultra_gap = 0;
volatile  int comp = 0;

volatile unsigned int BallCapture = 0; // 사용 안함.
#define Voltage ((int)((float)ADC_Value[16]*0.625 -2.5))

//경계선에 걸렸을때 탈출을 위한 변수들.
volatile bool escape = false;
volatile char lastdir = 0;

//************************************************************************************

// 메뉴 디스플레이
volatile char menu = 0;
//----------------------------------------------------------------------------
#define KP 0.5
//----------------------------------------------------------------------------

#include "SOCCERV3.h"
#include <TimerOne.h>
#include <EEPROM.h>

void view_line(void)
{
  Lcd_Clear();
  while (!ENTER)
  {
    Lcd_Cmd(LINE1);
    for (int i = 0; i < 2; i++)
    {
      Lcd_Data(i * 3 + '0');
      Lcd_String("h: ");

      DigitDisplay(ADC_Value[i + 12]);
      Lcd_Data(' ');
    }

    Lcd_Cmd(LINE2);
    for (int i = 2; i < 4; i++)
    {
      Lcd_Data(i * 3 + '0');
      Lcd_String("h: ");
      DigitDisplay(ADC_Value[i + 12]);
      Lcd_Data(' ');
    }
    delay(200);
  }
  while (ENTER) ;

}

void view_ir(void)
{
  int submenu = 0;

  Lcd_Clear();
  while (!ENTER)
  {
    if (PREV)
    {
      while (PREV) ;
      submenu -= 3;
      if (submenu < 0) submenu = 9;
    }
    else if (NEXT)
    {
      while (NEXT) ;
      submenu += 3;
      if (submenu > 9) submenu = 0;
    }

    Lcd_Cmd(LINE1);
    Hour_Display(submenu);

    for (int i = 0; i < 3; i++)
    {
      DigitDisplay(ADC_Value[i + submenu]);
      Lcd_Data(' ');
    }

    Lcd_Cmd(LINE2);
    int tmp = submenu + 3;
    if (tmp > 9) tmp = 0;
    Hour_Display(tmp);

    for (int i = 0; i < 3; i++)
    {
      DigitDisplay(ADC_Value[i + tmp]);
      Lcd_Data(' ');
    }
    delay(100);
  }
  while (ENTER) ;
}

void view_capture(void)
{
  Lcd_Clear();
  Lcd_Write_String(LINE1, "CAPTURE IR");
  while (!ENTER)
  {
    Lcd_Cmd(LINE2);
    DigitDisplay(ADC_Value[17]);
    delay(200);
  }
  while (ENTER) ;
}

void view_ultra(void)
{
  Lcd_Clear();
  while (!ENTER)
  {
    Lcd_Cmd(LINE1);
    for (int i = 0; i < 2; i++)
    {
      Lcd_Data(i * 3 + '0');
      Lcd_String("h: ");
      DigitDisplay((float)ultra[i] * 0.85);
      Lcd_Data(' ');
    }

    Lcd_Cmd(LINE2);
    for (int i = 2; i < 4; i++)
    {
      Lcd_Data(i * 3 + '0');
      Lcd_String("h: ");
      DigitDisplay(ultra[i] * 0.85);
      Lcd_Data(' ');
    }
    delay(200);
  }
  while (ENTER) ;
}

void set_Compass(void)
{
  char dt = (absCompass >> 8) & 0xFF;
  EEPROM.write(0, dt);
  dt = absCompass & 0xFF;
  EEPROM.write(1, dt);
  Lcd_Write_String(LINE2, "SETTING COMPLETE!   ");
  delay(1000);
  memComp = absCompass;

}

void Compass_IRON(void)
{
  Lcd_Clear();

  Lcd_Write_String(LINE1, "CALIBRATE.......");
  Lcd_Write_String(LINE2, "PLEASE WAIT.....");

  read_compass();

  int xmin = xin;
  int xmax = xin;
  int ymin = yin;
  int ymax = yin;

  move(25, 25, 25);

  for ( int i = 0; i < 50; i++)
  {
    delay(100);
    read_compass();

    if (xin < xmin) xmin = xin;
    if (xin > xmax) xmax = xin;
    if (yin < ymin) ymin = yin;
    if (yin > ymax) ymax = yin;
  }

  move(0, 0, 0);

  if ( (xmax - xmin) < 500 || (ymax - ymin) < 500)
  {
    Lcd_Write_String(LINE1, "FAILED!         ");
    Lcd_Write_String(LINE2, "CHECK MOTOR S/W ");
  }
  else
  {
    xs = 1.;
    ys = 1.;

    xs = (float)(ymax - ymin) / (float)(xmax - xmin);
    if (xs < 1.0) xs = 1.0;

    ys = (float)(xmax - xmin) / (float)(ymax - ymin);
    if (ys  < 1.0) ys = 1.0;

    xoff = (xmax - xmin) / 2 - xmax;
    yoff = (ymax - ymin) / 2 - ymax;

    int xscale = (int)(xs * 1000);
    int yscale = (int)(ys * 1000);

    char dt = (xoff >> 8) & 0xFF;
    EEPROM.write(10, dt);
    dt = xoff & 0xFF;
    EEPROM.write(11, dt);

    dt = (xscale >> 8) & 0xFF;
    EEPROM.write(12, dt);
    dt = xscale & 0xFF;
    EEPROM.write(13, dt);

    dt = (yoff >> 8) & 0xFF;
    EEPROM.write(14, dt);
    dt = yoff & 0xFF;
    EEPROM.write(15, dt);

    dt = (yscale >> 8) & 0xFF;
    EEPROM.write(16, dt);
    dt = yscale & 0xFF;
    EEPROM.write(17, dt);

    Lcd_Write_String(LINE2, "SETTING COMPLETE!   ");
  }
  while (!ENTER) ;  //  ENTER 키가 눌릴때까지 대기
  while (ENTER) ;

}

void set_Line(void)
{
  unsigned char minV[4] = {255, 255, 255, 255};
  unsigned char maxV[4] = {0, 0, 0, 0};
  unsigned char diff[4] = {0, 0, 0, 0};

  Lcd_Clear();

  Lcd_Write_String(LINE1, "AFTER MOVING    ");
  Lcd_Write_String(LINE2, "PRESS ENTER!    ");

  while (!ENTER)
  {
    for ( int i = 0 ; i < 4 ; i++)
    {
      if (ADC_Value[i + 12] < minV[i]) minV[i] = ADC_Value[i + 12];
      else if (ADC_Value[i + 12] > maxV[i]) maxV[i] = ADC_Value[i + 12];
      diff[i] = maxV[i] - minV[i];
    }
    delay(100);
  }

  if (diff[0] < 20 || diff[1] < 20 || diff[2] < 20 || diff[3] < 20)
  {
    Lcd_Write_String(LINE1, "SETTING FAILED! ");
    Lcd_Write_String(LINE2, "CONTROL VOLUME..");
  }
  else
  {
    Line0_White = minV[0] + (char)(diff[0] * 0.66);
    Lcd_Cmd(LINE2);
    Lcd_Data('0 : ');
    DigitDisplay(minV[0]);
    Lcd_Data(' ');
    DigitDisplay(maxV[0]);
    Lcd_Data(' ');
    DigitDisplay(Line0_White);
    Lcd_Data(' ');
    delay(1000);

    Line1_White = minV[1] + (char)(diff[1] * 0.66);
    Lcd_Cmd(LINE2);
    Lcd_Data('3 : ');
    DigitDisplay(minV[1]);
    Lcd_Data(' ');
    DigitDisplay(maxV[1]);
    Lcd_Data(' ');
    DigitDisplay(Line1_White);
    Lcd_Data(' ');
    delay(1000);

    Line2_White = minV[2] + (char)(diff[2] * 0.66);
    Lcd_Cmd(LINE2);
    Lcd_Data('6 : ');
    DigitDisplay(minV[2]);
    Lcd_Data(' ');
    DigitDisplay(maxV[2]);
    Lcd_Data(' ');
    DigitDisplay(Line2_White);
    Lcd_Data(' ');
    delay(1000);

    Line3_White = minV[3] + (char)(diff[3] * 0.66);
    Lcd_Cmd(LINE2);
    Lcd_Data('9 : ');
    DigitDisplay(minV[3]);
    Lcd_Data(' ');
    DigitDisplay(maxV[3]);
    Lcd_Data(' ');
    DigitDisplay(Line3_White);
    Lcd_Data(' ');
    delay(1000);

    EEPROM.write(2, Line0_White);
    EEPROM.write(3, Line1_White);
    EEPROM.write(4, Line2_White);
    EEPROM.write(5, Line3_White);

    Lcd_Write_String(LINE1, "SETTING COMPLETE ");
    Lcd_Cmd(LINE2);
    Lcd_Data(' ');
    DigitDisplay(Line0_White);
    Lcd_Data(' ');
    DigitDisplay(Line1_White);
    Lcd_Data(' ');
    DigitDisplay(Line2_White);
    Lcd_Data(' ');
    DigitDisplay(Line3_White);
    Lcd_Data(' ');
  }
  delay(3000);
  while (ENTER) ;
}


void menu_display(unsigned char no)
{
  switch (no)
  {
    case 0: Lcd_Write_String(LINE2, "RUN PROGRAM 1   ");
      break;
    case 1: Lcd_Write_String(LINE2, "RUN PROGRAM 2   ");
      break;
    case 2: Lcd_Write_String(LINE2, "[BALL FOLLOWER] ");
      break;
    case 3: Lcd_Write_String(LINE2, "[ GOAL FINDER ] ");
      break;
    case 4: Lcd_Write_String(LINE2, "VIEW IR         ");
      break;
    case 5: Lcd_Write_String(LINE2, "VIEW CAPTURE IR ");
      break;
    case 6: Lcd_Write_String(LINE2, "VIEW ULTRA      ");
      break;
    case 7: Lcd_Write_String(LINE2, "VIEW LINE       ");
      break;
    case 8: Lcd_Write_String(LINE2, "SET LINE THRE.  ");
      break;
    case 9: Lcd_Write_String(LINE2, "COMP CALIBRATION");
      break;
    case 10: Lcd_Write_String(LINE2, "SET COMPASS DIR.");
      break;
  }
}


void compass_move(int ma, int mb, int mc)
{
  comp = compass - 180;

  if (comp > 100)
  {
    move(50, 50, 50);
  }
  else if (comp < -100)
  {
    move(-50, -50, -50);
  }
  else
  {
    move(ma + comp * KP, mb + comp * KP, mc + comp * KP);
  }
}

void dir_move(int input_ball, int power)
{
  if (!escape) lastdir = input_ball;

  switch (input_ball)
  {
    case 0:
      compass_move(power, 0, -power);
      break;
    case 1:
      compass_move(power / 2, power / 2, -power);
      break;
    case 2:
      compass_move(0, power, -power);
      break;
    case 3:
      compass_move(-power / 2, power, -power / 2);
      break;
    case 4:
      compass_move(-power, power, 0);
      break;
    case 5:
      compass_move(-power, power / 2, power / 2);
      break;
    case 6:
      compass_move(-power, 0, power);
      break;
    case 7:
      compass_move(-power / 2, -power / 2, power);
      break;
    case 8:
      compass_move(0, -power, power);
      break;
    case 9:
      compass_move(power / 2, - power, power / 2);
      break;
    case 10:
      compass_move(power, -power, 0);
      break;
    case 11:
      compass_move(power, -power / 2, -power / 2);
      break;
  }
}

void ball_near(int dir, int power)
{
  switch (dir)
  {
    case 0:
      dir_move(0, 100);
      break;
    case 1:
      dir_move(1, 90);//기본 4
      break;
    case 2:
      dir_move(3, 90);
      break;
    case 3:
      dir_move(4, 90);//기본 6
      break;
    case 4:
      dir_move(7 , 90);
      break;
    case 5:
      dir_move(8, 90);
      break;
    case 6:                 // 초음파로 방향향 결정이 필요함.
      dir_move(9, 90);
      break;
    case 7:
      dir_move(4, 90);
      break;
    case 8:
      dir_move(5, 90);
      break;
    case 9:
      dir_move(8, 90);//기본 6
      break;
    case 10:
      dir_move(9, 90);//기본 7
      break;
    case 11:
      dir_move(9, 90);//기본 8
      break;
  }
}

void linechk(void)
{
  if (LineDetected && !escape)   //경계라인이 감지되었으면 탈출 함.
  {
    escape = true;
    delay(200);
    dir_move(Escape_Dir[LineDetected], 100);
    delay(500);
    move(0, 0, 0);
    delay(200);

    LineDetected = 0;
    escape = false;

  }
}
void PROGRAM1(void)//공격
{

  Lcd_Clear();
  Lcd_Write_String(LINE1, "RUNNING PROGRAM1");

  while (!ENTER)
  {
    Lcd_Cmd(LINE2);
    DigitDisplay(max_ir);
    read_compass();
    if (LineDetected == 1)
    {
      for (int a = 0; a < 400; a++)
      {
        read_compass();
        dir_move(6, 90);
        delay(1);
      }
      move(0, 0, 0);
      delay(200);
    }
    else if (LineDetected == 2)
    {
      for (int b = 0; b < 400; b++)
      {
        read_compass();
        dir_move(9, 90);
        delay(1);
      }
      move(0, 0, 0);
      delay(200);
    }
    else if (LineDetected == 4)
    {
      for (int c = 0; c < 400; c++)
      {
        read_compass();
        dir_move(0, 90);
        delay(1);
      }
      move(0, 0, 0);
      delay(200);
    }
    else if (LineDetected == 8)
    {
      for (int d = 0; d < 400; d++)
      {
        read_compass();
        dir_move(3, 90);
        delay(1);
      }
      move(0, 0, 0);
      delay(200);
    }
    else if (LineDetected == 10)
    {
      move(0, 0, 0);
      delay(200);
      for (int i = 0; i < 400; i++)
      {
        read_compass();
        dir_move(6, 30);
        delay(1);
      }
      move(0, 0, 0);
      delay(200);
    }
    else if (LineDetected == 9)
    {
      for (int t = 0; t < 400; t++)
      {
        read_compass();
        dir_move(4, 30);
        delay(1);
      }
      move(0, 0, 0);
      delay(200);
    }
    else if (LineDetected == 12)
    {
      for (int y = 0; y < 400; y++)
      {
        read_compass();
        dir_move(3, 30);
        delay(1);
      }
      move(0, 0, 0);
      delay(200);
    }

    else if (LineDetected == 6)
    {
      move(0, 0, 0);
      delay(200);
      for (int x = 0; x < 400; x++)
      {
        read_compass();
        dir_move(9, 30);
        delay(1);
      }
      move(0, 0, 0);
      delay(200);
    }
    else if (LineDetected == 3)
    {
      move(0, 0, 0);
      delay(200);
      for (int l = 0; l < 400; l++)
      {
        read_compass();
        dir_move(7, 30);
        delay(1);
      }
      move(0, 0, 0);
      delay(200);
    }
    
    //
    LineDetected = 0; //라인 감지 초기화
    if (max_ir >= 3)
    {
      LineDetected = 0;

      if (ball_dir > 6)           //이전에 공이 왼쪽에 있었음.
        last_pos = 1;
      else if (ball_dir >= 1 && ball_dir < 6) //이전에 공이 오른쪽에 있었음.
        last_pos = 0;

      //    linechk();
      /*
            if (LineDetected)   //경계라인이 감지되었으면 탈출 함.
            {

              delay(100);

              dir_move(Escape_Dir[LineDetected], 100);
              delay(400);

              move(0, 0, 0);

              LineDetected = 0;
            }
      */
      if (ball_dir < 12)
      {

        if (max_ir > SHOOT && (ball_dir == 0)) // || ball_dir == 1 || ball_dir == 11))     //공이 가까이 있을 때
        {
          ultra_gap = (int)((float)ultra[1] * 0.85) - (int)((float)ultra[3] * 0.85);

          comp = compass - 180 - ultra_gap;

          move(100 + comp * KP, comp * KP, -100 + comp * KP);

          dir_move(0, 100);
          compass_move(100, -ultra_gap * 2, -100);
        }
        else
        {
          if (max_ir > 40 && ball_dir == 6)
          {
            dir_move(4, 100);
          }
          else if (max_ir > NEAR)   ball_near(ball_dir, 100);
          else                      dir_move(ball_dir, 100);
        }
      }
    }
    else
    {
      dir_move(0, 0);
    }


  }
}


/*
  void PROGRAM2(void)//수비
  {
  int ultra_gap = 0;

  move(0,0,0);
  MOTORD(0);    //드리블러 STOP.
  //  MOTORD(-50);    //드리블러 ON

  Lcd_Clear();
  Lcd_Write_String(LINE1,"RUNNING PROGRAM2");

  while(ENTER) ;

  //  Shooting(500); // 0.5초간 솔레노이드 작동 시키기.

  while(!ENTER)   //  ENTER 키가 눌릴때까지 아래 명령 반복 EV3 루프와 동일
  {
    read_compass();
    comp = compass - 180;

    if(ball_dir > 6)
      last_pos = 1;
    else if(ball_dir >= 1 && ball_dir < 6)
      last_pos = 0;

    linechk();

    if((int)((float)ultra[2]*0.85) < 30)
      dir_m ove(0, 50);
    else
    {
      if(max_ir > SHOOT2 && (ball_dir == 0)) // || ball_dir == 1 || ball_dir == 11)     //공이 가까이 있을 때
      {
        for(int i=0;i<5;i++)
        {
          dir_move(ball_dir, 100);
           delay(100);
        }
        move(0,0,0);
      }
      if(max_ir > NEAR2)
      {
        dir_move(ball_dir, 100);
      }
      else
      {
        if((int)((float)ultra[1]*0.85) + (int)((float)ultra[3]*0.85) > 40 && (int)((float)ultra[1]*0.85) - (int)((float)ultra[3]*0.85) < 20 && (int)ultra[1] - (int)ultra[3] > -20 && (int)((float)ultra[2]*0.85) > 40)
          dir_move(6, 100);
        else if((int)((float)ultra[1]*0.85) - (int)((float)ultra[3]*0.85) > 10 && (int)((float)ultra[1]*0.85) + (int)((float)ultra[3]*0.85) > 40)
          dir_move(3, 90);
        else if((int)((float)ultra[1]*0.85) - (int)((float)ultra[3]*0.85) < -10 && (int)((float)ultra[1]*0.85) + (int)((float)ultra[3]*0.85) > 40)
          dir_move(9, 90);
        else
          dir_move(0, 0);
      }
    }
  }
  while(ENTER) ;
  }
*/
void PROGRAM2(void) //수비
{
  int ultra_gap = 0;

  move(0, 0, 0);
  MOTORD(0);    //드리블러 STOP.
  //  MOTORD(-50);    //드리블러 ON

  Lcd_Clear();
  Lcd_Write_String(LINE1, "RUNNING PROGRAM2");

  while (ENTER) ;

  //  Shooting(500); // 0.5초간 솔레노이드 작동 시키기.

  while (!ENTER)  //  ENTER 키가 눌릴때까지 아래 명령 반복 EV3 루프와 동일
  {
    if (ball_dir > 6)
      last_pos = 1;
    else if (ball_dir >= 1 && ball_dir < 6)
      last_pos = 0;

    if ((int)((float)ultra[2] * 0.85) < 30)
      dir_move(0, 50);
    else if ((float)ultra[1] * 0.85 < 15 && max_ir < 10)
    {
      move(0, 0, 0);
      delay(100);
      dir_move(9, 100);
      delay(700);
      LineDetected = 0;
    }
    else if ((float)ultra[3] * 0.85 < 15 && max_ir < 10)
    {
      move(0, 0, 0);
      delay(100);
      dir_move(3, 100);
      delay(700);
      LineDetected = 0;
    }
    else if (LineDetected)
    {
      move(0, 0, 0);
      delay(100);

      dir_move(Escape_Dir[LineDetected], 100);
      delay(400);

      LineDetected = 0;
    }
    else
    {
      if (max_ir > SHOOT2 && (ball_dir == 0))    //공이 가까이 있을 때
      {
        ultra_gap = (int)((float)ultra[1] * 0.85) - (int)((float)ultra[3] * 0.85);
        int comp;
        read_compass();
        //        comp = (int)compass / 10;
        comp = comp - 180 - ultra_gap;

        move(100 + comp * KP, comp * KP, -100 + comp * KP);

        dir_move(0, 100);
        for (int i = 0; i < 100; i++)
        {
          compass_move(100, -ultra_gap * 2, -100);
        }
        move(0, 0, 0);
        delay(100);
        dir_move(6, 100);
        delay(200);
      }
      if (max_ir > NEAR2)
      {
        dir_move(ball_dir, 100);
      }
      else
      {
        if ((int)((float)ultra[1] * 0.85) + (int)((float)ultra[3] * 0.85) > 40 && (int)((float)ultra[1] * 0.85) - (int)((float)ultra[3] * 0.85) < 20 && (int)ultra[1] - (int)ultra[3] > -20 && (int)((float)ultra[2] * 0.85) > 40)
          dir_move(6, 100);
        else if ((int)((float)ultra[1] * 0.85) - (int)((float)ultra[3] * 0.85) > 10 && (int)((float)ultra[1] * 0.85) + (int)((float)ultra[3] * 0.85) > 40)
          dir_move(3, 90);
        else if ((int)((float)ultra[1] * 0.85) - (int)((float)ultra[3] * 0.85) < -10 && (int)((float)ultra[1] * 0.85) + (int)((float)ultra[3] * 0.85) > 40)
          dir_move(9, 90);
        else
          dir_move(0, 0);
      }
    }
  }
  while (ENTER) ;

}




void PROGRAM3(void)
{
  Lcd_Clear();
  Lcd_Write_String(LINE1, "TEST");
  Lcd_Write_String(LINE2, "[BALL FOLLOWER]");
  while (ENTER) ;

  //  Shooting(500); // 0.5초간 솔레노이드 작동 시키기.

  while (!ENTER)  //  ENTER 키가 눌릴때까지 아래 명령 반복 EV3 루프와 동일
  {
    if (ball_dir > 11) motor_stop();  // 만약 공의 방향이 11시보다 크면 공을 못찾은것임 - 정지
    else if (ball_dir == 0)           // 아니고 만약 공의 방향이 0시 방향이고
    {
      if (ultra[0] < 7)               // 만약 전방 거리 측정값이 (7 * 0.85 = 0.59) 6 cm 보다 작으면
      {
        move(100, 0, -100);           // 공이 바로 앞에 있다고 판단 전속력으로 직진

        delay(300);// 1초 지속 후.
        MOTORD(0);

        motor_stop();                 // 정지
        delay(500);
      }

      else move(30, 0, -30);          // 전방 거리측정값이 6cm 보다 작지 않으면 파워 30으로 전진

    }
    else if (ball_dir <= 6) move(-30, -30, -30 ); // 만약 공의 방향이 6시방향보다 작거나 같으면 우회전
    else move(30, 30, 30);                      // 아니면 좌회전
    if (max_ir > 30) MOTORD(-100);
    else MOTORD(0);
  }
  while (ENTER) ;
  motor_stop();

}


void PROGRAM4(void)
{
  uint16_t blocks;

  /* PIXY 카메라 정보
    pixy.blocks[i].signature  The signature number of the detected object (1-7 for normal signatures)
    pixy.blocks[i].x          The x location of the center of the detected object (0 to 319)
    pixy.blocks[i].y          The y location of the center of the detected object (0 to 199)
    pixy.blocks[i].width      The width of the detected object (1 to 320)
    pixy.blocks[i].height     The height of the detected object (1 to 200)
    pixy.blocks[i].angle      The angle of the object detected object if the detected object is a color code.
    pixy.blocks[i].print()    A member function that prints the detected object information to the serial port(사용 금지)
  */

  Lcd_Clear();
  Lcd_Write_String(LINE1, "TEST X=         ");
  Lcd_Write_String(LINE2, " [ GOAL FINDER ]");
  while (ENTER) ;

  while (!ENTER)  //  ENTER 키가 눌릴때까지 아래 명령 반복 EV3 루프와 동일
  {
    Lcd_Move(0, 8);

    if (CameraExist)
    {
      blocks = pixy.getBlocks();
      if (blocks)
      {
        Lcd_Move(0, 8);
        DigitDisplay(pixy.blocks[0].x);

        if (pixy.blocks[0].x > 210)        move(0, 0, -40 ); // 만약 골대가 오른쪽에 있으면 우회전
        else if (pixy.blocks[0].x < 110)   move(40, 0, 0 );   // 만약 골대가 왼쪽에 있으면 좌회전
        else                              motor_stop();        // 골대가 중앙에 있으면 정지
      }
      else
      {
        motor_stop();
        Lcd_String("???");

      }
      delay(100);
    }
    else  Lcd_String("NO CAM");
  }
  while (ENTER) ;
  motor_stop();

}

void setup(void)
{

  init_devices();

  Wire.begin();

  delay(100);

  //FIND COMPASS SENSOR
  if (!Check_Compass())
  {
    Lcd_Clear();
    Lcd_Write_String(LINE1, "CHECK YOUR GY273");
    Lcd_Write_String(LINE2, "COMPASS SENSOR!");
    while (1) ;
  }

  Timer1.initialize(50);
  Timer1.attachInterrupt(Scan_Ultra); // blinkLED to run every 0.15 seconds

  // 컴파스 센서 기준값 읽기.
  memComp = EEPROM.read(0);
  memComp = ((memComp << 8) & 0xFF00) | EEPROM.read(1);

  // 라인 경계선 기준값 읽기.
  Line0_White = EEPROM.read(2);
  Line1_White = EEPROM.read(3);
  Line2_White = EEPROM.read(4);
  Line3_White = EEPROM.read(5);

  xoff = EEPROM.read(10);
  xoff = (xoff << 8 & 0xFF00) | EEPROM.read(11);

  int scale = EEPROM.read(12);
  scale = (scale << 8 & 0xFF00) | EEPROM.read(13);
  xs = (float)scale / 1000.;

  yoff = EEPROM.read(14);
  yoff = (yoff << 8 & 0xFF00) | EEPROM.read(15);

  scale = EEPROM.read(16);
  scale = (scale << 8 & 0xFF00) | EEPROM.read(17);
  ys = (float)scale / 1000.;

  /*

    read_compass();
    delay(100);
    read_compass();
    memComp = compass;
  */


  Lcd_Clear();
  Lcd_Write_String(LINE1, "RCKA");
  Lcd_Write_String(LINE2, "ROBOT SOCCER 3.0");

  if (CameraExist) pixy.init();

}

void loop(void)
{

  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];

  read_compass();
  Lcd_Move(0, 6);
  DigitDisplay(compass);
  Lcd_Data(0xDF);
  Volt_Display(Voltage);

  if (ENTER)
  {
    while (ENTER) ;
    menu_display(menu);
    while (1)
    {
      read_compass();
      Lcd_Move(0, 6);
      DigitDisplay(compass);
      Lcd_Data(0xDF);
      Volt_Display(Voltage);

      if (PREV)
      {
        while (PREV) ;
        menu--;
        if (menu < 0) menu = 10;
        menu_display(menu);
      }
      if (NEXT)
      {
        while (NEXT) ;
        menu++;
        if (menu > 10) menu = 0;
        menu_display(menu);
      }
      if (ENTER)
      {
        while (ENTER)  ;
        switch (menu)
        {
          case 0: PROGRAM1();
            break;
          case 1: PROGRAM2();
            break;
          case 2: PROGRAM3();
            break;
          case 3: PROGRAM4();
            break;
          case 4: view_ir();
            break;
          case 5: view_capture();
            break;
          case 6: view_ultra();
            break;
          case 7: view_line();
            break;
          case 8: set_Line();
            break;
          case 9: Compass_IRON();
            break;
          case 10: set_Compass();
            break;
        }

        Lcd_Clear();
        Lcd_Write_String(LINE1, "RCKA");
        menu_display(menu);
      }
      delay(200);
    }
  }
  delay(200);
}

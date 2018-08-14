/*
   SOCCERV3.c

   Created: 2017-010-02
   Author: JEON HAKYEONG (전하경)

   History

   2017-10-02
    Migration From AtmelStudio.

*/
//
// 키 입력 값
//
#define ENTER (~PINB & 0x40)  //ENTER KEY 입력 값
#define PREV (~PINB & 0x20)   //PREV KEY 입력 값
#define NEXT (~PINB & 0x10)   //NEXT KEY 입력 값
//
// 공격 수비 공 판단 기준 값
//
#define SHOOT 165  //공격수 슈팅 기준값, 먼곳에서 슈팅을 시도하면 이값을 키운다.
#define NEAR 13    //공격수 공 근처의 기준값,  공을 끼고 너무 크게 돌면 이값을 키운다.
#define SHOOT2 180 //수비수 슈팅 기준값, 먼곳에서 슈팅을 시도하면 이값을 키운다.
#define NEAR2 13   //수비수 공 감지 기준값, 가반응거리가 짧으면  이값을 줄인다.
//
// 아래 정의값들은 경기장 테두리의흰색 선의 기준값입니다.
// 경기장에서 직접 측정하여 변경하던지 가변저항을 조정하세요.
// 아래 설정값 보다 큰값이 입력되면 흰색 경계선으로 인식
//--------------------------------------------------------------------------
// 센서별로 흰색라인에 올려 놓고 가변저항을 조정하여 200정도에 맞춤.
// 경기장 바닥에서 입력되는 값과 흰색에서 입력되는 값의 중간 또는 3분의 2 정도로 설정
//---------------------------------------------------------------------------
//
#define Line0_White 15     // 0시 방향 경계선 감지 기준 값
#define Line1_White 15     // 3시 방향 경계선 감지 기준 값
#define Line2_White 30     // 6시 방향 경계선 감지 기준 값
#define Line3_White 15     // 9시 방향 경계선 감지 기준 값

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
#define NumberOfSamples 4   //가능한 변경하지 마세요.. 이값을 변경하고자 하면 Header File쪽도 수정해야 함.
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
volatile double compass;

volatile unsigned int BallCapture = 0; // 사용 안함.
#define Voltage ((int)((float)ADC_Value[16]*0.625 -2.5))

//************************************************************************************

// 메뉴 디스플레이
volatile char menu = 0;
//----------------------------------------------------------------------------
#define KP 0.9
//----------------------------------------------------------------------------

#include "SOCCERV3.h"

#include <TimerOne.h>

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

void view_max_ir(void)
{
  Lcd_Clear();
  Lcd_Write_String(LINE1, "MAX IR");
  while (!ENTER)
  {
    Lcd_Cmd(LINE2);
    DigitDisplay(max_ir);
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

void menu_display(unsigned char no)
{ 
  
  switch (no)
  {
    case 0: Lcd_Write_String(LINE2, "ATTACKER        ");
      break;
    case 1: Lcd_Write_String(LINE2, "DEFENDER        ");
      break;
    case 2: Lcd_Write_String(LINE2, "[BALL FOLLOWER] ");
      break;
    case 3: Lcd_Write_String(LINE2, "VIEW IR         ");
      break;
    case 4: Lcd_Write_String(LINE2, "VIEW MAX_IR     ");
      break;
    case 5: Lcd_Write_String(LINE2, "VIEW ULTRA      ");
      break;
    case 6: Lcd_Write_String(LINE2, "VIEW LINE       ");
      break;
  }
}


void compass_move(int ma, int mb, int mc)
{
  int comp;
  read_compass();
  //  comp = (int)compass / 10;
  comp = compass;
  comp = comp - 180;
  if (comp > 100) // 원래 +100
  {
    move(50, 50, 50);
  }
  else if (comp < -100) // 원래 -100
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
      compass_move(-power / 2, power, -power / 2);  // new1.5 -> old 2
      break;
    case 4:
      compass_move(-power, power,0); // new power/5 -> old 0 
      break;
    case 5:
      compass_move(-power, power / 2, power / 2); // 3/4 -> 1
      break;
    case 6:
      compass_move(-power, 0, power);
      break;
    case 7:
      compass_move(-power / 2, -power / 2, power);  // 3/4 -> 1
      break;
    case 8:
      compass_move(0, -power, power); // new -power/5 -> old 0
      break;
    case 9:
      compass_move(power / 2, - power, power / 2); // new 1.5 -> old 2 
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
      dir_move(0, power);
      break;
    case 1:
      dir_move(2, power);
      break;
    case 2:
      dir_move(4, power);
      break;
    case 3:
      dir_move(5, power);
      break;
    case 4:
      dir_move(6, power);
      break;
    case 5:
      dir_move(7, power);
      break;
    case 6:
      dir_move(8, power);
      break;
    case 7:
      dir_move(5, power);
      break;
    case 8:
      dir_move(4, power);
      break;
    case 9:
      dir_move(7, power);
      break;
    case 10:
      dir_move(8, power);
      break;
    case 11:
      dir_move(10, power);
      break;
  }
}
 void attack_move ( ) // 수비수 전용
 {
  int a =0;
  if(max_ir <=20) // 공이 멀다
  {
    switch(ball_dir)
        {
           case 0: dir_move(0,100);
                   break;
           case 1: dir_move(2,90);
                   break;
           case 2: dir_move(3,90);
                   break;
           case 3: dir_move(4,90);
                   break;
           case 4: dir_move(5,100);
                   break;
           case 5: dir_move(6,100);
                   break;
           case 6: dir_move(5,95);            
                   break;
           case 7: dir_move(6,100);
                    break;
           case 8: dir_move(7,100);
                   break;
           case 9: dir_move(8,90);
                   break;
           case 10: dir_move(9,90);
                    break;
           case 11: dir_move(10,90);
                    break;
        }
    } 

    else //공이 로봇으로 부터 중간거리
    { 
      switch(ball_dir)
      {
           case 0: dir_move(0,100);
                   break;
           case 1: if(max_ir >= 230 && (int)((float)ultra[2] * 0.85) <= 16) // 캡쳐링존 안에 공이 있다.
                   dir_move(0,100); 
//                   else if(max_ir >= 200)
//                   dir_move(4,50);
                   else if(ADC_Value[1] - ADC_Value[0] > 40 && max_ir > 100 && ADC_Value[0] >= 75)
                   dir_move(1,90);
                   else
                   dir_move(2,70);
                   break;
           case 2: if(max_ir >= 180)
                   dir_move(5,70);
                   else
                   dir_move(4,90);
                   break;
           case 3: dir_move(5,90);
                   break;
           case 4: dir_move(6,100);
                   break;
           case 5: dir_move(7,90);
                   break;
           case 6: dir_move(4,95);            
                   break;
           case 7: dir_move(5,90);
                    break;
           case 8: dir_move(6,100);
                   break;
           case 9: dir_move(7,90);
                   break;
           case 10: if(max_ir >= 180)
                    dir_move(7,70);
                    else
                    dir_move(8,90);
                    break;
           case 11: if(max_ir >= 230 && (int)((float)ultra[2] * 0.85) <= 16) // 캡쳐링존 안에 공이 있다
                     dir_move(0,100);
//                     else if(max_ir >= 200)
//                     dir_move(8,50);
                     else if(ADC_Value[11] - ADC_Value[0] > 40 && max_ir > 100 && ADC_Value[0] >= 75) 
                     dir_move(11,90);
                     else 
                     dir_move(10,70);
                     break ;  
        }
    }

 }
void attacking_move ( ) // 새로운 무브
 {
  if(max_ir <= 60) // 공 멀리 있을 떄
  {
    switch(ball_dir)
        {
           case 0: dir_move(0,100);
                   break;
           case 1: dir_move(2,100);
                   break;
           case 2: dir_move(3,100);
                   break;
           case 3: dir_move(4,100);
                   break;
           case 4: dir_move(6,100);
                   break;
           case 5: dir_move(6,100);
                   break;
           case 6: dir_move(4,100);
                   break;
           case 7: dir_move(6,100);
                    break;
           case 8: dir_move(6,100);
                   break;
           case 9: dir_move(8,100);
                   break;
           case 10: dir_move(9,100);
                    break;
           case 11: dir_move(10,100);
                    break;  
        }
    }

    else  // 공 가까이 있을 떄
    { 
      switch(ball_dir)
      {
           case 0: dir_move(0,100);
                   break;
           case 1: dir_move(3,100);
                   break;
           case 2: dir_move(3,100);
                   break;
           case 3: dir_move(4,100);
                   break;
           case 4: dir_move(5,100);
                   break;
           case 5: dir_move(6,100);
                   break;
           case 6: if((int)((float)ultra[2] * 0.85) <= 30)
                   {
                     dir_move(7,100);
                   }
                   else if((int)((float)ultra[4] * 0.85) <= 30)
                   {
                     dir_move(4,100);
                   }
                   break;
           case 7: dir_move(6,100);
                    break;
           case 8: dir_move(7,100);
                   break;
           case 9: dir_move(8,100);
                   break;
           case 10: dir_move(9,100);
                    break;
           case 11: dir_move(10,100);
                    break;    
      }
    }
 }
 void defense_move ( ) //수비수 전용
 {
  int a;
    switch(ball_dir)
        {
           case 0: if(max_ir <= 20)
                   dir_move(0,0);
                   else
                   dir_move(0,100);
                   break;
           case 1:  a= 0;
                   if(max_ir <= 40)
                   dir_move(2,50);
                   else
                   dir_move(3,80);
                   break;
           case 2: compass_move(-100/2, 100, -100/2); // 3시 방향
                   break;
           case 3: compass_move(-100/2, 100, -100/2);
                   break;
           case 4: compass_move(-100/2, 100, -100/2);
                   break;
           case 5: dir_move(6,100);
                   break;
           case 6: dir_move(4,95);           
                   break;
           case 7: dir_move(6,100);
                    break;
           case 8: compass_move(100/2, -100, 100/2);
                   break;
           case 9: compass_move(100/2, -100, 100/2);
                   break;
           case 10: compass_move(100/2, -100, 100/2); // 9시 방향
                    break;
           case 11:a =0;
                    if(max_ir <= 40)
                    dir_move(10,50);
                    else
                    dir_move(9,80);
                    break;
        }
   }


 void attacker_move ()
 {  
   if(max_ir <=20) // 공이 멀다
  {
    switch(ball_dir)
        {
           case 0: dir_move(0,100);
                   break;
           case 1: dir_move(2,100);
                   break;
           case 2: dir_move(3,100);
                   break;
           case 3: dir_move(4,100);
                   break;
           case 4: dir_move(5,100);
                   break;
           case 5: dir_move(6,100);
                   break;
           case 6: dir_move(5,100);            
                   break;
           case 7: dir_move(6,100);
                    break;
           case 8: dir_move(7,100);
                   break;
           case 9: dir_move(8,100);
                   break;
           case 10: dir_move(9,100);
                    break;
           case 11: dir_move(10,100);
                    break;
        }
    } 

    else //공이 로봇으로 부터 중간거리
    { 
      switch(ball_dir)
      {
           case 0: dir_move(0,100);
                   break;
           case 1: if(max_ir >= 230 && (int)((float)ultra[2] * 0.85) <= 16) // 캡쳐링존 안에 공이 있다.
                   dir_move(0,100); 
                   else if(ADC_Value[1] - ADC_Value[0] > 40 && max_ir > 100 && ADC_Value[0] >= 75)
                   dir_move(1,90);
                   else
                   dir_move(2,80);
                   break;
           case 2: if(max_ir >= 180)
                   dir_move(5,80);
                   else
                   dir_move(4,80);
                   break;
           case 3: if(max_ir >= 180)
                   dir_move(6,80);
                   else
                   dir_move(5,80);
                   break;
           case 4: dir_move(6,100);
                   break;
           case 5: dir_move(7,100);
                   break;
           case 6: dir_move(4,100);            
                   break;
           case 7: dir_move(5,100);
                    break;
           case 8: dir_move(6,100);
                   break;
           case 9: if(max_ir >= 200)
                   dir_move(6,80);
                   else
                   dir_move(7,80);
                   break;
           case 10: if(max_ir >= 180)
                    dir_move(7,80);
                    else
                    dir_move(8,80);
                    break;
           case 11: if(max_ir >= 230 && (int)((float)ultra[2] * 0.85) <= 16) // 캡쳐링존 안에 공이 있다
                     dir_move(0,100);
                     else if(ADC_Value[11] - ADC_Value[0] > 40 && max_ir > 100 && ADC_Value[0] >= 75)
                     dir_move(11,90);
                     else 
                     dir_move(10,80);
                     break ;  
        }
    }
 }
void NEWLINE(void) // 대각선의 흰선을 보았을때 나가지 않도록 프로그래밍 공격전용
{
  if((int)((float)ultra[0] * 0.85) < 35 && (int)((float)ultra[1] * 0.85) < 35 ) //&& ball_dir >= 11 && ball_dir <= 2 )  // 우측상단
  {
    move(0,0,0);
    delay(100);
    dir_move(7,70);
    delay(500);
    move(0,0,0);
    delay(100);
    dir_move(9,70);
    delay(100);
  }
  else if((int)((float)ultra[0] * 0.85) < 35 && (int)((float)ultra[3] * 0.85) < 35  ) //&& ball_dir >= 10 && ball_dir <= 1) // 좌측상단
  {
    move(0,0,0);
    delay(100);
    dir_move(5,70);
    delay(500);
    move(0,0,0);
    delay(100);
    dir_move(3,70);
    delay(100);
  }
  if((int)((float)ultra[2] * 0.85) < 35 && (int)((float)ultra[1] * 0.85) < 35 && ball_dir >= 1 && ball_dir <= 6 ) // 우측하단
  {
    move(0,0,0);
    delay(100);
    dir_move(10,70);
    delay(500);
    move(0,0,0);
    delay(100);
    dir_move(9,70);
    delay(100);
  }
  else if((int)((float)ultra[2] * 0.85) < 35 && (int)((float)ultra[3] * 0.85) < 35 && ball_dir >= 6 && ball_dir <= 11 ) // 좌측하단
  {
    move(0,0,0);
    delay(100);
    dir_move(2,70);
    delay(500);
    move(0,0,0);
    delay(100);
    dir_move(3,70);
    delay(100);
  }
}
void DEFLINE(void) // 대각선 수비 전용 ( 하단 대각선만 프로그래밍했다 ) 
{
  if((int)((float)ultra[2] * 0.85) < 30 && (int)((float)ultra[1] * 0.85) < 30 )//&& ball_dir >= 1 && ball_dir <=6 ) // 우측하단
  {
    move(0,0,0);
    dir_move(10,70);
    delay(300); // 500
    //move(0,0,0);
    //dir_move(9,70);
    //delay(200);
  }
  else if((int)((float)ultra[2] * 0.85) < 30 && (int)((float)ultra[3] * 0.85) < 30 && ball_dir >= 6 && ball_dir <= 11 ) // 좌측하단
  {
    move(0,0,0);
    dir_move(2,70);
    delay(300); // 500
    //move(0,0,0);
    //dir_move(3,70);
    //delay(200);
  }
}
void JOONG(void) // 중앙
{
  int alpha = 100;
  int k = 0;
  int ultra_sum = 0;
  int ultra_gap = 0;
  int ultraf = (int)((float)ultra[0] * 0.85);
  int ultrar = (int)((float)ultra[1] * 0.85);
  int ultrab = (int)((float)ultra[2] * 0.85);
  int ultral = (int)((float)ultra[3] * 0.85);
  ultra_gap = (int)((float)ultra[1] * 0.85) - (int)((float)ultra[3] * 0.85);
  ultra_sum = (int)((float)ultra[1] * 0.85) + (int)((float)ultra[3] * 0.85);
   if(ultra_sum < 115 && ultra_sum > 95 && (int)((float)ultra[2] * 0.85) <= 40 && ultrab >= 25 ) // && (ball_dir == 11 || ball_dir == 0 || ball_dir == 1)) // 울트라 합한 값, 한 쪽이 골대 안쪽을 인식해서 값이 작게 나온다. 그 현상 방지용
           {                   
             
             if((int)((float)ultra[1] * 0.85) > (int)((float)ultra[3] * 0.85))  // 오른쪽 울트라가 왼쪽 울트라보다 클 때 (로봇이 왼쪽에 있음)
             {

                  while(k < 90)
                  {
                    if(max_ir >= 5)
                    {
                      k = 301;
                      compass_move(0,0,0);
                    }

                     if(ultra_sum >= 155 && ultra_sum <= 165)
                    {
                      k = 301; 
                      compass_move(0,0,0);
                    }
                    
                    else
                    {
                    read_compass();              
                    //compass_move(-65/2,70,-80/2); // 2시 방향
                    dir_move(2,70);
                    k++;
                    }
                  }
                   compass_move(0,0,0);
                   alpha = 100;
                   k = 0;
               
             }

             else // 로봇이 오른쪽에 있음
             { 
             
                  while(k < 90)
                  { 
                    if(max_ir >= 5)
                    {
                      k = 301;
                      compass_move(0,0,0);
                    }

                    if(ultra_sum >= 155 && ultra_sum <= 165)
                    {
                      k = 301; 
                      compass_move(0,0,0);
                    }
                    
                      else
                    {
                       read_compass();
                       //compass_move(80/2,-70,65/2); // 9시 방향
                       dir_move(10,70);
                       k++;
                    }
                  }
                   compass_move(0,0,0);
                   k = 0;
                  alpha = 100;
             }
           }
  
}
void CENTER(void) // 중앙 맞추기 수비 프로그램에서 따온 공격전용 중앙 맞추기 (수비적 성격의 중앙 맞추기)
{
  int alpha = 100;
  int k = 0;
  int ultra_sum = 0;
  int ultra_gap = 0;
  int ultraf = (int)((float)ultra[0] * 0.85);
  int ultrar = (int)((float)ultra[1] * 0.85);
  int ultrab = (int)((float)ultra[2] * 0.85);
  int ultral = (int)((float)ultra[3] * 0.85);
  ultra_gap = (int)((float)ultra[1] * 0.85) - (int)((float)ultra[3] * 0.85);
  ultra_sum = (int)((float)ultra[1] * 0.85) + (int)((float)ultra[3] * 0.85);
  if((int)((float)ultra[1]*0.85) - (int)((float)ultra[3]*0.85) > 22 ) // && (int)((float)ultra[1]*0.85) + (int)((float)ultra[3]*0.85) < 150) // 로봇이 맵의 왼쪽에 있음
           {
             if((int)((float)ultra[2]*0.85) >= 75) // 우리팀 골대에서 멀리 있을 떄
             { 
               dir_move(4,70); // 대각선 3시 방향(3시, 4시 사이)으로 으로 내려감
               alpha = 100;
             }
             else // 우리팀 골대 가까이 있을 때
             {
                if(ultra_sum >= 155 && ultra_sum <= 165 && (int)((float)ultra[2]*0.85) <= 30)
                move(0,0,0);

                else if((int)((float)ultra[1]*0.85) >= 72 && (int)((float)ultra[1]*0.85) <= 88 && (int)((float)ultra[2]*0.85) <= 30 )
                move(0,0,0);

                else if((int)((float)ultra[3]*0.85) >= 72 && (int)((float)ultra[3]*0.85) <= 88 && (int)((float)ultra[2]*0.85) <= 30 )
                move(0,0,0);
                
                else 
                compass_move(-60/2, 60, -60/2); //3시 방향
                
                alpha = 100;
             }
             
           }
      
           else if((int)((float)ultra[1]*0.85) - (int)((float)ultra[3]*0.85) < -22 ) // 로봇이 맵의 오른쪽에 있음
           {
             
              
             if((int)((float)ultra[2] * 0.85) >= 75) // 골대에서 멀리 있을 때
             {
               dir_move(8,70);// 대각선 8시(9시 8시 사이) 방향으로 내려감
               alpha = 100;
             }
             else
             { 
                if(ultra_sum >= 150 && ultra_sum <= 165 && (int)((float)ultra[2]*0.85) <= 50 )
                move(0,0,0);

                else if((int)((float)ultra[1]*0.85) >= 72 && (int)((float)ultra[1]*0.85) <= 88 && (int)((float)ultra[2]*0.85) <= 50)
                move(0,0,0);

                else if((int)((float)ultra[3]*0.85) >= 72 && (int)((float)ultra[3]*0.85) <= 88 && (int)((float)ultra[2]*0.85) <= 50)
                move(0,0,0);
                
                else
                compass_move(60/2,-60, 60/2); // 9시 방향
                
                alpha = 100;
             }
              
           }

           else if(ultra_sum < 115 && ultra_sum > 95 && (int)((float)ultra[2] * 0.85) <= 40 && ultrab >= 25 ) // && (ball_dir == 11 || ball_dir == 0 || ball_dir == 1)) // 울트라 합한 값, 한 쪽이 골대 안쪽을 인식해서 값이 작게 나온다. 그 현상 방지용
           {                   
             
             if((int)((float)ultra[1] * 0.85) > (int)((float)ultra[3] * 0.85))  // 오른쪽 울트라가 왼쪽 울트라보다 클 때 (로봇이 왼쪽에 있음)
             {

                  while(k < 90)
                  {
                    if(max_ir >= 5)
                    {
                      k = 301;
                      compass_move(0,0,0);
                    }

                     if(ultra_sum >= 155 && ultra_sum <= 165)
                    {
                      k = 301; 
                      compass_move(0,0,0);
                    }
                    
                    else
                    {
                    read_compass();              
                    //compass_move(-65/2,70,-80/2); // 2시 방향
                    dir_move(2,70);
                    k++;
                    }
                  }
                   compass_move(0,0,0);
                   alpha = 100;
                   k = 0;
               
             }

             else // 로봇이 오른쪽에 있음
             { 
             
                  while(k < 90)
                  { 
                    if(max_ir >= 5)
                    {
                      k = 301;
                      compass_move(0,0,0);
                    }

                    if(ultra_sum >= 155 && ultra_sum <= 165)
                    {
                      k = 301; 
                      compass_move(0,0,0);
                    }
                    
                      else
                    {
                       read_compass();
                       //compass_move(80/2,-70,65/2); // 9시 방향
                       dir_move(10,70);
                       k++;
                    }
                  }
                   compass_move(0,0,0);
                   k = 0;
                  alpha = 100;
             }
           }
  
}
void ATTACKER(void)//TEST
{
  Lcd_Clear();
  Lcd_Write_String(LINE1, "ATTACKING");
 
  int alpha = 100;   //라인감지될 때 ball_dir 값                                                                   // ultra[2] = 6시 방향 (후)
   int beta = 12;    //max_ir >= 2 일 때의 공 방향    (max_ir >= 2 이면 공이 있다.)                                 // ultra[3] = 9시 방향 (좌)
   int theta = 0; // 공 없어졌을 때 카운트하게 하는 플래그 변수
   int ultra_gap = 0; //  우측 초음파 값 - 좌측 초음파 값
   int ultra_sum = 0; //  우측 초음파 값 + 좌측 초음파 값
   int k = 0;
       LineDetected = 0;
       compass_move(0, 0, 0);
       

       LineDetected = 0;
  
  while (!ENTER)
  {
       int ultraf = (int)((float)ultra[0] * 0.85);
       int ultrar = (int)((float)ultra[1] * 0.85);
       int ultrab = (int)((float)ultra[2] * 0.85);
       int ultral = (int)((float)ultra[3] * 0.85);
       ultra_gap = (int)((float)ultra[1] * 0.85) - (int)((float)ultra[3] * 0.85);
       ultra_sum = (int)((float)ultra[1] * 0.85) + (int)((float)ultra[3] * 0.85);
     if (LineDetected) // 라인
     {  
       alpha = 100; // 라인 감지
       while(k <= 70)
       {
        compass_move(0,0,0);
        k++;
       }
       k = 0;

       while(k <= 65)
       {
        NEWLINE;
        read_compass();
        dir_move(Escape_Dir[LineDetected], 80);
        k++;
       }
       k = 0;
       compass_move(0,0,0);
       alpha = ball_dir;
       LineDetected = 0;
    }  
    else //공찾기
    {
      if(max_ir >= 220 && ball_dir == 0) //지우턴
      {
        alpha = 100;
        ultra_gap = (int)((float)ultra[1] * 0.85) - (int)((float)ultra[3] * 0.85);
        int comp;
        read_compass();
        comp = (int)compass; //나누기 없애보기
        comp = comp -150 - ultra_gap; //150 다시 180으로 바꿔 해보기

        move(100, comp * 2 * KP, -100);      
      }

    
      else if(max_ir >= 3)// 공찾기
      {
        if(alpha == ball_dir && max_ir <= 90 && max_ir >= 4)
        {
          compass_move(0,0,0);
        }

         else
         {
           attacking_move();
           alpha = 100;
         } 
      } 
      else if (max_ir < 3) //공 없을 때
      {
        //JOONG(); 마지막 방향으로 가기 때문에 안쓰는걸로
        //CENTER();
        compass_move(0,0,0);
      }
       
      else 
      {
        compass_move(0,0,0);
      }
    
    }
     
  }                  //전체 반복문
}                    // void 닫기

void DEFENDER(void) // 수비                                                           // ultra[0] = 0시 방향 (전)
{                                                                                     // ultra[1] = 3시 방향 (우)
  int alpha = 0;   //라인감지될 때 ball_dir 값                                        // ultra[2] = 6시 방향 (후)
  int beta = 0;    //                                                                 // ultra[3] = 9시 방향 (좌)
  int a;
  int i = 0;
      a =0;
  int k = 0; // line 나가기
      int ultra_sum = 0;
      int ultra_gap = 0;

       int ultraf = (int)((float)ultra[0] * 0.85);
       int ultrar = (int)((float)ultra[1] * 0.85);
       int ultrab = (int)((float)ultra[2] * 0.85);
       int ultral = (int)((float)ultra[3] * 0.85);

      LineDetected = 0;
      move(0, 0, 0);

  Lcd_Clear();
  Lcd_Write_String(LINE1, "DEFENSING");

  while (!ENTER)  //  ENTER 키가 눌릴때까지 아래 명령 반복 EV3 루프와 동일
  {
    ultra_sum = (int)((float)ultra[1] * 0.85) + (int)((float)ultra[3] * 0.85);
    ultra_gap = (int)((float)ultra[1] * 0.85) - (int)((float)ultra[3] * 0.85);

    ultraf = (int)((float)ultra[0] * 0.85);
    ultrar = (int)((float)ultra[1] * 0.85);
    ultrab = (int)((float)ultra[2] * 0.85);
    ultral = (int)((float)ultra[3] * 0.85);
    if (LineDetected)
    {  
      alpha = 100;
      while(k <= 70)
      {
       compass_move(0,0,0);
       k++;
      }
      k = 0;
       while(k <= 65)
       {
        NEWLINE();  //공격 전용 수비는 중앙을 맞추고 포지션을 잡는데에 문제가 있어 주석처리함
        //DEFLINE(); // 태스트 해보아야함 수비 자리잡는 포지션에 문제되는지 체크
        read_compass();
        dir_move(Escape_Dir[LineDetected], 80);
        k++;
       }
       k = 0;
       compass_move(0,0,0);
       alpha = ball_dir;
       LineDetected = 0;
    }
    else // 라인 안볼 때
    {
      if (max_ir >= 220 && (ball_dir == 0)) //|| ball_dir == 1 || ball_dir == 11))     //공이 가까이 있을 때 ( 지우턴)
      {
        alpha = 100;
        ultra_gap = (int)((float)ultra[1] * 0.85) - (int)((float)ultra[3] * 0.85);
        int comp;
        read_compass();
        comp = (int)compass; //나누기 없애보기
        comp = comp -150 - ultra_gap; //150 다시 180으로 바꿔 해보기

        move(100, comp * 1.5 * KP, -100);
        //move(110 + comp * KP/1.2  , comp/3.5 * KP, -100 + comp * KP/1.2 );
        // dir_move(0, 100);
        //co-mpass_move(100, -ultra_gap * 2, -100);
     
      }

      //if(max_ir > 100) ball_near(ball_dir,100);
      
      else if (max_ir >= 3)   // 공 있을 때 
      {  
         if(ball_dir == alpha && ball_dir < 100 && max_ir < 80 && max_ir >= 4) 
         {
           move(0,0,0);  // 라인 밖 공이 있던 방향 체크 해서 나가지 않게 하기
         }
         else if(max_ir >=80 && (int)((float)ultra[2] * 0.85) >= 30)
         {
           alpha = 100;
           attacking_move(); // 평상시대로 움직이기
           //defense_move();
         }
         else if((int)((float)ultra[2] * 0.85) >= 30)
           {
            alpha = 100; // 알파 값 초기화
            attacking_move(); // 평상시대로 움직이기
            //defense_move();
           }
           else if((int)((float)ultra[2] * 0.85) < 30 )
           {
            alpha = 100;
            //attacking_move();
            defense_move();
           }          
           else
           { 
            alpha = 100;
            //attacking_move();
            defense_move();
           }
         }
          
 
      else if( max_ir <= 2)  // 골대로 돌아가기 공 없을 때
      {    
            alpha = 100;
            
           if((int)((float)ultra[1]*0.85) - (int)((float)ultra[3]*0.85) > 22 ) // && (int)((float)ultra[1]*0.85) + (int)((float)ultra[3]*0.85) < 150) // 로봇이 맵의 왼쪽에 있음
           {
             if((int)((float)ultra[2]*0.85) >= 50) // 우리팀 골대에서 멀리 있을 떄
             { 
               dir_move(4,70); // 대각선 3시 방향(3시, 4시 사이)으로 으로 내려감
               alpha = 100;
             }
             else // 우리팀 골대 가까이 있을 때
             {
                if(ultra_sum >= 155 && ultra_sum <= 165 && (int)((float)ultra[2]*0.85) <= 25)
                move(0,0,0);

                else if((int)((float)ultra[1]*0.85) >= 72 && (int)((float)ultra[1]*0.85) <= 88 && (int)((float)ultra[2]*0.85) <= 26 )
                move(0,0,0);

                else if((int)((float)ultra[3]*0.85) >= 72 && (int)((float)ultra[3]*0.85) <= 88 && (int)((float)ultra[2]*0.85) <= 26 )
                move(0,0,0);
                
                else 
                compass_move(-60/2, 60, -60/2); //3시 방향
                
                alpha = 100;
             }
             
           }
      
           else if((int)((float)ultra[1]*0.85) - (int)((float)ultra[3]*0.85) < -22 ) // 로봇이 맵의 오른쪽에 있음
           {
             
              
             if((int)((float)ultra[2] * 0.85) >= 50) // 골대에서 멀리 있을 때
             {
               dir_move(8,70);// 대각선 8시(9시 8시 사이) 방향으로 내려감
               alpha = 100;
             }
             else
             { 
                if(ultra_sum >= 150 && ultra_sum <= 165 && (int)((float)ultra[2]*0.85) <= 25 )
                move(0,0,0);

                else if((int)((float)ultra[1]*0.85) >= 72 && (int)((float)ultra[1]*0.85) <= 88 && (int)((float)ultra[2]*0.85) <= 25)
                move(0,0,0);

                else if((int)((float)ultra[3]*0.85) >= 72 && (int)((float)ultra[3]*0.85) <= 88 && (int)((float)ultra[2]*0.85) <= 25)
                move(0,0,0);
                
                else
                compass_move(60/2,-60, 60/2); // 9시 방향
                
                alpha = 100;
             }
              
           }

           else if(ultra_sum < 115 && ultra_sum > 95 && (int)((float)ultra[2] * 0.85) <= 40 && ultrab >= 25 ) // && (ball_dir == 11 || ball_dir == 0 || ball_dir == 1)) // 울트라 합한 값, 한 쪽이 골대 안쪽을 인식해서 값이 작게 나온다. 그 현상 방지용
           {                   
             
             if((int)((float)ultra[1] * 0.85) > (int)((float)ultra[3] * 0.85))  // 오른쪽 울트라가 왼쪽 울트라보다 클 때 (로봇이 왼쪽에 있음)
             {

                  while(k < 90)
                  {
                    if(max_ir >= 5)
                    {
                      k = 301;
                      compass_move(0,0,0);
                    }

                     if(ultra_sum >= 155 && ultra_sum <= 165)
                    {
                      k = 301; 
                      compass_move(0,0,0);
                    }
                    
                    else
                    {
                    read_compass();              
                    //compass_move(-65/2,70,-80/2); // 2시 방향
                    dir_move(2,70);
                    k++;
                    }
                  }
                   compass_move(0,0,0);
                   alpha = 100;
                   k = 0;
               
             }

             else // 로봇이 오른쪽에 있음
             { 
             
                  while(k < 90)
                  { 
                    if(max_ir >= 5)
                    {
                      k = 301;
                      compass_move(0,0,0);
                    }

                    if(ultra_sum >= 155 && ultra_sum <= 165)
                    {
                      k = 301; 
                      compass_move(0,0,0);
                    }
                    
                      else
                    {
                       read_compass();
                       //compass_move(80/2,-70,65/2); // 9시 방향
                       dir_move(10,70);
                       k++;
                    }
                  }
                   compass_move(0,0,0);
                   k = 0;
                  alpha = 100;
             }
           }
           

           else if((int)((float)ultra[2]*0.85) > 26 )  // 중앙을 맞춘 후 후진
           {
            if((int)((float)ultra[2] * 0.85) > 70) // 멀리 있을 떄
            {
             dir_move(6,80);
             alpha = 100;
            }

            else 
            {
               dir_move(6,60);
               alpha = 100;
            }

            
           }

           else if((int)((float)ultra[2] * 0.85) <= 17) // 벽에 붙었다.
           {
             dir_move(0, 40);
             alpha = 100;
           }
      
            else
           {
               if(ball_dir == 1 || ball_dir == 2|| ball_dir == 3 || ball_dir == 4)
               {
                 compass_move(-80/2, 80, -80/2); // 3시 방향
                  alpha = 100;
               }
               else if(ball_dir == 11 || ball_dir == 10 || ball_dir == 9 || ball_dir == 8)
               {
                  compass_move(80/2,-80, 80/2); // 9시 방향
                   alpha = 100;
               }
               else
               {
                 compass_move(0,0,0);
                 alpha = 100;
               }
            
            } 
        }
      }
    }
  }//수비 보이드 끝

       
          /*switch(beta)
         {
           case 0: dir_move(6,80);
                   delay(700);
                   move(0,0,0);
                   break;
           case 1: dir_move(7,80);
                   delay(700);
                   move(0,0,0);
                   break;
           case 2: dir_move(8,80);
                   delay(700);
                   move(0,0,0);
                   break;
           case 3: dir_move(9,80);
                   delay(700);
                   move(0,0,0);
                   break;
           case 4: dir_move(10,80);
                   delay(700);
                   move(0,0,0);
                   break;
           case 5: dir_move(11,80);
                   delay(700);
                   move(0,0,0);
                   break;
           case 6: dir_move(0,80);
                   delay(700);
                   move(0,0,0);
                   break;
           case 7: dir_move(1,80);
                   delay(700);
                   move(0,0,0);
                   break;
           case 8: dir_move(2,80);
                   delay(700);
                   move(0,0,0);
                   break;
           case 9: dir_move(3,80);
                   delay(700);
                   move(0,0,0);
                   break;
           case 10: dir_move(4,80);
                    delay(700);
                    move(0,0,0);
                    break;
           case 11: dir_move(5,80);
                    delay(700);
                    move(0,0,0);
                    break;
           case 12: dir_move(0,0);// beta 초기값
                    move(0,0,0);
                    break;                               
         } */             // 스위치 
void PROGRAM3(void)
{
  Lcd_Clear();
  Lcd_Write_String(LINE1, "TEST");
  Lcd_Write_String(LINE2, "[BALL FOLLOWER]");
  while (ENTER) ;
  int ultra_gap = 0;

  //  Shooting(500); // 0.5초간 솔레노이드 작동 시키기.

  while (!ENTER)  //  ENTER 키가 눌릴때까지 아래 명령 반복 EV3 루프와 동일
  {
      compass_move(0,0,0);
    
   /* if (ball_dir > 11) motor_stop();  // 만약 공의 방향이 11시보다 크면 공을 못찾은것임 - 정지
    else if (ball_dir == 0)           // 아니고 만약 공의 방향이 0시 방향이고
    {
      if (ultra[0] < 7)               // 만약 전방 거리 측정값이 (7 * 0.85 = 0.59) 6 cm 보다 작으면
      {
        move(100, 0, -100);           // 공이 바로 앞에 있다고 판단 전속력으로 직진
        delay(1000);                  // 1초 지속 후.
        motor_stop();                 // 정지
      }
      else move(30, 0, -30);          // 전방 거리측정값이 6cm 보다 작지 않으면 파워 30으로 전진
    }
    else if (ball_dir <= 6) move(-50, -50, -50 ); // 만약 공의 방향이 6시방향보다 작거나 같으면 우회전
    else move(50, 50, 50);                      // 아니면 좌회전
  } */
  }
}

void setup(void)
{

  init_devices();
  /*
    pixy.init();


    Wire.begin();
    // TWBR = 12;  // 400 kbit/sec I2C speed
    byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
    if (c == 0x71) // WHO_AM_I should always be 0x68
    {
      // Start by performing self test and reporting values
      myIMU.MPU9250SelfTest(myIMU.SelfTest);
      // Calibrate gyro and accelerometers, load biases in bias registers
      myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

      myIMU.initMPU9250();
      // Initialize device for active mode read of acclerometer, gyroscope, and
      // temperature
    //    Serial.println("MPU9250 initialized for active data mode....");

      // Read the WHO_AM_I register of the magnetometer, this is a good test of
      // communication
      byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

      // Get magnetometer calibration from AK8963 ROM
      myIMU.initAK8963(myIMU.magCalibration);
      // Initialize device for active mode read of magnetometer
    //    Serial.println("AK8963 initialized for active data mode....");
    } // if (c == 0x71)
    else
    {
      while(1) ; // Loop forever if communication doesn't happen
    }
  */

  Wire.begin();

  Lcd_Clear();
  Lcd_Write_String(LINE1, "CHECK YOUR GY273");
  Lcd_Write_String(LINE2, "COMPASS SENSOR!");

  // Initialise the sensor
  if (!mag.begin())
  {
    // There was a problem detecting the HMC5883 ... check your connections
    Lcd_Write_String(0, "No Compass Sensor");
    delay (2000);
  }

  /*
    //  mySensor.setWire(&Wire);

    //  mySensor.beginAccel();
    //  mySensor.beginMag();
  */
  Timer1.initialize(50);
  Timer1.attachInterrupt(Scan_Ultra); // blinkLED to run every 0.15 seconds

  read_compass();
  memComp = compass;

  Lcd_Clear();
  Lcd_Write_String(LINE1, "RCKA");
  Lcd_Write_String(LINE2, "ROBOT SOCCER 3.0");

}

void loop(void)
{

  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];

  /*
    // grab blocks!
    while(1)
    {
    blocks = pixy.getBlocks();
    Lcd_Move(0, 6);
    DigitDisplay(blocks);

    // If there are detect blocks, print them!
    if (blocks)
    {
    i++;

    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%50==0)
    {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j=0;f j<blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf);
        pixy.blocks[j].print();
    Lcd_Move(1, 6);
    DigitDisplay(pixy.blocks[j].x);


      }
    }
    }

      Lcd_Data(0xDF);
      Volt_Display(Voltage);

    }
  */

  /*
    while(1)
    {
    // If intPin goes high, all data registers have new data
    // On interrupt, check if data ready interrupt
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
      myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
      myIMU.getAres();

      // Now we'll calculate the accleration value into actual g's
      // This depends on scale being set
      myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
      myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
      myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

      myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
      myIMU.getGres();

      // Calculate the gyro value into actual degrees per second
      // This depends on scale being set
      myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
      myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
      myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

      myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
      myIMU.getMres();
      // User environmental x-axis correction in milliGauss, should be
      // automatically calculated
      myIMU.magbias[0] = +470.;
      // User environmental x-axis correction in milliGauss TODO axis??
      myIMU.magbias[1] = +120.;
      // User environmental x-axis correction in milliGauss
      myIMU.magbias[2] = +125.;

      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental
      // corrections
      // Get actual magnetometer value, this depends on scale being set
      myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
                 myIMU.magbias[0];
      myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
                 myIMU.magbias[1];
      myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
                 myIMU.magbias[2];
    } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

    // Must be called before updating quaternions!
    myIMU.updateTime();

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
    // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
    // (+ up) of accelerometer and gyro! We have to make some allowance for this
    // orientationmismatch in feeding the output to the quaternion filter. For the
    // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
    // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
    // modified to allow any convenient orientation convention. This is ok by
    // aircraft orientation standards! Pass gyro rate as rad/s
    //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
    MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                           myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                           myIMU.mx, myIMU.mz, myIMU.deltat);

    if (!AHRS)
    {
      myIMU.delt_t = millis() - myIMU.count;
      if (myIMU.delt_t > 500)
      {

        myIMU.count = millis();

      } // if (myIMU.delt_t > 500)
    } // if (!AHRS)
    else
    {
      // Serial print and/or display at 0.5 s rate independent of data rates
      myIMU.delt_t = millis() - myIMU.count;

      // update LCD once per half-second independent of read rate
      if (myIMU.delt_t > 500)
      {

    // Define output variables from updated quaternion---these are Tait-Bryan
    // angles, commonly used in aircraft orientation. In this coordinate system,
    // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
    // x-axis and Earth magnetic North (or true North if corrected for local
    // declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
    // arise from the definition of the homogeneous rotation matrix constructed
    // from quaternions. Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order which for this configuration is yaw,
    // pitch, and then roll.
    // For more see
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.
        myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                       (getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                      - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
        myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                       (getQ()+2)));
        myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                       (getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                      - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
        myIMU.pitch *= RAD_TO_DEG;
        myIMU.yaw   *= RAD_TO_DEG;
        // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
        //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
        // - http://www.ngdc.noaa.gov/geomag-web/#declination
        myIMU.yaw   -= 8.5;
        myIMU.roll  *= RAD_TO_DEG;

        myIMU.count = millis();
        myIMU.sumCount = 0;
        myIMU.sum = 0;


    float heading = atan2(myIMU.my, myIMU.mx);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.

    float declinationAngle = 0.22;
    //  heading += declinationAngle;

    // Correct for when signs are reversed.
    if(heading < 0)
      heading += 2*PI;

    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
      heading -= 2*PI;

    // Convert radians to degrees for readability.
    compass = heading * 180/M_PI;

        Lcd_Move(0, 6);
        DigitDisplay(compass);

      } // if (myIMU.delt_t > 500)
    } // if (AHRS)
        Lcd_Data(0xDF);
        Volt_Display(Voltage);

    }
  */

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
        if (menu < 0) menu = 6;
        menu_display(menu);
      }
      if (NEXT)
      {
        while (NEXT) ;
        menu++;
        if (menu > 6) menu = 0;
        menu_display(menu);
      }
      if (ENTER)
      {
        while (ENTER)  ;
        switch (menu)
        {
          case 0: ATTACKER();
            break;
          case 1: DEFENDER();
            break;
          case 2: PROGRAM3();
            break;
          case 3: view_ir();
            break;
          case 4: view_max_ir();
            break;
          case 5: view_ultra();
            break;
          case 6: view_line();
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


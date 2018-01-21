/*
 * SOCCERV3.c
 *
 * Created: 2017-010-02
 * Author: JEON HAKYEONG (전하경)
 *
 * History
 *
 * 2017-10-02
 *  Migration From AtmelStudio.
 *  
 */ 
 
//카메라 유무 설정
//#define CameraExist true     //카메라가 있음.
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
#define Line0_White 100     // 0시 방향 경계선 감지 기준 값
#define Line1_White 100     // 3시 방향 경계선 감지 기준 값
#define Line2_White 100     // 6시 방향 경계선 감지 기준 값
#define Line3_White 100     // 9시 방향 경계선 감지 기준 값

//
// 위 기준값을 넘은 경우가 있으면 아래 변수에 기록이 되어짐.
// 선을 감지하고 그에따른 액션을 수행한후에는 아래값을 Clear 시켜주세요.
//
// 0 bit :  0h direction line detected
// 1 bit :  3h direction line detected
// 2 bit :  6h direction line detected
// 3 bit :  9h direction line detected

unsigned  char LineDetected=0;    //경계라인 감지 상태 값
                                  //0 Bit : 0시방향 감지
                                  //1 Bit : 3시방향 감지
                                  //2 Bit : 6시방향 감지
                                  //3 Bit : 9시방향 감지

unsigned char Escape_Dir[16] = { 0  // 경계라인 감지 안됨 
                                ,6  // 0시 방향 감지 - 6시방향으로 탈출
                                ,9  // 3시 방향 감지 - 9시 방향으로 탈출
                                ,7  // 0시 3시 방향 감지 - 7시 방향으로 탈출
                                ,0  // 6시 방향 감지 - 0시 방향으로 탈출
                                ,3  // 0시 6시 방향 감지 - 3시 방향(또는 9시 방향)으로 탈출
                                ,10 // 3시, 6시 방향 감지 - 10시 방향으로 탈출
                                ,9  // 0,3,6시 방향 감지 - 9시 방향으로 탈출
                                ,3  // 9시 방향 감지 - 3시 방향으로 탈출
                                ,4  // 0,9시 방향 감지 - 4시 방향으로 탈출
                                ,6  // 3,9시 방향 감지 - 6시 방향(또는 0시 방향)으로 탈출
                                ,6  // 0,3,9시 방향 감지 - 6시 방향으로 탈출
                                ,2  // 6,9시 방향 감지 - 2시 방향으로 탈출
                                ,3  // 0,6,9 시 방향 감지 - 3시 방향으로 탈출
                                ,0  // 3,6,9시 방향 감지 - 0시 방향으로 탈출
                                ,0};// 모든 방향 감지됨 - 존재 할 수 없음 (F/W에서 방지 함.)
//
//Sample Numbers for Average of Analog Value. 
//
#define NumberOfSamples 4   //가능한 변경하지 마세요.. 이값을 변경하고자 하면 Header File쪽도 수정해야 함.

//평균을 구하기 위한 배열
volatile unsigned int ADC_Raw[NumberOfSamples+1][18];
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
volatile unsigned int ultra[4] = {0,0,0,0};    
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

volatile unsigned int BallCapture=0;  // 사용 안함.
#define Voltage ((int)((float)ADC_Value[16]*0.625 -2.5))

//************************************************************************************

// 메뉴 디스플레이
volatile char menu = 0;
//----------------------------------------------------------------------------
#define KP 1
//----------------------------------------------------------------------------

#include "SOCCERV3.h"
#include <TimerOne.h>

void view_line(void)
{
  Lcd_Clear();
  while(!ENTER)
  {
    Lcd_Cmd(LINE1);
    for(int i=0;i<2; i++)
    {
      Lcd_Data(i*3+'0');
      Lcd_String("h: ");
      DigitDisplay(ADC_Value[i+12]);
      Lcd_Data(' ');
    }

    Lcd_Cmd(LINE2);
    for(int i=2;i<4; i++)
    {
      Lcd_Data(i*3+'0');
      Lcd_String("h: ");
      DigitDisplay(ADC_Value[i+12]);
      Lcd_Data(' ');
    }
    delay(200);
  }
  while(ENTER) ;
}

void view_ir(void)
{
  int submenu=0;
 
  Lcd_Clear();
  while(!ENTER)
  {
    if(PREV)
    {
      while(PREV) ;
      submenu-=3;
      if (submenu<0) submenu=9;
    }
    else if(NEXT)
    {
      while(NEXT) ;
      submenu+=3;
      if (submenu>9) submenu=0;
    }

    Lcd_Cmd(LINE1);
    Hour_Display(submenu);
  
    for(int i=0;i<3; i++)
    {
      DigitDisplay(ADC_Value[i+submenu]);
      Lcd_Data(' ');
    }

    Lcd_Cmd(LINE2);
    int tmp=submenu+3;
    if(tmp > 9) tmp =0;
    Hour_Display(tmp);
    
    for(int i=0;i<3; i++)
    {
      DigitDisplay(ADC_Value[i+tmp]);
      Lcd_Data(' ');
    }
    delay(100);
  }
  while(ENTER) ;
}

void view_capture(void)
{
  Lcd_Clear();
  Lcd_Write_String(LINE1,"CAPTURE IR");
  while(!ENTER)
  {
    Lcd_Cmd(LINE2);
    DigitDisplay(ADC_Value[17]);
    delay(200);
  }
  while(ENTER) ;
}

void view_ultra(void)
{
  Lcd_Clear();
  while(!ENTER)
  {
    Lcd_Cmd(LINE1);
    for(int i=0;i<2; i++)
    {
      Lcd_Data(i*3+'0');
      Lcd_String("h: ");
      DigitDisplay((float)ultra[i]*0.85);
      Lcd_Data(' ');
    }

    Lcd_Cmd(LINE2);
    for(int i=2;i<4; i++)
    {
      Lcd_Data(i*3+'0');
      Lcd_String("h: ");
      DigitDisplay(ultra[i]*0.85);
      Lcd_Data(' ');
    }
    delay(200);
  }
  while(ENTER) ;
}

void menu_display(unsigned char no)
{
  switch(no)
  {
    case 0: Lcd_Write_String(LINE2,"RUN PROGRAM 1   ");
        break;
    case 1: Lcd_Write_String(LINE2,"RUN PROGRAM 2   ");
        break;
    case 2: Lcd_Write_String(LINE2,"[BALL FOLLOWER] ");
        break;
    case 3: Lcd_Write_String(LINE2,"[ GOAL FINDER ] ");
        break;
    case 4: Lcd_Write_String(LINE2,"VIEW IR         ");
        break;
    case 5: Lcd_Write_String(LINE2,"VIEW CAPTURE IR ");
        break;
    case 6: Lcd_Write_String(LINE2,"VIEW ULTRA      ");
        break;
    case 7: Lcd_Write_String(LINE2,"VIEW LINE       ");
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
  if (comp > 100)
  {
    move(50, 50, 50);
  }
  else if(comp < -100)
  {
    move(-50, -50, -50);
  }
  else
  {
    move(ma+comp*KP, mb+comp*KP, mc+comp*KP);
  }
}

void dir_move(int input_ball, int power)
{
  switch(input_ball)
  {
    case 0:
      compass_move(power, 0, -power);
      break;
    case 1:
      compass_move(power/2, power/2, -power);
      break;
    case 2:
      compass_move(0, power, -power);
      break;
    case 3:
      compass_move(-power/2, power, -power/2);
      break;
    case 4:
      compass_move(-power, power, 0);
      break;
    case 5:
      compass_move(-power, power/2, power/2);
      break;
    case 6:
      compass_move(-power, 0, power);
      break;
    case 7:
      compass_move(-power/2, -power/2, power);
      break;
    case 8:
      compass_move(0, -power, power);
      break;
    case 9:
      compass_move(power/2,- power, power/2);
      break;
    case 10:
      compass_move(power, -power, 0);
      break;
    case 11:
      compass_move(power, -power/2, -power/2);
      break;
  }
}

void ball_near(int dir, int power)
{
  switch(dir)
  {
    case 0:
      dir_move(0, power);
      break;
    case 1:
      dir_move(2, power);
      break;
    case 2:
      dir_move(5, power);
      break;
    case 3:
      dir_move(6, power);
      break;
    case 4:
      dir_move(7, power);
      break;
    case 5:
      dir_move(8, power);
      break;
    case 6:
      dir_move(8, power);
      break;
    case 7:
      dir_move(4, power);
      break;
    case 8:
      dir_move(5, power);
      break;
    case 9:
      dir_move(6, power);
      break;
    case 10:
      dir_move(7, power);
      break;
    case 11:
      dir_move(10, power);
      break;
  }
}

void PROGRAM1(void)//공격 
{
  int ultra_gap = 0;
  
  move(0,0,0);
  Lcd_Clear();
  Lcd_Write_String(LINE1,"RUNNING PROGRAM1");  
  
  LineDetected = 0;

  while(1)
  {
    if(ball_dir > 6)            //이전에 공이 왼쪽에 있었음.
      last_pos = 1;
    else if(ball_dir >= 1 && ball_dir < 6)  //이전에 공이 오른쪽에 있었음.
      last_pos = 0;

    if(LineDetected)    //경계라인이 감지되었으면 탈출 함.
    {
      move(0,0,0);
      delay(100);

      dir_move(Escape_Dir[LineDetected], 100);
      delay(400);

      move(0,0,0);

      LineDetected = 0;
    }

    if(ball_dir<12)
    {
      if(max_ir > SHOOT && (ball_dir == 0)) // || ball_dir == 1 || ball_dir == 11))     //공이 가까이 있을 때
      {
        ultra_gap = (int)((float)ultra[1]*0.85) - (int)((float)ultra[3]*0.85);
        int comp;
        read_compass();
        comp = compass;
        comp = comp - 180 - ultra_gap;
        
        move(100+comp*KP, comp*KP, -100+comp*KP);
        
        dir_move(0, 100);
        compass_move(100, -ultra_gap*2, -100);
      }
      else
      {
        if (max_ir > NEAR)  ball_near(ball_dir, 100);
        else                dir_move(ball_dir, 100);
      }
    }
    else dir_move(ball_dir, 0);
  }
}

void PROGRAM2(void)
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
    if(ball_dir > 6)
      last_pos = 1;
    else if(ball_dir >= 1 && ball_dir < 6)
      last_pos = 0;
      
    if((int)((float)ultra[2]*0.85) < 30)
      dir_move(0, 50);
    else if((float)ultra[1]*0.85 < 15 && max_ir < 80)
    {
      move(0,0,0);
      delay(100);
      dir_move(9,100);
      delay(700);
      LineDetected=0;
    }
    else if((float)ultra[3]*0.85 < 15 && max_ir < 80)
    {
      move(0,0,0);
      delay(100);
      dir_move(3,100);
      delay(700);  
      LineDetected=0;   
    }
    else if(LineDetected)
    {
      move(0,0,0);
      delay(100);

      dir_move(Escape_Dir[LineDetected], 100);
      delay(400);

      LineDetected = 0;
    }
    else
    {
      if(max_ir > SHOOT2 && (ball_dir == 0)) // || ball_dir == 1 || ball_dir == 11)     //공이 가까이 있을 때
      {
        ultra_gap = (int)((float)ultra[1]*0.85) - (int)((float)ultra[3]*0.85);
        int comp;
        read_compass();
        comp = (int)compass / 10;
        comp = comp - 180 - ultra_gap;
          
        move(100+comp*KP, comp*KP, -100+comp*KP);
          
        dir_move(0, 100);
        for (int i=0;i<100;i++)
        {
          compass_move(100, -ultra_gap*2, -100);
        }
        move(0,0,0);
        delay(100);
        dir_move(6,100);
        delay(200);
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

void PROGRAM3(void)
{
  Lcd_Clear();
  Lcd_Write_String(LINE1,"TEST");  
  Lcd_Write_String(LINE2,"[BALL FOLLOWER]");  
  while(ENTER) ;

//  Shooting(500); // 0.5초간 솔레노이드 작동 시키기.
  
  while(!ENTER)   //  ENTER 키가 눌릴때까지 아래 명령 반복 EV3 루프와 동일
  {
    if(ball_dir > 11) motor_stop();   // 만약 공의 방향이 11시보다 크면 공을 못찾은것임 - 정지
    else if(ball_dir == 0)            // 아니고 만약 공의 방향이 0시 방향이고
    { 
      if(ultra[0] < 7)                // 만약 전방 거리 측정값이 (7 * 0.85 = 0.59) 6 cm 보다 작으면
      {
        move(100,0,-100);             // 공이 바로 앞에 있다고 판단 전속력으로 직진
        delay(1000);                  // 1초 지속 후.
        motor_stop();                 // 정지
      }
      else move(30,0,-30);            // 전방 거리측정값이 6cm 보다 작지 않으면 파워 30으로 전진
    }
    else if (ball_dir <= 6) move(-50,-50,-50 ); // 만약 공의 방향이 6시방향보다 작거나 같으면 우회전
    else move(50,50,50);                        // 아니면 좌회전
  }
  while(ENTER) ;
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
  Lcd_Write_String(LINE1,"TEST X=         ");  
  Lcd_Write_String(LINE2," [ GOAL FINDER ]");  
  while(ENTER) ;

  while(!ENTER)   //  ENTER 키가 눌릴때까지 아래 명령 반복 EV3 루프와 동일
  {
    Lcd_Move(0, 8);
    
    if(CameraExist)
    {
      blocks = pixy.getBlocks();
      if(blocks)
      {
          Lcd_Move(0, 8);
          DigitDisplay(pixy.blocks[0].x);      

          if(pixy.blocks[0].x > 175)        move(-50,-50,-50 );   // 만약 골대가 오른쪽에 있으면 우회전
          else if(pixy.blocks[0].x < 145)   move(50,50,50 );      // 만약 골대가 왼쪽에 있으면 좌회전
          else                              motor_stop();        // 골대가 중앙에 있으면 정지
      }
      else
      {
          Lcd_String("???");
          motor_stop();
      }
      delay(100);
    }
    else  Lcd_String("NO CAM");
  }
  while(ENTER) ;
  motor_stop();
  
}

void setup(void) 
{

  init_devices();

  Wire.begin();

  //FIND COMPASS SENSOR
  if(!Check_Compass())
  {
    Lcd_Clear();
    Lcd_Write_String(LINE1,"CHECK YOUR GY273");
    Lcd_Write_String(LINE2,"COMPASS SENSOR!");
    while(1) ;
  }
  
  Timer1.initialize(50);
  Timer1.attachInterrupt(Scan_Ultra); // blinkLED to run every 0.15 seconds

  read_compass();
  delay(100);
  read_compass();
  memComp = compass;
  
  Lcd_Clear();
  Lcd_Write_String(LINE1,"RCKA");
  Lcd_Write_String(LINE2,"ROBOT SOCCER 3.0");

  if(CameraExist) pixy.init();
  
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

  if(ENTER)
  {
    while(ENTER) ;
    menu_display(menu);
    while(1)
    {
      read_compass();
      Lcd_Move(0, 6);
      DigitDisplay(compass);
      Lcd_Data(0xDF);
      Volt_Display(Voltage);
      
      if(PREV)
      {
        while(PREV) ;
        menu--;
        if (menu<0) menu=7;
        menu_display(menu);
      }
      if(NEXT)
      {
        while(NEXT) ;
        menu++;
        if (menu>7) menu=0;
        menu_display(menu);
      }
      if(ENTER)
      {
        while(ENTER)  ;
        switch(menu)
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
        }

        Lcd_Clear();
        Lcd_Write_String(LINE1,"RCKA");
        menu_display(menu);
      }
      delay(200);
    }
  }
  delay(200);
}


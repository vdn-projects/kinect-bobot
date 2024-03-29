#include <18F4550.h>
#include <STDDEF.H>
#FUSES NOWDT, HS, NOPUT, NOBROWNOUT, NOLVP, NOCPD, NOWRT, NODEBUG

#use delay(clock=12000000)
#use rs232 (baud=19200 , parity=n , xmit=pin_C6 , rcv=pin_C7 ,bits=8)
#byte PORTA =  0xf80
#byte PORTB =  0xf81
#byte PORTC =  0xf82
#byte PORTD =  0xf83
#byte PORTE =  0xf84

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

#BIT DIR_RIGHT = PORTE.1   //0: forward, 1: backward
#BIT DIR_LEFT  = PORTE.2

#define time_step 50535  // 0.005 sencond  5535 //
#define PERIMETER_LEFT 31.95
#define PERIMETER_RIGHT 31.9
#define RADIUS 20.25
#define THRES_RIGHT 55 //Choose suitable Threshold to synchronize 
#define THRES_LEFT 65  //two wheels speed
#define ERR 4
//Global variants
signed int32 num_Pulse_Right = 0, count_left = 0;
signed int32 num_Pulse_Left = 0;
float disTemp = 0;
int1 done_Timer1 = 0;

char  ss;
char data_receive[50] = "";
int8 index = 0;

signed int16 pw_duty_Left = 0, pw_duty_Right = 0;
signed int32 position_Right = 0, position_Left = 0, position_set_Left = 0, position_set_Right = 0;

float Kp_r, Kd_r, Ki_r;
float Kp_l, Kd_l, Ki_l;
signed int32 e_sum_r, e_del_r, e1_r, e2_r;
signed int32 e_sum_l, e_del_l, e1_l, e2_l;

float value_cmd, anpha = 0, angle_current = 90.0, distance_move = 0;
float x_cur = 0, y_cur = 0, x_cur_old = 0, y_cur_old = 0;
char  direction_cmd, direction_cmd_old;
int1 RobotFlag = 0, RunFlag = 0, pid_Left_Flag = 0, pid_Right_Flag = 0, donePID = 0, targetFlag = 0;

void pid_Robot();
void pid_Right();
void pid_Left();
void init_pid_Right();
void init_pid_Left();
void moveRobot(char direction, float value);
void xy_calculator(float distance);

#INT_RDA
void RDA_isr(){
char value[20];
ss = getc();
if(ss == '*')  //Beginning symbol
   {
      index = 0;
      *data_receive = "";
      RunFlag = 1;
   }
else if(ss == '#') //Terminal symbol
   {
    data_receive[index++]='\0';
    direction_cmd = data_receive[0];
    if ((direction_cmd != 'T') && direction_cmd_old == 'T' && (!donePID)) targetFlag = 1;
    direction_cmd_old = direction_cmd;
    strncpy(value, data_receive + 1, strlen(data_receive) - 1);
    value_cmd = atof(value);
    RobotFlag = 1;
   }

else data_receive[index++] = ss; 

}

       
#INT_EXT1 //Count pulses in Right wheel
void EXT1_isr()
{
disable_interrupts(int_ext1);
if(input(PIN_D1) == 1)
   num_Pulse_Right++;
else num_Pulse_Right--;
   position_Right =  num_Pulse_Right;//*360/300; 
   
enable_interrupts(int_ext1) ;
}

#INT_EXT2 //Count pulses in Right wheel
void EXT2_isr()
{
disable_interrupts(int_ext2);
   if(input(PIN_D2) == 1) 
   {   
      num_Pulse_Left++;
      if(direction_cmd == 'T') count_left++;
   }   
   else 
   {
      num_Pulse_Left--;
      if(direction_cmd == 'T') count_left--;
   }   

   position_Left =  num_Pulse_Left;//*360/300; 
  
enable_interrupts(int_ext2) ;

}


#INT_TIMER1  //Count number of pulses per time unit 
void TIMER1_isr() {
//printf(lcd_putc "\ncount %ld", ++count); 
// disable_interrupts(INT_TIMER1); 
 done_Timer1 = 1;
 set_timer1(time_step);
// enable_interrupts(INT_TIMER1) ;

}  

void main()
{
 set_tris_a(0);
 set_tris_b(0xFF);
 set_tris_e(0);
 set_tris_c(0x80);
 set_tris_d(0x0F);
 DIR_RIGHT = 0;  //right wheel forward
 DIR_LEFT = 0;
 /*
 lcd_init();
 lcd_putc("Test");
 lcd_gotoxy(1,2);
 lcd_putc("Dong Co");
 delay_ms(2000); */
    
 setup_adc_ports(NO_ANALOGS);
 setup_timer_1(T1_INTERNAL | T1_DIV_BY_1 );
 set_timer1(time_step);  
 setup_timer_2(T2_DIV_BY_4,255,1);  //f_pwm = 2.9 Khz, T_pwm = 4.(PR2+1).Tosc.Pre-scale
 output_low(PIN_C1); // Set CCP2 output low 
 output_low(PIN_C2); // Set CCP1 output low 

 setup_ccp1(CCP_PWM);    //khoi tao bo PWM1
 setup_ccp2(CCP_PWM);    //khoi tao bo PWM2
 set_pwm1_duty(0);      //PIN_C2, Right motor
 set_pwm2_duty(0);      //PIN_C1, Left motor
 
 enable_interrupts(INT_RDA); 
 enable_interrupts(int_ext1);
 enable_interrupts(int_ext2);
 ext_int_edge(1, H_TO_L );
 ext_int_edge(2, H_TO_L );
 enable_interrupts(INT_TIMER1);   
 enable_interrupts(GLOBAL);   
     
 printf("X%.3fY%.3fA%.3fN", x_cur, y_cur, angle_current);
 while(true)
 {
  if(done_Timer1)
   {
      moveRobot(direction_cmd, value_cmd);
      pid_Robot();
      done_Timer1 = 0;
    }
  }
}
void xy_calculator(float distance)
{
   x_cur = x_cur_old + distance*cos(angle_current*PI/180.0);
   y_cur = y_cur_old + distance*sin(angle_current*PI/180.0);
   x_cur_old = x_cur;
   y_cur_old = y_cur;      
}

void pid_Robot()
{
    if(targetFlag)
   {
    distance_move = PERIMETER_LEFT*count_left/300.0;
    xy_calculator(distance_move);
    count_left = 0;
    distance_move = 0;
    printf("X%.3fY%.3fA%.3fT", x_cur, y_cur, angle_current);
    targetFlag = 0;
   } 
   if(direction_cmd == 'T' ) // send distance continuously
   {
      disTemp = PERIMETER_LEFT*count_left/300.0;
      printf("X%.3fD", disTemp);
   }
   
    pid_Right();
    pid_Left();

 if ((!pid_Left_Flag) && (!pid_Right_Flag) && (!donePID))
   {
      if(direction_cmd == 'R') 
       {
        angle_current  -= value_cmd;
        direction_cmd = "";
       }
     else if(direction_cmd == 'L')
      {
      angle_current  += value_cmd;
      direction_cmd = "";
      }
      else if (direction_cmd == 'F' || direction_cmd == 'T')
       {
         xy_calculator(value_cmd);
         direction_cmd = "";
       }
     else if (direction_cmd == 'B')
      {
         xy_calculator(-value_cmd);
         direction_cmd = "";
      }


    if (angle_current >= 360.0) angle_current -= 360.0;
    if (angle_current < 0 ) angle_current += 360.0;
         
   printf("X%.3fY%.3fA%.3fN", x_cur, y_cur, angle_current);
    donePID = 1;   
   } 
}
void init_pid_Right()
{
Kp_r = 2.5;
ki_r = 0.0000001;
kd_r = 2.0;
e_sum_r = 0;
e_del_r = 0;
e1_r = 0;
e2_r = 0;
pw_duty_Right = 0;
}

void init_pid_Left()
{
Kp_l = 2.5;  //0.9
ki_l = 0.0000001;
kd_l =2.0;      //1
e_sum_l = 0;
e_del_l = 0;
e1_l = 0;
e2_l = 0;
pw_duty_Left = 0;
}

void pid_Right()
{
  signed int16 temp_kp = 0;
  signed int16 temp_ki = 0;
  signed int16 temp_kd = 0;
  signed int16 pw_duty_Right_Temp = 0;

  e2_r = position_set_Right - position_Right;
  e_sum_r += e2_r;
  e_del_r = e2_r - e1_r;
  e1_r = e2_r;

  temp_kp = (signed int16)((float)kp_r*e2_r);
  temp_ki = (signed int16)((float)ki_r*e_sum_r);
  temp_kd = (signed int16)((float)kd_r*e_del_r);
  
  pw_duty_Right_Temp = (signed int16)( temp_kp + temp_ki + temp_kd);

  if (pw_duty_Right_Temp > THRES_RIGHT) pw_duty_Right_Temp = THRES_RIGHT;
  if (pw_duty_Right_Temp < -THRES_RIGHT) pw_duty_Right_Temp = -THRES_RIGHT; 
  
  if (pw_duty_Right_Temp < 0)
    {
     DIR_RIGHT = 1;
     pw_duty_Right = - pw_duty_Right_Temp;
    }
  else
    {
      DIR_RIGHT = 0;
      pw_duty_Right = pw_duty_Right_Temp;
    }
  set_pwm1_duty((int16)pw_duty_Right); 
   if(e2_r > -ERR && e2_r < ERR )
   {
      pid_Right_Flag = 0;
   }
   else 
    {
      pid_Right_Flag = 1;
    }      

}

void pid_Left()
{
  signed int16 temp_kp = 0;
  signed int16 temp_ki = 0;
  signed int16 temp_kd = 0;
  signed int16 pw_duty_Left_Temp = 0;

  e2_l = position_set_Left - position_Left;
  e_sum_l += e2_l;
  e_del_l = e2_l - e1_l;
  e1_l = e2_l;

  temp_kp = (signed int16)((float)kp_l*e2_l);
  temp_ki = (signed int16)((float)ki_l*e_sum_l);
  temp_kd = (signed int16)((float)kd_l*e_del_l);
  
  pw_duty_Left_Temp = (signed int16)( temp_kp + temp_ki + temp_kd);
  if (pw_duty_Left_Temp > THRES_LEFT)   pw_duty_Left_Temp = THRES_LEFT;
  if (pw_duty_Left_Temp < -THRES_LEFT)  pw_duty_Left_Temp = -THRES_LEFT; 
  
  if (pw_duty_Left_Temp < 0)
    {
      DIR_LEFT = 1;
      pw_duty_Left = - pw_duty_Left_Temp;
    }
  else
    {
      DIR_LEFT = 0;
      pw_duty_Left = pw_duty_Left_Temp;
    }
   set_pwm2_duty((int16)pw_duty_Left); 
   if(e2_l > -ERR && e2_l < ERR)
   {
      pid_Left_Flag = 0;
   }
   else 
    {
      pid_Left_Flag = 1;
    }
}


void moveRobot(char direction, float value)
{
   if(RobotFlag)
   {
   init_pid_Right();
   init_pid_Left();
   pid_Left_Flag = 1;
   pid_Right_Flag = 1;
   donePID = 0;
   
    num_Pulse_Right = 0;
   num_Pulse_Left = 0;
   position_Right = 0;
   position_Left = 0;
   RobotFlag = 0;
   switch(direction)
   {
     case 'T':
      position_set_Left  = (unsigned int32)(300.0*value/PERIMETER_LEFT);
      position_set_Right = (unsigned int32)(300.0*value/PERIMETER_RIGHT);   
      distance_move = 0;
      count_left = 0;

      break;      
      case 'F':
        position_set_Left  = (unsigned int32)(300.0*value/PERIMETER_LEFT);
        position_set_Right = (unsigned int32)(300.0*value/PERIMETER_RIGHT);    
        break;
      case  'B':
        position_set_Left  =  (unsigned int32)(-300.0*value/PERIMETER_LEFT);
        position_set_Right =  (unsigned int32)(-300.0*value/PERIMETER_RIGHT);
        break;
      case 'R':
        anpha = - value*PI/180.0;
        position_set_Left  =  (unsigned int32)(-300.0*(anpha*RADIUS)/PERIMETER_LEFT);
        position_set_Right =  (unsigned int32)(300.0*(anpha*RADIUS)/PERIMETER_RIGHT);    
        break;
      case 'L':
        anpha = value*PI/180.0;     
        position_set_Left  =  (unsigned int32)(-300.0*(anpha*RADIUS)/PERIMETER_LEFT);
        position_set_Right =  (unsigned int32)(300.0*(anpha*RADIUS)/PERIMETER_RIGHT);
        break;         
      default:  break;
   }
   } 
}

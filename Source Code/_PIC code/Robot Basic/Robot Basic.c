#include <18f4550.h>

#FUSES HS //High speed Osc (> 4mhz)
#FUSES PUT //Power Up Timer
#FUSES NOPROTECT //Code not protected from reading
#FUSES NOBROWNOUT //No brownout reset
#FUSES NOLVP //Low Voltage Programming on B3(PIC16) or B5(PIC18)
#FUSES NOCPD //No EE protection
#FUSES NODEBUG //No Debug mode for ICD
#FUSES NOWDT

#use delay(clock=12000000)
#use rs232 (baud=19200 , parity=n , xmit=pin_C6 , rcv=pin_C7 ,bits=8)

#byte PORTA =  0xf80
#byte PORTB =  0xf81
#byte PORTC =  0xf82
#byte PORTD =  0xf83
#byte PORTE =  0xf84 

#use fast_io(b)
#use fast_io(c)
#use fast_io(a)
#use fast_io(d)
#use fast_io(e)

char ss;

/**************************************/
#INT_RDA                                                         
void RDA_isr()
{
ss = getc();
//clear_interrupt(INT_RDA);
}
#BIT DIR_RIGHT = PORTE.1   //0: , 1:
#BIT DIR_LEFT  = PORTE.2

void robot_forward();
void robot_left();
void robot_right();
void robot_backward();
void robot_stop();
/**************************************/
void main(){

       set_tris_D(0xFF);
       set_tris_A(0);
       set_tris_B(0xFF);
       set_tris_C(0b10000000);
       set_tris_E(0);       

setup_adc_ports(NO_ANALOGS);
enable_interrupts (INT_RDA);   
enable_interrupts(GLOBAL);


//Setup for motor control

/*--------------------------------------------------------------------
 setup_timer_2 (mode, period, postscale)
 period:  a int 0-255 that determines when the clock value is reset ( period = PR2)
  f_pwm = 1/([period + 1] • 4 • TOSC •(Prescale Value))
  f_pwm = 1/([149 + 1]• 4 • (12e6)^-1 • 4) = 5khz
--------------------------------------------------------------------- */
setup_timer_2(T2_DIV_BY_4,255,1);  //setup_timer_2(T2_DIV_BY_4,149,1);  //f = 7.5 Khz, 30 Khz, 2.929 Kz, 732.42 hz
output_low(PIN_C1); // Set CCP2 output low 
output_low(PIN_C2); // Set CCP1 output low 

setup_ccp1(CCP_PWM);    //khoi tao bo PWM1
setup_ccp2(CCP_PWM);    //khoi tao bo PWM2
set_pwm1_duty(0);
set_pwm2_duty(0);
ss = "";

while(true)
{
   if(ss == 'F') robot_forward();
   if(ss == 'B') robot_backward();
   if(ss == 'R') robot_right();
   if(ss == 'L') robot_left();
   if(ss == 'S') robot_stop();
}

}

void robot_forward()
{  
//Left wheel
   DIR_LEFT = 0;                  
   set_pwm1_duty(30L); 

//Right whee3
   DIR_RIGHT = 0;
   set_pwm2_duty(30L); 

//   delay_us(1000);
}

void robot_left()
{
//Left wheel
   DIR_LEFT = 1;            
   set_pwm1_duty(30L); 

//Right wheel
   DIR_RIGHT = 0;               
   set_pwm2_duty(30L); 

}

void robot_right()
{
//Left wheel
   DIR_LEFT = 0;                    
   set_pwm1_duty(30L); 

//Right wheel
   DIR_RIGHT = 1;                      
   set_pwm2_duty(30L); 
}

void robot_backward()
{
//Left wheel
   DIR_LEFT = 1;                      
   set_pwm1_duty(30L); 

//Right wheel
   DIR_RIGHT = 1;                    
   set_pwm2_duty(30L); 
}

void robot_stop()
{
   set_pwm1_duty(0);
   set_pwm2_duty(0);
}


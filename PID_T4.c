#include <30F2010.h>
#device adc=10
#FUSES NOWDT                    //No Watch Dog Timer
#FUSES HS                       //High speed Osc (> 4mhz for PCM/PCH) (>10mhz for PCD)
#FUSES PR                       //Primary Oscillator
#FUSES NOCKSFSM                 //Clock Switching is disabled, fail Safe clock monitor is disabled
#FUSES WPSB16                   //Watch Dog Timer PreScalar B 1:16
#FUSES WPSA512                  //Watch Dog Timer PreScalar A 1:512
#FUSES PUT64                    //Power On Reset Timer value 64ms
#FUSES NOBROWNOUT               //No brownout reset
//#FUSES BORV47                   //Brownout reset at 4.7V
#FUSES LPOL_HIGH                //Low-Side Transistors Polarity is Active-High (PWM 0,2,4 and 6)
   //PWM module low side output pins have active high output polar
#FUSES HPOL_HIGH                //High-Side Transistors Polarity is Active-High (PWM 1,3,5 and 7)
   //PWM module high side output pins have active high output polarity
#FUSES NOPWMPIN                 //PWM outputs drive active state upon Reset
#FUSES MCLR                     //Master Clear pin enabled
#FUSES NOPROTECT                //Code not protected from reading
#FUSES NOWRT                    //Program memory not write protected
#FUSES NODEBUG                  //No Debug mode for ICD
//#FUSES NOCOE                    //Device will reset into operational mode
//#FUSES ICS0                     //ICD communication channel 0
#FUSES RESERVED                 //Used to set the reserved FUSE bits

#use delay(clock=20000000)
#include <PID.h>
#include <math.h>
#include <internal_eeprom.c>
//The cycle time will be (1/clock)*PRx prescale*(period+1)

#use rs232(UART1,baud=19200,parity=N,bits=8)
#priority TIMER1,RDA

int16 duty1;
int16 loop;
void call_pid(void);
 int8 data_com[3];
 int8 comm1,comm2,comm3;
int1 flg_rda;
int8 n;
 int16 p_gain;
 int16 i_gain;
 int16 d_gain;
int16 read_eep;
void clear_eeprom(void);
void write_eep(void);
void read_eepr(void);

#int_RDA
void  RDA_isr(void) 
{ disable_interrupts(INT_TIMER1);

   data_com[n]=GETC();
   n++;
   if(n>1) 
   {
     FLG_RDA=1;enable_interrupts(INT_TIMER1); 
   }
   
}

#int_TIMER1
void  TIMER1_isr(void) 
{
  
//  set_timer1(0);
//  loop++;
 // if(loop>5)
 // {
 // OUTPUT_TOGGLE(PIN_D1);loop=0;
 // call_pid();
 // }//
}




void main()
{  
    SETUP_ADC_PORTS(sAN0|sAN1|VSS_VDD );
    SETUP_ADC(ADC_CLOCK_INTERNAL);
    //setup_timer1(TMR_INTERNAL|TMR_DIV_BY_1);
    
    setup_compare(1,COMPARE_PWM | COMPARE_TIMER3 );
    setup_timer3(TMR_INTERNAL | TMR_DIV_BY_1,1000 );

    setup_spi(SPI_SS_DISABLED);
    
   enable_interrupts(INT_RDA);
   disable_interrupts(INT_TIMER1);
   enable_interrupts(INTR_GLOBAL);

   setup_wdt(WDT_OFF);
   setup_timer1(TMR_INTERNAL|TMR_DIV_BY_1);
   
   set_pwm_duty(1,400);// pwm output on pin RD0 (pin number 15)
   
 // check data P-GAIN  D- GAIN AND I - GAIN  EEprom   
     read_eep=read_int16_eeprom(6);
    if(read_eep==2510)
    {
     p_gain = read_eep=read_int16_eeprom(0);
     i_gain = read_eep=read_int16_eeprom(2);
     d_gain = read_eep=read_int16_eeprom(4);
 
  
    }
    else
    {
    clear_eeprom();
    }  
   

   
  plantPID.iGain =150;
  plantPID.pGain =2;
  plantPID.dGain=4;
  plantPID.iMax=5;
  plantPID.iMin=-5; 
  
  
  flg_rda = 0;
  n=0;loop=0;
   while(true)
   {  set_adc_channel(0);
      delay_us(10);
      plantCommand = read_adc();
      
      set_adc_channel(1);
      delay_us(10);
      position = read_adc();

      
      if(flg_rda)
      {
       write_eeprom(6,2510);
       write_eep();
      }
      
     if(LOOP>5000)
     {
        OUTPUT_TOGGLE(PIN_D1);loop=0;
        call_pid();loop=0;
     }
     
      
      LOOP++;
 
   }

}

void call_pid(void)
{
drive = UpdatePID(&plantPID,(plantCommand - position), position );
IF(drive> 1000) drive =1000;
if(drive< -1000) drive = -1000;
if(drive < 0)//drive>0 && drive < 1000
{
drive=abs(drive);
duty1=drive;

  if(duty1>1000)duty1=1000;
  set_pwm_duty(1,duty1);

}

else if(drive>0 && drive <1000)//drive < 0
{

  duty1=drive;

  
 if(duty1<=1000)
 {
  duty1=1000-duty1;
  set_pwm_duty(1,duty1);
 }  
  
}
else
{

}

printf("%04LU%04lu%04lu%04LU%04lu%04lu",plantCommand,position,duty1,p_gain,i_gain,d_gain);
//printf("%04U%04u%04u",comm1,comm2,comm3);
printf("s");
//enable_interrupts(INT_RDA);
}


void write_eep(void)
{  
comm1=data_com[0];
comm2=data_com[1];
comm3=data_com[2];


switch (comm3) {

    case 1:
           if(comm2==1)
           {
             p_gain=p_gain+comm1;
             write_eeprom(0,p_gain); 
             read_eepr(); 
           }
           else
           {
             if(comm1<p_gain)
             {
             p_gain=p_gain-comm1;
             write_eeprom(0,p_gain);
             read_eepr();
             }
           }

           break;

    case 2:
           if(comm2==1)
           {
             i_gain=i_gain+comm1;
             write_eeprom(2,i_gain); 
             read_eepr();
           }
           else
           {
             if(comm1<i_gain)
             {
             i_gain=i_gain-comm1;
             write_eeprom(2,i_gain);
             read_eepr();
             }
           }
           break;
    case 3:
           if(comm2==1)
           {
             d_gain=d_gain+comm1;
             write_eeprom(4,d_gain); 
             read_eepr();
           }
           else
           {
             if(comm1<d_gain)
             {
             d_gain=d_gain-comm1;
             write_eeprom(4,d_gain);
             read_eepr();
             }
           }
           break;   
           
   default:

            break; }


   flg_rda=0;N=0;
}           

void clear_eeprom(void)
{p_gain=2; i_gain =150;  d_gain = 4;
 
write_eeprom(0,p_gain); 
write_eeprom(2,i_gain); 
write_eeprom(4,d_gain); 

}
void read_eepr(void)
{
     p_gain= read_int16_eeprom(0);
     i_gain= read_int16_eeprom(2);
     d_gain= read_int16_eeprom(4);
     
  plantPID.pGain = p_gain;
  plantPID.iGain = i_gain;  
  plantPID.dGain = d_gain;
}

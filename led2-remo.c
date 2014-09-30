/*
 * File:   main.c
 * Author: wj
 *
 * Created on 2014 6 24
 *
 * MPLAB XC8 Compiler v1.32 install, but when installing, warning/error occured
 * MPLAB X IDE v2.10 download & install
 *
 *
 *
 * 
 * picket3 + mplabx

 * MPLAB X IDE v2.15 upgrade
 * plugins MPLAB code configurator /Graphics Display Designer X
 */


/*
    PIC16F1713 ssop28

    // adc up to 28 external channels
    ra0(27) : adc channel 0 input
    ra1 : adc 1
    
    ra2(4) : dac1out1 -use
    ra5(7) : dac2out1

    ra6 : test output
    
    rc6(14) : tx/ck
    rc7(18) : rx

    100usec timer for ir remocon 

*/

/*
    uart 9600 N 8 1 format
    
    start bit :0 
    stop bit  :1 

    how to synchronize start bit in frame?
    how can it know where the start bit is?
    what do we need to know?

    -> uart uses a 16x internal clock to sample the start bit.
        the incomming data is continuously sampled until a failling edge is detected.
        once detected, the receiver wait 6 clocks to begine sampling.
        if start start bit is validated, begin sampling 16 clocks from center of start bit.

    -> in receiver side, internal async clock is used. 
        this asynchronous clock reload at falling edge of start bit.        


    Designe rs232 transmitter 115200n81 format
        building timer ticks at the baud rat(8.68usec)
        build a 9bit shift register with right-most bit connected to tx output
            shift right ten times at each tick
            when done shifting , ready to send another character
        piece of cake, 15 minutes task in HDL

    Design rs232 receiver
        you must have a tick generator that can be restarted on demand and generating ticks at half the bit period.
        and 9-bit shift register
            keep tick generator at zero, wating until the rx line input goes low(beginning of start bit)
            loop 10 times:
                wait for tick,right shift in the data line(rx)
                if reached last bit(stop bit) then exit the loop
            endof loop : register msb must be 1(stop bit), else you have a problem
                    
    
    
            
        
    0x55('U') : 
    0xff : all the low bit are genuine start bit

    115200baud = 
    9600baud = 104 usec
    4800baud = 208 usec
    2400baud = 416 usec



*/





 
/*
 First check!     Target board working/debuging ? as RA3 setting
 Do not use debug mode. just program and then run so that use in-circuit debugger pin

 * if MCLRE =1 &&  LVP=0 , !MCLR  enabled
 * if LVP=1 , !MCLR is enabled
 *
 */

/*
    First headache pb: connection problem 
    hold the button on the pICKIT 3 for 30 seconds without power then connect while still holding the button. 
    the PICKIT should be re-programmed when switching from PIC18 to PIC16 visa versa.
    http://www.microchip.com/forums/m784148.aspx#784148

    for sleep mode
    http://www.gooligum.com.au/tutorials/midrange/PIC_Mid_C_4.pdf

*/

/*
Decoding NEC Infrared Ray remote  Protocol

Leader code : 9m on + 4.5m off + custom code +!custom code + datacode +!datacode

Normal Leader code (13.5m)
Repeat Leader code (11.25m)

Data bit 1 (2.25m ): 0.56m  + sapce
Data bit 0(1.125m): 0.56m  + sapce

Falling edge interrupt 
*/

#include <stdio.h>
#include <stdlib.h>
//#define MY_DEBUG 1

#define WATCHDOG_ENABLE 1

// CONFIG1
#pragma config IESO = OFF    // Internal/External Switchover Mode->Internal/External Switchover Mode is disabled
#pragma config BOREN = OFF    // Brown-out Reset Enable->Brown-out Reset disabled
#pragma config PWRTE = OFF    // Power-up Timer Enable->PWRT disabled
#pragma config FOSC = INTOSC    // Oscillator Selection Bits->INTOSC oscillator: I/O function on CLKIN pin
#pragma config FCMEN = ON    // Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled
#pragma config MCLRE = ON    // MCLR Pin Function Select->MCLR/VPP pin function is MCLR
#pragma config CP = OFF    // Flash Program Memory Code Protection->Program memory code protection is disabled
#ifdef WATCHDOG_ENABLE
#pragma config WDTE = NSLEEP    // Watchdog Timer Enable->WDT enabled while running and disabled in Sleep
#else
#pragma config WDTE = OFF    // Watchdog Timer Enable->WDT disabled
#endif

#pragma config CLKOUTEN = OFF    // Clock Out Enable->CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin

// CONFIG2
#pragma config WRT = OFF    // Flash Memory Self-Write Protection->Write protection off
#pragma config ZCDDIS = ON    // Zero-cross detect disable->Zero-cross detect circuit is disabled at POR and can be enabled with ZCDSEN bit.
#pragma config LPBOR = OFF    // Low-Power Brown Out Reset->Low-Power BOR is disabled
#pragma config PPS1WAY = ON    // Peripheral Pin Select one-way control->The PPSLOCK bit cannot be cleared once it is set by software
#pragma config LVP = OFF    // Low-Voltage Programming Enable->High-voltage on MCLR/VPP must be used for programming
#pragma config STVREN = ON    // Stack Overflow/Underflow Reset Enable->Stack Overflow or Underflow will cause a Reset
#pragma config PLLEN = ON    // Phase Lock Loop enable->4x PLL is always enabled
#pragma config BORV = LO    // Brown-out Reset Voltage Selection->Brown-out Reset Voltage (Vbor), low trip point selected.


#include <xc.h>
#include "eusart.h"


#define abs(x) ((x) < 0 ? -(x) : (x))
#define max(a,b) (((a)>(b)) ? (a):(b))
#define min(a,b) (((a)<(b)) ?  (a):(b))

/* 
    Genitek remocon key
    main : epg : 0x8  + :0x20 - :0x21
    mood : info :0x12  +: 0x22 -: 0x23
    up: 0x9 down:0x7 
    left:  0xa right:0x06
    okay : 0x11
    mood key : menu :0x0f
    main key : EXIT : 0x10
   
*/
#define IR_FUNCTION_KEY 0x11
#define IR_MAIN_KEY 0x08
#define IR_MAIN_UP_KEY 0x20
#define IR_MAIN_DOWN_KEY 0x21
#define IR_MOOD_KEY 0x12
#define IR_MOOD_UP_KEY 0x22
#define IR_MOOD_DOWN_KEY 0x23

#define IR2_FUNCTION_KEY 0x18
#define IR2_MAIN_KEY 0x0f
#define IR2_MAIN_UP_KEY 0x0a
#define IR2_MAIN_DOWN_KEY 0x0b
#define IR2_MOOD_KEY 0x13
#define IR2_MOOD_UP_KEY 0x0c
#define IR2_MOOD_DOWN_KEY 0x0d


#define _XTAL_FREQ  2000000  //2Mhz
#define STRLEN 12
#define DAC_MAX  30    
#define ON  0
#define OFF 1

/* input */
#define MAIN_SW PORTAbits.RA7
#define MOOD_SW PORTAbits.RA6
#define TACT_SW PORTCbits.RC0

#define ADAPTOR_INPUT PORTCbits.RC3
#define CHARGE_INPUT PORTCbits.RC1
#define DONE_INPUT PORTCbits.RC2
#define REMOTE_IN  PORTCbits.RC7

//#define MAIN_VR PORTAbits.RA0
//#define MOOD_VR PORTAbits.RA3

/* output */
#define MAIN_LED_EN LATAbits.LATA1
#define MOOD_LED_EN LATAbits.LATA4
#define LED_CHARGE LATBbits.LATB0

#define LED_25 LATBbits.LATB1
#define LED_50 LATBbits.LATB2
#define LED_75 LATBbits.LATB3
#define LED_100 LATBbits.LATB4
#define REMOTE_EN LATCbits.LATC6

#define MAIN_DAC_OUT DAC1CON1
#define MOOD_DAC_OUT DAC2CON1


#define REMOTE_ADDRESS				0x2222
#define REMOTE_ADDRESS2				0xee11

/*
    5volts/255=0.02 volts/bit
*/
#if 0

#define BATTERY_100 ((410/2)+4)  /* 4.1V */
#define BATTERY_75  (390/2)   /* 3.9V */
#define BATTERY_50  (370/2)  /* 3.7V */
//#define BATTERY_25  (354/2)  /* 3.54V */
#define BATTERY_25  (320/2)  /* 3.54V */
#endif

#define BATTERY_100 ((51*41)/10)  /* 4.1V */   //209
#define BATTERY_75  ((51*39)/10)   /* 3.9V */
#define BATTERY_50  ((51*37)/10)  /* 3.7V */
#define BATTERY_25  ((51*32)/10)  /* 3.2V */


#define BATTERY_200mV 10

#define MAX_MAIN_DAC1 200
#define MAX_MAIN_DAC2 193
#define MAX_MOOD_DAC  24


//volatile unsigned int t0_10msec = 0;
//volatile unsigned int t0_20msec = 0;
//volatile unsigned int t0_500msec = 0;
//volatile unsigned int t0_1000msec = 0;
//volatile unsigned char t0_blink =0;

unsigned char counter_repeat_key=0;
unsigned char counter_ir_repeat_key=0;

unsigned short counter_timer_off=0;
unsigned short counter_timer_off_blinking=0;
unsigned char  timer_off_loop=0;
unsigned short counter_sleep_mode=0;

unsigned short counter_battery_display=0;
unsigned char counter_short_key_off=0;
unsigned char ir_mood_dac_out=0;
#if 0
volatile unsigned char t;
volatile unsigned char rcindex;
volatile unsigned char rcbuf[STRLEN];
#endif

unsigned char main_adc_val;
unsigned char main_adc_val_prev=180;

unsigned char mood_adc_val;
unsigned char mood_adc_val_prev=0;

unsigned char battery_val;

unsigned char keep_pressed_sos_cnt=0;
unsigned char sos_s_cnt=0;
unsigned char sos_status=0;
unsigned char timer0_sos=0;
unsigned char tact_pressed_cnt=0;
unsigned char tact_pressed_repeat=0;
unsigned char ir_function_pressed=0;

unsigned char led_status=0;

volatile unsigned char remote_interval_ticks=0;

volatile unsigned char ir_state;
volatile unsigned char ir_key_data=0;
unsigned char ir_key_buf=0;
unsigned char ir_key_pressed=0;
unsigned char ir_function_key_repeat=0;
unsigned char counter_ir_key_2sec=0;

unsigned char ir_key_data_prev;

volatile unsigned char ir_repeat_key=0;
volatile unsigned char ir_repeat_key_cnt=0;

volatile unsigned short ir_data=0;
volatile unsigned short ir_data_mask=0;
volatile unsigned char code8; 
volatile unsigned char edoc8; 



volatile unsigned char tick_100u=0;
volatile unsigned char tick_1m=0;
volatile unsigned short tick_1m_sos=0;
volatile unsigned short tick_1m_loop=0;
volatile unsigned char tick_1m_remo=0;
volatile unsigned char tick_1m_volume=0;

volatile unsigned short ioc_cnt=0;

unsigned char saved_last_main_dac_vol; // for sos

unsigned char last_main_dac_vol;
unsigned char last_mood_dac_vol;

unsigned char led_status_cnt;
//test
unsigned char ir_key_released_cnt=0;
unsigned char ir_key_released=0;


unsigned char sleep_timer_count=0;
unsigned char counter_4sec=0;
unsigned short counter_1hour=0;

unsigned char counter_ir_main_sw=0;
unsigned char my_state=0;

//unsigned char watchdog_enable=0;
//unsigned short watchdog_delay=0;

unsigned char enter_watchdog=0;
unsigned short test1=0;
unsigned char my_mood_cnt=0;
enum ir_status
{
	INIT,
	LEADER_DETECT,
	CUSTOM_DETECT,
	DATA_DETECT
	
};



volatile union
{
    unsigned char flags;
    struct 
    {
        unsigned main_sw:1;  
        unsigned mood_sw :1;
        unsigned tact_pressed :1;   //
        unsigned LedToggle :1;
        unsigned led_progressive :1;
        unsigned KeySosFound :1;  // led off before KeySosAction
        unsigned KeySosAction :1;
        unsigned battery_charging :1;
    };
}flag1;

volatile union
{
    unsigned char flags;
    struct
    {
        unsigned timer_sleeping:1;  
        unsigned battery_adaptor_blink :1;
        unsigned short_key :1;   //
        unsigned toggle :1;
        unsigned watchdog_reset :1;
        unsigned KeySosAction_remo :1;  // led off before KeySosAction
        unsigned battery_usb_in :1;
        unsigned ir :1;
    };
}flag2;

volatile union
{
    unsigned char flags;
    struct
    {
        unsigned remocon_type:1;  
        unsigned ir_sos :1;
        unsigned adaptor_init :1;   
        unsigned usb_in_display :1;
        unsigned my_test :1;
        unsigned charge_full :1;  
        unsigned main_sw_test :1;
        unsigned my_mood :1;
    };
}flag3;


volatile union
{
    unsigned char ticks;
    struct
    {
        unsigned tick_10m:1;  
        unsigned tick_20m :1;
        unsigned tick_100m :1;   //
        unsigned cc :1;
        unsigned dd :1;
        unsigned ee :1;  // led off before KeySosAction
        unsigned ff :1;
        unsigned gg :1;
    };
}tick;

#if 0
void uart_putc(unsigned char c)
{

    while(!TX1STAbits.TRMT); //wait until transmit shift register is empty
    TX1REG=c;

}

void uart_puts(const unsigned char *s)
{

    while(*s)
    {
        uart_putc(*s);
        s++;
    }

}
#endif

void interrupt isr(void)
{
    unsigned char interval;
#if 0
    if(INTCONbits.TMR0IF)  // 5msec timer
    {
        INTCONbits.TMR0IF = 0;
        TMR0 += 217;
        t0_10msec++;
        t0_20msec++;
        t0_500msec++;
        timer0_sos++;
    }
#else

    if(INTCONbits.TMR0IF)  // 100usec timer
    {
        INTCONbits.TMR0IF = 0;
        //TMR0 = 206;   // 206(0xce)org for 1msec
        TMR0 = 219;   // 218 okay key //217(0xd9) for 1msec
        remote_interval_ticks++;
        if(++tick_100u >= 10) 
        {
           tick_100u=0;
           tick_1m++;
           tick_1m_sos++;
           tick_1m_loop++;
           tick_1m_remo++;
           tick_1m_volume++;
           //MAIN_LED_EN^=1;

           
           //if(tick_1m==10) tick.tick_10m=1;
           //else if(tick_1m==20) { tick.tick_10m=1; tick.tick_20m=1; tick_1m=0;}

        }
        
        //t0_10msec++;
        //t0_20msec++;
        //t0_500msec++;
        //timer0_sos++;
    }

    
    if(INTCONbits.IOCIF==1 /*&& INTCONbits.IOCIE == 1*/)
    {
        INTCONbits.IOCIF=0;
        if(IOCCN7 ==1  && IOCCF7 == 1)
        {
            IOCCF7=0;  //should be!
            //@TODO Add handling code for IOC on pin RC7
            ioc_cnt++;
        	interval=remote_interval_ticks;
        	remote_interval_ticks=0;

        	if(ir_state == INIT)
        	{
        		ir_data=0;
        		ir_state= LEADER_DETECT;
        		code8=0;  //
        	}	
        	else if(ir_state == LEADER_DETECT)
        	{
        		if(interval>(121+2) && interval <(149-5))  /* reference 13.5 ms */
        		{
        			ir_data=0;
        			ir_data_mask=0x0001;
        			ir_state = CUSTOM_DETECT;
        			//
        			//ir_repeat_key=0;
        		}
        		
        		if(interval>101 && interval <(124))  /* reference 11.25 ms for Repeat code*/
        		{  
        			ir_data=0;
        			ir_data_mask=0x0001;
        			ir_state = CUSTOM_DETECT;
        			
        			ir_key_data =ir_repeat_key;
        			ir_repeat_key_cnt++;
        			//ir_key_data =0x55;
        		}
        	}	

        	else if(ir_state == CUSTOM_DETECT)
        	{
        		if(interval >= (20-2) && interval <25) 	/* reference 2.25 ms */
        		{
        			ir_data |= ir_data_mask;  
        	    }		
        		else if(interval > 9  && interval < 15) /* reference 1.125 ms */
        		{
        		 
                 //ir_data &= ~(ir_data_mask);
                 NOP();  
        		}
        		else 
        		{
        			ir_state = LEADER_DETECT;
        	    }		

        		ir_data_mask <<= 1;
        		if(ir_data_mask==0)
        		{
        		#if 0
        			code8 = ir_data;
        			edoc8= ~(ir_data>>8); 
        			if(code8 == edoc8)
        			{
        				ir_data=0;
        				ir_data_mask=0x0001;
        				ir_state = DATA_DETECT;
        			}
        		#endif
        		    if(ir_data==REMOTE_ADDRESS || ir_data==REMOTE_ADDRESS2)
        		    {
        		    
                        if(ir_data==REMOTE_ADDRESS ) flag3.remocon_type=0;
                        else flag3.remocon_type=1;
        				ir_data=0;
        				
        				ir_data_mask=0x0001;
        				ir_state = DATA_DETECT;
        		    }

        		}
        		
        	}
        	else if(ir_state == DATA_DETECT)
        	{
        		if(interval >= (20-2) && interval <25) 	/* reference 2.25 ms */
        		{
        			ir_data |= ir_data_mask;  
        		}	
        		else if(interval >= 9  && interval < 15) /* reference 1.125 ms */
        		{
        			//ir_data &= ~(ir_data_mask);
        			NOP();    
        		}
        		else 
        		{
        			ir_state = LEADER_DETECT;
        		}	
        		ir_data_mask <<= 1;
        		if(ir_data_mask==0)
        		{
        			
        			code8 = ir_data;
        			edoc8= ~(ir_data>>8); 
        			if(code8 == edoc8)
        			{
        			   ir_key_data = code8;	


                        #if 0
                       if( (ir_key_data==IR_MAIN_KEY) || (ir_key_data==IR_MOOD_KEY)  || (ir_key_data==IR2_MAIN_KEY) || (ir_key_data==IR2_MOOD_KEY))
                       {
                            ir_repeat_key=0;
                       }
                       else
                       {
                           ir_repeat_key = code8;
                       }
                       #endif
                        ir_repeat_key = code8;
                       
        			   ir_repeat_key_cnt=0;

        			}
        			ir_state = INIT;
        		}


        	}            //

            
        }
        
    }    

#endif


#if 0
    if(PIR1bits.RCIF) // check if 
    {
        t=RC1REG;
        if(t!='\n' && rcindex < STRLEN)
        {
            rcbuf[rcindex]=t;
            rcindex++;

        }
        else
        {
            rcindex=0;
            //uart_puts(&rcbuf[0]); //echo received string
        }

        PIR1bits.RCIF=0;
    }
#endif    

}
#if 0
void TMR0_ISR(void)  //5msec timer
{

    // clear the TMR0 interrupt flag
    INTCONbits.TMR0IF = 0;

    TMR0 += 217;
    timer0_5msec++;

    // add your TMR0 interrupt custom code
}
#endif
/*
void ADC_ISR(void)
{
    // Clear the ADC interrupt flag
    PIR1bits.ADIF = 0;
}
*/

unsigned char ADC_GetConversion(unsigned char channel)
{
    // Select the A/D channel
    ADCON0bits.CHS = channel;

    // Turn on the ADC module
    ADCON0bits.ADON = 1;

    // Acquisition time delay
    __delay_us(5);

    // Start the conversion
    ADCON0bits.GO_nDONE = 1;

    // Wait for the conversion to finish
    while (ADCON0bits.GO_nDONE);
    
    // Conversion finished, return the result
    //return ((ADRESH << 8) + ADRESL);
    return ADRESH;  // by left justified so ignore 2bit of ADRESL
    
}

#if 0
unsigned char get_adc(unsigned char channel)
{
    // Select the A/D channel
    ADCON0bits.CHS = channel;

    // Turn on the ADC module
    ADCON0bits.ADON = 1;

    // Acquisition time delay
    __delay_us(5);

    // Start the conversion
    ADCON0bits.GO_nDONE = 1;

    // Wait for the conversion to finish
    while (ADCON0bits.GO_nDONE);
    
    // Conversion finished, return the result
    //return ((ADRESH << 8) + ADRESL);
    if(ADRESH >= 250) return 4;
    if(ADRESH >= 192) return 3;
    if(ADRESH >= 128) return 2;
    if(ADRESH >= 64) return 1;
    if(ADRESH == 0) return 0;
    ///return ADRESH;  // by left justified so ignore 2bit of ADRESL
    
}
#endif

void led_toggle(unsigned char v)
{
    if(flag1.LedToggle==0)
    {
        
        MAIN_DAC_OUT=v;
        MAIN_LED_EN=ON;
        flag1.LedToggle=1;
    }
    else
    {
        //MAIN_DAC_OUT=0;
        MAIN_LED_EN=OFF;
        flag1.LedToggle=0;
    }
}
void battery_display2(unsigned char option)
{
    unsigned short var1,var2;
    //unsigned char real_battery_val;
#if 0
    unsigned char main;
    unsigned char mood;
    
    //LED_25=LED_50=LED_75=LED_100=OFF;
    main=MAIN_DAC_OUT;
    mood=MOOD_DAC_OUT;
    MAIN_DAC_OUT=0;
    MAIN_LED_EN=OFF;
    MOOD_DAC_OUT=0;
    MOOD_LED_EN=OFF;
    __delay_us(10);
    battery_val=ADC_GetConversion(16);
    MAIN_DAC_OUT=main;
    MOOD_DAC_OUT=mood;
    MAIN_LED_EN=ON;
    MOOD_LED_EN=ON;

#endif
    var1=var2=0;    
    if(flag1.main_sw==1)
    {
        //max=233(0xe9) -> 200
        // max-min = 160mv 
        // 100:51=16:x, x=8.16
        var1=last_main_dac_vol;
        //var1=(var1*14)/233;
        var1=(var1*86)/2000;
        
    }
    if(flag1.mood_sw==1)
    {
        //max=31(0x1f) ->24
        // max-min= 54mv
        var2=last_mood_dac_vol;
        //var2=(var2*10)/31;
        var2=(var2*27)/240;
    }

    battery_val=ADC_GetConversion(16);
    battery_val+=var1+var2;
    /*
    if(option==0)
    {
        if(battery_val >= BATTERY_100) LED_100^=1;
        else if(battery_val >= BATTERY_75) LED_75^=1;
        else if(battery_val >= BATTERY_50) LED_50^=1;
        else if(battery_val >= BATTERY_25)  LED_25^=1;
        else LED_CHARGE^=1;
    }
    else if(option==1)
    {
        if(battery_val >= BATTERY_100) { LED_100^=1; LED_75^=1; LED_50^=1;LED_25^=1; }
        else if(battery_val >= BATTERY_75) { LED_75^=1; LED_50^=1;LED_25^=1;}
        else if(battery_val >= BATTERY_50) { LED_50^=1;  LED_25^=1;}
        else if(battery_val >= BATTERY_25)  LED_25^=1;
        else LED_CHARGE^=1;
    }
    else if(option==2)
    {
        if(battery_val >= BATTERY_100) LED_100=ON;
        else if(battery_val >= BATTERY_75) LED_75=ON;
        else if(battery_val >= BATTERY_50) LED_50=ON;
        else if(battery_val >= BATTERY_25)  LED_25=ON;
        else LED_CHARGE=ON;

    }
    else 
    */
    if(option==3)
    {
        if(battery_val >= BATTERY_100)      { LED_100=ON; LED_75=ON; LED_50=ON;LED_25=ON; }  //---------------->review
        else if(battery_val >= BATTERY_75)  { LED_100=OFF; LED_75=ON; LED_50=ON;LED_25=ON;}
        else if(battery_val >= BATTERY_50)  { LED_100=OFF; LED_75=OFF;LED_50=ON;  LED_25=ON;}
        else if(battery_val >= BATTERY_25)  { LED_100=OFF; LED_75=OFF;LED_50=OFF; LED_25=ON;  }
        else {LED_100=OFF; LED_75=OFF;LED_50=OFF; LED_25^=1; }
    }

    
}
/*
    208 (4V)  
    200 (3.9V) 
    0~5V ADC 
    3.7V BATTERY : 4.1V 

*/
//unsigned char b_c=0;
//unsigned char battery_v[3];
void battery_display(unsigned char option)
{
    //LED_25=LED_50=LED_75=LED_100=OFF;
    //unsigned short va;
    unsigned short var1,var2;
    unsigned char tmp;
    if(option==0)
    {
        LED_25=LED_50=LED_75=LED_100=OFF;
        led_status=0;
        return;
    }
    else if(option==1)
    {
        LED_25=LED_50=LED_75=LED_100=OFF;
        battery_val=ADC_GetConversion(16);
        if(battery_val >= BATTERY_100) LED_100=ON;  
        else if(battery_val >= BATTERY_75) LED_75=ON; 
        else if(battery_val >= BATTERY_50) LED_50=ON;  
        else if(battery_val >= BATTERY_25)  LED_25=ON;  
        return;
    }
    else if(option==2)
    {
    
        var1=var2=0;    
        if(flag1.main_sw==1)
        {
            //max=233(0xe9)
            var1=last_main_dac_vol;
            //var1=(var1*10)/233;
            var1=(var1*86)/2000;
            
        }
        if(flag1.mood_sw==1)
        {
            //max=31(0x1f)
            var2=last_mood_dac_vol;
            //var2=(var2*8)/31;
            var2=(var2*27)/240; 
        }
        battery_val=ADC_GetConversion(16);
        battery_val+=var1+var2;
        #if 0
        battery_v[b_c]=battery_val;
        b_c++;
        if(b_c==3) {
            va=battery_v[0]+battery_v[1]+battery_v[2];
            va=va/3;
            battery_val=va;
            b_c=0;
            
            if(DONE_INPUT==ON)                  { LED_100=ON;  LED_75=OFF; LED_50=OFF; LED_25=OFF; }
            else if(battery_v[0] >= BATTERY_100 && battery_v[1] >= BATTERY_100 && battery_v[2] >= BATTERY_100) { LED_100=ON;  LED_75=ON;  LED_50=ON;  LED_25=ON; }
            else if(battery_val >= BATTERY_75)  { LED_100=OFF; LED_75=ON;  LED_50=ON;  LED_25=ON;}
            else if(battery_val >= BATTERY_50)  { LED_100=OFF; LED_50=OFF; LED_50=ON;  LED_25=ON;}
            else if(battery_val >= BATTERY_25)  { LED_100=OFF; LED_50=OFF; LED_50=OFF; LED_25=ON; }
        }   
        #else
        if(DONE_INPUT==ON)                  { LED_100=ON;  LED_75=OFF; LED_50=OFF; LED_25=OFF; }
        else if(battery_val >= BATTERY_100) { LED_100=ON;  LED_75=ON;  LED_50=ON;  LED_25=ON; }
        else if(battery_val >= BATTERY_75)  { LED_100=OFF; LED_75=ON;  LED_50=ON;  LED_25=ON;}
        else if(battery_val >= BATTERY_50)  { LED_100=OFF; LED_50=OFF; LED_50=ON;  LED_25=ON;}
        else if(battery_val >= BATTERY_25)  { LED_100=OFF; LED_50=OFF; LED_50=OFF; LED_25=ON; }
        #endif


    }
    else if(option==3)
    {
        switch(led_status)
        {
            case 0:
                //LED_CHARGE=ON;
                LED_25=LED_50=LED_75=LED_100=OFF;
                led_status=1;
                break;
            case 1:

                var1=var2=0;    
                if(flag1.main_sw==1)
                {
                    var1=last_main_dac_vol;
                    var1=(var1*86)/2000;
                    
                }
                if(flag1.mood_sw==1)
                {
                    var2=last_mood_dac_vol;
                    var2=(var2*27)/240; 
                }
                battery_val=ADC_GetConversion(16);
                battery_val+=var1+var2;
                
                //battery_val=ADC_GetConversion(16);
                if(battery_val >= BATTERY_100) { LED_100=ON;  LED_75=ON;  LED_50=ON;  LED_25=ON; }
                else if(battery_val >= BATTERY_75) { LED_100=OFF; LED_75=ON;  LED_50=ON;  LED_25=ON;}
                else if(battery_val >= BATTERY_50) { LED_100=OFF; LED_50=OFF; LED_50=ON;  LED_25=ON;}
                else if(battery_val >= BATTERY_25)  { LED_100=OFF; LED_50=OFF; LED_50=OFF; LED_25=ON; }
                led_status=7;
                break;
            case 2:
                LED_25=LED_50=LED_75=LED_100=OFF;
                led_status=3;
                break;
                
            case 3:
                LED_25=ON;
                led_status=4;
                break;
            case 4:
                LED_25=ON;    LED_50=ON;
                led_status=5;
                break;
            case 5:
                LED_25=ON;    LED_50=ON; LED_75=ON;
                led_status=6;
                break;
            case 6:
                LED_25=ON;    LED_50=ON; LED_75=ON; LED_100=ON;
                led_status=0;
                break;
            case 7:
                led_status=8;
                led_status_cnt=6;
                break;
            case 8:
                LED_CHARGE^=1;
                if(--led_status_cnt==0) 
                    led_status=9;
                break;
            case 9:
                led_status=2;
                break;
            default:
                // for test
                LED_100^=1;
                break;

        }
        return;
    }
    else if(option==4)
    {
        //@if(flag3.usb_in_display==1) LED_CHARGE=ON;
        switch(led_status)
        {

            case 0:
            case 1:
                //LED_CHARGE=ON;
                //LED_25=LED_50=LED_75=LED_100=OFF;
                //led_status=2;
            
                var1=var2=0;    
                if(flag1.main_sw==1)
                {
                    var1=last_main_dac_vol;
                    var1=(var1*86)/2000;
                    
                }
                if(flag1.mood_sw==1)
                {
                    var2=last_mood_dac_vol;
                    var2=(var2*27)/240; 
                }
                battery_val=ADC_GetConversion(16);
                battery_val+=var1+var2;
                
                //battery_val=ADC_GetConversion(16);
                /*
                    209 is battery_100
                    max adc =216,217,218,219

                */
                
                
                if(battery_val >= BATTERY_100) { tmp=4;  LED_100=ON;  LED_75=ON;  LED_50=ON;  LED_25=ON;  }
                else if(battery_val >= BATTERY_75) {tmp=3; LED_100=OFF; LED_75=ON;  LED_50=ON;  LED_25=ON;}
                else if(battery_val >= BATTERY_50) { tmp=2;LED_100=OFF; LED_50=OFF; LED_50=ON;  LED_25=ON; }
                else if(battery_val >= BATTERY_25)  { tmp=1; LED_100=OFF; LED_50=OFF; LED_50=OFF; LED_25=ON; }
                else tmp=0;
                led_status=2;

                if(battery_val>=BATTERY_100+8) { flag3.charge_full=1; led_status=0;   }
                else flag3.charge_full=0;


                break;
            case 2:
                LED_25=LED_50=LED_75=LED_100=OFF;
                led_status=3;
                break;
            case 3:
                if(tmp>=1) LED_25=ON;
                if(tmp==1) led_status=7;  
                else led_status=4;
                break;
            case 4:
                if(tmp>=2) LED_50=ON; 
                if(tmp==2) led_status=7; 
                else led_status=5;
                break;
            case 5:
                if(tmp>=3) LED_75=ON; 
                if(tmp==3) led_status=7;
                else led_status=6;
                break;
            case 6:
                if(tmp>=4) LED_100=ON;
                led_status=7;
                break;
            case 7:
                led_status=0;

                break;
        }                


        return;
    }

}
#define IO_RA7_PORT               RA7

#if 1
void factory_test(unsigned char cnt)
{
    while(cnt)
    {
        if(tick_1m_sos>=1000)
        {
           tick_1m_sos=0;   
           MOOD_LED_EN^=1;
           MAIN_LED_EN^=1;
           if(MOOD_LED_EN==OFF)   MOOD_DAC_OUT=0;
           else MOOD_DAC_OUT=last_mood_dac_vol;

           
           LED_25^=1;
           LED_50^=1;
           LED_75^=1;
           LED_100^=1;
           //LED_CHARGE^=1;
           cnt--;

           
        }
#ifdef WATCHDOG_ENABLE
        CLRWDT();
#endif        
    }

}
#endif

#if 1
void my_mood()
{
    static unsigned char m_led;
    static unsigned char o_led;
    //MAIN_DAC_OUT=200;
    if(tick_1m_sos>=40)
    {
        tick_1m_sos=0;

        switch(my_state)
        {
            case 0:
            
                //LED_100=ON;
                m_led=200; //last_main_dac_vol; //200;
                o_led=0;
                my_state=1;
                //MAIN_LED_EN=OFF;
                MAIN_DAC_OUT=m_led;
                MOOD_DAC_OUT=o_led;
                my_mood_cnt=0;
                break;
            case 1:
            
                //LED_100=OFF;
                //LED_75=ON;
                m_led--;
                o_led++;
                MAIN_DAC_OUT=m_led;
                MOOD_DAC_OUT=o_led;
                if(m_led==0) my_state=2;
                if(o_led>=24) o_led=23;
                break;
            case 2:
                if(++my_mood_cnt>=20) my_state=3;
                MAIN_LED_EN=OFF;
                break;
            case 3:
                MAIN_LED_EN=ON;
                //LED_75=OFF;
                //LED_50=ON;
                m_led++;
                o_led--;
                MAIN_DAC_OUT=m_led;
                MOOD_DAC_OUT=o_led;
                if(m_led>=last_main_dac_vol) my_state=4;
                if(o_led==0) o_led=1;
                break;
            case 4:
                MOOD_DAC_OUT=last_mood_dac_vol;
                //LED_50=OFF;
                //LED_25=ON;
                flag3.my_mood=0;
                break;
                


        }

     }   


}
#endif

#if 0

void mood_test()
{
    unsigned char tmp;
    unsigned char up=1;
    unsigned char down=0;
    MOOD_LED_EN=ON;
    tmp=1;
    MOOD_DAC_OUT=tmp;
    
    for(;;)
    {
        if(tick_1m_loop>=500)
        {
            tick_1m_loop=0;
            
            MOOD_DAC_OUT=tmp;
            if(up==1)
            {
                tmp++;
                
            }   
            if(down==1) tmp--;
            
            if(tmp==6)
            {
                up=0;
                down=1;
            }
            if(tmp==4)
            {
                up=1;
                down=0;
            }

        }   
#ifdef WATCHDOG_ENABLE
        CLRWDT();
#endif        

    }
    MOOD_LED_EN=OFF;
    MOOD_DAC_OUT=0;


}

void led_flash(unsigned char cnt)
{
    MOOD_LED_EN=ON;
    MAIN_LED_EN=ON;
    LED_25=LED_50=LED_75=LED_100=OFF;
    LED_CHARGE=OFF;
    
    MOOD_DAC_OUT=0x1f;
    MAIN_DAC_OUT=0xe9;
    for(;;)
    {
        
        if(tick_1m_loop>=500)
        {
           tick_1m_loop=0;   
          MAIN_LED_EN^=1;
          MOOD_LED_EN^=1;
          if(MOOD_LED_EN==OFF) MOOD_DAC_OUT=0;
          else MOOD_DAC_OUT=0X1F;
          
          LED_25^=1;
          LED_50^=1;
          LED_75^=1;
          LED_100^=1;
          LED_CHARGE^=1;
          if(--cnt == 0) break;

        }
        
#ifdef WATCHDOG_ENABLE
        CLRWDT();
#endif        

    }

    
    LED_25=LED_50=LED_75=LED_100=OFF;
    LED_CHARGE=OFF;
    
    MAIN_LED_EN=OFF;
    MOOD_LED_EN=OFF;
    MAIN_DAC_OUT=0;
    MOOD_DAC_OUT=0;
}
#endif

void sleep_mode()
{
    LED_CHARGE=OFF;
    IOCANbits.IOCAN7 = 1; // enable IOC on RA7 input(MAIN_SW)
    IOCANbits.IOCAN6 = 1; // enable IOC on RA6 input(MOOD_SW)

    IOCAPbits.IOCAP7 = 1; // enable IOC on RA7 input(MAIN_SW)
    IOCAPbits.IOCAP6 = 1; // enable IOC on RA6 input(MOOD_SW)

    //IOCCPbits.IOCCP7 = 1; // enable IOC on RC7 input(REMOTE IN)
    //IOCCNbits.IOCCN0 = 1; // enable IOC on RC0(TACK_SW)

    INTCONbits.IOCIE=1;// interrupt-on-change enable bit 
    ei();
    //LED_25=LED_50=LED_75=LED_100=OFF;

    MOOD_LED_EN=OFF;
    MAIN_LED_EN=OFF;
    MAIN_DAC_OUT=0;
    MOOD_DAC_OUT=0;
    counter_sleep_mode=0;

    // set portb input mode for prevent ir-remocon in the sleep 
    LATB = 0x00;
    TRISB = 0xFF;
    ANSELB = 0x3F;
    WPUB = 0x00;

    SLEEP();
}

/*
! when power off an on, main led not display
! remocon key value is comming when sos mode 
-> test with remove ir
-> should be clear repeat key 
! how to know completion of recharge 


    battery 10,400mAh : led 24ea
    charge time 5~7 hour
    
    test01 : start  10:30am with main full 75%
                    11:50   50%  but 75% after main sw off and on 
                    12:00   75%
                    12:30   50%
                    01:20   50%
                    02:05   50%
                    03:00   50%
                    03:30   50%
                    03:50   50% (3.69v(main on)/3.82v(main off)) about 120mv diff
                    04:40   25% (3.65v)  
                    05:10   25%
                    05:20
                    05:30   25%
                    06:00 
                    06:30   25% (3.44v) + mood on
                    07:00   25% (3.37v) + mood on

             continue
                    10:15   25% (3.38v)  + mood off
                    10:25   25% (3.52v) 
                    10:55   25% (3.34V)
                    11:15   0%  (2.76v)

        charge start                      
                    11:20 : 0%  (2.9v)
                    11:24 : 50% (3.54v)
                    11:30 : 75% (3.72v)
                    11:40 : 75% (3.82v)
                    12:00 : 75% (3.84v)
                    12:20 : 75% (3.88v)
                    01:25 : 75% (3.92v)

        charge start2
                    12:15 : 75% (3.82v)
                    12:23 : 75% (3.90v)
                    01:10 : 75% (3.94v)
                    01:35 : 75% (3.96v)
                    02:05 : 75% (3.94v)
                    02:30 : 75% (3.95v)
                    03:00 : 75% (3.97v)
                    ~~ test ~~
                    04:20 : 75% (4v)
                    05:25 : 75% (4.04v)
                    06:05 : 75% (4.08v)

        charge start3 
                    12:00 : 4.12v
                    01:10 : 4.16v
                    02:15 : 4.19v

        charge start4 
                    04:25 : 4.07v
                    04:55 : 4.10v
                    05:10 : 4.10v
                    05:50 : 4.16v
                    
bq2410x : potable power supply and battery charger, has power fts for up to 2a charge rate

3.7v 10400mAh  ICR 18650 li-ion battery pack 
nominal voltage :3.7v
cut off voltage :3.0v
operating voltage : 3~4.2v
cycle life :500 times
*/


/*
    function
    1. display battery remains by pressing function key
    2. sleep counter 5min or 10min by pressing function key twice.
        -> flushing led_100
        -> cancel by main_sw
    3. SOS signal by pressing funciton key for 2 sec above


    other defined key 
    main sw long key
    mood sw long key
*/


#if 0
unsigned char adc1[10];
unsigned char adc2[10];

unsigned short s_adc1;
unsigned short s_adc2;
#endif

int main(int argc, char** argv) {
    
    unsigned char i=0;
    #if 0
    unsigned char j=0;
    unsigned short cnt=0;
    #endif
    /* osc */
    // SPLLEN disabled; SCS INTOSC; IRCF 2MHz_HF; 
    OSCCON = 0x62;
    // OSTS intosc; HFIOFR disabled; SOSCR disabled; HFIOFS not0.5percent_acc; PLLR disabled; MFIOFR disabled; HFIOFL not2percent_acc; LFIOFR disabled; 
    OSCSTAT = 0x00;
    // TUN 0x0; 
    OSCTUNE = 0x00;
    // Set the secondary oscillator

    /* ports are all input*/
    LATA = 0x00;  //data latch register
    TRISA = 0xFF;  // 1: input 0: output
    ANSELA = 0x3F;  
    WPUA = 0x00;

    LATB = 0x00;
    TRISB = 0xFF;
    ANSELB = 0x3F;
    WPUB = 0x00;

    LATC = 0x00;
    TRISC = 0xFF;
    ANSELC = 0x7c; //0xFC;
    WPUC = 0x00;

    TRISE = 0x08;  //?
    WPUE = 0x00;   //?

    TRISAbits.TRISA7=1; // main sw
    //ANSELAbits.ANSA7=0;
    TRISCbits.TRISC0=1; //tact sw
    //ANSELCbits.ANSC0=0;

    TRISAbits.TRISA1=0;  
    TRISAbits.TRISA4=0;  

    TRISAbits.TRISA2=0;  
    TRISAbits.TRISA5=0;  

    
    TRISBbits.TRISB0=0;
    TRISBbits.TRISB1=0;
    TRISBbits.TRISB2=0;
    TRISBbits.TRISB3=0;
    TRISBbits.TRISB4=0;
    TRISCbits.TRISC6=0;
    
    // input port weak pull up 
    WPUC=0x8e;
    OPTION_REGbits.nWPUEN = 0x0;
    //ANSELCbits.ANSC1=0;
    ANSELCbits.ANSC2=0;
    ANSELCbits.ANSC3=0;
    ANSELCbits.ANSC7=0;

    //OPTION_REGbits.nWPUEN = 0x01; // all weak pull-up are disabled

    /* dac1 */
    // DAC1OE2 disabled; DAC1NSS VSS; DAC1OE1 enabled; DAC1PSS VDD; DAC1EN enabled; 
    //DAC1CON0 = 0xA0;
    DAC1CON0bits.DAC1EN=1;
    DAC1CON0bits.DAC1OE1= 1; //output on the DAC1OUT1 pin
    DAC1CON0bits.DAC1OE2 =0;  ////output off the DACOUT2 pin
    DAC1CON0bits.DAC1PSS= 0; // DAC positive source select bi is VDD 
    DAC1CON0bits.DAC1NSS= 0; // DAC negative source select bit is VSS
    DAC1CON1 = 0x00;

    /* dac 2 */
    DAC2CON0 = 0xA0;
    DAC2CON1 = 0x00;

    /* adc */
    //ADCON0 = 0x01;
    ADCON0bits.ADON=1;
    ADCON0bits.CHS=0;

    // ADPREF chip_VDD; ADNREF chip_VSS; ADFM left; ADCS FOSC/2; 
    ADCON1 = 0x00;
    // TRIGSEL no_auto_trigger; 
    ADCON2 = 0x00;
    // ADRESL 0x0; 
    ADRESL = 0x00;
    // ADRESH 0x0; 
    ADRESH = 0x00;
    //PIE1bits.ADIE = 1; // no interrupt service

    /* ioc */

    //rc7 -> rc0 for test
    WPUC7=1;
    ANSELCbits.ANSC7=0;  // digital io pin
    INTCONbits.IOCIE=1; //should be set //interrupt on change enable bit
    //INTCONbits.IOCIF=1; // ioc flag bit
    IOCCNbits.IOCCN7 = 1; //rc7 pin
    IOCCF7=1;
    /* usart */
#if 0    
    //EUSART_Initialize();
    TX1STAbits.TXEN=1; //enable tx
    TX1STAbits.BRGH=1; //hi speed
    TX1STAbits.TRMT=1; // TSR empty 
    RC1STAbits.CREN=1; //continuos receive enable
    // Baud Rate = 2400; SP1BRGL 207; 
    SP1BRGL = 0xCF;
    // Baud Rate = 2400; SP1BRGH 0; 
    SP1BRGH = 0x00;
    
    PIE1bits.RCIE = 1;  //enable rx interrupt
    //PIE1bits.TXIE = 1;  //enable tx interrupt
    
    RC1STAbits.SPEN=1; //serial port enable
    // additional function
    // BAUD1CON : wake-up enable for falling edge(in sleep mode), transmit inverted data to tx pin, auto baud detect
#endif
    /* timer0  5msec*/

    #if 0
    OPTION_REG = (OPTION_REG & 0xC0) | 0xD5 & 0x3F; 
    // TMR0 217; 
    TMR0 = 0xD9;
    #endif

    /* timer0 100usec : reload value =0xce*/
    /*
       option_reg : all weak pull-up are disabled
                    interrupt on rising edge of INT pin

    */
    //OPTION_REG = (OPTION_REG & 0xC0) | 0xDF & 0x3F; 
    
    OPTION_REGbits.PSA=1; // prescaler is not assigned
    OPTION_REGbits.TMR0CS=0; // internal clock 
    //OPTION_REGbits.nWPUEN=1; //all weak pull-up are disabled
    TMR0 = 0xCE;

    


    // Clear Interrupt flag before enabling the interrupt
    INTCONbits.TMR0IF = 0;

    // Enabling TMR0 interrupt
    INTCONbits.TMR0IE = 1;


    //uart_puts("Hello\n");
    //INTCONbits.PEIE=1; //enable peripheral interrupt
    
    INTCONbits.GIE=1; //enable interrupts


    //
    
    flag1.flags=0;
    flag2.flags=0;
    flag3.flags=0;

    //REMOTE_EN=0;
    
    MAIN_LED_EN=OFF;
    MOOD_LED_EN=OFF;
    MAIN_DAC_OUT=0;
    MOOD_DAC_OUT=0;
    LED_CHARGE=OFF;
    LED_25=LED_50=LED_75=LED_100=OFF;
    
    if (STATUSbits.nTO==0)
    {
        /* watchdog reset But never come to here!*/
        
        flag2.watchdog_reset=1;
        //led_flash(5);
    }
    else
    {
        /* normal reset */
        flag2.watchdog_reset=0;
    }    
#ifdef WATCHDOG_ENABLE
    
    /* Power control register */
    PCONbits.nRWDT=0; // watchdog timer reset has occurred(cleared by hardware)
    //WDTCONbits.WDTPS=0b01111; // interval 32s  okay
    //WDTCONbits.WDTPS=0b00110; // 128msec 
    //WDTCONbits.WDTPS=0b00110; // 64msec 
    WDTCONbits.WDTPS=0b00100; // 16msec 
    //WDTCONbits.WDTPS=0b00011; // 8msec okay here
    //WDTCONbits.WDTPS=0b00010; // 4msec 
    
    //WDTCONbits.WDTPS=0b00001; //2msec  nO !
#endif   
    
    #if 1  //basic test
    //led_flash(3);
    //led_mood();
    //mood_test();
    #endif

 
    
    for(;;)
    {
        unsigned char tmp;
        
        #if 1
        if(flag3.my_mood==1 && flag1.main_sw==1 && flag1.mood_sw==1)
        {
            if(flag2.battery_usb_in==0 && flag2.timer_sleeping==0)
                my_mood();
        }
        #endif
        if(tick_1m_volume >=30)
        {
            tick_1m_volume=0;

             
            if(flag2.toggle==0)
            {
                /*

                    if(mood==0), adc=193 ~ 0   3.7v
                    if(mood==1), adc=200 ~ 0   3.9v

                */
                #if 0
                adc1[i++]=ADC_GetConversion(0);
                if(i==10) { 
                    s_adc1= /*adc1[0]+adc1[1]+adc1[2]+adc1[3]+adc1[4]+*/adc1[5]+adc1[6]+adc1[7]+adc1[8]+adc1[9];
                    s_adc1=s_adc1/5;
                    i=0;
                    main_adc_val=s_adc1;
                }    
                #endif  
                //adc1[i++]=ADC_GetConversion(0);
                //if(i==2) { 
                //    s_adc1=min(adc1[0],adc1[1]);
                //    i=0;
                //    main_adc_val=s_adc1;

                main_adc_val=ADC_GetConversion(0);
                 
                if( abs(main_adc_val-main_adc_val_prev) > 3 && abs(main_adc_val-main_adc_val_prev) < 100)
                {
                     main_adc_val_prev=main_adc_val;
                     //flag2.timer_sleeping=0;

                    if(flag1.main_sw==1 && MOOD_SW==1/*flag1.mood_sw==1*/) //flag1.KeySosAction==0)
                    {
                         if(main_adc_val>=200) main_adc_val=200;
                         MAIN_DAC_OUT = 200 - main_adc_val;
                         
                         //if(main_adc_val = 0) MAIN_DAC_OUT=233;
                         
                         if(MAIN_DAC_OUT==0) MAIN_LED_EN=OFF;
                         else   MAIN_LED_EN=ON; 
                         //MAIN_LED_EN=ON;
                         last_main_dac_vol=MAIN_DAC_OUT;
                    }
                    else if(flag1.main_sw==1 && MOOD_SW==0 /*flag1.mood_sw==0*/) //flag1.KeySosAction==0)
                    {
                         if(main_adc_val>=193) main_adc_val=193;
                         MAIN_DAC_OUT = 193 - main_adc_val;
                         
                         //if(main_adc_val = 0) MAIN_DAC_OUT=233-2;
                         
                         if(MAIN_DAC_OUT==0) MAIN_LED_EN=OFF;
                         else   MAIN_LED_EN=ON; 
                         //MAIN_LED_EN=ON;
                         last_main_dac_vol=MAIN_DAC_OUT;
                    }
                    
                     
                     //LED_100=ON; //test
                 }
                 else
                 {
                     //LED_100=OFF; //test

                 }
                 //}
                 flag2.toggle=1;
             }
             else
             {

                /*
                    if(main=0), adc=25(24)~0    3.8/3.75v
                    if(main=1), adc=25~0    3.9v

                */
                #if 0
                adc2[j++]=ADC_GetConversion(3);
                if(j==10) { 
                    s_adc2=/*adc2[0]+adc2[1]+adc2[2]+adc2[3]+adc2[4]+*/adc2[5]+adc2[6]+adc2[7]+adc2[8]+adc2[9];
                    s_adc2=s_adc2/5;
                    j=0;
                    mood_adc_val=s_adc2;
                }    
                #endif    
                 mood_adc_val=ADC_GetConversion(3);
                
                 //if(mood_adc_val != mood_adc_val_prev)
                 if(abs(mood_adc_val-mood_adc_val_prev) > 3)
                 {
                     
                     mood_adc_val_prev=mood_adc_val;
                     
                     //flag2.timer_sleeping=0;
                     if(flag1.mood_sw==1 && 1) //flag1.KeySosAction==0)
                     {
                     
                         tmp=mood_adc_val>>3 & 0x1f;
                         //tmp=mood_adc_val;
                         if(tmp>=24) tmp=24;
                         
                         MOOD_DAC_OUT=24-tmp; // 5 bit dac  // 31-xxx

                         // if(tmp==0 || tmp==1 ) MOOD_DAC_OUT=31;
                         
                         if(MOOD_DAC_OUT==0) MOOD_LED_EN=OFF;
                         else   MOOD_LED_EN=ON; 
                         //MOOD_LED_EN=ON;
                         last_mood_dac_vol=MOOD_DAC_OUT;
                         
                     }
                     //LED_25=ON;
                 }
                 else
                 {
                    //LED_25=OFF;
                 }
                 flag2.toggle=0;
             }


        }

        /* 20msec switch detection */
        if(tick_1m >= 20)  // 2O msec
        {
            tick_1m=0;
            
            counter_timer_off++;
            counter_timer_off_blinking++;
            counter_sleep_mode++;
            counter_repeat_key++;
            counter_battery_display++;
            counter_short_key_off++;
            counter_ir_key_2sec++;
            counter_ir_repeat_key++;
            //test
            ir_key_released_cnt++;
            counter_4sec++;
            counter_ir_main_sw++;
            if(MAIN_SW==1)
            { 
                flag1.main_sw=1; 
            }
            else  
            {
                flag1.main_sw=0;
            }
            if(MOOD_SW==1) 
            { 
                flag1.mood_sw=1; 
            }
            else
            {
                flag1.mood_sw=0;
            }    
            
            if(TACT_SW==0 && flag1.tact_pressed==0 /*&& flag2.battery_usb_in==0*/)
            {
                flag1.tact_pressed=1;
                keep_pressed_sos_cnt=0;

                // fast off battery led
                counter_battery_display=0;
                
                LED_CHARGE=ON; //just for debug

            }
            else if(flag1.tact_pressed==1)
            {
               
                if(TACT_SW==1) // key released
                {
                    LED_CHARGE=OFF; //just for debug
                    flag1.tact_pressed=0;
                    if( flag1.KeySosFound==1)
                    {
                        flag1.KeySosFound=0;

                    }
                    else
                    {
                        if(flag1.KeySosAction==1)
                        {
                            flag1.KeySosAction=0;
                            MAIN_DAC_OUT=saved_last_main_dac_vol;
                            MAIN_LED_EN=ON;
                             
                        }    
                        else //action for short key
                        {
                            tact_pressed_cnt++;
                            counter_repeat_key=0;
                            tact_pressed_repeat=1;

                        }

                    }

                }  
                /* SOS long key detect */
                if(++keep_pressed_sos_cnt >= 100) // 2 sec
                {
                    keep_pressed_sos_cnt=0;
                    if( (flag1.KeySosAction==0) && (flag1.main_sw==1) )
                    {
                        flag1.KeySosFound=1;
                        flag1.KeySosAction=1;
                        sos_status=0;
                        flag1.LedToggle=0;
                        MOOD_DAC_OUT=0; // hardware pb ?
                        MOOD_LED_EN=OFF;
                        if(flag2.short_key==1) 
                        {
                            flag2.short_key=0;
                            LED_25=LED_50=LED_75=LED_100=OFF;             
                        }  
                         
                        if(flag2.timer_sleeping==1) 
                        {
                            flag2.timer_sleeping=0;
                            sleep_timer_count=0;
                            LED_100=OFF;
                        }
                        saved_last_main_dac_vol=last_main_dac_vol;
                        last_main_dac_vol= 200; // start with max dac
                        counter_1hour=0;
                    }

                }

            }
            
        }
        #if 1
        if(counter_4sec>=200)
        {
            counter_4sec=0;
            
            if(++counter_1hour>=900)
            //if(++counter_1hour>=5)
            {
                counter_1hour=0;
                
                flag3.my_mood=1;
                my_state=0;
            }
            
        }
        #endif
        /* MAIN SW OFF */
        if(flag1.main_sw==0)
        {
            MAIN_DAC_OUT=0;
            MAIN_LED_EN=OFF;
            
            if(flag1.KeySosAction==1)
            {
                flag1.KeySosAction=0;
            
            }
            counter_1hour=0;
            //
        }
        /* MOOD SW OFF */
        if(flag1.mood_sw==0)
        {
            //flag2.timer_sleeping=0;
            MOOD_DAC_OUT=0; // hardware pb ?
            MOOD_LED_EN=OFF;
            counter_1hour=0;

        }

        if(flag1.main_sw==0 && flag1.mood_sw==0)
        {
            //stop sleep timer
            if(flag2.timer_sleeping==1) 
            {
                flag2.timer_sleeping=0;
                sleep_timer_count=0;
                LED_100=OFF;
            }

        }
        
        /* short function key detection */
        if(tact_pressed_repeat==1 && counter_repeat_key > 20)  // after 20x20 msec press tact s/w
        {
            tact_pressed_repeat=0;

            if(tact_pressed_cnt==1)
            {
                //LED_25=ON;
                //led_status=0;
                
                if(flag2.battery_usb_in==0)
                {
                    if(flag2.short_key==1) {
                        LED_25=LED_50=LED_75=LED_100=OFF;
                        flag2.short_key=0;

                    }    
                    else
                    {
                        //LED_25=ON;
                        flag2.short_key=1;
                        counter_battery_display=100;  // <---------------------
                        counter_short_key_off=0;
                    }
                }
                else
                {
                    if(flag3.usb_in_display==1) {
                        flag3.usb_in_display=0;
                        battery_display(0);
                    
                    }    
                    else
                    {
                        flag3.usb_in_display=1;
                        led_status=1;
                        
                    }

                }
            }
            else if(tact_pressed_cnt==2)
            {
                if(flag2.battery_usb_in==1)
                {
                    if(flag3.usb_in_display==1)
                    {
                    flag3.usb_in_display=0;
                    battery_display(0);
                    }
                
                }
                
                // 5 min sleep timer off function
                if( (flag1.main_sw==1) || (flag1.mood_sw==1) )
                {
                    flag2.timer_sleeping=1;
                    counter_timer_off=0;

                    if(++sleep_timer_count>=3) sleep_timer_count=1;
                    
                    if(sleep_timer_count==1) timer_off_loop=4;  // 5min sleep timer
                    else if(sleep_timer_count==2) timer_off_loop=6; // 10min sleep timer
                    
                    
                    //a
                    if(flag2.short_key==1) {
                        LED_25=LED_50=LED_75=LED_100=OFF;
                        flag2.short_key=0;
                        //ir_key_buf=0;
                    
                    } 
                }
                
            }
            
            tact_pressed_cnt=0;
        }
        
        /* short function key action*/
        if(flag2.short_key==1 && flag2.battery_usb_in==0)
        {
            if(counter_battery_display>=60) // 20msec * 30   // ---------------->
            {
                counter_battery_display=0;
             
                battery_display2(3);

            }

        }
        
        /*  short key timeout after 5sec */
        if(flag2.short_key==1 && counter_short_key_off >= 250 )
        {
            flag2.short_key=0;
            LED_25=LED_50=LED_75=LED_100=OFF;
        
        }
        /* sos funtion key action */
        if(flag1.KeySosAction==1)
        {

            switch(sos_status)
            {
                case 0: // init
                    
                    tick_1m_sos=0;
                    sos_status=1;
                    sos_s_cnt=0;
                    break;
                case 1: // 's' signal toggle as 200msec interval
                    if(tick_1m_sos>=200) // 200msec
                    {
                        tick_1m_sos=0;
                        led_toggle(last_main_dac_vol);
                        if(++sos_s_cnt >= 6)
                        {
                            sos_s_cnt=0;
                            sos_status=2;
                        }
                    }
            
                    break;
                case 2: // led off 660msec
            
                    if(tick_1m_sos>=660)
                    {
                      tick_1m_sos=0;
                      sos_status=3;
                    }
                    break;
                case 3:  // 'o' signal toggle as 330msec interval
            
                    if(tick_1m_sos>=330)  //330msec
                    {
                        tick_1m_sos=0;
                        led_toggle(last_main_dac_vol );
                        if(++sos_s_cnt >= 6)
                        {
                            sos_s_cnt=0;
                            sos_status=4;
                        }
                    }
                    break;
                case 4: // led off 660 msec
                    if(tick_1m_sos>=660)
                    {
                      sos_status=0;
                    }
                    break;
            
                default:
                    break;
            
            }
            
        }
        
        /* usb/ac adaptor in check */
        if(tick_1m_loop >= 600  /* && flag2.timer_sleeping==0 */)  //100m
        {
            tick_1m_loop=0;
           
            
            //if(0)
            if(ADAPTOR_INPUT==ON)
            {
            
                // if short key is pressed already 
                if(flag2.short_key==1)
                {
                    LED_25=LED_50=LED_75=LED_100=OFF;
                    flag2.short_key=0;

                }
                if(flag3.adaptor_init==0)
                {
                    LED_CHARGE^=1;
                    if(CHARGE_INPUT==ON && DONE_INPUT==OFF)
                    {
                        LED_CHARGE=ON;
                        flag3.adaptor_init=1;
                        
                        flag3.usb_in_display=1;  //a
                        
                        flag2.battery_usb_in=1;
                    }
                }
                else
                {
                    
                    //if(DONE_INPUT==ON) { LED_CHARGE=OFF; LED_25=OFF; LED_50=OFF; LED_75=OFF; LED_100=ON; }
                    //else 
                    {
                        if(flag3.usb_in_display==1)
                        {
                           
                            battery_display(4);
                        }
                       
                    }  
                    
                    // check battery charge or done from stat1 of BQ24100
                    if(flag2.battery_usb_in==1)
                    {
                        #if 0
                        if(CHARGE_INPUT==ON && flag3.charge_full==0) LED_CHARGE=ON;
                        else if(CHARGE_INPUT==OFF) LED_CHARGE=OFF;
                        else if(flag3.charge_full==1) LED_CHARGE=OFF;
                        #endif

                        
                        //if(CHARGE_INPUT==ON && flag3.charge_full==0) LED_CHARGE=ON;
                        if(CHARGE_INPUT==OFF) LED_CHARGE=OFF;
                        else if(flag3.charge_full==1) LED_CHARGE=OFF;
                        else {LED_CHARGE=ON; LED_100=OFF;}
                    
                    }
                    //if(LED_CHARGE==OFF && DONE_INPUT==OFF ) LED_CHARGE=ON;

                    //test
                    //if(CHARGE_INPUT==OFF)    LED_CHARGE^=1;
                }
                
                //TEST
                //if(CHARGE_INPUT==OFF)    LED_25^=1;


            }
            else
            {
                if(flag2.battery_usb_in==1)
                {
                    battery_display(0);
                    LED_CHARGE=OFF;
                    LED_100=OFF;
                    flag2.battery_usb_in=0;
                    led_status=0;
                    flag3.adaptor_init=0;
                    
                }
            }
         
            
            
        }

        /* sleeping timer action */ 
        if(flag2.timer_sleeping==1)  // 5min  timer
        {

            if(counter_timer_off_blinking>=20 && timer_off_loop!=0)
            {

                counter_timer_off_blinking=0;
                MAIN_LED_EN^=1;
                MOOD_LED_EN^=1;
                
                //LED_25^=1;
                //LED_50^=1;
                //LED_75^=1;
                                    
                if(MOOD_LED_EN==OFF) MOOD_DAC_OUT=0;
                else if(MOOD_LED_EN==ON) MOOD_DAC_OUT=last_mood_dac_vol;
                timer_off_loop--;
                

            }
            if(counter_timer_off >= 50*60*5*sleep_timer_count) // 5min timer or 10min timer
            //if(counter_timer_off>=50*10)  // 10 sec timer for test
            {
                // led off
                MAIN_DAC_OUT=0;
                
                MOOD_DAC_OUT=0;
                MAIN_LED_EN=OFF;
                MOOD_LED_EN=OFF;

                if(flag2.short_key==1) flag2.short_key=0;
                LED_25=LED_50=LED_75=LED_100=OFF;
                flag2.timer_sleeping=0;
                
                counter_timer_off=0;
                // enter watchdog sleep mode
                if(flag2.battery_usb_in==0)
                    sleep_mode();

                //
                

            }
            
            else if(timer_off_loop==0 && flag2.short_key==0 &&  flag3.usb_in_display==0)
            {
                //test1=counter_timer_off/100;
                //test1=(unsigned char )((60 *test1)/(5*6*5*sleep_timer_count));
                test1=(unsigned short)(counter_timer_off/1000);
                test1=(unsigned short)((59 *test1)/(3*5*sleep_timer_count)); // 5*6=>30/10=3
                //if(test1>=60) LED_25=ON;
                if(counter_timer_off_blinking>=(60 - test1)  )
                
                //if(counter_timer_off_blinking>=60   )
                {
                    counter_timer_off_blinking=0;
                    LED_100^=1;

                }

            }
            // show display led for sleeping timeout running
            //else if(flag2.short_key==0 && LED_100==OFF && flag3.usb_in_display==0)
            //    LED_100=ON;


        }

#if 1
        /* review ir get key */
        if(tick_1m_remo > 50)
        {
            tick_1m_remo=0;

            if(ir_key_buf !=ir_key_data && ir_key_data!=0)
            {
                ir_key_buf=ir_key_data;
                ir_key_pressed=1;
                ir_key_data=0;
                ir_key_released_cnt=0;
                ir_key_released=0;
                counter_ir_key_2sec=0;
                counter_ir_main_sw=0;
                LED_CHARGE=ON;
                /* LED_25=ON; */
            }
            else if(ir_key_pressed==1 && ir_key_released==0)
            {
                if(ir_key_buf==ir_key_data) { ir_key_released_cnt=0; LED_CHARGE=ON;  }
                else { ir_key_data=0; LED_CHARGE=OFF; /* LED_25=OFF; */}


                /* key no need for release check */
                if((ir_key_data==IR_MAIN_UP_KEY) ||(ir_key_data==IR2_MAIN_UP_KEY) ) 
                { 
                    if(MAIN_DAC_OUT < 200 ) MAIN_DAC_OUT+=5; MAIN_LED_EN=ON; 
                    last_main_dac_vol=MAIN_DAC_OUT;
                }
                else if((ir_key_data==IR_MAIN_DOWN_KEY)||(ir_key_data==IR2_MAIN_DOWN_KEY) ) 
                { 
                    if(MAIN_DAC_OUT >= 5) MAIN_DAC_OUT-=5; 
                    if(MAIN_DAC_OUT < 5) MAIN_DAC_OUT=0;
                    if(MAIN_DAC_OUT==0) MAIN_LED_EN=OFF;
                    else   MAIN_LED_EN=ON; 

                    last_main_dac_vol=MAIN_DAC_OUT;

                }
                else if((ir_key_data==IR_MOOD_UP_KEY) || (ir_key_data==IR2_MOOD_UP_KEY)) 
                { 
                    if(MOOD_DAC_OUT < 24 ) MOOD_DAC_OUT+=1; MOOD_LED_EN=ON; 
                    last_mood_dac_vol=MOOD_DAC_OUT;


                }
                else if((ir_key_data==IR_MOOD_DOWN_KEY) || (ir_key_data==IR2_MOOD_DOWN_KEY)) 
                { 
                    if(MOOD_DAC_OUT >= 1) MOOD_DAC_OUT-=1; 
                    if(MOOD_DAC_OUT==0) MOOD_LED_EN=OFF;
                    else MOOD_LED_EN=ON;
                    last_mood_dac_vol=MOOD_DAC_OUT;

                }

                /* key need for release check */
                if(ir_key_released_cnt >= 7)  //single key release check
                {
                    ir_key_released=1;
                    ir_key_pressed=0;
                    
                    if(ir_key_buf==IR_MAIN_KEY || ir_key_buf==IR2_MAIN_KEY)
                    {

                        if(flag3.main_sw_test==1)
                        {
                           flag3.main_sw_test=0; 
                           factory_test(6);

                           //flag3.my_mood=1;
                           //my_state=0;
                           
                        }
                        else
                        {
                            if(flag1.KeySosAction==1)
                            {
                                //LED_100=OFF;
                                flag1.KeySosAction=0;
                                MAIN_LED_EN=OFF;
                            }
                            else
                            {
                                MAIN_LED_EN^=1; 
                            }
                        }
                    }
                    
                    
                    if((ir_key_buf==IR_MOOD_KEY) || (ir_key_buf==IR2_MOOD_KEY))  
                    {
                        MOOD_LED_EN^=1; 
                        if(MOOD_LED_EN==OFF)  MOOD_DAC_OUT=1; 
                        else MOOD_DAC_OUT=last_mood_dac_vol; 
                        
                    }    
                    
                    if( (ir_key_buf==IR_FUNCTION_KEY) || (ir_key_buf==IR2_FUNCTION_KEY)) 
                    { 
                        //LED_75^=1; 
                        ir_function_key_repeat++; 
                        if(ir_function_key_repeat==1) counter_ir_repeat_key=0; 
                        ir_function_pressed=1;
                    }
                    //else ir_function_key_repeat=0;
                    
                    ir_key_buf=0;
                    ir_repeat_key=0;  //
                    //if(ir_key_buf==IR2_MAIN_KEY) LED_100=ON;
                    counter_ir_key_2sec=0;
                    counter_ir_main_sw=0;
                }
                #if 1
                if(counter_ir_main_sw >=90)
                {
                    counter_ir_main_sw=0;
                    flag3.main_sw_test=1;
                }
                #endif
                if(counter_ir_key_2sec >=80  && flag1.KeySosAction==0 && flag1.main_sw==1 )
                {
                    counter_ir_key_2sec=0;

                    if( (ir_key_buf==IR_FUNCTION_KEY) || (ir_key_buf==IR2_FUNCTION_KEY)) 
                    {
                        //flag3.ir_sos=1;
                        flag1.KeySosAction=1;
                        flag1.KeySosFound=1;

                        sos_status=0;
                        flag1.LedToggle=0;

                        MOOD_DAC_OUT=0; // hardware pb ?
                        MOOD_LED_EN=OFF;
                        saved_last_main_dac_vol=last_main_dac_vol;
                        last_main_dac_vol= 200; // start with max dac
                    }

                }

                ir_key_data=0;
            }
            if(ir_function_pressed==1 && counter_ir_repeat_key >= 16+7)  // double(repeat) key check
            {
                ir_function_pressed=0;
                
                if(ir_function_key_repeat==1) 
                { 
                    if(flag1.KeySosFound==1)
                    {
                        flag1.KeySosFound=0;
                        if(flag2.short_key==1) 
                        {
                            LED_25=LED_50=LED_75=LED_100=OFF;
                            flag2.short_key=0;
                        }    

                        if(flag2.timer_sleeping==1) 
                        {
                            flag2.timer_sleeping=0;
                            sleep_timer_count=0;
                            LED_100=OFF;
                        
                        }
                        LED_100=OFF;

                    }
                    else
                    {
                        if(flag1.KeySosAction==1)
                        {
                            //LED_75=OFF;
                            
                            flag1.KeySosAction=0;
                             MAIN_DAC_OUT=saved_last_main_dac_vol;
                             MAIN_LED_EN=ON;
                             if(flag2.battery_usb_in==1) LED_CHARGE=ON;

                        }
                        else
                        {
                            //LED_50^=1; 
                            if(flag2.battery_usb_in==0)
                            {
                                if(flag2.short_key==1) {
                                    LED_25=LED_50=LED_75=LED_100=OFF;
                                    flag2.short_key=0;
                                
                                }    
                                else
                                {
                                    flag2.short_key=1;
                                    counter_short_key_off=0;
                                }
                            }
                            else
                            {

                                if(flag3.usb_in_display==1) {
                                    flag3.usb_in_display=0;
                                    battery_display(0);
                                    LED_CHARGE=ON;
                                
                                }    
                                else
                                {
                                    //LED_100=ON;
                                    flag3.usb_in_display=1;
                                    led_status=1;
                                    
                                }


                            }
                        }
                    }

                }
                if(ir_function_key_repeat==2 && flag1.KeySosAction==0) 
                { 
                    //LED_100^=1; 
                    
                    if(flag2.battery_usb_in==1)
                    {
                        if(flag3.usb_in_display==1)
                        {
                        flag3.usb_in_display=0;
                        battery_display(0);
                        }

                    }
                  
                    if(flag1.main_sw==1 || flag1.mood_sw==1)
                    {
                        flag2.timer_sleeping=1;
                        timer_off_loop=4;
                        counter_timer_off=0;
                        sleep_timer_count++;
                        
                        if(flag2.short_key==1) {
                            LED_25=LED_50=LED_75=LED_100=OFF;
                            flag2.short_key=0;
                        }    
                    }
                  
                    
                    if(flag2.battery_usb_in==1)
                    {
                       LED_CHARGE=ON;

                    }
                }
                ir_function_key_repeat=0;

            }

        }    
#endif
        
#ifdef WATCHDOG_ENABLE


        if( (flag1.main_sw==0) && (flag1.mood_sw==0) && (flag2.battery_usb_in==0) /* && (watchdog_enable==1)*/)
        {
        
            if(flag2.short_key==1)
            {
                LED_25=LED_50=LED_75=LED_100=OFF;
                flag2.short_key=0;
            }
            enter_watchdog=1;
            LED_CHARGE=ON;
            if(counter_sleep_mode > 50 * 5)
            {
                sleep_mode();

            }
        }
        else
        {
            counter_sleep_mode=0;
            
            if(enter_watchdog==1)
            {
                LED_CHARGE=OFF;
                enter_watchdog=0;
            }

        }
        CLRWDT();
#endif
        

    }
    

    
    return (EXIT_SUCCESS);
}










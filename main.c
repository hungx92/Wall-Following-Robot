// Hung Le
// Vinh Vu 
// CECS 347 
// Project2

// SYSTICK -----------
// 1ms 

// ADC --------------- 
// SAMPLE ORDER  - PE1->PE4->PE5
// Right sensor  - PE1
// Left Sensor   - PE4
// Potentiometer - PE5 

// LCD ---------------
// RST - GPIO  - PA7
// C/S - SSI   - PA3
// DC  - SSI   - PA6
// DIN - SSI   - PA5
// CLK - SSI   - PA2

// PWM ---------------
// PWM1A - PB6
// PWM1B - PB7 
// Direction - PB0,1,2,3
// Forward   - PB2,3 
// Backward  - PB0,1
  
// 1. Pre-processor Directives Section
// Constant declarations to access port registers using 
// symbolic names instead of addresses
#include <stdint.h>
#include "Nokia5110.h"
#include "tm4c123gh6pm.h"
#include <math.h>

#define GPIO_LOCK_KEY           0x4C4F434B 

#define SYSCTL_RCC_USEPWMDIV    0x00100000  // Enable PWM Clock Divisor
#define SYSCTL_RCC_PWMDIV_M     0x000E0000  // PWM Unit Clock Divisor
#define SYSCTL_RCC_PWMDIV_2     0x00000000  // /2

#define ten_mil       16000
#define half_duty      8000
#define percent_30     4800
#define percent_60		 9600
#define percent_80		12800
#define percent_98		15680

#define direction      GPIO_PORTB_DATA_R
#define forward        0x0C
#define backward			 0x03
// 2. Declarations Section
//   Global Variables
uint16_t  time, percent_val;
unsigned long adc_PE4, adc_PE5, adc_PE1, PE4_cal, PE4_look, PE5_cal, PE5_look;
float speed; 
float adc_arr[14] = {2570.0,1400.0,1000.0,750.0,600.0,503.0,370.0,270.0,190.0,120.0,80.0,60.0,40.0,38.0};
float dist_arr[14] = {5.0,10.0,15.0,20.0,25.0,30.0,35.0,40.0,45.0,50.0,55.0,60.0,65.0,70.0};

//   Function Prototypes
float  Distance_Cal(int adc_val);
int    find_index(int adc);
float  Dist_table(float adc_val);

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

void ADC_Init298(void);
void ADC_In298(unsigned long *ain2, unsigned long *ain9, unsigned long *ain8);

void steering(int left_dist, int right_int, int speed);
float speedFromADC(int adc_val);

void PortB_Init(void);
void PWM1A_Init(uint16_t period, uint16_t duty);
void PWM1B_Init(uint16_t period, uint16_t duty);
void PWM1A_Duty (uint16_t duty);
void PWM1B_Duty (uint16_t duty);

void SysTick_Init(unsigned long period);
void SysTick_Handler(void);

void WaitForInterrupt(void);

// 3. Subroutines Section
// MAIN: Mandatory for a C Program to be executable
int main(void){    
	DisableInterrupts();
	Nokia5110_Init();
	ADC_Init298();                  //PE1,4,5 ADC input
	SysTick_Init(400000); 					//systick triggers every 1ms
																	//1000 samples/s 
	PortB_Init();
	PWM1A_Init(ten_mil,0);          
	PWM1B_Init(ten_mil,0); 
	
	Nokia5110_Clear();
	Nokia5110_OutString("* Robot-CT *");
 						 
	PWM1A_Duty(percent_30);					//start motors at 30%
	PWM1B_Duty(percent_30);
	direction = forward;
	
	EnableInterrupts(); 
  while(1){
				/*
		 		 0 1 2 3 4 5 6 7 8 9 T E 
				__________________________
		
				|*   R o b o t - C T   * | 0 		
				|L T				         R T | 1 
				|0 0                 0 0 | 2
				|      			             | 3
				|        Pot_Data        | 4
				|0 0 0 0           0 0 % | 5
				__________________________
			  */

				speed = speedFromADC(adc_PE1);
		
				PE4_cal  = Distance_Cal(adc_PE4);  //right 
				//PE4_look = Dist_table(adc_PE4);
		
				PE5_cal  = Distance_Cal(adc_PE5);  //left
				//PE5_look = Dist_table(adc_PE5);   
				percent_val = (speed / 16000) * 100;
				steering(PE5_cal, PE4_cal, speed); 
				
				Nokia5110_SetCursor(0,1);
				Nokia5110_OutString("LT        RT");
				
				if (PE5_cal > 50){
					Nokia5110_SetCursor(2,2);
					Nokia5110_OutString("Out");
				}else{
					Nokia5110_SetCursor(0,2);
					Nokia5110_OutUDec(PE5_cal);
				}
				if (PE4_cal > 50){
					Nokia5110_SetCursor(9,2);
					Nokia5110_OutString("Out");
				}else{
					Nokia5110_SetCursor(7,2);
					Nokia5110_OutUDec(PE4_cal);
				}
				
				Nokia5110_SetCursor(0,4);
				Nokia5110_OutString("  Pot-Data  ");
				
				Nokia5110_OutUDec(speed);
				Nokia5110_OutUDec(percent_val);
				Nokia5110_OutString("%");
  }
}
float speedFromADC(int adc_val){ //ratio 0-4095 , 0-15680
	float speed = 0; 
	float temp = 0;
	temp = adc_val / 100;
	speed = (temp / 40) * 15680;
	return speed; 
}	
void steering(int left_dist, int right_dist, int speed){
	if((left_dist < 20 & right_dist < 20) ){ //both see obsticle
			PWM1A_Duty(0);           					//stop
			PWM1B_Duty(0);                    //stop 
	}
	else if(left_dist > 38 & right_dist > 38){
			PWM1A_Duty(0);           					//stop
			PWM1B_Duty(0);                    //stop 
	}
	else if(left_dist<30){                //left sees obsticle 
			PWM1A_Duty(speed); 			          //left  go
			PWM1B_Duty(0);                		//right stop 
	}
	else if(right_dist<30){ 							//right sees obsticle 
			PWM1A_Duty(0);  									//left  stop 
			PWM1B_Duty(speed);  							//right go 
	}
	else{																	//no obsticle in range
			PWM1A_Duty(speed);  							//go
			PWM1B_Duty(speed);  							//go 
 	}

}
float Dist_table(float adc_val){ //calculate distance using ratio 
	int index;
	float distance;
	index = find_index(adc_val);
	distance = dist_arr[index]  + (5/(adc_arr[index]-adc_arr[index+1]))*(adc_arr[index]-adc_val); 
	return distance;
}

int find_index(int adc){ //find the index of adc from table that is closest 
												 //but less than input value 
	int index = 0;
	for (index = 0; index < 13; index++){
		if (index == 12){
			if(adc < adc_arr[index]){
				return 0; 
			}if (adc == adc_arr[index]){
				return index;
			}
		}else{
			if(adc == adc_arr[index]){
				return index;
			}if(adc < adc_arr[index] && adc > adc_arr[index+1]){
				return index;
			}
		}
	}
	return 0;
}

float Distance_Cal(int adc_val){ //calculate distance using equation 
	float dist = 0.0;
	double adc = adc_val;
	dist = -15.43 *log(adc) + 124.13;
	return dist;

}
//systick interrupts 1k/s 

// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(unsigned long period){
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
                              // enable SysTick with core clock and interrupts
  NVIC_ST_CTRL_R |= 0x07;
}

void SysTick_Handler(void){
	ADC_In298(&adc_PE1,&adc_PE4, &adc_PE5);								//3. clear flag
	time++;
}

void ADC_Init298(void){
	volatile unsigned long delay;
	SYSCTL_RCGCGPIO_R |= 0x10;							  //1. clock for port E
	while ((SYSCTL_RCGCGPIO_R&0x10) ==0 ){};  //wait
	
	GPIO_PORTE_DIR_R 	 &= ~0x32;							//2. PE1,PE4,PE5 inputs
	GPIO_PORTE_AFSEL_R |=  0x32;							//3. en alt func PE1,4,5
	GPIO_PORTE_DEN_R	 &=	~0x32;							//4. dis digital I/O PE1,4,5
	GPIO_PORTE_AMSEL_R |=  0x32;							//5. en analog func PE1,4,5
		
	SYSCTL_RCGCADC_R   |=  0x01;							//6. activate ADC0 
	SYSCTL_RCGC0_R |= 0x00010000;
	delay = SYSCTL_RCGCADC_R;									//wait to stabilize 
	delay = SYSCTL_RCGCADC_R;									//wait to stabilize
	delay = SYSCTL_RCGCADC_R;									//wait to stabilize
		
	ADC0_PC_R		  =  0x01;										//7. 125k max sampling rate
	ADC0_SSPRI_R  =  0x3210;									//8. seg 1 pri0 		
	ADC0_ACTSS_R &= ~0x0004;									//9. dis seq 1
	ADC0_EMUX_R  &= ~0x0F00;									//10. seq1 is software trigger
	ADC0_SSMUX2_R =  0x0892;									//11. Ain8,9,2 (PE5,4,1)
	ADC0_SSCTL2_R =  0x0600; 									//12. no TS0, D0, yes IE0 END0
		
	ADC0_IM_R    &= ~0x0004;									//13. dis ss1 int
	ADC0_ACTSS_R |=  0x0004;									//14. en ss1		
}	
void PortB_Init(void){
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000002;
	delay = SYSCTL_RCGC2_R;				//delay
	GPIO_PORTB_LOCK_R = GPIO_LOCK_KEY;
  
	GPIO_PORTB_CR_R    |=  0x0F;
	GPIO_PORTB_AMSEL_R &=  0x0F;
	GPIO_PORTB_DIR_R   |=  0x0F;  //PB0,1,2,3 output
	GPIO_PORTB_AFSEL_R &= ~0x0F;  //Regular I/O
	GPIO_PORTB_PCTL_R  &= ~0x0F;  //GPIO
	GPIO_PORTB_PUR_R   &= ~0x0F;  //no pull-up res
	GPIO_PORTB_DEN_R   |=  0x0F;
}
void PWM1A_Init(uint16_t period, uint16_t duty){
	volatile unsigned long delay;
	SYSCTL_RCGCPWM_R  |= 0x01;           // 1) activate M0
  SYSCTL_RCGCGPIO_R |= 0x02;           // 2) activate port B
	delay = SYSCTL_RCGCGPIO_R;
	//while((SYSCTL_PRGPIO_R&0x01) == 0){}
		
	GPIO_PORTB_AFSEL_R |=  0x40;         //PB6 alt funct	
	GPIO_PORTB_PCTL_R  &= ~0x0F000000;  
	GPIO_PORTB_PCTL_R	 |=  0x04000000;	 //PB6 as PWM0
	
	GPIO_PORTB_AMSEL_R &= ~0x40;         //disable analog PB6
	GPIO_PORTB_DEN_R   |=  0x40;				 //digital I/O PB6
	
	SYSCTL_RCC_R = 0x00100000 |          // 3) use PWM divider
      (SYSCTL_RCC_R & (~0x000E0000));  // configure for /2 divider	
			
	PWM0_0_CTL_R  = 0;									 //reload down-count mode									
	PWM0_0_GENA_R = 0xC8;                //low on LOAD, high on CMPA
		
	PWM0_0_LOAD_R  = period - 1;         
	PWM0_0_CMPA_R  = duty - 1;	
	PWM0_0_CTL_R  |= 0x00000001;				 //start PWM0
	PWM0_ENABLE_R |= 0x00000001;         //enable PB6, M0PWM0
}
void PWM1A_Duty (uint16_t duty){
	PWM0_0_CMPA_R = duty-1;
}
void PWM1B_Init(uint16_t period, uint16_t duty){
	volatile unsigned long delay;
	SYSCTL_RCGCPWM_R  |= 0x01;
	SYSCTL_RCGCGPIO_R |= 0x02;
	delay = SYSCTL_RCGCGPIO_R;
	
	GPIO_PORTB_AFSEL_R |=  0x80;				 //PB7 alt funct
	GPIO_PORTB_PCTL_R  &= ~0xF0000000;   
	GPIO_PORTB_PCTL_R  |=  0x40000000;   //PB7 as PWM0
	GPIO_PORTB_AMSEL_R &= ~0x80;         //disable analog PB7
	GPIO_PORTB_DEN_R   |=  0x80;         //digital I/O PB7
	
	SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV; // 3) use PWM divider
  SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M; //    clear PWM divider field
  SYSCTL_RCC_R += SYSCTL_RCC_PWMDIV_2;  //    configure for /2 divider
	
	PWM0_0_CTL_R  = 0;                    //reload down-count mode	
	PWM0_0_GENB_R = 0xC08;                //low on LOAD, high on CMPA
	
	PWM0_0_LOAD_R  = period - 1;
	PWM0_0_CMPB_R  = duty - 1;
	PWM0_0_CTL_R  |= 0x00000001;				 	//start PWM0
	PWM0_ENABLE_R |= 0x00000002;         	//enable PB7, M0PWM1
}

void PWM1B_Duty (uint16_t duty){
	PWM0_0_CMPB_R = duty-1;
}
unsigned long median(unsigned long u1, unsigned long u2, unsigned long u3){
unsigned long result;
  if(u1>u2)
    if(u2>u3)   result=u2;     // u1>u2,u2>u3       u1>u2>u3
      else
        if(u1>u3) result=u3;   // u1>u2,u3>u2,u1>u3 u1>u3>u2
        else      result=u1;   // u1>u2,u3>u2,u3>u1 u3>u1>u2
  else
    if(u3>u2)   result=u2;     // u2>u1,u3>u2       u3>u2>u1
      else
        if(u1>u3) result=u1;   // u2>u1,u2>u3,u1>u3 u2>u1>u3
        else      result=u3;   // u2>u1,u2>u3,u3>u1 u2>u3>u1
  return(result);
}
// This function samples AIN2 (PE1), AIN9 (PE4), AIN8 (PE5) and
// returns the results in the corresponding variables.  Some
// kind of filtering is required because the IR distance sensors
// output occasional erroneous spikes.  This is a median filter:
// y(n) = median(x(n), x(n-1), x(n-2))
// Assumes: ADC initialized by previously calling ADC_Init298()
void ReadADCMedianFilter(unsigned long *ain2, unsigned long *ain9, unsigned long *ain8){
  //                   x(n-2)        x(n-1)
  static unsigned long ain2oldest=0, ain2middle=0;
  static unsigned long ain9oldest=0, ain9middle=0;
  static unsigned long ain8oldest=0, ain8middle=0;
  // save some memory; these do not need to be 'static'
  //            x(n)
  unsigned long ain2newest;
  unsigned long ain9newest;
  unsigned long ain8newest;
  ADC_In298(&ain2newest, &ain9newest, &ain8newest); // sample AIN2(PE1), AIN9 (PE4), AIN8 (PE5)
  *ain2 = median(ain2newest, ain2middle, ain2oldest);
  *ain9 = median(ain9newest, ain9middle, ain9oldest);
  *ain8 = median(ain8newest, ain8middle, ain8oldest);
  ain2oldest = ain2middle; ain9oldest = ain9middle; ain8oldest = ain8middle;
  ain2middle = ain2newest; ain9middle = ain9newest; ain8middle = ain8newest;
}

//------------ADC_In298------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: three 12-bit result of ADC conversions
// Samples AIN8, AIN9, and AIN2
// 125k max sampling
// software trigger, busy-wait sampling
// data returned by reference
// ain2 (PE1) 0 to 4095
// ain9 (PE4) 0 to 4095
// ain8 (PE5) 0 to 4095
void ADC_In298(unsigned long *ain2, unsigned long *ain9, unsigned long *ain8){
  ADC0_PSSI_R = 0x0004;            // 1) initiate SS2
  while((ADC0_RIS_R&0x04)==0){};   // 2) wait for conversion done
  *ain2 = ADC0_SSFIFO2_R&0xFFF;    // 3A) read first result
  *ain9 = ADC0_SSFIFO2_R&0xFFF;    // 3B) read second result
  *ain8 = ADC0_SSFIFO2_R&0xFFF;    // 3C) read third result
  ADC0_ISC_R = 0x0004;             // 4) acknowledge completion
}
/*
float adc_arr[13] = {1531.0,1055.0,843.0,711.0,611.0,533.0,467.0,430.0,300.0,263.0,234.0,210.0,200.0};
float dist_arr[13] = {10.0,15.0,20.0,25.0,30.0,35.0,40.0,45.0,50.0,55.0,60.0,65.0,70.0};

float adc_arr[13] = {1400.0,1000.0,75;;0.0,600.0,503.0,370.0,270.0,190.0,120.0,80.0,60.0,40.0,38.0};
float dist_arr[13] = {10.0,15.0,20.0,25.0,30.0,35.0,40.0,45.0,50.0,55.0,60.0,65.0,70.0};
*/
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

//magento registers
//hard iron registers
#define OFFSET_X_REG_L_M 0x45
#define OFFSET_X_REG_H_M 0x46
#define OFFSET_Y_REG_L_M 0x47
#define OFFSET_Y_REG_H_M 0x48
#define OFFSET_Z_REG_L_M 0x49
#define OFFSET_Z_REG_H_M 0x4A


//config registers
#define CFG_REG_A_M 0x60
#define CFG_REG_B_M 0x61
#define CFG_REG_C_M 0x62


//interrupt config registers
#define INT_CTRL_REG_M 0x63
#define INT_SOURCE_REG_M 0x64
#define INT_THS_L_REG_M 0x65
#define INT_THS_H_REG_M 0x66
#define STATUS_REG_M 0x67


//output registers
#define OUTX_L_REG_M 0x68
#define OUTX_H_REG_M 0x69
#define OUTY_L_REG_M 0x6A
#define OUTY_H_REG_M 0x6B
#define OUTZ_L_REG_M 0x6C
#define OUTZ_H_REG_M 0x6D


#define WHO_AM_I_M 0x4F





struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
  //structure that holds all the RCC registers that are responsible for managing the clock and reset functionalities of peripherals
  //with some of these registers we are able to turn on peripherals that are turned off initially

};



#define RCC ((struct rcc*) 0x40023800)

struct gpio{
	volatile uint32_t MODER,OTYPER,OSPEEDR,PUPDR,IDR,ODR,BSRR,LCKR,AFR[2];
};

#define GPIOD ((struct gpio *) 0x40020C00)
#define GPIOB ((struct gpio *) 0x40020400)
#define GPIOA ((struct gpio *) 0x40020000)
#define GPIOE ((struct gpio *) 0x40021000)

struct nvic{
	volatile uint32_t ISER[8U],RESERVED0[24U],ICER[8U], RESERVED1[24U],  ISPR[8U], RESERVED2[24U],
	ICPR[8U], RESERVED3[24U], IABR[8U], RESERVED4[56U], IP[240U], RESERVED5[644U], STIR;
};

#define NVIC ((struct nvic *) 0xE000E100)

struct i2c{
	volatile uint32_t CR1,CR2,OAR1,OAR2,DR,SR1,SR2,CCR,TRISE,FLTR;
};

#define I2C1 ((struct i2c*) 0x40005400)

struct systick{
	volatile uint32_t CTRL, LOAD,VAL,CALIB;
};
#define SYSTICK ((struct systick*) 0xE000E010)


#define magneto_addr 0b0011110
volatile uint16_t magneto_addr_read = 0x3D;
volatile uint8_t magneto_addr_write = 0x3C;

volatile uint8_t who_am_i = 0x4F;

#define FREQ 16000000 //clock freq



static inline void systick_init(){
	RCC->APB2ENR |= (1UL << 14); //write all the reference manual pages and shit here on comments
	SYSTICK->LOAD = 15999;
	SYSTICK->VAL = 0;
	SYSTICK->CTRL |= (1UL << 0) | (1UL << 1) | (1UL << 2);



}

volatile uint32_t counter;

void SysTick_Handler(void)
{
	//the original way was to increment a counter each time a clock cycle occured once that counter reached a certain number
	//of miliseconds  (16 MHZ means there is 16 million clock cycles per second)
	// the state of an LED changed
	//either rework that do it ur way without the using a expired_timer function
	//OR
	//find a diff way on the internet to use systick to

	// pass a different number of ticks to the initialization systick_init so that every interrupt the LED changes state
	// instead of using processor clock use an external clock that will trigger an interrupt every however wanted seconds
	// make it configurable so it can be easily changable.
	counter++;
}

void delay(uint32_t time) {
	//time is in ms
	counter =0;
	while(counter < time);
}

bool fix_bus(void){
	if(!(I2C1->SR2 & (1U<<1))) {
		return true;
	}

	I2C1->CR1 &=~(1U<<0);
	GPIOB->MODER &=~(1U<<12);
	GPIOB->MODER &=~(1U<<18);
	delay(10);
	if(!(GPIOB->IDR &(1U<<6))) {
		return false;
	}

	for(int i =0; i<12;i++){
		GPIOB->MODER |= (1U<<12);
		GPIOB->ODR &=~(1U<<6);
		delay(10);
		GPIOB->MODER &=~(1U<<12);
		delay(10);


		if(GPIOB->IDR & (1U<<9)) {
			break;
		}
	}

	if(!(GPIOB->IDR & (1U<<9))) {
		return false;
	}


	return ((GPIOB->IDR & (1U<<6)) && (GPIOB->IDR &(1U<<9)));

}


void I2C_init(void) {

//GPIOE to output high
	RCC->AHB1ENR |=(1U<<4); //enable GPIOE

	GPIOE->MODER |=(1U<<4); //turn pin E2 into output this corresponds to CS_XL of sensor
	GPIOE->MODER |=(1U<<10); //turn pin E5 into output this corresponds to CS_MAG of sensor

	GPIOE->BSRR |=(1U<<2); //set E2 high
	GPIOE->BSRR |=(1U<<5); //set E5 high

	//turning these 2 GPIOE pins high is needed in order to enable communication with the sensor using I2C


//GPIOB alternate function, open drain, pullup
	RCC->AHB1ENR |=(1U<<1);

	//reset
	GPIOB->MODER &=~(2U<<12);
	GPIOB->MODER &=~(2U<<18);

	GPIOB->MODER |=(2U<<12); //alternate function mode for SCL
	GPIOB->MODER |=(2U<<18); //alternate function mode for SDA


	GPIOB->PUPDR &=~(1U<<12); //pullup for SCL
	GPIOB->PUPDR &=~(1U<<18); //pullup for SDA

	GPIOB->PUPDR |=(1U<<12); //pullup for SCL
	GPIOB->PUPDR |=(1U<<18); //pullup for SDA


	GPIOB->OTYPER &=~(1U<<6); //open drain for SCL
	GPIOB->OTYPER &=~(1U<<9); //open drain for SDA

	GPIOB->OTYPER |=(1U<<6); //open drain for SCL
	GPIOB->OTYPER |=(1U<<9); //open drain for SDA


	GPIOB->AFR[0] &= ~(4U<<24);
	GPIOB->AFR[0] |= (4U<<24);

	GPIOB->AFR[1] &= ~(4U<<4);
	GPIOB->AFR[1] |= (4U<<4);




	RCC->APB1RSTR |=(1U<<21); //reset I2C
	RCC->APB1RSTR &=~(1U<<21); //reset I2C

	RCC->APB1ENR |=(1U<<21); //enable I2C clock

	I2C1->CR1 &=~(1U<<0); //turn off I2C


	I2C1->CR1 |=(1U<<15); //reset I2C
	I2C1->CR1 &=~(1U<<15); //clear reset bit



	if(!fix_bus()){
		exit(1); //if fails fuck it quit
	}
	else{
		if(!(GPIOB->MODER & (2U<<12)) && !(GPIOB->MODER & (2U<<18))) {
			GPIOB->MODER &=~(2U<<12);
			GPIOB->MODER &=~(2U<<18);

			GPIOB->MODER |=(2U<<12); //alternate function mode for SCL
			GPIOB->MODER |=(2U<<18); //alternate function mode for SDA
		}


		//I2C is found on the APB1 bus
		//the formula to find the clock frequency of the APB1 bus is: SYSCLK/APB1 Prescaler
		//we first need to determine the clock source for the system clock
		//this can be either:
		//High-speed Internal Oscillator (HSI)
		//High-speed External Oscillator (HSE)
		//Phase-locked Loop output (PLL)
		//to find this information we can look into the RCC register CFGR and look at the SW 0:1 bits
		//if the bits are set to:
		//00, HSI is used
		//01, HSE is used
		//10, PLL is used
		//11, not allowed
		//to figure out what the microcontroller is using we can run the debug and look into the registers
		//in debug option
		//in this case when we do that we SW 0:1 bits of CFGR show: 00 which is HSI
		// in the file stm32f4xx_hal_rcc one of the first few lines:
		//After reset the device is running from Internal High Speed oscillator (HSI 16MHz)
	    //after finding out the system clock speed of 16 MHz we need to find the prescaler
		//the prescaler can be found in the RCC register CFGR using the bits PPRE1
		//the PPRE1 register corresponds to the prescaler for the APB1 bus
		//the PPRE1 bits are found on bits: 10:12
		//0xx means the clock is not divided and the prescaler is 1
		//100 prescaler 2
		//101 prescaler 4
		//110 prescaler 8
		//111 prescaler 16
		//upon reset the prescaler value is 1 because the bits are 000

		//now that we know the speed of the bus I2C operates on we are able to set the
		//peripheral clock frequency which is required to be set to the speed of the APB1 bus
		//this register is responsible for generating the correct timings for communication across peripherals on I2C
		//it is used to set the correct timing for generating the SCL ( signal clock line)
		I2C1->CR2 = 16;

		//there are different I2C speeds, at normal speeds I2C operates at 100 KHz
		//and at fast speeds it operates at 400 KHz
		//for now we are interested in the normal mode only.

		//the CCR register controls the clock
		//the formula to work out CCR is APB1 clock frequency/(2x I2C speed)
		//in this case it will be:
		//16 MHz/(2x100 KHz)
		//which is 16 000 KHz / 200 KHz
		//this will result in 80
		//CCR is responsible for setting the periods of high and low periods on the SCL line
		//this makes it responsible for communication speed of the 2 peripherals
		I2C1->CCR =80;

		//TRISE is used to set a rise time
		//the rise time is used to tell the signal how long it should take from the to transfer from low to high
		//the formula to work out TRISE is
		//(APB1 clock speed/1Mhz) +1
		//which will be (16/1) +1 = 17
		I2C1->TRISE = 17;

		//finally turn the protocol on

		I2C1->CR1 |=(1U<<0);
	}





}

volatile uint32_t data;

void read_who(void){
	uint32_t temp;
	I2C1->CR1 |=(1U<<10); //enable ACK;

	I2C1->CR1 |=(1U<<8); //generate start bit
	while(!(I2C1->SR1 & (1U<<0))); //wait for start bit to be generated

	I2C1->DR = magneto_addr_write;
	while(!(I2C1->SR1 & (1U<<1))); //wait for ADDR flag to get set;
	temp = I2C1->SR1;
	temp = I2C1->SR2;

	I2C1->DR = WHO_AM_I_M;
	while(!(I2C1->SR1 & (1U<<7))); //wait for TXe data register to not be empty;

	I2C1->CR1 |=(1U<<8); //repeated start condition to indicate we want to read from a register
	while(!(I2C1->SR1 & (1U<<0))); //wait for start bit to be generated

	I2C1->DR = magneto_addr_read; //send reading address
	while(!(I2C1->SR1 & (1U<<1))); //wait for ADDR flag to get set;
	I2C1->CR1 &=~(1U<<10); //we want to disable ACK before the final message is sent
	temp = I2C1->SR1;
	temp = I2C1->SR2;


	I2C1->CR1 |=(1U<<9);
	while(!(I2C1->SR1 & (1U<<6))); //wait until the RXe register is empty

	data = I2C1->DR;




}

void send(uint8_t reg_addr, uint8_t value){
	uint32_t temp;
	I2C1->CR1 |=(1U<<10); //enable ACK

	I2C1->CR1 |=(1U<<8); //start
	while(!(I2C1->SR1 & (1U<<0))); //wait for start bit to get set

	I2C1->DR = magneto_addr_write;
	while(!(I2C1->SR1 & (1U<<1))); //wait for ADDR flag to get set
	temp = I2C1->SR1; //clear addr
	temp = I2C1->SR2; //clear addr
	while(!(I2C1->SR1 & (1U<<7))); //dont transfer anything untill TXe is empty
	I2C1->DR = reg_addr; //address of hte register
	while(!(I2C1->SR1 & (1U<<7))); //wait for TXE to be empty
	I2C1->DR = value; //value we want to send to the register


	while(!(I2C1->SR1 & (1U<<2))); //wait for byte transfer bit to get set
	I2C1->CR1 |=(1U<<9); //stop condition


}

uint8_t read(uint8_t reg_addr) {
	uint32_t temp;
	I2C1->CR1 |=(1U<<10); //enable ACK;

	I2C1->CR1 |=(1U<<8); //generate start bit
	while(!(I2C1->SR1 & (1U<<0))); //wait for start bit to be generated

	I2C1->DR = magneto_addr_write;
	while(!(I2C1->SR1 & (1U<<1))); //wait for ADDR flag to get set;
	temp = I2C1->SR1;
	temp = I2C1->SR2;

	I2C1->DR = reg_addr;
	while(!(I2C1->SR1 & (1U<<7))); //wait for TXe data register to not be empty;

	I2C1->CR1 |=(1U<<8); //repeated start condition to indicate we want to read from a register
	while(!(I2C1->SR1 & (1U<<0))); //wait for start bit to be generated

	I2C1->DR = magneto_addr_read; //send reading address
	while(!(I2C1->SR1 & (1U<<1))); //wait for ADDR flag to get set;
	I2C1->CR1 &=~(1U<<10); //we want to disable ACK before the final message is sent
	temp = I2C1->SR1;
	temp = I2C1->SR2;


	I2C1->CR1 |=(1U<<9);
	while(!(I2C1->SR1 & (1U<<6))); //wait until the RXe register is empty

	data = I2C1->DR;
	return data;

}

void reset_lsm(){
	read(CFG_REG_A_M);
	send(CFG_REG_A_M,0x0);
	read(CFG_REG_A_M);
	send(CFG_REG_A_M,0x20);
	read(CFG_REG_A_M);
	send(CFG_REG_A_M,0x40);
	read(CFG_REG_A_M);
	send(CFG_REG_A_M,0x0);
}

void self_test(){

	uint16_t OUTX_ST =0, OUTY_ST =0, OUTZ_ST=0;
	uint16_t temp =0;
	uint8_t status;



	send(CFG_REG_A_M,0x0C);
	send(CFG_REG_B_M,0x02);
	send(CFG_REG_C_M,0x10);

	delay(25);

	uint16_t OUTX_NOST =0;
	uint16_t OUTY_NOST =0;
	uint16_t OUTZ_NOST =0;


	//reason why it gets some outrageous value is because of the <<8 from other examples
	//could be going out of bounds or smth and causing errors
	//look more into that

	for(int i =0; i < 50; i++) {

		status = read(STATUS_REG_M);


        OUTX_NOST += read(OUTX_L_REG_M) + read(OUTX_H_REG_M);
        OUTY_NOST += read(OUTY_L_REG_M) + read(OUTY_H_REG_M);
        OUTZ_NOST += read(OUTZ_L_REG_M) + read(OUTZ_H_REG_M);



	}
	OUTX_NOST /=50;
	OUTY_NOST /=50;
	OUTZ_NOST /=50;


	send(CFG_REG_C_M,0x12);
	delay(65);

	status = read(STATUS_REG_M);


	//read registers to clear XYZDA bit
	read(OUTX_L_REG_M);
	read(OUTX_H_REG_M);

	read(OUTY_L_REG_M);
	read(OUTY_H_REG_M);

	read(OUTZ_L_REG_M);
	read(OUTZ_H_REG_M);

	for(int i=0; i<50;i++) {
		status = read(STATUS_REG_M);
        OUTX_ST += read(OUTX_L_REG_M) + read(OUTX_H_REG_M);
        OUTY_ST += read(OUTY_L_REG_M) + read(OUTY_H_REG_M);
        OUTZ_ST += read(OUTZ_L_REG_M) + read(OUTZ_H_REG_M);

	}
	OUTX_ST /=50;
	OUTY_ST /=50;
	OUTZ_ST /=50;

	const uint16_t abs_x = abs(OUTX_ST - OUTX_NOST);
	const uint16_t abs_y = abs(OUTY_ST - OUTY_NOST);
	const uint16_t abs_z = abs(OUTZ_ST - OUTZ_NOST);


	const bool x_valid = (abs_x >15 && abs_x < 500);
	const bool y_valid = (abs_y >15 && abs_y < 500);
	const bool z_valid = (abs_z >15 && abs_z < 500);

	if(!x_valid || !y_valid || !z_valid) {
		GPIOD->ODR |=(1U<<13);
		//orange for bad;
	}
	else {
		GPIOD->ODR |=(1U<<12);
		//green for good;
	}

	send(CFG_REG_C_M,0x10);
	send(CFG_REG_C_M,0x83);


}



void single_read(char *buf){
	send(CFG_REG_A_M, 0x00);
	send(CFG_REG_A_M, 0x01);

	uint8_t check= 0;
	while(!(check & 1U<<3)){
		check = read(STATUS_REG_M);
	}

	buf[0] = read(OUTX_L_REG_M);
	buf[1] = read(OUTX_H_REG_M);
	buf[2] = read(OUTY_L_REG_M);
	buf[3] = read(OUTY_H_REG_M);
	buf[4] = read(OUTZ_L_REG_M);
	buf[5] = read(OUTZ_H_REG_M);





}

uint16_t convert_twos_comp(uint16_t val){
	uint16_t mask = 0x8000;

	if((val & mask) == 0 ) {
		return val;
	}
	else{
		return -(~val +1);
	}
}

int main(void) {
	RCC->AHB1ENR |= (1U<<3);

	GPIOD->MODER |=(1U<<26);
	GPIOD->MODER |=(1U<<24);
	GPIOD->MODER |=(1U<<28);
	//MUST DO NOW
	//reset the sensor using one of the registers
	//check register with reset bit to see if it needs to be cleared or not
	//do the self test
	//if it fails then god damn
	//try to collect data from it atleast once


	//after which try to collect data from it multiple times but that is maybe for another day


	systick_init();
	I2C_init();
	delay(100);

	reset_lsm();

	delay(100);

	read(STATUS_REG_M);
	self_test();
	read(STATUS_REG_M);
	read(WHO_AM_I_M);

	reset_lsm();

	char buf[6] ={0};
	single_read(buf);

	uint16_t combined_X = buf[0] & buf[1];
	uint16_t combined_Y = buf[2] & buf[3];
	uint16_t combined_Z = buf[4] & buf[5];

	combined_X = convert_twos_comp(combined_X);
	combined_Y = convert_twos_comp(combined_Y);
	combined_Z = convert_twos_comp(combined_Z);

	uint16_t final_x = combined_X * 1.5;
	uint16_t final_y = combined_Y * 1.5;
	uint16_t final_z = combined_Z * 1.5;






	delay(100);
	if(read(WHO_AM_I_M) == 0x40) {
		GPIOD->ODR |=(1U<<14);
	}
	//add more delays throughout holy shit this works ahaha
	for(;;);
}

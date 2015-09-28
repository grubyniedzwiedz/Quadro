
#include <stdio.h>
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "math.h"

int inpwm[3];
	uint capture1=0;
	uint capture2=0;
	uint capture_ready=0;
	volatile uint16_t pilot_w;
	volatile float gxyzf[3];
	volatile float akxyzf[3];
	volatile int ster_pilot=0;

	volatile uint16_t pilot_w2;
	int inpwm2[3];
	uint capture12=0;
	uint capture22=0;
	uint capture_ready2=0;


//adresowanie gyro i adresy srejestrow
#define GYRO_ADDRESS_TR  0b11010111
#define GYRO_ADDRESS_REC 0b11010110

     int CTRL_REG1      = 0x20;
     int CTRL_REG4      = 0x23;
     int LOW_ODR        = 0x39;


//adresowanie ak i m oraz adresy rejestrow
#define AKM_ADDRESS_TR  0b00111011
#define AKM_ADDRESS_REC 0b00111010


//adresowanie barometru oraz adresy rejestrow
#define BAR_ADDRESS_TR  0b10111011
#define BAR_ADDRESS_REC 0b10111011

     	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 //Ustawienia diod
void init_LED(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13| GPIO_Pin_12| GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
}
																			//Ustawienia przycisku
void init_Button(void){

	GPIO_InitTypeDef GPIO_InitDef;


	    //Enable clock for GPOIA
	    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	    //Pin 0
	    GPIO_InitDef.GPIO_Pin = GPIO_Pin_0;
	    //Mode output
	    GPIO_InitDef.GPIO_Mode = GPIO_Mode_IN;
	    //Output type push-pull
	    GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
	    //With pull down resistor
	    GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_DOWN;
	    //50MHz pin speed
	    GPIO_InitDef.GPIO_Speed = GPIO_Speed_50MHz;

	    //Initialize pin on GPIOA port
	    GPIO_Init(GPIOA, &GPIO_InitDef);



}
																		//Ustawienia I2C i odczyt贸w z IMU
void init_I2C1(void){

	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for GPIOB
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/*
	 SCL1  PB6
	 SDA1  PB9
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9; // using PB6 i PB9
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors (external pull up resistor not needed now)
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB

	// Connect I2C1 pins to AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); // SDA

	// configure I2C1
	I2C_InitStruct.I2C_ClockSpeed = 400000; 		// 400 kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1

	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}

//Rozpoczecie komunikacji
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy anymore
	//!obejscie flagi zajetosci dla odczytu, poniewaz tak jest w data sheet
	if(direction == I2C_Direction_Transmitter){
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	}
	// Send I2C1 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	/* wait for I2C1 EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
     }
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}

}

//Wysyla bajt do slave
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// czeka na koniec transmisji
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

//Czyta zmienna z nadaniem bitu potwierdzenia
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	//Funkcja wysylajaca bit potwierdzenia po odbiorze bajtu
	//wlaczenie potwierdzenia ack
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	//czekanie na otrzymanie bajtu
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// odczytanie danych z rejestru I2C i wyslanie bitu potwierdzenia
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

//Czyta zmienna bez potwierdzenia odbioru (przed zakonczeniem komunikacji)
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	//wylacza potwiedzenie odbioru
	//!wysyla bit zakonczenia komunikacji
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// czekanie na otrzymanie bajtu
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// odczytanie bajtu z rejestru I2C i zwrocenie danych
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;

}

//wysyla bit stopu, co zwalnia magistrale
void I2C_stop(I2C_TypeDef* I2Cx){
	// wyslyla bit stopu
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void init_gyro(void){


	    //ustawianie parametr贸w urzadzenia
	     I2C_start(I2C1 ,GYRO_ADDRESS_TR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	     // Low_ODR = 0 (low speed ODR disabled)
	     I2C_write(I2C1, LOW_ODR);
	     I2C_write(I2C1,0x00);
	     I2C_stop(I2C1); // stop the transmission
	     I2C_start(I2C1 ,GYRO_ADDRESS_TR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	     // FS = 00 (+/- 250 dps full scale)  !!!changed 500dps
	     I2C_write(I2C1, CTRL_REG4);
	     I2C_write(I2C1,0x10);
	     I2C_stop(I2C1); // stop the transmission
	     I2C_start(I2C1 ,GYRO_ADDRESS_TR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	     // DR = 01 (200 Hz ODR); BW = 10 (50 Hz bandwidth); PD = 1 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
	     I2C_write(I2C1, CTRL_REG1);
	     I2C_write(I2C1, 0x6F);
	     I2C_stop(I2C1); // stop the transmission

	     I2C_start(I2C1 ,GYRO_ADDRESS_TR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	     //Digital high pass filter reference value
	     I2C_write(I2C1, 0x25);
	     I2C_write(I2C1, 0x00);
	     I2C_stop(I2C1); // stop the transmission


}

void init_akm(void){


	    //ustawianie parametr贸w urzadzenia

	// Accelerometer
    	 I2C_start(I2C1 ,AKM_ADDRESS_TR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
    	 // AODR = 0101 (50 Hz ODR); AZEN = AYEN = AXEN = 1 (all axes enabled)
    	 I2C_write(I2C1, 0x20);
    	 I2C_write(I2C1, 0x57);
    	 I2C_stop(I2C1); // stop the transmission
	     I2C_start(I2C1 ,AKM_ADDRESS_TR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	     // AFS = 1 (+/- 4 g full scale)
	     I2C_write(I2C1, 0x21);
	     I2C_write(I2C1, 0x08);
	     I2C_stop(I2C1); // stop the transmission



	// Magnetometer
	     I2C_start(I2C1 ,AKM_ADDRESS_TR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	     // M_RES = 11 (high resolution mode); M_ODR = 001 (6.25 Hz ODR) CTRL5
	     I2C_write(I2C1, 0x24);
	     I2C_write(I2C1, 0x64);
	     I2C_stop(I2C1); // stop the transmission
	     I2C_start(I2C1 ,AKM_ADDRESS_TR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	     // MFS = 01 (+/- 4 gauss full scale)CTRL6
	     I2C_write(I2C1, 0x25);
	     I2C_write(I2C1, 0x20);
	     I2C_stop(I2C1); // stop the transmission
	     I2C_start(I2C1 ,AKM_ADDRESS_TR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	     // MLP = 0 (low power mode off); MD = 00 (continuous-conversion mode)CTRL7
	     I2C_write(I2C1, 0x26);
	     I2C_write(I2C1, 0x00);
	     I2C_stop(I2C1); // stop the transmission


}

void init_bar(void){


	    //ustawianie parametr贸w urzadzenia

	// Barometr
    	 I2C_start(I2C1 ,BAR_ADDRESS_TR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
    	 // PD = 1 (active mode);  ODR = 011 (12.5 Hz pressure & temperature output data rate)
    	 I2C_write(I2C1, 0x20);
    	 I2C_write(I2C1, 0xB0);
    	 I2C_stop(I2C1); // stop the transmission



}

void gyro_read(void){

	 	 	 	 uint16_t gxyz[3];

	 	 	 	 int gxyzi[3];




				//Odczyt
	    		I2C_start(I2C1 ,GYRO_ADDRESS_TR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	    		I2C_write(I2C1, 0xA8); // wyslanie adresu rejestru z danymi z zyroskopu; adres 28, ale jezeli pierwszy bit 1 to wlaczona autoinkrementacja

	    		I2C_start(I2C1, GYRO_ADDRESS_REC, I2C_Direction_Receiver); // start a transmission in Master receiver mode

	    		//received_data[2] = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission
	    		//I2C_stop(I2C1); // stop the transmission
	    		//8 wyzszych bitow podawanych pozniej odpowiada za znak 0xFF znaczy ujemne
	    			//Dla zakresu 卤250 dps LSB przetwarzania wynosi zatem 0,00875 dps



	    			//!wykalibrowac i przemnozyc prez 0,00875
	    		gxyz[0]=I2C_read_ack(I2C1) | I2C_read_ack(I2C1)<<8;

	    		gxyz[1]=I2C_read_ack(I2C1) | I2C_read_ack(I2C1)<<8;

	    		gxyz[2]=I2C_read_ack(I2C1) | I2C_read_nack(I2C1)<<8;

	    		I2C_stop(I2C1); // stop the transmission

	    		if (gxyz[0]>>15){
	    			gxyzi[0]=-65536+gxyz[0];
	    		}else{
	    			gxyzi[0]=gxyz[0];
	    		}
	    		if (gxyz[1]>>15){
	    			gxyzi[1]=-65536+gxyz[1];
	    		}else{
	    			gxyzi[1]=gxyz[1];
	    		}
	    		if (gxyz[2]>>15){
	    			gxyzi[2]=-65536+gxyz[2];
	    		}else{
	    			gxyzi[2]=gxyz[2];
	    		}


	    		gxyzf[0]=(int)(gxyzi[0]);
	    		gxyzf[0]=gxyzf[0]/65.5;

	    		gxyzf[1]=(int)(gxyzi[1]);
	    		gxyzf[1]=gxyzf[1]/65.5;

	    		gxyzf[2]=(int)(gxyzi[2]);
	    		gxyzf[2]=gxyzf[2]/65.5;


	    		//Kalibracja r阠zna
	    		gxyzf[0] = gxyzf[0]-3;


	    				//printf("GX: %d           ", (int)(gxyzf[0]*100.0f));
	    				//printf("GY: %d           ", (int)(gxyzf[1]*100.0f));
	    				//printf("GZ: %d\r\n", (int)(gxyzf[2]*100.0f));
	    				//printf("GX: %d           ", gxyz[0]);
	    				//printf("GY: %d           ", gxyz[1]);
	    				//printf("GZ: %d\r\n", gxyz[2]);

}

void ak_read(void){

	 	 	 	uint16_t akxyz[3];

	 	 	 	int akxyzi[3];


				//Odczyt
	    		I2C_start(I2C1 ,AKM_ADDRESS_TR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	    		I2C_write(I2C1, 0xA8); // wyslanie adresu rejestru z danymi z zyroskopu; adres 28, ale jezeli pierwszy bit 1 to wlaczona autoinkrementacja

	    		I2C_start(I2C1, AKM_ADDRESS_REC, I2C_Direction_Receiver); // start a transmission in Master receiver mode

	    		//ustawic obslug臋 zmiennoprzecinkowa i poprawnie odczytac format danych


	    		akxyz[0]=I2C_read_ack(I2C1) | I2C_read_ack(I2C1)<<8;

	    		akxyz[1]=I2C_read_ack(I2C1) | I2C_read_ack(I2C1)<<8;

	    		akxyz[2]=I2C_read_ack(I2C1) | I2C_read_nack(I2C1)<<8;

	    		I2C_stop(I2C1); // stop the transmission



	    		if (akxyz[0]>>15){
	    			akxyzi[0]=-65536+akxyz[0];
	    		}else{
	    		akxyzi[0]=akxyz[0];
	    		}
	    		if (akxyz[1]>>15){
	    			akxyzi[1]=-65536+akxyz[1];
	    		}else{
	    			akxyzi[1]=akxyz[1];
	    		}
	    		if (akxyz[2]>>15){
	    			akxyzi[2]=-65536+akxyz[2];
	    		}else{
	    			akxyzi[2]=akxyz[2];
	    		}


	    		akxyzf[0]=(float)(akxyzi[0]);
	    		akxyzf[0]=akxyzf[0]/8192;
	    		akxyzf[1]=(float)(akxyzi[1]);
	    		akxyzf[1]=akxyzf[1]/8192;
	    		akxyzf[2]=(float)(akxyzi[2]);
	    		akxyzf[2]=akxyzf[2]/8192;


	    				//printf("AX: %d           ", (int)(akxyzf[0]*100.0f));
	    				//printf("AY: %d           ", (int)(akxyzf[1]*100.0f));
	    				//printf("AZ: %d\r\n"       , (int)(akxyzf[2]*100.0f));


}

void m_read(void){
	 	 	 	 uint8_t mxyztmp=0;
	 	 	 	 uint8_t mxyz[3];
	 	 	 	 mxyz[0]=0; //os X
	 	 	 	 mxyz[1]=0; //ox Y
	 	 	 	 mxyz[2]=0; //os Z

	 	 	 	 //mxyz[3]=0; //os X
	 	 	 	 //mxyz[4]=0; //ox Y
	 	 	 	 //mxyz[5]=0; //os Z


				//Odczyt
	    		I2C_start(I2C1 ,AKM_ADDRESS_TR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	    		I2C_write(I2C1, 0x88); // wyslanie adresu rejestru z danymi z zyroskopu; adres 28, ale jezeli pierwszy bit 1 to wlaczona autoinkrementacja

	    		I2C_start(I2C1, AKM_ADDRESS_REC, I2C_Direction_Receiver); // start a transmission in Master receiver mode

	    		//ustawic obslug臋 zmiennoprzecinkowa i poprawnie odczytac format danych

	    			mxyztmp=I2C_read_ack(I2C1);
	    			mxyz[0]=mxyztmp <<8 | I2C_read_ack(I2C1);
	    			mxyztmp=I2C_read_ack(I2C1);
	    			mxyz[1]=mxyztmp <<8 | I2C_read_ack(I2C1);
	    			mxyztmp=I2C_read_ack(I2C1);
	    			mxyz[2]=mxyztmp <<8 | I2C_read_nack(I2C1);
	    			I2C_stop(I2C1); // stop the transmission




	    				//printf("MX: %d           ", mxyz[0]);
	    				//printf("MY: %d           ", mxyz[1]);
	    				//printf("MZ: %d\r\n", mxyz[2]);

	    		/*
	    		akxyz[0]=I2C_read_ack(I2C1);
	    		akxyz[1]=I2C_read_ack(I2C1);
	    		akxyz[2]=I2C_read_ack(I2C1);
	    		akxyz[3]=I2C_read_ack(I2C1);
	    		akxyz[4]=I2C_read_ack(I2C1);
	    		akxyz[5]=I2C_read_nack(I2C1);
	    		I2C_stop(I2C1); // stop the transmission

				printf("AX: %d ", akxyz[0]);
				printf("%d           ", akxyz[1]);
				printf("AY: %d ", akxyz[2]);
				printf("%d           ", akxyz[3]);
				printf("AZ: %d ", akxyz[4]);
				printf("%d\r\n ", akxyz[5]);

*/
}

void bar_read(void){
	 	 	 	 uint8_t bartmp=0;
	 	 	 	 uint16_t bartemp=0;
	 	 	 	 uint32_t barpress=0;



				//Odczyt
	    		I2C_start(I2C1 ,BAR_ADDRESS_TR, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	    		I2C_write(I2C1, 0xA8); // wyslanie adresu rejestru z danymi z zyroskopu; adres 28, ale jezeli pierwszy bit 1 to wlaczona autoinkrementacja

	    		I2C_start(I2C1, BAR_ADDRESS_REC, I2C_Direction_Receiver); // start a transmission in Master receiver mode

	    		//ustawic obslug臋 zmiennoprzecinkowa i poprawnie odczytac format danych

	    			bartmp=I2C_read_ack(I2C1);
	    			barpress=bartmp <<8 |  I2C_read_ack(I2C1);
	    			barpress=barpress <<8 |  I2C_read_ack(I2C1);

	    			bartmp=I2C_read_ack(I2C1);
	    			bartemp=bartmp <<8 |  I2C_read_nack(I2C1);
	    			I2C_stop(I2C1); // stop the transmission



	    				//printf("BP: %d           ", barpress);
	    				//printf("BT: %d\r\n", bartemp);

	    		/*
	    		akxyz[0]=I2C_read_ack(I2C1);
	    		akxyz[1]=I2C_read_ack(I2C1);
	    		akxyz[2]=I2C_read_ack(I2C1);
	    		akxyz[3]=I2C_read_ack(I2C1);
	    		akxyz[4]=I2C_read_ack(I2C1);
	    		akxyz[5]=I2C_read_nack(I2C1);
	    		I2C_stop(I2C1); // stop the transmission

				printf("AX: %d ", akxyz[0]);
				printf("%d           ", akxyz[1]);
				printf("AY: %d ", akxyz[2]);
				printf("%d           ", akxyz[3]);
				printf("AZ: %d ", akxyz[4]);
				printf("%d\r\n ", akxyz[5]);

*/
}


																//Ustawienia sygna艂u PWM podawanego na ESC
void TIM_Config_ESC(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;


  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOC and GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);

  /* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* GPIOB Configuration:  TIM3 CH3 (PB0) and TIM3 CH4 (PB1) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect TIM3 pins to AF2 */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
}

void PWM_Config_ESC(int period)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

  uint16_t PrescalerValue = 0;
  /* Compute the prescaler value */


  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 60000) - 1;

  //PrescalerValue=0;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}

void PWM_SetDC(uint16_t channel,uint16_t dutycycle)
{
  if (channel == 1)
  {
    TIM3->CCR1 = dutycycle;
  }
  else if (channel == 2)
  {
    TIM3->CCR2 = dutycycle;
  }
  else if (channel == 3)
  {
    TIM3->CCR3 = dutycycle;
  }
  else
  {
    TIM3->CCR4 = dutycycle;
  }
}



																//Ustawienia PWM dla sygna艂u pobieranego z aparatury

/* Configure pins to be interrupts */
void Configure_PD0(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Enable clock for GPIOD */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Tell system that you will use PD0 for EXTI_Line0 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);

    /* PD0 is connected to EXTI_Line0 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);

    /* Add IRQ vector to NVIC */
    /* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}

void Configure_PB12(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Enable clock for GPIOB */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Tell system that you will use PB12 for EXTI_Line12 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);

    /* PB12 is connected to EXTI_Line12 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line12;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);

    /* Add IRQ vector to NVIC */
    /* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}

/* Set interrupt handlers */
/* Handle PD0 interrupt */
void EXTI0_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        /* Do your stuff when PD0 is changed */
    	if(capture_ready == 0)
    	{
    	 if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0) && !capture2) {

    		 inpwm[0]=TIM_GetCounter(TIM2);
    		 capture1=1;
    	  }
    	 if(!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0) && capture1){

    		  inpwm[1]=TIM_GetCounter(TIM2);
    		  capture2=1;

    	  }
    	}
    	if(capture1 && capture2){
    		capture_ready = 1;
    		capture1=0;
    		capture2=0;
    		inpwm[2] = inpwm[1] - inpwm[0];
    		if(inpwm[1]<inpwm[0])
    			inpwm[2]=1001+inpwm[2];
    		pilot_w=inpwm[2];
    		//printf("CI_end: %d\r\n", inpwm[2]);
    		//printf("CI_1: %d\r\n", inpwm[0]);
    		//printf("CI_2: %d\r\n", inpwm[1]);
    		//GPIO_SetBits(GPIOD, GPIO_Pin_13);



    		capture_ready =0;

    	}

        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

/* Handle PB12 interrupt */
void EXTI15_10_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
        /* Do your stuff when PB12 is changed */
    	if(capture_ready2 == 0)
    	    	{
    	    	 if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) && !capture22) {

    	    		 inpwm2[0]=TIM_GetCounter(TIM2);
    	    		 capture12=1;
    	    	  }
    	    	 if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) && capture12){

    	    		  inpwm2[1]=TIM_GetCounter(TIM2);
    	    		  capture22=1;

    	    	  }
    	    	}
    	    	if(capture12 && capture22){
    	    		capture_ready2 = 1;
    	    		capture12=0;
    	    		capture22=0;
    	    		inpwm2[2] = inpwm2[1] - inpwm2[0];
    	    		if(inpwm2[1]<inpwm2[0])
    	    			inpwm2[2]=1001+inpwm2[2];
    	    		pilot_w2=inpwm2[2];
    	    		//printf("CI_end: %d\r\n", inpwm2[2]);
    	    		//printf("CI_1: %d\r\n", inpwm[0]);
    	    		//printf("CI_2: %d\r\n", inpwm[1]);
    	    		//GPIO_SetBits(GPIOD, GPIO_Pin_13);



    	    		capture_ready2 =0;

    	    	}


        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}

void initTimers()
{
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

   TIM_TimeBaseInitTypeDef base_timer;
   TIM_TimeBaseStructInit(&base_timer);
   base_timer.TIM_Prescaler = (uint16_t) ((SystemCoreClock /2) / 50000) - 1;
   base_timer.TIM_Period = 1000;
   base_timer.TIM_ClockDivision = 0;
   base_timer.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM2, &base_timer);
   TIM_Cmd(TIM2, ENABLE);


   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

   //TIM_TimeBaseInitTypeDef base_timer;
   //TIM_TimeBaseStructInit(&base_timer);
   base_timer.TIM_Prescaler = (uint16_t) ((SystemCoreClock /2) / 20210) - 1;
   base_timer.TIM_Period = 100;
   base_timer.TIM_ClockDivision = 0;
   base_timer.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM4, &base_timer);
   TIM_Cmd(TIM4, ENABLE);

   TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

   NVIC_InitTypeDef nvicStructure;
   nvicStructure.NVIC_IRQChannel = TIM4_IRQn;
   nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
   nvicStructure.NVIC_IRQChannelSubPriority = 1;
   nvicStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&nvicStructure);
}






void TIM4_IRQHandler()
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
    }
}
















int main(void) {




    SystemInit();
    init_LED(); // initialize 4 LED's
    init_Button(); //initialize button

	init_I2C1(); // initialize I2C peripheral
    init_gyro(); // initialize gyroskope
    init_akm(); // initialize akcelerometr i magnetometr
    //init_bar();


    TIM_Config_ESC();
    PWM_Config_ESC(100);
    PWM_SetDC(1,50);
    PWM_SetDC(2,50);
    PWM_SetDC(3,50);
    PWM_SetDC(4,50);


    /* Configure PD0 as interrupt */
    Configure_PD0();
    /* Configure PB12 as interrupt */
    Configure_PB12();

    initTimers();


    GPIO_SetBits(GPIOD, GPIO_Pin_15);
    //GPIO_SetBits(GPIOD, GPIO_Pin_15);




    //printf("Odczyty: \r\n");

    //gyro_read();
    //ak_read();
    //m_read();
    //bar_read();

    //Zmienne silniki na przycisk
    int temp=0;
    int i;
    int wyp;


    float fxg = 0;
    float fyg = 0;
    float fzg = 0;
    float fxa = 0;
    float fya = 0;
    float fza = 0;
    float pitch, roll;
	float pitchacc;
	float rollacc;

	//float roll_tab[50];
	//float pitch_tab[50];
	//float ust_pitch=0;
	//float ust_roll=0;

	float dt;

    float out_pitch, previous_error_pitch, integral_pitch, derivative_pitch, error_pitch, setpoint_pitch, Kp_pitch, Ki_pitch, Kd_pitch;
    uint16_t out_pitch_int;

    float out_roll, previous_error_roll, integral_roll, derivative_roll, error_roll, setpoint_roll, Kp_roll, Ki_roll, Kd_roll;
    uint16_t out_roll_int;

    uint16_t ch1, ch2, ch3, ch4;
    dt=0.0034;

    //Pitch
    //Kp= 0.05;
    Kp_pitch= 0.05;
    //Ki=0.04;
    Ki_pitch=0.04;
    //Kd=0.02;
    Kd_pitch=0.02;

    //Roll
    //Kp= 0.01;
    Kp_roll= 0.2;
    //Ki=0.2;
    Ki_roll=0.04;
    //Kd=0.02;
    Kd_roll=0.02;



    previous_error_pitch = 0;
    integral_pitch = 0;
    previous_error_roll = 0;
    integral_roll = 0;
    setpoint_pitch = 0;
    setpoint_roll = 0;


while(1)
{
	//gyro_read();





	// Program sterowania


	//triger bezpiecze駍twa z pilota
	if(pilot_w2>90){
				ster_pilot=1;
			GPIO_ResetBits(GPIOD, GPIO_Pin_15);
			GPIO_SetBits(GPIOD, GPIO_Pin_14);
	}
			if(pilot_w2<60){
					ster_pilot=0;
					GPIO_SetBits(GPIOD, GPIO_Pin_15);
					GPIO_ResetBits(GPIOD, GPIO_Pin_14);
					PWM_SetDC(1,50);
					PWM_SetDC(2,50);
					PWM_SetDC(3,50);
					PWM_SetDC(4,50);
					temp=0;
					integral_pitch=0;
					derivative_pitch=0;
					integral_roll=0;
					derivative_roll=0;
					}



	//Button reaction

	//Returs pin state (1 if HIGH, 0 if LOW)
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)) {
		if(temp==0){
			//ster_pilot=1;
		//GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		//GPIO_SetBits(GPIOD, GPIO_Pin_14);


		temp=1;
		}else{
			ster_pilot=0;
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		PWM_SetDC(1,50);
		PWM_SetDC(2,50);
		PWM_SetDC(3,50);
		PWM_SetDC(4,50);
		temp=0;
		integral_pitch=0;
		derivative_pitch=0;
		integral_roll=0;
		derivative_roll=0;
		}

					//for (i = 0; i < 5000000; i++);

	     }
	gyro_read();
			//GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
			//printf("GX: %d           ", (int)(gxyzf[0]));
			//printf("GY: %d           ", (int)(gxyzf[1]));
			//printf("GZ: %d\r\n", (int)(gxyzf[2]));

	ak_read();
			//printf("AX: %d           ", (int)(akxyzf[0]*100.0f));
			//printf("AY: %d           ", (int)(akxyzf[1]*100.0f));
			//printf("AZ: %d\r\n"       , (int)(akxyzf[2]*100.0f));



	    fxa = akxyzf[0];
	   	fya = akxyzf[1];
	   	fza = akxyzf[2];
	    fxg = gxyzf[0];
	   	fyg = gxyzf[1];
	   	fzg = gxyzf[2];

	   	pitchacc= atan2f(fxa,fza)*180/M_PI;
	   	rollacc= atan2f(fya, fza)*180/M_PI;


	   	pitch = pitch - (fyg)*0.0034;
	   	roll = roll + (fxg)*0.0034;

	   	pitch = pitch*0.98 +pitchacc*0.02;
	   	roll = roll*0.98 + rollacc*0.02;


	   	//if((int)(pitch)>90){
	   		//   	GPIO_ToggleBits(GPIOD, GPIO_Pin_13);
	   		  // 	}


	   	if(pitch>70 || pitch<-70 || roll>70 || roll<-70){
	   		   						ster_pilot=0;
	   		   						GPIO_SetBits(GPIOD, GPIO_Pin_15);
	   		   						GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	   		   						PWM_SetDC(1,50);
	   		   						PWM_SetDC(2,50);
	   		   						PWM_SetDC(3,50);
	   		   						PWM_SetDC(4,50);
	   		   						temp=0;
	   		   						integral_pitch=0;
	   		   						derivative_pitch=0;
	   		   						integral_roll=0;
	   		   						derivative_roll=0;
	   		   						}




	   	if(integral_pitch>5)integral_pitch=5;
	   	if(integral_pitch<-5)integral_pitch=-5;

	   	if(integral_roll>5)integral_roll=5;
	   	if(integral_roll<-5)integral_roll=-5;




	   	if(ster_pilot){


	   	error_pitch = setpoint_pitch - pitch;
	   	integral_pitch = integral_pitch + error_pitch*dt;
	   	derivative_pitch = (error_pitch - previous_error_pitch)/dt;
	   	out_pitch = Kp_pitch*error_pitch + Ki_pitch*integral_pitch + Kd_pitch*derivative_pitch;
	   	previous_error_pitch = error_pitch;

	   	out_pitch_int=(int)(out_pitch);

	   	//out_pitch_int=0;




	   	error_roll = setpoint_roll - roll;
	   	integral_roll = integral_roll + error_roll*dt;
	   	derivative_roll = (error_roll - previous_error_roll)/dt;
	   	out_roll = Kp_roll*error_roll + Ki_roll*integral_roll + Kd_roll*derivative_roll;
	   	out_roll = Kp_roll*error_roll;
	   	previous_error_roll = error_roll;

	   	out_roll_int=(int)(out_roll);

	   	//out_roll_int=0;
	   	//GPIO_ToggleBits(GPIOD, GPIO_Pin_12);




	   	ch1=pilot_w-out_pitch_int-out_roll_int;
	   	ch2=pilot_w-out_pitch_int+out_roll_int;
	   	ch3=pilot_w+out_pitch_int-out_roll_int;
	   	ch4=pilot_w+out_pitch_int+out_roll_int;
	   	if(ch1>100) ch1=100;
	   	if(ch1<50) ch1=50;
	   	if(ch2>100) ch2=100;
	   	if(ch2<50) ch2=50;
	   	if(ch3>100) ch3=100;
	   	if(ch3<50) ch3=50;
	   	if(ch4>100) ch4=100;
	   	if(ch4<50) ch4=50;
	   	PWM_SetDC(1,ch1);
	   	PWM_SetDC(2,ch2);
	   	PWM_SetDC(3,ch3);
	   	PWM_SetDC(4,ch4);
	   	}

}



}







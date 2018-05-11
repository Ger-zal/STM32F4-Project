#include "stm32f4xx.h"

#define RTC_ADDRESS			0x68
#define RTC_REGISTER		0X00

#define DS1621_Address		0x4C
#define START_CONVERTION   	0xEE
#define READ_TEMPERATURE	0xAA
#define ACCESS_CONFIG 		0xAC
uint8_t TEMPS[7];
char Time[100]={0};
/***********************CONFIG_CLOCK******************************///
void Clock_Config(){						//HSI CLOCK 16MHz
	RCC->CR		|=	RCC_CR_HSION;			// Enable HSI
	while(!(RCC->CR & RCC_CR_HSIRDY));		// Wait till HSI READY
}
/***********************DELAY*************************************/
void delay(__IO uint32_t nCount){			// Delay Function
	while(nCount--);
}


//***************************************ESP SEND CHAR**********************************
void ESP_SendChar(char Tx)
{
   while((USART1->SR & USART_SR_TXE)==0);  // On attend à ce que TXBUFF soit vide (libere) ou (FLAG TXNE=1) ou Le caractere est envoyé
   USART1->DR=Tx;
}
//***************************************ESP SEND STRING**********************************
void ESP_SendTxt(char *Adr){
  while(*Adr)
  {
    ESP_SendChar(*Adr);
    Adr++;
  }
}
//***************************************ESP RECEIVE**********************************
void ESP_Receive(char * data){
	int i=0;
	for(i=0;i<3;i++){
		while (!(USART1->SR & USART_SR_RXNE ));
		data[i]= (USART1->DR & 0xFF);
		USART1->SR &= ~(USART_SR_RXNE);
	}
}
//***************************************FTDI SEND CHAR**********************************
void FTDI_SendChar(char Tx)
{
   while((USART6->SR & USART_SR_TXE)==0);  // On attend à ce que TXBUFF soit vide (libere) ou (FLAG TXNE=1) ou Le caractere est envoyé
   USART6->DR=Tx;
}
//***************************************ESP SEND STRING**********************************
void FTDI_SendTxt(char *Adr){
  while(*Adr)
  {
    FTDI_SendChar(*Adr);
    Adr++;
  }
}
//***************************************FTDI RECEIVE**********************************
void FTDI_Receive(char * data){
	int i=0;
	for(i=0;i<3;i++){
		while (!(USART6->SR & USART_SR_RXNE ));
		data[i]= (USART6->DR & 0xFF);
		USART6->SR &= ~(USART_SR_RXNE);
	}
}
char Receive(){
	/*char data[20]={0};
	int i=0;
	while(!(USART6->SR & USART_SR_RXNE));
	for(i=0;i<20;i++){
		data[i] = (char)(USART6->DR & 0xFF);
	}
	return (char) (data);*/
	while (!(USART6->SR &USART_SR_RXNE));
	char data=(USART6->DR&0xFF);
	USART6->SR &= ~(USART_SR_RXNE);
	return (char)data;
}
void SendChar_WIFI(char Tx){
   while(!(USART1->SR & USART_SR_TXE));  			// wait TXBUFF=1
   USART1->DR=Tx;
}

//***************************************USART SEND CHAR BY CHAR**********************************
void SendTxt_WIFI(char *Adr)
{
  while(*Adr){
    SendChar_WIFI(*Adr);
    Adr++;
  }
}
char reveive_WIFI(){
	if((USART1->SR & USART_SR_RXNE)!=0){
		char data = USART1->DR;
		return data;
	}
}
/***********************CONFIG_USART******************************/
void USART_Config_FTDI(){
	RCC->AHB1ENR 	|= 	RCC_AHB1ENR_GPIOCEN; 			// Enable clock for GPIOC
	RCC->APB2ENR	|= 	RCC_APB2ENR_USART6EN;   		// Enable clock for USART6
	GPIOC->AFR[0]	 =	0x88000000;  					// enable USART6_TX to PC6 and USART6_RX to PC7
	GPIOC->MODER	|=	GPIO_MODER_MODER6_1;			// configuring the USART6 ALTERNATE function PC6
	GPIOC->MODER	|=	GPIO_MODER_MODER7_1;				// configuring the USART6 ALTERNATE function PC7
	USART6->BRR		 =	0x682;    						// 9600 Baud
	USART6->CR1		|=	USART_CR1_UE |USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE; 	// USART6 enable(c=[TE: Transmitter enable %RE:Receiver enable]2=[RXNEIE:RXNE interrupt enable]2=[UE: USART enable] )
}
void USART_Config_WIFI(){
	RCC->AHB1ENR 	|= 	RCC_AHB1ENR_GPIOAEN; 			// Enable clock for GPIOA
	RCC->APB2ENR	|= 	RCC_APB2ENR_USART1EN;   		// Enable clock for USART1
	GPIOA->AFR[1]	 =	0x00000770;  					// enable USART1_TX to PA9 and USART1_RX to PA10
	GPIOA->MODER	|=	GPIO_MODER_MODER9_1;			// configuring the USART1 ALTERNATE function PA9
	GPIOA->MODER	|=	GPIO_MODER_MODER10_1;				// configuring the USART1 ALTERNATE function PA10
	USART1->BRR		 =	0x682;    						// 9600 Baud
	USART1->CR1		|=	USART_CR1_UE |USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE; 	// USART6 enable(c=[TE: Transmitter enable %RE:Receiver enable]2=[RXNEIE:RXNE interrupt enable]2=[UE: USART enable] )
}
void ESP_USART_Config(){
	RCC->AHB1ENR 	|=	RCC_AHB1ENR_GPIOAEN; 						// Enable clock for GPIOA
	RCC->APB2ENR	|= 	RCC_APB2ENR_USART1EN;   					// Enable clock for USART1
	GPIOA->AFR[1]	|= 	0x770;  									// Enable USART1_TX to PA9 and USART1_RX to PA10
	GPIOA->MODER	|=	GPIO_MODER_MODER9_1 |GPIO_MODER_MODER10_1;  // configuring the USART1 ALTERNATE function  to PA9 and PA10
	USART1->BRR		 =	System_Clock/ESP_Baud;    						// 1152000 Baud   ;
	USART1->CR1		|=	USART_CR1_UE |USART_CR1_TE |USART_CR1_RE;	// USART6 enable(c=[TE: Transmitter enable %RE:Receiver enable]2=[RXNEIE:RXNE interrupt enable]2=[UE: USART enable] )
}
void Config_ADC()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN; // Enable ADC1 ,ADC2, ADC3
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //clock enable GPIOA
	ADC->CCR = ADC_CCR_MULTI_1 | ADC_CCR_MULTI_2 ; // No DMA, Regular simultaneous mode only

	ADC1->CR2 = ADC_CR2_ADON; // Control Register 2: ADC1 ON
	ADC1->SQR3 = 0; // regular SeQuence Register 3

	ADC2->CR2 = ADC_CR2_ADON; // Control Register 2: ADC2 ON
	ADC2->SQR3 = ADC_SQR3_SQ1_1; // regular SeQuence Register 3

	ADC3->CR2 = ADC_CR2_ADON; // Control Register 2: ADC3 ON
	ADC3->SQR3 = ADC_SQR3_SQ1_2 ; // regular SeQuence Register 3

	GPIOA->MODER |= GPIO_MODER_MODER0_0 |GPIO_MODER_MODER0_1 |GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_0 |GPIO_MODER_MODER2_1 ;//Analog mode PA0 and PA1 & PA2
}
/***********************CONFIG_I2C********************************/
void I2C_Config(void){ //I2C on pins PB10<SCL-PB11<SDA
	RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOBEN ;// Enable Clock GPIOB
	RCC->APB1ENR 	|= RCC_APB1ENR_I2C2EN;// Enable Clock I2C
	GPIOB->AFR[1]  	|= 0x4400;
	GPIOB->MODER	|= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1 ;
	GPIOB->MODER	&= ~(GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0) ;
	GPIOB->OTYPER	|= GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11 ;
	GPIOB->PUPDR	&= ~(GPIO_PUPDR_PUPDR10_0 |GPIO_PUPDR_PUPDR10_1|GPIO_PUPDR_PUPDR11_0|GPIO_PUPDR_PUPDR11_0);
	I2C2->CR2		|= I2C_CR2_FREQ_4;
	I2C2->CCR		|= 0x50;
	I2C2->TRISE		|= 0x11;
	I2C2->CR1		|= I2C_CR1_PE; //Enable Peripheral
}
/***********************I2C_START*********************************/
void I2C_Start(){
	I2C2->CR1		|=I2C_CR1_START; //Enable Start Condition
	while(!(I2C2->SR1 & I2C_SR1_SB));				// wait until Flag UP (Start I2C)
	//SendTxt("...............START I2C\n");
}
/***********************I2C_SEND_ADDRESS*******************************/
void I2C_Send_Address(uint8_t Address, char direction){
	if (direction == 'w') {
			I2C2->DR		=(Address<<1)&0xFE;
			while( !(I2C2->SR1 &  I2C_SR1_ADDR));
			int status=I2C2->SR2;
			//SendTxt("...............I2C SEND ADDRESS WRITE\n");
		}
		else if (direction == 'r') {
			I2C2->DR = ((Address<<1)|0x1);
			while(!(I2C2->SR1 & I2C_SR1_ADDR));				//wait until ADDR is received && read selected
			//SendTxt("...............I2C SEND ADDRESS READ\n");
		}

}
/**********************I2C_SEND_DATA***********************************/
void I2C_Write (uint8_t data){
	I2C2->DR = data;
	while(!( I2C2->SR1 &I2C_SR1_TXE)); //wait until DR empty
	while(!(I2C2->SR1 & I2C_SR1_BTF));//wait until byteTrasferred
	//SendTxt("...............I2C WRITE\n");
}
/**************************I2C_STOP************************************/
void I2C_Stop(){
	I2C2->CR1 |= I2C_CR1_STOP;	// I2C STOP
	//SendTxt("...............I2C STOP\n\n");
}

void  DS1621_Config(){
	SendTxt("CONFIG I2C.......\n\n");
	I2C_Start();
	I2C_Send_Address(DS1621_Address,'w');
	I2C_Write(START_CONVERTION);
	I2C_Stop();

	I2C_Start();
	I2C_Send_Address(DS1621_Address,'w');
	I2C_Write(ACCESS_CONFIG);
	I2C_Write(0x08);
	I2C_Stop();

}
unsigned char DS1621_Read(){
	I2C_Start();
	I2C_Send_Address(DS1621_Address,'w');
	I2C_Write(READ_TEMPERATURE);
	I2C_Start();
	I2C_Send_Address(DS1621_Address,'r');
	I2C2->CR1	|=I2C_CR1_POS;	//read 2 byte
	int status2=I2C2->SR2;
	while(!(I2C2->SR1 & I2C_SR1_BTF));//wait until byteTrasferred
	I2C_Stop();
	unsigned char temp=I2C2->DR;
	int x=I2C2->DR;
	return temp;
}
void RTC_Read(uint8_t temps[]){
	I2C_Start();
	//SendTxt("trc start \n");
	I2C_Send_Address(RTC_ADDRESS,'w');
	//SendTxt("addr sent \n");
	I2C_Write(RTC_REGISTER);
	//SendTxt("reg sent \n");
	I2C_Start();
	I2C_Send_Address(RTC_ADDRESS,'r');
	//SendTxt("sent read \n");
	int i;
		for( i=0;i<=6;i++){
			if (i==6) {
				I2C2->CR1 &= ~ (I2C_CR1_ACK);	//not ackint status2=I2C2->SR2;
				int status2=I2C2->SR2;
				while(!(I2C2->SR1 & I2C_SR1_BTF));//wait until byteTrasferred

			}
			else {
				I2C2->CR1 |= I2C_CR1_ACK;	//ack
				int status2=I2C2->SR2;
				while(!(I2C2->SR1 & I2C_SR1_BTF));//wait until byteTrasferred
			}
			temps[i]=I2C2->DR;
		}
	I2C_Stop();
}
void RTC_Set(uint8_t hour,uint8_t min,uint8_t day,uint8_t date,uint8_t month,uint8_t year){
	I2C_Start();
	I2C_Send_Address(RTC_ADDRESS,'w');
	I2C_Write(RTC_REGISTER);
	I2C_Write(0x00);
	I2C_Write(min);
	I2C_Write(hour & 0x3F);
	I2C_Write(day);
	I2C_Write(date);
	I2C_Write(month);
	I2C_Write(year);
	I2C_Stop();
}
unsigned char LM35_Read(){
	ADC1->CR2 |= ADC_CR2_SWSTART; // simultaneous Start Conversion
	while(!(ADC1->SR & 0x2)); // wait for ADC1 conversion to complete
	int temp =	ADC1->DR;
	unsigned char temperature =(temp*100)*(3.3/4095);//4095 ADC 12 bits(2^12-1)
	return temperature;
}
void Config_PWM(void){

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //clock enable for TIM3
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //clock enable GPIOA
	GPIOA->MODER |= GPIO_MODER_MODER6_1; //Alternative function mode PA6
	TIM3->CCMR1 = TIM_CCMR1_OC1M_1 |TIM_CCMR1_OC1M_2; //PWM mode 1 on TIM3 Channel 1
	TIM3->PSC = 2;   // Fc=f/PSC+1    avec  f=16Mhz;; ARR=256;; =Fc/255
	TIM3->ARR = 255;
	TIM3->CCR1 = 0;
	GPIOA->AFR[0] = 0x2000000; //set GPIOA to AF2
	TIM3->CCER	|=TIM_CCER_CC1E;
	TIM3->EGR	|=TIM_EGR_CC1G;
	TIM3->CR1 	 = TIM_CR1_CEN; //enable counter of tim3

}
void TIM1_config() {//TIM 1 channel1 PE9
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;  // Enable GPIOE clock
   GPIOE->MODER |= GPIO_MODER_MODER9_1;  // Enable AF mode for PE9
   GPIOE->AFR[1]|= 0x0010;               // Select AF1 for E9 (TIM1)
   //GPIOE->PUPDR |=GPIO_PUPDR_PUPDR9_1;
   RCC->APB1ENR |= RCC_APB2ENR_TIM1EN;  // Enable TIM1 clock
   TIM1->CR1	|= ~(TIM_CR1_CEN);
   TIM1->CCMR1 	|= TIM_CCMR1_CC1S_0;                    // CC1 channel is configured as input, IC1 is mapped on TI1
   TIM1->CCMR1 	|= TIM_CCMR1_IC1F_0 |TIM_CCMR1_IC1F_1;// fSAMPLING=fCK_INT, N=8
   TIM1->CCER 	|= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);    // CC1P and CC1NP = 0 for rising edge
   TIM1->CCMR1 	|= ~(TIM_CCMR1_IC1PSC);	//PSC disabled
   TIM1->CCER 	|= TIM_CCER_CC1E; // Capture enabled.
   TIM1->DIER 	|=TIM_DIER_CC1IE;  //CC1 interrupt enabled
   //NVIC_EnableIRQ(TIM1_CC_IRQn); // Enable TIM1 IRQ
   TIM1->CR1 	|= TIM_CR1_CEN;  // Enable TIM1 counter
  // la valeur de la frequencee se trouve dans le registre CCR2 :: frequence=tim_counter_clock/tim_ccr2
}
int main(void){
	Clock_Config();
	USART_Config_FTDI();
	FTDI_USART_Config();
	ESP_USART_Config();
	/*
	__IO uint16_t ADCBuffer[2];
	char Current[100];
	char Voltage[100];
	uint16_t voltage;
	char Temmp[100];
	uint16_t  temperature;
	*/
	int LM35=11,DS1621=22,Voltage=33,Current=44,Actual_Speed=55;
	char Speed[3]={0},Sensors[]={0};
	FTDI_SendTxt("hello\n");
	char Temmp[100];
	SendTxt("hello\n");
	
while(1){
		if ((USART6->SR &USART_SR_RXNE))SendTxt("available\n");
		else SendTxt("not available\n");
		SendChar(Receive());
		SendTxt("\nfinish\n");
		
	//**********PWM ***************************:
		TIM3->CCR1++;
	if(TIM3->CCR1==TIM3->ARR)
	{
		TIM3->CCR1=0;
	}
	//***************FTDI********************
	FTDI_SendTxt("1,22,33,44,55\n");
		ESP_SendTxt("11,22,33,44,55\n");
		delay(16000000);
		
	///*****************ADC**************************
	ADC1->CR2 |= ADC_CR2_SWSTART; // simultaneous Start Conversion
	while((ADC1->SR & 0x2)==0); // wait for ADC1 conversion to complete
	int temp=ADC1->DR;
	temperature=(temp*100)*(3.3/4095);//4095 ADC 12 bits(2^12-1)
	sprintf(Temmp,"temperature = %d\n",temperature) ;
	SendTxt(Temmp);
	}
	delay(100000);
}

void TIM1_CC_IRQHandler()

{
	char Temmp[100];

	if(TIM1->SR &0x2){
		int periode= TIM1->CCR1;
		int freq = TIM1->CNT / TIM1->CCR1;
		sprintf(Temmp,"frequence =  %d\n",TIM1->CCR1) ;
		SendTxt(Temmp);
	}

TIM1->SR &= 0x0000; // Interrupt has been handled
}

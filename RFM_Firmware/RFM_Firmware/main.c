/*
 * RFM_Firmware.c
 *
 * Created: 20.09.2019 9:29:54
 * Author : Игорь
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

//команды контроллеру
enum contrCommand{
	COMMAND_NO_COMMAND=0,
	COMMAND_GET_REGISTER,
	COMMAND_SET_GERISTER
};





void initGPIO();
void initSPI();
void initUART();
void initTimers();
void decodePacket();
unsigned char CRC16(unsigned char *pcBlock, unsigned short len);
void getRegister();
void setRegister(unsigned char regAddress, unsigned char data);
void transmitUART();
void preparePacket();

unsigned char USARTInputArray[3]={0,0,0};
unsigned char transmitArray[3]={0,0,0};
unsigned char USARTCounter=0;
unsigned char SPICount=0;
unsigned char dataReady=0;
unsigned int bytesCounter=0;
int currentCommand=COMMAND_NO_COMMAND;
unsigned char SPIIncomingBuffer=0;


int main(void)
{
	initGPIO();
    initSPI();
	initUART();
	initTimers();
	sei();
    while(1){
		if(dataReady){
			transmitUART();
		}
	}
}
//////////////////////////////////////////////////////////////////////
void initGPIO(){
	DDRC|=(1<<PORTC0);//светодиод
	PORTC&=(~(1<<PORTC0));
	PORTB|=(1<<PORTB2);//подтягивающий резистор для SS ISP
}
//////////////////////////////////////////////////////////////////////
void initSPI(){
	SPCR|=(1<<SPE);//включаем SPI
	SPCR|=(1<<SPIE);//разрешаем прерывания от SPI
	SPCR|=(1<<MSTR);//режим МАСТЕР
}
//////////////////////////////////////////////////////////////////////
void initUART(){
	UBRR0L = 103;//Младшие 8 бит UBRRL_value
	UBRR0H = 103>>8;//Старшие 8 бит UBRRL_value
	UCSR0B|=(1<<RXEN0);//разрешение приема
	UCSR0B|=(1<<RXCIE0);//разрешения прерывания по завершению приемa
	UCSR0B|=(1<<TXCIE0);//разрешение прерывания по завершении передачи
	UCSR0C|=(3<<UCSZ00);//формат 8 бит данных
}
////////////////////////////////////////////////////////////////////////////////
void initTimers(){
	TIMSK1|=(1<<TOIE1);//Т1 - для обслуживания UART
	TCNT1=0;
	TCCR1B|=(1<<CS10)|(1<<CS12);//делитель на 1024
}
/////////////////////////////////////////////////////////////////////////////////
ISR(USART_RX_vect){	
	if(TCNT1>30){//если с момента последнего байта прошло больше 2мС, то это начало посылки
		USARTCounter=0;
	}
	TCNT1=0;
	USARTInputArray[USARTCounter]=UDR0;
	USARTCounter++;
	if(USARTInputArray[0]==USARTCounter){
		USARTCounter=0;
		decodePacket();
	}
	else if(USARTCounter>9){//размер пакета не может быть больше 10 байт
		USARTCounter=0;
	}
}
////////////////////////////////////////////////////////////
ISR(USART_TX_vect){
	if(bytesCounter!=transmitArray[0]){//если переданный байт не был последним
		dataReady=1;
		return;
	}
	else{//иначе переключаемся на прием
		UCSR0B&=(~(1<<TXEN0));//запрет передачи
		UCSR0B|=(1<<RXEN0);//разрешение приема
	}
}
///////////////////////////////////////////////////////////////
void decodePacket(){
	unsigned char size=USARTInputArray[0];
	unsigned char crc=CRC16(USARTInputArray,size-1);
	if(crc==USARTInputArray[size-1]){
		switch(USARTInputArray[1]){
			case(COMMAND_GET_REGISTER):{
				currentCommand=USARTInputArray[1];
				getRegister(USARTInputArray[2]);
				break;
			}
			case(COMMAND_SET_GERISTER):{
				
				break;
			}
			default:{
				return;	
			}
		}
	}
	else{
		return;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER1_OVF_vect){
	USARTCounter=0;
}
/////////////////////////////////////////////////////////////////////////////////
ISR(SPI_STC_vect){
	PORTC|=(1<<PORTC0);
	switch(currentCommand){
		case(COMMAND_GET_REGISTER):{
			currentCommand=COMMAND_NO_COMMAND;
			SPDR=0;
			break;
		}	
		case(COMMAND_SET_GERISTER):{
			
			break;
		}
		case(COMMAND_NO_COMMAND):{
			dataReady=1;
			SPIIncomingBuffer=SPDR;
			preparePacket();
			break;
		}
	}
}
///////////////////////////////////////////////////////////////////////////
unsigned char CRC16(unsigned char *pcBlock, unsigned short len){
	unsigned short crc=0xFFFF;
	unsigned char i;
	while(len--){
		crc^= *pcBlock++ << 8;
		for(i=0;i<8;i++){
			crc=crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
		}
	}
	return crc;
}
////////////////////////////////////////////////////////////////////////
void getRegister(unsigned int regAddress){
	regAddress&=(~(1<<7));
	SPDR=regAddress;
}
//////////////////////////////////////////////////////////////////////////
void setRegister(unsigned char regAddress, unsigned char data){
	
}
/////////////////////////////////////////////////////////////////////////
void transmitUART(){
	while ( !( UCSR0A & (1<<UDRE0)) );
	UCSR0B&=(~(1<<RXEN0));//запрет на прием
	UCSR0B|=(1<<TXEN0);//разрешение передачи
	UDR0=transmitArray[bytesCounter];
	bytesCounter++;
	dataReady=0;
}
/////////////////////////////////////////////////////////////////////////////
void preparePacket(){
	transmitArray[0]=3;
	transmitArray[1]=SPIIncomingBuffer;
	unsigned char crc=CRC16(transmitArray,2);
	transmitArray[2]=crc;
	bytesCounter=0;
}
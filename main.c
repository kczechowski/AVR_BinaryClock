/*
 * main.c
 *
 *  Created on: 16 sie 2017
 *      Author: Karol
 */
#include <avr/io.h>
#include <util/delay.h>

//hardware SPI

#define F_CPU 8000000

#define MOSI (1<<PB3) //serial
#define SCK (1<<PB5) //shift clock
#define LT (1<<PB2) //storage clock
#define SCL (1<<PC5) //TWI SCL
#define SDA (1<<PC4) //TWI SDA

#define LT_ON PORTB |= LT //storage clock on
#define LT_OFF PORTB &= ~LT //storage clock off

void SPI_init(void);
void SPI_send(uint8_t byte);
void TWI_init(void);
void TWI_start(void);
void TWI_stop(void);
void TWI_send(uint8_t data);
uint8_t TWI_getStatus(void);
uint8_t TWI_readACK();
uint8_t TWI_readNACK();

uint8_t getSeconds(void);
void enable1HzTicker(void);
void RTC_init(void);

uint8_t decToBcd(uint8_t dec);
uint8_t bcdToDec(uint8_t bcd);


uint8_t RTC_ADDRESS_READ = 0xA3;
uint8_t RTC_ADDRESS_WRITE = 0xA2;

uint8_t RTC_SECONDS_REGISTER = 0x02;
uint8_t RTC_SECONDS_MASK = 0x7F;

uint8_t seconds = 0;

uint8_t BCD2BINARY(uint8_t x)
{
	uint8_t binary;
	binary = ((x & 0xF0 )>>4 ) * 10 + (x & 0x0F);
	return (binary);
}


int main(void){

	_delay_ms(500);

	TWI_init();
	SPI_init();

	_delay_ms(50);

	SPI_send(0x00);

	RTC_init();
	enable1HzTicker();

	while(1){
		getSeconds();
		SPI_send(seconds);
		_delay_ms(100);
/*		_delay_ms(500);
		DDRB |= (1<<PB1);
		_delay_ms(500);
		DDRB &= ~(1<<PB1);*/
	}



}

uint8_t getSeconds(void){

	TWI_start();
	TWI_send(RTC_ADDRESS_WRITE);
	TWI_send(RTC_SECONDS_REGISTER);
	TWI_start(); //repeated start
	TWI_send(RTC_ADDRESS_READ);

	seconds = bcdToDec(TWI_readNACK());
	TWI_stop();

}

void RTC_init(void){

	TWI_start();
	TWI_send(RTC_ADDRESS_WRITE);
	TWI_send(RTC_SECONDS_REGISTER);
	TWI_send(0x00);
	TWI_stop();

}

void enable1HzTicker(void){

	TWI_start();
	TWI_send(RTC_ADDRESS_WRITE);
   	TWI_send(0x0D); //CLKOUT_control register
    TWI_send(0x83);
	TWI_stop();

}

void SPI_init(void){

	DDRB |= MOSI | SCK | LT; //set ports as output

	SPCR |= (1<<SPE) | (1<<MSTR); //turn on SPI and set device as master

	SPSR |= (1<<SPI2X); //double speed

}

void SPI_send(uint8_t byte){

	SPDR = byte;

	while (!(SPSR & (1<<SPIF)));

	LT_ON;
	LT_OFF;

}

void TWI_init(void){

	TWSR = 0x00; //prescaler value
	TWBR = 0x20; //set bitrate to 100kHz
	PORTC |= SCL | SDA; //pull-up resistors
	TWCR = (1<<TWEN); //enable TWI

}

void TWI_start(void){

	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

}

void TWI_stop(void){

	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);

}

void TWI_send(uint8_t data){

	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));

}

uint8_t TWI_readACK(){

	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));
	return TWDR;

}

uint8_t TWI_readNACK(){

	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	return TWDR;

}

uint8_t TWI_getStatus(void){
    uint8_t status;
    status = TWSR & 0xF8; //mask status
    return status;
}

uint8_t decToBcd(uint8_t dec){
  return ( (dec/10*16) + (dec%10) );
}

uint8_t bcdToDec(uint8_t bcd){
  return ( (bcd/16*10) + (bcd%16) );
}


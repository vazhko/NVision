

#include "fx2.h"
#include "fx2regs.h"



#define SDAT	2//PA2
#define SCLK	3//PA3
#define LOAD	1//PA1

#define SPI_PORT	IOA

#define SPI_SOFT_DELAY 1



#define  TESTBIT(var, b)   	((var) & (1 <<(b)))
#define  SETBIT(var, b)    	((var) |= (1 << (b)))
#define  CLRBIT(var, b)    	((var) &= ~(1 << (b)))



 #define  SPI_CS_1()	SETBIT(IOE, 0)
 #define  SPI_CS_0()	CLRBIT(IOE, 0)



/******************************************************************************/
void spi_soft_delay(void);
void spi_soft_init(void); 
void AD5624_write(WORD );
void spi_init(void);
void SPIByteWriteU (BYTE d);
void SPIWriteDAC (BYTE h, BYTE l);

void uart_init(void);
void uart_putc(BYTE b);

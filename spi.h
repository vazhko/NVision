

#include "fx2.h"
#include "fx2regs.h"


#define SCLK	2 //PA2
#define SDAT_O	3 //PA3
#define SDAT_I	4 //PA4 
#define LOAD	5 //PA6

#define SPI_PORT	IOA	
#define SPI_SOFT_DELAY 1



#define  TESTBIT(var, b)   	((var) & (1 <<(b)))
#define  SETBIT(var, b)    	((var) |= (1 << (b)))
#define  CLRBIT(var, b)    	((var) &= ~(1 << (b))) 




 #define  SPI1_CS_1()	SETBIT(IOE, 0)
 #define  SPI1_CS_0()	CLRBIT(IOE, 0)



/******************************************************************************/
void spi_soft_delay(void);
void spi_soft_init(void); 
void shape_reg_write(BYTE adr_h, BYTE adr_l, BYTE value);
BYTE shape_reg_read(BYTE adr_h, BYTE adr_l);
BYTE spi_soft_read_write(BYTE data_out);

void SPI1_init(void); 
void SPI1_WriteDAC (BYTE h, BYTE l);

void UART0_init(void);
void UART0_putc(BYTE b);

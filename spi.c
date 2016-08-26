#include "spi.h"
#include "fx2sdly.h" 
#include "fx2regs.h"

/******************************************************************************/
/*
static void spi_soft_delay(void){
	char delay;
	for(delay=0; delay < SPI_SOFT_DELAY; delay++) ;;
}
*/

/******************************************************************************/
BYTE byte_mirror(BYTE c) {
    c = ((c >> 1) & 0x55) | ((c << 1) & 0xAA);
    c = ((c >> 2) & 0x33) | ((c << 2) & 0xCC);
    c = ((c >> 4) & 0x0F) | ((c << 4) & 0xF0);
    return c;
}

/******************************************************************************/
void spi_soft_init(void) {

    SETBIT(OEA, SCLK);
    SETBIT(OEA, SDAT_O);
    CLRBIT(OEA, SDAT_I);
    SETBIT(OEA, LOAD);


    SETBIT(IOA, SCLK);
    CLRBIT(IOA, SDAT_O);
    SETBIT(IOA, LOAD);
}


/******************************************************************************/  
BYTE spi_soft_read_write(BYTE data_out) {  
    BYTE i, j, data_in;

    i = 0x80;
    j = 0;

    data_in = 0;

    do {
        CLRBIT(SPI_PORT, SCLK);

        if((data_out & i) > 0) SETBIT(SPI_PORT, SDAT_O);  else CLRBIT(SPI_PORT, SDAT_O);
		//SETBIT(SPI_PORT, SDAT_O);
		SYNCDELAY;

		if(TESTBIT(SPI_PORT, SDAT_I)) {	 			
			data_in |= i; 
		}

        SETBIT(SPI_PORT, SCLK);
        i >>= 1;

    } while(j++ < 7);

	return data_in;
}

/******************************************************************************/
void shape_reg_write(BYTE adr_h, BYTE adr_l, BYTE value) {     
    
    CLRBIT(SPI_PORT, LOAD);

	spi_soft_read_write(adr_h);
	spi_soft_read_write(adr_l);
	spi_soft_read_write(value);

    SETBIT(SPI_PORT, LOAD);
	SYNCDELAY;
	CLRBIT(SPI_PORT, LOAD);
	//EP0BUF[3] = spi_soft_read_write(0xaa);
	//EP0BUF[4] = spi_soft_read_write(0);
}

/******************************************************************************/
BYTE shape_reg_read(BYTE adr_h, BYTE adr_l) {  
	BYTE i;
    
    CLRBIT(SPI_PORT, LOAD);

   	spi_soft_read_write(adr_h | 0x80);
	spi_soft_read_write(adr_l);	
	spi_soft_read_write(0);


	SETBIT(SPI_PORT, LOAD);
	SYNCDELAY;
	CLRBIT(SPI_PORT, LOAD);

	return spi_soft_read_write(0);	    

}


/******************************************************************************/
/******************************************************************************/
void SPI1_init(void) {
    SPI1_CS_1();
    SCON1 = 0x03;
    OEE |= 0x01;

}

/******************************************************************************/
void SPI1_ByteWriteU(BYTE d)	 {

    TI1 = FALSE; //Clear flag
    SBUF1 = d; //Write byte
    while(!TI1);  //Wait until transmit done
}

/******************************************************************************/
void SPI1_WriteDAC(BYTE h, BYTE l) {
    SPI1_CS_0();
    SPI1_ByteWriteU(byte_mirror(h));
    SPI1_ByteWriteU(byte_mirror(l));
    SPI1_CS_1();
}


/******************************************************************************/
void UART0_init(void) {

    T2CON = 0x34 ;
    RCAP2H = 0xFF ;
    RCAP2L = 0x64;
    SCON0 = 0x5A ;
    TI = 1;


}
/******************************************************************************/
void UART0_putc(BYTE b)	{
    while(!TI);
    TI = FALSE;
    SBUF0 = b;
}


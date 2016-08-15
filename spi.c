#include "spi.h"



/******************************************************************************/
/*
static void spi_soft_delay(void){
	char delay;
	for(delay=0; delay < SPI_SOFT_DELAY; delay++) ;;
}
 */


/******************************************************************************/
BYTE invert(BYTE c) {
    c = ((c >> 1) & 0x55) | ((c << 1) & 0xAA);
    c = ((c >> 2) & 0x33) | ((c << 2) & 0xCC);
    c = ((c >> 4) & 0x0F) | ((c << 4) & 0xF0);
    return c;
}

/******************************************************************************/
void spi_soft_init(void) {

    SETBIT(SPI_PORT, SCLK);
    CLRBIT(SPI_PORT, SDAT);
    SETBIT(SPI_PORT, LOAD);
}

/******************************************************************************/
void AD5624_write(WORD value) {

    BYTE j = 24;
    DWORD ddata;

    ddata = (DWORD)value | (0x18000) ;
    ddata = (ddata << 12);

    SETBIT(SPI_PORT, SCLK);
    CLRBIT(SPI_PORT, LOAD);

    do {
        SETBIT(SPI_PORT, SCLK);
        if((ddata & 0x80000000) == 0) CLRBIT(SPI_PORT, SDAT);
        else SETBIT(SPI_PORT, SDAT);
        CLRBIT(SPI_PORT, SCLK);
        ddata <<= 1;
        j--;
    } while(j);

    SETBIT(SPI_PORT, LOAD);

}

/******************************************************************************/

void spi_init(void) {
    SPI_CS_1();
    SCON1 = 0x03;
    OEE |= 0x01;

    //TMOD |= 0x20;                      /* timer 1 mode 2: 8-Bit reload          */
    //TH1   = 0xf3;                      /* reload value 2400 baud                */
    //TR1   = 1;
}

/******************************************************************************/
void SPIByteWriteU(BYTE d)	 {


    TI1 = FALSE; //Clear flag
    SBUF1 = d; //Write byte
    while(!TI1);  //Wait until transmit done



}

/******************************************************************************/
void SPIWriteDAC(BYTE h, BYTE l) {
    SPI_CS_0();
    SPIByteWriteU(invert(h));
    SPIByteWriteU(invert(l));
    SPI_CS_1();
}



/******************************************************************************/
/******************************************************************************/
void uart_init(void) {

    T2CON = 0x34 ;
    RCAP2H = 0xFF ;
    RCAP2L = 0x64;
    SCON0 = 0x5A ;
    TI = 1;


}
/******************************************************************************/
void uart_putc(BYTE b)	{

    while(!TI);
    TI = FALSE;
    SBUF0 = b;

}


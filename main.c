
#pragma NOIV               // Do not generate interrupt vectors
//-----------------------------------------------------------------------------
//   File:       FX2_to_extsyncFIFO.c
//   Contents:   Hooks required to implement FX2 GPIF to external sync. FIFO
//               interface using CY4265-15AC
//
//   Copyright (c) 2003 Cypress Semiconductor, Inc. All rights reserved
//-----------------------------------------------------------------------------
#include "fx2.h"
#include "fx2regs.h"
#include "fx2sdly.h"            // SYNCDELAY macro, see Section 15.14 of FX2 Tech.
// Ref. Manual for usage details.

#include "spi.h"


#define EXTFIFONOTFULL   GPIFREADYSTAT & bmBIT1
#define EXTFIFONOTEMPTY  GPIFREADYSTAT & bmBIT0

#define GPIFTRIGRD 4

#define GPIF_EP2 0
#define GPIF_EP4 1
#define GPIF_EP6 2
#define GPIF_EP8 3

extern BOOL GotSUD; // Received setup data flag
extern BOOL Sleep;
extern BOOL Rwuen;
extern BOOL Selfpwr;

BYTE Configuration; // Current configuration
BYTE AlternateSetting; // Alternate settings
BOOL in_enable = FALSE; // flag to enable IN transfers
BOOL enum_high_speed = FALSE; // flag to let firmware know FX2 enumerated at high speed
extern const char xdata FlowStates[36];

#define  TESTBIT(var, b)   	((var) & (1 <<(b)))
#define  SETBIT(var, b)    	((var) |= (1 << (b)))
#define  CLRBIT(var, b)    	((var) &= ~(1 << (b)))


// my
BOOL scan_en = FALSE; // flag to enable autochange channels
BYTE current_ch = 0;
WORD d_ch_en = 0xffff; // активность каналов


//-----------------------------------------------------------------------------
// Task Dispatcher hooks
//   The following hooks are called by the task dispatcher.
//-----------------------------------------------------------------------------
void Setup_FLOWSTATE_Write(void);
void Setup_FLOWSTATE_Read(void);
void GpifInit();


/*************************************************************************************************/
void TD_Init(void) { // Called once at startup
    // set the CPU clock to 48MHz
    CPUCS = ((CPUCS & ~bmCLKSPD) | bmCLKSPD1);
    SYNCDELAY;

    EP2CFG = 0xA0; // EP2OUT, bulk, size 512, 4x buffered
    SYNCDELAY;
    EP4CFG = 0x00; // EP4 not valid
    SYNCDELAY;
    EP6CFG = 0xE0; // EP6IN, bulk, size 512, 4x buffered
    SYNCDELAY;
    EP8CFG = 0x00; // EP8 not valid
    SYNCDELAY;


    FIFORESET = 0x80; // set NAKALL bit to NAK all transfers from host
    SYNCDELAY;
    FIFORESET = 0x02; // reset EP2 FIFO
    SYNCDELAY;
    FIFORESET = 0x06; // reset EP6 FIFO
    SYNCDELAY;
    FIFORESET = 0x00; // clear NAKALL bit to resume normal operation
    SYNCDELAY;

    EP2FIFOCFG = 0x01; // allow core to see zero to one transition of auto out bit
    SYNCDELAY;
    EP2FIFOCFG = 0x11; // auto out mode, disable PKTEND zero length send, word ops
    SYNCDELAY;
    EP6FIFOCFG = 0x09; // auto in mode, disable PKTEND zero length send, word ops
    SYNCDELAY;

    GpifInit(); // initialize GPIF registers

    SYNCDELAY;
    EP2GPIFFLGSEL = 0x01; // For EP2OUT, GPIF uses EF flag
    SYNCDELAY;
    EP6GPIFFLGSEL = 0x02; // For EP6IN, GPIF uses FF flag
    SYNCDELAY;

    // global flowstate register initializations

    FLOWLOGIC = FlowStates[19]; // 0011 0110b - LFUNC[1:0] = 00 (A AND B), TERMA/B[2:0]=110 (FIFO Flag)
    SYNCDELAY;
    FLOWSTB = FlowStates[23]; // 0000 0100b - MSTB[2:0] = 100 (CTL4), not used as strobe
    SYNCDELAY;
    GPIFHOLDAMOUNT = FlowStates[26]; // hold data for one half clock (10ns) assuming 48MHz IFCLK
    SYNCDELAY;
    FLOWSTBEDGE = FlowStates[24]; // move data on both edges of clock
    SYNCDELAY;
    FLOWSTBHPERIOD = FlowStates[25]; // 20.83ns half period
    SYNCDELAY;


    // Инициализирую порты
    // Канал
    CLRBIT(IFCONFIG, 2);
    PORTECFG = 0x10;
    OEE = 0x00;
    IOE = 0x00;


    PORTACFG = 0x00;
    OEA = 0xff;
    IOA = 0;



    spi_soft_init();
    spi_init();

    uart_init();





}

/*************************************************************************************************/
void set_channel(BYTE ch) {

    //static BYTE d_count;

    IOA &= 0x0f;
    IOA |= ((ch << 4) & 0xf0);


}




/*************************************************************************************************/
void TD_Poll(void) {
    //    static WORD d_count;


    if(in_enable) {                           // if IN transfers are enabled
        if(GPIFTRIG & 0x80) {                   // if GPIF interface IDLE
            if(EXTFIFONOTEMPTY) {                 // if external FIFO is not empty
                if(!(EP68FIFOFLGS & 0x01)) {        // if EP6 FIFO is not full


                    if(scan_en == TRUE) {


                        while(!TESTBIT(d_ch_en, current_ch)) {
                            current_ch++;
                            current_ch &= 0x0f;
                        }

                        IOA &= 0x0f;
                        IOA |= ((current_ch << 4) & 0xF0);

                        current_ch ++;
                        current_ch &= 0x0f;

                    }



                    if(enum_high_speed) {
                        SYNCDELAY;
                        GPIFTCB1 = 0x01;                // setup transaction count (512 bytes/2 for word wide -> 0x0100)
                        SYNCDELAY;
                        GPIFTCB0 = 0x00;
                        SYNCDELAY;
                    } else {
                        SYNCDELAY;
                        GPIFTCB1 = 0x00;                // setup transaction count (64 bytes/2 for word wide -> 0x20)
                        SYNCDELAY;
                        GPIFTCB0 = 0x20;
                        SYNCDELAY;
                    }

                    Setup_FLOWSTATE_Read();           // setup FLOWSTATE registers for FIFO Read operation

                    SYNCDELAY;
                    GPIFTRIG = GPIFTRIGRD | GPIF_EP6; // launch GPIF FIFO READ Transaction to EP6 FIFO
                    SYNCDELAY;

                    while(!(GPIFTRIG & 0x80)) {       // poll GPIFTRIG.7 GPIF Done bit
                        ;
                    }

                    SYNCDELAY;
                }
            }
        }
    }



}

/*************************************************************************************************/
BOOL TD_Suspend(void) { // Called before the device goes into suspend mode
    return (TRUE);
}

/*************************************************************************************************/
BOOL TD_Resume(void) { // Called after the device resumes
    return (TRUE);
}

//-----------------------------------------------------------------------------
// Device Request hooks
//   The following hooks are called by the end point 0 device request parser.
//-----------------------------------------------------------------------------

/*************************************************************************************************/
BOOL DR_GetDescriptor(void) {
    return (TRUE);
}

/*************************************************************************************************/
BOOL DR_SetConfiguration(void) { // Called when a Set Configuration command is received
    if(EZUSB_HIGHSPEED()) {  // FX2 enumerated at high speed
        SYNCDELAY; //
        EP6AUTOINLENH = 0x02; // set AUTOIN commit length to 512 bytes
        SYNCDELAY; //
        EP6AUTOINLENL = 0x00;
        SYNCDELAY;
        enum_high_speed = TRUE;

        //CLRBIT(IOE, 4);
        //SETBIT(IOE, 6);

    } else { // FX2 enumerated at full speed
        SYNCDELAY;
        EP6AUTOINLENH = 0x00; // set AUTOIN commit length to 64 bytes
        SYNCDELAY;
        EP6AUTOINLENL = 0x40;
        SYNCDELAY;
        enum_high_speed = FALSE;

        //CLRBIT(IOE, 6);
        //SETBIT(IOE, 4);
    }

    Configuration = SETUPDAT[2];


    // Автоматическое включение питания для внешних устройств
    SETBIT(IOE, 5);
    // Автоматическое включение   генератора
    CLRBIT(IOA, 0);

    return (TRUE); // Handled by user code
}

/*************************************************************************************************/
BOOL DR_GetConfiguration(void) { // Called when a Get Configuration command is received
    EP0BUF[0] = Configuration;
    EP0BCH = 0;
    EP0BCL = 1;
    return (TRUE); // Handled by user code
}

/*************************************************************************************************/
BOOL DR_SetInterface(void) { // Called when a Set Interface command is received
    AlternateSetting = SETUPDAT[2];
    return (TRUE); // Handled by user code
}

/*************************************************************************************************/
BOOL DR_GetInterface(void) { // Called when a Set Interface command is received
    EP0BUF[0] = AlternateSetting;
    EP0BCH = 0;
    EP0BCL = 1;
    return (TRUE); // Handled by user code
}

/*************************************************************************************************/
BOOL DR_GetStatus(void) {
    return (TRUE);
}

/*************************************************************************************************/
BOOL DR_ClearFeature(void) {
    return (TRUE);
}

/*************************************************************************************************/
BOOL DR_SetFeature(void) {
    return (TRUE);
}




/*************************************************************************************************/
#define VX_B0 0xB0 //  активность каналов
#define VX_B1 0xB1 //  Echo/  reserv
#define VX_B2 0xB2 //  power
#define VX_B3 0xB3 // enable IN transfers
#define VX_B4 0xB4 //  вкл сканирование каналов
#define VX_B5 0xB5 //  Установка активного канала
#define VX_B6 0xB6 //  Установка коэфф усиления
#define VX_B7 0xB7 //  //Установка / сброс бита разрешения тактового генератора  
#define VX_B8 0xB8 // 	 DAC1
#define VX_B9 0xB9 //	 DAC2
#define VX_B10 0xBA //	 DAC3
#define VX_B11 0xBB //	 DAC4
#define VX_BC 0xBC //	 DAC
#define VX_BD 0xBD //	 UART
#define VX_BE 0xBD //	 Registers in Altera
/*************************************************************************************************/
BOOL DR_VendorCmnd(void) {
    WORD dac_data;
    WORD dac_num;

    switch(SETUPDAT[1]) {  // активность каналов при сканировании
        case VX_B0:

            d_ch_en = (SETUPDAT[2] << 8) | SETUPDAT[3];
            if(d_ch_en == 0) d_ch_en = 1;


            EP0BUF[0] = VX_B0;
            SYNCDELAY;
            EP0BUF[1] = SETUPDAT[2];
            SYNCDELAY;
            EP0BUF[2] = SETUPDAT[3];
            SYNCDELAY;
            EP0BCH = 0;
            EP0BCL = 3;
            EP0CS |= bmHSNAK;
            break;

        case VX_B1:

            *EP0BUF = VX_B1;
            EP0BCH = 0;
            EP0BCL = 1;
            EP0CS |= bmHSNAK;
            break;

            //Включение питания для внешних устройств
        case VX_B2:

            if(SETUPDAT[2] == 1) {
                SETBIT(OEE, 5);
                SYNCDELAY;
                SETBIT(IOE, 5);
                SYNCDELAY;

            } else {
                CLRBIT(IOE, 5);
                SYNCDELAY;
            }

            EP0BUF[0] = VX_B2;
            EP0BUF[1] = SETUPDAT[2];

            EP0BCH = 0;
            EP0BCL = 2; // Arm endpoint with # bytes to transfer
            EP0CS |= bmHSNAK; // Acknowledge handshake phase of device request
            break;


        case VX_B3: // enable / disable IN transfers


            if(SETUPDAT[2] == 1) {
                in_enable = TRUE;

            } else {
                in_enable = FALSE;
            }

            *EP0BUF = VX_B3;
            EP0BUF[1] = SETUPDAT[2];
            EP0BCH = 0;
            EP0BCL = 2;
            EP0CS |= bmHSNAK;
            break;

        case VX_B4: // вкл сканирование каналов

            if(SETUPDAT[2] == 1) {
                scan_en = TRUE;
            } else {
                scan_en = FALSE;
            }

            current_ch = 0;

            *EP0BUF = VX_B4;
            EP0BUF[1] = SETUPDAT[2];
            SYNCDELAY;
            EP0BCH = 0;
            EP0BCL = 2;
            EP0CS |= bmHSNAK;

            break;


        case VX_B5: // Установка активного канала

            EP0BUF[0] = VX_B5;
            SYNCDELAY;
            EP0BUF[1] = SETUPDAT[2];
            SYNCDELAY;

            set_channel(SETUPDAT[2]);

            EP0BCH = 0;
            EP0BCL = 2;
            EP0CS |= bmHSNAK;
            break;

        case VX_B6: // Установка коэфф усиления

            EP0BUF[0] = VX_B6;
            SYNCDELAY;
            EP0BUF[1] = SETUPDAT[2];
            SYNCDELAY;


            IOE &= 0xf0;
            IOE |= (SETUPDAT[2] & 0x0f);


            EP0BCH = 0;
            EP0BCL = 2;
            EP0CS |= bmHSNAK;
            break;


        case VX_B7: //Установка / сброс бита разрешения тактового генератора

            EP0BUF[0] = VX_B7;
            SYNCDELAY;
            if(SETUPDAT[2] == 1) CLRBIT(IOA, 0);
            else SETBIT(IOA, 0);

            EP0BUF[1] = SETUPDAT[2];
            SYNCDELAY;
            EP0BCH = 0;
            EP0BCL = 2;
            EP0CS |= bmHSNAK;

            break;

        case VX_B8: // Комманда установки ЦАП 1
        case VX_B9: // Комманда установки ЦАП 2
        case VX_B10: // Комманда установки ЦАП 3
        case VX_B11: // Комманда установки ЦАП 4

            dac_data = SETUPDAT[2] | (SETUPDAT[3] << 8);
            dac_data &=  0x7fff;

            switch(SETUPDAT[1]) {
                case VX_B8:
                    dac_num = 0x0000;
                    break;
                case VX_B9:
                    dac_num = 0x1000;
                    break;
                case VX_B10:
                    dac_num = 0x2000;
                    break;
                case VX_B11:
                    dac_num = 0x3000;
                    break;
            }


            AD5624_write(dac_num | dac_data);


            EP0BUF[0] = SETUPDAT[1];
            SYNCDELAY;
            EP0BUF[1] = SETUPDAT[2];
            SYNCDELAY;
            EP0BUF[2] = SETUPDAT[3];
            SYNCDELAY;
            EP0BCH = 0;
            EP0BCL = 3;
            EP0CS |= bmHSNAK;
            break;

        case VX_BC:
            SPIWriteDAC(SETUPDAT[3], SETUPDAT[2]);
            EP0BUF[0] = SETUPDAT[3];
            SYNCDELAY;
            EP0BUF[1] = SETUPDAT[2];
            SYNCDELAY;

            EP0BCH = 0;
            EP0BCL = 2;
            EP0CS |= bmHSNAK;
            break;

        case VX_BD:
            uart_putc(SETUPDAT[3]);
            uart_putc(SETUPDAT[2]);

            EP0BUF[0] = SETUPDAT[3];
            SYNCDELAY;
            EP0BUF[1] = SETUPDAT[2];
            SYNCDELAY;

            EP0BCH = 0;
            EP0BCL = 2;
            EP0CS |= bmHSNAK;
            break;
        default:
            return (TRUE);
    }

    return (FALSE);
}




//-----------------------------------------------------------------------------
// USB Interrupt Handlers
//   The following functions are called by the USB interrupt jump table.
//-----------------------------------------------------------------------------

// Setup Data Available Interrupt Handler

void ISR_Sudav(void) interrupt 0 {
    GotSUD = TRUE; // Set flag
    EZUSB_IRQ_CLEAR();
    USBIRQ = bmSUDAV; // Clear SUDAV IRQ
}

// Setup Token Interrupt Handler

void ISR_Sutok(void) interrupt 0 {
    EZUSB_IRQ_CLEAR();
    USBIRQ = bmSUTOK; // Clear SUTOK IRQ
}

void ISR_Sof(void) interrupt 0 {
    EZUSB_IRQ_CLEAR();
    USBIRQ = bmSOF; // Clear SOF IRQ
}

void ISR_Ures(void) interrupt 0 {
    // whenever we get a USB reset, we should revert to full speed mode
    pConfigDscr = pFullSpeedConfigDscr;
    ((CONFIGDSCR xdata *) pConfigDscr)->type = CONFIG_DSCR;
    pOtherConfigDscr = pHighSpeedConfigDscr;
    ((CONFIGDSCR xdata *) pOtherConfigDscr)->type = OTHERSPEED_DSCR;

    EZUSB_IRQ_CLEAR();
    USBIRQ = bmURES; // Clear URES IRQ
}

void ISR_Susp(void) interrupt 0 {
    Sleep = TRUE;
    EZUSB_IRQ_CLEAR();
    USBIRQ = bmSUSP;
}

void ISR_Highspeed(void) interrupt 0 {
    if(EZUSB_HIGHSPEED()) {
        pConfigDscr = pHighSpeedConfigDscr;
        ((CONFIGDSCR xdata *) pConfigDscr)->type = CONFIG_DSCR;
        pOtherConfigDscr = pFullSpeedConfigDscr;
        ((CONFIGDSCR xdata *) pOtherConfigDscr)->type = OTHERSPEED_DSCR;

    }

    EZUSB_IRQ_CLEAR();
    USBIRQ = bmHSGRANT;


}

void ISR_Ep0ack(void) interrupt 0 {
}

void ISR_Stub(void) interrupt 0 {
}

void ISR_Ep0in(void) interrupt 0 {
}

void ISR_Ep0out(void) interrupt 0 {
}

void ISR_Ep1in(void) interrupt 0 {
}

void ISR_Ep1out(void) interrupt 0 {
}

void ISR_Ep2inout(void) interrupt 0 {
}

void ISR_Ep4inout(void) interrupt 0 {
}

void ISR_Ep6inout(void) interrupt 0 {
}

void ISR_Ep8inout(void) interrupt 0 {
}

void ISR_Ibn(void) interrupt 0 {
}

void ISR_Ep0pingnak(void) interrupt 0 {
}

void ISR_Ep1pingnak(void) interrupt 0 {
}

void ISR_Ep2pingnak(void) interrupt 0 {
}

void ISR_Ep4pingnak(void) interrupt 0 {
}

void ISR_Ep6pingnak(void) interrupt 0 {
}

void ISR_Ep8pingnak(void) interrupt 0 {
}

void ISR_Errorlimit(void) interrupt 0 {
}

void ISR_Ep2piderror(void) interrupt 0 {
}

void ISR_Ep4piderror(void) interrupt 0 {
}

void ISR_Ep6piderror(void) interrupt 0 {
}

void ISR_Ep8piderror(void) interrupt 0 {
}

void ISR_Ep2pflag(void) interrupt 0 {
}

void ISR_Ep4pflag(void) interrupt 0 {
}

void ISR_Ep6pflag(void) interrupt 0 {
}

void ISR_Ep8pflag(void) interrupt 0 {
}

void ISR_Ep2eflag(void) interrupt 0 {
}

void ISR_Ep4eflag(void) interrupt 0 {
}

void ISR_Ep6eflag(void) interrupt 0 {
}

void ISR_Ep8eflag(void) interrupt 0 {
}

void ISR_Ep2fflag(void) interrupt 0 {
}

void ISR_Ep4fflag(void) interrupt 0 {
}

void ISR_Ep6fflag(void) interrupt 0 {
}

void ISR_Ep8fflag(void) interrupt 0 {
}

void ISR_GpifComplete(void) interrupt 0 {
}

void ISR_GpifWaveform(void) interrupt 0 {
}




void Setup_FLOWSTATE_Read(void) {
    FLOWSTATE = FlowStates[18];  // 1000 0011b - FSE=1, FS[2:0]=003
    SYNCDELAY;
    FLOWEQ0CTL = FlowStates[20]; // CTL1/CTL2 = 0 when flow condition equals zero (data flows)
    SYNCDELAY;
    FLOWEQ1CTL = FlowStates[21]; // CTL1/CTL2 = 1 when flow condition equals one (data does not flow)
    SYNCDELAY;
}


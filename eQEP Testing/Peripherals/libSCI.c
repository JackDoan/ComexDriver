/*
 * libSCI.c
 *
 *  Created on: May 22, 2015
 *      Author: jad140230
 */

#include "DSP28x_Project.h"

void scia_init(void) {
    InitSciaGpio();
    /*
     * Initalize the SCI FIFO
     */
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x2044;
    //SciaRegs.SCIFFRX.all=0x0000;
    SciaRegs.SCIFFCT.all=0x0;

    SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
    SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.all =0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA =1;
    SciaRegs.SCICTL2.bit.RXBKINTENA =1;

    SciaRegs.SCIHBAUD    =0x0000;  // 38400 baud @LSPCLK = 15MHz (60 MHz SYSCLK).
    SciaRegs.SCILBAUD    =0x0030;   // 38400 baud @LSPCLK = 15MHz (60 MHz SYSCLK).

    SciaRegs.SCICTL1.all =0x0023;  // Release SCI from Reset
}

/*
 * Transmit a character from the SCI
 */
void scia_xmit(int a) {
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF=a;
}

void scia_msg(char* msg) {
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}

void scia_PrintLF( void ) {
        scia_msg("\r\n");
}


/*
 * write bytes to the screen
 */

static unsigned char btoh( unsigned char num ) {
   num = num & 0x0F;
   return ( num<10 )? num+'0' : num-10+'A';
}

void scia_Byte2Hex( Uint16 byte ) {
        unsigned char c;
        int i;
        Uint16 digit;

        scia_xmit( '0' );
        scia_xmit( 'x' );
        for (i = 3; i >= 0; i--)
        {
            digit = (byte >> (i * 4)) & 0xf;
            c = btoh( digit );
            scia_xmit( c );
        }
        scia_xmit( ' ' );
}


/*
 * Read in a char
 */
Uint16 scia_read(void)
{
	while(SciaRegs.SCIFFRX.bit.RXFFST ==0) { }
        //while(SciaRegs.SCIFFRX.bit.RXFFST !=1) { } // wait for XRDY =1 for empty state
        return (SciaRegs.SCIRXBUF.all);
}


void printMenu(char** menu, int size) {
	int i = 0;
	while (i <= size) {
		scia_msg(menu[i]);
		scia_PrintLF();
		i++;
	}

}

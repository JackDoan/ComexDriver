/*
 * libSCI.h
 *
 *  Created on: May 22, 2015
 *      Author: jad140230
 */

#ifndef LIBSCI_H_
#define LIBSCI_H_


Uint16 scia_read(void);

void scia_init(void);
void scia_xmit(int a);
void scia_msg(char *msg);
void scia_PrintLF( void );
void scia_Byte2Hex( Uint16 byte );
static unsigned char btoh( unsigned char num );

void printMenu(char** menu, int size);



#endif /* LIBSCI_H_ */

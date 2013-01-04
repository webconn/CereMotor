#ifndef INCLUDE_SAT_DRIVER
#define INCLUDE_SAT_DRIVER

/*
 * Section 1. Definition of commands' codes
 */

#define EGDIR 0x01
#define DGDIR 0x02
#define EGLEN 0x03
#define EGPAT 0x04
#define DGSPD 0x05
#define RGSPD 0x06
#define EGSPD 0x07
#define DSDIR 0x08
#define DSSPD 0x09
#define RSSPD 0x0A
#define EVRST 0x0B
#define SWMAN 0x0C
#define SWAUT 0x0D
#define RSPRP 0x0E
#define RSDIF 0x0F
#define DSFSP 0x10

/*
 * Section 2. Definition of low-level functions
 */

void _sat_exchange(USART_TypeDef* USARTx, uint8_t command, uint8_t * io);

#endif

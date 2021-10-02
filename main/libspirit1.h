#ifndef LIBSPIRTI1_H
#define LIBSPIRTI1_H
#include <stdbool.h>


void spirit_init(void);
uint8_t spirit_get_buff(uint8_t *buff, uint8_t buffLen);
void spirit_send_buff(uint8_t *buff, uint8_t buffLen, uint8_t dstAdr);

#endif /* LIBSPIRIT1_H */
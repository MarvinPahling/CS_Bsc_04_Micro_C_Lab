#ifndef term_h
#define term_h

#include "../main.h"

         void term_init(void);

//return  0 -> Zeichen vorhanden
//       -1 -> Kein Zeichen vorhanden
NODISCARD int term_read(unsigned char *c,asyncsync_t asyncsync);

//return  0 -> Senden möglich
//       -1 -> Senden (derzeit) nicht möglich
NODISCARD int term_char_possible(void);

//return  0 -> Zeichen gesendet
//       -1 -> Zeichen nicht gesendet
NODISCARD int term_char(unsigned char c,asyncsync_t asyncsync);

//return  0 -> String gesendet
//       -1 -> String nicht (vollständig) gesendet
NODISCARD int term_string(const char *str,asyncsync_t asyncsync);

//return  0 -> Hexzahl gesendet
//       -1 -> Hexzahl nicht (vollständig) gesendet
NODISCARD int term_hex(unsigned int val, unsigned int places,asyncsync_t asyncsync);

//return  0 -> Zahl gesendet
//       -1 -> Zahl nicht (vollständig) gesendet
NODISCARD int term_unsigned(unsigned int val, unsigned int places, asyncsync_t asyncsync);

//return  0 -> Zahl gesendet
//       -1 -> Zahl nicht (vollständig) gesendet
NODISCARD int term_int(int val, unsigned int places, asyncsync_t asyncsync);

#endif
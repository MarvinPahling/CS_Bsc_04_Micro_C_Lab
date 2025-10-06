#include <string.h>  //fuer strlen() size_t
#include "term.h"
#include "helper.h"  //fuer unsigned2string()


#if defined(MODE_RAM) || defined(MODE_SIM)
  #include "../trace32/udmon3.h"
#endif

#ifdef MODE_GDBOPENOCD_RAM
  #include "../openOCD/dcc_stdio.h"  
#endif

#if defined(MODE_RAM) || defined(MODE_SIM)
  #include <stdlib.h>   //fuer NULL
  #include "byte_fifo_cb.h"
  byte_fifo_cb_t fifo_rx = BYTE_FIFO_CB_INIT(64,NULL);
  byte_fifo_cb_t fifo_tx = BYTE_FIFO_CB_INIT(64,NULL);
#endif

void term_init(void)
{
#if defined(MODE_RAM) || defined(MODE_SIM)
  udmon3_term_init(&fifo_rx,&fifo_tx,&fifo_tx.cb);
#endif  

#if defined(MODE_ROM)
  //hier könnte die UART Schnittstelle genutzt werden
#endif

#if defined(MODE_SAMBA)
  //hier könnte die bereits initialisierte USB Schnittstelle genutzt werden
#endif

#if defined(MODE_GDBOPENOCD_RAM)
  //Keine Initialisierung notwendig!
  //Nur (blockierendes) Senden möglich. Kein Empfang
#endif
}


//return  0 -> Zeichen vorhanden
//       -1 -> Kein Zeichen vorhanden
int term_read(unsigned char *c,asyncsync_t asyncsync)
{
#if defined(MODE_RAM) || defined(MODE_SIM)
	if(asyncsync == ASYNCSYNC_NONBLOCK)
		return byte_fifo_cb_get(&fifo_rx,c);
	else if(asyncsync == ASYNCSYNC_BLOCK) {
		while(byte_fifo_cb_get(&fifo_rx,c) != 0);
	}
	else if(asyncsync == ASYNCSYNC_ASYNCGET)
		return -1;
	return 0;
#elif defined(MODE_GDBOPENOCD_RAM)
	(void)c;
	(void)asyncsync;
	return -1; //keine Read Funktion in dcc_stdio vorhanden!
#else
	(void)c;
	(void)asyncsync;
	return -1;
#endif
}


//return  0 -> Senden möglich
//       -1 -> Senden (derzeit) nicht möglich
int term_char_possible(void)
{
#if defined(MODE_RAM) || defined(MODE_SIM)
	return byte_fifo_cb_put_possible(&fifo_tx);
#elif defined(MODE_GDBOPENOCD_RAM)
	return 0;  //Es kann nur 'blockierend' gesendet werden
	           //Daher Senden immer möglich
#else
	return -1;
#endif
}


//return  0 -> Zeichen gesendet
//       -1 -> Zeichen nicht gesendet
int term_char(unsigned char c,asyncsync_t asyncsync)
{
#if defined(MODE_RAM) || defined(MODE_SIM)
	if(asyncsync == ASYNCSYNC_NONBLOCK)
		return byte_fifo_cb_put(&fifo_tx,c);
	else if(asyncsync == ASYNCSYNC_BLOCK) {
		while(byte_fifo_cb_put(&fifo_tx,c) != 0);
	}
	else if(asyncsync == ASYNCSYNC_ASYNCGET)
		return -1;
	return 0;
#elif defined(MODE_GDBOPENOCD_RAM)
	(void)asyncsync;   //wird hier nicht unterstützt
	dbg_write_char((char) c);
	return 0;
#else
	return -1;
#endif
}


//return  0 -> String gesendet
//       -1 -> String nicht (vollständig) gesendet
int term_string(const char *str,asyncsync_t asyncsync)
{
	if(asyncsync == ASYNCSYNC_ASYNCGET)
		return -1;
#if defined(MODE_RAM) || defined(MODE_SIM)
    size_t ret;
	ret = byte_fifo_cb_putsize(&fifo_tx,(const unsigned char *)str,strlen(str));
	if(ret == 0)
		return 0;
	if(asyncsync == ASYNCSYNC_NONBLOCK)
		return -1;
	str+=strlen(str)-ret;
	do
	{
		while(byte_fifo_cb_put(&fifo_tx,(unsigned char)*str) != 0);
		str++;
	}
	while(*str != 0);
	return 0;
#elif defined(MODE_GDBOPENOCD_RAM)
	(void)asyncsync;   //wird hier nicht unterstützt
	dbg_write_str(str);	
	return 0;
#else
	return -1;
#endif
}

//return  0 -> Hexzahl gesendet
//       -1 -> Hexzahl nicht (vollständig) gesendet
int term_hex(unsigned int val, unsigned int places,asyncsync_t asyncsync)
{
	char buf[9];
	return term_string(hex2string(buf,val,places),asyncsync);
}

//return  0 -> Zahl gesendet
//       -1 -> Zahl nicht (vollständig) gesendet
int term_unsigned(unsigned int val, unsigned int places, asyncsync_t asyncsync)
{
	char buf[12];
	return term_string(unsigned2string(buf,val,places,0),asyncsync);
}

//return  0 -> Zahl gesendet
//       -1 -> Zahl nicht (vollständig) gesendet
int term_int(int val, unsigned int places, asyncsync_t asyncsync)
{
	char buf[12];
	return term_string(unsigned2string(buf,(val < 0) ? -val : val, places, (val < 0)),asyncsync);
}

#ifndef  __HELPER_H__
#define __HELPER_H__


//buf    -> Buffer mit mindestens 12 Zeichen Platz
//val    -> zu konvertierender Wert
//places -> Mindestbreite (Ausgabe ist rechtsbündig)
//sign   -> Vorzeichen 1->'-'
//return -> Zeiger auf 1. Zeichen des konvertierten Strings
char *unsigned2string(char buf[12], unsigned int val, unsigned int places, unsigned int sign);

//buf    -> Buffer mit mindestens 9 Zeichen Platz
//val    -> zu konvertierender Wert
//places -> Mindestbreite (mit '0' auffüllen, wenn Zahl 'kleiner' ist)
//return -> Zeiger auf 1. Zeichen des konvertierten Strings
char *hex2string(char buf[9],unsigned int val, unsigned int places);

#endif
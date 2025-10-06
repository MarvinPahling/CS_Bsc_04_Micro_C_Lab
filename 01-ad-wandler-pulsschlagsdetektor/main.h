#ifndef main_h
#define main_h
#include <stdint.h>

//ASCII-Zeichen
//\a The “alert” character, Ctrl-g, ASCII code 7 (BEL). (This usually makes some sort of audible noise.)
//\b Backspace, Ctrl-h, ASCII code 8 (BS).
//\f Formfeed, Ctrl-l, ASCII code 12 (FF).
//\n Newline, Ctrl-j, ASCII code 10 (LF).
//\r Carriage return, Ctrl-m, ASCII code 13 (CR).
//\t Horizontal TAB, Ctrl-i, ASCII code 9 (HT).
//\v Vertical tab, Ctrl-k, ASCII code 11 (VT).
//\nnn The octal value nnn, where nnn stands for 1 to 3 digits between ‘0’ and ‘7’. For example, the code for the ASCII ESC (escape) character is ‘\033’.
//\xhh...The hexadecimal value hh, where hh stands for a sequence of hexadecimal digits (‘0’–‘9’, and either ‘A’–‘F’ or ‘a’–‘f’). 
//       Like the same construct in ISO C, the escape sequence continues until 
//       the first nonhexadecimal digit is seen. (c.e.) However, using more 
//       than two hexadecimal digits produces undefined results. 
//       (The ‘\x’ escape sequence is not allowed in POSIX awk.)
//\/ A literal slash (necessary for regexp constants only). This sequence 
//       is used when you want to write a regexp constant that contains a 
//       slash. Because the regexp is delimited by slashes, you need to 
//       escape the slash that is part of the pattern, in order to tell 
//       awk to keep processing the rest of the regexp.
//\"  A literal double quote (necessary for string constants only). 
//       This sequence is used when you want to write a string constant 
//       that contains a double quote. Because the string is delimited by 
//       double quotes, you need to escape the quote that is part of 
//       the string, in order to tell awk to keep processing the rest
//       of the string.#define ANSI_BLACK   30

//https://www-user.tu-chemnitz.de/~heha/hsn/terminal/terminal.htm
//https://gist.github.com/ConnerWill/d4b6c776b509add763e17f9f113fd25b
//Cursor Control
#define VT100_CURSORHOME      "\e[H"
#define VT100_CLEARSCREEN     "\e[2J"
#define VT100_GOTOYX          "\e[%d;%dH"
#define VT100_SAVEPOS         "\e7"
#define VT100_RESTOREPOS      "\e8"
#define VT100_ROLLY1Y2        "\e[%d;%dr"
#define VT100_ROLLOFF         "\e[r"
//Erase Functions
#define VT100_ERASEFROMCURSOR "\e[0J"
#define VT100_ERASETOCURSOR   "\e[1J"
#define VT100_ERASESCREEN     "\e[2J"
#define VT100_DEL_UNTILEOL    "\e[K"
#define VT100_DEL_TOCUR       "\e[1K"
#define VT100_DEL_LINE        "\e[2K"
//Color/Graphics Mode
#define VT100_DEFAULT         "\e[0m"  //stellt hellgraue Schrift auf schwarzem Grund ein
#define VT100_FETT              "\e[1m"
#define VT100_FETT_AUS          "\e[22m"
#define VT100_UNTERSTRICHEN     "\e[4m"
#define VT100_UNTERSTRICHEN_AUS "\e[24m"
#define VT100_BLINKEN           "\e[5m"
#define VT100_BLINKEN_AUS       "\e[25m"
#define VT100_INVERS            "\e[7m"
#define VT100_INVERS_AUS        "\e[27m"
#define VT100_UNSICHTBAR        "\e[8m"
#define VT100_SICHTBAR          "\e[28m"
#define VT100_VORDERGRUND_SCHWARZ "\e[30m"
#define VT100_VORDERGRUND_ROT     "\e[31m"
#define VT100_VORDERGRUND_GRUEN   "\e[32m"
#define VT100_VORDERGRUND_BRAUN   "\e[33m"
#define VT100_VORDERGRUND_BLAU    "\e[34m"
#define VT100_VORDERGRUND_BLAUROT "\e[35m"
#define VT100_VORDERGRUND_ZYAN    "\e[36m"
#define VT100_VORDERGRUND_WEISS   "\e[37m"  //Default
#define VT100_VORDERGRUND_DEFAULT "\e[39m"  //Nicht VT100
#define VT100_HINTERGRUND_SCHWARZ "\e[40m"  //Default
#define VT100_HINTERGRUND_ROT     "\e[41m"  
#define VT100_HINTERGRUND_GRUEN   "\e[42m"  
#define VT100_HINTERGRUND_BRAUN   "\e[43m"  
#define VT100_HINTERGRUND_BLAU    "\e[44m"  
#define VT100_HINTERGRUND_BLAUROT "\e[45m"  
#define VT100_HINTERGRUND_ZYAN    "\e[46m"  
#define VT100_HINTERGRUND_WEISS   "\e[47m"  
#define VT100_HINTERGRUND_DEFAULT "\e[49m"

                                                         /* Main Clock [Hz] */
#define MAINCK            18432000
                                     /* Maseter Clock (PLLRC div by 2) [Hz] */
#define MCK               47923200
                                             /* System clock tick rate [Hz] */
#define BSP_TICKS_PER_SEC 1000

//#define NODISCARD __attribute__((warn_unused_result))  
#define NODISCARD  [[nodiscard]]

typedef enum __attribute__((packed)) {ASYNCSYNC_NONBLOCK,ASYNCSYNC_BLOCK,ASYNCSYNC_ASYNCGET} asyncsync_t;

typedef enum {SENSOR_1,SENSOR_2,SENSOR_3,SENSOR_4,SENSOR_MAX} sensor_t;

typedef enum {MOTOR_A,MOTOR_B,MOTOR_C} motor_t;

typedef enum {MOTOR_BREAK, MOTOR_FLOAT} motor_zustand_t;

typedef enum {SENSOR_OFF=0x00, SENSOR_9V_PULSED=0x01, SENSOR_9V=0x10 } sensor_power_t;

typedef enum {BATTERY_AA, BATTERY_ACCU} battery_t;

typedef struct {
	uint8_t orange : 1;
	uint8_t left : 1;
	uint8_t right : 1;
	uint8_t grey : 1;
	uint8_t reserved : 4;
} button_t;

#define I2C_BAUDRATE 10000  //Orignal 9600

extern uint32_t __stack_start__[];   //Definiert in link.ld
extern uint32_t __stack_end__;       //Definiert in link.ld
#define STACK_FILL 0x11111111

static __inline__ void stack_fill(void) __attribute__((always_inline));
static __inline__ void stack_fill(void)
{
	         uint32_t *ptr;
	register uint32_t *sp asm("r13");
	for(ptr=&__stack_start__[0];ptr<sp;ptr++)
		*ptr=STACK_FILL;
}

static __inline__ int32_t stack_check(void) __attribute__((always_inline));
static __inline__ int32_t stack_check(void)
{
	         uint32_t *ptr;
//	register uint32_t *sp asm("r13");
	for(ptr=&__stack_start__[0];*ptr==STACK_FILL;++ptr);
	return (int32_t)(ptr-&__stack_start__[0]);
}

#endif

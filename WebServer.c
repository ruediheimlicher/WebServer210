/*********************************************
 * vim:sw=8:ts=8:si:et
 * To use the above modeline in vim you must have "set modeline" in your .vimrc
 * Author: Guido Socher
 * Copyright: GPL V2
 * See http://www.gnu.org/licenses/gpl.html
 *
 * Chip type           : Atmega88 or Atmega168 or Atmega328 with ENC28J60
 * Note: there is a version number in the text. Search for tuxgraphics
 *********************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "ip_arp_udp_tcp.c"
#include "websrv_help_functions.c"
#include "enc28j60.h"
//#include "timeout.h"
#include "net.h"

#include <avr/wdt.h>
#include "lcd.c"
#include "adc.c"
//#include "websr.c"
#include "web_SPI.c"
//#include "out_slave.c"
#include "datum.c"
#include "version.c"
#include "homedata.c"
//# include "twimaster.c"
//***********************************
//									*
//									*
//***********************************
//
//									*
//***********************************
// Tux-Version
// 1: Erste Version. LCD auf

#define TESTSERVER   0

#define TUXVERSION   1

#define SDAPIN		4
#define SCLPIN		5

/*
 #define TASTE1		38
 #define TASTE2		46
 #define TASTE3		54
 #define TASTE4		72
 #define TASTE5		95
 #define TASTE6		115
 #define TASTE7		155
 #define TASTE8		186
 #define TASTE9		205
 #define TASTEL		225
 #define TASTE0		235
 #define TASTER		245
 */

#define	LOOPLEDPORT		PORTB
#define	LOOPLEDPORTPIN	DDRB
#define	LOOPLED			1
#define	TWIPIN			0



#define STARTDELAYBIT		0
#define HICOUNTBIT			1
#define CLIENTBIT          3
#define WDTBIT             7
#define TASTATURPIN			0           //	Eingang fuer Tastatur
#define THERMOMETERPIN		1           //	Eingang fuer Thermometer
#define RELAISPIN          5           //	Ausgang fuer Reset-Relais

#define MASTERCONTROLPIN	4           // Eingang fuer MasterControl: Meldung MasterReset

volatile uint8_t rxdata =0;
volatile uint16_t EventCounter=0;
static char baseurl[]="http://ruediheimlicherhome.dyndns.org/";



/* *************************************************************************  */
/* Eigene Deklarationen                                                       */
/* *************************************************************************  */
#define NULLTASK					0xB0	// Nichts tun
#define ERRTASK					0xA0	// F

#define STATUSTASK				0xB1	// Status des TWI aendern
#define STATUSCONFIRMTASK		0xB2	// Statusaenderung des TWI bestaetigen
#define EEPROMREADTASK			0xB8	// von EEPROM lesen
#define EEPROMSENDTASK			0xB9	// Daten vom HomeServer an HomeCentral senden
#define EEPROMRECEIVETASK		0xB6	// Adresse fuer EEPROM-Write empfangen
#define EEPROMWRITETASK			0xB7	// auf EEPROM schreiben
#define EEPROMCONFIRMTASK		0xB5	// Quittung an HomeCentral senden
#define EEPROMREPORTTASK		0xB4	// Daten vom EEPROM an HomeServer senden

#define EEPROMREADWOCHEATASK	0xBA
#define EEPROMREADWOCHEBTASK	0xBB
#define EEPROMREADPWMTASK     0xBC  // Daten fuer PWM-Array im EEPROM holen

#define RESETTASK					0xBF	// HomeCentral reseten

#define DATATASK					0xC0	// Normale Loop im Webserver
#define SOLARTASK					0xC1	// Daten von solar

#define MASTERERRTASK			0xC7	// Fehlermeldung vom Master senden



#define HOMECALLBACK       0
#define SOLARCALLBACK       1
#define ALARMCALLBACK       2
#define EXPCALLBACK        3
#define PINGCALLBACK       7


#define STATUSTIMEOUT			0x0080
//
// Eventuell kritische Werte
#define START_BYTE_DELAY		2				// Timerwert fuer Start-Byte
#define BYTE_DELAY				2				// Timerwert fuer Data-Byte

volatile uint16_t					timer2_counter=0;

//enum webtaskflag{IDLE, TWISTATUS,EEPROMREAD, EEPROMWRITE};

static uint8_t  webtaskflag =0;
//uint8_t  webspistatus =0;

static uint8_t monitoredhost[4] = {10,0,0,7};

//#define STR_BUFFER_SIZE 24
//static char strbuf_A[STR_BUFFER_SIZE+1];


#define TAGPLANBREITE		0x40	// 64 Bytes, 2 page im EEPROM
#define RAUMPLANBREITE		0x200	// 512 Bytes
#define twi_buffer_size		8
#define buffer_size			8
#define page_size				32
#define eeprom_buffer_size 8


volatile uint8_t	TWI_Pause=1;


volatile uint8_t StartDaten;


static volatile uint8_t Temperatur;
/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[twi_buffer_size];
volatile uint8_t txstartbuffer;

static char HeizungDataString[64];
static char SolarDataString[64];
static char EEPROM_String[96];

//static  char d[4]={};
//static char* key1;
//static char *sstr;

//char HeizungVarString[64];

static char AlarmDataString[64];

static char ErrDataString[32];


volatile uint8_t oldErrCounter=0;
volatile uint16_t datcounter=0;

volatile uint8_t callbackstatus=0;

/*
 uint8_t EEMEM EETitel;	//HomeCentral
 uint8_t EEMEM EEAdresse;
 uint8_t EEMEM EEVorlauf;
 uint8_t EEMEM EERuecklauf;
 uint8_t EEMEM EEAussen;
 uint8_t EEMEM EEInnen;
 */



static volatile uint8_t stepcounter=0;

// Prototypes
void lcdinit(void);
void r_itoa16(int16_t zahl, char* string);
void tempbis99(uint16_t temperatur,char*tempbuffer);


// the password string (only the first 5 char checked), (only a-z,0-9,_ characters):
static char password[10]="ideur00"; // must not be longer than 9 char
static char resetpassword[10]="ideur!00!"; // must not be longer than 9 char


uint8_t TastenStatus=0;
uint16_t Tastencount=0;
uint16_t Tastenprellen=0x01F;


static volatile uint8_t pingnow=1; // 1 means time has run out send a ping now
static volatile uint8_t resetnow=0;
static volatile uint8_t reinitmac=0;
//static uint8_t sendping=1; // 1 we send ping (and receive ping), 0 we receive ping only
static volatile uint8_t pingtimer=1; // > 0 means wd running
//static uint8_t pinginterval=30; // after how many seconds to send or receive a ping (value range: 2 - 250)
static char *errmsg; // error text

unsigned char TWI_Transceiver_Busy( void );
//static volatile uint8_t twibuffer[twi_buffer_size+1]; // Buffer fuer Data aus/fuer EEPROM
static volatile char twiadresse[4]; // EEPROM-Adresse
//static volatile uint8_t hbyte[4];
//static volatile uint8_t lbyte[4];
extern volatile uint8_t twiflag;
static uint8_t aktuelleDatenbreite=8;
static volatile uint8_t send_cmd=0;


void Timer0(void);
uint8_t WochentagLesen(unsigned char ADRESSE, uint8_t hByte, uint8_t lByte, uint8_t *Daten);
uint8_t SlavedatenLesen(const unsigned char ADRESSE, uint8_t *Daten);
void lcd_put_tempAbMinus20(uint16_t temperatur);

/* ************************************************************************ */
/* Ende Eigene Deklarationen																 */
/* ************************************************************************ */


// Note: This software implements a web server and a web browser.
// The web server is at "myip" and the browser tries to access "websrvip".
//
// Please modify the following lines. mac and ip have to be unique
// in your local area network. You can not have the same numbers in
// two devices:
//static uint8_t mymac[6] = {0x54,0x55,0x58,0x10,0x00,0x29};

//RH4701 52 48 34 37 30 31
static uint8_t mymac[6] = {0x52,0x48,0x34,0x37,0x30,0x32};

// how did I get the mac addr? Translate the first 3 numbers into ascii is: TUX
// This web server's own IP.
//static uint8_t myip[4] = {10,0,0,29};
//static uint8_t myip[4] = {192,168,255,100};

// IP des Webservers
static uint8_t myip[4] = {192,168,1,210};
static uint8_t mytestip[4] = {192,168,1,213};

// IP address of the web server to contact (IP of the first portion of the URL):
//static uint8_t websrvip[4] = {77,37,2,152};


// ruediheimlicher
// static uint8_t websrvip[4] = {193,17,85,42}; // ruediheimlicher 193.17.85.42 nine
//static uint8_t websrvip[4] = {213,188,35,156}; //   30.7.2014 msh
//static uint8_t websrvip[4] = {64,37,49,112}; //     64.37.49.112   28.02.2015 hostswiss // Pfade in .pl angepasst: cgi-bin neu in root dir
static uint8_t websrvip[4] = {217,26,52,16};//        217.26.52.16  24.03.2015 hostpoint

static uint8_t localwebsrvip[4] = {127,0,0,1};//        localhost


// The name of the virtual host which you want to contact at websrvip (hostname of the first portion of the URL):


#define WEBSERVER_VHOST "www.ruediheimlicher.ch"


// Default gateway. The ip address of your DSL router. It can be set to the same as
// websrvip the case where there is no default GW to access the
// web server (=web server is on the same lan as this host)

// ************************************************
// IP der Basisstation !!!!!
// Runde Basisstation :
//static uint8_t gwip[4] = {192,168,1,5};// Rueti

// Viereckige Basisstation:
static uint8_t gwip[4] = {192,168,1,1};// Rueti

// ************************************************

static char urlvarstr[21];
// listen port for tcp/www:
#define MYWWWPORT 80

#define MYTESTWWWPORT 1401//

#define BUFFER_SIZE 800


static uint8_t buf[BUFFER_SIZE+1];
static uint8_t pingsrcip[4];
static uint8_t start_web_client=0;
static uint8_t web_client_attempts=0;
static uint8_t web_client_sendok=0;
static volatile uint8_t sec=0;
static volatile uint8_t cnt2step=0;



#define tag_start_adresse 0

#define lab_data_size 8

void timer2 (uint8_t wert);


void str_cpy(char *ziel,char *quelle)
{
	uint8_t lz=strlen(ziel); //startpos fuer cat
	//printf("Quelle: %s Ziellaenge: %d\n",quelle,lz);
	uint8_t lq=strlen(quelle);
	//printf("Quelle: %s Quelllaenge: %d\n",quelle,lq);
	uint8_t i;
	for(i=0;i<lq;i++)
	{
		//printf("i: %d quelle[i]: %c\n",i,quelle[i]);
		ziel[i]=quelle[i];
	}
	lz=strlen(ziel);
	ziel[lz]='\0';
}

void str_cat(char *ziel,char *quelle)
{
	uint8_t lz=strlen(ziel); //startpos fuer cat
	//printf("Quelle: %s Ziellaenge: %d\n",quelle,lz);
	uint8_t lq=strlen(quelle);
	//printf("Quelle: %s Quelllaenge: %d\n",quelle,lq);
	uint8_t i;
	for(i=0;i<lq;i++)
	{
		//printf("i: %d quelle[i]: %c\n",i,quelle[i]);
		ziel[lz+i]=quelle[i];
		
	}
	//printf("ziel: %s\n",ziel);
	lz=strlen(ziel);
	ziel[lz]='\0';
}

// http://stackoverflow.com/questions/122616/how-do-i-trim-leading-trailing-whitespace-in-a-standard-way
char *trimwhitespace(char *str)
{
   char *end;
   
   // Trim leading space
   while(isspace(*str)) str++;
   
   if(*str == 0)  // All spaces?
      return str;
   
   // Trim trailing space
   end = str + strlen(str) - 1;
   while(end > str && isspace(*end)) end--;
   
   // Write new null terminator
   *(end+1) = 0;
   
   return str;
}


void Timer0()
{
	//----------------------------------------------------
	// Set up timer 0 to generate interrupts @ 1000Hz
	//----------------------------------------------------
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS00) | _BV(CS02);
	OCR0A = 0x2;
	TIMSK0 = _BV(OCIE0A);
	
}

ISR(TIMER0_COMPA_vect)
{
   
   TCNT0=0;
	if(EventCounter < 0x8FFF)// am Zaehlen, warten auf beenden von TWI // 0x1FF: 2 s
	{
		
	}
	else // Ueberlauf, TWI ist beendet,SPI einleiten
	{
		
		EventCounter =0;
		//lcd_gotoxy(0,0);
		//lcd_puthex(webspistatus);
		if (!(webspistatus & (1<<TWI_WAIT_BIT)))        // TWI soll laufen
		{
			webspistatus |= (1<< SPI_SHIFT_BIT);         // shift_out veranlassen
		}
      
		if (webspistatus & (1<<TWI_STOP_REQUEST_BIT))	// Gesetzt in cmd=2: Vorgang Status0 von HomeServer ist angemeldet
		{
			webspistatus |= (1<<SPI_STATUS0_BIT);			// STATUS 0 soll noch an Master gesendet werden.
			
			webspistatus &= ~(1<<TWI_STOP_REQUEST_BIT);	//Bit zuruecksetzen
		}
		
		
		if (webspistatus & (1<<SPI_STATUS0_BIT))
		{
			webspistatus |= (1<<TWI_WAIT_BIT);				// SPI/TWI soll in der naechsten schleifen nicht mehr ermoeglicht werden
         pendenzstatus |= (1<<SEND_STATUS0_BIT);		// Bestaetigung an Homeserver schicken, dass Status 0 angekommen ist. In cmd=10 zurueckgesetzt.
         
			webspistatus &= ~(1<<SPI_STATUS0_BIT);			//Bit zuruecksetzen
		}
		
		
	}
   
	EventCounter++;
   
}

uint16_t http200ok(void)
{
	return(fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n")));
}

void timer2 (uint8_t wert)
{
	PRR&=~(1<<PRTIM2); // write power reduction register to zero
	TIMSK2=(1<<OCIE2A); // compare match on OCR2A
	TCNT2=0;  // init counter
   
	TCCR2B |= (1<<CS20)|(1<<CS21)|(1<<CS22);	//Takt /1024	Intervall 32 us
   
	TCCR2A |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC
   
	TIFR2 |= (1<<TOV2); 				//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
   
	TCNT2 = 0x00;					//Zaehler zuruecksetzen
	
	OCR2A = wert;					//Setzen des Compare Registers auf Servoimpulsdauer
}

/* setup timer T2 as an interrupt generating time base.
 * You must call once sei() in the main program */
/*
 void init_cnt2(void)
 {
 cnt2step=0;
 PRR&=~(1<<PRTIM2); // write power reduction register to zero
 TIMSK2=(1<<OCIE2A); // compare match on OCR2A
 TCNT2=0;  // init counter
 OCR2A=244; // value to compare against
 TCCR2A=(1<<WGM21); // do not change any output pin, clear at compare match
 // divide clock by 1024: 12.5MHz/128=12207 Hz
 TCCR2B=(1<<CS22)|(1<<CS21)|(1<<CS20); // clock divider, start counter
 // 12207/244=50Hz
 }
 */

// called when TCNT2==OCR2A

/*
 ISR(TIMER2_COMPA_vect)
 {
 
 }
 */
// we were ping-ed by somebody, store the ip of the ping sender
// and trigger an upload to http://tuxgraphics.org/cgi-bin/upld
// This is just for testing and demonstration purpose
void ping_callback(uint8_t *ip)
{
   uint8_t i=0;
   // trigger only first time in case we get many ping in a row:
   if (start_web_client==0)
   {
      start_web_client=1;
      //			lcd_gotoxy(12,0);
      //			lcd_puts("ping\0");
      // save IP from where the ping came:
      while(i<4)
      {
         pingsrcip[i]=ip[i];
         i++;
      }
      
   }
}


void exp_browserresult_callback(uint8_t statuscode,uint16_t datapos)
{
   // datapos is not used in this example
   if (statuscode==0)
   {
      
      lcd_gotoxy(0,0);
      lcd_puts("        \0");
      lcd_gotoxy(0,0);
      lcd_puts("e cb OK\0");
      
      web_client_sendok++;
      //				sei();
      
   }
   else
   {
      lcd_gotoxy(0,0);
      lcd_puts("        \0");
      lcd_gotoxy(0,0);
      lcd_puts("e cb err\0");
      lcd_puthex(statuscode);
      
   }
}



void solar_browserresult_callback(uint8_t statuscode,uint16_t datapos)
{
   // datapos is not used in this example
   if (statuscode==0)
   {
      
      lcd_gotoxy(0,0);
      lcd_puts("        \0");
      lcd_gotoxy(0,0);
      lcd_puts("s cb OK\0");
      
      web_client_sendok++;
      //				sei();
      
   }
   else
   {
      lcd_gotoxy(0,0);
      lcd_puts("        \0");
      lcd_gotoxy(0,0);
      lcd_puts("s cb err\0");
      lcd_puthex(statuscode);
      
   }
}

void home_browserresult_callback(uint8_t statuscode,uint16_t datapos)
{
   // datapos is not used in this example
   if (statuscode==0)
   {
      
      lcd_gotoxy(0,0);
      lcd_puts("        \0");
      lcd_gotoxy(0,0);
      lcd_puts("h cb OK\0");
      
      web_client_sendok++;
      //				sei();
      
   }
   else
   {
      lcd_gotoxy(0,0);
      lcd_puts("        \0");
      lcd_gotoxy(0,0);
      lcd_puts("h cb err\0");
      lcd_puthex(statuscode);
      
   }
}



void alarm_browserresult_callback(uint8_t statuscode,uint16_t datapos)
{
   // datapos is not used in this example
   if (statuscode==0)
   {
      
      lcd_gotoxy(0,0);
      lcd_puts("        \0");
      lcd_gotoxy(0,0);
      lcd_puts("a cb OK\0");
      
      web_client_sendok++;
      //				sei();
      
   }
   else
   {
      lcd_gotoxy(0,0);
      lcd_puts("         \0");
      lcd_gotoxy(0,0);
      lcd_puts("a cb err\0");
      lcd_puthex(statuscode);
      
   }
}

/* ************************************************************************ */
/* Eigene Funktionen														*/
/* ************************************************************************ */

uint8_t verify_password(char *str)
{
	// the first characters of the received string are
	// a simple password/cookie:
	if (strncmp(password,str,7)==0)
   {
		return(1);                 // PW OK
	}
	return(0);                    //PW falsch
}


uint8_t verify_reset_password(char *str)
{
	// the first characters of the received string are
	// a simple password/cookie:
	if (strncmp(resetpassword,str,7)==0)
   {
		return(1); // Reset-PW OK
	}
	return(0); //Reset-PW falsch
}


void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

uint8_t Hex2Int(char *s)
{
   long res;
   char *Chars = "0123456789ABCDEF", *p;
   
   if (strlen(s) > 8)
   /* Error ... */ ;
   
   for (res = 0L; *s; s++) {
      if ((p = strchr(Chars, toupper(*s))) == NULL)
      /* Error ... */ ;
      res = (res << 4) + (p-Chars);
   }
   
   return res;
}

void tempbis99(uint16_t temperatur,char*tempbuffer)
{
	char buffer[8]={};
	//uint16_t temp=(temperatur-127)*5;
	uint16_t temp=temperatur*5;
	
	//itoa(temp, buffer,10);
	
	r_itoa16(temp,buffer);
	
	//lcd_puts(buffer);
	//lcd_putc('*');
	
	//char outstring[7]={};
	
	tempbuffer[6]='\0';
	tempbuffer[5]=' ';
	tempbuffer[4]=buffer[6];
	tempbuffer[3]='.';
	tempbuffer[2]=buffer[5];
	if (abs(temp)<100)
	{
		tempbuffer[1]=' ';
		
	}
	else
	{
		tempbuffer[1]=buffer[4];
		
	}
	tempbuffer[0]=buffer[0];
	
	
}

void tempAbMinus20(uint16_t temperatur,char*tempbuffer)
{
   
   char buffer[8]={};
   int16_t temp=(temperatur)*5;
   temp -=200;
   char Vorzeichen=' ';
   if (temp < 0)
   {
      Vorzeichen='-';
   }
   
   r_itoa16(temp,buffer);
   //		lcd_puts(buffer);
   //		lcd_putc(' * ');
   
   //		char outstring[7]={};
   
   tempbuffer[6]='\0';
   //outstring[5]=0xDF; // Grad-Zeichen
   tempbuffer[5]=' ';
   tempbuffer[4]=buffer[6];
   tempbuffer[3]='.';
   tempbuffer[2]=buffer[5];
   if (abs(temp)<100)
   {
		tempbuffer[1]=Vorzeichen;
		tempbuffer[0]=' ';
   }
   else
   {
		tempbuffer[1]=buffer[4];
		tempbuffer[0]=Vorzeichen;
   }
   //		lcd_puts(outstring);
}


// search for a string of the form key=value in
// a string that looks like q?xyz=abc&uvw=defgh HTTP/1.1\r\n
//
// The returned value is stored in the global var strbuf_A

// Andere Version in Webserver help funktions
/*
 uint8_t find_key_val(char *str,char *key)
 {
 uint8_t found=0;
 uint8_t i=0;
 char *kp;
 kp=key;
 while(*str &&  *str!=' ' && found==0){
 if (*str == *kp)
 {
 kp++;
 if (*kp == '\0')
 {
 str++;
 kp=key;
 if (*str == '=')
 {
 found=1;
 }
 }
 }else
 {
 kp=key;
 }
 str++;
 }
 if (found==1){
 // copy the value to a buffer and terminate it with '\0'
 while(*str &&  *str!=' ' && *str!='&' && i<STR_BUFFER_SIZE)
 {
 strbuf_A[i]=*str;
 i++;
 str++;
 }
 strbuf_A[i]='\0';
 }
 // return the length of the value
 return(i);
 }
 */


#pragma mark analye_get_url

// takes a string of the form ack?pw=xxx&rst=1 and analyse it
// return values:  0 error
//                 1 resetpage and password OK
//                 4 stop wd
//                 5 start wd
//                 2 /mod page
//                 3 /now page
//                 6 /cnf page

uint8_t analyse_get_url(char *str)	// codesnippet von Watchdog
{
	char actionbuf[32];
	errmsg="inv.pw";
	
   webtaskflag =0; //webtaskflag zuruecksetzen. webtaskflag bestimmt aktion, die ausgefuehrt werden soll. Wird an Master weitergegeben.
	// ack
	
	// str: ../ack?pw=ideur00&rst=1
	if (strncmp("ack",str,3)==0)
	{
		lcd_clr_line(1);
		lcd_gotoxy(0,1);
		lcd_puts("ack\0");
      
		// Wert des Passwortes eruieren
		if (find_key_val(str,actionbuf,10,"pw"))
		{
			urldecode(actionbuf);
			
			// Reset-PW?
			if (verify_reset_password(actionbuf))
			{
				return 15;
			}
         
			// Passwort kontrollieren
			if (verify_password(actionbuf))
			{
				if (find_key_val(str,actionbuf,10,"tst"))
				{
					return(1);
				}
			}
		}
      
		return(0);
		
	}//ack
	
	if (strncmp("twi",str,3)==0)										//	Daten von HC beginnen mit "twi"
	{
		//lcd_clr_line(1);
		//lcd_gotoxy(17,0);
		//lcd_puts("twi\0");
		
		// Wert des Passwortes eruieren
		if (find_key_val(str,actionbuf,10,"pw"))					//	Passwort kommt an zweiter Stelle
		{
			urldecode(actionbuf);
			webtaskflag=0;
			//lcd_puts(actionbuf);
			// Passwort kontrollieren
			
			
			if (verify_password(actionbuf))							// Passwort ist OK
			{
				//OSZILO;
				if (find_key_val(str,actionbuf,10,"status"))		// Status soll umgeschaltet werden
				{
					
					webtaskflag =STATUSTASK;							// Task setzen
               
					out_startdaten=STATUSTASK; // B1
               
					//lcd_gotoxy(6,0);
					//lcd_puts(actionbuf);
					outbuffer[0]=atoi(actionbuf);
					out_lbdaten=0x00;
					out_lbdaten=0x00;
               
					if (actionbuf[0]=='0') // twi aus
					{
                  
						//WebTxDaten[1]='0';
						outbuffer[1]=0;
						out_hbdaten=0x00;
						out_lbdaten=0x00;
						return (2);
					}
					if (actionbuf[0]=='1') // twi ein
					{
                  
						//WebTxDaten[1]='0';
						outbuffer[1]=1;
						out_hbdaten=0x01;
						out_lbdaten=0x00;
						return (3);				// Status da, sendet Bestaetigung an Homeserver
					}
               
				}//st
				
				
				if (find_key_val(str,actionbuf,10,"radr"))		// EEPROM lesen. read-Request fuer EEPROM-Adresse wurde gesendet
				{
					// Nur zweite Stelle der EEPROM-Adresse, default 0xA0 wird im Master zugefuegt
					//WebTxDaten[0]=atoi(actionbuf);
					//lcd_clr_line(3);
					//lcd_gotoxy(19,3);
					//lcd_puts(actionbuf);
					outbuffer[0]=atoi(actionbuf);
					
					webtaskflag = EEPROMREADTASK;						// Task setzen
					//WebTxStartDaten=webtaskflag;
					out_startdaten=EEPROMREADTASK;					// B8 Task an HomeCentral senden
					
					if (find_key_val(str,actionbuf,10,"hb"))		//hbyte ist da, an HomeCentral senden
					{
						//lcd_gotoxy(14,3);
						//lcd_puts(actionbuf);
                  
						//strcpy((char*)hbyte,actionbuf);
						out_hbdaten=atoi(actionbuf);
						//lcd_gotoxy(14,3);
						//lcd_puthex(out_hbdaten);
						
					}
					
					if (find_key_val(str,actionbuf,10,"lb"))		//lbyte ist da, an HomeCentral senden
					{
						//lcd_puts(actionbuf);
						//strcpy((char*)lbyte,actionbuf);
						out_lbdaten=atoi(actionbuf);
						//lcd_puthex(out_lbdaten);
                  
					}
					
					return (6);					// EEPROM read, sendet Bestaetigung an Homeserver
				}
				
				
				
				if (find_key_val(str,actionbuf,10,"rdata"))		// Homeserver hat geantwortet,
				{
					//lcd_clr_line(1);// EEPROM-Daten sollen von HomeCentral an HomeServer gesendet werden
					lcd_gotoxy(18,1);
					lcd_putc('$');
					//	lcd_puts(actionbuf);
					
					webtaskflag = EEPROMSENDTASK; // B9
					//WebTxStartDaten=webtaskflag;
               //		out_startdaten=EEPROMSENDTASK;
					aktuelleDatenbreite = eeprom_buffer_size;
					
					/*
                if (find_key_val(str,actionbuf,10,"data")) //datenstring
                {
                strcpy((char*)hbyte,actionbuf);
                //WebTxDaten[1]=atoi(actionbuf);
                outbuffer[1]=atoi(actionbuf);
                }
                */
               //lcd_gotoxy(10,1);
               //lcd_puts("rdata\0");
               //lcd_putint2(atoi(actionbuf));
					
					//if (atoi(actionbuf)==1)
					
					// Wert von rdata wird dekrementiert
					if (atoi(actionbuf))
                  
					{
                  //lcd_gotoxy(19,1);
                  //lcd_putc('1');
                  
						return (8);// Daten von HomeCentral anfordern, senden
                  
					}
					else
					{
						webspistatus &= ~(1<<SPI_DATA_READY_BIT);
						
					}
               
				}
				
				
				
				// Daten fuer EEPROM von Homeserver empfangen
				
				if (find_key_val(str,actionbuf,10,"wadr"))			// EEPROM-Daten werden von Homeserver gesendet
				{
					//webspistatus |= (1<< TWI_WAIT_BIT);				// Daten nicht an HomeCentral senden
               
					// Nur erste Stelle der EEPROM-Adresse, default 0xA0 wird im Master zugefuegt
               
					outbuffer[0]=atoi(actionbuf);
					
					//				lcd_gotoxy(17,1);
					//				lcd_puthex(txbuffer[0]);
					//				lcd_gotoxy(17,2);
					//				lcd_puts(actionbuf);
					
					uint8_t dataOK=0;
					webtaskflag = EEPROMRECEIVETASK; // B6
               
					//out_startdaten=EEPROMRECEIVETASK;
					
					aktuelleDatenbreite = buffer_size;
					
					if (find_key_val(str,actionbuf,10,"hbyte"))		//hbyte der Adresse
					{
						dataOK ++;
						//strcpy((char*)hbyte,actionbuf);
						//outbuffer[1]=atoi(actionbuf);
						out_hbdaten=atoi(actionbuf);
					}
					
					if (find_key_val(str,actionbuf,10,"lbyte"))		// lbyte der Adresse
					{
						dataOK ++;
						//strcpy((char*)lbyte,actionbuf);
						//outbuffer[2]=atoi(actionbuf);
						out_lbdaten=atoi(actionbuf);
					}
					
					if (find_key_val(str,actionbuf,28,"data"))		// Datenstring mit '+' - Trennzeichen
					{
						//lcd_gotoxy(0,0);
						//lcd_puthex(strlen(actionbuf));
						//lcd_putc(' ');
						//lcd_puts(actionbuf);
						//lcd_puts("    \0");
						//lcd_gotoxy(14,1);
						//lcd_putc('A');
						
						webtaskflag = EEPROMWRITETASK;					// Task setzen
						//EEPROMTxStartDaten=webtaskflag;
						
						out_startdaten=EEPROMWRITETASK; // B7
						
						// Test
						/*
                   EEPROMTxDaten[1]=1;
                   EEPROMTxDaten[2]=2;
                   EEPROMTxDaten[3]=3;
                   */
						//lcd_putc('B');
						
						dataOK ++;
						char* buffer= malloc(32);
						//lcd_putc('C');
						
						strcpy(buffer, actionbuf);
						
						//lcd_putc('D');
						
						uint8_t index=0;
						char* linePtr = malloc(32);
						
						linePtr = strtok(buffer,"+");
						
						while (linePtr !=NULL)								// Datenstring: Bei '+' trennen
						{
							//EEPROMTxDaten[index++] = strtol(linePtr,NULL,16); //http://www.mkssoftware.com/docs/man3/strtol.3.asp
							outbuffer[index++] = strtol(linePtr,NULL,16); //http://www.mkssoftware.com/docs/man3/strtol.3.asp
							linePtr = strtok(NULL,"+");
						}
						free(linePtr);
						free(buffer);
					} // if data
					
               //				if (dataOK==2) // alle Daten da
					{
						
						return (9);												// Empfang bestätigen
					}
				} // wadr
				
				if (find_key_val(str,actionbuf,10,"iswriteok"))		// Anfrage ob writeok
				{
               
					return (7);
				}
            
				if (find_key_val(str,actionbuf,12,"isstat0ok"))		// Anfrage ob statusok isstat0ok
				{
					lcd_gotoxy(7,0);
					lcd_putc('*');
					return (10);
				}
				
            if (find_key_val(str,actionbuf,10,"reset")) // HomeCentral reseten
            {
               
            }
            
            if (find_key_val(str,actionbuf,10,"servo")) // Master soll Daten fuer PCM-Array im Master holen
            {
               
            }
            
            
            
				
			}//verify pw
		}//find_key pw
		return(0);
	}//twi
   
	
   
	errmsg="inv.url";
	return(0);
}




uint16_t print_webpage_send_EEPROM_Data(uint8_t *buf,  uint8_t* data)
{
	// EEPTROM-Daten von hb, lb senden
	uint16_t plen;
	
	plen=fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n"));
   
	/*
    // hByte einsetzen
    plen=fill_tcp_data_p(buf,plen,PSTR("<p>hb="));
    //itoa((int)hbyte,TWIString,16);
    plen=fill_tcp_data(buf,plen,(char*)hbyte);
    
    // lbyte einsetzen
    plen=fill_tcp_data_p(buf,plen,PSTR("<p>lb="));
    //itoa((int)lbyte,TWIString,16);
    plen=fill_tcp_data(buf,plen,(char*)lbyte);
    */
	
	
	
	// Datenstring einsetzen
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>data="));
	//plen=fill_tcp_data(buf,plen,"hallo\0");
   
	plen=fill_tcp_data(buf,plen,(void*)data);
   
	return plen;
	
}

uint16_t print_webpage_ok(uint8_t *buf,uint8_t *okcode)
{
	// Schickt den okcode als Bestaetigung fuer den Empfang des Requests
	uint16_t plen;
	plen=fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n"));
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>okcode="));
	plen=fill_tcp_data(buf,plen,(void*)okcode);
	return plen;
}



uint16_t print_webpage_confirm(uint8_t *buf)
{
	uint16_t plen;
	plen=fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n"));
	plen=fill_tcp_data_p(buf,plen,PSTR("<h2>Bearbeiten</h2><p>Passwort OK.</p>\n"));
	
	//
	/*
    plen=fill_tcp_data_p(buf,plen,PSTR("<form action=/ram method=get>"));
    plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=gto value=0 checked> Heizung <br></p>\n"));
    plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=gto value=1> Werkstatt<br></p>\n"));
    plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=gto value=3> Buero<br></p>\n"));
    plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=gto value=4> Labor<br></p><input type=submit value=\"Gehen\"></form>\n"));
    */
	
	plen=fill_tcp_data_p(buf,plen,PSTR("<form action=/cde method=get>"));										// Code fuer Tagbalken eingeben
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>Raum: <input type=text name=raum size=2><br>"));				// Raum
	plen=fill_tcp_data_p(buf,plen,PSTR("Wochentag: <input type=text name=wochentag  size=2><br>"));		// Wochentag
	plen=fill_tcp_data_p(buf,plen,PSTR("Objekt: <input type=text name=objekt  size=2><br>"));				// Objekt
	plen=fill_tcp_data_p(buf,plen,PSTR("Stunde: <input type=text name=stunde  size=2><br>"));				// Stunde
	plen=fill_tcp_data_p(buf,plen,PSTR("<input type=submit value=\"Lesen\"></p>"));
	
	/*
    plen=fill_tcp_data_p(buf,plen,PSTR("<p><input type=radio name=std value=0 checked> OFF <br>"));
    plen=fill_tcp_data_p(buf,plen,PSTR("<input type=radio name=std value=2> erste halbe Stunde<br>"));
    plen=fill_tcp_data_p(buf,plen,PSTR("<input type=radio name=std value=1> zweite halbe Stunde<br>"));
    plen=fill_tcp_data_p(buf,plen,PSTR("<input type=radio name=std value=3> ganze Stunde<br></p>"));
    */
   
	plen=fill_tcp_data_p(buf,plen,PSTR("Stunde: <input type=text name=code  size=2><br>"));				// code
   
	plen=fill_tcp_data_p(buf,plen,PSTR("<input type=submit value=\"Setzen\"></form>"));
   
	
	
	
	plen=fill_tcp_data_p(buf,plen,PSTR("<form action=/twi method=get>"));
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=hidden name=pw value=\"ideur00\"></p>"));
	if (PIND & (1<<TWIPIN)) // TWI ist ON
	{
		plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=status value=0> OFF<br></p>\n")); // st: Statusflag
		plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=status value=1 checked> ON <br></p>\n"));
		
	}
	else
	{
		plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=status value=0 checked> OFF<br></p>\n"));
		plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=status value=1> ON <br></p>\n"));
		
	}
	//	plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=hidden name=pw value=\"ideur00\"><input type=radio name=st value=0> OFF<br></p>\n"));
	//	plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=st value=1 checked> ON <br></p>\n"));
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=submit value=\"OK\"></form>\n"));
	
	//
	plen=fill_tcp_data_p(buf,plen,PSTR("<a href=\"/\">&lt;&lt;zur&uuml;ck zu Status</a></p>\n"));
	
	
	return(plen);
}

#pragma mark Webpage_status

// prepare the webpage by writing the data to the tcp send buffer
uint16_t print_webpage_status(uint8_t *buf)
{
	
	uint16_t plen=0;
	//char vstr[5];
	plen=http200ok();
	
   if (TESTSERVER)
   {
      plen=fill_tcp_data_p(buf,plen,PSTR("<h1>HomeCentral Test</h1>"));
   }
   else
   {
      plen=fill_tcp_data_p(buf,plen,PSTR("<h1>HomeCentral</h1>"));
   }
   //
	
   
   
   char	TemperaturString[7];
	
	//
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>  HomeCentral<br>  Falkenstrasse 20<br>  8630 Rueti"));
	plen=fill_tcp_data_p(buf,plen,PSTR("<hr><h4><font color=\"#00FF00\">Status</h4></font></p>"));
	
	
	//return(plen);
	
	
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>	Vorlauf: "));
	Temperatur=0;
	
	//Temperatur=WebRxDaten[2];
	Temperatur=inbuffer[2]; // Vorlauf
	//Temperatur=Vorlauf;
	tempbis99(Temperatur,TemperaturString);
	
	//r_itoa(Temperatur,TemperaturStringV);
	plen=fill_tcp_data(buf,plen,TemperaturString);
	plen=fill_tcp_data_p(buf,plen,PSTR("<br> Ruecklauf: "));
	Temperatur=0;
	//Temperatur=WebRxDaten[3];
	Temperatur=inbuffer[3]; // Ruecklauf
	tempbis99(Temperatur,TemperaturString);
	plen=fill_tcp_data(buf,plen,TemperaturString);
	
	plen=fill_tcp_data_p(buf,plen,PSTR("<br> Aussen: "));
	
	//char	AussenTemperaturString[7];
	Temperatur=0;
	//Temperatur=WebRxDaten[4];
	Temperatur=inbuffer[4];
	//lcd_gotoxy(10,1);
	//lcd_putc(' ');
	//lcd_puthex(Temperatur);
	
	//tempbis99(Temperatur,TemperaturString);
	tempAbMinus20(Temperatur,TemperaturString);
	//lcd_putc(' ');
	//lcd_puts(TemperaturString);
	plen=fill_tcp_data(buf,plen,TemperaturString);
	
   //	return(plen);
	//
	//initADC(THERMOMETERPIN);
	uint16_t tempBuffer=0;
   //	tempBuffer = readKanal(THERMOMETERPIN);
	plen=fill_tcp_data_p(buf,plen,PSTR("<br> Innen: "));
	//Temperatur= (tempBuffer >>2);
	//Temperatur *= 1.11;
	//Temperatur=WebRxDaten[5];
	//Temperatur=WebRxDaten[7];
	Temperatur=inbuffer[7];
	
	//Temperatur -=40;
	//lcd_gotoxy(0,1);
	//lcd_putc(' ');
	//lcd_puthex(Temperatur);
	
	tempbis99(Temperatur,TemperaturString);
	//lcd_putc(' ');
	//lcd_puts(TemperaturString);
	plen=fill_tcp_data(buf,plen,TemperaturString);
	//closeADC();
	
	plen=fill_tcp_data_p(buf,plen,PSTR("<br>TWI: "));
   
	plen=fill_tcp_data_p(buf,plen,PSTR("<br>Status: "));
	uint8_t		Status=0;
	char		StatusString[7]={};
	
	itoa((int)inbuffer[5],StatusString,10);
	
	
	plen=fill_tcp_data(buf,plen,StatusString);
	
	//r_itoa16((uint16_t)WebRxDaten[4],StatusString);
	
	plen=fill_tcp_data_p(buf,plen,PSTR("<br>Brenner: "));
	//Status=WebRxDaten[5];
	Status=inbuffer[5];
	Status &= 0x04; // Bit 2	0 wenn ON
	if (Status)
	{
		plen=fill_tcp_data_p(buf,plen,PSTR(" OFF"));
	}
	else
	{
		plen=fill_tcp_data_p(buf,plen,PSTR(" ON "));
	}
	
   
   return(plen);
   
	// Taste und Eingabe fuer Passwort
	plen=fill_tcp_data_p(buf,plen,PSTR("<form action=/ack method=get>"));
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>\nPasswort: <input type=password size=10 name=pw ><input type=hidden name=tst value=1>  <input type=submit value=\"Bearbeiten\"></p></form>"));
	
	plen=fill_tcp_data_p(buf,plen,PSTR("<p><hr>"));
	plen=fill_tcp_data(buf,plen,DATUM);
	plen=fill_tcp_data_p(buf,plen,PSTR("  Ruedi Heimlicher"));
	plen=fill_tcp_data_p(buf,plen,PSTR("<br>Version :"));
	plen=fill_tcp_data(buf,plen,VERSION);
	plen=fill_tcp_data_p(buf,plen,PSTR("\n<hr></p>"));
	
	//
	
	/*
	 // Tux
	 plen=fill_tcp_data_p(buf,plen,PSTR("<h2>web client status</h2>\n<pre>\n"));
	 
	 char teststring[24];
	 strcpy(teststring,"Data");
	 strcat(teststring,"-\0");
	 strcat(teststring,"Uploads \0");
	 strcat(teststring,"mit \0");
	 strcat(teststring,"ping : \0");
	 plen=fill_tcp_data(buf,plen,teststring);
	 
	 plen=fill_tcp_data_p(buf,plen,PSTR("Data-Uploads mit ping: "));
	 // convert number to string:
	 itoa(web_client_attempts,vstr,10);
	 plen=fill_tcp_data(buf,plen,vstr);
	 plen=fill_tcp_data_p(buf,plen,PSTR("\nData-Uploads aufs Web: "));
	 // convert number to string:
	 itoa(web_client_sendok,vstr,10);
	 plen=fill_tcp_data(buf,plen,vstr);
	 plen=fill_tcp_data_p(buf,plen,PSTR("\n</pre><br><hr>"));
	 */
	return(plen);
}

void master_init(void)
{
	
	DDRB |= (1<<PORTB1);	//Bit 1 von PORT B als Ausgang für Kontroll-LED
	PORTB |= (1<<PORTB1);	//Pull-up
	DDRB |= (1<<PORTB0);	//Bit 1 von PORT B als Ausgang für Kontroll-LED
	PORTB |= (1<<PORTB0);	//Pull-up
	
   //	DDRD |=(1<<RELAISPIN); //Pin 5 von Port D als Ausgang fuer Reset-Relais
   //	PORTD |=(1<<RELAISPIN); //HI
	// Eventuell: PORTD5 verwenden, Relais auf Platine
	
   
	DDRD &= ~(1<<MASTERCONTROLPIN); // Pin 4 von PORT D als Eingang fuer MasterControl
	PORTD |= (1<<MASTERCONTROLPIN);	// HI
	DDRD = 0xFF;
	pendenzstatus=0;
	
}

void initOSZI(void)
{
	OSZIPORTDDR |= (1<<PULS);
	OSZIPORT |= (1<<PULS);
}

void lcdinit()
{
	//*********************************************************************
	//	Definitionen LCD im Hauptprogramm
	//	Definitionen in lcd.h
	//*********************************************************************
	
	/*
    DDRC |= (1<<DDC3); //PIN 5 von PORT D als Ausgang fuer LCD_RSDS_PIN
    DDRC |= (1<<DDC4); //PIN 6 von PORT D als Ausgang fuer LCD_ENABLE_PIN
    DDRC |= (1<<DDC5); //PIN 7 von PORT D als Ausgang fuer LCD_CLOCK_PIN
    */
   LCD_DDR |= (1<<LCD_RSDS_PIN); //PIN 5 von PORT D als Ausgang fuer LCD_RSDS_PIN
   LCD_DDR |= (1<<LCD_ENABLE_PIN); //PIN 6 von PORT D als Ausgang fuer LCD_ENABLE_PIN
   LCD_DDR |= (1<<LCD_CLOCK_PIN); //PIN 7 von PORT D als Ausgang fuer LCD_CLOCK_PIN
   
   LCD_PORT |= (1<<LCD_RSDS_PIN); //PIN 5 von PORT D als Ausgang fuer LCD_RSDS_PIN
	LCD_PORT |= (1<<LCD_ENABLE_PIN); //PIN 6 von PORT D als Ausgang fuer LCD_RSDS_PIN
	LCD_PORT |= (1<<LCD_CLOCK_PIN); //PIN 7 von PORT D als Ausgang fuer LCD_CLOCK_PIN
   
   
   
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	lcd_puts("LCD Init\0");
	delay_ms(300);
	lcd_cls();
	
}

void setTWI_Status(uint8_t status)
{
   // Status wird in webpage_confirm abgefragt
	if (status)
	{
		//lcd_puts("ON");
		// TWI-Bit  einschalten
		PORTB |= (1<<TWIPIN);
		lcd_gotoxy(0,0);
		lcd_puts("TWI      \0");
      
	}
	else
	{
		//lcd_puts("OFF");
		
		PORTB &= ~(1<<TWIPIN); // TWI-Bit  ausschalten
		lcd_gotoxy(0,0);
		lcd_puts("SPI       \0");
      
	}
   
}

/****************************************************************************
 Call this function to test if the TWI_ISR is busy transmitting.
 ****************************************************************************/
unsigned char TWI_Transceiver_Busy( void )
{
   return ( TWCR & (1<<TWIE) );                  // IF TWI Interrupt is enabled then the Transceiver is busy
}

/****************************************************************************
 Auslesen der Daten vom EEPROM.
 ****************************************************************************/





void WDT_off(void)
{
   cli();
   wdt_reset();
   /* Clear WDRF in MCUSR */
   MCUSR &= ~(1<<WDRF);
   /* Write logical one to WDCE and WDE */
   /* Keep old prescaler setting to prevent unintentional time-out
    */
   WDTCSR |= (1<<WDCE) | (1<<WDE);
   /* Turn off WDT */
   WDTCSR = 0x00;
   sei();
}

uint8_t i=0;



/* ************************************************************************ */
/* Ende Eigene Funktionen														*/
/* ************************************************************************ */

void SPI_shift_out(void)
{
	//OSZILO;
	uint8_t byteindex=0;
	in_startdaten=0;
	in_enddaten=0;
	
	in_lbdaten=0;
	in_hbdaten=0;
	
	
	CS_HC_ACTIVE; // CS LO fuer Slave: Beginn Uebertragung
	//delay_ms(1);
	_delay_us(20);
	//OSZILO;
	in_startdaten=SPI_shift_out_byte(out_startdaten);
	//OSZIHI;
	_delay_us(out_PULSE_DELAY);
	in_lbdaten=SPI_shift_out_byte(out_lbdaten);
	
	_delay_us(out_PULSE_DELAY);
	in_hbdaten=SPI_shift_out_byte(out_hbdaten);
	
	_delay_us(out_PULSE_DELAY);
	for (byteindex=0;byteindex<out_BUFSIZE;byteindex++)
	{
		_delay_us(out_PULSE_DELAY);
		inbuffer[byteindex]=SPI_shift_out_byte(outbuffer[byteindex]);
		//
	}
	_delay_us(out_PULSE_DELAY);
	
	// Enddaten schicken: Zweiercomplement von in-Startdaten
	uint8_t complement = ~in_startdaten;
	in_enddaten=SPI_shift_out_byte(complement);
	
	_delay_us(100);
	CS_HC_PASSIVE; // CS HI fuer Slave: Uebertragung abgeschlossen
	
	lcd_gotoxy(19,1);
	
	if (out_startdaten + in_enddaten==0xFF)
	{
		lcd_putc('+');
		
	}
	else
	{
		lcd_putc('-');
		errCounter++;
		SPI_ErrCounter++;
	}
	
	
	//	lcd_gotoxy(17,3);
	//	lcd_puthex(errCounter & 0x00FF);
	//OSZIHI;
}

#pragma mark main
int main(void)
{
	
	/* ************************************************************************ */
	/* Eigene Main														*/
   
   /*
    lfuses sind 60 !!
    */

	/* ************************************************************************ */
	//JTAG deaktivieren (datasheet 231)
   //	MCUCSR |=(1<<7);
   //	MCUCSR |=(1<<7);
   
	MCUSR = 0;
	wdt_disable();
	Temperatur=0;
   
   if (TESTSERVER)
   {
   }
   

   
   
	//SLAVE
	//uint16_t Tastenprellen=0x0fff;
	uint16_t loopcount0=0;
	uint16_t loopcount1=0;
	//	Zaehler fuer Wartezeit nach dem Start
	//uint16_t startdelay0=0x001F;
	//uint16_t startdelay1=0;
	
	//Zaehler fuer Zeit von (SDA || SCL = LO)
	//uint16_t twi_LO_count0=0;
	//uint16_t twi_LO_count1=0;
	
	//Zaehler fuer Zeit von (SDA && SCL = HI)
	//uint16_t twi_HI_count0=0;
   
	/*
	 eepromWDT_Count0: Zaehler der wdt-Resets mit restart.
	 
	 Neu:
	 Wenn ein wdt-Reset abläuft, wird Bit 7 in eepromWDT_Count0 gesetzt.
	 Dadurch wartet der Prozessor mit dem Initialisieren des TWI-Slave, bis eine neue Startbedingung erscheint.
	 Anschliessend wird das Bit 7 wieder zurückgesetzt.
	 
	 alt:
	 eepromWDT_Count1: Zaehler fuer neuen wdt-Reset. Wenn wdt anspricht, wird der Zaheler erhoeht.
	 Beim Restart wird bei anhaltendem LO auf SDA oder SCL gewartet.
	 Wenn SCL und SDA beide HI sind, wird der Zaehler auf den Wert von eepromWDT_Count0 gesetzt
	 und der TWI-Slave gestartet.
	 
	 */
	//uint8_t StartStatus=0x00; //	Status des Slave
   // ETH
	//uint16_t plen;
	uint8_t i=0;
	int8_t cmd;
	
	
	// set the clock speed to "no pre-scaler" (8MHz with internal osc or
	// full external speed)
	// set the clock prescaler. First write CLKPCE to enable setting of clock the
	// next four instructions.
	CLKPR=(1<<CLKPCE); // change enable
	CLKPR=0; // "no pre-scaler"
	delay_ms(1);
	
	/* enable PD2/INT0, as input */
	
	//DDRD&= ~(1<<DDD2);				// INT0 als Eingang
	
	/*initialize enc28j60*/
	enc28j60Init(mymac);
	enc28j60clkout(2);				// change clkout from 6.25MHz to 12.5MHz
	delay_ms(1);
	
	
	/* Magjack leds configuration, see enc28j60 datasheet, page 11 */
	// LEDB=yellow LEDA=green
	//
	// 0x476 is PHLCON LEDA=links status, LEDB=receive/transmit
	// enc28j60PhyWrite(PHLCON,0b0000 0100 0111 01 10);
	enc28j60PhyWrite(PHLCON,0x476);
	delay_ms(20);
	
	i=1;
	//	WDT_off();
	//init the ethernet/ip layer:
   //	init_ip_arp_udp_tcp(mymac,myip,MYWWWPORT);
	// timer_init();
	
	//     sei(); // interrupt enable
	master_init();
	delay_ms(20);
   
	lcdinit();
   
   delay_ms(20);
	lcd_puts("Guten Tag \0");
	lcd_gotoxy(13,0);
	lcd_puts("V:\0");
	lcd_puts(VERSION);
   
	delay_ms(1600);
	//lcd_cls();
	
	TWBR =0;
   
	
	for (i=0;i< DATENBREITE;i++)
	{
      //		WebRxDaten[i]='A'+i;
      //		WebTxDaten[i] = i;
	}
	
	txstartbuffer = 0x00;
	uint8_t sendWebCount=0;	// Zahler fuer Anzahl TWI-Events, nach denen Daten des Clients gesendet werden sollen
	webspistatus=0;
	
	Init_SPI_Master();
	initOSZI();
	/* ************************************************************************ */
	/* Ende Eigene Main														*/
	/* ************************************************************************ */
	
	
	uint16_t dat_p;
	char str[30];
	
	// set the clock speed to "no pre-scaler" (8MHz with internal osc or
	// full external speed)
	// set the clock prescaler. First write CLKPCE to enable setting of clock the
	// next four instructions.
	CLKPR=(1<<CLKPCE); // change enable
	CLKPR=0; // "no pre-scaler"
	_delay_loop_1(0); // 60us
	
	/*initialize enc28j60*/
	enc28j60Init(mymac);
	enc28j60clkout(2); // change clkout from 6.25MHz to 12.5MHz
	_delay_loop_1(0); // 60us
	
	sei();
	
	/* Magjack leds configuration, see enc28j60 datasheet, page 11 */
	// LEDB=yellow LEDA=green
	//
	// 0x476 is PHLCON LEDA=links status, LEDB=receive/transmit
	// enc28j60PhyWrite(PHLCON,0b0000 0100 0111 01 10);
	enc28j60PhyWrite(PHLCON,0x476);
	
	//DDRB|= (1<<DDB1); // LED, enable PB1, LED as output
	//PORTD &=~(1<<PD0);;
	
	//init the web server ethernet/ip layer:
   if (TESTSERVER)
   {
      mymac[5] = 0x13;
      
      myip[3] = 213;

      init_ip_arp_udp_tcp(mymac,myip,MYTESTWWWPORT);
      // init the web client:
      client_set_gwip(gwip);  // e.g internal IP of dsl router
      
      client_set_wwwip(localwebsrvip);
     
   }
   else
   {
      init_ip_arp_udp_tcp(mymac,myip,MYWWWPORT);
      // init the web client:
      client_set_gwip(gwip);  // e.g internal IP of dsl router
   
      client_set_wwwip(websrvip);
   }
	register_ping_rec_callback(&ping_callback);
	setTWI_Status(1);
	
   //		SPI
   for (i=0;i<out_BUFSIZE;i++)
	{
      //outbuffer[i]='-';
      outbuffer[i]= 0;
	}
   
   //			end SPI
   
   
	Timer0();
   //
   lcd_clr_line(1);
   OSZIHI;
   //DDRD = 0xFF;
#pragma  mark "while"
	while(1)
	{
		sei();
		//Blinkanzeige
		
      
		if (PIND & (1<<MASTERCONTROLPIN))               //  Defaultposition: Pin ist HI
		{
			// Ende des Reset?
			if (pendenzstatus & (1<<RESETDELAY_BIT))     //  Bit ist noch gesetzt, Reset war am laufen
			{
				pendenzstatus &= ~(1<<RESETDELAY_BIT);    //  Bit zuruecksetzen,
            //Init_SPI_Master();
				// Neustart rsp TWI_OFF simulieren
				webspistatus |= (1<<TWI_WAIT_BIT);        //  SPI/TWI soll in der naechsten schleifen nicht mehr ermoeglicht werden
				TimeoutCounter=0;                         // wird anschliessend hochgezaehlt
            loopcount0=0;
            
            //PORTD &= ~(1<<RELAISPIN);
			}
			// sonst: Alles OK, nichts tun
		}
		else                                            // PIN ist LO, der Master ist vom Resetter abgeschaltet
		{
			if (!(pendenzstatus & (1<<RESETDELAY_BIT)))  // Bit ist noch nicht gesetzt, Reset hat gerade begonnen
			{
				pendenzstatus |= (1<<RESETDELAY_BIT);     // Bit setzen, Takt an Master unterbinden
				//PORTD |= (1<<RELAISPIN);                  // Relais ON
            //Clear_SPI_Master();
            
            // TWI_OFF simulieren
				//pendenzstatus |= (1<<SEND_STATUS0_BIT);	// SEND_STATUS0_BIT setzen, deaktiviert Takt an Master
				webspistatus &= ~(1<<SPI_SHIFT_BIT);      // eventuelles Shift-Bit zuruecksetzen
            sendWebCount=0;
            delay_ms(50);
            //PORTD &= ~(1<<RELAISPIN);                  // Relais OFF
            //setTWI_Status(1);
            
         }
			
		}
      
		loopcount0++;
		if (loopcount0>=0x2FFF)
		{
			loopcount0=0;
			// *** SPI senden
			//waitspi();
			//StartTransfer(loopcount1,1);
			
         if (loopcount1 >= 0x00FF)
			{
				
				loopcount1 = 0;
				//OSZITOGG;
				//LOOPLEDPORT |= (1<<TWILED);           // TWILED setzen, Warnung
				//TWBR=0;
				//lcdinit();
			}
			else
			{
				loopcount1++;
			}
         
			if (LOOPLEDPORTPIN &(1<<LOOPLED))
			{
				sec++;
			}
			LOOPLEDPORT ^=(1<<LOOPLED);
			
			if (webspistatus & (1<<TWI_WAIT_BIT))       // TWI ist aus, Timout ist am laufen, Sicherheit fuer wieder einschalten, wenn vergessen
         {
				lcd_gotoxy(0,0);
				lcd_puthex(TimeoutCounter);
				lcd_putc(' ');
				TimeoutCounter++;
				if (TimeoutCounter>=STATUSTIMEOUT)      // Timeout abgelaufen, alles zuruecksetzen
				{
					//lcd_gotoxy(0,0);
					//lcd_puts("Timeout  \0");
					setTWI_Status(1);
					webspistatus &= ~(1<<TWI_WAIT_BIT); // TWI wieder einschalten
					out_startdaten=STATUSTASK; // B1
					
					//lcd_gotoxy(6,0);
					//lcd_puts(actionbuf);
					outbuffer[0]=1;
					outbuffer[1]=1;
					out_hbdaten=0x01;
					out_lbdaten=0x00;
					
					TimeoutCounter=0;
					pendenzstatus &= ~(1<<SEND_STATUS0_BIT);
				}
			}
			//webspistatus |= (1<< SPI_SHIFT_BIT);
			
			//lcd_clr_line(1);
			//lcd_putc(out_startdaten);
		}
		
		//**	Beginn Start-Routinen Webserver	***********************
		
		
		//**	Ende Start-Routinen	***********************
		
		
		//**	Beginn SPI-Routinen	***********************
		
		
		//**    End SPI-Routinen*************************
		
		
		
		
		// +++Tastenabfrage+++++++++++++++++++++++++++++++++++++++
		/*
		 if (!(PINB & (1<<PORTB0))) // Taste 0
		 {
		 //lcd_gotoxy(12,1);
		 //lcd_puts("P0 Down\0");
		 
		 if (! (TastenStatus & (1<<PORTB0))) //Taste 0 war nich nicht gedrueckt
		 {
		 TastenStatus |= (1<<PORTB0);
		 Tastencount=0;
		 //lcd_gotoxy(0,1);
		 //lcd_puts("P0 \0");
		 //lcd_putint(TastenStatus);
		 //delay_ms(800);
		 }
		 else
		 {
		 
		 
		 Tastencount ++;
		 //lcd_gotoxy(7,1);
		 //lcd_puts("TC \0");
		 //lcd_putint(Tastencount);
		 
		 if (Tastencount >= Tastenprellen)
		 {
		 Tastencount=0;
		 TastenStatus &= ~(1<<PORTB0);
		 if (!(webspistatus & (1<< SEND_REQUEST_BIT)))
		 {
		 //			_delay_ms(2);
		 webspistatus |= (1<< SEND_REQUEST_BIT);
		 }
		 
		 }
		 }//else
		 
		 }	// Taste 0
		 */
		
		// ++++++++++++++++++++++++++++++++++++++++++
		
		rxdata=0;
		
		if (sendWebCount >8)
		{
			//start_web_client=1;
			sendWebCount=0;
		}
		
		//		sendWebCount=0;
		
		
		// **	Beginn SPI-Routinen	***********************
		
		//			if (((webspistatus & (1<<DATA_RECEIVE_BIT)) || webspistatus & (1<<SEND_SERIE_BIT)))
		
		if (webspistatus & (1<<SPI_SHIFT_BIT)) // SPI gefordert, in ISR von timer0 gesetzt
		{
			
			lcd_clr_line(1);
			lcd_gotoxy(0,1);
			lcd_puts("oH \0");
			
			lcd_puthex(out_startdaten);
			lcd_putc(' ');
			
         // Vom HomeServer empfangene Daten, in analyse_get_url in outdaten gesetzt > weiterleiten an Master
			
         lcd_puthex(out_hbdaten);
			lcd_puthex(out_lbdaten);
			lcd_putc(' ');
			
			lcd_puthex(outbuffer[0]);
			lcd_puthex(outbuffer[1]);
			lcd_puthex(outbuffer[2]);
			lcd_puthex(outbuffer[3]);
			
			
			for (i=0 ; i < out_BUFSIZE; i++) // 9.4.11
			{
				inbuffer[i]=0;
			}
         
			// ******************************
			// Daten auf SPI schieben
			// ******************************
#pragma mark shift
         
  			// Marker fuer SPI-Event setzen
			lcd_gotoxy(19,0);
			lcd_putc('*');
         
			SPI_shift_out();
         
			
			// Input-Daten der SPI-Aktion Daten von Master
			
			//	if (in_startdaten==0xB8)
			{
				lcd_clr_line(2);
				lcd_gotoxy(0,2);
				lcd_puts("iH \0");
				lcd_puthex(in_startdaten);
				lcd_putc(' ');
				lcd_puthex(in_hbdaten);
				lcd_puthex(in_lbdaten);
				lcd_putc(' ');
				uint8_t byteindex;
				for (byteindex=0;byteindex<4;byteindex++)
				{
					lcd_puthex(inbuffer[byteindex]);
					//lcd_putc(inbuffer[byteindex]);
				}
				
				// Fehlerbytes
				lcd_gotoxy(0,3);
				lcd_puts("iErr \0");
            //		lcd_puthex(inbuffer[24]);	// Read_Err
            //		lcd_puthex(inbuffer[25]);	// Write_Err
				lcd_puthex(inbuffer[26]);	// Zeitminuten
				lcd_puthex(inbuffer[27]);	// Heizung-Stundencode
				lcd_puthex(inbuffer[28]);	// Brenner-Stundencode
				lcd_puthex(inbuffer[29]);	// sync fehler 9.4.11
				
				
			} // in_startdaten
			
         if ((in_hbdaten==0xFF)&&(in_lbdaten==0xFF)) // Fehler
         {
            in_startdaten=DATATASK;
            
            
         }
         
         
         // Marker fuer SPI-Event entfernen
         lcd_gotoxy(19,0);
         lcd_putc(' ');
         //lcd_gotoxy(12,0);
         //lcd_putc(' ');
         
#pragma mark HomeCentral-Tasks
			switch (in_startdaten)     // Auftrag von Master
			{
				case STATUSCONFIRMTASK:	// B2, Status 0 bestaetigen
				{
					webspistatus |= (1<<STATUS_CONFIRM_BIT);
				}break;
					
					
				case DATATASK:		// C0, Daten von Homecentral an Homeserver schicken
				{
					// inbuffer wird vom Master via SPI zum Webserver geschickt.
					SolarDataString[0]='\0';
					
					char key1[]="pw=";
					char sstr[]="Pong";
					
					strcpy(SolarDataString,key1);
					strcat(SolarDataString,sstr);
					
					char d[5]={};
					//char dd[4]={};
					strcat(SolarDataString,"&d0=");
					itoa(inbuffer[9]++,d,16);
					strcat(SolarDataString,d);
					
					strcat(SolarDataString,"&d1=");
					itoa(inbuffer[10]++,d,16);
					strcat(SolarDataString,d);
					
					strcat(SolarDataString,"&d2=");
					itoa(inbuffer[11]++,d,16);
					strcat(SolarDataString,d);
					
					strcat(SolarDataString,"&d3=");
					itoa(inbuffer[12]++,d,16);
					strcat(SolarDataString,d);
					
					strcat(SolarDataString,"&d4=");
					itoa(inbuffer[13]++,d,16);
					strcat(SolarDataString,d);
					
					
					strcat(SolarDataString,"&d5=");
					itoa(inbuffer[14]++,d,16);
					strcat(SolarDataString,d);
					
					
					strcat(SolarDataString,"&d6=");
					itoa(inbuffer[15]++,d,16);
					strcat(SolarDataString,d);
					
					
					strcat(SolarDataString,"&d7=");
					itoa(inbuffer[16]++,d,16);
					strcat(SolarDataString,d);
					
					
					HeizungDataString[0]='\0';
					
               //					key1="pw=\0";
               //					sstr="Pong\0";
               
               /* In TWI_Master:
                outbuffer[0] = (HEIZUNG << 5);					// Bit 5-7: Raumnummer
                outbuffer[0] |= (Zeit.stunde & 0x1F);			//	Bit 0-4: Stunde, 5 bit
                outbuffer[1] = (0x01 << 6);						// Bits 6,7: Art=1
                outbuffer[1] |= Zeit.minute & 0x3F;				// Bits 0-5: Minute, 6 bit
                outbuffer[2] = HeizungRXdaten[0];				//	Vorlauf
                
                outbuffer[3] = HeizungRXdaten[1];				//	Rücklauf
                outbuffer[4] = HeizungRXdaten[2];				//	Aussen
                
                outbuffer[5] = 0;
                outbuffer[5] |= HeizungRXdaten[3];				//	Brennerstatus Bit 2
                outbuffer[5] |= HeizungStundencode;			// Bit 4, 5 gefiltert aus Tagplanwert von Brenner und Mode
                outbuffer[5] |= RinneStundencode;				// Bit 6, 7 gefiltert aus Tagplanwert von Rinne
                
                uebertragen in d5 von HeizungDataString
                */
               
					//char d[4]={};
					strcpy(HeizungDataString,key1);
					strcat(HeizungDataString,sstr);
					
					strcpy(HeizungDataString,"&d0=");
					itoa(inbuffer[0]++,d,16);
					strcat(HeizungDataString,d);
					
					strcat(HeizungDataString,"&d1=");
					itoa(inbuffer[1]++,d,16);
					strcat(HeizungDataString,d);
					
					strcat(HeizungDataString,"&d2=");
					itoa(inbuffer[2]++,d,16);
					strcat(HeizungDataString,d);
					
					strcat(HeizungDataString,"&d3=");
					itoa(inbuffer[3]++,d,16);
					strcat(HeizungDataString,d);
					
					strcat(HeizungDataString,"&d4=");
					itoa(inbuffer[4]++,d,16);
					strcat(HeizungDataString,d);
					
					
					strcat(HeizungDataString,"&d5=");
					itoa(inbuffer[5]++,d,16);
					strcat(HeizungDataString,d);
					
					
					strcat(HeizungDataString,"&d6=");
					itoa(inbuffer[6]++,d,16);
					strcat(HeizungDataString,d);
					
					
					strcat(HeizungDataString,"&d7=");
					itoa(inbuffer[7]++,d,16);
					strcat(HeizungDataString,d);
					
					//key1="pw=\0";
					//sstr="Pong\0";
               
               // inbuffer wird vom Master via SPI zum Webserver geschickt.
               
               // AlarmDataString geht an alarm.pl
               
					AlarmDataString[0]='\0';
					strcpy(AlarmDataString,"pw=");
					strcat(AlarmDataString,"Pong");
					
               // Alarm vom Master:
               // Bits:
               // WASSERALARMESTRICH    1
               // TIEFKUEHLALARM        3
               // WASSERALARMKELLER     4
               strcat(AlarmDataString,"&d0=");
					itoa(inbuffer[40]++,d,16);
					strcat(AlarmDataString,d);
               
               // TWI-errcount Master> main> l 1672
					strcat(AlarmDataString,"&d1=");
					itoa(inbuffer[30]++,d,16);
					strcat(AlarmDataString,d);
					
               // Echo von WoZi
					strcat(AlarmDataString,"&d2=");
					itoa(inbuffer[29]++,d,16);
					strcat(AlarmDataString,d);
					
               // txbuffer[0] von Heizung: Stundencode l 2807
					strcat(AlarmDataString,"&d3=");
					itoa(inbuffer[28]++,d,16);
					strcat(AlarmDataString,d);
					
               // HeizungStundencode l 2773
					strcat(AlarmDataString,"&d4=");
					itoa(inbuffer[27]++,d,16);
					strcat(AlarmDataString,d);
					
					strcat(AlarmDataString,"&d5=");
					itoa(inbuffer[26]++,d,16);
					strcat(AlarmDataString,d);
               
					strcat(AlarmDataString,"&d6=");
					itoa(inbuffer[25]++,d,16);
					strcat(AlarmDataString,d);
               
					strcat(AlarmDataString,"&d7=");
					itoa(inbuffer[24]++,d,16);
					strcat(AlarmDataString,d);
               
               // Zeit.minute, 6 bit
					strcat(AlarmDataString,"&d8=");
					itoa(inbuffer[23]++,d,16);
					strcat(AlarmDataString,d);
					
					
					strcat(AlarmDataString,"&d9=");
               //					uint8_t diff=(errCounter-oldErrCounter);
               //					itoa(diff++,d,16); // nur Differenz übermitteln
               
					itoa(errCounter++,d,16);
					strcat(AlarmDataString,d);
					//oldErrCounter = errCounter;
					
					strcat(AlarmDataString,"&d10=");
					itoa(SPI_ErrCounter++,d,16);
					strcat(AlarmDataString,d);
					
               
               // End Alarm
 					
					//in_startdaten=0;
					
					//lcd_clr_line(0);
					
					//out_startdaten=DATATASK;
					
					//	lcd_gotoxy(0,1);
					//	lcd_puts("l:\0");
					//	lcd_putint2(strlen(SolarDataString));
					//lcd_puts(SolarDataString);
					//lcd_puts(SolarVarString);
					
					// 5.8.10
					sendWebCount++;
					//		sendWebCount=1; // SolarDataString schicken
					
					//						lcd_gotoxy(5,0);
					//						lcd_puts("cnt:\0");
					//						lcd_puthex(sendWebCount);
					
					
					// *** SPI senden
					//StartTransfer(WebRxStartDaten++,1);
					
					
				}
					break;
					
				case EEPROMREPORTTASK:	// B4 EEPROM-Daten von HomeCentral angekommen, an Homeserver schicken
				{
					//				sendWebCount=6;															// senden von TWI-Daten an HomeServer verzoegern
					
					lcd_gotoxy(19,0);
					lcd_putc('4');
					
					lcd_clr_line(3);
					lcd_gotoxy(0,3);
					lcd_puts("oW \0");
					lcd_puthex(in_startdaten);
					lcd_putc(' ');
					lcd_puthex(in_hbdaten);
					lcd_puthex(in_lbdaten);
					lcd_putc(' ');
					uint8_t byteindex;
					for (byteindex=0;byteindex<4;byteindex++)
					{
						lcd_puthex(inbuffer[byteindex]);
					}
					
					EEPROM_String[0]='\0';
					//OSZILO;
					uint8_t i=0;
					char d[4]={};
					for (i=0;i< twi_buffer_size;i++)
					{
						itoa(inbuffer[i],(char*)d,16);
						strcat(EEPROM_String,(char*)d);
						
						if (i < (twi_buffer_size-1))
						{
							strcat(EEPROM_String,"+\0"); // Trennzeichen einfuegen
						}
						
						
					}
					//OSZIHI;
					//lcd_gotoxy(11,3);
					//lcd_puts("   ");
					//lcd_puts(EEPROM_String);
					/*
					 lcd_putc(EEPROM_String[0]);
					 lcd_putc(EEPROM_String[1]);
					 lcd_putc(EEPROM_String[2]);
					 lcd_putc(EEPROM_String[3]);
					 lcd_putc(EEPROM_String[4]);
					 lcd_putc(EEPROM_String[5]);
					 lcd_putc(EEPROM_String[6]);
					 lcd_putc(EEPROM_String[7]);
					 */
					//lcd_gotoxy(16,1);
					//lcd_putc('#');
					//lcd_putc(' ');
					//lcd_putc(' ');
					
					//sendWebCount++;
					
					webspistatus |= (1<<SPI_DATA_READY_BIT);			// EEPROM-Daten sind bereit
					
					//out_startdaten=DATATASK;
					
					rxdata=0;
				}break;  // EEPROMREPORTTASK
					
				case EEPROMCONFIRMTASK: // EEPROM-Daten vom HomeServer auf die HomeCentral geschickt
				{
					lcd_gotoxy(12,0);
					lcd_putc('w');
					webspistatus |= (1<<WRITE_CONFIRM_BIT);			// EEPROM-Daten sind geschrieben
					
				}break; // EEPROMCONFIRMTASK
					
				case ERRTASK:
				{
               /* 
                // in Master:
                out_lbdaten=13;
                out_hbdaten=13;
                
                outbuffer[0]=Read_Err;
                outbuffer[1]=Write_Err;
                outbuffer[2]=EEPROM_Err;

                */
					sendWebCount=8; // Error senden
               
				}break;
					
					
				default:
				{
					//			out_startdaten=DATATASK;
					
				}
					break;
			}
			
			switch (out_startdaten) // in analyse_get_url gesetzt
			{
				case EEPROMREADTASK:
					delay_ms(800);// warten, dann nochmals senden, um Daten vom EEPROM zu laden
					//delay_ms(800);
					break;
					
				case EEPROMWRITETASK:
					delay_ms(800);// warten, dann nochmals senden, um Daten zum EEPROM zu senden
					break;
               
				case STATUSTASK:
				{
					if (out_hbdaten==0x00) // Status soll Null werden
					{
						lcd_gotoxy(7,0);
						lcd_puts("pend\0");
					}
               
				}break;
               
				default:
					webspistatus &= ~(1<<SPI_SHIFT_BIT); // SPI erledigt
               break;
			}
         
			
			// Ausgangsdaten reseten
			out_startdaten=DATATASK;
			out_hbdaten=0;
			out_lbdaten=0;
			
			uint8_t i=0;
			for (i=0 ; i<8; i++)
			{
	//			outbuffer[i]=0;
			}
			
			// Marker fuer SPI-Event loeschen
			lcd_gotoxy(19,0);
			lcd_putc(' ');
			
			//PORTD |=(1<<3);
			//webspistatus &= ~(1<<SPI_SHIFT_BIT);
			//
		}														// ** Ende SPI-Routinen  *************************
		else				// Normalfall, kein shift-Bit gesetzt
		{
#pragma mark packetloop
			
			// **	Beginn Ethernet-Routinen	***********************
			
			// handle ping and wait for a tcp packet
			cli();
			
			dat_p=packetloop_icmp_tcp(buf,enc28j60PacketReceive(BUFFER_SIZE, buf));
			//dat_p=1;
			
			if(dat_p==0) // Kein Aufruf, eigene Daten senden an Homeserver
			{
            datcounter++;
				//lcd_gotoxy(0,1);
				//lcd_puts("TCP\0");
				//lcd_puthex(start_web_client);
				//lcd_puthex(sendWebCount);
				
				if ((start_web_client==1)) // In Ping_Calback gesetzt: Ping erhalten
				{
					//OSZILO;
					sec=0;
					//lcd_gotoxy(0,0);
					//lcd_puts("    \0");
					lcd_clr_line(1);
					lcd_gotoxy(12,0);
					lcd_puts("ping ok\0");
					lcd_clr_line(1);
					delay_ms(100);
					start_web_client=2;
					web_client_attempts++;
					
					
					mk_net_str(str,pingsrcip,4,'.',10);
					char* pingsstr="ideur01\0";
					//						lcd_gotoxy(0,1);
					//						lcd_puts(str);
					//delay_ms(1000);
					
					urlencode(pingsstr,urlvarstr);
					//lcd_gotoxy(0,1);
					//lcd_puts(urlvarstr);
					//delay_ms(1000);
					
					//strcat(urlvarstr,"data=
				//	client_browse_url(PSTR("/cgi-bin/solar.pl?pw="),urlvarstr,PSTR(WEBSERVER_VHOST),&ping_callback);
					//client_browse_url(PSTR("/blatt/cgi-bin/home.pl?"),urlvarstr,PSTR(WEBSERVER_VHOST),&browserresult_callback);
					
				}
				
				// reset after a delay to prevent permanent bouncing
				if (sec>10 && start_web_client==5)
				{
					start_web_client=0;
				}
            
            if (TESTSERVER)
            {
               if (sendWebCount == 1) // Solar-Daten an HomeServer -> solar schicken
               {
                  start_web_client=2;
                  web_client_attempts++;
                  //strcat(SolarVarString,SolarDataString);
                  start_web_client=0;
                  lcd_gotoxy(10,0);
                  lcd_putc('s');
                  // Daten an Solar schicken
                  client_browse_url((char*)PSTR("/cgi-bin/experiment.pl?"),SolarDataString,(char*)PSTR(WEBSERVER_VHOST),&exp_browserresult_callback);
                  callbackstatus &= ~(1<< EXPCALLBACK);
                  
                  sendWebCount++;
                  
               }
            }
            else
            {
#pragma mark SolarDaten an HomeServer schicken
               if (sendWebCount == 1) // Solar-Daten an HomeServer -> solar schicken
               {
                  
                  start_web_client=2;
                  web_client_attempts++;
                  
                  //strcat(SolarVarString,SolarDataString);
                  start_web_client=0;
                  
                  
                  lcd_gotoxy(11,0);
                  lcd_putc('s');
                  
                  // Daten an Solar schicken
                  client_browse_url((char*)PSTR("/cgi-bin/solar.pl?"),SolarDataString,(char*)PSTR(WEBSERVER_VHOST),&solar_browserresult_callback);
                  
                  
                  sendWebCount++;
                  
               }
               
               // +++++++++++++++
#pragma mark HeizungDaten an HomeServer schicken
               if (sendWebCount == 3) // Home-Daten an HomeServer -> home schicken
               {
                  
                  start_web_client=5;
                  web_client_attempts++;
                  
                  
                  start_web_client=0;
                  
                  lcd_gotoxy(11,0);
                  lcd_putc('h');
                  
                  // Daten an Home schicken
                  client_browse_url((char*)PSTR("/cgi-bin/home.pl?"),HeizungDataString,(char*)PSTR(WEBSERVER_VHOST),&home_browserresult_callback);
                  
                  
                  //client_browse_url("/cgi-bin/home.pl?",HeizungDataString,WEBSERVER_VHOST,&home_browserresult_callback);
                  
                  //lcd_puts("cgi l:\0");
                  //lcd_putint2(strlen(SolarDataString));
                  
                  sendWebCount++;
               }
               
#pragma mark AlarmDaten an HomeServer schicken
               if (sendWebCount == 6) // Alarm-Daten an HomeServer ->Alarm schicken
               {
                  
                  start_web_client=7;
                  web_client_attempts++;
                  
                  start_web_client=0;
                  
                  lcd_gotoxy(11,0);
                  lcd_putc('a');
                  
                  // Daten an Alarm schicken
                  client_browse_url((char*)PSTR("/cgi-bin/alarm.pl?"),AlarmDataString,(char*)PSTR(WEBSERVER_VHOST),&alarm_browserresult_callback);
                  
                  
                  //client_browse_url("/cgi-bin/alarm.pl?",AlarmDataString,WEBSERVER_VHOST,&alarm_browserresult_callback);
                  
                  //lcd_puts("cgi l:\0");
                  //lcd_putint2(strlen(SolarDataString));
                  
                  sendWebCount++;
               }
               
               if (sendWebCount == 8) // incrementieren
               {
                  
                  
                  sendWebCount++;
               }
            } // if TESTSERVER
            // ++++++++++++++
				continue;
				
			} // dat_p=0
			
			/*
			 if (strncmp("GET ",(char *)&(buf[dat_p]),4)!=0)
			 {
			 // head, post and other methods:
			 //
			 // for possible status codes see:
			 
			 // http://www.w3.org/Protocols/rfc2616/rfc2616-sec10.html
			 lcd_gotoxy(0,0);
			 lcd_puts("*GET*\0");
			 dat_p=http200ok();
			 dat_p=fill_tcp_data_p(buf,dat_p,PSTR("<h1>HomeCentral 200 OK</h1>"));
			 goto SENDTCP;
			 }
			 */
			
			
			if (strncmp("/ ",(char *)&(buf[dat_p+4]),2)==0) // Slash am Ende der URL, Status-Seite senden
			{
				lcd_gotoxy(10,0);
				lcd_puts("+/+\0");
				dat_p=http200ok(); // Header setzen
				dat_p=fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n<h1>200 OK</h1>"));
				//		dat_p=fill_tcp_data_p(buf,dat_p,PSTR("<h1>HomeCentral 200 OK</h1>"));
				dat_p=print_webpage_status(buf);
				goto SENDTCP;
			}
			else
			{
				// Teil der URL mit Form xyz?uv=... analysieren
				
#pragma mark cmd
				
//				out_startdaten=DATATASK;	// default
				
				// out_daten setzen
				cmd=analyse_get_url((char *)&(buf[dat_p+5]));
				
				//lcd_gotoxy(5,0);
				//lcd_puts("cmd:\0");
				//lcd_putint(cmd);
				//lcd_putc(' ');
				if (cmd == 1)
				{
					dat_p = print_webpage_confirm(buf);
				}
				else if (cmd == 2)	// TWI > OFF
				{
#pragma mark cmd 2
					//lcd_puts("OFF");
					setTWI_Status(0);
					//webspistatus |= (1<< TWI_WAIT_BIT);
					out_startdaten=STATUSTASK;
					lcd_clr_line(1);
					lcd_gotoxy(0,1);
					lcd_puts("oS \0");
					
					lcd_puthex(out_startdaten); // B1, in analyse_get_url gesetzt
					lcd_putc(' ');
					
					// Vom HomeServer empfangene Angaben fuer Status
					//lcd_putc('h');
					lcd_puthex(out_hbdaten);
					//lcd_putc(' ');
					//lcd_putc('l');
					lcd_puthex(out_lbdaten);
					lcd_putc(' ');
					lcd_puthex(outbuffer[0]);
					lcd_puthex(outbuffer[1]);
					lcd_puthex(outbuffer[2]);
					lcd_puthex(outbuffer[3]);
					lcd_gotoxy(19,0);
					lcd_putc('2');
					OutCounter++;
					OFFCounter++;
					//	twibuffer[0]=0;
					
					EventCounter=0x2FFF;
					webspistatus |= (1<<TWI_STOP_REQUEST_BIT); // TWI ausschalten anmelden, schaltet in ISR das TWI_WAIT_BIT ein
					TimeoutCounter =0;
					
					dat_p = print_webpage_ok(buf,(void*)"status0"); // status 0 erst bestaetigen, wenn an Master uebertragen
					
					//EventCounter=0x18FF; // Timer fuer SPI vorwaertsstellen 1Aff> 0.7s
               
				}
				else if (cmd == 3)
				{
#pragma mark cmd 3
					//lcd_puts("ON ");
					setTWI_Status(1);
					//webspistatus |= (1<< SEND_REQUEST_BIT);
					//webspistatus &= ~(1<< TWI_WAIT_BIT);			// Daten wieder an HomeCentral senden
					
               
               
               lcd_clr_line(0);
					lcd_gotoxy(13, 0);
					lcd_puts("V:\0");
					lcd_puts(VERSION);
					
					lcd_clr_line(1);
					lcd_gotoxy(0,1);
					lcd_puts("oS \0");
					
					lcd_puthex(out_startdaten);
					lcd_putc(' ');
					
					// Empfangene Angaben fuer Status
					//lcd_putc('h');
					lcd_puthex(out_hbdaten);
					//lcd_putc(' ');
					//lcd_putc('l');
					lcd_puthex(out_lbdaten);
					lcd_putc(' ');
					lcd_puthex(outbuffer[0]);
					lcd_puthex(outbuffer[1]);
					lcd_puthex(outbuffer[2]);
					lcd_puthex(outbuffer[3]);
					
					OutCounter++;
					ONCounter++;
					
					//						twibuffer[0]=1;
					
					//						send_cmd &= ~(1<<1);									// Data-bereit-bit zueruecksetzen
					//						webspistatus &= ~(1<<SPI_DATA_READY_BIT);		// Data-bereit-bit zueruecksetzen
					
					EventCounter=0x2FFF;						// Eventuell laufende TWI-Vorgaenge noch beenden lassen
					webspistatus &= ~(1<<TWI_WAIT_BIT); // TWI wieder einschalten
					
					// an HomeServer bestaetigen
					dat_p = print_webpage_ok(buf,(void*)"status1");
					
					//EventCounter=0x18FF; // Timer fuer SPI vorwaertsstellen 1Aff> 0.7s
					//	webspistatus |= (1<<SPI_REQUEST_BIT);
					
					
				}
#pragma mark cmd 6
				else if (cmd == 6) //   Bestaetigung fuer readEEPROM-Request an HomeServer senden, mit lb, hb
				{
					lcd_clr_line(3);
					lcd_gotoxy(0,3);
					lcd_puts("rE \0");
					
					lcd_puthex(out_startdaten); // EEPROMREADTASK, in analyse_get_url gesetzt
					lcd_putc(' ');
					
					// Empfangene Angaben fuer EEPRPOM
					lcd_puthex(out_hbdaten);
					lcd_puthex(out_lbdaten);
					lcd_putc(' ');
					lcd_puthex(outbuffer[0]);
					lcd_puthex(outbuffer[1]);
					lcd_puthex(outbuffer[2]);
					lcd_puthex(outbuffer[3]);
					
					// EEPROM-Daten von HomeCentral laden
					
					webspistatus |= (1<<SPI_SHIFT_BIT);
					
					// an HomeServer bestaetigen
					dat_p = print_webpage_ok(buf,(void*)"radr\0");
					//lcd_gotoxy(17,0);
					//lcd_puts("    \0");
					TimeoutCounter =0;
				}
				
#pragma mark cmd 8
				else if (cmd == 8) // von HomeCentral empfangene EEPROM-Daten an HomeServer senden
				{
					//lcd_gotoxy(19,1);
					//lcd_putc('*');
					//lcd_puts(" - ");
					//lcd_gotoxy(0,1);
					//lcd_puts("rE\0");
					/*
					 lcd_puthex(out_startdaten);
					 
					 lcd_putc(' ');
					 lcd_puthex(out_hbdaten);
					 //lcd_putc(' ');
					 //lcd_putc('l');
					 lcd_puthex(out_lbdaten);
					 
					 //lcd_puthex(EEPROMRxDaten[0]);
					 //lcd_putc(' ');
					 //lcd_puthex(EEPROMRxDaten[1]);
					 //lcd_puthex(EEPROMRxDaten[2]);
					 
					 lcd_putc(' ');
					 lcd_puthex(outbuffer[0]);
					 lcd_puthex(outbuffer[1]);
					 //		lcd_puthex(outbuffer[2]);
					 //		lcd_puthex(outbuffer[3]);
					 */
					uint8_t j=0;
					//for (j=0;j<12;j++)
					{
						//	lcd_putc(EEPROM_String[j]);
					}
					/*
					 lcd_putc(EEPROM_String[0]);
					 lcd_putc(EEPROM_String[1]);
					 lcd_putc(EEPROM_String[2]);
					 lcd_putc(EEPROM_String[3]);
					 
					 lcd_gotoxy(6,0);
					 lcd_putc('s');
					 lcd_puthex(send_cmd);
					 */
					//webspistatus |= (1<<SEND_REQUEST_BIT);
					//EventCounter=0x2FF0; // Timer fuer SPI vorwaertsstellen 1Aff> 0.7s
					//webspistatus |= (1<<SPI_REQUEST_BIT);
					//webspistatus |= (1<<SPI_SHIFT_BIT);
					if (webspistatus &(1<<SPI_DATA_READY_BIT)) // Daten sind bereit
					{
						//dat_p = print_webpage_ok(buf,(void*)"eeprom+\0");
						
						// an HomeServer senden
						dat_p = print_webpage_send_EEPROM_Data(buf,(void*)EEPROM_String);
						webspistatus &= ~(1<<SPI_DATA_READY_BIT);		// Data-bereit-bit zueruecksetzen
					}
					else
					{
						dat_p = print_webpage_ok(buf,(void*)"eeprom-\0");
						
					}
					TimeoutCounter =0;
				}
				
				
				else if (cmd == 9) // EEPROM-Daten vom HomeServer an Master senden
				{
#pragma mark cmd 9
					//lcd_gotoxy(17,0);
					//lcd_puts(" - ");
					lcd_clr_line(3);
					lcd_gotoxy(0,3);
					lcd_puts("wE\0");
					lcd_putc(' ');
					
					lcd_puthex(out_startdaten);
					lcd_putc(' ');
					lcd_puthex(out_hbdaten);
					lcd_puthex(out_lbdaten);
					lcd_putc(' ');
					
					lcd_puthex(outbuffer[0]);
					lcd_puthex(outbuffer[1]);
					lcd_puthex(outbuffer[2]);
					lcd_puthex(outbuffer[3]);
					//lcd_puthex(outbuffer[4]);
					//lcd_puthex(outbuffer[5]);
					//lcd_puthex(outbuffer[6]);
					//lcd_puthex(outbuffer[7]);
					
					webspistatus |= (1<<SPI_SHIFT_BIT);
					
					// an HomeServer bestaetigen
					dat_p = print_webpage_ok(buf,(void*)"wadr");
					TimeoutCounter =0;
				}
            
#pragma mark cmd 7
				else if (cmd == 7)					// Anfrage, ob EEPROM-Daten erfolgreich geschrieben
				{
					lcd_gotoxy(19,0);
					lcd_putc('7');
					if (webspistatus & (1<< WRITE_CONFIRM_BIT))
					{
						lcd_gotoxy(13,0);
						lcd_putc('+');
						dat_p = print_webpage_ok(buf,(void*)"write+");
						
						// WRITE_CONFIRM_BIT zuruecksetzen
						//webspistatus &= ~(1<< WRITE_CONFIRM_BIT);
						
					}
					else
					{
						//lcd_putc('-');
						dat_p = print_webpage_ok(buf,(void*)"write-");
						
					}
					//webspistatus &= ~(1<< TWI_WAIT_BIT);			// Daten wieder an HomeCentral senden
					TimeoutCounter =0;
				}
				
				else if (cmd == 15) // Homebus reset
				{
					PORTC |=(1<<PORTC0); //Pin 0 von Port C als Ausgang fuer Reset-Relais
					_delay_ms(100);
					PORTC &=~(1<<PORTC0);
					// Verwirrung stiften: 401-Fehler wenn OK
					dat_p=fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 401 Unauthorized\r\nContent-Type: text/html\r\n\r\n<h1>401 Zugriff auf Home abgelehnt</h1>"));
					
				}
				
#pragma mark cmd 10
				else if (cmd == 10) // Status0 bestaetigen
				{
					lcd_gotoxy(8,0);
					lcd_putc('+');
					if (pendenzstatus & (1<<SEND_STATUS0_BIT))
					{
						lcd_putc('*');
						dat_p = print_webpage_ok(buf,(void*)"status0+");
						pendenzstatus &=  ~(1<< SEND_STATUS0_BIT); // Sache hat sich erledigt
					}
					else
					{
						dat_p = print_webpage_ok(buf,(void*)"status0-");
					}
					
				}
				
				else
				{
					dat_p=fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 401 Unauthorized\r\nContent-Type: text/html\r\n\r\n<h1>401 Zugriff verweigert</h1>"));
				}
            
            
            
				cmd=0;
				// Eingangsdaten reseten, sofern nicht ein Status0-Wait im Gang ist:
				//if ((pendenzstatus & (1<<SEND_STATUS0_BIT)))
				{
					in_startdaten=0;
					in_hbdaten=0;
					in_lbdaten=0;
					
					uint8_t i=0;
					for (i=0 ; i<8; i++)
					{
						inbuffer[i]=0;
					}
				}
				
				goto SENDTCP;
			}
			//
			//	OSZIHI;
		SENDTCP:
			www_server_reply(buf,dat_p); // send data
			
			
		} // twi not busy
	}
	return (0);
}

/*********************************************
 * vim:sw=8:ts=8:si:et
 * To use the above modeline in vim you must have "set modeline" in your .vimrc
 * Author: Guido Socher
 * Copyright: GPL V2
 *
 * Some common utilities needed for IP and web applications
 *********************************************/
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "ip_config.h"

#ifdef FROMDECODE_websrv_help
// search for a string of the form key=value in
// a string that looks like q?xyz=abc&uvw=defgh HTTP/1.1\r\n
//
// The returned value is stored in strbuf. You must allocate
// enough storage for strbuf, maxlen is the size of strbuf.
// I.e the value it is declated with: strbuf[5]-> maxlen=5

uint8_t find_key_val(char *str,char *strbuf, uint8_t maxlen,char *key)
{
        uint8_t found=0;
        uint8_t i=0;
        char *kp;
        kp=key;
        while(*str &&  *str!=' ' && *str!='\n' && found==0){
                if (*str == *kp){
                        kp++;
                        if (*kp == '\0'){
                                str++;
                                kp=key;
                                if (*str == '='){
                                        found=1;
                                }
                        }
                }else{
                        kp=key;
                }
                str++;
        }
        if (found==1){
                // copy the value to a buffer and terminate it with '\0'
                while(*str &&  *str!=' ' && *str!='\n' && *str!='&' && i<maxlen-1){
                        *strbuf=*str;
                        i++;
                        str++;
                        strbuf++;
                }
                *strbuf='\0';
        }
        // return the length of the value
        return(i);
}



// Analyse_get_url von test_twitter

/*
// analyse the url given
// return values: -1 invalid password
//                -2 no command given 
//                0 switch off
//                1 switch on
//
//                The string passed to this function will look like this:
//                /?mn=1&pw=secret HTTP/1.....
//                / HTTP/1.....
int8_t analyse_get_url(char *str)
{
        uint8_t mn=0;
        char kvalstrbuf[10];
        // the first slash:
        if (str[0] == '/' && str[1] == ' ')
		  {
                // end of url, display just the web page
                return(-2);
        }
        // str is now something like ?pw=secret&mn=0 or just end of url
        if (find_key_val(str,kvalstrbuf,10,"mn")){
                if (kvalstrbuf[0]=='1'){
                        mn=1;
                }
                // to change the mail notification one needs also a valid passw:
                if (find_key_val(str,kvalstrbuf,10,"pw")){
                        if (verify_password(kvalstrbuf)){
                                return(mn);
                        }else{
                                return(-1);
                        }
                }
        }
        // browsers looking for /favion.ico, non existing pages etc...
        return(-1);
}

*/
// convert a single hex digit character to its integer value
unsigned char h2int(char c)
{
        if (c >= '0' && c <='9'){
                return((unsigned char)c - '0');
        }
        if (c >= 'a' && c <='f'){
                return((unsigned char)c - 'a' + 10);
        }
        if (c >= 'A' && c <='F'){
                return((unsigned char)c - 'A' + 10);
        }
        return(0);
}

// decode a url string e.g "hello%20joe" or "hello+joe" becomes "hello joe"
void urldecode(char *urlbuf)
{
	char c;
	char *dst;
	dst=urlbuf;
	while ((c = *urlbuf)) 
	{
		if (c == '+') c = ' ';
		if (c == '%') 
		{
			urlbuf++;
			c = *urlbuf;
			urlbuf++;
			c = (h2int(c) << 4) | h2int(*urlbuf);
		}
		*dst = c;
		dst++;
		urlbuf++;
	}
	*dst = '\0';
}

// decode a url string e.g "hello%20joe" or "hello+joe" becomes "hello joe"
void eepromdecode(char *urlbuf, char data[])
{
		uint8_t index=0;
        char c;
       char *dst;
        //dst=urlbuf;
        while ((c = *urlbuf)) 
		  {
			  
			  if (c == '+') // Trennung
			  {
				  strcpy(dst, &(data[index]));
				  index++;
				  urlbuf++;
				  //c = *urlbuf;
				  //urlbuf++;
				  //c = (h2int(c) << 4) | h2int(*urlbuf);
			  }
			  *dst = c;
			  dst++;
			  urlbuf++;
        } // while
        *dst = '\0';
}



#endif //  FROMDECODE_websrv_help

#ifdef URLENCODE_websrv_help

// convert a single character to a 2 digit hex str
// a terminating '\0' is added
void int2h(char c, char *hstr)
{
        hstr[1]=(c & 0xf)+'0';
        if ((c & 0xf) >9){
                hstr[1]=(c & 0xf) - 10 + 'a';
        }
        c=(c>>4)&0xf;
        hstr[0]=c+'0';
        if (c > 9){
                hstr[0]=c - 10 + 'a';
        }
        hstr[2]='\0';
}

// there must be enoug space in urlbuf. In the worst case that is
// 3 times the length of str
void urlencode(char *str,char *urlbuf)
{
        char c;
        while ((c = *str)) 
		{
                if (c == ' '||isalnum(c))
				{ 
                        if (c == ' ')
						{ 
                                c = '+';
                        }
                        *urlbuf=c;
                        str++;
                        urlbuf++;
                        continue;
                }
                *urlbuf='%';
                urlbuf++;
                int2h(c,urlbuf);
                urlbuf++;
                urlbuf++;
                str++;
        }
        *urlbuf='\0';
}

#endif // URLENCODE_websrv_help

// parse a string and extract the IP to bytestr
uint8_t parse_ip(uint8_t *bytestr,char *str)
{
        char *sptr;
        uint8_t i=0;
        sptr=NULL;
        while(i<4){
                bytestr[i]=0;
                i++;
        }
        i=0;
        while(*str && i<4){
                // if a number then start
                if (sptr==NULL && isdigit(*str)){
                        sptr=str;
                }
                if (*str == '.'){
                        *str ='\0';
                        bytestr[i]=(atoi(sptr)&0xff);
                        i++;
                        sptr=NULL;
                }
                str++;
        }
        *str ='\0';
        if (i==3){
                bytestr[i]=(atoi(sptr)&0xff);
                return(0);
        }
        return(1);
}

// take a byte string and convert it to a human readable display string  (base is 10 for ip and 16 for mac addr), len is 4 for IP addr and 6 for mac.
void mk_net_str(char *resultstr,uint8_t *bytestr,uint8_t len,char separator,uint8_t base)
{
        uint8_t i=0;
        uint8_t j=0;
        while(i<len){
                itoa((int)bytestr[i],&resultstr[j],base);
                // search end of str:
                while(resultstr[j]){j++;}
                resultstr[j]=separator;
                j++;
                i++;
        }
        j--;
        resultstr[j]='\0';
}

// end of websrv_help_functions.c

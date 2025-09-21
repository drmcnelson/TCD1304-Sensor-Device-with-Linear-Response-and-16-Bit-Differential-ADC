/* ====================================================================
   Parse srings for words, numbers, etc

   Apart from wordLength(), these return a pointer to the
   next character in the string, or null if they fail
   
 */

#include "Arduino.h"

#include <ctype.h>
#include <limits.h>

char printbuffer[256];
unsigned int nprintbuffer = 0;
int serialPrintf(const char* format, ...)
{
  int n;
  va_list argptr;
  va_start(argptr, format);
  n = vsprintf (&printbuffer[nprintbuffer], format, argptr );
  va_end(argptr);
  if (n > 0) {
    Serial.print(&printbuffer[nprintbuffer]);
    nprintbuffer = (nprintbuffer+n)&127;
  }
  else if (n<0) {
    Serial.print( "Error: vsprintf");
    Serial.println(format);
  }
  return n;
}

int serialPrintlnf(const char* format, ...)
{
  int n;
  va_list argptr;
  va_start(argptr, format);
  n = vsprintf (&printbuffer[nprintbuffer], format, argptr );
  va_end(argptr);
  if (n > 0) {
    Serial.println(&printbuffer[nprintbuffer]);
    nprintbuffer = (nprintbuffer+n)&127;
  }
  else if (n<0) {
    Serial.print( "Error: vsprintf");
    Serial.println(format);
  }
  return n;
}

char *eow(char *s) {
  while ( s && *s && !isspace(*s) ) s++;
  return s;
}

char *eos(char*s) {
  while ( s && *s && isspace(*s) ) s++;
  return s;
}

unsigned int wordLength( char *s ) {
  char *s0 = s;
  while( *s && !isspace( *s ) ){
    s++;
  }
  return (s-s0);  
}

char *nextWord( char *s ) {
  while ( s  && *s && !isspace(*s) ) {
    s++;
  }
  while ( s  && *s && isspace(*s) ) {
    s++;
  }
  if ( s && *s ) {
    return s;
  }
  return 0;   
}


unsigned int countWords( char *s ) {
  unsigned int n = 0;
  if ( s && *s && !isspace(*s) ) {
    n++;
  }
  while( (s=nextWord(s)) ) {
    n++;
  }
  return n;
}

bool strmatch(const char *s, const char *key) {
  char *k = (char *)key;
  if (!s || !(*s) ||
      !key || !(*key) ||
      isspace(*s) || isspace(*key) ||
      (strlen(s) != strlen(key))
      ) return  false;

  while( *s && *k && (tolower(*s)==tolower(*k))) {
    s++;
    k++;
  }

  return ( tolower(*s) == tolower(*k) );
}
  
char *startsWith( char *s, const char *key ) {

  int n = strlen(key);

  if ( s && *s && key && *key ) {

    // skip leading spaces
    while ( *s && isspace(*s) ) s++;
      
    if ( *s && !strncmp( s, key, n ) ) {
      return s + n;
    }
    
  }
  return 0;
}

char *parseBool(char *s, bool *b) {
  char c = 0;
  s = nextWord(s);
  if (s) {
    c = tolower(*s);
    if ((c == 'y')||(c == 't')||(c == '1')) {
      if (b) *b = true;
      return eow(s);
    }
    if ((s[0] == 'n')||(s[0] == 'f')||(s[0] == '0')) {
      if (b) *b = false;
      return eow(s);
    }
  }
  return 0;
}

/*
char *parseUint8( char *s, uint8_t *u ) {
  unsigned long int l;
  char *p = s;
  if ( p && *p ) {
    l = strtoul( s, &p, 0 );
    if ( (p > s) && (l <= 0xFF) ) {
      *u = (uint8_t) l;
      return p;
    }
  }
  return 0;
}
*/

char *parseUint8( char *s, uint8_t *u, uint8_t ulim=0xFF ) {
  unsigned long int l;
  char *p = s;
  if ( p && *p ) {
    l = strtoul( s, &p, 0 );
    if ( (p > s) && (l <= ulim) ) {
      if (u) *u = (uint8_t) l;
      return p;
    }
  }
  return 0;
}


char *parseUint( char *s, unsigned int *u ) {
  unsigned long int l;
  char *p = s;
  if ( p && *p ) {
    l = strtoul( s, &p, 0 );
    if ( (p > s) && (l <= UINT_MAX) ) {
      if (u) *u = (unsigned int) l;
      return p;
    }
  }
  return 0;
}

char *parseUint16( char *s, uint16_t *u ) {
  unsigned long int l;
  char *p = s;
  if ( p && *p ) {
    l = strtoul( s, &p, 0 );
    if ( (p > s) && (l <= UINT16_MAX) ) {
      if (u) *u = (uint16_t) l;
      return p;
    }
  }
  return 0;
}

char *parseUint32( char *s, uint32_t *u ) {
  unsigned long int l;
  char *p = s;
  if ( p && *p ) {
    l = strtoul( s, &p, 0 );
    if ( (p > s) && (l <= UINT32_MAX) ) {
      if (u) *u = (uint32_t) l;
      return p;
    }
  }
  return 0;
}

char *scaling( float *f, char *s) {
  if (s && !isspace(s[0])) {
    switch (s[0])
      {
        case 'n':
        case 'N':
          *f *= 1.E-9;
          s++;
          break;
        case 'u':
        case 'U':
          *f *= 1.E-6;
          s++;
          break;
        case 'm':
          *f *= 1.E-3;
          s++;
          break;
        case 'k':
        case 'K':
          *f *= 1.E3;
          s++;
          break;
        case 'M':
          *f *= 1.E6;
          s++;
          break;
        case 'g':
          *f *= 1.E9;
          s++;
          break;
        case 'G':
          *f *= 1.E9;
          s++;
          break;
      }
  }
  return s;
}

char *parseFlt( char *s, float *p ) {
  char *endptr = s;
  float f;
  if (s && *s) {
    f = strtof( s, &endptr );
    if (endptr > s) {
      if (p) *p = f;
      endptr = scaling(p,endptr);
      return endptr;
    }
  }
  return 0;
}

unsigned int parseUints( char *pc, unsigned int *p, unsigned int nmax ) {
  unsigned int n = 0;
  while( n < nmax && (pc = parseUint( pc, p ) ) ) {
    n++;
    p++;
  }
  return n;
}

unsigned int parseUint32s( char *pc, uint32_t *p, unsigned int nmax ) {
  unsigned int n = 0;
  while( n < nmax && (pc = parseUint32( pc, p ) ) ) {
    n++;
    p++;
  }
  return n;
}

unsigned int parseFlts( char *pc, float *p, unsigned int nmax ) {
  unsigned int n = 0;
  while( n < nmax && (pc = parseFlt( pc, p ) ) ) {
    n++;
    p++;
  }
  return n;
}

// parse each word as usecs, else as float seconds, convert to usecs
unsigned int parseUsecs( char *pc, uint32_t *p, unsigned int nmax ) {
  char *pc1 = pc;
  uint32_t ptemp = 0;
  float ftemp = 0;
  unsigned int n = 0;
  while( n < nmax && pc && pc[0] ) {
    pc1 = parseUint32(pc,&ptemp);
    if (pc1 && pc1[0] && isspace(pc1[0])) {
      *p++ = ptemp;
      n++;
      pc = pc1;
    }
    else if (parseFlt(pc,&ftemp)) {
      *p++ = (uint32_t) (ptemp * 1.E6);
      n++;
      pc = pc1;
    }
    else {
      break;
    }
  }
  return n;
}

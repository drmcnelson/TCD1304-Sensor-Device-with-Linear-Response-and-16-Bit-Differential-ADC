/* ====================================================================
   Parse srings for words, numbers, etc

   Apart from wordLength(), these return a pointer to the
   next character in the string, or null if they fail
   
 */

#include "Arduino.h"

#include <ctype.h>
#include <limits.h>

#include "parselib2.h"

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

// return next character after end of word
char *eow(char *s)
{
  while ( s && *s && !isspace(*s) ) s++;
  return s;
}

// return next character after end of spaces
char *eos(char*s)
{
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

char *nextWord( char *s )
{
  // skip leading non-space characters
  while ( s  && *s && !isspace(*s) ) {
    s++;
  }
  // skip over space characters
  while ( s  && *s && isspace(*s) ) {
    s++;
  }
  if ( s && *s ) {
    return s;
  }
  return 0;   
}

char *basenamef(const char *cs)
{
  char *s = (char *) cs;
  if (strchr(cs,'/')) {
    return strrchr(cs, '/')+1;
  }
  else if (strchr(cs,'\\')) {
    return strrchr(cs, '\\')+1;
  }
  else {
    return s;
  }
}

unsigned int countWords( char *s )
{
  unsigned int n = 0;
  if ( s && *s && !isspace(*s) ) {
    n++;
  }
  while( (s=nextWord(s)) ) {
    n++;
  }
  return n;
}

bool strMatch(const char *cs, const char *key, char **next)
{
  unsigned int n,m;
  char *s = (char *)cs;

  if (!s || !key || !(*s) || !(*key)) return false;

  while((*s) && isspace(*s)) s++;
  if (!(*s)) return false;

  n = strlen(key);
  m = strlen(s);
  if ((m < n) || strncmp(s,key,n)) return false;

  if (next) {
    *next = s+n;
  }

  return true;
}
  
bool strBool(const char *cs, bool *b, char **next)
{
  char *s = (char *) cs;
  char c = 0;
  s = nextWord(s);
  if (s) {
    c = tolower(*s);
    if ((c == 'y')||(c == 't')||(c == '1')) {
      if (b) *b = true;
      if (next) *next = eow(s);
      return true;
    }
    if ((s[0] == 'n')||(s[0] == 'f')||(s[0] == '0')) {
      if (b) *b = false;
      if (next) *next = eow(s);
      return true;
    }
  }
  return false;
}

bool strUint8lim(const char *cs, uint8_t *u, char **next, uint8_t ulim)
{
  char *s = (char *) cs;
  unsigned long int l;
  char *p = s;
  if ( p && *p ) {
    l = strtoul( s, &p, 0 );
    if ( (p > s) && (l <= ulim) ) {
      if (*p && !isspace(*p)) return false;
      if (u) *u = (uint8_t) l;
      if (next) *next = p;
      return true;
    }
  }
  return false;
}

bool strUint8(const char *cs, uint8_t *u, char **next)
{
  char *s = (char *) cs;
  unsigned long int l;
  char *p = s;
  if ( p && *p ) {
    l = strtoul( s, &p, 0 );
    if ( (p > s) && (l <= 0xFF) ) {
      if (*p && !isspace(*p)) return false;
      if (u) *u = (uint8_t) l;
      if (next) *next = p;
      return true;
    }
  }
  return false;
}

bool strUint(const char *cs, unsigned int *u, char **next)
{
  char *s = (char *) cs;
  unsigned long int l;
  char *p = s;
  if ( p && *p ) {
    l = strtoul( s, &p, 0 );
    if ( (p > s) && (l <= UINT_MAX) ) {
      if (*p && !isspace(*p)) return false;
      if (u) *u = (unsigned int) l;
      if (next) *next = p;
      return true;
    }
  }
  return false;
}

bool strUint16(const char *cs, uint16_t *u, char **next)
{
  char *s = (char *) cs;
  unsigned long int l;
  char *p = s;
  if ( p && *p ) {
    l = strtoul( s, &p, 0 );
    if ( (p > s) && (l <= UINT16_MAX) ) {
      if (*p && !isspace(*p)) return false;
      if (u) *u = (uint16_t) l;
      if (next) *next = p;
      return true;
    }
  }
  return false;
}

bool strUint32(const char *cs, uint32_t *u, char **next)
{
  char *s = (char *) cs;
  unsigned long int l;
  char *p = s;
  if ( p && *p ) {
    l = strtoul( s, &p, 0 );
    if ( (p > s) && (l <= UINT32_MAX) ) {
      if (*p && !isspace(*p)) return false;
      if (u) *u = (uint32_t) l;
      if (next) *next = p;
      return true;
    }
  }
  return false;
}

// this is for internal use
char *scaling( float *f, char *s)
{
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

bool strFlt(const char *cs, float *p, char **next)
{
  char *s = (char *) cs;
  char *endptr = s;
  float f;
  if (s && *s) {
    f = strtof( s, &endptr );
    if (endptr > s) {
      if (p) *p = f;
      endptr = scaling(p,endptr);
      if (next) *next = endptr;
      return true;
    }
  }
  return false;
}

unsigned int strUints(const char *cs, unsigned int *p, unsigned int nmax, char **next)
{
  char *s = (char *) cs;
  unsigned int n = 0;
  while( n < nmax && strUint(s, p, &s) ) {
    n++;
    p++;
  }
  if (n && next) *next = s;
  return n;
}

unsigned int strUint32s(const char *cs, uint32_t *p, unsigned int nmax, char **next)
{
  char *s = (char *) cs;
  unsigned int n = 0;
  while( n < nmax && strUint32(s, p, &s) ) {
    n++;
    p++;
  }
  if (n && next) *next = s;
  return n;
}

unsigned int strFlts(const char *pc, float *p, unsigned int nmax, char **next)
{
  char *s = (char *) pc;
  unsigned int n = 0;
  while( n < nmax && strFlt(s, p, &s) ) {
    n++;
    p++;
  }
  if (n && next) *next = s;
  return n;
}


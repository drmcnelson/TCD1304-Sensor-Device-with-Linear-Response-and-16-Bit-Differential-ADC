/* ====================================================================
   Parse srings for words, numbers, etc

   Apart from wordLength(), these return a pointer to the
   next character in the string, or null if they fail
   
*/

#ifndef PARSELIB_h
#define PARSELIB_H

/*
#ifdef __cplusplus
extern "C" {
#endif
*/

int serialPrintf( const char *fmt, ... );
int serialPrintlnf( const char *fmt, ... );

unsigned int wordLength( char *s );

char *nextWord( char *s );

unsigned int countWords( char *s );
  
char *startsWith( char *s, const char *key );

char *parseBool(char *s, bool *b);

char *parseUint8( char *s, uint8_t *u, uint8_t ulim=0xFF );
//char *parseUint8( char *s, uint8_t *u );

char *parseUint( char *s, unsigned int *u );
  
char *parseUint16( char *s, uint16_t *u );
  
char *parseUint32( char *s, uint32_t *u );
  
char *parseFlt( char *s, float *p );

unsigned int parseUints( char *pc, unsigned int *p, unsigned int nmax );
  
unsigned int parseUint32s( char *pc, uint32_t *p, unsigned int nmax );
  
unsigned int parseFlts( char *pc, float *p, unsigned int nmax );
  
// parse each word as usecs, else as float seconds, convert to usecs
unsigned int parseUsecs( char *pc, uint32_t *p, unsigned int nmax );

/*
  #ifdef __cplusplus
  }
  #endif
*/

#endif

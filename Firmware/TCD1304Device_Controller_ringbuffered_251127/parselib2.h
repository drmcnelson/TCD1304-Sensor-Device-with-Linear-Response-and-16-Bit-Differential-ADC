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

char *basenamef(const char *cs);

bool strMatch(const char *s, const char *key, char **next);

bool strBool(const char *s, bool *b, char **next=0);

bool strUint8(const char *s, uint8_t *u, char **next );

bool strUint8lim(const char *s, uint8_t *u, char **next, uint8_t ulim);

bool strUint(const char *s, unsigned int *u, char **next);
  
bool strUint16(const char *s, uint16_t *u, char **next);
  
bool strUint32(const char *s, uint32_t *u, char **next);
  
bool strFlt(const char *s, float *p, char **next);

unsigned int strUints(const char *pc, unsigned int *p, unsigned int nmax, char **next);
  
unsigned int strUint32s(const char *pc, uint32_t *p, unsigned int nmax, char **next);
  
unsigned int strFlts(const char *pc, float *p, unsigned int nmax, char **next);
  
/*
  #ifdef __cplusplus
  }
  #endif
*/

#endif

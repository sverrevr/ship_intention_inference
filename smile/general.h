#ifndef DSL_GENERAL_H
#define DSL_GENERAL_H

// {{SMILE_PUBLIC_HEADER}}

#include <string>
#include <cstdio>
#include <ctime>
#include <cmath>
#include "constants.h"
#include "errors.h"

#define DSL_FALSE             0
#define DSL_TRUE              1

#define DSL_FILE_LINE_LENGTH 1021
#define DSL_FILE_TOKEN_LENGTH (10 * DSL_FILE_LINE_LENGTH)

#define DSL_NUMBER_PRECISION 8
#define DSL_EPSILON    0.000005 

#if defined(_WIN32)
	#define DSL_stricmp _stricmp
	#define DSL_strnicmp _strnicmp
	inline bool DSL_isnan(double x) { return _isnan(x) != 0; }
#else
	#define DSL_stricmp strcasecmp
	#define DSL_strnicmp strncasecmp
	inline bool DSL_isnan(double x) { return std::isnan(x) != 0; }
#endif

FILE* DSL_fopen(const char *filename, const char *mode);

time_t DSL_time(time_t *);
clock_t DSL_clock();

inline bool DSL_isFinite(double x)
{
	return !DSL_isnan(x) && x <= DBL_MAX && x >= -DBL_MAX; 
}

double DSL_nan();

bool DSL_isCharValidForIdentifier(char c, bool start);
bool DSL_isValidIdentifier(const char *id);
bool DSL_makeValidId(char *id);
bool DSL_nameToIdentifier(std::string &s, int outcomeIndex);

int DSL_stringToDouble(const char *theString, double &here);
int DSL_doubleToString(double theNumber, char *here, int precission = DSL_NUMBER_PRECISION);
int DSL_intToString(int theNumber, char *here);
int DSL_stringToInt(char *theString, int &here);
void DSL_appendInt(std::string &s, int x);
void DSL_appendDouble(std::string &s, double x);

#endif // DSL_GENERAL_H

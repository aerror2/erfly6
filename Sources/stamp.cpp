#include "er9x.h"
//#include "stamp-er9x.h"

#define STR2(s) #s
#define DEFNUMSTR(s)  STR2(s)

#define DATE_STR "11.01.2022"
#define TIME_STR "23:15:02"
#define SUB_VERS  10
#define SVN_VERS "1.0.4"
#define MOD_VERS "FlySky-I6 Base"
#define AUTOR    "Kotello,Aerror"

const char Stamps[] = "VERS: V" DEFNUMSTR(VERS) "." DEFNUMSTR(SUB_VERS) " -" AUTOR "\037"\
" MOD: " MOD_VERS;



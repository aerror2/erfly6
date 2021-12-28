#include "er9x.h"
//#include "stamp-er9x.h"

#define STR2(s) #s
#define DEFNUMSTR(s)  STR2(s)

#define DATE_STR "29.10.2019"
#define TIME_STR "08:15:02"
#define SUB_VERS  10
#define SVN_VERS "1.0.3"
#define MOD_VERS "FlySky-I6 Base"
#define AUTOR    "Kotello"

const char Stamps[] = "VERS: V" DEFNUMSTR(VERS) "." DEFNUMSTR(SUB_VERS) " -" AUTOR "\037"\
" MOD: " MOD_VERS;



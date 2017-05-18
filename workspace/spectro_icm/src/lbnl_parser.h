#ifndef LBNL_PARSE_H_
#define LBNL_PARSE_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include "lbnl_typedefs.h"

#define CANT_OPEN_FILE -1001
#define INVALID_NUM_VALUE -1002
#define ENV_VAR_DOESNT_EXIST -1003

#define PARSER_LINE_LENGTH 80


int parse(char* filename, bool isInt, u16* ivals, f32* fvals);

#endif /*LBNL_PARSE_H_*/

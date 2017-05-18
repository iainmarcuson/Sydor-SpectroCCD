#include "lbnl_parser.h"

bool iswhitespace(char* line) {
  int len = strlen(line);
  int i = 0;
  for (; i < len; i++) {
    if (! isspace(line[i])) {
      return false;
    }
  }
  return true;
}

int parse(char* filename, bool isInt, u16* ivals, f32* fvals) {
  FILE* fp;
  fp = fopen(filename, "r");
  if (fp == NULL) {
    return CANT_OPEN_FILE;
  }

  char line[PARSER_LINE_LENGTH];
  int i = 0;
  while (fgets(line, PARSER_LINE_LENGTH, fp)) {

    //remove comment if it exists
    char* comment;
    comment = strchr(line, '#');
    if (comment != NULL) {
      comment[0] = '\0';
    }

    //skip empty lines
    if (iswhitespace(line)) {
      continue;
    }

    char* str_end;
    if (isInt) {
      u16 val = (u16) strtol(line, &str_end, 10);
      ivals[i++] = val;
    } else {
      f32 val = strtof(line, &str_end);
      fvals[i++] = val;
    }

    //if strtof/strtol couldn't parse a number or there is non-whitespace
    //stuff left over
    if (*str_end != '\0' && !isspace(*str_end)) {
  	  return INVALID_NUM_VALUE;
    }

  }

  fclose(fp);
  return 0;
}

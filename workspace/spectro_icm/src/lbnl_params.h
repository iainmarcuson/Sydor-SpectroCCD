#ifndef LBNL_PARAMS_H
#define LBNL_PARAMS_H
#include "lbnl_typedefs.h"

char *lbnl_param_to_string(const struct param_struct *param);
char *lbnl_param_to_packed(const struct param_struct *param, int *data_length);

#endif

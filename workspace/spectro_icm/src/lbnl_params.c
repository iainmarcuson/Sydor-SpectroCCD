#include "lbnl_typedefs.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static int concat_float(char *dest, float *array, int num_elem, int width, char *format_string, char *separator)
{
  int idx_val;
  char *curr_str;

  curr_str = malloc(sizeof(char)*width);

  if (curr_str == NULL)
    {
      return 1;
    }
  
  for (idx_val = 0; idx_val < num_elem; idx_val++)
    {
      sprintf(curr_str, format_string, array[idx_val]);
      strcat(dest, curr_str);
      if (idx_val != (num_elem-1))
	{
	  strcat(dest,separator);
	}

    }

  free(curr_str);

  return 0;
}

static int concat_int(char *dest, i32 *array, int num_elem, int width, char *format_string, char *separator)
{
  int idx_val;
  char *curr_str;

  curr_str = malloc(sizeof(char)*width);

  if (curr_str == NULL)
    {
      return 1;
    }
  
  for (idx_val = 0; idx_val < num_elem; idx_val++)
    {
      sprintf(curr_str, format_string, array[idx_val]);
      strcat(dest, curr_str);
      if (idx_val != (num_elem-1))
	{
	  strcat(dest,separator);
	}
    }

  free(curr_str);

  return 0;
}

char *lbnl_param_to_string(const struct param_struct *param)
{
  char *output_string=NULL;		/* Full output string of all values */
  char *curr_num_str=NULL;		/* String for current value */
  int val_idx;			/* Index of current value to process */
  char *desc_string=NULL;		/* String for concatenated descriptions */
  char *hi_string=NULL;		/* String for concatenated high values */
  char *lo_string=NULL;		/* String for concatenated low values */
  char *val_string=NULL;		/* String for concatenated values */
  int i,j,k;			/* Index variables */
  int errors=0;			/* Holds return status for errors that may occur */
  int out_string_len = 0;	/* Holds the length of the final string */

  //First, create string for all descriptions
  desc_string = calloc(1, param->num_elem*(EPICS_STRLEN+1)+2); /* Account for nulls and comma */
  //TODO XXX FIXME Account for failure
  
  for (val_idx = 0; val_idx < param->num_elem; val_idx++)
    {
      strcat(desc_string, param->desc[val_idx]);
      if (val_idx != (param->num_elem-1)) /* don't put a comma after the last entry */
	{
	  strcat(desc_string, ",");
	}
    }

  printf("Concatenated desc string is: %s.\n", desc_string);

  //Next, create strings for the data, low, and high values
  val_string = calloc(1, (param->val_len+1)*param->num_elem+1);
  lo_string = calloc(1, (param->val_len+1)*param->num_elem+1);
  hi_string = calloc(1, (param->val_len+1)*param->num_elem+1); /* Account for comma and terminator */
  if (param->param_type == TYPE_F32)
    {
      errors += concat_float(lo_string, param->lo_array.f, param->num_elem, param->val_len, param->format, ",");
      errors += concat_float(hi_string, param->hi_array.f, param->num_elem, param->val_len, param->format, ",");
      errors += concat_float(val_string, param->val_array.f, param->num_elem, param->val_len, param->format, ",");
      //XXX TODO handle errors
    }
  else if (param->param_type == TYPE_I32)
    {
     errors += concat_int(lo_string, param->lo_array.i, param->num_elem, param->val_len, param->format, ",");
     errors += concat_int(hi_string, param->hi_array.i, param->num_elem, param->val_len, param->format, ",");
     errors += concat_int(val_string, param->val_array.i, param->num_elem, param->val_len, param->format, ",");
    }

  printf("Concatenated lo string is: %s\n", lo_string);
  printf("Concatenated hi string is: %s\n", hi_string);
  printf("Concatenated val string is: %s\n", val_string);

  out_string_len = strlen(desc_string)+2+strlen(val_string)+2+strlen(lo_string)+2+strlen(hi_string)+2+1; /* +2 for \r\n */

  printf("Output string length is %i.\n", out_string_len);

  output_string = calloc(sizeof(char), out_string_len);
  strcat(output_string, desc_string);
  strcat(output_string, "\r\n");
  strcat(output_string, val_string);
  strcat(output_string, "\r\n");
  strcat(output_string, lo_string);
  strcat(output_string, "\r\n");
  strcat(output_string, hi_string);
  strcat(output_string, "\r\n");

  printf("The calculated output string is: %s\n", output_string);
  free(desc_string);
  free(val_string);
  free(lo_string);
  free(hi_string);
  
  return output_string;

}

char *lbnl_param_to_packed(const struct param_struct *param, int *data_length)
{
	char *output_string=NULL;		/* Full output string of all values */
	char *curr_num_str=NULL;		/* String for current value */
	  int val_idx;			/* Index of current value to process */
	  char *desc_string=NULL;		/* String for concatenated descriptions */
	  float *hi_string=NULL;		/* String for concatenated high values */
	  float *lo_string=NULL;		/* String for concatenated low values */
	  float *val_string=NULL;		/* String for concatenated values */
	  int i,j,k;			/* Index variables */
	  int errors=0;			/* Holds return status for errors that may occur */
	  int out_string_len = 0;	/* Holds the length of the final string */
	  int desc_str_ptr = 0;	/* Where we are in the packed null-separated string */
	  int curr_str_len = 0; /* The length of the current description string */
	  int total_str_len = 0; /* The length of the full null-separated description string */
	  int final_str_ptr = 0; /* Pointer for the parts of the final packed string */
	  //First, create string for all descriptions
	  desc_string = calloc(1, param->num_elem*(EPICS_STRLEN+1)+2); /* Account for nulls and comma */
	  //TODO XXX FIXME Account for failure

	  //Have to separate the strings with nulls, which messes up string functions
	  for (val_idx = 0; val_idx < param->num_elem; val_idx++)
	    {
	      strcpy(desc_string+desc_str_ptr, param->desc[val_idx]);
	      total_str_len += strlen(param->desc[val_idx])+1;
	      desc_str_ptr = total_str_len;
	    }

	  //printf("Concatenated desc string is: %s.\n", desc_string);

	  //Next, create buffers for binary packed data
	  //f32 indicates that we are using a 32-bit size
	  val_string = calloc(1, sizeof(f32)*param->num_elem);
	  lo_string = calloc(1, sizeof(f32)*param->num_elem);
	  hi_string = calloc(1, sizeof(f32)*param->num_elem); /* Account for comma and terminator */

	  //XXX DEBUGGING
	  printf("hi_string is currently %p\n", hi_string);

	  //Copy the data over XXX Typepunning
	  memcpy(val_string, param->val_array.f, sizeof(f32)*param->num_elem);
	 memcpy(lo_string, param->lo_array.f, sizeof(f32)*param->num_elem);
	  memcpy(hi_string, param->hi_array.f, sizeof(f32)*param->num_elem);
	  //TODO Delete this when I get stuff checked into Perforce
	  //if (param->param_type == TYPE_F32)
	  //  {
	  //    errors += concat_float(lo_string, param->lo_array.f, param->num_elem, param->val_len, param->format, ",");
	  //    errors += concat_float(hi_string, param->hi_array.f, param->num_elem, param->val_len, param->format, ",");
	  //    errors += concat_float(val_string, param->val_array.f, param->num_elem, param->val_len, param->format, ",");
	  //    //XXX TODO handle errors
	   // }
	  //else if (param->param_type == TYPE_I32)
	  //  {
	  //   errors += concat_int(lo_string, param->lo_array.i, param->num_elem, param->val_len, param->format, ",");
	  //   errors += concat_int(hi_string, param->hi_array.i, param->num_elem, param->val_len, param->format, ",");
	  //   errors += concat_int(val_string, param->val_array.i, param->num_elem, param->val_len, param->format, ",");
	   // }

	  //printf("Concatenated lo string is: %s\n", lo_string);
	  //printf("Concatenated hi string is: %s\n", hi_string);
	  //printf("Concatenated val string is: %s\n", val_string);

	  out_string_len = total_str_len+3*(sizeof(f32)*param->num_elem); /* Concatenated descriptions plus val, lo, hi */

	  printf("Output string length is %i.\n", out_string_len);

	  final_str_ptr = 0;
	  output_string = calloc(sizeof(char), out_string_len);
	  memcpy(output_string+final_str_ptr, desc_string, total_str_len);
	  final_str_ptr += total_str_len;
	  memcpy(output_string+final_str_ptr, val_string, (sizeof(f32)*param->num_elem));
	  final_str_ptr += (sizeof(f32)*param->num_elem);
	  memcpy(output_string+final_str_ptr, lo_string, (sizeof(f32)*param->num_elem));
	  final_str_ptr += (sizeof(f32)*param->num_elem);
	  memcpy(output_string+final_str_ptr, hi_string, (sizeof(f32)*param->num_elem));
	  final_str_ptr += (sizeof(f32)*param->num_elem);

	  *data_length = out_string_len;
	  //printf("The calculated output string is: %s\n", output_string);
	  //XXX DEBUGGING
	  	  	  //printf("hi_string is currently %p\n", hi_string);
	  	  free(hi_string);
	  	  //printf("Freed hi_string\n");
	  free(desc_string);
	 // printf("Freed desc_string\n");
	  free(val_string);
	 //printf("Freed val_string\n");
	  free(lo_string);
	 // printf("Freed lo_string\n");


	 // printf("Freed temporary buffers.\n");

	  return output_string;
}

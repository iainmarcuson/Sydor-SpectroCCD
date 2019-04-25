#ifndef LBNL_AUX_H		/* Inclusion guard */
#define LBNL_AUX_H

int lbnl_controller_read_aux(dref fd, u32 *ret_val);
int lbnl_controller_read_gpio(dref fd, u32 reg, u32 *ret_val);

#endif	/* End inclusion guard */

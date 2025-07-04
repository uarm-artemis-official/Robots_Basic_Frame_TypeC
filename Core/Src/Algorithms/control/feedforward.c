/*******************************************************************************
* @file           : feedforward.c
* @brief          : Use feedforward control to improve accuracy of the response of the object
*                   based on input value
* @created time	  : Aug, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/


#ifndef __FEEDFORWARD_C__
#define __FEEDFORWARD_C__

#include "feedforward.h"

/* define internal vars */
/* define internal functions */

/**
  * @brief    feed forward control init
  * @param[in] ff: pointer to feed forward control struct
  * @param[in] kf: ff_gain value
  */
void ff_param_init(FeedForward_t *ff, float kf){
	ff->ff_gain = kf;
	ff->last_input = 0;
	ff->output = 0;
}
/**
  * @brief     First-order feedforward control approximation to compensate
  * 		   for the possible lag effect of pid
  * @param[in] input: input value, usually be the target value
  * @retval    feedforwad prediction value
  *
  * Understanding: Gff(S) = 1 / (Gc_fb(s))
  *
  */
float feedforward(FeedForward_t *ff, float input){
	ff->output = (input - ff->last_input) * ff->ff_gain + input;
	ff->last_input = input;
	return ff->output;
}

#endif /*__FEEDFORWARD_C__ */

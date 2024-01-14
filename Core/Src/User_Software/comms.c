/*******************************************************************************
* @file           : comms.c
* @brief          : communication related functions
* @created time	  : Aug, 2023
* @author         : Haoran
*
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __COMMS_C__
#define __COMMS_C__

#include "comms.h"
#include "main.h"
#include "Control_App.h"

/********************************************************************************************
 * Communication Messages Subcription System
 *
 *  	As we have more and more communication messages, a msg subscription managment system
 *  is needed. This file basically modelizes the communication massage system, and provides
 *  a simple interface to subscribe and unsubscribe messages for each dev board.
 *
 *  Usage:
 *   // subscribe a msg from gimbal board as a receiver
 *   comm_subscribe(&gimbal_comm->sublist, COMM_REMOTE_CONTROL, Receiver);

*********************************************************************************************/
/* define vars here */
//Comm_t comm_pack;

/* declare all used message */
/* init comm for gimbal angle info */
void gimbal_angle_message_init(CommMessageUnion_t *cmu){
	for(uint8_t i=0;i<4;i++)
		cmu->comm_ga.angle_data[i] = 0.0f;
	cmu->comm_ga.send_flag = 0;//reset flag
}

CommMessage_t gimbal_angle_message = {
    .message_type = COMM_GIMBAL_ANGLE,
    .init = gimbal_angle_message_init
};

/* init comm for rc info */
void rc_message_init(CommMessageUnion_t *cmu){
	for(uint8_t i=0;i<2;i++)
		cmu->comm_rc.rc_data[i] = 0;
	cmu->comm_rc.rc_data[2] = SW_MID;
	cmu->comm_rc.rc_data[3] = SW_MID;
	cmu->comm_rc.send_flag = 0;//reset flag
}

CommMessage_t rc_message = {
    .message_type = COMM_REMOTE_CONTROL,
    .init = rc_message_init
};

/* init comm for pc info */
void pc_message_init(CommMessageUnion_t *cmu){
	for(uint8_t i=0;i<2;i++)
		cmu->comm_pc.pc_data[i] = 0;
	cmu->comm_pc.pc_data[2] = RELEASED;
	cmu->comm_pc.pc_data[3] = RELEASED;
	cmu->comm_pc.send_flag = 0;//reset flag
}

CommMessage_t pc_message = {
    .message_type = COMM_PC_CONTROL,
    .init = pc_message_init
};

void pc_ext_message_init(CommMessageUnion_t *cmu){
	for(uint8_t i = 0;i<3; i++)
		cmu->comm_ext_pc.pc_data[i] = RELEASED;
	cmu->comm_ext_pc.send_flag = 0;//reset flag
}

CommMessage_t pc_add_message = {
    .message_type = COMM_EXT_PC_CONTROL,
    .init = pc_ext_message_init
};


/* init comm for referee info */
void ref_message_init(CommMessageUnion_t* cmu){
	/* not implement yet */
	return;
}

CommMessage_t ref_message = {
    .message_type = COMM_REFEREE,
    .init = ref_message_init
};

/* init comm for computer vision massage info */
void vision_message_init(CommMessageUnion_t* cmu){
	/* not implement yet */
	return;
}

CommMessage_t vision_message = {
    .message_type = COMM_VISION,
    .init = vision_message_init
};

/*
 * @brief 	  comm message subscribe function
 * @param[in] sub: comm message subscribe list
 * @param[in] msgType: comm message type
 * @retval    None
 */
void comm_subscribe(CommMessageSublist_t *sub, CommMessageType_t msgType, CommRole_t role) {
    /* update subscription list */
    sub->sub_list |= msgType;//use mask to set bit
    sub->sub_list_num++;			//total num plus one

    /* call the appropriate init function */
    switch (msgType) {
        case COMM_GIMBAL_ANGLE:
        	gimbal_angle_message.role = role;
            gimbal_angle_message.init(&(gimbal_angle_message.message));
            break;
        case COMM_REMOTE_CONTROL:
        	rc_message.role = role;
            rc_message.init(&(rc_message.message));
            break;
        case COMM_PC_CONTROL:
			pc_message.role = role;
			pc_message.init(&(pc_message.message));
			break;
        case COMM_EXT_PC_CONTROL:
        	pc_add_message.role = role;
        	pc_add_message.init(&(pc_add_message.message));
        	break;
        case COMM_REFEREE:
        	ref_message.role = role;
            ref_message.init(&(ref_message.message));
            break;
        case COMM_VISION:
        	vision_message.role = role;
            vision_message.init(&(vision_message.message));
            break;
    }
}

CommSubscribeStatus_t isSubscribed(CommMessageSublist_t *sub, CommMessageType_t msgType){
     if((sub->sub_list & msgType) != 0)
    	 return SUB_SUCCESS;
     else
    	 return SUB_FAIL;
}

/*
 * @brief 	  comm message unsubscribe function
 * @param[in]
 * @retval    None
 */
void comm_unsubscribe(CommMessageSublist_t *sub, CommMessageType_t msgType){
    sub->sub_list &= ~msgType;
}




#endif /* __COMMS_C__*/

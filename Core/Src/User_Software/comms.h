/*******************************************************************************
* @file           : comms.h
* @brief          : communication related functions
* 					mini PC
* @created time	  : Aug, 2023
* @author         : Haoran
*
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __COMMS_H__
#define __COMMS_H__

#include <referee.h>

#define MAX_COMMS_MODULE 5

/* define the structures */
/* enum for different message types (assuming you've defined it) */
typedef enum {
    COMM_GIMBAL_ANGLE = 1 << 0,    // 0b00001
    COMM_REMOTE_CONTROL = 1 << 1,  // 0b00010
	COMM_PC_CONTROL = 1 << 2,      // 0b00100
    COMM_REFEREE = 1 << 3,         // 0b01000
    COMM_VISION = 1 << 4,          // 0b10000
	COMM_EXT_PC_CONTROL = 1 << 5,  // 0b100000

    							   // add more here if needed
} CommMessageType_t;

typedef enum {
	SUB_SUCCESS = 0,
	SUB_FAIL = 1,
}CommSubscribeStatus_t;

typedef enum{
	Receiver = 0,
	Transmitter = 1
}CommRole_t;

/* communication packs */
/* vision pack */
typedef struct {
	uint8_t auto_aim_flag; //indicate whether na object detected
	int32_t yaw_data;	   //yaw delta angle
	int32_t pitch_data;    //pitch delta angle
	int32_t dist_data;	   //distance between the camera and the object
	int32_t target_num;	   //number on the target  armor plate
	int32_t pack_cond;     //packet condition - correct or not
}CommVision_t;

/* follow gimbal pack -> gimbal to chassis */
typedef struct{
	float angle_data[4]; // ecd_rel_angle,abs_abg,idle,idle
	uint8_t send_flag;
}CommGimbalAngle_t;

/* remote controller pack */
typedef struct{
	int16_t rc_data[4]; //ch2,ch3,s1,s2
	uint8_t send_flag;
}CommRemoteControl_t;

/* remote pc pack */
typedef struct{
	int16_t pc_data[4]; //mouse_x,mouse_y,left click,right click
	uint8_t send_flag;
}CommPCControl_t;

typedef struct{
	int16_t pc_data[3]; //switch to shoot mode, 180 degree turn around, switch barrel
	uint8_t send_flag;
}CommExtPCControl_t;

/* referee info   -> chassis to gimbal */
typedef struct{
	game_status_t 	  game_status_data;
	game_robot_HP_t   HP_data;
	robot_status_t 	  robot_status_data;
	power_heat_data_t power_heat_data;
	shoot_data_t      shoot_data;
}CommReferee_t;


/* define comm message union */
typedef union {
	/* relative angle -> gimbal to chassis */
	CommGimbalAngle_t comm_ga;
	/* remote controller -> chassis to gimbal */
	CommRemoteControl_t comm_rc;
	/* pc control -> chassis to gimbal */
	CommPCControl_t comm_pc;
	/* referee info   -> chassis to gimbal */
	CommReferee_t comm_ref;
	/* vision info    -> chassis to gimbal */
	CommVision_t vision;
	/* pc control -> chassis to gimbal */
	CommExtPCControl_t comm_ext_pc;
}CommMessageUnion_t;

/* Abstract communication module interface */
typedef struct {
    CommMessageType_t message_type;
    CommRole_t role;
    CommMessageUnion_t message;
    void (*init)(CommMessageUnion_t* cmu);
}CommMessage_t;

typedef struct {
	uint16_t sub_list; //sub list mask
	uint16_t sub_list_num; //sub list number
}CommMessageSublist_t;

/* temp use comm struct */
typedef struct{
	/* relative angle -> gimbal to chassis */
	CommGimbalAngle_t comm_ga;
	/* remote control -> chassis to gimbal */
	CommRemoteControl_t comm_rc;
	/* referee info   -> chassis to gimbal */
	CommReferee_t comm_ref;
	/* vision info    -> chassis to gimbal */
	CommVision_t vision;
}Comm_t;//now not used

/* define externs var */
//extern Comm_t comm_pack;
extern CommMessage_t gimbal_angle_message;
extern CommMessage_t rc_message;
extern CommMessage_t pc_message;
extern CommMessage_t ref_message;
extern CommMessage_t vision_message;


/* declare functions here*/
void gimbal_angle_message_init(CommMessageUnion_t *cmu);
void rc_message_init(CommMessageUnion_t *cmu);
void ref_message_init(CommMessageUnion_t* cmu);
void vision_message_init(CommMessageUnion_t *cmu);
void comm_subscribe(CommMessageSublist_t *sub, CommMessageType_t msgType, CommRole_t role);
uint8_t isSubscribed(CommMessageSublist_t *sub, CommMessageType_t msgType);
void comm_unsubscribe(CommMessageSublist_t *sub, CommMessageType_t msgType);

#endif /* __COMMS_H__*/

/*******************************************************************************
* @file           : auto_aim.h
* @brief          : auto aim
* @created time	  : Oct, 2023
* @author         : Haoran Qi
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/


#ifndef __AUTO_AIM_H__
#define __AUTO_AIM_H__

/* define the parse states */
#define UC_RX_PACKLEN  12 // total length of the packet in bytes
#define UC_RX_DATALEN  10 // the end of the position of the used data in a pack in bytes
#define UC_RX_READ_START 1



/* rm_vision open-source pack data structures using CRC16 */
typedef struct{
  uint8_t header;
  uint8_t tracking;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float z_2;
  uint16_t checksum;
} RecvPacket_t;

typedef struct{
  uint8_t header;
  uint8_t robot_color : 1; // 0: red, 1: blue
  uint8_t task_mode : 2; // no use
  uint8_t reserved : 5;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum;
} SendPacket_t;

/* UARM rm_vision adapted pack, using checksum calibration*/
typedef struct{
	/* header */
	uint8_t header; // 1 byte

	/* auto aiming */
	float delta_yaw;	   // 4 bytes
	float delta_pitch;	   // 4 bytes
	int8_t target_num;    // 1 byte

	/* checksum calibration */
	uint16_t checksum;      // 2 bytes
}UC_Recv_Pack_t; //receive packet from upper computer

extern UC_Recv_Pack_t uc_rx_pack;
/* delace the auto aim related functions here */
void uc_rx_pack_init(UC_Recv_Pack_t *uc_pack);
uint16_t calculate_checksum(const char data[], size_t length);
uint8_t uc_parse_recv_packet(const char recv_pack[], UC_Recv_Pack_t *uc_pack);


#endif /*__AUTO_AIM_H__*/

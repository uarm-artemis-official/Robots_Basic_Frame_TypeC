/*******************************************************************************
* @file           : auto_aim.c
* @brief          : This is the new uto aim arch based on rm_vision python
* @created time	  : Oct, 2023
* @author         : Haoran Qi
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/


#ifndef __AUTO_AIM_C__
#define __AUTO_AIM_C__

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include "auto_aim.h"

/* define the global UC pack */
UC_Recv_Pack_t uc_rx_pack;

void uc_rx_pack_init(UC_Recv_Pack_t *uc_rx_pack){
	/* header */
	uc_rx_pack->header = 0;           // 1 byte

	/* auto aiming */
	uc_rx_pack->delta_yaw = 0.0;	   // 4 bytes
	uc_rx_pack->delta_pitch = 0.0;	   // 4 bytes
	uc_rx_pack->target_num=-1;       // 1 byte

	/* checksum calibration */
	uc_rx_pack->checksum= 0;           // 2 bytes
}

uint16_t calculate_checksum(const char data[], size_t length){
	uint16_t checksum = 0;
	for (size_t i = UC_RX_READ_START; i < length; i++){
		checksum += (uint8_t)data[i];  // Cast to unsigned char to ensure correct addition
	}
	checksum = checksum % 256;
	return checksum;  // Return a value in the range [0, 255]
}

uint8_t uc_parse_recv_packet(const char recv_pack[], UC_Recv_Pack_t *uc_rx_pack) {
	uc_rx_pack->header = recv_pack[0];
	if (uc_rx_pack->header != 0x1A) { // header was hard-coding
		return -1; //header failed, drop the pack
	}

	// assuming little-endian byte order
	uc_rx_pack->delta_yaw = *((float*)(recv_pack + 1));
	uc_rx_pack->delta_pitch = *((float*)(recv_pack + 5));
	uc_rx_pack->target_num = recv_pack[9];
	uc_rx_pack->checksum = (recv_pack[10] << 8) | recv_pack[11];

	// calculate and compare checksum
	uint16_t calculated_checksum = calculate_checksum(recv_pack, UC_RX_DATALEN);  // exclude the checksum field itself
	if (calculated_checksum != (uc_rx_pack->checksum % 256)) {
		return -1; // check failed, drop the pack
	}
	else {
		return 0; // check successfully
	}
}











































/* Copied from rm_vision open source lib
 * C++ base, need to modify it to C
 * Those two functions are actually the send and recv code
 * in PC end, we need to reverse it from our side*/

//void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Target::SharedPtr msg)
//{
//  try {
//    SendPacket packet;
//    packet.tracking = msg->tracking;
//    packet.x = msg->position.x;
//    packet.y = msg->position.y;
//    packet.z = msg->position.z;
//    packet.yaw = msg->yaw;
//    packet.vx = msg->velocity.x;
//    packet.vy = msg->velocity.y;
//    packet.vz = msg->velocity.z;
//    packet.v_yaw = msg->v_yaw;
//    packet.r1 = msg->radius_1;
//    packet.r2 = msg->radius_2;
//    packet.z_2 = msg->z_2;
//    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
//
//    std::vector<uint8_t> data = toVector(packet);
//
//    serial_driver_->port()->send(data);
//
//    std_msgs::msg::Float64 latency;
//    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
//    RCLCPP_INFO_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
//    latency_pub_->publish(latency);
//  } catch (const std::exception & ex) {
//    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
//    reopenPort();
//  }
//}
//
//void RMSerialDriver::receiveData()
//{
//  std::vector<uint8_t> header(1);
//  std::vector<uint8_t> data;
//  data.reserve(sizeof(ReceivePacket));
//
//  while (rclcpp::ok()) {
//    try {
//      serial_driver_->port()->receive(header);
//
//      if (header[0] == 0x5A) {
//        data.resize(sizeof(ReceivePacket) - 1);
//        serial_driver_->port()->receive(data);
//
//        data.insert(data.begin(), header[0]);
//        ReceivePacket packet = fromVector(data);
//
//        bool crc_ok =
//          crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
//        if (crc_ok) {
//          sensor_msgs::msg::JointState joint_state;
//          joint_state.header.stamp = this->now();
//          joint_state.name.push_back("pitch_joint");
//          joint_state.name.push_back("yaw_joint");
//          joint_state.position.push_back(packet.pitch);
//          joint_state.position.push_back(packet.yaw);
//          joint_state_pub_->publish(joint_state);
//
//          if (packet.aim_x > 0.01) {
//            aiming_point_.header.stamp = this->now();
//            aiming_point_.pose.position.x = packet.aim_x;
//            aiming_point_.pose.position.y = packet.aim_y;
//            aiming_point_.pose.position.z = packet.aim_z;
//            marker_pub_->publish(aiming_point_);
//          }
//        } else {
//          RCLCPP_ERROR(get_logger(), "CRC error!");
//        }
//      } else {
//        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
//      }
//    } catch (const std::exception & ex) {
//      RCLCPP_ERROR_THROTTLE(
//        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
//      reopenPort();
//    }
//  }
//}


#endif /*__AUTO_AIM_C__*/

/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "canard_driver.h"
#include "canard.h"
#include "uavcan/equipment/esc/Status.h"
#include "uavcan/equipment/esc/RawCommand.h"
#include "uavcan/equipment/esc/RPMCommand.h"
#include "uavcan/equipment/actuator/Command.h"
#include "uavcan/equipment/actuator/ArrayCommand.h"
#include "uavcan/equipment/actuator/Status.h"
#include "uavcan/equipment/actuator/SetLimits.h"
#include "uavcan/protocol/Panic.h"

#include "conf_general.h"
#include "app.h"
#include "comm_can.h"
#include "commands.h"
#include "mc_interface.h"
#include "hw.h"
#include "timeout.h"
#include "terminal.h"
#include "encoder.h"

// Constants
#define APP_NODE_NAME									"org.vesc." HW_NAME

#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE			((3015 + 7) / 8)
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE		0xee468a8121c46a9e
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID				1

#define UAVCAN_NODE_STATUS_MESSAGE_SIZE					7
#define UAVCAN_NODE_STATUS_DATA_TYPE_ID					341
#define UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE			0x0f0868d0c1a7c6f1

#define UAVCAN_NODE_HEALTH_OK							0
#define UAVCAN_NODE_HEALTH_WARNING						1
#define UAVCAN_NODE_HEALTH_ERROR						2
#define UAVCAN_NODE_HEALTH_CRITICAL						3

#define UAVCAN_NODE_MODE_OPERATIONAL					0
#define UAVCAN_NODE_MODE_INITIALIZATION					1

#define UNIQUE_ID_LENGTH_BYTES							16

#define STATUS_MSGS_TO_STORE							10

#define UAVCAN_PROTOCOL_PANIC_INTERVAL                  100 //ms

// Private datatypes
typedef struct {
	int id;
	systime_t rx_time;
	uavcan_equipment_esc_Status msg;
} status_msg_wrapper_t;

// Private variables
static CanardInstance canard;
static uint8_t canard_memory_pool[1024];
static uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
static uint8_t node_mode = UAVCAN_NODE_MODE_OPERATIONAL;
static int debug_level;
static status_msg_wrapper_t stat_msgs[STATUS_MSGS_TO_STORE];

// Threads
static THD_WORKING_AREA(canard_thread_wa, 2048);
static THD_FUNCTION(canard_thread, arg);

// Private functions
static void sendEscStatus(void);
static void readUniqueID(uint8_t* out_uid);
static void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE]);
static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);
static bool shouldAcceptTransfer(const CanardInstance* ins,
		uint64_t* out_data_type_signature,
		uint16_t data_type_id,
		CanardTransferType transfer_type,
		uint8_t source_node_id);
static void terminal_debug_on(int argc, const char **argv);

void canard_driver_init(void) {
	debug_level = 0;

	for (int i = 0;i < STATUS_MSGS_TO_STORE;i++) {
		stat_msgs[i].id = -1;
	}

	chThdCreateStatic(canard_thread_wa, sizeof(canard_thread_wa), NORMALPRIO, canard_thread, NULL);

	terminal_register_command_callback(
			"uavcan_debug",
			"Enable UAVCAN debug prints (0 = off)",
			"[level]",
			terminal_debug_on);
}

static void sendEscStatus(void) {
	uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];
	uavcan_equipment_esc_Status status;
	status.current = mc_interface_get_tot_current();
	status.error_count = mc_interface_get_fault();
	status.esc_index = app_get_configuration()->uavcan_esc_index;
	status.power_rating_pct = (fabsf(mc_interface_get_tot_current()) /
			mc_interface_get_configuration()->l_current_max *
			mc_interface_get_configuration()->l_current_max_scale) * 100.0;
	status.rpm = mc_interface_get_rpm();
	status.temperature = mc_interface_temp_fet_filtered() + 273.15;
	status.voltage = GET_INPUT_VOLTAGE();

	uavcan_equipment_esc_Status_encode(&status, buffer);

	static uint8_t transfer_id;

	canardBroadcast(&canard,
			UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
			UAVCAN_EQUIPMENT_ESC_STATUS_ID,
			&transfer_id,
			CANARD_TRANSFER_PRIORITY_LOW,
			buffer,
			UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE);
}

static void sendPanic( void ) {
    uint8_t buffer[UAVCAN_PROTOCOL_PANIC_MAX_SIZE];
    uavcan_protocol_Panic panic;
    uint8_t fault = (uint8_t) mc_interface_get_fault();
    if ( fault == FAULT_CODE_NONE ||
         fault == FAULT_CODE_UAVCAN_PANIC){
        return;
    }
    // We can only print two digit fault codes, which should be plenty!
    char fc[9];
    itoa( fault, fc, 10);
    char msg[7] = {'V', 'E', 'S','C','_',fc[0], fc[1]};
    panic.reason_text.len = 7;
    panic.reason_text.data = (uint8_t *)msg;
    uavcan_protocol_Panic_encode(&panic, buffer);
    static uint8_t transfer_id;
    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_PANIC_SIGNATURE,
                    UAVCAN_PROTOCOL_PANIC_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_HIGHEST,
                    buffer,
                    UAVCAN_PROTOCOL_PANIC_MAX_SIZE);
}

static void sendActuatorStatus(void) {
    uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];
    uavcan_equipment_actuator_Status status;
    status.actuator_id = app_get_configuration()->uavcan_esc_index;
    status.position = mc_interface_get_pid_pos_now() * M_PI / 180.0f; // this is mechanical angle corrected in [rad]
    status.force = mc_interface_get_tot_current_directional();        // this is in [A], you'll need the current constant for your motor here!
    status.speed = mc_interface_get_mech_rpm() * 2.0f * M_PI / 60.0f; // this is rotational speed in [rad/s]
    status.power_rating_pct = (fabsf(mc_interface_get_tot_current()) /
                               mc_interface_get_configuration()->l_current_max *
                               mc_interface_get_configuration()->l_current_max_scale) * 100.0;

    uavcan_equipment_actuator_Status_encode(&status, buffer);

    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
                    UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE);
}

static void readUniqueID(uint8_t* out_uid) {
	for (uint8_t i = 0; i < UNIQUE_ID_LENGTH_BYTES; i++) {
		out_uid[i] = i;
	}
}

static void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE]) {
	memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);
	const uint32_t uptime_sec = ST2S(chVTGetSystemTimeX());
	canardEncodeScalar(buffer,  0, 32, &uptime_sec);
	canardEncodeScalar(buffer, 32,  2, &node_health);
	canardEncodeScalar(buffer, 34,  3, &node_mode);
}

/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer) {
	if (debug_level > 0) {
		commands_printf("UAVCAN transfer RX: NODE: %d Type: %d ID: %d",
				transfer->source_node_id, transfer->transfer_type, transfer->data_type_id);
	}

	if ((transfer->transfer_type == CanardTransferTypeRequest) &&
			(transfer->data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID)) {

		uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE];
		memset(buffer, 0, sizeof(buffer));

		// NodeStatus
		makeNodeStatusMessage(buffer);

		// SoftwareVersion
		buffer[7] = FW_VERSION_MAJOR;
		buffer[8] = FW_VERSION_MINOR;
		buffer[9] = 1; // Optional field flags, VCS commit is set
		uint32_t u32 = 0;// GIT_HASH;
		canardEncodeScalar(buffer, 80, 32, &u32);
		// Image CRC skipped

		// HardwareVersion
		// Major skipped
		// Minor skipped
		readUniqueID(&buffer[24]);
		// Certificate of authenticity skipped

		// Name
		const size_t name_len = strlen(APP_NODE_NAME);
		memcpy(&buffer[41], APP_NODE_NAME, name_len);

		const size_t total_size = 41 + name_len;

		/*
		 * Transmitting; in this case we don't have to release the payload because it's empty anyway.
		 */
		canardRequestOrRespond(ins,
				transfer->source_node_id,
				UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE,
				UAVCAN_GET_NODE_INFO_DATA_TYPE_ID,
				&transfer->transfer_id,
				transfer->priority,
				CanardResponse,
				&buffer[0],
				(uint16_t)total_size);
	} else if ((transfer->transfer_type == CanardTransferTypeBroadcast) &&
			(transfer->data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID)) {
		uavcan_equipment_esc_RawCommand cmd;
		uint8_t buffer[UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_MAX_SIZE];
		memset(buffer, 0, sizeof(buffer));
		uint8_t *tmp = buffer;

		if (uavcan_equipment_esc_RawCommand_decode_internal(transfer, transfer->payload_len, &cmd, &tmp, 0, true) >= 0) {
			if (cmd.cmd.len > app_get_configuration()->uavcan_esc_index) {
				mc_interface_set_duty(((float)cmd.cmd.data[app_get_configuration()->uavcan_esc_index]) / 8192.0);
				timeout_reset();
			}
		}
	} else if ((transfer->transfer_type == CanardTransferTypeBroadcast) &&
			(transfer->data_type_id == UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_ID)) {
		uavcan_equipment_esc_RPMCommand cmd;
		uint8_t buffer[UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_MAX_SIZE];
		memset(buffer, 0, sizeof(buffer));
		uint8_t *tmp = buffer;

		if (uavcan_equipment_esc_RPMCommand_decode_internal(transfer, transfer->payload_len, &cmd, &tmp, 0, true) >= 0) {
			if (cmd.rpm.len > app_get_configuration()->uavcan_esc_index) {
				mc_interface_set_pid_speed(cmd.rpm.data[app_get_configuration()->uavcan_esc_index]);
				timeout_reset();
			}
		}
    } else if ((transfer->transfer_type == CanardTransferTypeBroadcast) &&
               (transfer->data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID)) {
        uavcan_equipment_actuator_ArrayCommand cmd;
        uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_MAX_SIZE];
        memset(buffer, 0, sizeof(buffer));
        uint8_t *tmp = buffer;
        if (uavcan_equipment_actuator_ArrayCommand_decode_internal(transfer, transfer->payload_len, &cmd, &tmp, 0) >= 0) {
            for ( int i = 0; i < cmd.commands.len; i++){
                if( cmd.commands.data[i].actuator_id == app_get_configuration()->uavcan_esc_index )
                {
                    switch ( cmd.commands.data[i].command_type ) {
                        case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_POSITION:
                            mc_interface_set_pid_pos( cmd.commands.data[i].command_value * 180.0f / M_PI ); // vesc needs [deg] but UAVCAN uses [rad]
                            timeout_reset();
                            break;
                        case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_FORCE:
                            mc_interface_set_current(cmd.commands.data[i].command_value); // in [A]
                            timeout_reset();
                            break;
                        case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_SPEED:
                            mc_interface_set_pid_mech_speed(cmd.commands.data[i].command_value * 60 / ( 2.0f * M_PI )); // vesc needs [rpm], uavcan uses [rad/s]
                            timeout_reset();
                            break;
                    }
                    break;
                }
            }
        }
	} else if ((transfer->transfer_type == CanardTransferTypeBroadcast) &&
			(transfer->data_type_id == UAVCAN_EQUIPMENT_ESC_STATUS_ID)) {
		uavcan_equipment_esc_Status msg;
		if (uavcan_equipment_esc_Status_decode_internal(transfer, transfer->payload_len, &msg, 0, 0, true) >= 0) {
			for (int i = 0;i < STATUS_MSGS_TO_STORE;i++) {
				status_msg_wrapper_t *msgw = &stat_msgs[i];
				if (msgw->id == -1 || msgw->id == transfer->source_node_id) {
					msgw->id = transfer->source_node_id;
					msgw->rx_time = chVTGetSystemTimeX();
					msgw->msg = msg;
					break;
				}
			}
		}
	}
	else if ((transfer->transfer_type == CanardTransferTypeBroadcast) &&
            (transfer->data_type_id == UAVCAN_PROTOCOL_PANIC_ID)) {
        // TODO: implement proper panic behaviour (wait for Panic min messages in panic max interval)
        if(app_get_configuration()->uavcan_fault_on_panic) mc_interface_fault_stop(FAULT_CODE_UAVCAN_PANIC);
    }
	else if ((transfer->transfer_type == CanardTransferTypeBroadcast) &&
             (transfer->data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_SETLIMITS_ID)){
        uavcan_equipment_actuator_SetLimits msg;
        uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_SETLIMITS_MAX_SIZE];
        memset(buffer, 0, sizeof(buffer));
        uint8_t *tmp = buffer;
        if ( uavcan_equipment_actuator_SetLimits_decode_internal(transfer, transfer->payload_len, &msg, &tmp, 0) >= 0 ){
            if ( msg.actuator_id == app_get_configuration()->uavcan_esc_index ){
                // read current esc configuration
                if ( msg.home_now
                && encoder_is_configured()){
                    mc_interface_reset_home();
                }
                else {
                    mc_interface_set_home_offset(msg.offset * 180.0f / M_PI);
                }
                mc_configuration mcconf = *mc_interface_get_configuration();
                mcconf.si_use_mech_limits = msg.use_limits;
                mcconf.si_mech_pose_limit_max = msg.limit_max * 180.0f / M_PI; // vesc is in [deg], uavcan is in [rad]
                mcconf.si_mech_pose_limit_min = msg.limit_min * 180.0f / M_PI; // vesc is in [deg], uavcan is in [rad]
                conf_general_store_mc_configuration(&mcconf);
                mc_interface_set_configuration(&mcconf);
            }
        }

	}
}

/**
 * This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 * by the local node.
 * If the callback returns true, the library will receive the transfer.
 * If the callback returns false, the library will ignore the transfer.
 * All transfers that are addressed to other nodes are always ignored.
 */
static bool shouldAcceptTransfer(const CanardInstance* ins,
		uint64_t* out_data_type_signature,
		uint16_t data_type_id,
		CanardTransferType transfer_type,
		uint8_t source_node_id) {
	(void)ins;
	(void)source_node_id;

	if (debug_level > 0) {
		commands_printf("UAVCAN shouldAccept: NODE: %d Type: %d ID: %d",
				source_node_id, transfer_type, data_type_id);
	}

	if ((transfer_type == CanardTransferTypeRequest) && (data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID)) {
		*out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
		return true;
	}

	if ((transfer_type == CanardTransferTypeBroadcast) && (data_type_id == UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID)) {
		*out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
		return true;
	}

	if ((transfer_type == CanardTransferTypeBroadcast) && (data_type_id == UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_ID)) {
		*out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_SIGNATURE;
		return true;
	}
	if ((transfer_type == CanardTransferTypeBroadcast) && (data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID)) {
		*out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
		return true;
	}
	if ((transfer_type == CanardTransferTypeBroadcast) && (data_type_id == UAVCAN_EQUIPMENT_ESC_STATUS_ID)) {
		*out_data_type_signature = UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE;
		return true;
	}
    if ((transfer_type == CanardTransferTypeBroadcast) && (data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_SETLIMITS_ID)) {
        *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_SETLIMITS_SIGNATURE;
        return true;
    }
    if ((transfer_type == CanardTransferTypeBroadcast) && (data_type_id == UAVCAN_PROTOCOL_PANIC_ID)) {
        *out_data_type_signature = UAVCAN_PROTOCOL_PANIC_SIGNATURE;
        return true;
    }

	return false;
}

static void terminal_debug_on(int argc, const char **argv) {
	if (argc == 2) {
		int level = -1;
		sscanf(argv[1], "%d", &level);

		if (level >= 0) {
			debug_level = level;
			commands_printf("UAVCAN debug level is now %d", debug_level);
		} else {
			commands_printf("Invalid argument(s).\n");
		}
	} else {
		commands_printf("This command requires one argument.\n");
	}
}

static THD_FUNCTION(canard_thread, arg) {
	(void)arg;
	chRegSetThreadName("UAVCAN");

	canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool), onTransferReceived, shouldAcceptTransfer, NULL);

	systime_t last_status_time = 0;
	systime_t last_esc_status_time = 0;
	systime_t last_panic_time = 0;


	for (;;) {
		const app_configuration *conf = app_get_configuration();

		if (conf->can_mode != CAN_MODE_UAVCAN) {
			chThdSleepMilliseconds(100);
			continue;
		}

		canardSetLocalNodeID(&canard, conf->controller_id);

		CANRxFrame *rxmsg;
		while ((rxmsg = comm_can_get_rx_frame()) != 0) {
			CanardCANFrame rx_frame;

			if (rxmsg->IDE == CAN_IDE_EXT) {
				rx_frame.id = rxmsg->EID | CANARD_CAN_FRAME_EFF;
			} else {
				rx_frame.id = rxmsg->SID;
			}

			rx_frame.data_len = rxmsg->DLC;
			memcpy(rx_frame.data, rxmsg->data8, rxmsg->DLC);

			canardHandleRxFrame(&canard, &rx_frame, ST2US(chVTGetSystemTimeX()));
		}

		for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
			comm_can_transmit_eid(txf->id, txf->data, txf->data_len);
			canardPopTxQueue(&canard);
		}

		if (ST2MS(chVTTimeElapsedSinceX(last_status_time)) >= 1000) {
			last_status_time = chVTGetSystemTimeX();
			canardCleanupStaleTransfers(&canard, ST2US(chVTGetSystemTimeX()));

			uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
			makeNodeStatusMessage(buffer);

			static uint8_t transfer_id;
			canardBroadcast(&canard,
					UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
					UAVCAN_NODE_STATUS_DATA_TYPE_ID,
					&transfer_id,
					CANARD_TRANSFER_PRIORITY_LOW,
					buffer,
					UAVCAN_NODE_STATUS_MESSAGE_SIZE);
		}

        if( mc_interface_get_fault()!=FAULT_CODE_NONE
            && mc_interface_get_fault()!= FAULT_CODE_UAVCAN_PANIC
            && ST2US(chVTTimeElapsedSinceX(last_panic_time)) >= 1000 * UAVCAN_PROTOCOL_PANIC_INTERVAL ) {
            last_panic_time = chVTGetSystemTimeX();
            sendPanic();
        }

		if (ST2US(chVTTimeElapsedSinceX(last_esc_status_time)) >= 1000000 / conf->send_can_status_rate_hz  &&
				conf->send_can_status == CAN_STATUS_1) {
			last_esc_status_time = chVTGetSystemTimeX();
			sendActuatorStatus();
		}
        else if (ST2US(chVTTimeElapsedSinceX(last_esc_status_time)) >= 1000000 / conf->send_can_status_rate_hz &&
                conf->send_can_status != CAN_STATUS_DISABLED) {
            last_esc_status_time = chVTGetSystemTimeX();
            sendEscStatus();
        }

        chThdSleepMicroseconds(100);
	}
}

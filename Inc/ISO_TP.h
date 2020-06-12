/* @file: ISO_TP.h
 * @details: Header file for ISO_TP.c
 */
#pragma once

/* ---------------------------------INCLUDES----------------------------------*/
#include <stdint.h>
/* -------------------------------PUBLIC TYPES---------------------------------*/

/*
 * @brief Defines types of errors
 */
typedef enum {
	/* @breif Statuses */
	 ISOTP_NO_ERROR,
	 ISOTP_RECEIVEING_IN_PROGRESS,
	 ISOTP_RECEIVING_COMPLETED,

	 /* @brief Errors */
	 ISOTP_MSG_SIZE_EXCEEDS_4095,
	 ISOTP_INVALID_FLOW_FRAME_FIELD,
	 ISOTP_RECEIVED_DATA_INCOMPATIBLE,
	 ISOTP_INCORRECT_SINGLE_DATA_LENGTH,
	 ISOTP_INCORRECT_FIRST_DATA_LENGTH,
	 ISOTP_INCORRECT_FLOW_DATA_LENGTH,
	 ISOTP_UNEXPECTED_CONSECUTIVE_FRAME,
	 ISOTP_SENDING_OTHER_MSG_IN_PROGRESS
}ISOTP_ErrorType_t;

typedef enum {
	ISOTP_FC_CONTINUE,	/* @brief: Request for continuation of sending */
	ISOTP_FC_WAIT,		/* @brief: Request for waiting for next Flow frame */
	ISOTP_FC_ABORT		/* @brief: Request for abortion the transmission */
}ISOTP_FC_Flags_t;

/* ----------------------------FUNCTIONS PROTOTYPES---------------------------*/

/*
 * @brief Function used for getting current MCU time stamp in 0.1 ms resolution.
 * To be defined by USER.
 * @param Time stamp value in 0.1 millisecond resolution.
 */
void ISOTP_GetTimeStamp(uint32_t * timeStamp_pu32);

/*
 * @brief Function used for sending one CAN message.
 * To be defined by USER.
 * @param CAN message Id.
 * @param CAN DLC field value.
 * @param Pointer to Tx data.
 */
void ISOTP_SendCanFrame(uint32_t msgId_u32, uint8_t dataLength_u8, uint8_t * data_pu8);

/*
 * @brief State machine used for sending ISOTP message. Should be invoked every
 * 100 microsecond by USER.
 */
void ISOTP_TxStateMachine(void);

/*
 * @brief Function sends flow frame to abort current transmission
 */
void ISOTP_Abort(void);

/*
 * @brief USER request for sending message in format compatible with ISOTP.
 * @param CAN ID field
 * @param Isotp payload.
 * @param Length of payload in bytes.
 * @return ISOTP Error Status, @ref ISOTP_ErrorType_t.
 */
ISOTP_ErrorType_t ISOTP_SendIsotpMsg(uint32_t CanMsgId_u32, uint8_t * isotpPayload_pu8, uint16_t dataSize_u16);

/*
 * @brief Function parses raw CAN data into ISOTP payload. Should be invoked each
 * time when CAN message is received.
 * @param Raw CAN data.
 * @param CAN Id of received message.
 * @param Value of CAN DLC field.
 * @param [out] Parsed ISOTP payload.
 * @param Length of payload in bytes.
 * @return ISOTP Error Status, @ref ISOTP_ErrorType_t.
 */
ISOTP_ErrorType_t ISOTP_GetIsotpPayload(uint8_t * rawData_pu8, uint32_t canId_u32,
		uint8_t canDLC_u8, uint8_t * isotpPayload_pu8, uint16_t * payloadSize_pu16);

/*
 * @brief Function configures settings of transmission - Flow frame fields.
 * @param FC flag Flow frame field, for allowed values @ref ISOTP_FC_Flags_t.
 * @param Number of Consecutive frames allowed to being send. All when equal to 0.
 * @param Delay between two Consecutive frames. Values up to 127 - time in ms.
 * Values from 241 to 249 - time from 100 to 900 us. Other values are forbidden.
 * @return ISOTP Error Status, @ref ISOTP_ErrorType_t.
 */
ISOTP_ErrorType_t ISOTP_ConfigureTransmission(uint8_t FC_flag_u8, uint8_t BlockSize_u8, uint8_t ST_u8);

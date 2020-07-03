/*
 * @file: ISO_TP.c
 * @brief: Implementation of ISO_TP Library
 */
/* ---------------------------------INCLUDES----------------------------------*/
#include "ISO_TP.h"
#include <string.h>
/* ---------------------------------DEFINES-----------------------------------*/

#ifndef RESET
#define RESET 0
#endif

#ifndef SET
#define SET 1
#endif
/*
 * @brief State machine states
 */
#define IDLE_STATE						(0U)
#define SEND_SINGLE_FRAME_STATE			(1U)
#define SEND_FIRST_FRAME_STATE			(2U)
#define WAIT_FOR_FLOW_FRAME_STATE		(3U)
#define SEND_CONSECUTIVE_FRAME_STATE	(4U)
#define SEND_FLOW_FRAME_STATE			(5U)

/* ----------------------------------MACROS-----------------------------------*/

/* ---------------------------------TYPEDEFS----------------------------------*/

/* ----------------------------------ENUMS------------------------------------*/
typedef enum
{
    ISOTP_SINGLE_FRAME,
    ISOTP_FIRST_FRAME,
    ISOTP_CONSECUTIVE_FRAME,
    ISOTP_FLOW_FRAME,
    ISOTP_NO_COMPATIBLE = 0xFF
} ISOTP_FrameType_t;

typedef enum
{
    ISOTP_SINGLE_FRAME_MSG, ISOTP_MULTI_FRAME_MSG, ISOTP_FLOW_FRAME_MSG
} ISOTP_MsgType_t;
/* --------------------------------STRUCTURES---------------------------------*/

/*
 * @brief Structure used for transmission configuration.
 */
typedef struct
{
    uint8_t FC_Flag_u8;
    uint8_t BlockSize_u8;
    uint8_t ST_u8;
} ISOTP_TransmissionConfig_t;

/*
 * @brief Tx transmission configuration
 */
ISOTP_TransmissionConfig_t ISOTP_TransmissionConfigTx_s;

/*
 * @brief Rx transmission configuration
 */
ISOTP_TransmissionConfig_t ISOTP_TransmissionConfigRx_s;

/* --------------------------------VARIABLES----------------------------------*/
/*
 * @brief State machine status
 */
static uint8_t CurrentState_u8 = (uint8_t) IDLE_STATE;

/*
 * @brief Recent library error
 */
static ISOTP_ErrorType_t CurrentErrorStatus_u8 = ISOTP_NO_ERROR;

/*
 * @brief Flag indicating Flow message receiving
 */
static uint8_t IsFlowMsgReceived_u8 = RESET;

/*
 * @brief CAN Id field of current sending message
 */
static uint32_t CanId_u32;

/*
 * @brief Array of ISOTP payload
 */
static uint8_t IsotpPayload_au8[4095];

/*
 * @brief Size of payload of current isotp message
 */
static uint16_t IsotpPayloadSize_u16;

/* ----------------------------FUNCTIONS PROTOTYPES---------------------------*/
/*
 * @brief Function checks what type of message should be created to send the data.
 * @param Number of bytes to being transmitted.
 * @return ISOTP message type @ref ISOTP_MsgType_t
 */
static ISOTP_MsgType_t ISOTP_CheckMsgType( uint16_t dataSize_u16 );

/*
 * @brief Function creates and sends Single frame.
 * @param Pointer to ISOTP payload.
 * @param Length of ISOTP payload.
 */
static void ISOTP_SendSingleFrame( uint8_t * data_pu8, uint8_t dataSize_u8 );

/*
 * @brief Function creates and sends First frame.
 * @param ISOTP message size.
 * @param Pointer to First frame payload.
 * @param Size of payload.
 */
static void ISOTP_SendFirstFrame( uint16_t msgSize_u16, uint8_t * data_pu8,
        uint8_t dataSize_u8 );

/*
 * @brief Function creates and sends Consecutive frame.
 * @param Pointer to ISOTP payload.
 * @param Length of ISOTP payload.
 * @param Id of Consecutive frame.
 */
static void ISOTP_SendConsFrame( uint8_t * data_pu8, uint8_t dataSize_u8,
        uint8_t frameId_u8 );

/*
 * @brief Function creates and sends Flow frame.
 * @param Response Can Id, should be same as last received first or consecutive frame.
 * @param FC flag Flow frame field, for allowed values @ref ISOTP_FC_Flags_t.
 * @param Number of Consecutive frames allowed to being send. All when equal to 0.
 * @param Delay between two Consecutive frames. Values up to 127 - time in ms.
 * Values from 241 to 249 - time from 100 to 900 us. Other values are forbidden.
 */
static void ISOTP_SendFlowFrame( uint32_t canRxId_u32, uint8_t FC_u8,
        uint8_t blockSize_u8, uint8_t ST_u8 );

/*
 * @brief Function checks whether received data is compatible with ISOTP.
 * @param Pointer to received data.
 * @return Type of frame. @ref ISOTP_FrameType_t
 */
static ISOTP_FrameType_t ISOTP_CheckIsotpCompatibility( uint8_t * rawData_pu8 );

/* ---------------------------FUNCTIONS DEFINITIONS---------------------------*/

static ISOTP_MsgType_t ISOTP_CheckMsgType( uint16_t dataSize_u16 )
{
    ISOTP_MsgType_t retValue_u8;
    CurrentErrorStatus_u8 = ISOTP_NO_ERROR;

    /* Check data size */
    if (dataSize_u16 <= 7)
    {
        retValue_u8 = ISOTP_SINGLE_FRAME_MSG;
    } else if ((dataSize_u16 >= 8) && (dataSize_u16 <= 4095))
    {
        retValue_u8 = ISOTP_MULTI_FRAME_MSG;
    } else
    {
        retValue_u8 = (ISOTP_MsgType_t) ISOTP_MSG_SIZE_EXCEEDS_4095;
        CurrentErrorStatus_u8 = ISOTP_MSG_SIZE_EXCEEDS_4095;
    }
    return retValue_u8;
}

static void ISOTP_SendSingleFrame( uint8_t * data_pu8, uint8_t dataSize_u8 )
{
    uint8_t rxBuffer_au8[8];

    /* Fill rx buffer */
    rxBuffer_au8[0] = (0x00 | (dataSize_u8 & 0x0F));
    memcpy(&rxBuffer_au8[1], data_pu8, dataSize_u8);

    /* Send CAN message */
    ISOTP_SendCanFrame(CanId_u32, (1 + dataSize_u8), rxBuffer_au8);
}

static void ISOTP_SendFirstFrame( uint16_t msgSize_u16, uint8_t * data_pu8,
        uint8_t dataSize_u8 )
{
    uint8_t rxBuffer_au8[8];

    /* Fill rx buffer */
    rxBuffer_au8[0] = (0x10 | ((msgSize_u16 >> 8) & 0x0F));
    rxBuffer_au8[1] = (uint8_t) (msgSize_u16 & 0xFF);
    memcpy(&rxBuffer_au8[2], data_pu8, dataSize_u8);

    /* Send CAN message */
    ISOTP_SendCanFrame(CanId_u32, (2 + dataSize_u8), rxBuffer_au8);
}

static void ISOTP_SendConsFrame( uint8_t * data_pu8, uint8_t dataSize_u8,
        uint8_t frameId_u8 )
{
    uint8_t rxBuffer_au8[8];

    /* Fill rx buffer */
    rxBuffer_au8[0] = (0x20 | (frameId_u8 & 0x0F));
    memcpy(&rxBuffer_au8[1], data_pu8, dataSize_u8);

    /* Send CAN message */
    ISOTP_SendCanFrame(CanId_u32, (1 + dataSize_u8), rxBuffer_au8);
}

static void ISOTP_SendFlowFrame( uint32_t canRxId_u32, uint8_t FC_u8,
        uint8_t blockSize_u8, uint8_t ST_u8 )
{
    uint8_t rxBuffer_au8[8];

    /* Fill rx buffer */
    rxBuffer_au8[0] = (0x30 | (FC_u8 & 0x0F));
    rxBuffer_au8[1] = blockSize_u8;
    rxBuffer_au8[2] = ST_u8;

    /* Send CAN message */
    ISOTP_SendCanFrame(canRxId_u32, 3, rxBuffer_au8);
}

static ISOTP_FrameType_t ISOTP_CheckIsotpCompatibility( uint8_t * rawData_pu8 )
{
    ISOTP_FrameType_t retValue_u8;
    uint8_t dataType_u8 = (rawData_pu8[0] & 0xF0);

    switch ( dataType_u8 )
    {
    case 0x00:
        retValue_u8 = ISOTP_SINGLE_FRAME;
        break;
    case 0x10:
        retValue_u8 = ISOTP_FIRST_FRAME;
        break;
    case 0x20:
        retValue_u8 = ISOTP_CONSECUTIVE_FRAME;
        break;
    case 0x30:
        retValue_u8 = ISOTP_FLOW_FRAME;
        break;
    default :
        retValue_u8 = ISOTP_NO_COMPATIBLE;
        break;
    }

    return retValue_u8;
}

void ISOTP_Abort( void )
{
    /* Discard pending transmission */
    ISOTP_SendFlowFrame(CanId_u32, ISOTP_FC_ABORT, 0, 0);

}

void ISOTP_TxStateMachine( void )
{
    /* Variable for Consecutive frame transmitting */
    static uint16_t frameCounter_u16;
    static uint32_t timeStampStart_u32;
    static uint32_t timeStampStop_u32;
    static uint8_t frameIndex_u8;
    static uint8_t isFirstConsFrameTranssmited_u8 = RESET;
    uint8_t currentFramePayloadSize_u8;
    uint8_t currentTimeStampOffset_u8;

    switch ( CurrentState_u8 )
    {
    case IDLE_STATE:

        /* Do nothing */

        break;

    case SEND_SINGLE_FRAME_STATE:

        /* Send Single frame message */
        ISOTP_SendSingleFrame(IsotpPayload_au8, (uint8_t) IsotpPayloadSize_u16);
        CurrentState_u8 = IDLE_STATE;
        break;

    case SEND_FIRST_FRAME_STATE:

        /* Send First frame message */
        ISOTP_SendFirstFrame(IsotpPayloadSize_u16, IsotpPayload_au8, 6);

        /* Decrement message payload size */
        IsotpPayloadSize_u16 -= 6;

        /* Shift Tx buffer data */
        memcpy(&IsotpPayload_au8[0], &IsotpPayload_au8[6],
                IsotpPayloadSize_u16);

        /* Reset frame counter */
        frameCounter_u16 = 0;

        /* Set frame index for the next consecutive frame */
        frameIndex_u8 = 0x01;

        /* Indicate that first consecutive frame has not been transmitted yet */
        isFirstConsFrameTranssmited_u8 = RESET;

        CurrentState_u8 = WAIT_FOR_FLOW_FRAME_STATE;
        break;

    case WAIT_FOR_FLOW_FRAME_STATE:

        /* Check whether Flow frame is received */
        if (IsFlowMsgReceived_u8 == SET)
        {
            CurrentState_u8 = SEND_CONSECUTIVE_FRAME_STATE;
            IsFlowMsgReceived_u8 = RESET;
        }
        break;

    case SEND_CONSECUTIVE_FRAME_STATE:

        /* Check Consecutive frame payload size */
        if (IsotpPayloadSize_u16 > 7)
        {
            currentFramePayloadSize_u8 = 7;
        } else
        {
            currentFramePayloadSize_u8 = IsotpPayloadSize_u16;
        }

        /* Set time stamp offset */
        if ((ISOTP_TransmissionConfigTx_s.ST_u8 > 0)
                && (ISOTP_TransmissionConfigTx_s.ST_u8 < 127))
        {
            /* Set offset and adjust value from 0.1 ms to 1 ms resolution */
            currentTimeStampOffset_u8 =
                    (ISOTP_TransmissionConfigTx_s.ST_u8 * 10);
        } else if ((ISOTP_TransmissionConfigTx_s.ST_u8 >= 241)
                && (ISOTP_TransmissionConfigTx_s.ST_u8 <= 249))
        {
            /* Set offset from 0.1 (1) to 0.9 ms (9) */
            currentTimeStampOffset_u8 = (ISOTP_TransmissionConfigTx_s.ST_u8
                    - 240);
        } else
        {
            /* Wrong ST field */
            currentTimeStampOffset_u8 = 0;
        }

        /* Check Control Flag */
        switch ( ISOTP_TransmissionConfigTx_s.FC_Flag_u8 )
        {
        case ISOTP_FC_CONTINUE:

            /* Check whether first consecutive frame is transmitted */
            if (isFirstConsFrameTranssmited_u8 == RESET)
            {

                ISOTP_SendConsFrame(IsotpPayload_au8,
                        currentFramePayloadSize_u8, frameIndex_u8);

                /* Increment frame index */
                if (frameIndex_u8 <= 15)
                {
                    frameIndex_u8++;
                } else
                {
                    frameIndex_u8 = 0;
                }

                /* Decrement message payload size */
                IsotpPayloadSize_u16 -= currentFramePayloadSize_u8;

                /* Shift Tx buffer data */
                memcpy(&IsotpPayload_au8[0],
                        &IsotpPayload_au8[currentFramePayloadSize_u8],
                        IsotpPayloadSize_u16);

                /* Get initial time stamp value */
                ISOTP_GetTimeStamp(&timeStampStart_u32);

                /* Set final time stamp value */
                timeStampStop_u32 = timeStampStart_u32
                        + (uint32_t) currentTimeStampOffset_u8;

                /* Increment frame counter */
                frameCounter_u16++;

                /* Set the flag indicating that first frame is transmitted */
                isFirstConsFrameTranssmited_u8 = SET;
            } else
            {
                /* Check whether delay between two consecutive frame is completed */
                ISOTP_GetTimeStamp(&timeStampStart_u32);
                if (timeStampStart_u32 >= timeStampStop_u32)
                {
                    /* Check whether there  limit of sending Consecutive frame is achieved */
                    if ((ISOTP_TransmissionConfigTx_s.BlockSize_u8 == 0)
                            || (frameCounter_u16
                                    < ISOTP_TransmissionConfigTx_s.BlockSize_u8))
                    {
                        ISOTP_SendConsFrame(IsotpPayload_au8,
                                currentFramePayloadSize_u8, frameIndex_u8);

                        /* Increment frame index */
                        if (frameIndex_u8 <= 15)
                        {
                            frameIndex_u8++;
                        } else
                        {
                            frameIndex_u8 = 0;
                        }

                        /* Decrement message payload size */
                        IsotpPayloadSize_u16 -= currentFramePayloadSize_u8;

                        /* Shift Tx buffer data */
                        memcpy(&IsotpPayload_au8[0],
                                &IsotpPayload_au8[currentFramePayloadSize_u8],
                                IsotpPayloadSize_u16);

                        /* Get initial time stamp value */
                        ISOTP_GetTimeStamp(&timeStampStart_u32);

                        /* Set final time stamp value */
                        timeStampStop_u32 = timeStampStart_u32
                                + (uint32_t) currentTimeStampOffset_u8;

                        /* Increment frame counter */
                        frameCounter_u16++;

                    } else
                    {
                        /* Wait for Flow frame (permission for continuation) */
                    }
                } else
                {
                    /* Wait */
                }
            }

            /* Check whether full message is transmitted */
            if (IsotpPayloadSize_u16 == 0)
            {
                CurrentState_u8 = IDLE_STATE;
            }
            break;

        case ISOTP_FC_WAIT:

            /* Reset frame counter */
            frameCounter_u16 = 0;
            break;

        case ISOTP_FC_ABORT:

            /* Abort transmission */
            CurrentState_u8 = IDLE_STATE;
            break;
        }

    }

}

ISOTP_ErrorType_t ISOTP_SendIsotpMsg( uint32_t CanMsgId_u32,
        uint8_t * isotpPayload_pu8, uint16_t dataSize_u16 )
{
    uint8_t retStatus_u8 = ISOTP_NO_ERROR;

    /* Check whether sending new message is allowed */
    if (CurrentState_u8 == IDLE_STATE)
    {

        /* Check type of requested message */
        ISOTP_MsgType_t msgType = ISOTP_CheckMsgType(dataSize_u16);

        /* Get CAN ID field */
        CanId_u32 = CanMsgId_u32;

        /* Update size of message payload */
        IsotpPayloadSize_u16 = dataSize_u16;

        /* Get payload */
        memcpy(IsotpPayload_au8, isotpPayload_pu8, dataSize_u16);

        switch ( msgType )
        {
        case ISOTP_SINGLE_FRAME_MSG:
        {
            CurrentState_u8 = SEND_SINGLE_FRAME_STATE;
            break;
        }
        case ISOTP_MULTI_FRAME_MSG:
        {
            CurrentState_u8 = SEND_FIRST_FRAME_STATE;
            break;
        }
        default :
            break;
        }
    } else
    {
        CurrentErrorStatus_u8 = ISOTP_SENDING_OTHER_MSG_IN_PROGRESS;
        retStatus_u8 = CurrentErrorStatus_u8;
    }

    return retStatus_u8;
}

ISOTP_ErrorType_t ISOTP_GetIsotpPayload( uint8_t * rawData_pu8,
        uint32_t canId_u32, uint8_t canDLC_u8, uint8_t * isotpPayload_pu8,
        uint16_t * payloadSize_pu16 )
{
    ISOTP_ErrorType_t retStatus_u8 = ISOTP_NO_ERROR;
    static uint16_t fullMsgSize_u16;
    static uint16_t remainingMsgSize_u16;
    static uint8_t expectedConsFrameIndex_u8;
    static _Bool consecutiveFrameExpected_bo = RESET;
    uint16_t isotpPayloadLength_u16;

    /* Check whether received data is compatible with ISO-TP */
    uint8_t frameType_u8 = ISOTP_CheckIsotpCompatibility(rawData_pu8);

    /* Init payload size */
    *payloadSize_pu16 = 0;

    if (frameType_u8 != ISOTP_NO_COMPATIBLE)
    {
        switch ( frameType_u8 )
        {
        case ISOTP_SINGLE_FRAME:

            /* Get declared payload size */
            isotpPayloadLength_u16 = (rawData_pu8[0] & 0x0F);

            if ((isotpPayloadLength_u16 + 1) == canDLC_u8)
            {
                /* Update payload */
                memcpy(&isotpPayload_pu8[0], &rawData_pu8[1],
                        isotpPayloadLength_u16);

                /* Update payload size */
                *payloadSize_pu16 = isotpPayloadLength_u16;

                retStatus_u8 = ISOTP_RECEIVING_COMPLETED;
            } else
            {
                retStatus_u8 = ISOTP_INCORRECT_SINGLE_DATA_LENGTH;
            }

            break;

        case ISOTP_FIRST_FRAME:

            /* Get declared message size */
            isotpPayloadLength_u16 = ((rawData_pu8[0] & 0x0F) | rawData_pu8[1]);

            fullMsgSize_u16 = isotpPayloadLength_u16;
            remainingMsgSize_u16 = isotpPayloadLength_u16;

            /* Check whether message should be multi frame */
            if (canDLC_u8 == 8)
            {
                /* Update payload */
                memcpy(&isotpPayload_pu8[0], &rawData_pu8[2], 6);

                /* Decrement expected payload size */
                remainingMsgSize_u16 -= 6;

                /* Updated next consecutive frame index */
                expectedConsFrameIndex_u8 = 0x01;

                consecutiveFrameExpected_bo = SET;

                /* Send flow frame */
                ISOTP_SendFlowFrame(canId_u32,
                        ISOTP_TransmissionConfigRx_s.FC_Flag_u8,
                        ISOTP_TransmissionConfigRx_s.BlockSize_u8,
                        ISOTP_TransmissionConfigRx_s.ST_u8);

                retStatus_u8 = ISOTP_RECEIVEING_IN_PROGRESS;
            } else
            {
                retStatus_u8 = ISOTP_INCORRECT_FIRST_DATA_LENGTH;
            }

            break;

        case ISOTP_CONSECUTIVE_FRAME:

            /* Check whether consecutive frame is expected */
            if (consecutiveFrameExpected_bo == SET)
            {
                /* Get frame index */
                uint8_t frameIndex_u8 = (rawData_pu8[0] & 0x0F);

                /* Check whether current frame index is expected */
                if (frameIndex_u8 == expectedConsFrameIndex_u8)
                {
                    /* Get offset for payload data */
                    uint8_t offset_u8 = (fullMsgSize_u16 - remainingMsgSize_u16);

                    /* Copy payload */
                    memcpy(&isotpPayload_pu8[offset_u8], &rawData_pu8[1],
                            (canDLC_u8 - 1));

                    /* Decrement expected payload size */
                    remainingMsgSize_u16 -= (canDLC_u8 - 1);

                    /* Increment expected frame index */
                    expectedConsFrameIndex_u8++;

                    /* Reset indexer if it exceeds 15 */
                    if (expectedConsFrameIndex_u8 > 15)
                    {
                        expectedConsFrameIndex_u8 = 0;
                    }

                    /* Check whether receiving is completed */
                    if (remainingMsgSize_u16 == 0)
                    {
                        /* Update payload size */
                        *payloadSize_pu16 = fullMsgSize_u16;

                        consecutiveFrameExpected_bo = RESET;

                        retStatus_u8 = ISOTP_RECEIVING_COMPLETED;
                    } else
                    {
                        retStatus_u8 = ISOTP_RECEIVEING_IN_PROGRESS;
                    }
                } else
                {
                    retStatus_u8 = ISOTP_UNEXPECTED_CONSECUTIVE_FRAME;
                }
            } else
            {
                retStatus_u8 = ISOTP_UNEXPECTED_CONSECUTIVE_FRAME;
            }
            break;

        case ISOTP_FLOW_FRAME:

            /* Inform state machine about receiving Flow frame */
            IsFlowMsgReceived_u8 = SET;

            /* Update transmitting configuration */
            ISOTP_TransmissionConfigTx_s.FC_Flag_u8 = (rawData_pu8[0] & 0x0F);
            ISOTP_TransmissionConfigTx_s.BlockSize_u8 = rawData_pu8[1];
            ISOTP_TransmissionConfigTx_s.ST_u8 = rawData_pu8[2];

            retStatus_u8 = ISOTP_RECEIVING_COMPLETED;

            break;
        }
    } else
    {
        retStatus_u8 = ISOTP_RECEIVED_DATA_INCOMPATIBLE;
    }

    return retStatus_u8;
}

ISOTP_ErrorType_t ISOTP_ConfigureTransmission( uint8_t FC_flag_u8,
        uint8_t BlockSize_u8, uint8_t ST_u8 )
{
    ISOTP_ErrorType_t retValue_u8 = ISOTP_NO_ERROR;

    /* Check whether FC flag is in range */
    if (FC_flag_u8 <= 2)
    {
        ISOTP_TransmissionConfigRx_s.FC_Flag_u8 = FC_flag_u8;
    } else
    {
        retValue_u8 = ISOTP_INVALID_FLOW_FRAME_FIELD;
        CurrentErrorStatus_u8 = ISOTP_INVALID_FLOW_FRAME_FIELD;
    }

    /* Update Block Size for receiving */
    ISOTP_TransmissionConfigRx_s.BlockSize_u8 = BlockSize_u8;

    /* Check whether ST is in range */
    if ((ST_u8 > 0 && ST_u8 <= 127) || (ST_u8 >= 241 && ST_u8 <= 249))
    {
        ISOTP_TransmissionConfigRx_s.ST_u8 = ST_u8;
    } else
    {
        retValue_u8 = ISOTP_INVALID_FLOW_FRAME_FIELD;
        CurrentErrorStatus_u8 = ISOTP_INVALID_FLOW_FRAME_FIELD;
    }

    return retValue_u8;
}

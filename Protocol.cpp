/****************************************Copyright(c)*****************************************************
**                            Shenzhen Yuejiang Technology Co., LTD.
**
**                                 http://www.dobot.cc
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           Protocol.cpp
** Latest modified Date:2016-06-01
** Latest Version:      V1.0.0
** Descriptions:        Protocol interface
**
**--------------------------------------------------------------------------------------------------------
** Created by:          Liu Zhufu
** Created date:        2016-03-14
** Version:             V1.0.0
** Descriptions:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
#include "Protocol.h"
#include <stdio.h>
#include <string.h>
#include <HardwareSerial.h>

/*********************************************************************************************************
** Protocol buffer definition
*********************************************************************************************************/
#define RAW_BYTE_BUFFER_SIZE    256
#define PACKET_BUFFER_SIZE  4

// Serial
uint8_t gSerialTXRawByteBuffer[RAW_BYTE_BUFFER_SIZE];
uint8_t gSerialRXRawByteBuffer[RAW_BYTE_BUFFER_SIZE];
Packet gSerialTXPacketBuffer[PACKET_BUFFER_SIZE];
Packet gSerialRXPacketBuffer[PACKET_BUFFER_SIZE];

ProtocolHandler gSerialProtocolHandler;

/*********************************************************************************************************
** Function name:       ProtocolInit
** Descriptions:        Init the protocol buffer etc.
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void ProtocolInit(void)
{
    // Init Serial protocol
    RingBufferInit(&gSerialProtocolHandler.txRawByteQueue, gSerialTXRawByteBuffer, RAW_BYTE_BUFFER_SIZE, sizeof(uint8_t));
    RingBufferInit(&gSerialProtocolHandler.rxRawByteQueue, gSerialRXRawByteBuffer, RAW_BYTE_BUFFER_SIZE, sizeof(uint8_t));
    RingBufferInit(&gSerialProtocolHandler.txPacketQueue, gSerialTXPacketBuffer, PACKET_BUFFER_SIZE, sizeof(Packet));
    RingBufferInit(&gSerialProtocolHandler.rxPacketQueue, gSerialRXPacketBuffer, PACKET_BUFFER_SIZE, sizeof(Packet));
}

/*********************************************************************************************************
** Function name:       ProtocolProcess
** Descriptions:        Process the protocol
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void ProtocolProcess(void)
{
    Message message;
    MessageProcess(&gSerialProtocolHandler);
    //ÃƒÂ¯Ã‚Â¿Ã‚Â½Ãƒï¿½Ã‚Â·ÃƒÂ¯Ã‚Â¿Ã‚Â½ÃƒÂ¯Ã‚Â¿Ã‚Â½ÃƒÂ¯Ã‚Â¿Ã‚Â½
    if (RingBufferGetCount(&gSerialProtocolHandler.txRawByteQueue)) {
        uint8_t data;
        while (RingBufferIsEmpty(&gSerialProtocolHandler.txRawByteQueue) == false) {
            RingBufferDequeue(&gSerialProtocolHandler.txRawByteQueue, &data);
            Serial1.write(data);
        }
        if(MessageRead(&gSerialProtocolHandler, &message)==ProtocolNoError){
            #if 1
            //ÃƒÂ¦Ã¢â‚¬Â°Ã¢â‚¬Å“ÃƒÂ¥Ã¯Â¿Â½Ã‚Â°ÃƒÂ¦Ã…Â½Ã‚Â¥ÃƒÂ¦Ã¢â‚¬ï¿½Ã‚Â¶ÃƒÂ¥Ã‹â€ Ã‚Â°ÃƒÂ§Ã…Â¡Ã¢â‚¬Å¾ÃƒÂ¨Ã‚Â¿Ã¢â‚¬ï¿½ÃƒÂ¥Ã¢â‚¬ÂºÃ…Â¾ÃƒÂ¦Ã¢â‚¬Â¢Ã‚Â°ÃƒÂ¦Ã¯Â¿Â½Ã‚Â®
            printf("Rx message: ");
            printf("message id:%d, rw:%d, isQueued:%d, paramsLen:%d\r\n",
                    message.id, message.rw, message.isQueued, message.paramsLen);
            printf("params: ");
            for(int i=0; i<message.paramsLen; i++)
            {
                printf("%02x ", message.params[i]);
            }
            printf("\r\n");
            #endif
        }
    }
}

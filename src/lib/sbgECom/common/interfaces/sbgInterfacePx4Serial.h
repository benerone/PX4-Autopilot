#ifndef SBG_INTERFACE_PX4_SERIAL_H
#define SBG_INTERFACE_PX4_SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sbgInterfaceSerial.h"
#include <poll.h>

typedef SbgErrorCode (*SbgInterfaceCallbackReadFunc) (void*, size_t*, void*);
typedef SbgErrorCode (*SbgInterfaceCallbackWriteFunc) (void*, size_t*, void*);

/*!
 *	Callback definition called each time a new raw log is received.
 *	\param[in]	pHandle									Valid handle on the sbgECom instance that has called this callback.
 *	\param[in]	buffer								    raw buffer
 *	\param[in]	bufferSize								raw buffer
 *	\param[in]	pUserArg								Optional user supplied argument.
 *	\return												SBG_NO_ERROR if the received log has been used successfully.
 */
typedef SbgErrorCode (*SbgEComReceiveLogRawFunc)(uint8_t * buffer,size_t bufferSize , void *pUserArg);


typedef struct _SbgInterfacePx4Serial
{
	void								*userArg;
	void 								*userArgRaw;
	SbgInterfaceCallbackReadFunc		 readCallback;
	SbgInterfaceCallbackWriteFunc		 writeCallback;
	SbgEComReceiveLogRawFunc			 rawReadCallback;
	int									 serialHandle;
} SbgInterfacePx4Serial;

SbgErrorCode sbgInterfacePx4SerialCreate(SbgInterface *pSbgInterface, int serialHandle, uint32_t baudRate);

SbgErrorCode sbgInterfacePx4SerialFlush(SbgInterface *pHandle);

SbgErrorCode sbgInterfacePx4SerialRead(SbgInterface *pSbgInterface, void *pBuffer, size_t *pBytesRead, size_t bytesToRead);

SbgErrorCode sbgInterfacePx4SerialWrite(SbgInterface *pHandle, const void *pBuffer, size_t bytesToWrite);

void sbgInterfacePx4SerialSetReadCallback(SbgInterface *pHandle, SbgInterfaceCallbackReadFunc callback, void *user_arg);
void sbgInterfacePx4SerialSetReadRawCallback(SbgInterface *pHandle, SbgEComReceiveLogRawFunc callback, void *user_arg);
#ifdef __cplusplus
}
#endif

#endif /* SBG_INTERFACE_PX4_SERIAL */

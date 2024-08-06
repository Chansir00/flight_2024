#ifndef _REMOTE_H
#define _REMOTE_H


#define SUCCESS 0
#undef FAILED
	#define FAILED  1

#include <stdbool.h>
#include <stdint.h>

typedef struct tagRemoteConnect
{
	uint16_t	ConnectTimeCnt;		//连接时间计时，每3ms自加；	
	uint8_t		ConnectState;		//1-连接，0-未连接
	uint8_t		RemoteSignalLost;
	uint16_t		RemSigLostTimeCnt;

}strRemoteConnect;

extern strRemoteConnect RemoteConnect;		//连接时间计时，每3ms自加；

extern void RC_Analy(void);

#endif


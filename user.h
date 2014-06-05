/*
 * user.h
 *
 *  Created on: Mar 15, 2011
 */

#ifndef USER_H_
#define USER_H_

#include "uav.h"

#pragma pack(push,4)
class clsUser {
public:
	clsUser();
	virtual ~clsUser();

protected:
	COMMAND m_cmd;

	pthread_mutex_t m_mtxCmd;

public:
	void GetCommand(COMMAND *pCmd);

protected:
	int m_idThread;

public:
	int Run();

public:
	BOOL StartUserThread(int priority);

protected:
	static void *UserThread(void *pParameter);
};

#pragma pack(pop)
#endif /* USER_H_ */

/*
 * thread.h
 *
 *  Created on: Mar 15, 2011
 *
 *  this is the header file for public class clsThread, which stand for a public base class for all task threads
 */

#ifndef THREAD_H_
#define THREAD_H_

#include <sys/neutrino.h>
#include <stdint.h>

#ifndef BOOL
#define BOOL	int
#define TRUE	1
#define FALSE	0
#endif

#define PULSECODE_MAIN              _PULSE_CODE_MINAVAIL
#define PULSECODE_EXIT              _PULSE_CODE_MINAVAIL + 1
#define PULSECODE_DATA				_PULSE_CODE_MINAVAIL + 2				//temporary used for IM6 data

class clsThread {
public:
	clsThread();
	virtual ~clsThread();

protected:
	int m_nCount;

public:
	int GetCount() { return m_nCount; }

protected:
	BOOL m_bReady;

public:
	BOOL IsReady() { return m_bReady; }

//protected:
private:
	int m_chThread, m_coidThread;				//message channel and connection id

	pthread_cond_t m_condThread;
	pthread_mutex_t m_mtxThread;

public:
	void SendPulse(int priority, int code, int value = 0);
	int WaitForEvent(uint64_t nsec);
	int ReceivePulse(_pulse *pPulse);
	void SetEvent();

protected:
	pthread_t m_idThread;

protected:
	virtual int Run();
	virtual int EveryRun();

	virtual BOOL InitThread();
	virtual void ExitThread();

protected:
	static void *Thread(void *pParameter);

public:
	BOOL StartThread(int priority);
};

#endif /* THREAD_H_ */

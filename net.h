#ifndef NET_H_
#define NET_H_

//#include <sys/types.h>
//#include <sys/socket.h>
#include <netinet/in.h>

#include "thread.h"

#define NETPORT_BROADCAST		4404

#define HOSTLEN 256
#define SERVER_PORT_NUM	/* 40002 */ NET_PORT
#define CLIENT_PORT_NUM /* 40001 */ NET_PORT
#define PEER_HOST_PORT_LISTEN	40001
#define PEER_HOST_PORT_REQUEST 40002

#define MAX_UDPMSG_LEN 	1000		//TODO 1000 bytes for reservation usage, need to specify the value
#define SEND_BUF_LEN	100
#define RECV_BUF_LEN	100

#define COUNT_CLIENT	5			// send update info for Formation Flight every 100ms (20ms * 5)
#define PEER_HOST1 "172.20.73.99"					/* "192.168.1.1" */	// leader
#define PEER_HOST2 "172.20.73.102"	// follower1 -- shelion
#define PEER_HOST3 "192.168.1.3"	// follower2
#define PEER_HOST11	"192.168.2.3"			//gumstix2

#define CLIENT_SEND_MAX_LENGTH	60
#define testBuf "This is a test"

struct PEERADDRESS {
	char *ip;
	int rxport, txport;
};

class clsNET : public clsThread
{
protected:
	int m_socket, m_clientSocket;
	sockaddr_in m_addr;
	BOOL m_bNetStart;		// flag to indicate if the NET is started
	unsigned int m_npkt;
public:
	BOOL Init();

public:
	clsNET();
	virtual ~clsNET();

	virtual BOOL InitThread();
	virtual int EveryRun();
	virtual void ExitThread();

	void MakeTelegraph(TELEGRAPH *pTele, short code, double time, void *pData, int nDataSize);
	int ClientSendTo(const char *host, int dstPort, char *sendBuf, int nBuffer);

	void ToggleNetStart() { m_bNetStart = !m_bNetStart; }
	BOOL CheckNetStart() { return m_bNetStart; }
	void FindDestPeers(const int nfrom, PEERADDRESS& peerAddr);
	void GetHostAddr(const int nfrom, PEERADDRESS& peerAddr);

protected:
	char m_buffer[MAXSIZE_TELEGRAPH];
	int m_nBuffer;

protected:
	pthread_t m_idListenThread;

public:
	BOOL StartListenThread(int priority);

protected:
	static void *ListenThread(void *pParameter);
	void Listen();	// an always running thread every 20ms, receive commands and send reply message

	BOOL ParseBuffer(char *pBuffer, int nBuffer, ADDRESSEDPACKAGE *pPackage);
	void ProcessPackage(ADDRESSEDPACKAGE *pPackage);

protected:
	pthread_mutex_t m_mtxClient;
};

#endif				//NET_H_

#ifndef CMM_H_
#define CMM_H_

#include "thread.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//CMM
#define CMM_BAUDRATE    9600 //115200

#define MAX_MESSAGE			64
#define MAX_SIZE_MESSAGE	256
//#define GCS_IPADDR			"192.168.121.1"
//#define GCS_IPADDR			"192.168.110.1"
//#define GCS_IPADDR			"172.20.0.10"

struct CMM_CONFIG {
	char devPort[32];
	short flag;
	int baudrate;
};

struct TCP_CONFIG {
	char ipaddress[32];
	int portNum;
	short protocol;
	short flag;
};

/*union CMM_CONFIG {

};*/

struct FORMATION_DATA {
//	BOOL token;
	double x, y, z, c;
	double ug, vg, wg;
	double rzv1, rzv2, rzv3;
	double lat0, long0, alt0;
};

struct VICON_DATA {
	double x, y, z;
	double a, b, c;
};

struct TCPIP_RECORD {
	char ipaddrSrc[INET_ADDRSTRLEN];
	int	 nRcvPkt;
};

struct CMM_RECORD {
	int nSenderID;
	int nRcvPkt;
};
class clsCMM : public clsThread {
public:
	clsCMM();
	virtual ~clsCMM();

public:
	virtual BOOL InitThread();
	virtual int EveryRun();
	virtual void ExitThread();

	//new added, network integrated
protected:
	int m_socket;
	char m_bufferNet[MAX_CMMBUFFER];
	int m_nBufferNet;
	FORMATION_DATA m_formationData;
	VICON_DATA	m_viconData;
	CMM_RECORD m_cmmRcd[5];

public:
	BOOL InitSocket();
	BOOL StartListenThread(int priority);
//	void AccountCMMRecord(int nFromID);
	void CollectViconData(VICON_DATA viconData);
	VICON_DATA &GetViconData() { return m_viconData; }

protected:
	static void *ListenThread(void *pParameter);
	void Listen();

protected:
	static BOOL ParseBuffer(char *pBuffer, int &nBuffer, ADDRESSEDPACKAGE *pPackage);

	void ProcessPackage(ADDRESSEDPACKAGE *pPackage);

protected:
	int m_nsCMM;

	char m_szBuffer[MAX_CMMBUFFER];
	int m_nBuffer;

	int m_idInputThread;

	COMMAND m_cmd;
	pthread_mutex_t m_mtxCmd;

	char m_gcsIPAddr[64];
	CMM_CONFIG m_cmmConfig;
	TCP_CONFIG m_tcpConfig;
	BOOL m_bVicon;
	bool m_bLostConnection;
	double m_LostConnectionTime;

public:
	BOOL Open();
	void Close();

	void GetCommand(COMMAND *pCmd);
	void PutCommand(COMMAND *pCmd);

protected:
	char m_szMessage[MAX_MESSAGE][MAX_SIZE_MESSAGE];
	int m_nMessage;

	pthread_mutex_t m_mtxMessage;

public:
	void PutMessage(char *pszMessage);
	void SendAllMessages();

	void SendMessage(const char *pszMessage);
	void SendRawMessage(char *message);

	void SendPackage(ADDRESSEDPACKAGE *pPackage);
	void SendTelegraph(TELEGRAPH *pTele);

	CMM_CONFIG GetCmmConfig() const {return m_cmmConfig;}
	TCP_CONFIG GetTcpConfig() const {return m_tcpConfig;}

	void SetCmmConfig(char *devPort, short size, short flag, int baudrate);
	void SetTcpConfig(char *ipaddr, short size, int portNum, short protocol, short flag);

	BOOL GetGCSIPAddr(char *fileName, char *gcsIPAddr);

	void SetViconFlag()		{ m_bVicon = TRUE; }
	BOOL GetViconFlag() const { return m_bVicon; }
protected:
	int SendViaNet(const char *host, char *buffer, int nBuffer);

protected:
	BOOL ReadCommand(COMMAND *pCmd);

	static void MakeTelegraph(TELEGRAPH *pTele, short code, double time, const void *pData, int nDataSize);

	static void MakeTelegraph(ADDRESSEDPACKAGE *pPackage, TELEGRAPH *pTele);

protected:
	static void *InputThread(void *pParameter);

	void Input();	// an always running thread every 20ms, receive commands and send reply message

public:
	BOOL StartInputThread(int priority);
};

#endif				//CMM_H_

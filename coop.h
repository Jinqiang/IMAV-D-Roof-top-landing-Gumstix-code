#ifndef COOP_H_
#define COOP_H_

#include <string.h>
#include <iostream.h>

#include "ctl.h"

#ifndef BOOL
#define BOOL int
#define TRUE 1
#define FALSE 0
#endif

#define NUM_COOPUAVS	3	// currently 2: HeLion + SheLion
#define ACTION_PARAS	11	// [t,x, y, z, c, u, v, w, acx, acy, acz]
#define COOPCMD_PARAS	4	// eg. "formation(1,2,3,4)", path(1) for leader, path(2) for follower1, ..etc.

#define ROLE_LEADER		6	// eg. HeLion
#define ROLE_FOLLOWER1	7	// eg. SheLion
#define ROLE_FOLLOWER2	8	// eg. HengLion

#define FORMATION_UPDATE	50		// 1 update pkt per second

//#define LEADERPATH_N		1	// path(1) set as leader's tracking path in formation at present
//#define FOLLOWERPATH_N		2	// current path(2) is ref of follower

enum coop_pkt_t {NONE, FORMATION_GS, ENQUERY, ADJUST, ACK, COOP_REPLY_FL, LEADERFORMATION_UPDATE, CONNECT_INIT_LD, \
		CONNECT_ACK_FL, CONNECT_ESTED_LD, READY_FL};
//enum coop_action_t {, , ATTACKCMD_ISSUED};

enum role_t {LEADER, FOLLOWER};
//enum role_t {LEADER, FOLLOWER1};

#pragma pack(push, 1)
struct COOP_PKT
{
	role_t enRole;
	coop_pkt_t enCoopType;
//	coop_action_t nAction;
	unsigned int npkt;
	int coopCmdParas[COOPCMD_PARAS];
	double actionParas[ACTION_PARAS];
};

struct COOPSTATE
{
	unsigned int ntx, nrx;
	int nFrom;
};

#pragma pack(pop)

struct BEHAVIOR;
//struct COOP_ADJUST
//{
//	int nRole;
//	double adjust[NUM_ADJUSTSTATES];	// adjust(x,y,z,c)
//};

class clsCoop : public clsThread
{
public:
	clsCoop();
	virtual ~clsCoop();
	
	virtual BOOL InitThread();
	virtual int EveryRun();
	virtual void ExitThread();
	
private:
	BOOL m_b1stCoop;
	BOOL m_bCoopStart;
	BOOL m_bConnect;
	
	role_t m_role;
	BEHAVIOR m_coopBeh;
	double m_leaderLong0, m_leaderLati0;
	// coop stats
	int m_nCoop;	// # cooperations btw leader follower
	int m_nReq, m_nRpl, m_nAck;
	double m_tCoop, m_tCoopStart;
	COMMAND m_coopCmdPkt;		// put the coop pkt in the cmm rcved pkg
	COOP_PKT m_coopPkt, m_coopedPkt;
	COOPSTATE m_coopState;
	pthread_mutex_t m_mtxPkt, m_mtxCoop;

public:
	void PutCoopPkt(COMMAND& coopPkt);
	void GetCoopPkt(COMMAND& coopPkt);
	double GetLeaderLong0() { return m_leaderLong0; }
	double GetLeaderLati0()	{ return m_leaderLati0; }
	BOOL CoopMethod();
	BOOL LdProcCoopPkt();
	BOOL FlProcCoopPkt();
	BOOL MakeCoopReqPkt(int nTo, string& strPkt);
//	BOOL CoopMethod(int Participents[], COOP_ADJUST coopOutput[NUM_COOPUAVS]);	// (coopOutput[0],x,y,z,c) - leader;
															// (coopOutput[1],x,y,z,c) - follower	
	void LdFormation();
	void FlFormation();
	
	void LdAttackMethod();
	void LdSync();		// currently for position synchronization
	BOOL GetFlRef(double ldStatus[4], double flRef[3]);

	
public:
//	BOOL ProcFlCoopPkt(int role);
	COOPSTATE GetCoopState() const { return m_coopState; }
	BOOL MakeCoopRplPkt(int nTo, string& strPkt);
	
	void SetCoop1st(BOOL bCoop1st) { m_bCoopStart = bCoop1st; }
	BOOL LogCoopStats();
	
	void SetConnectFlag()	{ m_bConnect = TRUE; }
	BOOL GetConnectFlag()	{ return m_bConnect; }
	void ResetConnectFlag()	{ m_bConnect = FALSE; }
	role_t GetRole() { return m_role; }
	void SetRole(int nID);
	double GetCoopTime() { return m_tCoop; }
	BOOL GetCoopStart()	{ return m_bCoopStart; }
	void MakeCoopPktNum(coop_pkt_t enCoopType, unsigned int npkt, int cmdPara[COOPCMD_PARAS], double actionPara[ACTION_PARAS]);
	void MakeCoopPkt(coop_pkt_t enCoopType, int cmdPara[COOPCMD_PARAS], double actionPara[ACTION_PARAS]);
	COOP_PKT FetchCoopedPkt() { return m_coopedPkt; }

public:
	BOOL AttackDropBall();		// drop a ball to attack the target
	
	friend class clsDLG;
	friend class clsCTL;
};

#endif /*COOP_H_*/

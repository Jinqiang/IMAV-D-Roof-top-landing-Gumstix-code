//coop.h

#include <pthread.h>

#include "uav.h"
#include "coop.h"
#include "net.h"
#include "state.h"

extern int _HELICOPTER;
extern clsState _state;

clsCoop::clsCoop()
{
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&m_mtxPkt, &attr);
	pthread_mutex_init(&m_mtxCoop, &attr);
	
	m_tCoop = -1;
	m_b1stCoop = FALSE;
	m_tCoopStart = -1;
	m_bCoopStart = FALSE;
	m_nCoop = 0;
}

clsCoop::~clsCoop()
{
	pthread_mutex_destroy(&m_mtxPkt);
	pthread_mutex_destroy(&m_mtxCoop);
}

BOOL clsCoop::InitThread()
{
	SetRole(_HELICOPTER);
	printf("[COOP] start %d\n", GetRole());
	return TRUE;
}

int clsCoop::EveryRun()
{
	COMMAND coopPkt;
	GetCoopPkt(coopPkt);
	if(coopPkt.code != 0)
	{
		if ( !CoopMethod() ) return FALSE;
		m_tCoop = ::GetTime();
	}

	return TRUE;
}

void clsCoop::SetRole(int nID)
{
	switch (nID) {
	case 6:
		m_role = LEADER;
		break;
	case 7:
		m_role = FOLLOWER;
		break;
	default:
		break;
	}
}
void clsCoop::PutCoopPkt(COMMAND& cmd)
{
	pthread_mutex_lock(&m_mtxPkt);
	if (cmd.code == COMMAND_FORMATION || cmd.code == COMMAND_COOP_PKT)
		m_coopCmdPkt = cmd;
	pthread_mutex_unlock(&m_mtxPkt);
	
}

void clsCoop::GetCoopPkt(COMMAND& coopCmdPkt)
{
	pthread_mutex_lock(&m_mtxPkt);
	
	if (m_coopCmdPkt.code == 0)
		coopCmdPkt.code = 0;
	else
	{
		coopCmdPkt = m_coopCmdPkt;
		m_coopCmdPkt.code = 0;
	} 
	
	pthread_mutex_unlock(&m_mtxPkt);
}

BOOL clsCoop::LdProcCoopPkt()	
{

	COOP_PKT m_coopPkt;
	::memcpy(&m_coopPkt, m_coopCmdPkt.parameter, sizeof(COOP_PKT) );
//	COOP_PKT coopPkt = (COOP_PKT &)m_coopPkt.parameter;
	int nLdPath,nFlPath;
	coop_pkt_t nCoopType = m_coopPkt.enCoopType;
	if ( nCoopType == FORMATION_GS )
	{
//		UAVSTATE& state = _state.GetState();
		
		printf("FORMATION_GS \n");
		m_coopBeh.behavior = BEHAVIOR_FORMATION;		// other coop behavior for other commands otherwise
		(int &)nLdPath = (int &)m_coopPkt.coopCmdParas[0];
		(int &)nFlPath = (int &)m_coopPkt.coopCmdParas[1];
		_ctl.m_pLdPath = _ctl.GetPath(nLdPath-1);
		_ctl.m_pFlPath = _ctl.GetPath(nFlPath-1);	// leader knows the path of the follower

		_ctl.m_behavior.behavior = BEHAVIOR_PATH;
		(int &)_ctl.m_behavior.parameter[0] = nLdPath;
		(int &)_ctl.m_behavior.parameter[4] = PATHTRACKING_ADDON;
		double longitude, latitude;
		_state.GetCoordination(&longitude, &latitude);
		double actionPara4Fl[11] = {longitude, latitude, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		MakeCoopPkt(CONNECT_INIT_LD, NULL, actionPara4Fl);
		COOP_PKT coopedPkt = FetchCoopedPkt();
		PEERADDRESS peerAddr;
		_net.FindDestPeers(_HELICOPTER, peerAddr);
		_net.ClientSendTo(peerAddr.ip /*PEER_HOST2*/, peerAddr.rxport /*PEER_HOST2_PORT_LISTEN*/, (char *)&coopedPkt, sizeof(COOP_PKT));
	}
	
	else if (nCoopType == CONNECT_ACK_FL)
	{
//		m_bCoopStart = TRUE;
		SetConnectFlag();
		printf("CONNECT_ACK_FL \n");
//		MakeCoopPkt(CONNECT_ESTED_LD, NULL, NULL);		
		m_coopPkt.enCoopType = NONE;		// reset
	}
	
	else if (nCoopType == COOP_REPLY_FL) {
//		printf("COOP_REPLY_FL \n");		// reserved for feedback processing
	}
	
	else if (nCoopType == READY_FL)
	{
//		printf("READY_FL \n");
		if ( m_coopBeh.behavior == BEHAVIOR_FORMATION && !m_b1stCoop )
		{
			_ctl.m_behavior.behavior = BEHAVIOR_PATH;
			(int &)_ctl.m_behavior.parameter[0] = nLdPath;
			(int &)_ctl.m_behavior.parameter[4] = PATHTRACKING_ADDON;
			LdFormation();
			m_b1stCoop = TRUE;
		}
//		else
//			LdFormation();
	}
		
//	LdFormation();
	
////		_ctl.m_fControl = CONTROLFLAG_B5 | CONTROLFLAG_A2;	// leader control blocks
//		(int &)nLdPath = (int &)m_coopPkt.coopCmdParas[0];
//		(int &)nFlPath = (int &)m_coopPkt.coopCmdParas[1];
//		_ctl.m_behavior.behavior = BEHAVIOR_PATH;
////		_ctl.m_behavior.behavior = BEHAVIOR_FORMATION;
//		(int &)_ctl.m_behavior.parameter[0] = nLdPath;
//		(int &)_ctl.m_behavior.parameter[4] = PATHTRACKING_ADDON;
//		
//		_ctl.m_pLdPath = _ctl.GetPath(nLdPath-1);
//		_ctl.m_pFlPath = _ctl.GetPath(nFlPath-1);	// leader knows the path of the follower
//		LdFormation();
	
//	else if ( nCoopType == ACK )		// coop pkt ack from follower
		
//	else /* if ( nCoopType == COOP_REPLY_FL ) */
//	{
//		printf("COOP_REPLY_FL \n");
//		LdFormation();
//	}
//	m_coopPkt.enCoopType = NONE;		// reset
		
	return TRUE;
}

BOOL clsCoop::FlProcCoopPkt()
{
	COOP_PKT m_coopPkt;
	::memcpy(&m_coopPkt, m_coopCmdPkt.parameter, sizeof(COOP_PKT) );
	coop_pkt_t nCoopType = m_coopPkt.enCoopType;
	unsigned int nPktSeq = m_coopPkt.npkt;	// pkt seq# from leader
	
	if ( nCoopType == CONNECT_INIT_LD )
	{
		m_leaderLong0 = m_coopPkt.actionParas[0];
		m_leaderLati0 = m_coopPkt.actionParas[1];
		printf("CONNECT_INIT_LD \n");
		MakeCoopPktNum(CONNECT_ACK_FL, nPktSeq, NULL, NULL);
		COOP_PKT coopedPkt = FetchCoopedPkt();
//		_net.ToggleNetStart();		// enable
		SetConnectFlag();
		PEERADDRESS peerAddr;
		_net.FindDestPeers(_HELICOPTER, peerAddr);
		_net.ClientSendTo(peerAddr.ip /*PEER_HOST2*/, peerAddr.rxport /*PEER_HOST2_PORT_LISTEN*/, (char *)&coopedPkt, sizeof(COOP_PKT));
//		_net.ToggleNetStart();		// disable
//		m_coopPkt.enCoopType = NONE;
	}
		
	else if (nCoopType == CONNECT_ESTED_LD)
	{
		printf("CONNECT_ESTED_LD \n");
		MakeCoopPktNum(READY_FL, nPktSeq, NULL, NULL);
	}

	else if ( nCoopType == LEADERFORMATION_UPDATE )
	{
//		printf("LEADERFORMATION_UPDATE \n");
//		_ctl.m_fControl = CONTROLFLAG_B5 | CONTROLFLAG_A2;		// follower runs the path tracking behavior
//		_ctl.m_behavior.behavior = BEHAVIOR_PATH;

//		(int &)_ctl.m_behavior.parameter[4] = PATHTRACKING_FIXED;
//		_pathTmp.AddPathPoint((double &)m_coopPkt.actionParas[0], (double *)(m_coopPkt.actionParas + 8) );
		
		int nIndex = _pathTmp.GetMatrix().GetM();
		if (nIndex == 0)	// first coop point
		{	
//			_ctl.m_behavior.behavior = BEHAVIOR_TEST;
//			(int &)_ctl.m_behavior.parameter[0] = 4;	// test(4)
			
//			_ctl.SetBehavior(BEHAVIOR_PATH, -1, PATHTRACKING_FIXED);
			
			_ctl.m_behavior.behavior = BEHAVIOR_PATH;
			(int &)_ctl.m_behavior.parameter[0] = -1;
			(int &)_ctl.m_behavior.parameter[4] = PATHTRACKING_FIXED;
			
//			_ctl.B5_t0 = ::GetTime();
			m_tCoopStart = /*m_coopPkt.actionParas[0]*/ ::GetTime();
			_pathTmp.AddPathPointRPT(0.0, &(m_coopPkt.actionParas[1]), &(m_coopPkt.actionParas[5]),	&(m_coopPkt.actionParas[8]));
		}
		else {
			double t = ::GetTime();
			double dt = t - m_tCoopStart;
			m_tCoopStart = t;
			_pathTmp.AddPathPointRPT(dt /*0.2*/, &m_coopPkt.actionParas[1], &m_coopPkt.actionParas[5], &m_coopPkt.actionParas[8]);
			
//			cout<<"[clsCoop] x "<<m_coopPkt.actionParas[1]<<" y "<<m_coopPkt.actionParas[2]<<" z "<<m_coopPkt.actionParas[3]<<endl;
//			cout<<"[clsCoop] u "<<m_coopPkt.actionParas[5]<<" v "<<m_coopPkt.actionParas[6]<<" w "<<m_coopPkt.actionParas[7]<<endl;
		}
		
		MakeCoopPktNum(COOP_REPLY_FL, nPktSeq, NULL, NULL);
		COOP_PKT coopedPkt = FetchCoopedPkt();
//		_net.ToggleNetStart();		// enable
		PEERADDRESS peerAddr;
		_net.FindDestPeers(_HELICOPTER, peerAddr);
		_net.ClientSendTo(peerAddr.ip /*PEER_HOST2*/, peerAddr.rxport /*PEER_HOST2_PORT_LISTEN*/, (char *)&coopedPkt, sizeof(COOP_PKT));
//		_net.ToggleNetStart();		// disable
	}
	return TRUE;
}

BOOL clsCoop::CoopMethod()
{
	role_t role = GetRole();
	
	if ( role == LEADER )
		LdProcCoopPkt();
	else if ( role == FOLLOWER )
		FlProcCoopPkt();
	else 
		return FALSE;
	
	return TRUE;
}

void clsCoop::LdAttackMethod()
{
	
}

void clsCoop::FlFormation()
{
	UAVSTATE& state = _state.GetState();
	int cmdPara4Fl[4] = {0};
	double flStatus[9] = {state.x, state.y, state.z, state.c, 0, 0, 0, 0, 0};
	MakeCoopPkt(COOP_REPLY_FL, cmdPara4Fl, flStatus);
}

void clsCoop::LdFormation()
{
//	if (m_nCount % FORMATION_UPDATE != 0)	// currently leader just sends ref to follower without interacting
//		return;
	
	// below give the follower ref given leader's status (time_in_path,x,y,z)
//	double tPath = _ctl.B5_tPath;  //GetTrackTime();
	UAVSTATE& state = _state.GetState();
//	RPTSTATE& RPTState = _state.GetRPTState();
	
	
	double t = ::GetTime();
	double ldStatus[11] = {/*tPath*/ t, state.x, state.y, state.z, state.c, state.ug, state.vg, state.wg, state.p, state.q, state.r};
	double flRef[4];
	GetFlRef(ldStatus, flRef);
//	int cmdPara4Fl[4] = {0};
//	double actionPara4Fl[9] = {flRef[0], flRef[1], flRef[2], flRef[3], 0, 0, 0, 0, 0};
//	MakeCoopPkt(LEADERFORMATION_UPDATE, cmdPara4Fl, actionPara4Fl);
	MakeCoopPkt(LEADERFORMATION_UPDATE, NULL, ldStatus);
		
}

void clsCoop::LdSync()
{
//	double flPos[3] = { (double &)m_coopPkt.actionParas[0], (double &)m_coopPkt.actionParas[8], (double &)m_coopPkt.actionParas[16] };
	
}

BOOL clsCoop::GetFlRef(double ldStatus[4], double flRef[3])
{
	double tPath = ldStatus[0];
	double ldRef[3];
	if (_ctl.m_pLdPath == NULL || _ctl.m_pFlPath == NULL) return FALSE;
	
	_ctl.B5_pPath->GetPositionVelocity(tPath, ldRef, NULL, FALSE);
	
	double ldPos[3] = {ldStatus[1], ldStatus[2], ldStatus[3]};
	
//	clsMatrix path = *(_ctl.m_pFlPath);
	double tTimeRef = _ctl.m_pLdPath->GetMappedRefTimeOnNorm(tPath, ldPos);
	
	double flRefPos[3] = {0,0,0};
	_ctl.m_pFlPath->GetPositionVelocity(tTimeRef, flRefPos, NULL, FALSE);

	return TRUE;
}

void clsCoop::MakeCoopPkt(coop_pkt_t enCoopType, int cmdPara[COOPCMD_PARAS], double actionPara[ACTION_PARAS])
{
	::memset(&m_coopedPkt, 0, sizeof(m_coopedPkt));
	m_coopedPkt.enRole = GetRole();
	m_coopedPkt.enCoopType = enCoopType;
	m_coopState.ntx = m_coopedPkt.npkt = ++m_nCoop;
	
	if (cmdPara != NULL)
	{
//		for (int i=0; i<COOPCMD_PARAS; i++)
//			m_coopedPkt.coopCmdParas[i] = cmdPara[i];
		::memcpy(m_coopedPkt.coopCmdParas, cmdPara, COOPCMD_PARAS*sizeof(int));
	}
	
	if (actionPara != NULL)
	{
//		for (int i=0; i<ACTION_PARAS; i++)
//			m_coopedPkt.actionParas[i] = actionPara[i];
		::memcpy(m_coopedPkt.actionParas, actionPara, ACTION_PARAS*sizeof(double));
	}	
}

void clsCoop::MakeCoopPktNum(coop_pkt_t enCoopType, unsigned int npkt, int cmdPara[COOPCMD_PARAS], double actionPara[ACTION_PARAS])
{
	m_coopedPkt.enRole = GetRole();
	m_coopedPkt.enCoopType = enCoopType;
	m_coopState.nrx = m_coopedPkt.npkt = npkt;

	if (cmdPara != NULL)
	{
//		for (int i=0; i<COOPCMD_PARAS; i++)
//			m_coopedPkt.coopCmdParas[i] = cmdPara[i];
		::memcpy(m_coopedPkt.coopCmdParas, cmdPara, COOPCMD_PARAS*sizeof(int));
	}
	
	if (actionPara != NULL)
	{
//		for (int i=0; i<ACTION_PARAS; i++)
//			m_coopedPkt.actionParas[i] = actionPara[i];
		::memcpy(m_coopedPkt.actionParas, actionPara, ACTION_PARAS*sizeof(double));
	}	
}

void clsCoop::ExitThread()
{
	cout<<"[COOP] quit"<<endl;
}

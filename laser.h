#ifndef LASER_H_
#define LASER_H_

#include "state.h"
#include "im9.h"
#include "height.h"
#include "slam.h"

#define MAX_URG	128
#define MAX_URG_PT 1081
#define MAX_URG_4 769

#define NUSUAV_LASERH_OFFSET_TO_GROUND 400
#define NUSUAV_LASERH_HEIGHT_START_INDEX 1041
#define NUSUAV_LASERH_HEIGHT_END_INDEX 1045



struct LASER_CONFIG {
	char laserDevice[32];
	short flag;
};

extern clsState _state;

struct LASERRAWDATA
{
	long range[MAX_URG_PT];
};

struct LASERRAWDATA4
{
	long range[MAX_URG_4];
};
#pragma pack(push, 1)

struct LASERINFO
{
	BOOL on_off;
	BOOL updated;
	double x, y, z;
	double psi;
};
#pragma pack(pop)


// Class URG defined here
class clsURG : public clsThread {
protected:
	pthread_mutex_t m_mtxURG;
	int m_nURG;

	double m_tURG[MAX_URG];
	long ranges[MAX_URG][MAX_URG_PT];
	long ranges4[MAX_URG][MAX_URG_4];
	long range[MAX_URG_PT];
	LASER_CONFIG m_laserConfig;

private:
	double x_log[MAX_URG];
	double y_log[MAX_URG];
	double z_log[MAX_URG];
	double phi_log[MAX_URG];
	double tht_log[MAX_URG];
	double psi_log[MAX_URG];

	LASERRAWDATA m_laser_rawData;
	LASERRAWDATA4 m_laser_rawData4;
	double m_tURG0;
	double goodMeasurement;
	int jumpCount;
	bool firstMeasurement;
public:
	void SetLaserConfig(char *laserDevice, short size, short flag);
	LASER_CONFIG GetLaserConfig() const {return m_laserConfig;}

	double getURGTime(){return m_tURG0;}
	LASERRAWDATA &getLaserData()
	{
		for(int i=0; i<MAX_URG_PT; i++ )
			m_laser_rawData.range[i]=ranges[1][i];
			//m_laser_rawData.range[i]=0;
		return m_laser_rawData;
	}
	LASERRAWDATA4 &getLaserData4()
	{
		UAVSTATE state = _state.GetState();
		m_laser_rawData4.range[0] =(long) (state.x*1000);
		m_laser_rawData4.range[1] =(long) (state.y*1000);
		m_laser_rawData4.range[2] =(long) (state.z*1000);
		m_laser_rawData4.range[3] =(long) (state.a*1000);
		m_laser_rawData4.range[4] =(long) (state.b*1000);
		m_laser_rawData4.range[5] =(long) (state.c*1000);
		m_laser_rawData4.range[6] =(long) (laser_angle*1000);
		for(int i=7; i<MAX_URG_4; i++ )
			m_laser_rawData4.range[i]=ranges4[1][i];
			//m_laser_rawData.range[i]=0;
		//printf("[Laser Data]:%d\n",m_laser_rawData4.range[0] );
		return m_laser_rawData4;
	}

	virtual BOOL InitThread();
	virtual int EveryRun();
	virtual void ExitThread();
public:
	clsURG();
	long int range_4[MAX_URG_4];
	void request_urg4(long int range_4[]);
	void request_urg30(long int range_30[]);
	void Process_URG4(long int range_4[], char buf[], int size);
	void Process_URG30(long int range_30[], char buf[], int size);
	double GetHeight();
	void CheckHeightJump();
	int _urg4, _urg30;
	int sdsdf;

	slam SAFMC;
	height SAFMC_height;
	FILE *infile30;
    FILE *infile4;
    FILE *outfile4;
	int start_localisation;
    double input[1086];
	virtual ~clsURG();
	void set_start_localisation(int i);
	void Init_URG_30();
	void Init_URG_4();
public:
	LASERINFO m_info;
	LASERINFO &GetInfo() { return m_info; }
	double laser_angle;
	void set_laser_angle(double x){laser_angle = x;}
public:
//	friend class clsMAP;
	friend class clsDLG;
};

#endif /* LASER_H_ */

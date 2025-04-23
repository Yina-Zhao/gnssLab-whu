/****************************************************************************
目的：    定义DGPS流动站软件需要的所有结构体
编写时间：20011.7.10
作者：    王甫红
版本:     V2.0
版权：    武汉大学测绘学院
****************************************************************************/

#ifndef _DecodeStruct_H_
#define _DecodeStruct_H_

#include "TimeStruct2.h"
#include "DecodeConst.h"

/* 导航卫星系统定义 */
enum GNSSSys { UNKS=0, GPS, BDS, GLONASS, GALILEO};

/* 观测数据类型定义 */
enum OBSTYPE  { UNKOBS=0, C1, P1, L1, D1, C2, P2, L2, D2, C3, L3, D3, S1, S2 };


struct IONOPARA
{
    double alpha[4];        /* 电离层改正模型参数 alpha */
    double beta[4];         /* 电离层改正模型参数 beta  */
    bool   IsValid;         /* 电离层改正参数有效为true */

    IONOPARA()
    {
        IsValid = false;
        for(int i=0; i<4; i++)   alpha[i] = beta[i] = 0.0;
    }
};

struct GPSEPHREC
{
    short       PRN;
    GNSSSys     System;
    GPSTIME  	TOC;
    double		ClkBias;
    double		ClkDrift;
    double		ClkDriftRate;
    double		IODE;
    double		DetlaN;
    double		M0;
    double		e;
    double		SqrtA;
    GPSTIME	    TOE;
    double		Crs;
    double		Cuc;
    double		Cus;
    double		Cic;
    double		Cis;
    double		Crc;
    double		OMEGA;
    double		i0;
    double		omega;
    double		OMEGADot;
    double		iDot;
    double		SVAccuracy;
    double		SVHealth;
    double		TGD1, TGD2;
    double		IODC;

    GPSEPHREC()
    {
        PRN = 0;
        System = GNSSSys::UNKS;
        IODE = IODC = 0;
        ClkBias = ClkDrift = ClkDriftRate = SqrtA = M0 = e = OMEGA = i0 = omega = Crs = Cuc=
                Cus = Cic = Cis = Crc = TGD1 = TGD2 = 0.0;
    }
};


/*  每颗卫星的观测数据定义  */
struct SATOBSDATA
{
    short           Prn;
    GNSSSys         System;
    double   c1,p2,l1,l2,d1;

    SATOBSDATA()
    {
        Prn = 0;
        System = UNKS;
        c1=p2=l1=l2=d1=0.0;
    }
};

/*  每个历元的观测数据定义  */
struct EPOCHOBSDATA
{
    GPSTIME           Time;
    short             EpochFlag;
    short             SatNum;
    SATOBSDATA        SatObs[MAXCHANNUM];
    double            Pos[3];      // 保存NovAtel接收机定位结果

    EPOCHOBSDATA()
    {
        EpochFlag = 0;
        SatNum    = 0;
        Pos[0] = Pos[1] = Pos[2] = 0.0;
    }
};

/* 定位计算(包括单点定位和滤波定位)需要用到的每颗卫星的中间计算结果 */
struct SATMIDRESULT
{
    GPSTIME Ttr;           // 信号发射时刻的GPS时间
    double SatPos[3];
    double SatVel[3];
    double SatClkOft;
    double SatClkSft;
    double Elevation;
    double IonoCorr;
    double TropCorr;
    double Tgd;
    short Valid;			//整个中间信息是否可用:0-没有星历,1-计算正常,2-切换，使用广播星历时有意义
    double IODE;

    SATMIDRESULT()
    {
        SatPos[0]=SatPos[1]=SatPos[2]=0.0;
        SatVel[0]=SatVel[1]=SatVel[2]=0.0;
        Elevation=pi/2.0;
        SatClkOft=0.0;
        SatClkSft=0.0;
        Tgd = IonoCorr=0.0;
        TropCorr = 0.0;
        IODE = 0.0;
        Valid=0;
    }
};

/*  单点定位的卫星列表  */
struct SATLIST
{
    short   Prn;
    GNSSSys System;
    short   NumFQ;  // 频率数，双频组合为2，单频为1
    double  PIF;    // 伪距无电离层组合观测值
    SATMIDRESULT Mid;
    short   PosSts, VelSts;   // -3:星历计算失败;-1:滤波前检测的周跳;1:连续跟踪;2:新加入卫星;3:广播星历切换
    bool    ObsValid;

    SATLIST()
    {
        Prn = 0;
        System = UNKS;
        NumFQ  = 0;
        PosSts = VelSts = 0;
        PIF=0.0;
        ObsValid=false;
    }
};

/* 预报的接收机先验位置 */
struct APRPOS
{
    double Pos[4];
    double Vel[4];
    double Accu;
    bool   Valid;

    APRPOS()
    {
        for(int i=0;i<4;i++)   Pos[i]=Vel[i]=0.0;
        Accu = 999.0;
        Valid = false;
    }
};

/* 每个历元单点定位和测速的结果及其精度指标 */
struct PPRESULT
{
    GPSTIME Time;
    double Position[3];
    double Velocity[3];
    double RcvClkOft;         /* 0 为GPS钟差  */
    double RcvClkSft;
    double dSysClkOft;       /* GPS与北斗时间系统之间的差异 */
    double PDOP;
    double SigmaPos;
    double SigmaVel;
    SATLIST SatList[MAXCHANNUM];     /* 单点定位卫星列表, 和观测值顺序保持一致 */
    double PrePos[3];                // 先验位置信息
    short  Iterator;                 /* 单点定位迭代次数 */
    double Coverage;                 /* 单点定位收敛数据 */
    short  GPSSatNum, BDSSatNum;                /* 单点定位使用的GPS卫星数 */
    short  AllSatNum;                /* 观测历元的所有卫星数   */
    bool   IsSuccess;                /* 单点定位是否成功, 1为成功, 0为失败 */

    PPRESULT()
    {
        for (int i=0; i<3; i++)		Position[i] = Velocity[i] = 0.0;
        RcvClkOft = RcvClkSft = dSysClkOft = 0.0;
        PDOP = SigmaPos = SigmaVel = 999.9;

        Coverage = 999.9;
        Iterator = 20;
        GPSSatNum = BDSSatNum = AllSatNum = 0;
        IsSuccess = false;
    }
};

struct ROVERCFGINFO   // 配置信息
{
    short  IsFileData;       // 1=FILE, 0=COM
    int    Port, Baud;
    char   NetIP[20];        // ip address
    short  NetPort;          // port
    double CodeNoise;        // 伪距噪声
    double ElevThreshold;           // 高度角阈值

    char  ObsDatFile[256];         //  观测数据的文件名
    char  ResFile[256];            //  结果数据文件名
    char  DiffFile[256];           //  定位结果与接收机位置差值文件名

    ROVERCFGINFO()
    {
        IsFileData = 1;
        Port = Baud = NetPort = 0;
        CodeNoise = ElevThreshold = 0.0;
    }
};

#endif

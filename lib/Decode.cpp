//****************************************************************************
// decode the OEM719 RANGEData/GPSEPH/BDSEPH ...
//
// this code is modified from code by written by Wang Fuhong, SGG, Wuhan University
//
// 2024/11/05
//****************************************************************************/

#include <memory.h>
#include <math.h>
#include "Decode.h"
#include "CoordConvert2.h"
#include "GnssStruct.h"
#include "NavEphGPS.hpp"
#include "NavEphBDS.h"
#include <iostream>

using namespace std;

IONOPARA IonPara;

/* crc-32 parity ---------------------------------------------------------------
* compute crc-32 parity for novatel raw
* args   : unsigned char *buff I data
*          int    len    I      data length (bytes)
* return : crc-32 parity
* notes  : see NovAtel OEMV firmware manual 1.7 32-bit CRC
*-----------------------------------------------------------------------------*/
unsigned int crc32(const unsigned char* buff, int len)
{
    int i, j;
    unsigned int crc = 0;

    for (i = 0; i < len; i++) {
        crc ^= buff[i];
        for (j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ POLYCRC32;
            else crc >>= 1;
        }
    }
    return crc;
}

int decode_rangeb_oem7(unsigned char *buff, ObsData* obsData)
{
    double waveLen;//波长
    unsigned int track;//跟踪状态字
    int i, j, m, n;//一些临时变量
    int nobs;//观测值数量
    int prn, sat;
    //锁相标志，钟跳标志，奇偶校验标志，信号类型，半周期标志
    unsigned char freq, plockf, parityf, clockf, sigtype, halfc;
    GNSSSys sys;

    unsigned char *p = buff + OEM7HeaderLength;//跳过头数据的读取，因为已经读取过了

    nobs = U4(p);//观测值数量
    n = 0;
    SatTypeValueMap stvData;
    TypeValueMap typeObsGPS, typeObsBDS;

    std::vector<SatID> satIndex(nobs);
    for (i = 0, p+=4; i < nobs; i++, p+=44){
        track = U4(p+40);
        switch ((track >> 16)&7) {//提取系统标识符
            case 0:
                sys = GPS;
                satIndex[i].system = "G";
                break;
            case 4:
                sys = BDS;
                satIndex[i].system = "C";
                break;
            default:
                sys = UNKS;
                break;

        }

        sat = U2(p);
        if (sys == GPS || sys == BDS)
            satIndex[i].id = sat;//todo：这里相当于完成了SatID的读取，要想办法和ObsData里面对应起来
        else continue;

        plockf = (track>>10)&1;
        clockf = (track>>12)&1;
        parityf = (track>>11)&1;
        halfc = (track>>28)&1;
        sigtype = (track>>21)&0x1F;//观测值类型

        if (sys == GPS){
            switch (sigtype) {
                case 0:
                    freq=0;
                   // typeObsStr = "L1";todo：这里我犯了一个错误，这里只是频率，频率1包括了伪距值和载波相位值
                    break;   // L1C/A
                    //	case 5: Freq=1; break;   // L2P
                case 9:
                    freq=1;
                 //   typeObsStr = "L2";
                    break;   // L2P(Y),semi-codeless
                    //	case 14: Freq=2; break;  // L5(Q)
                    //	case 16: Freq=1; break;  // L1C(P)
                    //	case 17: Freq=1; break;  // L2C(M)
                default: freq=-1; break;

            }
        }

        if (sys == BDS){
            switch (sigtype) {

                case 0:
                    freq=0;
                   // typeObsStr = "L2";
                    break;  // B1I D1
                    //	case 1: Freq=1; break;  // B2I D1
                case 2:
                    freq=1;
                  //  typeObsStr = "L7";
                    break;  // B3I D1
                case 4:
                    freq=0;
                  //  typeObsStr = "L2";
                    break;  // B1I D2
                    //	case 5: Freq=1; break;  // B2I D2
                case 6:
                    freq=1;
                 //   typeObsStr = "L7";
                    break;  // B3I D2
                    //case 7: Freq=-1; break;  // B1C
                    //case 9: Freq=-2; break;  // B2a
                default: freq=-1; break;

            }
        }

        if(freq==-1 || freq>=2)  continue;

        m = n;
        //todo；现在要开始往obsData里面存数据了
        if (freq != 0){//因为按照顺序是先解码1频率的
            for (j = 0; j < MAXCHANNUM; j++){
                if (satIndex[j].system == satIndex[i].system && satIndex[j].id == sat){
                    m = j;
                    break;
                }
            }
        }

        satIndex[m].system = satIndex[i].system;
        satIndex[m].id = sat;
        //计算波长
        if(sys==GPS)
        {
            if(prn>MAXGPSPRN)  continue;
            waveLen=freq==0? C_Light/FG1_GPS : C_Light/FG2_GPS;
        }
        if(sys==BDS){
            if(prn>MAXBDSPRN)  continue;
            waveLen=freq==0? C_Light/FG1_CPS : C_Light/FG3_CPS;
        }

        if (freq == 0){
            if (sys == GPS){
                typeObsGPS["C1"] = (!clockf)? 0.0 : R8(p+4);
                typeObsGPS["L1"] = (!plockf)? 0.0 : -R8(p+16)*waveLen;  // half cycle
                typeObsGPS["D1"] = R4(p+28)*waveLen;
                stvData[satIndex[m]] = typeObsGPS;
                obsData->satTypeValueData = stvData;
               // cout << "GPS C1 L1" << obsData;
            }
            if (sys == BDS){
                typeObsBDS["C2"] = (!clockf)? 0.0 : R8(p+4);
                typeObsBDS["L2"] = (!plockf)? 0.0 : -R8(p+16)*waveLen;  // half cycle
                typeObsBDS["D2"] = R4(p+28)*waveLen;
                stvData[satIndex[m]] = typeObsBDS;
                obsData->satTypeValueData = stvData;
            }
            n++;
        }

        if (freq == 1){
            if (sys == GPS){
                typeObsGPS["C2"] = (!clockf)? 0.0 : R8(p+4);
                typeObsGPS["L2"] = (!plockf)? 0.0 : -R8(p+16)*waveLen;  // half cycle
                stvData[satIndex[m]] = typeObsGPS;
                obsData->satTypeValueData = stvData;
            }
            if (sys == BDS){
                typeObsBDS["C7"] = (!clockf)? 0.0 : R8(p+4);
                typeObsBDS["L7"] = (!plockf)? 0.0 : -R8(p+16)*waveLen;  // half cycle
                stvData[satIndex[m]] = typeObsBDS;
                obsData->satTypeValueData = stvData;
            }
        }
    }
    obsData->SatNum = n;
    return  1;
}

int decode_rangeb_oem6(unsigned char *buff, EPOCHOBSDATA* obs)
{
    double wl;
    unsigned int track;
    int i, j, m, n, nobs, prn, sat;
    unsigned char Freq, plockf, parityf, clockf, sigtype, halfc;
    GNSSSys sys;
    unsigned char *p=buff+OEM7HeaderLength;

    nobs=U4(p);
    n=0;
    for (i=0,p+=4;i<nobs;i++,p+=44)
    {
        track=U4(p+40);
        switch((track>>16)&7)
        {
            case 0: sys=GPS;     break;
//		case 3: sys=GALILEO; break;
            case 4: sys=BDS;     break;
//		case 5: sys=QZSS;    break;
            default:  sys= UNKS; break;
        }

        sat=U2(p);
        if(sys==GPS || sys==BDS)  prn=sat;
        else continue;

        plockf = (track>>10)&1;
        clockf = (track>>12)&1;
        parityf = (track>>11)&1;
        halfc = (track>>28)&1;
        sigtype = (track>>21)&0x1F;

        if(sys==GPS)
        {
            switch(sigtype)
            {
                case 0: Freq=0; break;   // L1C/A
                    //	case 5: Freq=1; break;   // L2P
                case 9: Freq=1; break;   // L2P(Y),semi-codeless
                    //	case 14: Freq=2; break;  // L5(Q)
                    //	case 16: Freq=1; break;  // L1C(P)
                    //	case 17: Freq=1; break;  // L2C(M)
                default: Freq=-1; break;
            }
        }
        if(sys==BDS)
        {
            switch(sigtype)
            {
                case 0: Freq=0; break;  // B1I D1
                    //	case 1: Freq=1; break;  // B2I D1
                case 2: Freq=1; break;  // B3I D1
                case 4: Freq=0; break;  // B1I D2
                    //	case 5: Freq=1; break;  // B2I D2
                case 6: Freq=1; break;  // B3I D2
                    //case 7: Freq=-1; break;  // B1C
                    //case 9: Freq=-2; break;  // B2a
                default: Freq=-1; break;
            }
        }

        if(Freq==-1 || Freq>=2)  continue;

        m=n;
        if(Freq!=0)  // ����L1�۲�ֵ�����λ��
        {
            for(j=0;j<=MAXCHANNUM;j++)
            {
                if(obs->SatObs[j].Prn==prn&&obs->SatObs[j].System==sys){
                    m=j;
                    break;
                }
            }
        }

        obs->SatObs[m].Prn=prn;
        obs->SatObs[m].System=sys;
        if(sys==GPS)
        {
            if(prn>MAXGPSPRN)  continue;
            wl=Freq==0? C_Light/FG1_GPS : C_Light/FG2_GPS;
        }
        if(sys==BDS){
            if(prn>MAXBDSPRN)  continue;
            wl=Freq==0? C_Light/FG1_CPS : C_Light/FG3_CPS;
        }

        if(Freq==0){
            obs->SatObs[m].c1=(!clockf)? 0.0 : R8(p+4);
            obs->SatObs[m].l1=(!plockf)? 0.0 : -R8(p+16)*wl;  // half cycle
            obs->SatObs[m].d1=R4(p+28)*wl;
            n++;
        }
        if(Freq==1){
            obs->SatObs[m].p2=(!clockf)? 0.0 : R8(p+4);
            obs->SatObs[m].l2=(!plockf)? 0.0 : -R8(p+16)*wl;  // half cycle

        }
    }
    obs->SatNum=n;
    return 1;
}


/* decode decode_gpsephem ------------------------------------------------------*/
int decode_GPSEph(unsigned char* buff, NavEphGPS* navEphGPS)
{
    NavEphGPS* originalPtr = navEphGPS;//保存原始指针地址
    unsigned char *p = buff + OEM7HeaderLength;
    int prn;
    prn = U4(p);    p+=4;



    if(prn<1 || prn>MAXGPSPRN)
    {
        return 1;
    }

    navEphGPS = navEphGPS + prn -1;
    navEphGPS->prn = prn;
    navEphGPS->beginValid.m_timeSystem = TimeSystem::GPS;
    navEphGPS->endValid.m_timeSystem = TimeSystem::GPS;
    navEphGPS->Toc = R8(p); p+=8;
    navEphGPS->SV_health = U4(p);   p+=4;
    navEphGPS->IODE = U4(p);    p+=8;
    navEphGPS->GPSWeek = U4(p); p+=8;
    navEphGPS->Toe = R8(p); p+=8;
    navEphGPS->sqrt_A = sqrt(R8(p));    p+=8;
    navEphGPS->Delta_n = R8(p); p+=8;
    navEphGPS->M0 = R8(p);  p+=8;
    navEphGPS->ecc = R8(p); p+=8;
    navEphGPS->omega = R8(p);   p+=8;
    navEphGPS->Cuc = R8(p); p+=8;
    navEphGPS->Cus = R8(p); p+=8;
    navEphGPS->Crc = R8(p); p+=8;
    navEphGPS->Crs = R8(p); p+=8;
    navEphGPS->Cic = R8(p); p+=8;
    navEphGPS->Cis = R8(p); p+=8;
    navEphGPS->i0 = R8(p);  p+=8;
    navEphGPS->IDOT = R8(p);    p+=8;
    navEphGPS->OMEGA_0 = R8(p); p+=8;
    navEphGPS->OMEGA_DOT = R8(p);   p+=8;
    navEphGPS->IODC = U4(p);    p+=4;
    navEphGPS->Toc = R8(p); p+=8;
    navEphGPS->TGD = R8(p); p+=8;
    navEphGPS->af0       = R8(p); p+=8;
    navEphGPS->af1      = R8(p); p+=8;
    navEphGPS->af2  = R8(p); p+=20;
    navEphGPS->URA    = R8(p);

    WeekSecond weekSecond;
    weekSecond.week =  navEphGPS->GPSWeek;
    weekSecond.sow = navEphGPS->Toc;
    WeekSecond2CommonTime(weekSecond, navEphGPS->ctToc);
    navEphGPS->ctToe = navEphGPS->ctToc;
    navEphGPS->ctToe.setTimeSystem(TimeSystem::GPS);

   /* cout << "G" <<navEphGPS->prn << " ";
    cout << "omega: " << navEphGPS->omega << endl;
    cout << "ecc: " << navEphGPS->ecc << " ";
    cout << "M0: " << navEphGPS->M0 << endl;
    cout << "GPSWeek: " << navEphGPS->GPSWeek << "GPSToc: " << navEphGPS->Toc << "GPSToe" << navEphGPS->Toe << endl;*/


    navEphGPS = originalPtr;
    return 2;


}
int decode_gpsephem(unsigned char* buff, GPSEPHREC* GPS)
{
    unsigned char *p=buff+OEM7HeaderLength;
    int prn;

    GPSEPHREC* gps;

    prn       =U4(p);           p+=4;
    gps = gps+prn-1;//todo:这里有个问题，我这里改变了地址，再次传入时需要重新初始化
    gps->PRN = prn;
    gps->System = GNSSSys::GPS;
    gps->TOC.SecOfWeek = R8(p); p+=8;
    gps->SVHealth      = U4(p); p+=4;
    gps->IODE          = U4(p); p+=8;
    gps->TOC.Week =gps->TOE.Week = U4(p); p+=8;
    gps->TOE.SecOfWeek = R8(p); p+=8;
    gps->SqrtA         = sqrt(R8(p)); p+=8;
    gps->DetlaN        = R8(p); p+=8;
    gps->M0            = R8(p); p+=8;
    gps->e             = R8(p); p+=8;
    gps->omega         = R8(p); p+=8;
    gps->Cuc           = R8(p); p+=8;
    gps->Cus           = R8(p); p+=8;
    gps->Crc           = R8(p); p+=8;
    gps->Crs           = R8(p); p+=8;
    gps->Cic           = R8(p); p+=8;
    gps->Cis           = R8(p); p+=8;
    gps->i0            = R8(p); p+=8;
    gps->iDot          = R8(p); p+=8;
    gps->OMEGA         = R8(p); p+=8;
    gps->OMEGADot      = R8(p); p+=8;
    gps->IODC          = U4(p); p+=4;
    gps->TOC.SecOfWeek = R8(p); p+=8;
    gps->TGD1          = R8(p); p+=8;
    gps->ClkBias       = R8(p); p+=8;
    gps->ClkDrift      = R8(p); p+=8;
    gps->ClkDriftRate  = R8(p); p+=20;
    gps->SVAccuracy    = R8(p);

    cout << gps->PRN << " " << "gps sqrt_A: " << gps->SqrtA << " " << "gps omega: " << gps->omega << endl;

    return 2;
}


/****************************************************************************
  decode_bdsephem

  Ŀ�ģ�����BDS�㲥�������ݣ�������OEM6

  ����:
  buff    ���ݻ�����
  eph     BDS�㲥����
****************************************************************************/

int decode_BDSEph(unsigned char* buff, NavEphBDS* bd)
{
    NavEphBDS* originalPtr = bd;
    unsigned char *p=buff+OEM7HeaderLength;
    int prn;


    prn       =U4(p);   p+=4;
    if(prn<1 || prn>MAXBDSPRN)
        return 1;

    bd = bd+prn-1;//todo:这里有个问题，我这里改变了地址，再次传入时需要重新初始化
    bd->beginValid.m_timeSystem = TimeSystem::BDT;
    bd->endValid.m_timeSystem = TimeSystem::BDT;
    bd->prn = prn;
    bd->BDSWeek = U4(p)+1356; p+=4;
    bd->URA    = R8(p); p+=8;
    bd->SV_health      = U4(p); p+=4;
    bd->TGD1          = R8(p); p+=8;
    bd->TGD2          = R8(p); p+=12;
    bd->Toc = U4(p); p+=4;
    bd->af0       = R8(p); p+=8;
    bd->af1      = R8(p); p+=8;
    bd->af2  = R8(p); p+=12;
    bd->Toe = U4(p); p+=4;
    bd->sqrt_A         = R8(p); p+=8;
    bd->ecc             = R8(p); p+=8;
    bd->omega         = R8(p); p+=8;
    bd->Delta_n        = R8(p); p+=8;
    bd->M0            = R8(p); p+=8;
    bd->OMEGA_0         = R8(p); p+=8;
    bd->OMEGA_DOT      = R8(p); p+=8;
    bd->i0            = R8(p); p+=8;
    bd->IDOT          = R8(p); p+=8;
    bd->Cuc           = R8(p); p+=8;
    bd->Cus           = R8(p); p+=8;
    bd->Crc           = R8(p); p+=8;
    bd->Crs           = R8(p); p+=8;
    bd->Cic           = R8(p); p+=8;
    bd->Cis           = R8(p); p+=8;
    bd->AODE = bd->AODC = (int)(bd->Toe + 0.001) / SECPERHOUR;

    WeekSecond weekSecond;
    weekSecond.week =  bd->BDSWeek;
    weekSecond.sow = bd->Toc;
    WeekSecond2CommonTime(weekSecond, bd->ctToc);
    bd->ctToe = bd->ctToc;
    bd->ctToe.setTimeSystem(TimeSystem::BDT);

  /*  cout << "C" <<bd->prn << " ";
    cout << "omega: " << bd->omega << endl;
    cout << "ecc: " << bd->ecc << " ";
    cout << "M0: " << bd->M0 << endl;
    cout << "BDSWeek: " << bd->BDSWeek << "BDToc: " << bd->Toc << "BDToe" << bd->Toe << endl;*/

    bd = originalPtr;
    return 2;

}
int decode_bdsephem(unsigned char* buff, GPSEPHREC* bds)
{
    unsigned char *p=buff+OEM7HeaderLength;
    int prn;

    GPSEPHREC* bd;

    prn       =U4(p);   p+=4;
    if(prn<1 || prn>MAXBDSPRN)   return 2;

    bd = bd+prn-1;
    bd->System = GNSSSys::BDS;
    bd->PRN = prn;
    bd->TOC.Week =bd->TOE.Week = U4(p)+1356; p+=4;
    bd->SVAccuracy    = R8(p); p+=8;
    bd->SVHealth      = U4(p); p+=4;
    bd->TGD1          = R8(p); p+=8;
    bd->TGD2          = R8(p); p+=12;
    bd->TOC.SecOfWeek = U4(p); p+=4;
    bd->ClkBias       = R8(p); p+=8;
    bd->ClkDrift      = R8(p); p+=8;
    bd->ClkDriftRate  = R8(p); p+=12;
    bd->TOE.SecOfWeek = U4(p); p+=4;
    bd->SqrtA         = R8(p); p+=8;
    bd->e             = R8(p); p+=8;
    bd->omega         = R8(p); p+=8;
    bd->DetlaN        = R8(p); p+=8;
    bd->M0            = R8(p); p+=8;
    bd->OMEGA         = R8(p); p+=8;
    bd->OMEGADot      = R8(p); p+=8;
    bd->i0            = R8(p); p+=8;
    bd->iDot          = R8(p); p+=8;
    bd->Cuc           = R8(p); p+=8;
    bd->Cus           = R8(p); p+=8;
    bd->Crc           = R8(p); p+=8;
    bd->Crs           = R8(p); p+=8;
    bd->Cic           = R8(p); p+=8;
    bd->Cis           = R8(p); p+=8;
    bd->IODE = bd->IODC = (int)(bd->TOE.SecOfWeek + 0.001) / SECPERHOUR;

    cout << bd->PRN << " " << "bd sqrt_A: " << bd->SqrtA << " " << "bd omega: " << bd->omega << endl;


    return 2;
}

int decode_ionutc(unsigned char* buff, IONOPARA* para)
{
    unsigned char *p=buff+OEM7HeaderLength;
    para->alpha[0] = R8(p); p+=8;
    para->alpha[1] = R8(p); p+=8;
    para->alpha[2] = R8(p); p+=8;
    para->alpha[3] = R8(p); p+=8;
    para->beta[0]  = R8(p); p+=8;
    para->beta[1]  = R8(p); p+=8;
    para->beta[2]  = R8(p); p+=8;
    para->beta[3]  = R8(p); p+=8;
    para->IsValid = true;

    return 3;
}

int decode_psrpos(unsigned char* buff, double pos[])
{
    unsigned char *p=buff+OEM7HeaderLength+8;
    double x[3];
    x[0] = R8(p)*Rad;  p+=8;
    x[1] = R8(p)*Rad;  p+=8;
    x[2] = R8(p);  p+=8;
    x[2] += R4(p);

    BLHToXYZ(x,pos,R_WGS84,F_WGS84);
    return 3;
}

int exsign(unsigned int v, int bits)
{
    return (int)(v&(1<<(bits-1))?v|(~0u<<bits):v);
}

unsigned int getbitu(const unsigned char *buff, int pos, int len)
{
    unsigned int bits=0;
    int i;
    for (i=pos;i<pos+len;i++) bits=(bits<<1)+((buff[i/8]>>(7-i%8))&1u);
    return bits;
}
int getbits(const unsigned char *buff, int pos, int len)
{
    unsigned int bits=getbitu(buff,pos,len);
    if (len<=0||32<=len||!(bits&(1u<<(len-1)))) return (int)bits;
    return (int)(bits|(~0u<<len)); /* extend sign */
}

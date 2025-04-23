
#include <iostream>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <thread>
#include <mutex>

#include "GnssStruct.h"
#include "RinexNavStore.hpp"
#include "SPPUCCodePhase.h"
#include "RinexObsReader.h"
#include "DecodeConst.h"
#include "Decode.h"
#include "Const.h"
#include "sockets.h"
#include "GnssFunc.h"

#include <bitset>

using namespace std;



int main()
{
    //基准站
    char netIP[] = "8.148.22.229";//IP地址，是字符数组的形式
    unsigned short netPort = 4002;//NovAtel默认端口

    //流动站
    char netIP2[] = "8.148.22.229";
    unsigned short netPort2 = 7002;

    SOCKET  socketGNSS;//定义套接字，基准站
    SOCKET  socketGNSS2;//流动站

    bool connected(false);//是否连接上了，基准站
    bool connected2(false);//流动站

    //基准站
    WSADATA wsaData;
    SOCKADDR_IN addrSrv;

    //流动站
    WSADATA wsaData2;
    SOCKADDR_IN addrSrv2;

    if(!WSAStartup(MAKEWORD(1, 1), &wsaData))//Windows特有
    {
        if( (socketGNSS = socket(AF_INET, SOCK_STREAM ,0)) != INVALID_SOCKET )
        {
            addrSrv.sin_addr.S_un.S_addr = inet_addr(netIP);
            addrSrv.sin_family = AF_INET;
            addrSrv.sin_port = htons(netPort);
            connect(socketGNSS, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));
            connected = true;//准备就绪
        }
    }

    if(!WSAStartup(MAKEWORD(1, 1), &wsaData2))//Windows特有
    {
        if( (socketGNSS2 = socket(AF_INET, SOCK_STREAM ,0)) != INVALID_SOCKET )
        {
            addrSrv2.sin_addr.S_un.S_addr = inet_addr(netIP2);
            addrSrv2.sin_family = AF_INET;
            addrSrv2.sin_port = htons(netPort2);
            connect(socketGNSS2, (SOCKADDR*)&addrSrv2, sizeof(SOCKADDR));
            connected2 = true;//准备就绪
        }
    }

    // 如果没有连接成功，退出
    if(!connected)
    {
        cerr << "The ip %s was not opened\n" << netIP << endl;
        closesocket(socketGNSS);
        WSACleanup();
        return 0;
    }

    if (!connected2)
    {
        cerr << "The ip %s was not opened\n" << netIP2 << endl;
        closesocket(socketGNSS2);
        WSACleanup();
        return 0;
    }



    std::string solFile =  "oem7.rtk.bds.out";//这个文件里面只有gps的单点定位和rtk浮点解
    std::fstream solStream(solFile, ios::out);
    if (!solStream) {
        cerr << "solution file open error!" << strerror(errno) << endl;
        exit(-1);
    }

    ObsData obsData;//单历元观测数据容器,基准站
    ObsData obsData2;//流动站



    IONOPARA    IonPara;//电离层参数

    RinexNavStore navStore;

    WeekSecond weekSecond;//基准站
    WeekSecond weekSecond2;//流动站

    std::map<string, std::pair<string, string>> ifCodeTypes;//已经加入北斗系统
    ifCodeTypes["G"].first = "C1";
    ifCodeTypes["G"].second = "C2";
    ifCodeTypes["C"].first = "C2";
    ifCodeTypes["C"].second = "C7";

    std::map<string, std::pair<string, string>> dualCodeTypes;//双频观测数据，已经加入北斗系统
    dualCodeTypes["G"].first = "C1";
    dualCodeTypes["G"].second = "C2";
    dualCodeTypes["C"].first = "C2";
    dualCodeTypes["C"].second = "C7";

    //流动站
    SPPUCCodePhase sppUCCodePhaseRover;
    sppUCCodePhaseRover.setRinexNavStore(&navStore);
    sppUCCodePhaseRover.setDualCodeTypes(dualCodeTypes);

    //基准站
    SPPUCCodePhase sppUCCodePhaseBase;
    sppUCCodePhaseBase.setRinexNavStore(&navStore);//同流动站
    sppUCCodePhaseBase.setStationAsBase();//isRover = false;
    sppUCCodePhaseBase.setDualCodeTypes(dualCodeTypes);//同流动站

    //>>> classes for rtk;
    SolverLSQ solverRTK;

    //基准站
    int lenBuffRecv, lenBuffExt;
    unsigned char buffRecv[MAXRAWLEN];//原始数据接收缓冲区
    std::vector<unsigned char> buffExt;//扩展缓冲区，处理不完整数据包

    //流动站
    int lenBuffRecv2, lenBuffExt2;
    unsigned char buffRecv2[MAXRAWLEN];//原始数据接收缓冲区
    std::vector<unsigned char> buffExt2;//扩展缓冲区，处理不完整数据包

    char str[50];

    lenBuffExt = 0;
    lenBuffRecv = 0;

    lenBuffRecv2 = 0;
    lenBuffExt2 = 0;

    while (true){//todo:开始循环读取数据
        //-------------------------------------------
        // step 1: 从网络流中循环读取数据
        //-------------------------------------------
        // 服务器每1s播发一次观测值和星历数据，所以用sleep函数，以防止频繁读取
        Sleep(980);
        // 从本地缓存中读取最新的数据，这里只是获取长度
        lenBuffRecv = recv(socketGNSS, (char*)buffRecv, MAXRAWLEN, 0);

        lenBuffRecv2 = recv(socketGNSS2, (char*)buffRecv2, MAXRAWLEN, 0);

        // 如果小于等于0，说明没有数据都进来，继续读
        if (lenBuffRecv <= 0 || lenBuffRecv2 <= 0)
            continue;

        // 如果都进来了，就把新都进来的buffRecv加入到原来已经处理的buffExt后面
        // buffExt是上一次都进来处理完剩下的不完整的数据包
        buffExt.insert(buffExt.end(), buffRecv, buffRecv + lenBuffRecv);
        lenBuffExt = buffExt.size();

        buffExt2.insert(buffExt2.end(), buffRecv2, buffRecv2 + lenBuffRecv2);
        lenBuffExt2 = buffExt2.size();

        //-------------------------------------------
        // step 2: 根据二进制数包格式，循环解码RangeData，GPSEph，BDSEph等数据
        //-------------------------------------------

        // 解码 oem7的数据包（数据包有可能由观测值，GPS星历，BDS星历等多个数据包构成，所以需要循环解码）
        int headerDataLen;
        double tow, pos[3];//周内秒和位置
        int messageType, week, messageID;
        unsigned char tempData[MAXRAWLEN];

        int headerDataLen2;
        double pos2[3];
        int messageType2, messageID2;
        unsigned char tempData2[MAXRAWLEN];

        // 多个数据包的循环读取；
        int index (0);
        bool foundObs(false);

        int index2 = 0;
        bool foundObs2(false);
        int foundBDEph = 0;
        int foundGPSEph = 0;

        while (true){//todo:开始循环解码基准站
            // 三个同步字的位置为[i,i+1,i+2], 数组最大维为（lenBuffExt-1）
            // buffExt中剩余的字节数少于3个，退出数据包匹配
            if ((index + 2) > (lenBuffExt - 1))
                break;

            // 先找第一个数据包的头；匹配不成功，继续向下移动; 数据同步字3个字节长；
            bool isSynced(false);
            while ((index + 2) <= (lenBuffExt - 1))
            {
                if (buffExt[index] == OEM7SyncByte1 && buffExt[index + 1] == OEM7SyncByte2 && buffExt[index + 2] == OEM7SyncByte3)
                {
                    isSynced = true;
                    break;
                }
                index++;
            }

            // 如果在所有的buffExt中[i,lenBuffExt-1]中没有找到同步字，退出查找
            if (!isSynced)
                break;

            // 如果同步成功，需要检查剩余的空间是否能够存储下头文件的长度
            // OEM7的Header的长度为28： 包括3个同步字+25个字的信息数据
            //这里其实就是判断读进来的二进制数据是不是一个完整的数据包，也就是进来的数据有没有完整的包括头文件
            if ((index + OEM7HeaderLength - 1) > (lenBuffExt - 1))
                break;

            // 存储头文件数据到tempData
            for (int j = 0; j < OEM7HeaderLength; j++)
                tempData[j] = buffExt[index + j];

            // 读取头文件的第8，9位，取其数值，其值为Data的长度(应该指的是一个数据包的长度）；
            headerDataLen = U2(tempData + 8) + OEM7HeaderLength;

            // 数据包长度检验，看能不能完整的存储一个数据包
            if ((index + headerDataLen + OEM7CRCLength - 1) > (lenBuffExt - 1))
                break;

            for (int j = OEM7HeaderLength; j < headerDataLen + OEM7CRCLength; j++)
                tempData[j] = buffExt[index + j];

            // crc32,奇偶检验，如果不成功，则说明数据缺失，跳过这个数据包；
            if (crc32(tempData, headerDataLen) != U4(tempData + headerDataLen))
            {
                index += headerDataLen + OEM7CRCLength;
                continue;
            }

            //----------------------------------------
            // Message type: 8 bits,
            // bit 0-4: source;
            // bit 5-6:
            //	00, binary,
            //	01, ascii,
            //  10: Abbreviated ASCII, NMEA
            //  11: reserved
            //----------------------------------------
            messageType = (U1(tempData + 6) >> 5) & 0x3;
            if (messageType != 0)
                continue;

            weekSecond.week = U2(tempData + 14);
            weekSecond.sow = U4(tempData + 16) * 0.001;
            CommonTime epoch;
            WeekSecond2CommonTime(weekSecond, epoch);
          //  cout << "Epoch: " << epoch << endl;//这里转换没有问题

            // messageID
            messageID = U2(tempData + 4);

            cout << "messageID:" << messageID << endl;
            switch (messageID) {
                case ID_RANGE:

                    // clear obsData;初始化数据结构
                    obsData = {};

                    obsData.antennaPosition << pos[0], pos[1], pos[2]; //todo:这个地方pos的值是在ID_BESTPOS里面求出来的

                    obsData.epoch = epoch;
                    foundObs = decode_rangeb_oem7(tempData, &obsData);
                    break;
                case ID_BDSEPHEM:
                    foundBDEph = decode_BDSEph(tempData, navStore.bdsNav);
                    break;
                case ID_GPSEPHEM:
                    foundGPSEph = decode_GPSEph(tempData, navStore.gpsNav);
                    break;
                case ID_IONUTC:
                    decode_ionutc(tempData, &IonPara);
                    break;
                case ID_BESTPOS:
                    decode_psrpos(tempData, pos);
                    break;
                default:
                    break;
            }
            index += headerDataLen + 4;
        }

        // 删除已经跳过的数据:
        //		情况1：crc32检验失效，丢掉的数据；
        //		情况2：crc32检验成功，读取的数据
        buffExt.erase(buffExt.begin(), buffExt.begin() + index);

        int foundBD2 = 0, foundGPS2 = 0;
        while (true){//todo:开始循环解码流动站
            // 三个同步字的位置为[i,i+1,i+2], 数组最大维为（lenBuffExt-1）
            // buffExt中剩余的字节数少于3个，退出数据包匹配
            if ((index2 + 2) > (lenBuffExt2 - 1))
                break;

            // 先找第一个数据包的头；匹配不成功，继续向下移动; 数据同步字3个字节长；
            bool isSynced(false);
            while ((index2 + 2) <= (lenBuffExt2 - 1))
            {
                if (buffExt2[index2] == OEM7SyncByte1 && buffExt2[index2 + 1] == OEM7SyncByte2 && buffExt2[index2 + 2] == OEM7SyncByte3)
                {
                    isSynced = true;
                    break;
                }
                index2++;
            }

            // 如果在所有的buffExt中[i,lenBuffExt-1]中没有找到同步字，退出查找
            if (!isSynced)
                break;

            // 如果同步成功，需要检查剩余的空间是否能够存储下头文件的长度
            // OEM7的Header的长度为28： 包括3个同步字+25个字的信息数据
            //这里其实就是判断读进来的二进制数据是不是一个完整的数据包，也就是进来的数据有没有完整的包括头文件
            if ((index2 + OEM7HeaderLength - 1) > (lenBuffExt2 - 1))
                break;

            // 存储头文件数据到tempData
            for (int j = 0; j < OEM7HeaderLength; j++)
                tempData2[j] = buffExt2[index2 + j];

            // 读取头文件的第8，9位，取其数值，其值为Data的长度(应该指的是一个数据包的长度）；
            headerDataLen2 = U2(tempData2 + 8) + OEM7HeaderLength;

            // 数据包长度检验，看能不能完整的存储一个数据包
            if ((index2 + headerDataLen2 + OEM7CRCLength - 1) > (lenBuffExt2 - 1))
                break;

            for (int j = OEM7HeaderLength; j < headerDataLen2 + OEM7CRCLength; j++)
                tempData2[j] = buffExt2[index2 + j];

            // crc32,奇偶检验，如果不成功，则说明数据缺失，跳过这个数据包；
            if (crc32(tempData2, headerDataLen2) != U4(tempData2 + headerDataLen2))
            {
                index2 += headerDataLen2 + OEM7CRCLength;
                continue;
            }

            //----------------------------------------
            // Message type: 8 bits,
            // bit 0-4: source;
            // bit 5-6:
            //	00, binary,
            //	01, ascii,
            //  10: Abbreviated ASCII, NMEA
            //  11: reserved
            //----------------------------------------
            messageType2 = (U1(tempData2 + 6) >> 5) & 0x3;
            if (messageType2 != 0)
                continue;

            weekSecond2.week = U2(tempData2 + 14);
            weekSecond2.sow = U4(tempData2 + 16) * 0.001;
            CommonTime epoch;
            WeekSecond2CommonTime(weekSecond2, epoch);
            //  cout << "Epoch: " << epoch << endl;//这里转换没有问题

            // messageID
            messageID2 = U2(tempData2 + 4);

            cout << "messageID:" << messageID2 << endl;
            switch (messageID2) {
                case ID_RANGE:

                    // clear obsData;初始化数据结构
                    obsData2 = {};

                    obsData2.antennaPosition << pos2[0], pos2[1], pos2[2]; //todo:这个地方pos的值是在ID_BESTPOS里面求出来的

                    obsData2.epoch = epoch;
                    foundObs2 = decode_rangeb_oem7(tempData2, &obsData2);
                    break;
                case ID_BDSEPHEM:
                    foundBD2 = decode_BDSEph(tempData2, navStore.bdsNav);
                    break;
                case ID_GPSEPHEM:
                   foundGPS2 = decode_GPSEph(tempData2, navStore.gpsNav);
                    break;
                case ID_IONUTC:
                    decode_ionutc(tempData2, &IonPara);
                    break;
                case ID_BESTPOS:
                    decode_psrpos(tempData2, pos2);
                    break;
                default:
                    break;
            }
            index2 += headerDataLen2 + 4;
        }

        // 删除已经跳过的数据:
        //		情况1：crc32检验失效，丢掉的数据；
        //		情况2：crc32检验成功，读取的数据
        buffExt2.erase(buffExt2.begin(), buffExt2.begin() + index2);

        //-------------------------------------------
        // step 3: 如果上一次网络流数据中包含了观测值数据，就进行单点定位、定速等工作
        //-------------------------------------------

        if (foundBD2 == 2 && obsData2.SatNum > 5 && obsData.SatNum > 5)//todo:还要判断是否有卫星星历数据
        {
            cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
           // cout << "EPH:"  << foundBD2 << endl;
            cout << ">> week:" << weekSecond2.week
                 << "secOfWeek:" << weekSecond2.sow
                 << "numSats:" << obsData2.SatNum << endl;


          /*  int satNum = obsData2.SatNum;
            SatIDSet satRejectedSet;//这里存储的是不要卫星号
            for (auto stv:obsData2.satTypeValueData){
                SatID sat(stv.first);
                int id = sat.id;
                if (navStore.gpsNav[id].prn == 0){
                    satNum--;
                    satRejectedSet.insert(sat);
                }//todo:这里是按照prn号索引的，我要注意那个prn要不要减1，等下输出prn试试，如果要减1，那么要注意findGPS和findBDS函数哪里要改一下
            }


            //todo: 这里需要把没有对应卫星星历的卫星删除，那么判断条件也要改成是否有观测值以及卫星数量
            // remove bad sat;
            for (auto sat: satRejectedSet) {
                obsData.satTypeValueData.erase(sat);
            }*/



           /* for (int i = 0; i < obsData2.SatNum; i++){
                cout << obsData2 << endl;
            }*/

            // print obs for each sat
           /* for (int i = 0; i < obsData.SatNum; i++)
            {
                cout << obsData2;
            }*/

            cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
            CommonTime epoch = obsData2.epoch;
            try
            {
                //todo:这里有问题,这里都没有进去函数
                sppUCCodePhaseRover.solve(obsData2);//之前有了广播星历的数据了,这个函数没有返回值，但是给xyz赋值
            }catch (SVNumException &e)
            {
                cout << "rover less 4";
                continue;
            }
            EquSys equSysRover = sppUCCodePhaseRover.getEquSys();//获取观测方程
            SatID datumSat = sppUCCodePhaseRover.getDatumSat();//SatID需要实现bd2和bd3的区分，这里是找到仰角最高的卫星
            Vector3d xyzRover = sppUCCodePhaseRover.getXYZ();//这个函数就是用来获取solve()函数得到的xyz值，是单点定位的坐标值，精度不够高。*/

            try
            {
                sppUCCodePhaseBase.solve(obsData);
            }catch (SVNumException &e) {continue;}

            EquSys equSysBase = sppUCCodePhaseBase.getEquSys();

            // compute between-station single-difference equation
            EquSys equSysSD;
            differenceStation(equSysRover, equSysBase, equSysSD);//站间单差

            // compute between-sat single-difference equation
            EquSys equSysDD;
            differenceSat(datumSat, equSysSD, equSysDD);

            // solve solution
            solverRTK.solve(equSysDD);
            Vector3d dxyzRTK = solverRTK.getdxyz();

            Vector3d xyzRTKFloat;
            xyzRTKFloat = xyzRover + dxyzRTK;

            if (SYS == "C")printSolution(solStream, epoch, xyzRover, xyzRTKFloat);

        }

    }

    closesocket(socketGNSS);
    closesocket(socketGNSS2);
    WSACleanup();
    return 0;

}
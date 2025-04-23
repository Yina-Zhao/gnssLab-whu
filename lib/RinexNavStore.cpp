///////////////////////////////////////////////////////////////////////////////
//
// Read and store RINEX formated navigation message (Rinex3Nav) data, 
// following the RINEX 3.02 spec. Support for GNSS GPS, GAL, GLO, BDS, QZS.
//
///////////////////////////////////////////////////////////////////////////////

#include "RinexNavStore.hpp"
#include "StringUtils.h"

using namespace std;
#define debug 0

const string RinexNavStore::stringVersion = "RINEX VERSION / TYPE";
const string RinexNavStore::stringRunBy = "PGM / RUN BY / DATE";
const string RinexNavStore::stringComment = "COMMENT";
const string RinexNavStore::stringIonoCorr = "IONOSPHERIC CORR";
const string RinexNavStore::stringTimeSysCorr = "TIME SYSTEM CORR";
const string RinexNavStore::stringLeapSeconds = "LEAP SECONDS";
//R2.10GLO
const string RinexNavStore::stringCorrSysTime = "CORR TO SYSTEM TIME";
//R2.11GPS
const string RinexNavStore::stringDeltaUTC = "DELTA-UTC: A0,A1,T,W";
//R2.11GEO
const string RinexNavStore::stringDUTC = "D-UTC A0,A1,T,W,S,U";
//R2.11
const string RinexNavStore::stringIonAlpha = "ION ALPHA";
//R2.11
const string RinexNavStore::stringIonBeta = "ION BETA";
const string RinexNavStore::stringEoH = "END OF HEADER";


void RinexNavStore::loadGPSEph(NavEphGPS &gpsEph, string &line, fstream &navFileStream) {

    SatID sat(line.substr(0,3));

    ///add each sat into the satTable
    vector<SatID>::iterator result = find(satTable.begin(), satTable.end(), sat);
    if (result == satTable.end()) {
        satTable.push_back(sat);
    }

    int yr = safeStoi(line.substr(4, 4));
    int mo = safeStoi(line.substr(9, 2));
    int day = safeStoi(line.substr(12, 2));
    int hr = safeStoi(line.substr(15, 2));
    int min = safeStoi(line.substr(18, 2));
    double sec = safeStod(line.substr(21, 2));

    /// Fix RINEX epochs of the form 'yy mm dd hr 59 60.0'
    short ds = 0;
    if (sec >= 60.) {
        ds = sec;
        sec = 0;
    }
    CivilTime cvt(yr, mo, day, hr, min, sec);
    gpsEph.CivilToc = cvt;
    gpsEph.ctToe = CivilTime2CommonTime(cvt);;

    if (ds != 0) gpsEph.ctToe += ds;
    gpsEph.ctToe.setTimeSystem(TimeSystem::GPS);

    GPSWeekSecond gws;
    CommonTime2WeekSecond(gpsEph.ctToe, gws);     // sow is system-independent

    gpsEph.Toc = gws.sow;
    gpsEph.af0 = safeStod(line.substr(23, 19));
    gpsEph.af1 = safeStod(line.substr(42, 19));
    gpsEph.af2 = safeStod(line.substr(61, 19));

    ///orbit-1
    int n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    gpsEph.IODE = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.Crs = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.Delta_n = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.M0 = safeStod(line.substr(n, 19));
    ///orbit-2
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    gpsEph.Cuc = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.ecc = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.Cus = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.sqrt_A = safeStod(line.substr(n, 19));
    ///orbit-3
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    gpsEph.Toe = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.Cic = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.OMEGA_0 = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.Cis = safeStod(line.substr(n, 19));
    ///orbit-4
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    gpsEph.i0 = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.Crc = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.omega = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.OMEGA_DOT = safeStod(line.substr(n, 19));
    ///orbit-5
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    gpsEph.IDOT = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.L2Codes = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.GPSWeek = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.L2Pflag = safeStod(line.substr(n, 19));
    ///orbit-6
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    gpsEph.URA = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.SV_health = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.TGD = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.IODC = safeStod(line.substr(n, 19));
    ///orbit-7
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    gpsEph.HOWtime = safeStod(line.substr(n, 19));
    n += 19;
    gpsEph.fitInterval = safeStod(line.substr(n, 19));
    n += 19;

    /// some process
    /// Some RINEX files have HOW < 0.
    while (gpsEph.HOWtime < 0) {
        gpsEph.HOWtime += (long) FULLWEEK;
        gpsEph.GPSWeek--;
    }

    /// In RINEX *files*, weeknum is the week of TOE.
    /// Internally (Rx3NavData), weeknum is week of HOW
    if (gpsEph.HOWtime - gpsEph.Toe > HALFWEEK)
        gpsEph.GPSWeek--;
    else if (gpsEph.HOWtime - gpsEph.Toe < -HALFWEEK)
        gpsEph.GPSWeek++;

    /// Get week for clock, to build Toc
    long adjHOWtime = gpsEph.HOWtime;
    short adjWeeknum = gpsEph.GPSWeek;
    long lToc = (long) gpsEph.Toc;
    if ((gpsEph.HOWtime % SEC_PER_DAY) == 0 &&
        ((lToc) % SEC_PER_DAY) == 0 &&
        gpsEph.HOWtime == lToc) {
        adjHOWtime = gpsEph.HOWtime - 30;
        if (adjHOWtime < 0) {
            adjHOWtime += FULLWEEK;
            adjWeeknum--;
        }
    }

    double dt = gpsEph.Toc - adjHOWtime;
    int week = gpsEph.GPSWeek;
    if (dt < -HALFWEEK) week++; else if (dt > HALFWEEK) week--;
    GPSWeekSecond gws2 = GPSWeekSecond(week, gpsEph.Toc, TimeSystem::GPS);
    WeekSecond2CommonTime(gws2, gpsEph.ctToc);

    gpsEph.ctToc.setTimeSystem(TimeSystem::GPS);

    gpsEphData[sat][gpsEph.ctToe] = gpsEph;
}

void RinexNavStore::loadBDSEph(NavEphBDS& bdsEph, string& line, fstream &navFileStream)
{
    SatID sat(line.substr(0,3));//读取prn号

    ///add each sat into the satTable，同时也避免了重复添加
    vector<SatID>::iterator result = find(satTable.begin(), satTable.end(), sat);
    if (result == satTable.end()) {
        satTable.push_back(sat);
    }

    int yr = safeStoi(line.substr(4, 4));
    int mo = safeStoi(line.substr(9, 2));
    int day = safeStoi(line.substr(12, 2));
    int hr = safeStoi(line.substr(15, 2));
    int min = safeStoi(line.substr(18, 2));
    double sec = safeStod(line.substr(21, 2));

    /// Fix RINEX epochs of the form 'yy mm dd hr 59 60.0'
    short ds = 0;
    if (sec >= 60.) {
        ds = sec;
        sec = 0;
    }

    CivilTime cvt(yr, mo, day, hr, min, sec);
    bdsEph.CivilToc = cvt;
    bdsEph.ctToe = CivilTime2CommonTime(cvt);;

    if (ds != 0) bdsEph.ctToe += ds;
    bdsEph.ctToe.setTimeSystem(TimeSystem::BDT);

    BDSWeekSecond bws;
    CommonTime2WeekSecond(bdsEph.ctToe, bws);     // sow is system-independent

    bdsEph.Toc = bws.sow;
    bdsEph.af0 = safeStod(line.substr(23, 19));
    bdsEph.af1 = safeStod(line.substr(42, 19));
    bdsEph.af2 = safeStod(line.substr(61, 19));

    ///orbit-1
    int n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    bdsEph.AODE = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.Crs = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.Delta_n = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.M0 = safeStod(line.substr(n, 19));
    ///orbit-2
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    bdsEph.Cuc = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.ecc = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.Cus = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.sqrt_A = safeStod(line.substr(n, 19));
    ///orbit-3
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    bdsEph.Toe = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.Cic = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.OMEGA_0 = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.Cis = safeStod(line.substr(n, 19));
    ///orbit-4
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    bdsEph.i0 = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.Crc = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.omega = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.OMEGA_DOT = safeStod(line.substr(n, 19));
    ///orbit-5
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    bdsEph.IDOT = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.spare1 = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.BDSWeek = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.spare2 = safeStod(line.substr(n, 19));
    ///orbit-6
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    bdsEph.URA = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.SV_health = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.TGD1 = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.TGD2 = safeStod(line.substr(n, 19));
    ///orbit-7
    n = 4;
    getline(navFileStream, line);
    replace(line.begin(), line.end(), 'D', 'e');
    bdsEph.HOWtime = safeStod(line.substr(n, 19));
    n += 19;
    bdsEph.AODC = safeStod(line.substr(n, 19));
    n += 19;

    /// some process
    /// Some RINEX files have HOW < 0.
    while (bdsEph.HOWtime < 0) {
        bdsEph.HOWtime += (long) FULLWEEK;
        bdsEph.BDSWeek--;
    }

    /// In RINEX *files*, weeknum is the week of TOE.
    /// Internally (Rx3NavData), weeknum is week of HOW
    if (bdsEph.HOWtime - bdsEph.Toe > HALFWEEK)
        bdsEph.BDSWeek--;
    else if (bdsEph.HOWtime - bdsEph.Toe < -HALFWEEK)
        bdsEph.BDSWeek++;

    /// Get week for clock, to build Toc
    long adjHOWtime = bdsEph.HOWtime;
    short adjWeeknum = bdsEph.BDSWeek;
    long lToc = (long) bdsEph.Toc;
    if ((bdsEph.HOWtime % SEC_PER_DAY) == 0 &&
        ((lToc) % SEC_PER_DAY) == 0 &&
        bdsEph.HOWtime == lToc) {
        adjHOWtime = bdsEph.HOWtime - 30;
        if (adjHOWtime < 0) {
            adjHOWtime += FULLWEEK;
            adjWeeknum--;
        }
        }

    double dt = bdsEph.Toc - adjHOWtime;
    int week = bdsEph.BDSWeek;
    if (dt < -HALFWEEK) week++; else if (dt > HALFWEEK) week--;
    BDSWeekSecond bws2 = BDSWeekSecond(week, bdsEph.Toc, TimeSystem::BDT);
    WeekSecond2CommonTime(bws2, bdsEph.ctToc);

    bdsEph.ctToc.setTimeSystem(TimeSystem::BDT);

    bdsEphData[sat][bdsEph.ctToe] = bdsEph;
}


void RinexNavStore::loadFile(string &file) {
    rx3NavFile = file;
    if (rx3NavFile.size() == 0) {
        cout << "the nav file path is empty!" << endl;
        exit(-1);
    }

    /*if (debug)
        cout << "RinexNavStore: fileName:" << rx3NavFile << endl;*/

    fstream navFileStream(rx3NavFile.c_str(), ios::in);
    if (!navFileStream) {
        cerr << "can't open file:" << rx3NavFile << endl;
        exit(-1);
    }

    int lineNumber(0);

    ///first, we should read nav head
    while (1) {
        string line;
        getline(navFileStream, line);

        /*if (debug)
            cout << "RinexNavStore:" << line << endl;*/

        stripTrailing(line);

        if (line.length() == 0) continue;
        else if (line.length() < 60) {
            cout << line << endl;
            cout << "line.length is fault" << line.length() << endl;
            FFStreamError e("Invalid line length, \n"
                            "may be the file is generated by windows, \n"
                            "please use dos2unix to convert the file!");
            throw (e);
        }

        lineNumber++;

        string thisLabel(line, 60, 20);

        /// following is huge if else else ... endif for each record type
        if (thisLabel == stringVersion) {
            /// "RINEX VERSION / TYPE"
            version = safeStod(line.substr(0, 20));
            fileType = strip(line.substr(20, 20));
            if(version<3.0)
            {
                FileMissingException e("don't support navigation file with version less than 3.0");
                throw(e);
            }
            if (version >= 3) {                        // ver 3
                if (fileType[0] != 'N' && fileType[0] != 'n') {
                    FFStreamError e("File type is not NAVIGATION: " + fileType);
                    throw(e);
                }
                fileSys = strip(line.substr(40, 20));   // not in ver 2
            }
            fileType = "NAVIGATION";
        } else if (thisLabel == stringRunBy) {
            /// "PGM / RUN BY / DATE"
            fileProgram = strip(line.substr(0, 20));
            fileAgency = strip(line.substr(20, 20));
            // R2 may not have 'UTC' at end
            date = strip(line.substr(40, 20));
        } else if (thisLabel == stringComment) {
            /// "COMMENT"
            commentList.push_back(strip(line.substr(0, 60)));
        } else if (thisLabel == stringIonoCorr) {
            /// "IONOSPHERIC CORR"
            string ionoCorrType = strip(line.substr(0, 4));
            vector<double> ionoCorrCoeff;
            for (int i = 0; i < 4; i++) {
                double ionoCorr = safeStod(line.substr(5 + 12 * i, 12));
                ionoCorrCoeff.push_back(ionoCorr);
            }
            ionoCorrData[ionoCorrType].clear();
            ionoCorrData[ionoCorrType] = ionoCorrCoeff;
        } else if (thisLabel == stringTimeSysCorr) {
            /// "TIME SYSTEM CORR"
            string timeSysCorrType = strip(line.substr(0, 4));

            TimeSysCorr timeSysCorrValue;
            timeSysCorrValue.A0 = safeStod(line.substr(5, 17));
            timeSysCorrValue.A1 = safeStod(line.substr(22, 16));
            timeSysCorrValue.refSOW = safeStoi(line.substr(38, 7));
            timeSysCorrValue.refWeek = safeStoi(line.substr(45, 5));
            timeSysCorrValue.geoProvider = string(" ");
            timeSysCorrValue.geoUTCid = 0;

            timeSysCorrData[timeSysCorrType] = timeSysCorrValue;
        } else if (thisLabel == stringLeapSeconds) {
            /// "LEAP SECONDS"
            leapSeconds = safeStoi(line.substr(0, 6));
            leapDelta = safeStoi(line.substr(6, 6));
            leapWeek = safeStoi(line.substr(12, 6));
            leapDay = safeStoi(line.substr(18, 6));
        } else if (thisLabel == stringEoH) {
            /// "END OF HEADER"
            break;
        }
    }

    ///now, start read nav data
    while (navFileStream.peek() != EOF) {
        string line;
        getline(navFileStream, line);

       // if (debug)
         //   cout << "RinexNavStore:" << line << endl;

        replace(line.begin(), line.end(), 'D', 'e');

        if (line[0] == 'G' && SYS == "G") {
            //if (1) cout << "gps" << endl;
          //  if (1) cout << "RinexNavStore:" << line << endl;
            NavEphGPS gpsEph;
            loadGPSEph(gpsEph, line, navFileStream);

            /*if (1)
            {

                cout << gpsEph.af0 << " " << gpsEph.af1 << " " << gpsEph.af2 << endl;
                cout << gpsEph.IODE << " " << gpsEph.Crs << endl;

            }*/
        }
        if (line[0] == 'C' && SYS == "C")
        {

           // if (1) cout << "bds" << endl;
           // if (1) cout << "RinexNavStore:" << line << endl;
            NavEphBDS bdsEph;
            loadBDSEph(bdsEph, line, navFileStream);
            /*if (1)
            {

                cout << bdsEph.IDOT << " " << bdsEph.spare1 << " " << bdsEph.BDSWeek << " " << bdsEph.spare2 << endl;
                cout << bdsEph.URA << " " << bdsEph.SV_health<< " " << bdsEph.TGD1 << " " << bdsEph.TGD2 << endl;

            }*/
           // if (1) cout << "cToe" << bdsEph.ctToe << endl;

        }
    }
}

Xvt RinexNavStore::getXvt(const SatID &sat, const CommonTime &epoch) {
    Xvt xvt;
    CommonTime realEpoch;
    TimeSystem ts;
    /*if (debug)
        cout << sat << endl;*/
    if (sat.system == "G" && SYS == "G") {
        ts = TimeSystem::GPS;
        realEpoch = convertTimeSystem(epoch, ts);
       // if (1) cout << "realEpoch: " << realEpoch << endl;

        /*if (debug)
            cout << CommonTime2CivilTime(epoch) << endl;*/

        //todo:这里就涉及到星历的获取方式了，改这里！！！！
        NavEphGPS gpsEph = findGPSEph(sat, realEpoch);
        cout << "sat: " << sat << " prn: " << gpsEph.prn << endl;
       // cout << "Toc: " << gpsEph.Toc << endl;

       /* if (1)
        {
            cout << gpsEph.Toc <<" " << gpsEph.af0 << " " << gpsEph.af1 << endl;
            cout << gpsEph.IODE << " " << gpsEph.Crs << endl;
        }*/
        //todo:这里打印有个问题就是钟差钟漂等时间信息没有读进来（刚才看了一下解码信息，是我现在的解码函数没有和原先的名称对应起来）
        //todo: 改了一下名称，现在还剩下Toc数据有问题，在原先代码的定义中，Toc输出的是年月日时分秒，而在我的解码函数中，toc是周内秒的定义
        //todo:所以这里需要注意在后续的计算中要把toc当成周内秒
       /* if (1) {
            cout << "RinexNavStore::GPS eph:" << endl;
            gpsEph.printData();
        }*/

        xvt = gpsEph.svXvt(realEpoch);

       // if (1) cout << "gpsxvt" << endl;

       /* if (debug) {
            cout << "RinexNavStore::xvt:" << endl;
            cout << xvt << endl;
        }*/
    }else if (sat.system == "C" && SYS == "C") {



        ts = TimeSystem::BDT;
        realEpoch = convertTimeSystem(epoch, ts);
       // if (1) cout << "realEpoch: " << realEpoch << endl;//todo:有输出，但是时间系统显示还是GPS,但是确实是北斗时间系统，因为和GPS的输出相差了14秒



        NavEphBDS bdsEph = findBDSEph(sat, realEpoch);//todo:初步判断这里没错


      /* if (1)
        {
            cout << bdsEph.Toc <<" " << bdsEph.af0 << " " << bdsEph.af1 << endl;
            cout << bdsEph.AODE << " " << bdsEph.Crs << endl;
        }*/


        xvt = bdsEph.svXvt(realEpoch);//todo:问题在这里！！！！问题在这里 ！！！！！初步判断为传参不成功
        //if (1) cout << "bdsxvt" << endl;

    }
    else {
        InvalidRequest e("RinexNavStore: don't support the input satellite system!");
        throw (e);
    }

    return xvt;
}

NavEphGPS RinexNavStore::findGPSEph(const SatID &sat, const CommonTime &epoch) {
  //  GPSWeekSecond targetWS;
    //cout << "epoch: " << epoch << endl;//todo:这里参数传入是正常的
  //  CommonTime2WeekSecond(epoch, targetWS);
   // cout << "targetWS: " << targetWS.week << " " << targetWS.sow <<  endl;//todo:这里输出不了，说明转换有问题
    NavEphGPS GPSEph;
    GPSEph = gpsNav[sat.id - 1];

    // todo:
    // 这里应该改进，寻找最接近的历元的卫星星历
   /* for (auto it: gpsEphData[sat]) {
        GPSWeekSecond ws;
        CommonTime2WeekSecond(it.first, ws);
        double diff = ws.sow - targetWS.sow;
        if (diff >= -7201.0 && diff <= 7201.0) {
            gpsEph = it.second;
            break;
        }
    }*/

    return GPSEph;
}

NavEphBDS RinexNavStore::findBDSEph(const SatID& sat, const CommonTime& epoch)
{
  /*  BDSWeekSecond targetWS;
    CommonTime2WeekSecond(epoch, targetWS);*/
    NavEphBDS bdsEph;
    bdsEph = bdsNav[sat.id - 1];
    // 3.2 北斗GEO卫星星历有效期较长（通常4小时），其他卫星2小时
   /* double maxValidDiff = (sat.id <= 5) ? 14400.0 : 7200.0;  // GEO(PRN1-5)有效期4小时，其他2小时
    double minDiff = std::numeric_limits<double>::max();

    for (auto it: bdsEphData[sat]) {
        BDSWeekSecond ws;
        CommonTime2WeekSecond(it.first, ws);
        double diff = std::abs(ws.sow - targetWS.sow);
        if (diff <= maxValidDiff && diff < minDiff) {
            minDiff = diff;
            bdsEph = it.second;
        }
    }*/
    return bdsEph;

}







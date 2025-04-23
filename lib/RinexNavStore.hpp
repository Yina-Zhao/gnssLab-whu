///////////////////////////////////////////////////////////
//
// Function
//
// Read and store RINEX formated navigation message (Rinex3Nav) data, following
// the RINEX 3.02 spec. Support for GNSS GPS, GAL, GLO, BDS, QZS (GEO TBD).
//
// copyright
// 
// shoujian zhang
//
///////////////////////////////////////////////////////////

#ifndef RinexNavStore_HPP
#define RinexNavStore_HPP

#include <iostream>
#include <string>
#include <list>
#include <map>
#include <algorithm>
#include <fstream>

#include "NavEphGPS.hpp"
#include "NavEphBDS.h"
#include "GnssStruct.h"


using namespace std;

class RinexNavStore {
public:

    RinexNavStore() {};

    void loadGPSEph(NavEphGPS &gpsEph, string &line, fstream &navFile);
    void loadBDSEph(NavEphBDS &bdsEph, string &line, fstream &navFile);
    void loadFile(string &file);
    Xvt getXvt(const SatID &sat, const CommonTime &epoch);
    NavEphGPS findGPSEph(const SatID &sat, const CommonTime &epoch);
    NavEphBDS findBDSEph(const SatID &sat, const CommonTime &epoch);

    NavEphGPS gpsNav[MAXGPSPRN];
    NavEphBDS bdsNav[MAXBDSPRN];//广播星历，以prn为索引//todo:我先暂时只定义一个结构来存储


    /// destructor
    virtual ~RinexNavStore() {};

    struct TimeSysCorr {
        double A0;
        double A1;
        int refSOW;
        int refWeek;
        string geoProvider;
        int geoUTCid;
    };

    typedef map<string, vector<double>> ionoCorrMap;
    typedef map<string, TimeSysCorr> timeSysCorrMap;

    string rx3NavFile;
    double version;                ///< RINEX Version
    std::string fileType;          ///< File type "N...."
    std::string fileSys;           ///< File system string
    std::string fileProgram;       ///< Program string
    std::string fileAgency;        ///< Agency string
    std::string date;              ///< Date string; includes "UTC" at the end
    std::vector<std::string> commentList;  ///< Comment list

    ionoCorrMap ionoCorrData;
    timeSysCorrMap timeSysCorrData;

    long leapSeconds;              ///< Leap seconds
    long leapDelta;                ///< Change in Leap seconds at ref time
    long leapWeek;                 ///< Week number of ref time
    long leapDay;                  ///< Day of week of ref time

    static const std::string stringVersion;      /// "RINEX VERSION / TYPE"
    static const std::string stringRunBy;        /// "PGM / RUN BY / DATE"
    static const std::string stringComment;      /// "COMMENT"
    // R3.x
    static const std::string stringIonoCorr;     /// "IONOSPHERIC CORR"
    static const std::string stringTimeSysCorr;  /// "TIME SYSTEM CORR"
    static const std::string stringLeapSeconds;  /// "LEAP SECONDS"
    static const std::string stringDeltaUTC;     /// "DELTA-UTC: A0,A1,T,W" // R2.11 GPS
    static const std::string stringCorrSysTime;  /// "CORR TO SYSTEM TIME"  // R2.10 GLO
    static const std::string stringDUTC;         /// "D-UTC A0,A1,T,W,S,U"  // R2.11 GEO
    static const std::string stringIonAlpha;     /// "ION ALPHA"            // R2.11
    static const std::string stringIonBeta;      /// "ION BETA"             // R2.11
    static const std::string stringEoH;          /// "END OF HEADER"

    ///tables that record the sat of different sat system
    ///in order to get the eph data num easily
    vector<SatID> satTable;

    map<SatID, std::map<CommonTime, NavEphGPS>> gpsEphData;
    map<SatID, std::map<CommonTime, NavEphBDS>> bdsEphData;



};


#endif // RinexNavStore_HPP

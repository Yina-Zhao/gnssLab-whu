//
// Created by shjzh on 2025/3/9.
//

#ifndef GNSSLAB_RINEXOBSREADER_H
#define GNSSLAB_RINEXOBSREADER_H
#include <fstream>
#include "GnssStruct.h"

class RinexObsReader {
public:
    RinexObsReader()
    : pFileStream(NULL), isHeaderRead(false)
    {};

    void setFileStream(std::fstream* pStream)
    {
        pFileStream = pStream;
    };

    void setSelectedTypes(std::map<string, std::set<string>>& systemTypes)
    {
        sysTypes = systemTypes;
    };

    void parseRinexHeader();
    ObsData parseRinexObs();

    ObsData parseRinexObs(CommonTime& syncEpoch)
    {
        // store current stream pos;
        streampos sp( pFileStream->tellg() );
        ObsData obsData;
        while(true){
            if( pFileStream->peek() == EOF ){
                break;
            }
            // read a record from current strm;
            obsData = parseRinexObs();

            // 首先寻找大于等于参考时刻的历元
            // 只要大于等于，就意味着时间是同步的或者是超过了给定参考时刻的
            if(obsData.epoch >=syncEpoch)
            {
                break;
            }
        }
        // 如果流动站的时刻大于给定的参考时刻+容许的误差，则说明流动站观测值超前了，
        // 此时，同步失败，且要把流动站的流重置到文件开头，以实现下一个历元的同步。
        if(obsData.epoch > (syncEpoch + 0.001) )
        {
            pFileStream->seekg( sp );
            SyncException e("Rx3ObsData::can't synchronize the obs!");
            throw(e);
        }

        return obsData;
    };

    CommonTime parseTime(const string &line);
    void chooseObs(ObsData &obsData);

    ~RinexObsReader()
    {};

private:
    std::fstream* pFileStream;
    RinexHeader rinexHeader;
    std::map<string, std::set<string>> sysTypes;
    bool isHeaderRead;
};


#endif //GNSSLAB_RINEXOBSREADER_H

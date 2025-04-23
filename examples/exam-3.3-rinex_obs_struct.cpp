//
// Created by shjzh on 2025/2/17.
//
#include <iostream>
#include <vector>
#include <map>
#include <string>

// 观测值数据结构
struct ObservationValue {
    double value;          // 观测值
    int lli;               // 失锁指示器 (0-7)
    int signalStrength;    // 信号强度 (0-9)

    ObservationValue(double v = 0.0, int l = 0, int s = 0)
            : value(v), lli(l), signalStrength(s) {}
};

// 卫星观测数据
struct SatelliteData {
    std::string satelliteID;  // 卫星标识 (如 "G01", "E23")
    std::map<std::string, ObservationValue> observations;

    SatelliteData(const std::string &id) : satelliteID(id) {}
};

// 观测记录时间戳
struct ObservationTime {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    double second;

    ObservationTime(int y, int mo, int d, int h, int mi, double s)
            : year(y), month(mo), day(d), hour(h), minute(mi), second(s) {}
};

// 单个观测记录
struct ObservationRecord {
    ObservationTime time;
    std::vector<SatelliteData> satellites;

    ObservationRecord(const ObservationTime &t) : time(t) {}
};

// RINEX文件头信息
struct RinexHeader {
    double version = 3.04;
    std::string program;
    std::string observer;
    std::string marker;
    std::vector<std::string> observationTypes;
    std::string timeSystem;

    void printHeader() const {
        std::cout << "RINEX Version: " << version << std::endl;
        std::cout << "Observation Types: ";
        for (const auto &type: observationTypes) {
            std::cout << type << " ";
        }
        std::cout << "\n";
    }
};

// 完整的RINEX文件结构
class RinexFile {
public:
    RinexHeader header;
    std::vector<ObservationRecord> records;

    void addRecord(const ObservationRecord &record) {
        records.push_back(record);
    }

    void printSummary() const {
        header.printHeader();
        std::cout << "Total Records: " << records.size() << std::endl;
        if (!records.empty()) {
            std::cout << "First Record Time: "
                      << records[0].time.year << "-"
                      << records[0].time.month << "-"
                      << records[0].time.day << " "
                      << records[0].time.hour << ":"
                      << records[0].time.minute << ":"
                      << records[0].time.second << std::endl;
        }
    }
};

int main() {
    // 创建RINEX文件结构
    RinexFile rinex;

    // 设置头信息
    rinex.header.version = 3.04;
    rinex.header.observationTypes = {"C1C", "L1C", "D1C", "S1C"};
    rinex.header.timeSystem = "GPS";

    // 创建第一个观测记录
    ObservationTime t1(2023, 10, 5, 12, 0, 0.0);
    ObservationRecord rec1(t1);

    // 添加GPS卫星数据
    SatelliteData gps1("G01");
    gps1.observations["C1C"] = {23456789.123, 0, 7};
    gps1.observations["L1C"] = {123456789.123, 1, 8};
    rec1.satellites.push_back(gps1);

    // 添加Galileo卫星数据
    SatelliteData gal1("E11");
    gal1.observations["C1C"] = {34567890.456, 0, 6};
    gal1.observations["L1C"] = {234567890.456, 0, 7};
    rec1.satellites.push_back(gal1);

    rinex.addRecord(rec1);

    // 创建第二个观测记录
    ObservationTime t2(2023, 10, 5, 12, 0, 30.0);
    ObservationRecord rec2(t2);

    SatelliteData gps2("G02");
    gps2.observations["C1C"] = {34567891.234, 0, 6};
    gps2.observations["L1C"] = {134567891.234, 0, 7};
    rec2.satellites.push_back(gps2);

    rinex.addRecord(rec2);

    // 打印摘要信息
    rinex.printSummary();

    // 打印详细数据
    std::cout << "\nDetailed Data:\n";
    for (const auto &record: rinex.records) {
        std::cout << "\nTime: " << record.time.year << "-"
                  << record.time.month << "-" << record.time.day << " "
                  << record.time.hour << ":" << record.time.minute << ":"
                  << record.time.second << std::endl;

        for (const auto &sat: record.satellites) {
            std::cout << "Satellite: " << sat.satelliteID << std::endl;
            for (const auto &obs: sat.observations) {
                std::cout << "  " << obs.first << ": " << obs.second.value
                          << " (LLI: " << obs.second.lli
                          << ", SS: " << obs.second.signalStrength << ")\n";
            }
        }
    }

    return 0;
}
//
// Created by shjzh on 2025/2/17.
//
#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>

// 定义卫星系统的枚举类型
enum class SatelliteSystem {
    GPS,
    GLONASS,
    GALILEO,
    BEIDOU,
    // 可以添加更多卫星系统
};

// 定义一个结构来联合表达卫星系统和整数值
struct SatelliteID {
    SatelliteSystem system;
    int prn; // 卫星编号 (PRN for GPS, Slot number for Galileo, etc.)

    bool operator==(const SatelliteID &other) const {
        return system == other.system && prn == other.prn;
    }
};

// 为了在unordered_map中使用SatelliteID作为键，需要定义hash函数
namespace std {
    template<>
    struct hash<SatelliteID> {
        std::size_t operator()(const SatelliteID &id) const {
            return hash<int>()(static_cast<int>(id.system)) ^ hash<int>()(id.prn);
        }
    };
}

// 定义一个类来封装GPS时间 (周和周内秒)
class GPSTime {
public:
    int gpsWeek; // GPS周
    double gpsTOW; // 周内秒

    GPSTime(int week, double tow) : gpsWeek(week), gpsTOW(tow) {}

    // 计算与另一个GPSTime实例的时间差（绝对值）
    double timeDifference(const GPSTime &other) const {
        return std::fabs((gpsWeek - other.gpsWeek) * 604800 + gpsTOW - other.gpsTOW);
    }
};

struct GPSEphemerisData {
    GPSTime time; // 使用GPSTime类表示的时间
    // 简化的广播星历参数，实际应用中需要更多参数
    double toe;      // 星历参考时间 (ephemeris reference time)
    double af0, af1, af2; // 时钟改正项
    double iode, iodc; // 星历数据期次
    double M0, delta_n; // 平近点角, 平均运动的改正
    double e;        // 轨道偏心率
    double sqrtA;    // 轨道半长轴的平方根
    double omega0, i0, w; // 升交点赤经, 轨道倾角, 近地点角距

    // 构造函数方便初始化
    GPSEphemerisData(int week, double tow,
                     double _toe, double _af0, double _af1, double _af2,
                     double _iode, double _iodc, double _M0, double _delta_n,
                     double _e, double _sqrtA, double _omega0, double _i0, double _w)
            : time(week, tow),
              toe(_toe), af0(_af0), af1(_af1), af2(_af2),
              iode(_iode), iodc(_iodc), M0(_M0), delta_n(_delta_n),
              e(_e), sqrtA(_sqrtA), omega0(_omega0), i0(_i0), w(_w) {}
};

using EphemerisMap = std::unordered_map<SatelliteID, std::vector<GPSEphemerisData>>;

// 查找与给定时间和卫星号最近的观测历元的星历数据
const GPSEphemerisData *
findNearestEpoch(const EphemerisMap &ephemerisMap, const SatelliteID &satelliteID, const GPSTime &targetTime) {
    auto it = ephemerisMap.find(satelliteID);
    if (it != ephemerisMap.end()) {
        auto &data = it->second;
        auto nearestIt = std::min_element(data.begin(), data.end(),
                                          [&targetTime](const GPSEphemerisData &a, const GPSEphemerisData &b) -> bool {
                                              return a.time.timeDifference(targetTime) <
                                                     b.time.timeDifference(targetTime);
                                          });
        return &(*nearestIt);
    }
    return nullptr; // 如果没有找到匹配的卫星号，返回空指针
}

int main() {
    EphemerisMap ephemerisMap;

    // 示例数据添加
    SatelliteID satId{SatelliteSystem::GPS, 1};
    ephemerisMap[satId].emplace_back(2137, 3600, 3600, 1e-9, 1e-12, 0, 1, 1, 0.1, 0.0001, 0.5, 5153.6, 0.9, 0.7, 0.3);
    ephemerisMap[satId].emplace_back(2137, 7200, 7200, 1e-9, 1e-12, 0, 1, 1, 0.1, 0.0001, 0.5, 5153.6, 0.9, 0.7, 0.3);

    GPSTime searchTime(2137, 6000);
    const GPSEphemerisData *result = findNearestEpoch(ephemerisMap, satId, searchTime);

    if (result != nullptr) {
        std::cout << "Found data for satellite PRN " << satId.prn << " in system " << static_cast<int>(satId.system)
                  << " at GPS week " << result->time.gpsWeek << ", TOW " << result->time.gpsTOW << std::endl;
    } else {
        std::cout << "No matching data found." << std::endl;
    }

    return 0;
}
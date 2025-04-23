#include <iostream>
#include <string>

using namespace std;

struct SatID {
    string system;
    int id;

    // 构造函数
    SatID() : system(""), id(-1) {}

    // 从字符串构造函数
    SatID(string satStr) {
        system = satStr.substr(0, 1);
        id = stoi(satStr.substr(1, 2));
    }

    // Overload the equality operator as a member function
    bool operator==(const SatID &other) const {
        return this->system == other.system && this->id == other.id;
    }

    // Overload the less-than operator as a member function
    bool operator<(const SatID &other) const {
        if (this->system != other.system)
            return this->system < other.system;
        return this->id < other.id;
    }
};

// 全局重载的 << 运算符
ostream &operator<<(ostream &os, const SatID &satid) {
    os << satid.system << satid.id;
    return os;
}

int main() {
    SatID sat1("G15");
    SatID sat2("C03");

    cout << "Satellite ID: " << sat1 << endl;
    cout << "Satellite ID: " << sat2 << endl;

    if (sat1 < sat2) {
        cout << "sat1 is less than sat2" << endl;
    } else {
        cout << "sat1 is not less than sat2" << endl;
    }

    if (sat1 == sat2) {
        cout << "sat1 and sat2 are equal" << endl;
    } else {
        cout << "sat1 and sat2 are not equal" << endl;
    }

    return 0;
}//
// Created by shjzh on 2025/2/25.
//

#ifndef CONFIGREADER_H
#define CONFIGREADER_H

#include <map>
#include <string>
#include <stdexcept>

class ConfigReader {
private:
    std::map<std::string, std::string> config;

    bool isComment(const std::string &line);

    bool isEmpty(const std::string &line);

    int stringToInt(const std::string &str);

    double stringToDouble(const std::string &str);

    std::string getConfigValue(const std::string &key);

public:
    ConfigReader(const std::string &filename);

    int getValueAsInt(const std::string &key);

    std::string getValueAsString(const std::string &key);

    bool getValueAsBool(const std::string &key);

    double getValueAsDouble(const std::string &key);
};

#endif // CONFIGREADER_H

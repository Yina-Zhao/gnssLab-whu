//
// Created by shjzh on 2024/12/24.
//
#include "ConfigReader.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cctype>

bool ConfigReader::isComment(const std::string &line) {
    return line.find("#") == 0 || line.empty();
}

bool ConfigReader::isEmpty(const std::string &line) {
    return line.find_first_not_of(" \t\n\r\f\v") == std::string::npos;
}

ConfigReader::ConfigReader(const std::string &filename) {
    std::ifstream file(filename);
    std::string line;

    if (file.is_open()) {
        while (getline(file, line)) {
            line.erase(0, line.find_first_not_of(" \t\n\r\f\v")); // Trim leading whitespace
            if (!isComment(line) && !isEmpty(line)) {
                size_t delimiterPos = line.find("=");
                if (delimiterPos != std::string::npos) {
                    std::string key = line.substr(0, delimiterPos);
                    std::string value = line.substr(delimiterPos + 1);
                    // Trim whitespace from key and value
                    key.erase(key.find_last_not_of(" \t\n\r\f\v") + 1);
                    value.erase(0, value.find_first_not_of(" \t\n\r\f\v"));
                    config[key] = value;
                }
            }
        }
        file.close();
    } else {
        throw std::runtime_error("Unable to open file: " + filename);
    }
}

int ConfigReader::stringToInt(const std::string &str) {
    try {
        return std::stoi(str);
    } catch (const std::exception &e) {
        throw std::runtime_error("Invalid integer value: " + str);
    }
}

double ConfigReader::stringToDouble(const std::string &str) {
    try {
        return std::stod(str);
    } catch (const std::exception &e) {
        throw std::runtime_error("Invalid double value: " + str);
    }
}

std::string ConfigReader::getConfigValue(const std::string &key) {
    auto it = config.find(key);
    if (it == config.end()) {
        throw std::runtime_error("Key not found: " + key);
    }
    return it->second;
}

int ConfigReader::getValueAsInt(const std::string &key) {
    return stringToInt(getConfigValue(key));
}

std::string ConfigReader::getValueAsString(const std::string &key) {
    return getConfigValue(key);
}

bool ConfigReader::getValueAsBool(const std::string &key) {
    std::string value = getConfigValue(key);
    return value == "1" || value == "true" || stringToInt(value) != 0;
}

double ConfigReader::getValueAsDouble(const std::string &key) {
    return stringToDouble(getConfigValue(key));
}
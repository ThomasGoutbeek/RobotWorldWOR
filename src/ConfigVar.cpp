#include "ConfigVar.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>

bool ConfigVar::dataParsed = false;
std::map<std::string, std::map<std::string, std::string>> ConfigVar::data;

ConfigVar::ConfigVar(const std::string& filename) {
    if(!dataParsed){
        parseFile(filename);
        dataParsed = true;
    }
}

void ConfigVar::parseFile(const std::string& filename) {
    std::ifstream file(filename);

    std::string line;
    std::string section;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        std::string value;

        if (line[0] == '[' && line.back() == ']') {
            section = line.substr(1, line.size() - 2);
        } else {
            std::getline(iss, key, '=');
            std::getline(iss, value);
            data[section][key] = value;
        }
    }
}

std::string ConfigVar::getVar(const std::string& section, const std::string& key) const {
    auto secIt = data.find(section);
    if (secIt != data.end()) {
        auto keyIt = secIt->second.find(key);
        if (keyIt != secIt->second.end()) {
            return keyIt->second;
        }
    }
    return "";
}
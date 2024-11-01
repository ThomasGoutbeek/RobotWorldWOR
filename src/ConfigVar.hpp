#ifndef CONFIGVAR_HPP_
#define CONFIGVAR_HPP_

#include <string>
#include <map>
#include <iostream>

class ConfigVar {
    public:
        ConfigVar(const std::string& filename);
        virtual ~ConfigVar(){

        };
        std::string getVar(const std::string& section, const std::string& key) const;
        void parseFile(const std::string& filename);

    private:
        static bool dataParsed;
        static std::map<std::string, std::map<std::string, std::string>> data;
};

#endif // CONFIGVAR_HPP
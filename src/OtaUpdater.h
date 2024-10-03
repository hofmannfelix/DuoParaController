#ifndef OtaUpdater_h
#define OtaUpdater_h

#include <string>



class OtaUpdater {
    const std::string ssid = "Felix iPhone";
    const std::string password = "1wskwrp0flxv";
    
    bool isActive = false;
    bool isConnected = false;

public:
    OtaUpdater();
    void activate();
    std::string getUpdateUrl();
};

#endif
#include "EscData.h"
#include "arduino.h"

EscData::EscData() : escRpmBuffer() {}

bool EscData::isRpmChangeToLarge(EscData &escData) {
    if (escData.escRpmBuffer.capacity != escData.escRpmBuffer.size() || 
        escRpmBuffer.capacity != escRpmBuffer.size()) return false;

    if (avgRpm() < MIN_RPM_FOR_DIFF_SHUTOFF && escData.avgRpm() < MIN_RPM_FOR_DIFF_SHUTOFF) return false;

    if (abs(escData.escRpmBuffer.last() - escRpmBuffer.last()) > SHUTOFF_RPM_DIFF) {
        if (escData.escRpmBuffer.first() - escRpmBuffer.last() > SHUTOFF_RPM_DIFF) return true;
        if (escRpmBuffer.first() - escData.escRpmBuffer.last() > SHUTOFF_RPM_DIFF) return true;
    }
    return false;
}

int EscData::avgRpm() {
    int avg = 0;
    for (int i = 0; i < escRpmBuffer.size(); i++) avg += escRpmBuffer[i];
    if (escRpmBuffer.size() > 0) avg /= escRpmBuffer.size();
    return avg;
}

bool EscData::isLastRpmUpdateExceeded() {
    return millis() - lastRpmUpdate > SHUTOFF_LAST_UPDATE_MS;
}
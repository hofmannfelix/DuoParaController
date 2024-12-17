#include "EscData.h"
#include "arduino.h"

EscData::EscData() : escRpmBuffer() {}

bool EscData::isRpmChangeToLarge(EscData &escData) {
    int highestDiffIndex = -1;
    int maxDiff = 0;

    if (escData.escRpmBuffer.capacity != escData.escRpmBuffer.size() || 
        escRpmBuffer.capacity != escRpmBuffer.size()) return false;

    int avg = 0, escDataAvg = 0;
    for (int i = 0; i < escRpmBuffer.size(); i++) {
        avg += escRpmBuffer[i];
        escDataAvg += escData.escRpmBuffer[i];
    }
    avg /= escRpmBuffer.size();
    escDataAvg /= escData.escRpmBuffer.size();
    auto diff = abs(avg - escDataAvg);
    auto shutoffPercentage = SHUTOFF_RPM_DIFF_PERCENT * 0.01;

    if (avg < MIN_RPM_FOR_DIFF_SHUTOFF && escDataAvg < MIN_RPM_FOR_DIFF_SHUTOFF)
        return false;
    else
        return diff > max(avg, escDataAvg) * shutoffPercentage;
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
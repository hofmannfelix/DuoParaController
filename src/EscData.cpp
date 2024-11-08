#include "EscData.h"
#include "arduino.h"

EscData::EscData() : escRpmBuffer() {}

bool EscData::isRpmChangeToLarge(EscData &escData) {
    int highestDiffIndex = -1;
    int maxDiff = 0;
    if (escData.escRpmBuffer.capacity != escData.escRpmBuffer.size() || 
        escRpmBuffer.capacity != escRpmBuffer.size()) return 0;

    int previousAvg = 0, avg = 0, escDataPreviousAvg = 0, escDataAvg = 0;
    for (int i = 0; i < escRpmBuffer.size()/2; i++) {
        avg += escRpmBuffer[i];
        escDataAvg += escData.escRpmBuffer[i];
        previousAvg += escRpmBuffer[i + escRpmBuffer.size()/2];
        escDataPreviousAvg += escData.escRpmBuffer[i + escRpmBuffer.size()/2];
    }
    avg /= 2;
    escDataAvg /= 2;
    previousAvg /= 2;
    escDataPreviousAvg /= 2;

    auto rpmChangeRate = abs(previousAvg - avg);
    auto escDataRpmChangeRate = abs(escDataPreviousAvg - escDataAvg);
    auto rpmChangeDiff = abs(rpmChangeRate - escDataRpmChangeRate);

    return rpmChangeRate > rpmChangeRate * SHUTOFF_RPM_DIFF_PERCENT * 0.01;
}

bool EscData::isLastRpmUpdateExceeded() {
    return millis() - lastRpmUpdate > SHUTOFF_LAST_UPDATE_MS;
}
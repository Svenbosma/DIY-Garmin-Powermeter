void CadencePowerCalc(){
    float avgTorque = (torqueSampleCount > 0) ? (sumTorqueNm / torqueSampleCount) : 0.0f;
    float avgGyro = (imuSampleCount > 0) ? (sumGyroX / imuSampleCount) : 0.0f;

    unsigned long now = millis();
    float dt = (now - lastRevMs) / 1000.0f;
    lastRevMs = now;
    if (dt < 0.2f) dt = 0.2f;
    float cadence_rpm = 60.0f / dt;

    float angVelRad = avgGyro * DEG_TO_RAD;
    float powerW = calcPower(avgTorque, angVelRad);

    cumulativeCrankRevs++;
    lastCrankEventTime = (uint16_t)((now * 1024UL) / 1000UL);

    sendCyclingPowerMeasurement((int16_t)powerW);

    sumTorqueNm = 0.0f; torqueSampleCount = 0;
    sumGyroX = 0.0f; sumAccelX = 0.0f; imuSampleCount = 0;

    logPrint("Cadence="); logPrint(cadence_rpm,1);
    logPrint("rpm  Power="); logPrintln(powerW,1);
}
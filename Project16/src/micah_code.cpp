void currentCheck_task() {
  Serial.println("CURRENT CHECK START");

  // --- CONFIG ---
  const float SENS_V_PER_A = 0.066f;  // ACS712-30A = 66 mV/A (before divider)
  const float DIV_RATIO = 2.0f / 3.0f; // 1 k / 2 k divider ⇒ Vadc = Vsensor × 2/3
  const int samples = 400;             // good for 50/60 Hz RMS
  const int sampleDelay_us = 200;      // ≈5 kS/s
  double sumSq = 0;
  double sumMv = 0;

  // --- SAMPLE LOOP ---
  for (int i = 0; i < samples; ++i) {
    int mv = analogReadMilliVolts(sensorPin);   // ESP32 returns mV, auto-scaled
    float deltaMv_adc = mv - (zeroOffsetV * 1000.0f);  // remove offset (ADC domain)
    float deltaMv_sensor = deltaMv_adc / DIV_RATIO;    // undo divider
    float instCurrent = (deltaMv_sensor / 1000.0f) / SENS_V_PER_A;
    sumSq += instCurrent * instCurrent;
    sumMv += mv;
    delayMicroseconds(sampleDelay_us);
  }

  // --- RMS COMPUTE ---
  float Irms = sqrt(sumSq / samples);
  float avgMv = sumMv / samples;

  // slow auto-trim offset when no current
  if (Irms < 0.05f) {
    zeroOffsetV = 0.995f * zeroOffsetV + 0.005f * (avgMv / 1000.0f);
  }

  AcsValueF = Irms;
  currentValue = Irms;

  Serial.printf("Zero offset: %.3f V | Irms: %.4f A\n", zeroOffsetV, Irms);

  printLine(1, "I:" + String(Irms, 4) + " A RMS");

  // --- ALARM ---
  if (Irms > 1.0f) {
    Serial.println("** ALERT: Overcurrent condition! **");
    isAlarmBuzzing(true, 100, 5);
    printLine(0, "** OVERCURRENT! **");
    isCooldown = true;
  }
}
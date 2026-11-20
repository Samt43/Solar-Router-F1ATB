/* Calibrates the potentiometer of the ZMPT101B module */
/* ----------------------------------------------------*/
#include <stdint.h>

/* Instructions:
1. Set up the hardware
2. Plug the ZMPT101B to an AC voltage source. This voltage should be close to the voltage you will eventually read during the operation of this device. Do not connect to AC sources that generate square waves (no-break, inverters, etc...).
3. Edit *calibrate-pot.ino* and change the GPIO pin according to your hardware.
4. Compile and upload the code from Arduino IDE.
5. Open the Serial Plotter view, set baud rate to 115200bps.
6. Turn the potentiometer screw clockwise or counterclockwise until you see that the bottom and top of the curves transition from flat to curved. Do not exceed too much the transition zone (between flat top and curved top) because it decreases the sensibility of the sensor.

Acquisition waits 200 us between sample pairs, plus ADC read time.
Each 1000-sample capture takes over 200 ms.

Live phase measurement:
After each capture the mains frequency and the phase difference between
CURRENT_PIN and ZMPT101B_PIN are computed (Hann-windowed DFT on real timestamps).
All floating-point computations use double precision.
Both results are printed: without correction (shared timestamp) and with correction
for the delay between the two sequential ADC reads.
Phase < 0 means the current lags the voltage. A resistive load should give ~0 deg.
Set DUMP_RAW to true to plot voltage and current in Tools > Serial Plotter
(115200 baud); the text summary is then not printed.
*/

// Pin configuration
#define ZMPT101B_PIN 34
#define CURRENT_PIN 35

const unsigned long SAMPLE_INTERVAL_US = 200;
const int SAMPLE_COUNT = 1000;
const bool DUMP_RAW = true;  // true: plot raw samples; false: print phase summary
const unsigned long PAUSE_MS = 1000;

// Keep capture buffers out of the ESP32 task stack; ADC readings fit in 16 bits.
static uint16_t zmpt_value[SAMPLE_COUNT];
static uint16_t current_value[SAMPLE_COUNT];
static uint32_t t_start_us[SAMPLE_COUNT];
static double v_ac[SAMPLE_COUNT];
static double i_ac[SAMPLE_COUNT];
static double t_s[SAMPLE_COUNT];  // seconds, shared by both channels
static double win[SAMPLE_COUNT];

// Windowed DFT of x sampled at times t, at frequency f.
static void dft(const double *x, const double *t, double f, double &re, double &im)
{
  const double w = 2.0 * PI * f;
  re = 0;
  im = 0;
  for (int k = 0; k < SAMPLE_COUNT; k++) {
    const double a = w * t[k];
    re += win[k] * x[k] * cos(a);
    im -= win[k] * x[k] * sin(a);
  }
}

static double dftPower(const double *x, const double *t, double f)
{
  double re, im;
  dft(x, t, f, re, im);
  return re * re + im * im;
}

static void removeDC(const uint16_t *samples, double *ac)
{
  double mean = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    mean += samples[i];
  }
  mean /= SAMPLE_COUNT;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    ac[i] = samples[i] - mean;
  }
}

static void searchFrequency(double start, double end, double step, double &bestF, double &bestP)
{
  for (double f = start; f <= end; f += step) {
    const double p = dftPower(v_ac, t_s, f);
    if (p > bestP) { bestP = p; bestF = f; }
  }
}

static double wrapPhase(double phase)
{
  while (phase > 180.0) phase -= 360.0;
  while (phase <= -180.0) phase += 360.0;
  return phase;
}

void setup()
{
  Serial.begin(115200);

// Analog attenuation: When using 0-1V sensors the attenuation should be decreased to improve accuracy.
//    ADC_0db: sets no attenuation. ADC can measure up to approximately 800 mV (1V input = ADC reading of 1088).
//    ADC_2_5db: The input voltage of ADC will be attenuated, extending the range of measurement to up to approx. 1100 mV. (1V input = ADC reading of 3722).
//    ADC_6db: The input voltage of ADC will be attenuated, extending the range of measurement to up to approx. 1350 mV. (1V input = ADC reading of 3033).
//    ADC_11db (default): The input voltage of ADC will be attenuated, extending the range of measurement to up to approx. 2600 mV. (1V input = ADC reading of 1575).

analogSetAttenuation(ADC_11db);

  pinMode(ZMPT101B_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);

  for (int k = 0; k < SAMPLE_COUNT; k++) {
    win[k] = 0.5 - 0.5 * cos(2.0 * PI * k / (SAMPLE_COUNT - 1));
  }
}

void loop()
{
  // Capture (voltage first, then current), timing each ADC read
  double readV_us = 0, readI_us = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    const uint32_t ta = micros();
    zmpt_value[i] = analogRead(ZMPT101B_PIN);
    const uint32_t tb = micros();
    current_value[i] = analogRead(CURRENT_PIN);
    const uint32_t tc = micros();
    t_start_us[i] = ta;
    readV_us += tb - ta;
    readI_us += tc - tb;
    delayMicroseconds(SAMPLE_INTERVAL_US);
  }
  readV_us /= SAMPLE_COUNT;
  readI_us /= SAMPLE_COUNT;

  // Remove DC and build timestamps (seconds, relative to first sample)
  removeDC(zmpt_value, v_ac);
  removeDC(current_value, i_ac);
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    t_s[i] = (t_start_us[i] - t_start_us[0]) * 1e-6;
  }
  const double sampleInterval_us = (t_start_us[SAMPLE_COUNT - 1] - t_start_us[0])
                                  / (double)(SAMPLE_COUNT - 1);

  // Mains frequency: coarse then fine search on the voltage spectrum peak
  double bestF = 50.0, bestP = -1;
  searchFrequency(45.0, 65.0, 0.1, bestF, bestP);
  const double f0 = bestF;
  searchFrequency(f0 - 0.1, f0 + 0.1, 0.005, bestF, bestP);

  double vr, vi, ir, ii;
  dft(v_ac, t_s, bestF, vr, vi);
  dft(i_ac, t_s, bestF, ir, ii);

  const double phase = wrapPhase((atan2(ii, ir) - atan2(vi, vr)) * 180.0 / PI);
  const double dt_ms = phase / 360.0 / bestF * 1000.0;

  // The current is read after the voltage, by (readV + readI) / 2 between read centres.
  const double seqDelay_us = 0.5 * (readV_us + readI_us);
  const double phaseCorr = wrapPhase(phase - 360.0 * bestF * seqDelay_us * 1e-6);

  // Amplitude of a sine through a Hann window: |X| = A * N / 4
  const double norm = 0.25 * SAMPLE_COUNT;
  // The Serial Plotter cannot parse text lines, so the summary is skipped while plotting.
  if (!DUMP_RAW) {
    Serial.printf("f=%.3f Hz  phase(I-V): without corr=%.2f deg, with corr=%.2f deg (seq delay %.1f us)  dt(no corr)=%.3f ms  Vamp=%.0f Iamp=%.0f  Ts=%.1f us\n",
                  bestF, phase, phaseCorr, seqDelay_us, dt_ms,
                  sqrt(vr * vr + vi * vi) / norm,
                  sqrt(ir * ir + ii * ii) / norm,
                  sampleInterval_us);
  }

  if (DUMP_RAW) {
    // Both plotted channels share timestamps, so align using the uncorrected phase.
    const double Ts_s = sampleInterval_us * 1e-6;
    const double lagSamples = -phase / 360.0 / (bestF * Ts_s);
    for (int i = 0; i < SAMPLE_COUNT; i++) {
      const double pos = constrain(i - lagSamples, 0.0, (double)(SAMPLE_COUNT - 1));
      const int j = min((int)pos, SAMPLE_COUNT - 2);
      const double frac = pos - j;
      const double shifted = (1.0 - frac) * zmpt_value[j] + frac * zmpt_value[j + 1];
      Serial.print("ZMPT101B_PIN:");
      Serial.print(zmpt_value[i]);
      Serial.print(",");
      Serial.print("ZMPT101B_SHIFTED:");
      Serial.print(shifted, 1);
      Serial.print(",");
      Serial.print("CURRENT_PIN:");
      Serial.println(current_value[i]);
    }
  }

  delay(PAUSE_MS);
}

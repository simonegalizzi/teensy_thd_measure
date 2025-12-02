/*******************************************************
   THD+N robusto con Teensy 4.x + Audio Shield
   Migliorie: check clipping, interp. parabolica, ±1 bin sum,
   smoothing (EMA) THD, stampa noisefloor
********************************************************/

#include <Arduino.h>
#include <Audio.h>
#include <arm_math.h>

#define FFT_SIZE 1024
#define BLOCK_SIZE 128
#define FS 44100.0f

AudioInputI2S        i2s_in;
AudioRecordQueue     queue_in;
AudioFilterStateVariable filter1;        //xy=680,330
//AudioConnection          patchCord1(i2s_in, 0, filter1, 0);
AudioConnection      patchCord(i2s_in, 1, queue_in, 0);
AudioControlSGTL5000 sgtl5000_1;

float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t window[FFT_SIZE];
float32_t mag[FFT_SIZE/2];

// Parametri migliorativi
const int HARM_MAX = 100;       // numero armoniche da cercare (si ferma a Nyquist)
const int BIN_SUM_RADIUS = 1;  // somma energia su bin-1..bin+1
const float WINDOW_GAIN_CORR = 2.0f; // correzione Hann (1/0.5)

unsigned long time_acquire = 5000;
unsigned long Starttime;

// smoothing THD
float thd_ema = 0.0f;
const float EMA_ALPHA = 0.2f; // 0.0..1.0 (più alto = meno smoothing)

void makeHannWindow() {
  for (int i=0; i < FFT_SIZE; i++) {
    window[i] = 0.5f * (1.f - cosf(2.f * PI * i / (FFT_SIZE - 1)));
  }
}

// parabolic interpolation: ritorna (delta, peak)
// delta = spostamento in bins da 0 (range -0.5..+0.5 tipicamente)
// y = array dei valori (lineari, non log)
void parabolicInterp(float ym1, float y0, float yp1, float &delta, float &peak) {
  float denom = (ym1 - 2.0f*y0 + yp1);
  if (fabs(denom) < 1e-12f) {
    delta = 0.0f;
    peak = y0;
    return;
  }
  delta = 0.5f * (ym1 - yp1) / denom;
  // peak value (parabola apex) estimated:
  peak = y0 - 0.25f * (ym1 - yp1) * delta;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("THD robusto: avvio...");

  AudioMemory(30);
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(AUDIO_INPUT_LINEIN);

  // riduci lineInLevel durante le misure per evitare clipping
  sgtl5000_1.lineInLevel(5);

  makeHannWindow();
  queue_in.begin();
}
void THD()
{
  if (queue_in.available() < 8) return;

  // ACQUISIZIONE 1024 campioni
  uint16_t idx = 0;
  float maxAbs = 0.0f;
  for (int b=0; b<8; b++) {
    int16_t *block = queue_in.readBuffer();
    if (!block) continue;
    for (int i=0; i < BLOCK_SIZE; i++) {
      float v = (float)block[i] / 32768.0f; // normalizza
      fft_input[idx++] = v;
      float av = fabsf(v);
      if (av > maxAbs) maxAbs = av;
    }
    queue_in.freeBuffer();
  }

  // diagnosi clipping
  bool clipping = (maxAbs > 0.99f);

  // applica finestra
  for (int i=0; i<FFT_SIZE; i++) fft_input[i] *= window[i];

  // FFT (real)
  arm_rfft_fast_instance_f32 S;
  arm_rfft_fast_init_f32(&S, FFT_SIZE);
  arm_rfft_fast_f32(&S, fft_input, fft_output, 0);

  // magnitudini e normalizzazione (divide per N e corregge finestra)
  for (int i=0; i < FFT_SIZE/2; i++) {
    float re = fft_output[2*i];
    float im = fft_output[2*i+1];
    mag[i] = sqrtf(re*re + im*im) / (float)FFT_SIZE * WINDOW_GAIN_CORR;
  }

  // trova il picco fondamentale (ignora DC)
  int fund_bin = 3;
  float fund_val = mag[fund_bin];
  for (int i=4; i < FFT_SIZE/2; i++) {
    if (mag[i] > fund_val) {
      fund_val = mag[i];
      fund_bin = i;
    }
  }

  // interpolazione parabolica del picco per miglior ampiezza
  float interpDelta=0.0f, interpPeak=0.0f;
  {
    int b0 = fund_bin;
    float y_m = (b0-1 >= 1) ? mag[b0-1] : mag[b0];
    float y_0 = mag[b0];
    float y_p = (b0+1 < FFT_SIZE/2) ? mag[b0+1] : mag[b0];
    parabolicInterp(y_m, y_0, y_p, interpDelta, interpPeak);
    // usa interpPeak come A1 corretto
    fund_val = interpPeak;
  }

  // Calcolo potenza armoniche: somma su ±BIN_SUM_RADIUS bins attorno ad ogni armonica
  double sumHarmPow = 0.0;
  for (int h = 2; h <= HARM_MAX; h++) {
    int hb = fund_bin * h;
    if (hb >= FFT_SIZE/2) break;
    // somma su -R..+R
    double powHB = 0.0;
    for (int r = -BIN_SUM_RADIUS; r <= BIN_SUM_RADIUS; r++) {
      int b = hb + r;
      if (b >= 1 && b < FFT_SIZE/2) powHB += (double)mag[b] * (double)mag[b];
    }
    sumHarmPow += powHB;
  }

  // THD classico (2..H)
  float thd = (fund_val > 1e-12f) ? sqrtf((float)sumHarmPow) / fund_val : 0.0f;

  // THD+N: energia totale eccetto DC, sottrai potenza della fondamentale (sommata su ±R)
  double totalPow = 0.0;
  for (int b=1; b < FFT_SIZE/2; b++) totalPow += (double)mag[b] * (double)mag[b];

  // potenza fondamentale (±R)
  double fundPow = 0.0;
  for (int r = -BIN_SUM_RADIUS; r <= BIN_SUM_RADIUS; r++) {
    int b = fund_bin + r;
    if (b >= 1 && b < FFT_SIZE/2) fundPow += (double)mag[b] * (double)mag[b];
  }

  double thdnPow = totalPow - fundPow;
  if (thdnPow < 0) thdnPow = 0;
  float thdn = (fund_val > 1e-12f) ? sqrtf((float)thdnPow) / fund_val : 0.0f;

  // smoothing THD (EMA)
  thd_ema = EMA_ALPHA * thd + (1.0f - EMA_ALPHA) * thd_ema;

  // noise floor estimate: median of low-energy bins or RMS of bins excluding harmonics
  // semplice estimate: energia medio bin (escludi primi 4 e ±harmoniche)
  double noisePow = 0.0;
  int noiseBins = 0;
  for (int b = 4; b < FFT_SIZE/2; b++) {
    // skip harmonic regions
    bool skip = false;
    for (int h=1; h <= HARM_MAX; h++) {
      int hb = fund_bin * h;
      if (hb < 1 || hb >= FFT_SIZE/2) break;
      if (abs(b - hb) <= BIN_SUM_RADIUS) { skip = true; break; }
    }
    if (skip) continue;
    noisePow += (double)mag[b] * (double)mag[b];
    noiseBins++;
  }
  float noiseRms = 0.0f;
  if (noiseBins > 0) noiseRms = sqrtf((float)(noisePow / noiseBins));

  // PRINT diagnostico
  Serial.print("F0=");
  Serial.print(fund_bin * (FS/FFT_SIZE), 2);
  Serial.print("Hz bin=");
  Serial.print(fund_bin);
  Serial.print("  A1=");
  Serial.print(fund_val, 6);
  Serial.print("  THD=");
  Serial.print(thd, 6);
  Serial.print("  THD_EMA=");
  Serial.print(thd_ema, 6);
  Serial.print("  THD+N=");
  Serial.print(thdn, 6);
  Serial.print("  NoiseRMS=");
  Serial.print(noiseRms, 6);
  Serial.print("  MaxSample=");
  Serial.print(maxAbs, 6);
  Serial.print("  Clipping=");
  Serial.println(clipping ? "YES" : "NO");

  // opzionale: invio spettro CSV (disabilita se vuoi solo diagnostica)
  // for (int i=0; i<FFT_SIZE/2; i++) {
   //  Serial.print(mag[i], 6);
   //  if (i < FFT_SIZE/2-1) Serial.print(",");
   //  }
  // Serial.println();

  //delay(40);
}
void loop() {
 if(Serial.available())
  {
      String ch;
      ch = Serial.readString();
      ch.trim();
      if(ch=="thd?"||ch=="THD?")
      {
         Starttime = millis();
         while((millis() - Starttime) < time_acquire)
        {
         THD();
        }
      }
   } 
}

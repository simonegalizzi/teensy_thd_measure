/*******************************************************
   THD con Teensy 4.x + Audio Shield
   Ingresso: LINE-IN
   FFT CMSIS su 1024 campioni reali
********************************************************/

#include <Arduino.h>
#include <Audio.h>
#include <arm_math.h>

// ---------- AUDIO SYSTEM ----------
AudioInputI2S           i2s_in;      // prende da LINE-IN
AudioRecordQueue        queue_in;    // buffer DMA dall'audio shield
AudioConnection         patchCord(i2s_in, 0, queue_in, 0);

AudioControlSGTL5000    sgtl5000_1;  // codec della shield

unsigned long time_acquire = 5000;
unsigned long Starttime;
// ---------- FFT SETTINGS ----------
#define FFT_SIZE   1024
float32_t fft_input[FFT_SIZE];
float32_t fft_output[FFT_SIZE];
float32_t window[FFT_SIZE];

// ---------- FUNZIONI ----------
void makeHannWindow() {
  for (int i=0; i < FFT_SIZE; i++) {
    window[i] = 0.5f * (1.f - cosf(2.f * PI * i / (FFT_SIZE - 1)));
  }
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Avvio sistema THD...");

  AudioMemory(30);            // memoria sufficiente per DMA
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000_1.lineInLevel(10);     // gain line-in

  makeHannWindow();

  queue_in.begin();           // avvia il buffer DMA

  Serial.println("Pronto. Inizio acquisizione...");
}
void THD()
{
   // serve avere 1024 campioni = 1024/128 = 8 blocchi audio
  if (queue_in.available() < 8) return;

  // ----------- ACQUISIZIONE 1024 CAMPIONI -----------
  uint16_t index = 0;

  for (int b=0; b < 8; b++) {
    int16_t *block = queue_in.readBuffer();
    for (int i=0; i < 128; i++) {
      fft_input[index++] = (float32_t)block[i];
    }
    queue_in.freeBuffer();
  }

  // ----------- APPLICA FINESTRA HANN -----------
  for (int i = 0; i < FFT_SIZE; i++) {
    fft_input[i] *= window[i];
  }

  // ----------- FFT CMSIS -----------
  arm_rfft_fast_instance_f32 S;
  arm_rfft_fast_init_f32(&S, FFT_SIZE);
  arm_rfft_fast_f32(&S, fft_input, fft_output, 0);

  // magnitudo (solo metà spettro)
  float32_t mag[FFT_SIZE/2];
  for (int i = 0; i < FFT_SIZE/2; i++) {
    float32_t re = fft_output[2*i];
    float32_t im = fft_output[2*i+1];
    mag[i] = sqrtf(re*re + im*im);
  }

  // ----------- TROVA FONDAMENTALE -----------
  int fund_bin = 1;
  float32_t fund_val = 0;

  for (int i = 1; i < FFT_SIZE/2; i++) {
    if (mag[i] > fund_val) {
      fund_val = mag[i];
      fund_bin = i;
    }
  }

  float32_t fs = 44100.0f;
  float32_t bin_hz = fs / FFT_SIZE;
  float32_t f0 = fund_bin * bin_hz;

  // ----------- CALCOLO THD -----------
  float32_t sum_harm = 0;

  for (int h = 2; h <= 5; h++) {
    int hb = fund_bin * h;
    if (hb < FFT_SIZE/2) {
      sum_harm += mag[hb] * mag[hb];
    }
  }

  float32_t thd = sqrtf(sum_harm) / fund_val;

  // ----------- OUTPUT -----------
  Serial.printf("F0: %.1f Hz | THD: %.5f | BIN: %d\n", f0, thd, fund_bin);
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

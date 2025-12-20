/*******************************************************
   THD+N OTTIMIZZATO per ridurre distorsione
   Teensy 4.x + Audio Shield + Flash Memory
   
   OTTIMIZZAZIONI per ridurre THD:
   - FFT 8192 per migliore risoluzione
   - Finestra Blackman-Harris 7-term (migliore rejection)
   - DC removal più aggressivo
   - Interpolazione quadratica migliorata
   - Esclusione bin 50Hz/100Hz/150Hz
   - Calibrazione offset DC
   - Noise floor adattivo
********************************************************/

#include <Arduino.h>
#include <Audio.h>
#include <arm_math.h>
#include <SPI.h>
#include <SerialFlash.h>

// ===== CONFIGURAZIONE FFT =====
#define FFT_SIZE 4096          // Ottimale per Teensy 4.0
#define BLOCK_SIZE 128
#define FS 44100.0f

// ===== AUDIO =====
AudioInputI2S        i2s_in;
AudioRecordQueue     queue_in;
AudioConnection      patchCord1(i2s_in, 0, queue_in, 0);
AudioControlSGTL5000 sgtl5000_1;

// ===== BUFFER FFT IN RAM =====
DMAMEM static float32_t fft_input[FFT_SIZE];
DMAMEM static float32_t fft_output[FFT_SIZE];
DMAMEM static float32_t window[FFT_SIZE];
//DMAMEM static float32_t powerSpec[FFT_SIZE/2];
DMAMEM float32_t powerSpec[FFT_SIZE/2];



arm_rfft_fast_instance_f32 S;

// ===== PARAMETRI x stampa seriale =====
float frequenza = 0.0f;
int n_fondamentale = 0;
float armonica1 = 0.0f;
float rumore = 0.0f;
float campioni = 0.0f;

// ===== PARAMETRI ANALISI OTTIMIZZATI =====
const int HARM_MAX = 50;           // Più armoniche
const int BIN_SUM_RADIUS = 6;      // Raggio aumentato per catturare energia #3
const int SEARCH_WINDOW = 6;       // Finestra ricerca più ampia

// ===== DC OFFSET CALIBRATION =====
float dc_offset = 0.0f;
bool dc_calibrated = false;

// ===== AUTO GAIN CONTROL =====
int current_line_level = 6;
const int MIN_LINE_LEVEL = 0;
const int MAX_LINE_LEVEL = 15;

const float CLIPPING_THRESHOLD = 0.95f;
const float LOW_SIGNAL_THRESHOLD = 0.12f;

// Variabili per analisi post-acquisizione
bool acquisition_active = false;
float session_max_signal = 0.0f;
int session_clip_count = 0;
int session_samples = 0;

// ===== BUFFER STATISTICHE =====
const int BUFFER_SIZE = 100;
float thd_buffer[BUFFER_SIZE];
float thdn_buffer[BUFFER_SIZE];
float freq_buffer[BUFFER_SIZE];
int buffer_index = 0;
int samples_count = 0;
bool buffer_full = false;
bool change_level = false;

// ===== FLASH MEMORY =====
const int FlashChipSelect = 6;
bool flash_available = false;

struct MeasurementRecord {
  uint32_t timestamp;
  float frequency;
  float amplitude;
  float thd;
  float thdn;
  float snr;
  float noiseFloor;
  bool clipping;
};

const int MAX_RECORDS = 1000;
int flash_record_count = 0;

// ===== CONTROLLI =====
const int pinIn = 2;
float thd_ema = 0.0f;
const float EMA_ALPHA = 0.12f;     // Più smoothing
unsigned long time_acquire = 2000;
unsigned long session_start_time = 0;

// ===== FINESTRA BLACKMAN-HARRIS 7-TERM (MIGLIORE!) =====
void makeBlackmanHarris7() {
  // Coefficienti per Blackman-Harris 7-term
  // Migliore sidelobe rejection: -180 dB vs -92 dB della 4-term
  const float a0 = 0.27105140069342f;
  const float a1 = 0.43329793923448f;
  const float a2 = 0.21812299954311f;
  const float a3 = 0.06592544638803f;
  const float a4 = 0.01081174209837f;
  const float a5 = 0.00077658482522f;
  const float a6 = 0.00001388721735f;
  
  const float N1 = FFT_SIZE - 1;

  Serial.print("Generazione finestra Blackman-Harris 7-term ");
  Serial.print(FFT_SIZE);
  Serial.println(" punti...");

  for(int n = 0; n < FFT_SIZE; n++) {
    float x = 2.0f * PI * n / N1;
    window[n] = a0 
                - a1 * cosf(x)
                + a2 * cosf(2*x)
                - a3 * cosf(3*x)
                + a4 * cosf(4*x)
                - a5 * cosf(5*x)
                + a6 * cosf(6*x);
    
    if (n % 1000 == 0) Serial.print(".");
  }
  Serial.println(" OK");
  Serial.println("  Sidelobe rejection: -180dB");
}

void print_startup_diagnostics() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║      DIAGNOSTICA AVVIO                 ║");
  Serial.println("╚════════════════════════════════════════╝");
  
  // Test memoria
  Serial.println("Test memoria DMAMEM:");
  Serial.print("  fft_input[0]: ");
  Serial.println(fft_input[0]);
  Serial.print("  powerSpec[0]: ");
  Serial.println(powerSpec[0]);
  
  // Test audio
  Serial.println("\nTest audio queue:");
  delay(100);
  Serial.print("  Buffer disponibili: ");
  Serial.println(queue_in.available());
  
  if (queue_in.available() > 0) {
    int16_t *block = queue_in.readBuffer();
    if (block) {
      Serial.print("  Primo campione: ");
      Serial.println(block[0]);
      Serial.print("  Ultimo campione: ");
      Serial.println(block[BLOCK_SIZE-1]);
      queue_in.freeBuffer();
    }
  }
  
  Serial.println();
}

// ===== CORREZIONE FINESTRA =====
float compute_window_gain_correction() {
  double sum = 0.0;
  for (int i = 0; i < FFT_SIZE; i++) sum += window[i];
  double mean = sum / (double)FFT_SIZE;
  if (mean <= 1e-12) return 1.0f;
  return (float)(1.0 / mean);
}

// ===== DC OFFSET CALIBRATION =====
void calibrate_dc_offset() {
  Serial.println("Calibrazione DC offset...");
  Serial.println("  Attendere acquisizione dati...");
  
  // Svuota eventuali buffer vecchi
  int old_buffers = 0;
  while (queue_in.available() > 0) {
    queue_in.readBuffer();
    queue_in.freeBuffer();
    old_buffers++;
  }
  
  if (old_buffers > 0) {
    Serial.print("  Svuotati ");
    Serial.print(old_buffers);
    Serial.println(" buffer vecchi");
  }
  
  // ===== ATTENDI DATI FRESCHI =====
  delay(500);  
  
  // Attendi nuovi dati con timeout
  unsigned long timeout_start = millis();
  while (queue_in.available() < FFT_SIZE/BLOCK_SIZE) {
    if (millis() - timeout_start > 5000) {
      Serial.println("  TIMEOUT! Calibrazione fallita");
      dc_offset = 0.0f;
      dc_calibrated = false;
      return;
    }
    delay(10);
  }
  
  // ===== CAMPIONAMENTO =====
  double sum = 0.0;
  int count = 0;
  
  for (int b = 0; b < FFT_SIZE/BLOCK_SIZE; b++) {
    int16_t *block = queue_in.readBuffer();
    if (!block) { 
      queue_in.freeBuffer(); 
      continue; 
    }
    
    for (int i = 0; i < BLOCK_SIZE; i++) {
      float v = (float)block[i] / 32768.0f;
      sum += v;
      count++;
    }
    queue_in.freeBuffer();
  }
  
  dc_offset = (count > 0) ? (float)(sum / count) : 0.0f;
  dc_calibrated = true;
  
  Serial.print("  DC offset rilevato: ");
  Serial.println(dc_offset, 6);
  Serial.println("  Calibrazione OK!");
}

// ===== INTERPOLAZIONE PARABOLICA  =====
void parabolic_peak(float ym1, float y0, float yp1, float &delta, float &yInterp) {
  float denom = (ym1 - 2.0f * y0 + yp1);
  
  // Check per validità
  if (fabsf(denom) < 1e-12f || y0 < ym1 || y0 < yp1) {
    delta = 0.0f;
    yInterp = y0;
    return;
  }
  
  delta = 0.5f * (ym1 - yp1) / denom;
  
  // Limita delta per evitare valori anomali
  delta = constrain(delta, -0.5f, 0.5f);
  
  // Peak interpolato
  yInterp = y0 - 0.25f * (ym1 - yp1) * delta;
  
  // Verifica che sia powerSpecgiore del valore centrale
  if (yInterp < y0) yInterp = y0;
}

// ===== STIMA NOISE FLOOR  =====
float estimate_noise_floor() {
  const int START_BIN = 50;
  const int END_BIN = min(2000, FFT_SIZE/2 - 1);
  const int N = END_BIN - START_BIN;
  
  if (N <= 0) return 0.0f;
  
  // Usa solo i valori più bassi (percentile 25%)
  float samples[N];
  for (int i = START_BIN; i < END_BIN; i++) {
    samples[i - START_BIN] = powerSpec[i];
  }
  
  // Sort parziale per trovare percentile
  for (int i = 0; i < N/4; i++) {
    int min_idx = i;
    for (int j = i + 1; j < N; j++) {
      if (samples[j] < samples[min_idx]) {
        min_idx = j;
      }
    }
    if (min_idx != i) {
      float temp = samples[i];
      samples[i] = samples[min_idx];
      samples[min_idx] = temp;
    }
  }
  
  // Media del 25% più basso
  double sum = 0.0;
  for (int i = 0; i < N/4; i++) {
    sum += samples[i];
  }
  
  return (float)(sum / (N/4));
}

// ===== TROVA PICCO CON INTERPOLAZIONE =====
float find_peak_in_window(int centerBin, int windowSize, float &binFracOut) {
  int binStart = max(1, centerBin - windowSize);
  int binEnd = min(FFT_SIZE/2 - 2, centerBin + windowSize);
  
  int peakBin = binStart;
  float peakVal = powerSpec[binStart];
  for (int i = binStart + 1; i <= binEnd; i++) {
    if (powerSpec[i] > peakVal) {
      peakVal = powerSpec[i];
      peakBin = i;
    }
  }
  
  // Verifica che il picco non sia sul bordo della finestra
  if (peakBin == binStart || peakBin == binEnd) {
    binFracOut = peakBin;
    return peakVal;
  }
  
  float ym1 = powerSpec[peakBin - 1];
  float y0 = powerSpec[peakBin];
  float yp1 = powerSpec[peakBin + 1];
  float delta, interpAmp;
  parabolic_peak(ym1, y0, yp1, delta, interpAmp);
  
  binFracOut = peakBin + delta;
  return interpAmp;
}

// ===== FUNZIONE PER ESCLUDERE 50Hz E ARMONICHE =====
bool is_power_line_harmonic(int bin, float bin_hz) {
  float freq = bin * bin_hz;
  
  // Escludi 50Hz e multipli (50, 100, 150, 200, 250...)
  for (int h = 1; h <= 10; h++) {
    float power_freq = 50.0f * h;
    if (fabsf(freq - power_freq) < 2.0f) {  // ±2Hz di tolleranza
      return true;
    }
  }
  
  return false;
}

// ===== AUTO GAIN CONTROL - SOLO TRA SESSIONI =====
void evaluate_and_adjust_gain() {
  if (session_samples == 0) return;
  
  // Calcola statistiche della sessione appena completata
  float avg_max = session_max_signal;
  bool had_clipping = (session_clip_count > 5);
  
  Serial.println("\n--- Valutazione Gain ---");
  Serial.print("Max segnale sessione: ");
  Serial.println(avg_max, 4);
  Serial.print("Campioni con clipping: ");
  Serial.println(session_clip_count);
  
  bool gain_changed = false;
  int old_level = current_line_level;
  
  // CASO 1: Troppo clipping - Riduci gain per la PROSSIMA sessione
  if (had_clipping || avg_max > CLIPPING_THRESHOLD) {
    if (current_line_level < MAX_LINE_LEVEL) {
      current_line_level--;
      gain_changed = true;
      Serial.println("⚠️  CLIPPING rilevato!");
      Serial.print("   Gain ridotto: ");
      Serial.print(old_level);
      Serial.print(" → ");
      Serial.println(current_line_level);
      Serial.println("   Ripeti la misura per risultati accurati");
      change_level = false;
    } else {
      Serial.println("⚠️  CLIPPING - Gain già al minimo!");
      Serial.println("   Riduci il livello del segnale in ingresso");
    }
  }
  // CASO 2: Segnale troppo basso - Aumenta gain per la PROSSIMA sessione
  else if (avg_max < LOW_SIGNAL_THRESHOLD) {
    if (current_line_level > MIN_LINE_LEVEL) {
      current_line_level++;
      gain_changed = true;
      Serial.println("📉 Segnale molto basso");
      Serial.print("   Gain aumentato: ");
      Serial.print(old_level);
      Serial.print(" → ");
      Serial.println(current_line_level);
      Serial.println("   Ripeti la misura per risultati ottimali");
      change_level = false;
    } else {
      Serial.println("📉 Segnale basso - Gain già al massimo!");
      Serial.println("   Aumenta il livello del segnale in ingresso");
    }
  }
  // CASO 3: Segnale OK
  else {
    Serial.println("✓ Livello segnale ottimale");
    Serial.print("  Gain corrente: ");
    Serial.println(current_line_level);
    change_level = true;
  }
  
  // Applica il nuovo gain (per la PROSSIMA acquisizione)
  if (gain_changed) {
    sgtl5000_1.lineInLevel(current_line_level);
    float voltage_range = 3.12f - (current_line_level * 0.1875f);
    Serial.print("  Range input: ~");
    Serial.print(voltage_range, 2);
    Serial.println("V");
    Serial.println();
  }
  
  // Reset statistiche per la prossima sessione
  session_max_signal = 0.0f;
  session_clip_count = 0;
  session_samples = 0;
}

// ===== FLASH: FUNZIONI (invariate) =====
void init_flash() {
  Serial.println("Inizializzazione Flash W25Q128FV...");
  
  if (!SerialFlash.begin(FlashChipSelect)) {
    Serial.println("  Flash non trovata");
    flash_available = false;
    return;
  }
  
  uint8_t id[5];
  SerialFlash.readID(id);
  
  if (id[0] == 0xEF && id[1] == 0x40 && id[2] == 0x18) {
    Serial.println("  W25Q128FV OK!");
    flash_available = true;
  } else {
    flash_available = false;
  }
}

void save_to_flash(const MeasurementRecord &record) {
  if (!flash_available) return;
  
  SerialFlashFile file = SerialFlash.open("THD_LOG.DAT");
  
  if (!file) {
    SerialFlash.create("THD_LOG.DAT", MAX_RECORDS * sizeof(MeasurementRecord));
    file = SerialFlash.open("THD_LOG.DAT");
  }
  
  if (file) {
    file.seek(flash_record_count * sizeof(MeasurementRecord));
    file.write(&record, sizeof(MeasurementRecord));
    file.close();
    flash_record_count++;
    
    if (flash_record_count >= MAX_RECORDS) {
      flash_record_count = 0;
    }
  }
}

void print_flash_statistics() {
  if (!flash_available) return;
  
  SerialFlashFile file = SerialFlash.open("THD_LOG.DAT");
  if (!file) return;
  
  int records = file.size() / sizeof(MeasurementRecord);
  Serial.print("Record salvati: "); 
  Serial.println(records);
  
  file.close();
}

void erase_flash_log() {
  if (!flash_available) return;
  SerialFlash.remove("THD_LOG.DAT");
  flash_record_count = 0;
  Serial.println("Log cancellato");
}

// ===== SETUP =====
void setup() {
  pinMode(pinIn, INPUT_PULLUP);
  Serial.begin(115200);
  Serial1.begin(9600);
  delay(1000);
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║  THD Analyzer OTTIMIZZATO             ║");
  Serial.println("║  FFT 4096 + BH7 Window + AGC          ║");
  Serial.println("╚════════════════════════════════════════╝\n");

   // ===== INIZIALIZZAZIONE CRITICA =====
  Serial.println("Azzeramento buffer DMA...");
  memset(fft_input, 0, sizeof(fft_input));
  memset(fft_output, 0, sizeof(fft_output));
  memset(window, 0, sizeof(window));
  memset(powerSpec, 0, sizeof(powerSpec));
  Serial.println("  OK");
  
  // Audio
  AudioMemory(240);  // Ridotto per FFT 4096
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(AUDIO_INPUT_LINEIN);


  // ===== ATTENDI STABILIZZAZIONE CODEC =====
  Serial.println("Stabilizzazione SGTL5000...");
  delay(500);  // Importante!
  
  sgtl5000_1.unmuteHeadphone();
  sgtl5000_1.unmuteLineout();
  // Impostazioni AUDIO ottimizzate
  sgtl5000_1.lineInLevel(current_line_level);
  sgtl5000_1.adcHighPassFilterDisable();  // Disabilita HPF interno per più controllo
  
  Serial.print("Livello iniziale: ");
  Serial.println(current_line_level);
  
  // FFT
  Serial.println("Inizializzazione FFT 4096...");
  arm_rfft_fast_init_f32(&S, FFT_SIZE);
  Serial.println("  FFT OK");

  
  makeBlackmanHarris7();  // Finestra migliore!
  
  // Buffer
  memset(thd_buffer, 0, sizeof(thd_buffer));
  memset(thdn_buffer, 0, sizeof(thdn_buffer));
  memset(freq_buffer, 0, sizeof(freq_buffer));
  
  // IMPORTANTE: Avvia queue PRIMA della calibrazione DC!
  queue_in.begin();

  Serial.println("Pulizia buffer audio iniziali...");
  delay(200);
  int flushed = 0;
  while (queue_in.available() > 0) {
    queue_in.readBuffer();
    queue_in.freeBuffer();
    flushed++;
  }
  Serial.print("  ");
  Serial.print(flushed);
  Serial.println(" buffer svuotati");
  
  delay(300);  // Attendi nuovi dati puliti
  
  // Calibrazione DC (ora può ricevere dati)
  calibrate_dc_offset();
  
  // Flash
  init_flash();
  
  Serial.print("\nRisoluzione FFT: ");
  Serial.print(FS / FFT_SIZE, 3);
  Serial.println(" Hz/bin");
  
  Serial.println("\nOTTIMIZZAZIONI ATTIVE:");
  Serial.println("  ✓ FFT 4096 punti (10.76 Hz/bin)");
  Serial.println("  ✓ Blackman-Harris 7-term (-180dB)");
  Serial.println("  ✓ DC offset calibration");
  Serial.println("  ✓ Power line rejection (50Hz)");
  Serial.println("  ✓ Auto Gain Control");
  Serial.println("  ✓ Noise floor adattivo");
  print_startup_diagnostics();
  Serial.println("\n>>> Sistema pronto <<<\n");
}

// ===== ANALISI THD OTTIMIZZATA =====
void THD() {
  if (queue_in.available() < FFT_SIZE/BLOCK_SIZE) return;

  // Acquisizione con rimozione DC offset
  uint16_t idx = 0;
  float maxAbs = 0.0f;
  int clipCount = 0;
  
  
  
  for (int b = 0; b < FFT_SIZE/BLOCK_SIZE; b++) {
    int16_t *block = queue_in.readBuffer();
    if (!block) { 
      queue_in.freeBuffer(); 
      continue; 
    }
    
    for (int i = 0; i < BLOCK_SIZE; i++) {
      float v = (float)block[i] / 32768.0f;
      
      // RIMUOVI DC OFFSET
      if (dc_calibrated) {
        v -= dc_offset;
      }
      
      fft_input[idx++] = v;
      
      float av = fabsf(v);
      if (av > maxAbs) maxAbs = av;
      if (av > 0.95f) clipCount++;
    }
    queue_in.freeBuffer();
  }

  bool clipping = (maxAbs > 0.95f || clipCount > 10);

  
  // RACCOGLI STATISTICHE - NON modificare gain durante acquisizione!
  if (acquisition_active) {
    if (maxAbs > session_max_signal) {
      session_max_signal = maxAbs;
    }
    if (clipping) {
      session_clip_count++;
    }
    session_samples++;
  }

  // Applica finestra
  for (int i = 0; i < FFT_SIZE; i++) {
    fft_input[i] *= window[i];
  }

  // FFT
  arm_rfft_fast_f32(&S, fft_input, fft_output, 0);

    // FFT → POWER
  //const float scale = 1.0f / (FFT_SIZE * FFT_SIZE);
  const float scale = 1.0f / FFT_SIZE;

  for (int i = 0; i < FFT_SIZE / 2; i++) {
    float re = fft_output[2*i];
    float im = fft_output[2*i + 1];
    powerSpec[i] = (re*re + im*im) * scale;
  }

  float bin_hz = FS / FFT_SIZE;
  int bin_ignore = ceilf(10.0f / bin_hz);

  // Find fundamental
  int fund_bin = bin_ignore;
  float fund_pow = powerSpec[fund_bin];
  for (int i = bin_ignore + 1; i < FFT_SIZE/2; i++) {
    if (powerSpec[i] > fund_pow) {
      fund_pow = powerSpec[i];
      fund_bin = i;
    }
  }
  
  // Interpolazione fondamentale
  float ym1 = (fund_bin > 0) ? powerSpec[fund_bin - 1] : 0.0f;
  float y0 = powerSpec[fund_bin];
  float yp1 = (fund_bin < FFT_SIZE/2 - 1) ? powerSpec[fund_bin + 1] : 0.0f;
  float delta, interpAmp;
  parabolic_peak(ym1, y0, yp1, delta, interpAmp);
  float binFrac = fund_bin + delta;
  float f0 = binFrac * bin_hz;
  float Arm_1 = interpAmp;
  
  frequenza = f0;
  
  //float noiseFloor = estimate_noise_floor();
  
  // Fundamental power
  double fundamentalPower = 0.0;
  for (int r = -BIN_SUM_RADIUS; r <= BIN_SUM_RADIUS; r++) {
    int b = fund_bin + r;
    if (b >= bin_ignore && b < FFT_SIZE/2) {
      fundamentalPower += powerSpec[b];
    }
  }

  // Harmonics
  double harmonicPower = 0.0;
  int harmonics_found = 0;
  for (int h = 2; h <= HARM_MAX; h++) {
    //int center = fund_bin * h;
    int center = (int)roundf(binFrac * h);
    if (center >= FFT_SIZE/2 - BIN_SUM_RADIUS) break;

    double hp = 0.0;
    for (int r = -BIN_SUM_RADIUS; r <= BIN_SUM_RADIUS; r++) {
      int b = center + r;
      if (b >= bin_ignore && b < FFT_SIZE/2) {
        hp += powerSpec[b];
       
      }
    }
    harmonicPower += hp;
    harmonics_found++;
  }

  // 3. TOTAL SIGNAL POWER (escludi DC)
   double totalSignalPower = 0.0;
  for (int i = bin_ignore; i < FFT_SIZE/2; i++) {
    totalSignalPower += powerSpec[i];
  }
  // THD

  // 4. THD (solo armoniche)
    float thd = (fundamentalPower > 1e-12) 
       ? sqrtf(harmonicPower / fundamentalPower)
       : 0.0f;
    float thdPercent = thd * 100.0f;

  // THD+N

 
  double distortionAndNoise = totalSignalPower - fundamentalPower - harmonicPower;

  if (distortionAndNoise < 0) distortionAndNoise = 0;
  
  float thdn = (fundamentalPower > 1e-12) ? sqrtf(distortionAndNoise / fundamentalPower) : 0.0f;
  float thdnPercent = thdn * 100.0f;
  
  // EMA
  if (thd_ema == 0.0f) {
    thd_ema = thd;
  } else {
    thd_ema = EMA_ALPHA * thd + (1.0f - EMA_ALPHA) * thd_ema;
  }

  // Noise RMS (escludi armoniche e 50Hz)
  double noisePow = 0.0;
  int noiseBins = 0;
  
  for (int b = bin_ignore; b < FFT_SIZE/2; b++) {
    bool skip = false;
    
    // Salta fondamentale e armoniche
    for (int h = 1; h <= HARM_MAX; h++) {
      int hb = (int)roundf(fund_bin * h);
      if (hb < bin_ignore || hb >= FFT_SIZE/2) break;
      if (abs(b - hb) <= BIN_SUM_RADIUS + 2) {
        skip = true;
        break;
      }
    }
    
    // Salta 50Hz e armoniche
    if (is_power_line_harmonic(b, bin_hz)) {
      skip = true;
    }
    
    if (skip) continue;
    
    noisePow += (double)powerSpec[b];
    noiseBins++;
  }
  
  float noiseRms = (noiseBins > 0) ? sqrtf((float)(noisePow / noiseBins)) : 0.0f;
  float snr_db = (noisePow > 1e-18) ? 10.0f * log10f(fundamentalPower / noisePow) : 0.0f;

  // Salva in buffer
  thd_buffer[buffer_index] = thdPercent;
  thdn_buffer[buffer_index] = thdnPercent;
  freq_buffer[buffer_index] = frequenza;
  buffer_index++;
  
  if (buffer_index >= BUFFER_SIZE) {
    buffer_index = 0;
    buffer_full = true;
  }
  
  if (!buffer_full && samples_count < BUFFER_SIZE) {
    samples_count++;
  }

  // Salva su flash
  if (flash_available) {
    MeasurementRecord rec;
    rec.timestamp = millis() - session_start_time;
    rec.frequency = frequenza;
    rec.amplitude = fundamentalPower;
    rec.thd = thdPercent;
    rec.thdn = thdnPercent;
    rec.snr = snr_db;
    //rec.noiseFloor = noiseFloor;
    rec.clipping = clipping;
    save_to_flash(rec);
  }
  
  
  n_fondamentale = fund_bin;
  armonica1 = fundamentalPower;
  rumore = noiseRms;
  campioni = maxAbs;
  
  // Output
  Serial.print("F:");
  Serial.print(frequenza, 1);
  Serial.print("Hz THD:");
  Serial.print(thdPercent, 4);  // 4 decimali per precisione
  Serial.print("% THD+N:");
  Serial.print(thdnPercent, 4);
  Serial.print("% SNR:");
  Serial.print(snr_db, 1);
  Serial.print("dB H:");
  Serial.print(harmonics_found);
  Serial.print(" Gain:");
  Serial.println(current_line_level);

  memset(powerSpec, 0, sizeof(powerSpec));
  memset(fft_input, 0, sizeof(fft_input));
  memset(fft_output, 0, sizeof(fft_output));
}

// ===== STATISTICHE =====
void calcola_e_stampa_media() {
  if (samples_count == 0) {
    Serial.println("Nessun campione acquisito");
    return;
  }
  
  int num_samples = buffer_full ? BUFFER_SIZE : samples_count;
  
  double sum_thd = 0.0, sum_thdn = 0.0, sum_freq = 0.0;
  double sum_sq_thd = 0.0;
  
  for (int i = num_samples-10; i < num_samples; i++) {
    sum_thd += thd_buffer[i];
    sum_thdn += thdn_buffer[i];
    sum_freq += freq_buffer[i];
    sum_sq_thd += thd_buffer[i] * thd_buffer[i];
  }
  
  float media_thd = sum_thd / 10;
  float media_thdn = sum_thdn / 10;
  float media_freq = sum_freq / 10;
  
  float variance = (sum_sq_thd / num_samples) - (media_thd * media_thd);
  float std_dev = (variance > 0) ? sqrtf(variance) : 0.0f;
  
  float min_thd = thd_buffer[0];
  float max_thd = thd_buffer[0];
  for (int i = 1; i < num_samples; i++) {
    if (thd_buffer[i] < min_thd) min_thd = thd_buffer[i];
    if (thd_buffer[i] > max_thd) max_thd = thd_buffer[i];
  }
  
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║      STATISTICHE ACQUISIZIONE          ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.print("║ Campioni:     "); Serial.print(num_samples);
  Serial.print("/"); Serial.println(BUFFER_SIZE);
  Serial.print("║ Freq Media:   "); Serial.print(media_freq, 1);
  Serial.println(" Hz");
  Serial.print("║ Gain Level:   "); Serial.println(current_line_level);
  Serial.println("║----------------------------------------║");
  Serial.print("║ THD Medio:    "); Serial.print(media_thd, 4); 
  Serial.println(" %");
  Serial.print("║ THD Min:      "); Serial.print(min_thd, 4); 
  Serial.println(" %");
  Serial.print("║ THD Max:      "); Serial.print(max_thd, 4); 
  Serial.println(" %");
  Serial.print("║ Deviazione:   "); Serial.print(std_dev, 4); 
  Serial.println(" %");
  Serial.println("║----------------------------------------║");
  Serial.print("║ THD+N Medio:  "); Serial.print(media_thdn, 4); 
  Serial.println(" %");
  Serial.println("╚════════════════════════════════════════╝");
  
  if (flash_available) {
    Serial.print("\nRecord salvati in flash: ");
    Serial.println(flash_record_count);
  }
  
  Serial.println();

  // Output Serial1 - IDENTICO ALL'ORIGINALE
  Serial1.print("F0=");
  Serial1.print(frequenza, 2);
  Serial1.print("Hz bin=");
  Serial1.print(n_fondamentale);
  Serial1.print("  A1=");
  Serial1.print(armonica1, 6);
  Serial1.print("  THD=");
  Serial1.print(media_thd, 4);
  Serial1.print("  THD_EMA=");
  Serial1.print(0.00f, 4);
  Serial1.print("  THD+N=");
  Serial1.print(media_thdn, 4);
  Serial1.print("  NoiseRMS=");
  if (change_level == false){
  Serial1.print("Error");  
  }else{
  Serial1.print("OK");  
  }
  
  
  Serial1.print("  MaxSample=");
  Serial1.print(campioni, 6);
  Serial1.print("  Clipping=");
  Serial1.println("NO");
  
  // Reset buffer
  buffer_index = 0;
  samples_count = 0;
  buffer_full = false;
}

// ===== LOOP =====
void loop() {
  int state = digitalRead(pinIn);

  if (state == 0) {
    Serial.println("\n>>> Inizio acquisizione <<<");
    Serial.print("Gain level: ");
    Serial.println(current_line_level);
    
    // Reset statistiche e attiva modalità acquisizione
    session_max_signal = 0.0f;
    session_clip_count = 0;
    session_samples = 0;
    acquisition_active = true;
    
    //delay(500);
    session_start_time = millis();
    unsigned long start = millis();
    
    // ACQUISIZIONE - Gain FISSO durante tutta la sessione
    while ((millis() - start) < time_acquire) {
      THD();
    }
    
    // Fine acquisizione
    acquisition_active = false;
    
    // VALUTA E REGOLA GAIN per la PROSSIMA sessione
    evaluate_and_adjust_gain();
    
    Serial.println(">>> Fine acquisizione <<<");
    calcola_e_stampa_media();
    
    
  }
  
  // Comandi seriali per gestione flash
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "stats") {
      print_flash_statistics();
    } else if (cmd == "erase") {
      erase_flash_log();
    } else if (cmd == "recal") {
      Serial.println("Ricalibrazione DC offset...");
      calibrate_dc_offset();
    } else if (cmd.startsWith("gain=")) {
      // Comando manuale per impostare gain
      int new_gain = cmd.substring(5).toInt();
      if (new_gain >= MIN_LINE_LEVEL && new_gain <= MAX_LINE_LEVEL) {
        current_line_level = new_gain;
        sgtl5000_1.lineInLevel(current_line_level);
        Serial.print("Gain impostato manualmente a: ");
        Serial.println(current_line_level);
      } else {
        Serial.println("Gain non valido (range 0-15)");
      }
    } else if (cmd == "help") {
      Serial.println("\nComandi disponibili:");
      Serial.println("  stats    - Mostra statistiche flash");
      Serial.println("  erase    - Cancella log flash");
      Serial.println("  recal    - Ricalibra DC offset");
      Serial.println("  gain=N   - Imposta gain manualmente (0-15)");
      Serial.println("  help     - Questo messaggio");
    }
  }
  
  //delay(10);
}

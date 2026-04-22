#include <Arduino.h>
#include <arduinoFFT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <RadioLib.h>
#include <algorithm>

#define LORA_NSS 8
#define LORA_RESET 12
#define LORA_BUSY 13
#define LORA_DIO1 14
#define WINDOW_SIZE 128
#define BASE_SAMPLING_RATE_MS 10

const bool ENABLE_LORAWAN = false;
const bool PRINT_UNFILTERED_BONUS = true;

const char* ssid = "Pixel_3478";
const char* password = "sounciocco";
const char* mqtt_server = "10.204.104.193";
const char* topic_average = "iot/edge/average";

uint64_t joinEUI = 0x0000000000000000;
uint64_t devEUI = 0x70B3D57ED00770E1;
uint8_t appKey[] = { 0x6A, 0x7D, 0xF6, 0xED, 0x89, 0x13, 0x48, 0xDA, 0x30, 0x29, 0x16, 0xDE, 0x73, 0xFC, 0x82, 0xE3 };

TaskHandle_t TaskGenerator, TaskProcessor, TaskComm;
QueueHandle_t dataQueue;
WiFiClient espClient;
PubSubClient client(espClient);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RESET, LORA_BUSY);
LoRaWANNode node(&radio, &EU868);

struct SignalData {
    float value;
    bool isAnomaly;
    uint32_t timestamp;
};


volatile int globalScenario = 0;
volatile float currentSamplingDelay = BASE_SAMPLING_RATE_MS;
volatile float avgNormal = 0.0;
volatile float avgZ = 0.0;
volatile float avgH = 0.0;

volatile uint32_t executionTime = 0;
volatile uint32_t endToEndLatency = 0;
volatile double dominantFreq = 0.0;
volatile uint32_t hardwareMaxSamplingFreq = 0;
volatile float TPR_Z = 0.0, FPR_Z = 0.0;
volatile uint32_t execTime_Z = 0;
volatile float TPR_H = 0.0, FPR_H = 0.0;
volatile uint32_t execTime_H = 0;


volatile double dominantFreqUnfiltered = 0.0;
volatile float adaptiveFreqUnfiltered = 0.0;
volatile double dominantFreqZ = 0.0;
volatile float adaptiveFreqZ = 0.0;

float generateGaussianNoise(float mu, float sigma) {
    float u1 = random(10000) / 10000.0;
    float u2 = random(10000) / 10000.0;
    if(u1 == 0) u1 = 0.0001;
    float z0 = sqrt(-2.0 * log(u1)) * cos(TWO_PI * u2);
    return z0 * sigma + mu;
}

void SignalGeneratorTask(void *pvParameters) {
    SignalData sample;
    while(1) {
        float t = micros() / 1000000.0;
        sample.timestamp = millis();
        sample.isAnomaly = false;
        float cleanSignal = 0;
        if (globalScenario == 0) {
            cleanSignal = 5 * sin(TWO_PI * 0.5 * t);
        } 
        else if (globalScenario == 1) {
            cleanSignal = 3 * sin(TWO_PI * 20 * t);
        } 
        else if (globalScenario == 2) {
            cleanSignal = 2 * sin(TWO_PI * 3 * t) + 4 * sin(TWO_PI * 5 * t);
        }
        else if (globalScenario == 3) {
            cleanSignal = 2 * sin(TWO_PI * 3 * t) + 4 * sin(TWO_PI * 5 * t);
        }
        if (globalScenario == 3) {
            float noise = generateGaussianNoise(0, 0.2);
            float anomaly = 0;
            if (random(1000) < 100) {
                sample.isAnomaly = true;
                anomaly = random(500, 1500) / 100.0;
                if (random(2) == 0) anomaly = -anomaly;
            }
            sample.value = cleanSignal + noise + anomaly;
        } else {
            sample.value = cleanSignal;
        }
        xQueueSend(dataQueue, &sample, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(currentSamplingDelay));
    }
}

void DataProcessorTask(void *pvParameters) {
    SignalData buffer[WINDOW_SIZE];
    double vReal[WINDOW_SIZE];
    double vImag[WINDOW_SIZE];
    ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, WINDOW_SIZE, 1000.0/currentSamplingDelay);
    
    while(1) {
        int totalAnomalies = 0;
        int normalSamples = 0;
        for(int i = 0; i < WINDOW_SIZE; i++) {
            xQueueReceive(dataQueue, &buffer[i], portMAX_DELAY);
            vImag[i] = 0.0;
            if(buffer[i].isAnomaly) totalAnomalies++;
        }
        normalSamples = WINDOW_SIZE - totalAnomalies;
        uint32_t tStart = micros();
        float mean = 0;
        for(int i=0; i<WINDOW_SIZE; i++) mean += buffer[i].value;
        mean /= WINDOW_SIZE;
        avgNormal = mean; 

        if (globalScenario == 3) {
            // FFT
            if (PRINT_UNFILTERED_BONUS) {
                double vRealRaw[WINDOW_SIZE];
                double vImagRaw[WINDOW_SIZE];
                for(int i = 0; i < WINDOW_SIZE; i++) {
                    vRealRaw[i] = buffer[i].value;
                    vImagRaw[i] = 0.0;
                }
                ArduinoFFT<double> FFTRaw = ArduinoFFT<double>(vRealRaw, vImagRaw, WINDOW_SIZE, 1000.0/currentSamplingDelay);
                FFTRaw.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
                FFTRaw.compute(FFT_FORWARD);
                FFTRaw.complexToMagnitude();
                dominantFreqUnfiltered = FFTRaw.majorPeak();
                adaptiveFreqUnfiltered = dominantFreqUnfiltered * 2.0;
                
                // Limite applicato solo allo scenario 0, altrimenti sicurezza a 1.0 Hz
                if (globalScenario == 0) {
                    if (adaptiveFreqUnfiltered < 10.0) adaptiveFreqUnfiltered = 10.0;
                } else {
                    if (adaptiveFreqUnfiltered < 1.0) adaptiveFreqUnfiltered = 1.0;
                }
            }
            // -----------------------------------------------------------

            // Zscore & FFT
            uint32_t t0 = micros();
            float stddev = 0;
            for(int i=0; i<WINDOW_SIZE; i++) stddev += pow(buffer[i].value - mean, 2);
            stddev = sqrt(stddev / WINDOW_SIZE);
            int tp_Z = 0, fp_Z = 0;
            float sum_Z = 0;
            
            double vRealZ[WINDOW_SIZE]; 
            double vImagZ[WINDOW_SIZE];

            for(int i=0; i<WINDOW_SIZE; i++) {
                bool flagged = (abs(buffer[i].value - mean) / stddev > 3.0);
                if (flagged && buffer[i].isAnomaly) tp_Z++;
                if (flagged && !buffer[i].isAnomaly) fp_Z++;
                
                if(flagged) {
                    sum_Z += mean;
                    vRealZ[i] = mean; 
                } else {
                    sum_Z += buffer[i].value;
                    vRealZ[i] = buffer[i].value;
                }
                vImagZ[i] = 0.0;
            }
            execTime_Z = micros() - t0;
            TPR_Z = (totalAnomalies > 0) ? ((float)tp_Z / totalAnomalies) * 100.0 : 100.0;
            FPR_Z = (normalSamples > 0) ? ((float)fp_Z / normalSamples) * 100.0 : 0.0;
            avgZ = sum_Z / WINDOW_SIZE;

            // FFT
            if (PRINT_UNFILTERED_BONUS) {
                ArduinoFFT<double> FFTZ = ArduinoFFT<double>(vRealZ, vImagZ, WINDOW_SIZE, 1000.0/currentSamplingDelay);
                FFTZ.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
                FFTZ.compute(FFT_FORWARD);
                FFTZ.complexToMagnitude();
                dominantFreqZ = FFTZ.majorPeak();
                adaptiveFreqZ = dominantFreqZ * 2.0;
                
                // Limite applicato solo allo scenario 0, altrimenti sicurezza a 1.0 Hz
                if (globalScenario == 0) {
                    if (adaptiveFreqZ < 10.0) adaptiveFreqZ = 10.0;
                } else {
                    if (adaptiveFreqZ < 1.0) adaptiveFreqZ = 1.0;
                }
            }

            // Hampel
            uint32_t t1 = micros();
            float temp[WINDOW_SIZE];
            for(int i=0; i<WINDOW_SIZE; i++) temp[i] = buffer[i].value;
            std::sort(temp, temp + WINDOW_SIZE);
            float median = (temp[WINDOW_SIZE/2 - 1] + temp[WINDOW_SIZE/2]) / 2.0;
            float devs[WINDOW_SIZE];
            for(int i=0; i<WINDOW_SIZE; i++) devs[i] = abs(buffer[i].value - median);
            std::sort(devs, devs + WINDOW_SIZE);
            float mad = (devs[WINDOW_SIZE/2 - 1] + devs[WINDOW_SIZE/2]) / 2.0;
            float sigma_hampel = 1.4826 * mad; 
            int tp_H = 0, fp_H = 0;
            float sum_H = 0;
            
            for(int i=0; i<WINDOW_SIZE; i++) {
                bool flagged = (abs(buffer[i].value - median) > 3.0 * sigma_hampel);
                if (flagged && buffer[i].isAnomaly) tp_H++;
                if (flagged && !buffer[i].isAnomaly) fp_H++;
                if(flagged) {
                    vReal[i] = median; 
                } else {
                    vReal[i] = buffer[i].value;
                }
                sum_H += vReal[i];
            }
            execTime_H = micros() - t1;
            TPR_H = (totalAnomalies > 0) ? ((float)tp_H / totalAnomalies) * 100.0 : 100.0;
            FPR_H = (normalSamples > 0) ? ((float)fp_H / normalSamples) * 100.0 : 0.0;
            avgH = sum_H / WINDOW_SIZE;
        } else {
            for(int i = 0; i < WINDOW_SIZE; i++) {
                vReal[i] = buffer[i].value;
            }
        }

        // FFT
        FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(FFT_FORWARD);
        FFT.complexToMagnitude();
        dominantFreq = FFT.majorPeak();

        float newFs = (dominantFreq * 2.0);
        
        if (globalScenario == 0) {
            if (newFs < 10.0) newFs = 10.0;
        } else {
            if (newFs < 1.0) newFs = 1.0; 
        }
        
        currentSamplingDelay = 1000.0 / newFs;

        executionTime = micros() - tStart; 
        endToEndLatency = millis() - buffer[0].timestamp;
    }
}

void CommTask(void *pvParameters) {
    bool isJoined = false; 
    int tickCounter = 0;

    if (ENABLE_LORAWAN) {
        if (Serial) Serial.println("\n[LoRaWAN] Initializing radio module...");
        int state = radio.begin();
        if (state != RADIOLIB_ERR_NONE) {
            if (Serial) Serial.printf("[LoRaWAN] Radio init error: %d\n", state);
        } else {
            if (Serial) Serial.println("[LoRaWAN] Configuring TTN keys...");
            radio.setTCXO(1.8);
            radio.setDio2AsRfSwitch(true); 
            node.beginOTAA(joinEUI, devEUI, appKey, appKey);
            
            while (!node.isActivated()) {
                if (Serial) Serial.println("Sending Join Request to TTN...");
                int16_t joinState = node.activateOTAA();
                if (node.isActivated()) {
                    if (Serial) Serial.println("Successfully connected to TTN!");
                    isJoined = true; 
                } else {
                    if (Serial) Serial.printf("TTN Join FAILED! Error code: %d. Retrying in 10s...\n", joinState);
                    vTaskDelay(pdMS_TO_TICKS(10000));
                }
            }
        }
    } else {
        if (Serial) Serial.println("\nModule LORA for local testing.");
    }

    if (Serial) Serial.printf("\n[Wi-Fi] Connecting to network %s...\n", ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(500));
        if (Serial) Serial.print(".");
    }
    if (Serial) {
        Serial.println("\nWi-Fi Successfully connected!");
        Serial.print("Wi-Fi Assigned IP address: ");
        Serial.println(WiFi.localIP());
    }
    client.setServer(mqtt_server, 1883);

    while(1) {
        while (!client.connected()) {
            if (Serial) Serial.print("MQTT Attempting connection...");
            String clientId = "ESP32S3-Client-" + String(random(0xffff), HEX);
            if (client.connect(clientId.c_str())) {
                if (Serial) Serial.println(" connected to Mosquitto");
            } else {
                if (Serial) {
                    Serial.print(" failed, rc=");
                    Serial.print(client.state());
                    Serial.println(" try again in 5 seconds");
                }
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
        }
        client.loop();
        vTaskDelay(pdMS_TO_TICKS(5000));
        tickCounter++; 
        if (tickCounter == 1 && Serial) {
            if (globalScenario == 0) Serial.println("Signal 0 Slow");
            else if (globalScenario == 1) Serial.println("Signal 1 Fast");
            else if (globalScenario == 2) Serial.println("Signal 2 Complex");
            else if (globalScenario == 3) Serial.println("Bonus");
        }
        if (Serial) Serial.println("\nSYSTEM PERFORMANCE REPORT");
        if (globalScenario != 3) {
            char msg[50];
            snprintf(msg, sizeof(msg), "Avg:%.2f", avgNormal);
            bool mqttSuccess = client.publish(topic_average, msg);
            if (Serial) Serial.printf("> MQTT Transmission         : %s (%s)\n", msg, mqttSuccess ? "SENT" : "FAILED");
            int loraState = RADIOLIB_ERR_NONE;
            if (tickCounter == 6) {
                if (ENABLE_LORAWAN) {
                    if (isJoined) {
                        if (Serial) Serial.println("> LoRaWAN Status            : Attempting scheduled uplink...");
                        loraState = node.sendReceive((uint8_t*)msg, strlen(msg));
                        if (loraState == RADIOLIB_ERR_NONE || loraState == -1116) {
                            Serial.printf("> LoRaWAN Transmission      : %s (SENT TO TTN)\n", msg);
                        } else {
                            Serial.printf("> LoRaWAN Transmission      : ERROR (%d)\n", loraState);
                        }
                    }
                } else {
                    Serial.printf("> LoRaWAN Transmission      : SKIPPED (Local Test)\n");
                }
            } else {
                if (ENABLE_LORAWAN) {
                    Serial.printf("> LoRaWAN Uplink Scheduled  : In %d seconds\n", (6 - tickCounter) * 5);
                } else {
                    Serial.printf("> LoRaWAN Uplink Scheduled  : DISABLED\n");
                }
            }
            Serial.printf("> Saved Data Volume         : %d bytes\n", strlen(msg));
            Serial.printf("> Execution Time per window : %u us\n", executionTime);
            Serial.printf("> End-to-End Latency        : %u ms\n", endToEndLatency);
            Serial.printf("> FFT Dominant Frequency    : %.2f Hz\n", dominantFreq);
            Serial.printf("> Adaptive Sampling Freq    : %.1f Hz\n", (1000.0/currentSamplingDelay));
            Serial.printf("> Average Calculated    : %.2f\n", avgNormal);
        } 
        else {
            Serial.printf("> CPU Execution Time        : %u us\n", executionTime);
            Serial.printf("> End-to-End Latency        : %u ms\n", endToEndLatency);
            
            // STAMPA DELLE 3 FFT A CONFRONTO SE RICHIESTO
            if (PRINT_UNFILTERED_BONUS) {
                Serial.printf("> [RAW]   FFT Dominant Freq : %.2f Hz\n", dominantFreqUnfiltered);
                Serial.printf("> [RAW]   Adaptive Samp Freq: %.1f Hz\n", adaptiveFreqUnfiltered);
                Serial.printf("> [Z-SCR] FFT Dominant Freq : %.2f Hz\n", dominantFreqZ);
                Serial.printf("> [Z-SCR] Adaptive Samp Freq: %.1f Hz\n", adaptiveFreqZ);
            }
            
            // STAMPA DELLA FFT CON I DATI FILTRATI DA HAMPEL (SISTEMA NORMALE)
            Serial.printf("> [HMPL]  FFT Dominant Freq : %.2f Hz\n", dominantFreq);
            Serial.printf("> [HMPL]  Adaptive Samp Freq: %.1f Hz\n", (1000.0/currentSamplingDelay));
            
            Serial.printf("> CPU Time Z-Score          : %u us\n", execTime_Z);
            Serial.printf("> CPU Time Hampel           : %u us\n", execTime_H);
            Serial.printf("> Z-Score (TPR/FPR)         : %.1f %% / %.1f %%\n", TPR_Z, FPR_Z);
            Serial.printf("> Hampel (TPR/FPR)          : %.1f %% / %.1f %%\n", TPR_H, FPR_H);
            Serial.printf("> Final Average (Z-Score)   : %.2f\n", avgZ);
            Serial.printf("> Final Average (Hampel)    : %.2f\n", avgH);
        }
        Serial.println("-----------------------------------");
        if (tickCounter >= 6) {
            tickCounter = 0;
            globalScenario = (globalScenario + 1) % 4;
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH); 

    Serial.println("\n[SYSTEM] Calculating Maximum Hardware Sampling Frequency...");
    uint32_t sampleCount = 0;
    uint32_t startTime = micros();
    while (micros() - startTime < 1000000) {
        volatile int dummyValue = analogRead(1);
        sampleCount++;
    }
    hardwareMaxSamplingFreq = sampleCount;
    Serial.printf("[SYSTEM] Benchmark Complete! Max Freq: %u Hz\n", hardwareMaxSamplingFreq);
    
    dataQueue = xQueueCreate(WINDOW_SIZE * 2, sizeof(SignalData));
    xTaskCreatePinnedToCore(SignalGeneratorTask, "Generator", 4096, NULL, 3, &TaskGenerator, 1);
    xTaskCreatePinnedToCore(DataProcessorTask, "Processor", 10240, NULL, 2, &TaskProcessor, 1);
    xTaskCreatePinnedToCore(CommTask, "Comm", 8192, NULL, 1, &TaskComm, 0); 
}

void loop() {
    vTaskDelete(NULL);
}
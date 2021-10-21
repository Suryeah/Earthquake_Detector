#include "arduinoFFT.h"
#include "Protocentral_ADS1220.h"
#include <Integrator.h>
#include <TimeStep.h>
#include <filters.h>
#include <SPI.h>
#include <cppQueue.h>
#include <WiFi.h>
#include <FIR.h>
#include <stdio.h>
#include <stdlib.h>

Protocentral_ADS1220 pc_ads1220;
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
FIR<float, 16> fir_lp;

#define ADS1220_CS_PIN    5
#define ADS1220_DRDY_PIN  4
#define ALERT_PIN         2

double mean, thresh, threshN  = 0;

uint32_t POSTSMP = 2048;
const uint32_t offset = 4096;
const uint32_t samples = 4096;         // MUST ALWAYS be a power of 2
const double samplingFrequency = 1160; // Hz, must be less than 10000 due to ADC
double OFFSET[samples];
double vReal[samples];
double vImag[samples];
double x;
volatile double max_v = 0.0;

int32_t sensorValue;
double adc, fadc, Acc, hp, peak, Velo_z;
bool SET;
bool LED;
bool CONT = 0;
bool startup = 1;

TaskHandle_t Task1;
TaskHandle_t Task2;

const float cutoff_freq   = 1;        //Cutoff frequency in Hz
const float sampling_time = 0.000862;     //Sampling time in seconds.
Filter fhp(cutoff_freq, sampling_time, IIR::ORDER::OD3, IIR::TYPE::HIGHPASS);

cppQueue  qz(sizeof(adc), samples, LIFO, true);

const char* ssid = "Radopsys Lab";
const char* password =  "Radopsys@2019";
WiFiServer wifiServer(80);

const IPAddress local_IP(192, 168, 1, 120);
const IPAddress gateway(192, 168, 1, 1);
const IPAddress subnet(255, 255, 255, 0);

void mean_sampling()
{
  sensorValue = pc_ads1220.Read_WaitForData();
  if (sensorValue) {
    adc = (sensorValue);
    adc -= mean;
    qz.push(&(adc));
    Serial.println(adc, 4);
  }
}

void sampling()
{
  sensorValue = pc_ads1220.Read_WaitForData();
  if (sensorValue) {
    adc = (sensorValue);
    adc -= mean;
    hp = fhp.filterIn(adc);
    fadc = fir_lp.processReading(hp);
    Acc = (fadc * 0.000000536);
    qz.push(&(Acc));
    Serial.println(Acc, 5);
  }
}

void calib_mean()
{
  for (uint32_t i = 0; i < offset;)
  {
    mean_sampling();
    if (sensorValue > 0)
      i++;
  }
  for (uint32_t i = 0; i < offset; i++)
  {
    qz.pop(&Velo_z);
    mean += (Velo_z);
  }

  mean = mean / offset;

  Serial.print("mean: ");
  Serial.println(mean, 4);
  qz.flush();
}

void setup()
{
  pinMode(ALERT_PIN , OUTPUT);
  WiFi.config(local_IP, gateway, subnet);
  float coef_lp[16] = {1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.};
  fir_lp.setFilterCoeffs(coef_lp);

  pc_ads1220.begin(ADS1220_CS_PIN, ADS1220_DRDY_PIN);
  pc_ads1220.writeRegister(CONFIG_REG0_ADDRESS, 0x00);
  pc_ads1220.writeRegister(CONFIG_REG1_ADDRESS, 0xB4);
  pc_ads1220.writeRegister(CONFIG_REG2_ADDRESS, 0x50);
  pc_ads1220.writeRegister(CONFIG_REG3_ADDRESS, 0x00);
  pc_ads1220.select_mux_channels(MUX_SE_CH3);
  Serial.begin(115200);
  delay(100);
  calib_mean();


  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code,   /* Task function. */
    "Task1",     /* name of task. */
    15000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code,   /* Task function. */
    "Task2",     /* name of task. */
    15000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task2,      /* Task handle to keep track of created task */
    1);          /* pin task to core 1 */
  delay(500);

  disableCore0WDT();
  disableCore1WDT();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
  wifiServer.begin();
}

void Task1code( void * pvParameters ) {
  for (;;) {
    sampling();

    if (startup == 0 )
    {
      if ((Serial.read() == 'l') || (Acc > (thresh)) || (Acc < (threshN)))
      {
        peak = Acc;
        LED = 1;
        for (uint32_t j = 0; j < POSTSMP;)
        {
          sampling();
          if (sensorValue > 0)
            j++;
        }
        for (uint32_t i = 0; i < samples; i++)
        {
          qz.pop(&Velo_z);
          vReal[i] = Velo_z;
          vImag[i] = 0;
        }

        FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
        FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
        FFT.DCRemoval(vReal, samples);
        FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
        x = FFT.MajorPeak(vReal, samples, samplingFrequency);
        Serial.print("Frequency: ");
        Serial.println(x, 4); //Print out what frequency is the most dominant.
        max_v = 0;
        for (int i = 0; i < samples; i++)
        {
          if (vReal[i] > max_v)
            max_v = vReal[i];
        }
        max_v = (max_v / samplingFrequency);
        Serial.print("Velocity: ");
        Serial.println(max_v, 4);
        SET = 1;                      // Trigger to send data via TCP
      }
    }
  }
}

void Task2code( void * pvParameters ) {
  for (;;) {
    String line;
    WiFiClient client = wifiServer.available();
    if (client) {
      while (client.connected()) {
        if (startup)
        {
          client.print("Set Threshold value : ");
          while (!client.available()) {}
          line = client.readStringUntil('\r');
          thresh = line.toFloat();
          client.print("Threshold Point is at: ");
          client.println(thresh,4);
          threshN = (thresh*(-1));
          startup = 0 ;
        }
        if (LED)
        {
          client.println("Peak Detected ");
          LED = 0;
        }
        if (SET)
        {
          client.println(String(x, 4) + '\n' + "@" + String(max_v, 4) + '\n' + "@" + String(millis()) + " ms" + '\n');
          SET = 0;
        }
      }
      delay(10);
    }
    client.stop();
  }
}

void loop() {
  if (LED == 1) {
    digitalWrite(ALERT_PIN, HIGH);
    delay(500);
    digitalWrite(ALERT_PIN, LOW);
    LED = 0;
  }
}

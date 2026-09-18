#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <SparkFun_CY8CMBR3.h>
#include <Adafruit_SHT31.h>
#include <esp_sleep.h>

// ==================================================
// I2C設定
// ==================================================

constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;

constexpr uint8_t PCA9546_ADDRESS = 0x70;
constexpr uint8_t SHT31_ADDRESS = 0x44;
constexpr uint8_t SENSOR_COUNT = 4;

// ==================================================
// microSD設定
// ==================================================

constexpr int SD_CS   = 5;
constexpr int SD_MOSI = 23;
constexpr int SD_CLK  = 18;
constexpr int SD_MISO = 19;

// 動作実績のある400 kHz
constexpr uint32_t SD_FREQUENCY = 400000;

// ==================================================
// 測定設定
// ==================================================

// deep sleep時間：10分
constexpr uint64_t SLEEP_INTERVAL_US = 10ULL * 60ULL * 1000000ULL;

// シリアルモニター確認用の待機時間
constexpr unsigned long SERIAL_STARTUP_DELAY_MS = 10000;
constexpr unsigned long BEFORE_SLEEP_DELAY_MS = 5000;

// trueにすると起動ごとにRTCをコンパイル時刻へ設定する
// 通常はfalse。RTC未設定時は自動で一度だけ設定する。
constexpr bool SET_RTC_FROM_COMPILE_TIME = false;

const char *CSV_FILE = "/soil8_log.csv";
const char *EVENT_FILE = "/soil8_events.csv";

const int SENSOR_DEPTH_CM[SENSOR_COUNT] = {
  10,
  30,
  60,
  90
};

// ==================================================
// 機器オブジェクト
// ==================================================

SfeCY8CMBR3ArdI2C moistureSensors[SENSOR_COUNT];
Adafruit_SHT31 sht31Sensors[SENSOR_COUNT];
RTC_PCF8523 rtc;

bool sdCardAvailable = false;
bool eventLogAvailable = false;
bool pcaAvailable = false;
bool rtcAvailable = false;
bool moistureSensorAvailable[SENSOR_COUNT] = { false };
bool sht31SensorAvailable[SENSOR_COUNT] = { false };

// ==================================================
// PCA9546チャンネル選択
// ==================================================

bool selectPCAChannel(uint8_t channel)
{
  if (channel >= SENSOR_COUNT)
  {
    return false;
  }

  Wire.beginTransmission(PCA9546_ADDRESS);
  Wire.write(1 << channel);

  return Wire.endTransmission() == 0;
}

// ==================================================
// タイムスタンプ作成
// ==================================================

void formatTimestamp(char *timestamp, size_t timestampSize)
{
  if (!rtcAvailable)
  {
    snprintf(
      timestamp,
      timestampSize,
      "NO_RTC_%lu",
      millis() / 1000UL
    );

    return;
  }

  DateTime now = rtc.now();

  snprintf(
    timestamp,
    timestampSize,
    "%04d-%02d-%02d %02d:%02d:%02d",
    now.year(),
    now.month(),
    now.day(),
    now.hour(),
    now.minute(),
    now.second()
  );
}

// ==================================================
// イベントログ保存
// ==================================================

void logEvent(
  const char *event,
  const char *component,
  int channel,
  const char *message)
{
  char timestamp[24];
  formatTimestamp(timestamp, sizeof(timestamp));

  Serial.print("[EVENT] ");
  Serial.print(timestamp);
  Serial.print(" ");
  Serial.print(event);
  Serial.print(" ");
  Serial.print(component);
  Serial.print(" ch=");
  Serial.print(channel);
  Serial.print(" ");
  Serial.println(message);

  if (!eventLogAvailable)
  {
    return;
  }

  File file = SD.open(EVENT_FILE, FILE_APPEND);

  if (!file)
  {
    Serial.println("Failed to open soil8_events.csv.");
    eventLogAvailable = false;
    return;
  }

  file.print(timestamp);
  file.print(",");
  file.print(event);
  file.print(",");
  file.print(component);
  file.print(",");
  file.print(channel);
  file.print(",");
  file.println(message);

  file.close();
}

// ==================================================
// 各チャンネルのセンサー初期化
// ==================================================

void initializeChannel(uint8_t channel)
{
  Serial.println();
  Serial.print("Initializing Channel ");
  Serial.print(channel);
  Serial.print(" / ");
  Serial.print(SENSOR_DEPTH_CM[channel]);
  Serial.println(" cm");

  // SEN-30480
  if (!selectPCAChannel(channel))
  {
    Serial.println("Channel selection failed.");
    logEvent(
      "INIT_ERROR",
      "PCA9546",
      channel,
      "Channel selection failed for SEN-30480"
    );
  }
  else
  {
    delay(100);

    if (!moistureSensors[channel].begin())
    {
      Serial.println("SEN-30480 connection failed.");
      logEvent(
        "INIT_ERROR",
        "SEN-30480",
        channel,
        "SEN-30480 connection failed"
      );
    }
    else if (!selectPCAChannel(channel))
    {
      Serial.println("Channel reselection failed.");
      logEvent(
        "INIT_ERROR",
        "PCA9546",
        channel,
        "Channel reselection failed for SEN-30480"
      );
    }
    else
    {
      delay(50);

      if (!moistureSensors[channel].defaultMoistureSensorInit())
      {
        Serial.println("SEN-30480 initialization failed.");
        logEvent(
          "INIT_ERROR",
          "SEN-30480",
          channel,
          "SEN-30480 initialization failed"
        );
      }
      else
      {
        moistureSensorAvailable[channel] = true;
        Serial.println("SEN-30480 initialization OK.");
      }
    }
  }

  // FS304-SHT31
  if (!selectPCAChannel(channel))
  {
    Serial.println("Channel selection failed.");
    logEvent(
      "INIT_ERROR",
      "PCA9546",
      channel,
      "Channel selection failed for FS304-SHT31"
    );
  }
  else
  {
    delay(100);

    if (!sht31Sensors[channel].begin(SHT31_ADDRESS))
    {
      Serial.println("FS304-SHT31 initialization failed.");
      logEvent(
        "INIT_ERROR",
        "FS304-SHT31",
        channel,
        "FS304-SHT31 initialization failed"
      );
    }
    else
    {
      sht31SensorAvailable[channel] = true;
      Serial.println("FS304-SHT31 initialization OK.");
    }
  }
}

// ==================================================
// microSD初期化
// ==================================================

bool initializeSDCard()
{
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);

  for (uint8_t attempt = 1; attempt <= 5; attempt++)
  {
    Serial.print("SD initialization attempt ");
    Serial.println(attempt);

    if (SD.begin(SD_CS, SPI, SD_FREQUENCY))
    {
      if (SD.cardType() != CARD_NONE)
      {
        Serial.println("SD initialization OK.");
        return true;
      }
    }

    delay(1000);
  }

  return false;
}

// ==================================================
// CSVファイル作成
// ==================================================

bool createCSVFile()
{
  if (SD.exists(CSV_FILE))
  {
    Serial.println("soil8_log.csv already exists.");
    return true;
  }

  File file = SD.open(CSV_FILE, FILE_WRITE);

  if (!file)
  {
    Serial.println("Failed to create soil8_log.csv.");
    return false;
  }

  file.println(
    "timestamp_jst,"
    "capacitance_10cm_pf,humidity_10cm_rh,temperature_10cm_c,"
    "capacitance_30cm_pf,humidity_30cm_rh,temperature_30cm_c,"
    "capacitance_60cm_pf,humidity_60cm_rh,temperature_60cm_c,"
    "capacitance_90cm_pf,humidity_90cm_rh,temperature_90cm_c,"
    "status"
  );

  file.close();

  Serial.println("Created soil8_log.csv.");

  return true;
}

bool createEventLogFile()
{
  if (SD.exists(EVENT_FILE))
  {
    Serial.println("soil8_events.csv already exists.");
    return true;
  }

  File file = SD.open(EVENT_FILE, FILE_WRITE);

  if (!file)
  {
    Serial.println("Failed to create soil8_events.csv.");
    return false;
  }

  file.println(
    "timestamp_jst,event,component,channel,message"
  );

  file.close();

  Serial.println("Created soil8_events.csv.");

  return true;
}

// ==================================================
// 測定値をCSVへ保存
// ==================================================

bool saveMeasurement(
  const char *timestamp,
  const uint8_t capacitance[],
  const float humidity[],
  const float temperature[],
  const char *status)
{
  File file = SD.open(CSV_FILE, FILE_APPEND);

  if (!file)
  {
    Serial.println("Failed to open soil8_log.csv.");
    return false;
  }

  file.print(timestamp);

  for (uint8_t channel = 0;
       channel < SENSOR_COUNT;
       channel++)
  {
    file.print(",");

    if (capacitance[channel] == 0)
    {
      file.print("NA");
    }
    else
    {
      file.print(capacitance[channel]);
    }

    file.print(",");

    if (isnan(humidity[channel]))
    {
      file.print("NA");
    }
    else
    {
      file.print(humidity[channel], 2);
    }

    file.print(",");

    if (isnan(temperature[channel]))
    {
      file.print("NA");
    }
    else
    {
      file.print(temperature[channel], 2);
    }
  }

  file.print(",");
  file.println(status);

  file.close();

  return true;
}

// ==================================================
// すべてのセンサーを測定
// ==================================================

void takeMeasurement()
{
  char timestamp[24];
  formatTimestamp(timestamp, sizeof(timestamp));

  uint8_t capacitance[SENSOR_COUNT];
  float humidity[SENSOR_COUNT];
  float temperature[SENSOR_COUNT];

  bool allSensorsOK = true;

  for (uint8_t channel = 0;
       channel < SENSOR_COUNT;
       channel++)
  {
    capacitance[channel] = 0;
    humidity[channel] = NAN;
    temperature[channel] = NAN;

    if (!pcaAvailable)
    {
      allSensorsOK = false;
      continue;
    }

    // SEN-30480
    if (!moistureSensorAvailable[channel])
    {
      allSensorsOK = false;
    }
    else if (!selectPCAChannel(channel))
    {
      allSensorsOK = false;
      logEvent(
        "READ_ERROR",
        "PCA9546",
        channel,
        "Channel selection failed for SEN-30480 read"
      );
    }
    else
    {
      delay(20);

      capacitance[channel] =
        moistureSensors[channel].readCapacitancePF();

      if (capacitance[channel] == 0)
      {
        allSensorsOK = false;
        logEvent(
          "READ_ERROR",
          "SEN-30480",
          channel,
          "SEN-30480 capacitance read failed"
        );
      }
    }

    // FS304-SHT31
    if (!sht31SensorAvailable[channel])
    {
      allSensorsOK = false;
    }
    else if (!selectPCAChannel(channel))
    {
      allSensorsOK = false;
      logEvent(
        "READ_ERROR",
        "PCA9546",
        channel,
        "Channel selection failed for FS304-SHT31 read"
      );
    }
    else
    {
      delay(20);

      humidity[channel] =
        sht31Sensors[channel].readHumidity();

      temperature[channel] =
        sht31Sensors[channel].readTemperature();

      if (isnan(humidity[channel]) || isnan(temperature[channel]))
      {
        allSensorsOK = false;
        logEvent(
          "READ_ERROR",
          "FS304-SHT31",
          channel,
          "FS304-SHT31 humidity or temperature read failed"
        );
      }
    }
  }

  const char *status =
    allSensorsOK ? "OK" : "SENSOR_READ_ERROR";

  // シリアルモニタへ表示
  Serial.println();
  Serial.println("========== Measurement ==========");

  Serial.print("Time: ");
  Serial.println(timestamp);

  for (uint8_t channel = 0;
       channel < SENSOR_COUNT;
       channel++)
  {
    Serial.print("Channel ");
    Serial.print(channel);
    Serial.print(" / ");
    Serial.print(SENSOR_DEPTH_CM[channel]);
    Serial.println(" cm");

    Serial.print("  Capacitance: ");

    if (capacitance[channel] == 0)
    {
      Serial.println("READ ERROR");
    }
    else
    {
      Serial.print(capacitance[channel]);
      Serial.println(" pF");
    }

    Serial.print("  Relative humidity: ");

    if (isnan(humidity[channel]))
    {
      Serial.println("READ ERROR");
    }
    else
    {
      Serial.print(humidity[channel], 2);
      Serial.println(" %RH");
    }

    Serial.print("  Temperature: ");

    if (isnan(temperature[channel]))
    {
      Serial.println("READ ERROR");
    }
    else
    {
      Serial.print(temperature[channel], 2);
      Serial.println(" degC");
    }
  }

  Serial.print("Status: ");
  Serial.println(status);

  if (!sdCardAvailable)
  {
    Serial.println("SD card is not available. Measurement was not saved.");
    logEvent(
      "SD_ERROR",
      "SD",
      -1,
      "Measurement was not saved because SD card is not available"
    );
  }
  else if (
    saveMeasurement(
      timestamp,
      capacitance,
      humidity,
      temperature,
      status
    )
  )
  {
    Serial.println("Saved to SD card.");
  }
  else
  {
    Serial.println("SD write failed.");
    logEvent(
      "SD_ERROR",
      "SD",
      -1,
      "SD write failed for soil8_log.csv"
    );
  }
}

// ==================================================
// 初期設定
// ==================================================

void setup()
{
  Serial.begin(115200);
  delay(SERIAL_STARTUP_DELAY_MS);

  Serial.println();
  Serial.println(
    "Four SEN-30480 + Four FS304-SHT31 "
    "+ PCF8523 + microSD test"
  );

  // I2C開始
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // PCF8523
  rtcAvailable = rtc.begin();
  bool rtcTimeInvalid = false;

  if (rtcAvailable)
  {
    Serial.println("PCF8523 detected.");

    rtcTimeInvalid = !rtc.initialized() || rtc.lostPower();

    if (SET_RTC_FROM_COMPILE_TIME || rtcTimeInvalid)
    {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      rtc.start();

      if (rtcTimeInvalid)
      {
        Serial.println(
          "RTC time was invalid. "
          "RTC was set from the computer compile time."
        );
      }
      else
      {
        Serial.println(
          "RTC was set from the computer compile time."
        );
      }
    }

    Serial.println("PCF8523 initialization OK.");
  }
  else
  {
    Serial.println("PCF8523 was not detected.");
  }

  // microSD
  sdCardAvailable = initializeSDCard();

  if (sdCardAvailable)
  {
    bool measurementLogAvailable = createCSVFile();
    eventLogAvailable = createEventLogFile();

    if (!measurementLogAvailable)
    {
      sdCardAvailable = false;
      logEvent(
        "SD_ERROR",
        "SD",
        -1,
        "Failed to create soil8_log.csv"
      );
    }

    if (!eventLogAvailable)
    {
      logEvent(
        "SD_ERROR",
        "SD",
        -1,
        "Failed to create soil8_events.csv"
      );

      Serial.println(
        "Event logging to SD is disabled for this wake cycle."
      );
    }
  }
  else
  {
    logEvent(
      "SD_ERROR",
      "SD",
      -1,
      "SD initialization failed after 5 attempts"
    );

    Serial.println(
      "Measurement will continue without saving."
    );
  }

  if (!sdCardAvailable)
  {
    Serial.println("SD measurement logging is disabled for this wake cycle.");
  }

  if (!rtcAvailable)
  {
    logEvent(
      "INIT_ERROR",
      "PCF8523",
      -1,
      "PCF8523 was not detected"
    );
  }
  else if (rtcTimeInvalid)
  {
    logEvent(
      "RTC_SET",
      "PCF8523",
      -1,
      "RTC time was invalid and set from compile time"
    );
  }
  else if (SET_RTC_FROM_COMPILE_TIME)
  {
    logEvent(
      "RTC_SET",
      "PCF8523",
      -1,
      "RTC was set from compile time by configuration"
    );
  }

  // PCA9546確認
  Wire.beginTransmission(PCA9546_ADDRESS);
  pcaAvailable = Wire.endTransmission() == 0;

  if (pcaAvailable)
  {
    Serial.println("PCA9546 detected.");

    // 4チャンネルを初期化
    for (uint8_t channel = 0;
         channel < SENSOR_COUNT;
         channel++)
    {
      initializeChannel(channel);
    }
  }
  else
  {
    Serial.println("PCA9546 was not detected at 0x70.");
    logEvent(
      "INIT_ERROR",
      "PCA9546",
      -1,
      "PCA9546 was not detected at 0x70"
    );
  }

  Serial.println();
  Serial.println("Device initialization completed.");

  // 起動直後に1回測定
  takeMeasurement();

  Serial.println();
  Serial.println("Going to deep sleep for 10 minutes.");
  Serial.flush();
  delay(BEFORE_SLEEP_DELAY_MS);

  esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
  esp_deep_sleep_start();
}

// ==================================================
// 繰り返し
// ==================================================

void loop()
{
}

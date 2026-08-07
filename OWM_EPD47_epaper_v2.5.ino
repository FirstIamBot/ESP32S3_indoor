/**
 * @file main.cpp
 * @brief ESP32 Weather Display for LilyGo EPD 4.7" Display.
 * @details Данное ПО получает данные погоды через Open Weather Map API, 
 *          считывает локальные датчики (BMP085, SHT21), принимает данные 
 *          по протоколу ESP-NOW от внешнего сенсора и выводит их на E-Paper экран.
 * 
 * @author David Bird (Original concept, 2021)
 * @author Modified by User
 * @date 2026
 */

// =======================================================================
// 1. INCLUDES (#include)
// =======================================================================
#include <Arduino.h>           // In-built
#include <esp_task_wdt.h>       // In-built
#include "freertos/FreeRTOS.h"  // In-built
#include "freertos/task.h"      // In-built
#include "epd_driver.h"         // https://github.com/Xinyuan-LilyGO/LilyGo-EPD47
#include "esp_adc_cal.h"        // In-built

#include <ArduinoJson.h>        // https://github.com/bblanchon/ArduinoJson
#include <HTTPClient.h>         // In-built

#include <WiFi.h>               // In-built
#include <SPI.h>                // In-built
#include <time.h>               // In-built
#include <sys/time.h>           // In-built - settimeofday() для DS3231-фолбеку
#include <esp_system.h>         // In-built - esp_reset_reason() для діагностики brownout
#include <esp_now.h>
#include <esp_wifi.h>           // esp_wifi_set_channel() / нужен для диагностики канала ESP-NOW
#include <Preferences.h>        // In-built - зберігання відомих MAC-адрес сенсорів у NVS

#include <WiFiManager.h>        // https://github.com/tzapu/WiFiManager
#include <Wire.h>               // In-built
#include "SHTSensor.h"
#include <Adafruit_BMP085.h>    // https://github.com/adafruit/Adafruit-BMP085-Library

#include "owm_credentials.h"
#include "forecast_record.h"
#include "lang.h"

// Fonts
#include "opensans8b.h"
#include "opensans10b.h"
#include "opensans12b.h"
#include "opensans18b.h"
#include "opensans24b.h"

#include "driver/rtc_io.h"
#include <esp_sleep.h>
#include <touch.h>

// =======================================================================
// 2. DEFINES & CONSTANTS (#define)
// =======================================================================

/** @brief Канал Wi-Fi для работы протокола ESP-NOW. */
#define ESPNOW_WIFI_CHANNEL 1

/* GPIO Pins configuration */
#define S2 GPIO_NUM_34          /**< @brief Вывод GPIO 34. */
#define S3 GPIO_NUM_35          /**< @brief Вывод GPIO 35 (Кнопка Discovery). */
#define S4 GPIO_NUM_36          /**< @brief Вывод GPIO 36 (SENSOR_VP / ADC). */
#define S6 GPIO_NUM_4           /**< @brief Вывод GPIO 4. */
#define TOUCH_PANEL GPIO_SEL_13 /**< @brief Маска выбора GPIO для сенсора касания. */
#define TOUCH_INT   13          /**< @brief Вывод прерывания от сенсорной панели. */

#define SCREEN_WIDTH   EPD_WIDTH   /**< @brief Ширина экрана EPD. */
#define SCREEN_HEIGHT  EPD_HEIGHT  /**< @brief Высота экрана EPD. */

/* E-Paper Grayscale levels */
#define White         0xFF  /**< @brief Белый цвет. */
#define LightGrey     0xBB  /**< @brief Светло-серый цвет. */
#define Grey          0x88  /**< @brief Серый цвет. */
#define DarkGrey      0x44  /**< @brief Темно-серый цвет. */
#define Black         0x00  /**< @brief Черный цвет. */

/* Graph options */
#define autoscale_on  true  /**< @brief Автомасштабирование графика: ВКЛ. */
#define autoscale_off false /**< @brief Автомасштабирование графика: ВЫКЛ. */
#define barchart_on   true  /**< @brief Режим гистограммы: ВКЛ. */
#define barchart_off  false /**< @brief Режим гистограммы: ВЫКЛ. */

#define Large  20           /**< @brief Размер для отрисовки больших иконок. */
#define Small  8            /**< @brief Размер для отрисовки малых иконок. */

#define max_readings 24     /**< @brief Максимальное количество записей прогноза. */

#define MAX_KNOWN_SENSORS 8                                  /**< @brief Максимальное число сохраняемых сенсоров. */
#define DISCOVERY_MODE_DURATION_MS (60UL * 1000UL)           /**< @brief Время работы режима поиска новых сенсоров (мс). */
#define ESPNOW_SYNC_MAGIC 0xA5                               /**< @brief Магический байт подтверждения синхронизации. */

#define OUTDOOR_MAX_WAIT_MS  (10UL * 60UL * 1000UL)          /**< @brief Максимальное время ожидания внешнего сенсора (мс). */
#define INDOOR_MIN_ACTIVE_MS (3UL  * 60UL * 1000UL)          /**< @brief Минимальное время активной работы экрана (мс). */

#define DS3231_ADDR 0x68    /**< @brief I2C адрес часов реального времени DS3231. */

// Архітектура сну: Light Sleep (esp_light_sleep_start()) + epd_poweroff_light()
// (3.3В/PWR_EN лишається увімкненим). Обидва хардкоджені навмисно - Light
// Sleep зберігає стан змінних між пробудженнями (не потребує "нового
// циклу" через reboot), а увімкнене 3.3В потрібне для роботи QuickTouchWake().

// =======================================================================
// 3. ENUMS & STRUCTURES
// =======================================================================

/**
 * @enum alignment
 * @brief Выравнивание текста при вычислении координат печати.
 */
enum alignment {
  LEFT,   /**< Выравнивание по левому краю. */
  RIGHT,  /**< Выравнивание по правому краю. */
  CENTER  /**< Выравнивание по центру. */
};

/**
 * @struct espnow_sensor_msg_t
 * @brief Структура пакета данных, получаемого от внешнего датчика по ESP-NOW.
 */
typedef struct __attribute__((packed)) {
  float temperature;        /**< Температура (°C). */
  float humidity;           /**< Влажность (%). */
  float pressure;           /**< Атмосферное давление (Па). */
  float batteryVoltage;     /**< Напряжение аккумулятора (В). */
  uint8_t battery_percent;  /**< Заряд аккумулятора (%). */
  float g_solar_voltage;    /**< Напряжение солнечной панели (В). */
  uint8_t solar_percent;    /**< Эффективность/уровень солнечной панели (%). */
  int64_t timestamp;        /**< Метка времени от отправляющего устройства. */
} espnow_sensor_msg_t;

/**
 * @struct espnow_sync_ack_t
 * @brief Структура ответа для синхронизации интервалов сна внешнего датчика.
 */
typedef struct __attribute__((packed)) {
  uint8_t  magic;             /**< Магическое число подтверждения (0xA5). */
  uint32_t next_wake_in_sec;  /**< Время в секундах до следующего пробуждения. */
} espnow_sync_ack_t;

/**
 * @struct touchArea
 * @brief Прямоугольная область экрана для обработчика нажатий касания.
 */
struct touchArea {
  int x1; /**< Левая граница (X). */
  int x2; /**< Правая граница (X). */
  int y1; /**< Верхняя граница (Y). */
  int y2; /**< Нижняя граница (Y). */
};

// =======================================================================
// 4. GLOBAL VARIABLES & OBJECTS
// =======================================================================

String version = "2.5 / 4.7in";  /**< Версия прошивки. */

boolean LargeIcon   = true;   /**< Флаг большого размера иконок. */
boolean SmallIcon   = false;  /**< Флаг малого размера иконок. */
String  Time_str = "--:--:--";  /**< Строка форматированного времени. */
String  Date_str = "-- --- ----"; /**< Строка форматированной даты. */
int     wifi_signal, CurrentHour = 0, CurrentMin = 0, CurrentSec = 0, EventCnt = 0, vref = 1100;

RTC_DATA_ATTR Forecast_record_type  WxConditions[1];  /**< Данные о текущей погоде - переживає Deep Sleep для швидкого редраву за дотиком. */
Forecast_record_type  WxForecast[max_readings];   /**< Данные о прогнозе погоды. */

float pressure_readings[max_readings]    = {0};   /**< Массив графика давления. */
float temperature_readings[max_readings] = {0};   /**< Массив графика температуры. */
float humidity_readings[max_readings]    = {0};   /**< Массив графика влажности. */
float rain_readings[max_readings]        = {0};   /**< Массив графика осадков (дождь). */
float snow_readings[max_readings]        = {0};   /**< Массив графика осадков (снег). */

long SleepDuration   = 30;    /**< Периодичность сна (в минутах). */
int  WakeupHour      = 8;     /**< Час начала дневной работы. */
int  SleepHour       = 23;    /**< Час ухода в ночной сон. */
long StartTime       = 0;     /**< Метка времени запуска цикла (мс). */
long SleepTimer      = 0;     /**< Рассчитанное время сна (сек). */
long Delta           = 30;    /**< Дельта компенсации времени. */

GFXfont  currentFont;         /**< Текущий активный шрифт. */
uint8_t *framebuffer;         /**< Указатель на буфер кадра дисплея. */

// WiFi & Sensors
WiFiManager wm;               /**< Менеджер Wi-Fi соединений. */
Adafruit_BMP085 bmp;          /**< Объект барометра BMP085. */
SHTSensor sht;                /**< Объект датчика влажности/температуры SHT. */
TouchClass touch;             /**< Объект сенсорной панели. */

// Sensor Data (Local)
// RTC_DATA_ATTR - ці змінні переживають Deep Sleep (лишаються в RTC-пам'яті
// через перезавантаження), тому швидкий редрав "за дотиком" (без WiFi/OWM/
// ESP-NOW) може використати останні відомі значення одразу після пробудження.
RTC_DATA_ATTR float local_temperature = 0.0;  /**< Локальная температура с датчиков. */
RTC_DATA_ATTR float local_humidity = 0.0;     /**< Локальная влажность с датчиков. */
RTC_DATA_ATTR float local_pressure = 0.0;     /**< Локальное давление с датчиков. */
bool sensors_available = false; /**< Флаг доступности встроенных датчиков. */

// Sensor Data (External ESP-NOW)
RTC_DATA_ATTR float extendet_temperature = 0.0;     /**< Внешняя температура по ESP-NOW. */
RTC_DATA_ATTR float extendet_humidity = 0.0;        /**< Внешняя влажность по ESP-NOW. */
RTC_DATA_ATTR float extendet_pressure = 0.0;        /**< Внешнее давление по ESP-NOW. */
RTC_DATA_ATTR float extendet_batteryVoltage = 0;    /**< Напряжение батареи внешнего датчика. */
RTC_DATA_ATTR uint8_t extendet_battery_percent;     /**< Процент заряда батареи внешнего датчика. */
RTC_DATA_ATTR float extendet_g_solar_voltage;       /**< Напряжение солнечной панели внешнего датчика. */
RTC_DATA_ATTR uint8_t extendet_solar_percent = 0;   /**< Эффективность солнечной панели внешнего датчика. */
RTC_DATA_ATTR int64_t extendet_timestamp;           /**< Timestamp получения внешних данных. */

espnow_sensor_msg_t   lastEspNowMsg;       /**< Последний полученный пакет ESP-NOW. */
uint8_t               lastSenderMac[6] = {0}; /**< MAC-адрес последнего отправителя. */
volatile bool         espNowDataReceived = false; /**< Флаг получения пакета ESP-NOW. */
volatile unsigned long espNowLastRxMillis = 0;   /**< Время приема пакета ESP-NOW. */

Preferences sensorPrefs;                            /**< Объект NVS энергонезависимой памяти. */
uint8_t knownSensorMacs[MAX_KNOWN_SENSORS][6];      /**< Список известной MAC-адресов датчиков. */
int     knownSensorCount = 0;                       /**< Количество сохраненных датчиков. */

bool          discoveryModeActive = false;          /**< Флаг активности режима поиска сенсоров. */
unsigned long discoveryModeStartMillis = 0;         /**< Время старта режима поиска. */

touchArea leftTouch;   /**< Левая сенсорная зона переключения страниц. */
touchArea rihgtTouch;  /**< Правая сенсорная зона переключения страниц. */

static uint8_t page = 2;       /**< Общее количество доступных страниц. */
RTC_DATA_ATTR uint8_t currentPage = 0;       /**< Індекс активної сторінки - переживає Deep Sleep. */

static bool cycleDataHandled = false; /**< Флаг обработки цикла данных ESP-NOW. */

// =======================================================================
// 5. FUNCTION PROTOTYPES
// =======================================================================
void SendSyncAck(const uint8_t *mac_addr);
bool IsKnownSensor(const uint8_t *mac);
void LoadKnownSensors();
void SaveNewSensor(const uint8_t *mac);
void CheckDiscoveryButton();
void OnEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
bool StartEspNowListener();
bool GetEspNowData();
void BeginSleep();
boolean SetupTime();
static uint8_t ds3231_bcd2dec(uint8_t bcd);
static uint8_t ds3231_dec2bcd(uint8_t dec);
void DS3231_SetTime(const struct tm *timeinfo);
bool DS3231_GetTime(struct tm *timeinfo);
uint8_t StartWiFi();
void StopWiFi();
bool InitialiseSensors();
bool ReadLocalSensors();
void InitialiseSystem();
bool IsWakeTime();
void FetchAndShowOnlineWeather();
void Convert_Readings_to_Imperial();
bool DecodeWeather(WiFiClient& json, String Type);
String ConvertUnixTime(int unix_time);
bool obtainWeatherData(WiFiClient & client, const String & RequestType);
float mm_to_inches(float value_mm);
float hPa_to_inHg(float value_hPa);
int JulianDate(int d, int m, int y);
float SumOfPrecip(float DataArray[], int readings);
String TitleCase(String text);
double NormalizedMoonPhase(int d, int m, int y);
void DisplayGeneralInfoSection();
void DisplayWeatherIcon(int x, int y);
void DisplayMainWeatherSection(int x, int y);
void DisplayDisplayWindSection(int x, int y, float angle, float windspeed, int Cradius);
String WindDegToOrdinalDirection(float winddirection);
void DisplayTemperatureSection(int x, int y);
void DisplayForecastTextSection(int x, int y);
void DisplayPressureSection(int x, int y, float pressure, String slope);
void DisplayForecastWeather(int x, int y, int index);
void DisplayAstronomySection(int x, int y);
void DrawMoon(int x, int y, int dd, int mm, int yy, String hemisphere);
String MoonPhase(int d, int m, int y, String hemisphere);
void DisplayForecastSection(int x, int y);
void DisplayConditionsSection(int x, int y, String IconName, bool IconSize);
void arrow(int x, int y, int asize, float aangle, int pwidth, int plength);
void DrawSegment(int x, int y, int o1, int o2, int o3, int o4, int o11, int o12, int o13, int o14);
void DrawPressureAndTrend(int x, int y, float pressure, String slope);
void DisplayStatusSection(int x, int y, int rssi);
void DrawRSSI(int x, int y, int rssi);
boolean UpdateLocalTime();
void DrawBattery(int x, int y);
void addcloud(int x, int y, int scale, int linesize);
void addrain(int x, int y, int scale, bool IconSize);
void addsnow(int x, int y, int scale, bool IconSize);
void addtstorm(int x, int y, int scale);
void addsun(int x, int y, int scale, bool IconSize);
void addfog(int x, int y, int scale, int linesize, bool IconSize);
void Sunny(int x, int y, bool IconSize, String IconName);
void MostlySunny(int x, int y, bool IconSize, String IconName);
void MostlyCloudy(int x, int y, bool IconSize, String IconName);
void Cloudy(int x, int y, bool IconSize, String IconName);
void Rain(int x, int y, bool IconSize, String IconName);
void ExpectRain(int x, int y, bool IconSize, String IconName);
void ChanceRain(int x, int y, bool IconSize, String IconName);
void Tstorms(int x, int y, bool IconSize, String IconName);
void Snow(int x, int y, bool IconSize, String IconName);
void Fog(int x, int y, bool IconSize, String IconName);
void Haze(int x, int y, bool IconSize, String IconName);
void CloudCover(int x, int y, int CCover);
void Visibility(int x, int y, String Visi);
void addmoon(int x, int y, int scale, bool IconSize);
void Nodata(int x, int y, bool IconSize, String IconName);
void DrawGraph(int x_pos, int y_pos, int gwidth, int gheight, float Y1Min, float Y1Max, String title, float DataArray[], int readings, boolean auto_scale, boolean barchart_mode);
void drawString(int x, int y, String text, alignment align);
void fillCircle(int x, int y, int r, uint8_t color);
void drawFastHLine(int16_t x0, int16_t y0, int length, uint16_t color);
void drawFastVLine(int16_t x0, int16_t y0, int length, uint16_t color);
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void drawCircle(int x0, int y0, int r, uint8_t color);
void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void drawPixel(int x, int y, uint8_t color);
void setFont(GFXfont const &font);
void edp_update();
void DisplayWeather();
void DisplayLocalWeather();
void DisplayPage();

// =======================================================================
// SETUP & LOOP
// =======================================================================

/**
 * @brief Запускає новий робочий цикл: WiFi, NTP, OWM, локальні датчики,
 *        перезапуск ESP-NOW-слухача. Викликається з setup() (перший старт)
 *        і повторно з BeginSleep() одразу після пробудження з Light Sleep -
 *        оскільки Light Sleep НЕ перезавантажує чіп, setup() вдруге вже не
 *        викликається, тож весь "новий цикл" треба запускати явно тут.
 */
void StartNewCycle() {
  StartTime = millis();
  espNowDataReceived = false;
  cycleDataHandled = false;

  WiFi.mode(WIFI_STA);
  StartEspNowListener();

  bool wifiOK = (StartWiFi() == WL_CONNECTED && SetupTime());

  ReadLocalSensors(); 

  if (!wifiOK) {
    Serial.println("No WiFi - reading local sensors only");
    GetEspNowData();
    currentPage = 1;
    DisplayPage();
    return;
  }

  if (IsWakeTime()) {
    FetchAndShowOnlineWeather(); 
  }

  GetEspNowData(); 
}

/**
 * @brief Функция начальной настройки устройства.
 */
void setup() {
  InitialiseSystem();
  StartNewCycle();
}

/**
 * @brief Главный цикл программы.
 */
void loop() {
  unsigned long elapsed = millis() - StartTime;

  CheckDiscoveryButton(); 

  if (discoveryModeActive) {
    DisplayPage();
    return;
  }

  if (espNowDataReceived && !cycleDataHandled) {
    Serial.println("ESP-NOW: дані від OUTDOOR отримано - виводимо на екран");
    GetEspNowData(); 
    epd_poweron();
    epd_clear();
    memset(framebuffer, 0xFF, EPD_WIDTH * EPD_HEIGHT / 2);
    DisplayWeather();
    edp_update();
    epd_poweroff_light(); 

    Serial.println("Надсилаємо sync ACK (timestamp) на OUTDOOR...");
    SendSyncAck(lastSenderMac);
    cycleDataHandled = true;
  }

  if (cycleDataHandled) {
    if (elapsed >= INDOOR_MIN_ACTIVE_MS) {
      Serial.println("Мінімальний час активності екрана вичерпано - йдемо спати");
      cycleDataHandled = false; 
      BeginSleep();
      return;
    }
    DisplayPage();
    return;
  }

  if (elapsed < OUTDOOR_MAX_WAIT_MS) {
    DisplayPage();
    return;
  }

  Serial.println("ESP-NOW: за 10 хвилин OUTDOOR не знайдено - засинаємо без синхронізації");
  BeginSleep();
}

// =======================================================================
// 6. FUNCTIONS REALIZATION
// =======================================================================

/**
 * @brief Проверяет, известен ли указанный MAC-адрес сенсора.
 * @param mac Указатель на массив байт MAC-адреса (6 байт).
 * @return true Если сенсор есть в списке известных, иначе false.
 */
bool IsKnownSensor(const uint8_t *mac) {
  for (int i = 0; i < knownSensorCount; i++) {
    if (memcmp(knownSensorMacs[i], mac, 6) == 0) return true;
  }
  return false;
}

/**
 * @brief Загружает список известных MAC-адресов датчиков из NVS памяти.
 */
void LoadKnownSensors() {
  sensorPrefs.begin("sensors", true); 
  knownSensorCount = sensorPrefs.getUInt("count", 0);
  if (knownSensorCount > MAX_KNOWN_SENSORS) knownSensorCount = MAX_KNOWN_SENSORS;
  for (int i = 0; i < knownSensorCount; i++) {
    char key[8];
    snprintf(key, sizeof(key), "mac%d", i);
    sensorPrefs.getBytes(key, knownSensorMacs[i], 6);
  }
  sensorPrefs.end();

  if (knownSensorCount == 0) {
    const uint8_t defaultMac[6] = {0x20, 0x6E, 0xF1, 0xAF, 0xE0, 0x88};
    memcpy(knownSensorMacs[0], defaultMac, 6);
    knownSensorCount = 1;
    Serial.println("NVS: список сенсорів порожній - додано типовий сенсор за замовчуванням");
  }

  Serial.printf("NVS: завантажено %d відомих сенсор(ів):\n", knownSensorCount);
  for (int i = 0; i < knownSensorCount; i++) {
    Serial.printf("  [%d] %02X:%02X:%02X:%02X:%02X:%02X\n", i,
                  knownSensorMacs[i][0], knownSensorMacs[i][1], knownSensorMacs[i][2],
                  knownSensorMacs[i][3], knownSensorMacs[i][4], knownSensorMacs[i][5]);
  }
}

/**
 * @brief Сохраняет новый MAC-адрес сенсора в энергонезависимую память (NVS).
 * @param mac Указатель на массив из 6 байт MAC-адреса.
 */
void SaveNewSensor(const uint8_t *mac) {
  if (knownSensorCount >= MAX_KNOWN_SENSORS) {
    Serial.println("NVS: досягнуто ліміту сенсорів - новий не додано");
    return;
  }
  memcpy(knownSensorMacs[knownSensorCount], mac, 6);
  knownSensorCount++;

  sensorPrefs.begin("sensors", false); 
  sensorPrefs.putUInt("count", knownSensorCount);
  char key[8];
  snprintf(key, sizeof(key), "mac%d", knownSensorCount - 1);
  sensorPrefs.putBytes(key, mac, 6);
  sensorPrefs.end();

  Serial.printf("NVS: новий сенсор збережено -> %02X:%02X:%02X:%02X:%02X:%02X (всього: %d)\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], knownSensorCount);
}

/**
 * @brief Проверяет нажатие физической кнопки S3 для активации режима поиска датчиков.
 */
void CheckDiscoveryButton() {
  static bool lastState = HIGH;
  bool state = digitalRead(S3);

  if (lastState == HIGH && state == LOW) { 
    discoveryModeActive = true;
    discoveryModeStartMillis = millis();
    Serial.println("Кнопка S3: увімкнено режим пошуку нового сенсора (60 сек)");
  }
  lastState = state;

  if (discoveryModeActive && (millis() - discoveryModeStartMillis > DISCOVERY_MODE_DURATION_MS)) {
    discoveryModeActive = false;
    Serial.println("Режим пошуку нового сенсора: час вийшов, вимкнено");
  }
}

/**
 * @brief Колбэк-функция получения данных ESP-NOW.
 * @param mac_addr MAC-адрес отправителя.
 * @param data Указатель на полученные данные.
 * @param data_len Длина данных в байтах.
 */
void OnEspNowRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Serial.printf("ESP-NOW: пакет від MAC %02X:%02X:%02X:%02X:%02X:%02X, розмір %d байт\n",
                mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
                data_len);

  bool known = IsKnownSensor(mac_addr);

  if (!known) {
    if (!discoveryModeActive) {
      Serial.println("ESP-NOW: пакет від невідомого MAC (режим пошуку вимкнено) - ігноруємо");
      return;
    }
    if (data_len != sizeof(espnow_sensor_msg_t)) {
      Serial.println("ESP-NOW: невідомий MAC у режимі пошуку, але розмір пакета не збігається - ігноруємо");
      return;
    }
    if (!esp_now_is_peer_exist(mac_addr)) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, mac_addr, 6);
      peer.channel = ESPNOW_WIFI_CHANNEL;
      peer.ifidx   = WIFI_IF_STA;
      peer.encrypt = false;
      esp_now_add_peer(&peer);
    }
    SaveNewSensor(mac_addr);
    discoveryModeActive = false; 
    Serial.println("ESP-NOW: новий сенсор знайдено, режим пошуку вимкнено");
  }

  if (data_len == sizeof(espnow_sensor_msg_t)) {
    memcpy((void*)&lastEspNowMsg, data, sizeof(lastEspNowMsg));
    memcpy(lastSenderMac, mac_addr, 6); 
    espNowDataReceived = true;
    espNowLastRxMillis = millis();
    Serial.printf("ESP-NOW: прийняті дані -> T=%.2fC H=%.2f%% P=%.2fPa Vbat=%.2fV(%u%%) Vsolar=%.2fV(%u%%) ts=%lld\n",
                  lastEspNowMsg.temperature, lastEspNowMsg.humidity, lastEspNowMsg.pressure,
                  lastEspNowMsg.batteryVoltage, lastEspNowMsg.battery_percent,
                  lastEspNowMsg.g_solar_voltage, lastEspNowMsg.solar_percent,
                  (long long)lastEspNowMsg.timestamp);
  } else {
    Serial.printf("ESP-NOW: unexpected payload size %d (expected %d)\n",
                  data_len, (int)sizeof(espnow_sensor_msg_t));
  }
}

/**
 * @brief Инициализирует модуль ESP-NOW и регистрирует приемник.
 * @return true В случае успещной инициализации, иначе false.
 */
bool StartEspNowListener() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW: init failed");
    return false;
  }
  esp_now_register_recv_cb(OnEspNowRecv);

  LoadKnownSensors(); 

  for (int i = 0; i < knownSensorCount; i++) {
    if (!esp_now_is_peer_exist(knownSensorMacs[i])) {
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, knownSensorMacs[i], 6);
      peer.channel = ESPNOW_WIFI_CHANNEL;
      peer.ifidx   = WIFI_IF_STA;
      peer.encrypt = false;
      esp_now_add_peer(&peer);
    }
  }

  pinMode(S3, INPUT); 

  Serial.println("ESP-NOW: listening for sensor data in background...");
  return true;
}

/**
 * @brief Копирует полученные через ESP-NOW данные во внешние переменные состояния.
 * @return true Если новые данные были получены, false если за цикл данных не поступало.
 */
bool GetEspNowData() {
  if (espNowDataReceived) {
      extendet_temperature = lastEspNowMsg.temperature;;
      extendet_humidity = lastEspNowMsg.humidity;
      extendet_pressure = lastEspNowMsg.pressure;
      extendet_batteryVoltage = lastEspNowMsg.batteryVoltage;  
      extendet_solar_percent = lastEspNowMsg.solar_percent;  
      extendet_battery_percent = lastEspNowMsg.battery_percent; 
      extendet_g_solar_voltage = lastEspNowMsg.g_solar_voltage;
      extendet_timestamp = lastEspNowMsg.timestamp;   

    Serial.printf("ESP-NOW data: T=%.1f°C H=%.1f%% P=%.1fhPa Bat=%.2fV Solar=%u%% (%lus назад)\n",
                  extendet_temperature, extendet_humidity, extendet_pressure / 100.0f,
                  extendet_batteryVoltage, extendet_solar_percent,
                  (millis() - espNowLastRxMillis) / 1000);
    return true;
  }
  Serial.println("ESP-NOW: за этот цикл данных не поступало");
  return false;
}

/**
 * @brief Отправляет пакет синхронизации (ACK) внешнему устройству по ESP-NOW.
 * @param mac_addr MAC-адрес получателя.
 */
void SendSyncAck(const uint8_t *mac_addr) {
  if (!esp_now_is_peer_exist(mac_addr)) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac_addr, 6);
    peer.channel = ESPNOW_WIFI_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
      Serial.println("ESP-NOW: не вдалося додати пір для sync ack");
      return;
    }
  }

  const long WAKE_MARGIN_SEC = 45; // 
  long secUntilNextWake;

  time_t now = time(nullptr);
  bool timeIsValid = (now >= 1700000000); 
  if (timeIsValid) {
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    long secIntoWindow = (long)(timeinfo.tm_min % SleepDuration) * 60L + timeinfo.tm_sec;
    secUntilNextWake = SleepDuration * 60L - secIntoWindow;
    if (secUntilNextWake <= 0) secUntilNextWake += SleepDuration * 60L;
    secUntilNextWake = max(1L, secUntilNextWake - WAKE_MARGIN_SEC);
  } else {
    secUntilNextWake = max(1L, SleepDuration * 60L - WAKE_MARGIN_SEC);
    Serial.println("ESP-NOW: час ще не синхронізовано (NTP) - шлемо ЗАПАСНИЙ (не вирівняний) sync ack");
  }

  espnow_sync_ack_t ack = { ESPNOW_SYNC_MAGIC, (uint32_t)secUntilNextWake };
  esp_err_t res = esp_now_send(mac_addr, (const uint8_t *)&ack, sizeof(ack));
  Serial.printf("ESP-NOW: sync ack -> наступне пробудження через %lu с (%s, %s)\n",
                (unsigned long)secUntilNextWake, res == ESP_OK ? "надіслано" : "помилка відправки",
                timeIsValid ? "вирівняно по NTP" : "запасний інтервал");
}

/**
 * @brief Переводить мікроконтролер у Light Sleep. Light Sleep НЕ перезавантажує
 *        чіп (на відміну від Deep Sleep) - RAM/змінні зберігаються, тому
 *        виконання просто продовжується одразу після esp_light_sleep_start().
 *        Через це весь "новий цикл" (WiFi/NTP/OWM/ESP-NOW) запускається явно
 *        через StartNewCycle() тут же, а не через повторний виклик setup().
 */
void BeginSleep() {
  StopWiFi(); 
  epd_poweroff_light(); // 3.3В лишається увімкненим - потрібно і для тачскріна, і для коректного Light Sleep

  UpdateLocalTime();
  SleepTimer = (SleepDuration * 60 - ((CurrentMin % SleepDuration) * 60 + CurrentSec)) + Delta; 
  esp_sleep_enable_timer_wakeup(SleepTimer * 1000000LL); 
  Serial.println("Awake for : " + String((millis() - StartTime) / 1000.0, 3) + "-secs");
  Serial.println("Entering " + String(SleepTimer) + " (secs) of sleep time");
  Serial.println("Starting light-sleep period...");
  esp_light_sleep_start();

  // ---- Виконання продовжується ТУТ ЖЕ після пробудження ----
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_EXT1) {
    Serial.println("Пробудження дотиком під час Light Sleep");
    QuickTouchWake(); // швидкий редрав + рекурсивне повернення в сон на решту часу
    return;
  }

  Serial.println("Прокинулись з Light Sleep (таймер) - починаємо новий цикл");
  StartNewCycle();
}

/**
 * @brief Настраивает конфигурацию времени через NTP и таймзону.
 * @return true В случае успеха обновления времени.
 */
boolean SetupTime() {
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, "time.nist.gov"); 
  setenv("TZ", Timezone, 1);  
  tzset(); 
  delay(100);
  return UpdateLocalTime();
}

/**
 * @brief Преобразует BCD значение в десятичный формат.
 * @param bcd Число в формате BCD.
 * @return Десятичное число.
 */
static uint8_t ds3231_bcd2dec(uint8_t bcd) { return ((bcd >> 4) * 10) + (bcd & 0x0F); }

/**
 * @brief Преобразует десятичное значение в BCD формат.
 * @param dec Десятичное число.
 * @return Число в формате BCD.
 */
static uint8_t ds3231_dec2bcd(uint8_t dec) { return ((dec / 10) << 4) | (dec % 10); }

/**
 * @brief Записывает текущее время из структуры tm в RTC модуль DS3231.
 * @param timeinfo Указатель на структуру со временем.
 */
void DS3231_SetTime(const struct tm *timeinfo) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x00); 
  Wire.write(ds3231_dec2bcd(timeinfo->tm_sec));
  Wire.write(ds3231_dec2bcd(timeinfo->tm_min));
  Wire.write(ds3231_dec2bcd(timeinfo->tm_hour));      
  Wire.write(ds3231_dec2bcd(timeinfo->tm_wday + 1));  
  Wire.write(ds3231_dec2bcd(timeinfo->tm_mday));
  Wire.write(ds3231_dec2bcd(timeinfo->tm_mon + 1));
  Wire.write(ds3231_dec2bcd(timeinfo->tm_year - 100)); 
  uint8_t res = Wire.endTransmission();
  if (res == 0) {
    Serial.println("DS3231: час записано (синхронізовано з NTP)");
  } else {
    Serial.printf("DS3231: помилка запису часу (код %d)\n", res);
  }
}

/**
 * @brief Читает текущее время из DS3231.
 * @param timeinfo Указатель на структуру `tm` для сохранения времени.
 * @return true В случае успещного чтения, иначе false.
 */
bool DS3231_GetTime(struct tm *timeinfo) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  Wire.requestFrom(DS3231_ADDR, 7);
  if (Wire.available() < 7) return false;

  uint8_t sec    = ds3231_bcd2dec(Wire.read() & 0x7F);
  uint8_t minute = ds3231_bcd2dec(Wire.read());
  uint8_t hour   = ds3231_bcd2dec(Wire.read() & 0x3F); 
  uint8_t wday   = ds3231_bcd2dec(Wire.read());
  uint8_t mday   = ds3231_bcd2dec(Wire.read());
  uint8_t month  = ds3231_bcd2dec(Wire.read() & 0x1F);
  uint8_t year   = ds3231_bcd2dec(Wire.read());

  timeinfo->tm_sec   = sec;
  timeinfo->tm_min   = minute;
  timeinfo->tm_hour  = hour;
  timeinfo->tm_wday  = wday - 1;
  timeinfo->tm_mday  = mday;
  timeinfo->tm_mon   = month - 1;
  timeinfo->tm_year  = year + 100; 
  timeinfo->tm_isdst = 0;
  return true;
}

/**
 * @brief Инициализирует и подключает Wi-Fi через WiFiManager.
 * @return Статус подключения (WL_CONNECTED и т.д.).
 */
uint8_t StartWiFi() {
  Serial.println("\r\nConnecting to WiFi...");
  
  wm.setConfigPortalTimeout(180); 
  wm.setConnectTimeout(30);        
  
  bool res = wm.autoConnect("ESP32_Weather", "12345678");
  
  if (!res) {
    Serial.println("Failed to connect or hit timeout");
    return WL_CONNECT_FAILED;
  }
  
  wifi_signal = WiFi.RSSI(); 
  Serial.println("WiFi connected!");
  Serial.println("SSID: " + WiFi.SSID());
  Serial.println("IP: " + WiFi.localIP().toString());
  Serial.println("MAC adress: " + WiFi.macAddress());

  uint8_t actualChannel = WiFi.channel();
  Serial.printf("WiFi channel: %d (ESPNOW_WIFI_CHANNEL = %d)\n", actualChannel, ESPNOW_WIFI_CHANNEL);
  if (actualChannel != ESPNOW_WIFI_CHANNEL) {
    Serial.println("WARNING: router channel != ESPNOW_WIFI_CHANNEL - "
                    "ESP-NOW от узла-датчика может не приниматься! "
                    "Зафиксируйте канал роутера или поправьте ESPNOW_WIFI_CHANNEL.");
  }

  return WiFi.status();
}

/**
 * @brief Отключает модуль Wi-Fi для экономии энергии.
 */
void StopWiFi() {
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi switched Off");
}

/**
 * @brief Инициализирует локальные физические датчики BMP085 и SHT21 по I2C.
 * @return true Если оба датчика успешно инициализированы.
 */
bool InitialiseSensors() {
  Wire.begin(15, 14);  
  bool sht_ok = false;
  bool bmp_ok = false;
  
  if (bmp.begin()) {
    Serial.println("BMP085 sensor found!");
    bmp_ok = true;
  } else {
    Serial.println("Could not find BMP085 sensor!");
    return bmp_ok ;
  }
  
  if (sht.init()) {
    Serial.println("SHT21 sensor initialization successful.");
    return sht_ok = true;
  } else {
    Serial.println("SHT21 sensor initialization failed!");
    return sht_ok;
  }
}

/**
 * @brief Считывает данные с локальных датчиков.
 * @return true Если данные успешно прочитаны, false при ошибках NaN.
 */
bool ReadLocalSensors() {
  float sht21_temp;
  float sht21_hymidity;

  if (sht.readSample()) {
    sht21_temp = sht.getTemperature();
    sht21_hymidity = sht.getHumidity();
  } else {
    Serial.println("Error reading sensor data");
  }

  float temp_bmp = bmp.readTemperature();
  float press = bmp.readPressure() / 100.0F; 
  
  local_temperature = (temp_bmp + sht21_temp) * 0.5; 
  local_pressure = press;
  local_humidity = sht21_hymidity;

  Serial.printf("Local data: T=%.2f°C, H=%.2f%%, P=%.2f hPa\n", 
                local_temperature, local_humidity, local_pressure);
                
  if(isnan(local_temperature) && isnan(local_pressure) && isnan(local_humidity)){
    return false;
  }
  return true;
}

/**
 * @brief Инициализирует аппаратные ресурсы системы (Serial, EPD, Touch, Sleep sources).
 */
void InitialiseSystem() {
  StartTime = millis();
  Serial.begin(115200);
  while (!Serial);
  Serial.println(String(__FILE__) + "\nStarting...");

  esp_reset_reason_t reset_reason = esp_reset_reason();
  Serial.printf("Причина запуску/скидання: %d (%s)\n", reset_reason,
                reset_reason == ESP_RST_BROWNOUT ? "BROWNOUT!!! Проблема з живленням/струмом" :
                reset_reason == ESP_RST_DEEPSLEEP ? "нормальне пробудження з Deep Sleep" :
                reset_reason == ESP_RST_POWERON ? "звичайне включення живлення" :
                reset_reason == ESP_RST_PANIC ? "PANIC (crash)" : "інше");

  epd_init();
  framebuffer = (uint8_t *)ps_calloc(sizeof(uint8_t), EPD_WIDTH * EPD_HEIGHT / 2);
  if (!framebuffer) Serial.println("Memory alloc failed!");
  memset(framebuffer, 0xFF, EPD_WIDTH * EPD_HEIGHT / 2);
  
  sensors_available = InitialiseSensors();
  if (sensors_available) {
    Serial.println("Local sensors initialized successfully");
  } else {
    Serial.println("Warning: No local sensors found");
  }
  if (!touch.begin()) {
    Serial.println("start touchscreen failed");
    while (1);
  }

  // Пробудження від дотику дозволене знову - тепер обробляється швидким
  // шляхом (QuickTouchWake(), без WiFi/OWM/ESP-NOW), який одразу повертає
  // пристрій у сон на РЕШТУ розрахованого часу - розклад не порушується.
  esp_sleep_enable_ext1_wakeup(TOUCH_PANEL, ESP_EXT1_WAKEUP_ANY_HIGH);
}

// Швидке пробудження дотиком: перемикає сторінку і одразу перемальовує
// екран ОСТАННІМИ збереженими даними (без WiFi/OWM/ESP-NOW - секунди, а не
// хвилини), після чого негайно повертається в Deep Sleep на решту
// розрахованого часу до найближчої межі SleepDuration. Час рахується напряму
// з DS3231 (швидко, без очікування NTP - WiFi тут навіть не піднімається).
// Викликається з BeginSleep() одразу після esp_light_sleep_start() -
// Light Sleep НЕ перезавантажує чіп, тому це просто продовження виконання,
// а не окремий "боот".
void QuickTouchWake() {
  currentPage = (currentPage + 1) % page;
  Serial.printf("Дотик під час сну: перемикаємо на сторінку %d (швидкий редрав)\n", currentPage);

  epd_poweron();
  epd_clear();
  memset(framebuffer, 0xFF, EPD_WIDTH * EPD_HEIGHT / 2);
  if (currentPage == 0) {
    DisplayWeather();
  } else {
    DisplayLocalWeather();
  }
  edp_update();
  epd_poweroff_light(); // 3.3В лишається увімкненим - тачскрін готовий до наступного дотику

  struct tm timeinfo;
  if (DS3231_GetTime(&timeinfo)) {
    long secIntoWindow = (long)(timeinfo.tm_min % SleepDuration) * 60L + timeinfo.tm_sec;
    SleepTimer = SleepDuration * 60 - secIntoWindow;
    if (SleepTimer <= 0) SleepTimer += SleepDuration * 60;
  } else {
    SleepTimer = SleepDuration * 60; // останній запасний варіант, якщо навіть DS3231 недоступний
  }

  Serial.println("Швидке пробудження: повертаємось у Light Sleep на " + String(SleepTimer) + " с");
  esp_sleep_enable_timer_wakeup((uint64_t)SleepTimer * 1000000ULL);
  esp_light_sleep_start();

  // Знову продовження одразу після пробудження - могло бути ще одне
  // торкання, або спрацював таймер (тоді час запускати новий повний цикл).
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_EXT1) {
    QuickTouchWake(); // ще один дотик - рекурсивно повторюємо швидкий редрав
  } else {
    Serial.println("Прокинулись з Light Sleep (таймер) після дотику - починаємо новий цикл");
    StartNewCycle();
  }
}

/**
 * @brief Проверяет, входит ли текущий час в интервал бодрствования.
 * @return true Если устройство должно работать, false если время ночного сна.
 */
bool IsWakeTime() {
  if (WakeupHour > SleepHour)
    return (CurrentHour >= WakeupHour || CurrentHour <= SleepHour);
  return (CurrentHour >= WakeupHour && CurrentHour <= SleepHour);
}

/**
 * @brief Выполняет загрузку данных погоды с OpenWeatherMap и обновляет дисплей.
 */
void FetchAndShowOnlineWeather() {
  byte Attempts = 1;
  bool RxWeather  = false;
  bool RxForecast = false;
  WiFiClient client;   
  while ((!RxWeather || !RxForecast) && Attempts <= 2) { 
    if (!RxWeather)  RxWeather  = obtainWeatherData(client, "weather");
    if (!RxForecast) RxForecast = obtainWeatherData(client, "forecast");
    Attempts++;
  }
  Serial.println("Received all weather data...");

  if (RxWeather && RxForecast) { 
    GetEspNowData();    
    epd_poweron();
    epd_clear();
    DisplayWeather();
    edp_update();
    epd_poweroff_light(); 
  }
}

// =======================================================================
// HELPER & DISPLAY FUNCTIONS
// =======================================================================

/**
 * @brief Конвертирует значения из метрических единиц в империалистические.
 */
void Convert_Readings_to_Imperial() { 
  WxConditions[0].Pressure = hPa_to_inHg(WxConditions[0].Pressure);
  WxForecast[0].Rainfall   = mm_to_inches(WxForecast[0].Rainfall);
  WxForecast[0].Snowfall   = mm_to_inches(WxForecast[0].Snowfall);
}

/**
 * @brief Декодирует JSON-ответ от OpenWeather API.
 * @param json Ссылка на поток данных от HTTP-клиента.
 * @param Type Тип данных ("weather" или "forecast").
 * @return true В случае успещного парсинга, false при ошибке.
 */
bool DecodeWeather(WiFiClient& json, String Type) {
  Serial.print(F("\nCreating object...and "));
  DynamicJsonDocument doc(64 * 1024);                      
  DeserializationError error = deserializeJson(doc, json); 
  if (error) {                                             
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return false;
  }
  
  JsonObject root = doc.as<JsonObject>();
  Serial.println(" Decoding " + Type + " data");
  if (Type == "weather") {
    WxConditions[0].Main0       = root["weather"][0]["main"].as<const char*>();        Serial.println("Main: " + String(WxConditions[0].Main0));
    WxConditions[0].Forecast0   = root["weather"][0]["description"].as<const char*>(); Serial.println("For0: " + String(WxConditions[0].Forecast0));
    WxConditions[0].Icon        = root["weather"][0]["icon"].as<const char*>();        Serial.println("Icon: " + String(WxConditions[0].Icon));
    WxConditions[0].Temperature = root["main"]["temp"].as<float>();              Serial.println("Temp: " + String(WxConditions[0].Temperature));
    WxConditions[0].Pressure    = root["main"]["pressure"].as<float>();          Serial.println("Pres: " + String(WxConditions[0].Pressure));
    WxConditions[0].Humidity    = root["main"]["humidity"].as<float>();          Serial.println("Humi: " + String(WxConditions[0].Humidity));
    WxConditions[0].Low         = root["main"]["temp_min"].as<float>();          Serial.println("TLow: " + String(WxConditions[0].Low));
    WxConditions[0].High        = root["main"]["temp_max"].as<float>();          Serial.println("THig: " + String(WxConditions[0].High));
    WxConditions[0].Windspeed   = root["wind"]["speed"].as<float>();             Serial.println("WSpd: " + String(WxConditions[0].Windspeed));
    WxConditions[0].Winddir     = root["wind"]["deg"].as<float>();               Serial.println("WDir: " + String(WxConditions[0].Winddir));
    WxConditions[0].Cloudcover  = root["clouds"]["all"].as<int>();               Serial.println("CCov: " + String(WxConditions[0].Cloudcover)); 
    WxConditions[0].Visibility  = root["visibility"].as<int>();                  Serial.println("Visi: " + String(WxConditions[0].Visibility)); 
    WxConditions[0].Rainfall    = root["rain"]["1h"].as<float>();                Serial.println("Rain: " + String(WxConditions[0].Rainfall));
    WxConditions[0].Snowfall    = root["snow"]["1h"].as<float>();                Serial.println("Snow: " + String(WxConditions[0].Snowfall));
    WxConditions[0].Sunrise     = root["sys"]["sunrise"].as<int>();              Serial.println("SRis: " + String(WxConditions[0].Sunrise));
    WxConditions[0].Sunset      = root["sys"]["sunset"].as<int>();               Serial.println("SSet: " + String(WxConditions[0].Sunset));
    WxConditions[0].Timezone    = root["timezone"].as<int>();                    Serial.println("TZon: " + String(WxConditions[0].Timezone));
  }
  if (Type == "forecast") {
    Serial.print(F("\nReceiving Forecast period - ")); 
    JsonArray list                  = root["list"];
    for (byte r = 0; r < max_readings; r++) {
      Serial.println("\nPeriod-" + String(r) + "--------------");
      WxForecast[r].Dt                = list[r]["dt"].as<int>();
      WxForecast[r].Temperature       = list[r]["main"]["temp"].as<float>();              Serial.println("Temp: " + String(WxForecast[r].Temperature));
      WxForecast[r].Low               = list[r]["main"]["temp_min"].as<float>();          Serial.println("TLow: " + String(WxForecast[r].Low));
      WxForecast[r].High              = list[r]["main"]["temp_max"].as<float>();          Serial.println("THig: " + String(WxForecast[r].High));
      WxForecast[r].Pressure          = list[r]["main"]["pressure"].as<float>();          Serial.println("Pres: " + String(WxForecast[r].Pressure));
      WxForecast[r].Humidity          = list[r]["main"]["humidity"].as<float>();          Serial.println("Humi: " + String(WxForecast[r].Humidity));
      WxForecast[r].Icon              = list[r]["weather"][0]["icon"].as<const char*>();        Serial.println("Icon: " + String(WxForecast[r].Icon));
      WxForecast[r].Rainfall          = list[r]["rain"]["3h"].as<float>();                Serial.println("Rain: " + String(WxForecast[r].Rainfall));
      WxForecast[r].Snowfall          = list[r]["snow"]["3h"].as<float>();                Serial.println("Snow: " + String(WxForecast[r].Snowfall));
      WxForecast[r].Period            = list[r]["dt_txt"].as<const char*>();                    Serial.println("Peri: " + String(WxForecast[r].Period));
    }

    float pressure_trend = WxForecast[0].Pressure - WxForecast[2].Pressure; 
    pressure_trend = ((int)(pressure_trend * 10)) / 10.0; 
    WxConditions[0].Trend = "=";
    if (pressure_trend > 0)  WxConditions[0].Trend = "+";
    if (pressure_trend < 0)  WxConditions[0].Trend = "-";
    if (pressure_trend == 0) WxConditions[0].Trend = "0";

    if (Units == "I") Convert_Readings_to_Imperial();
  }
  return true;
}

/**
 * @brief Преобразует время Unix Timestamp в отформатированную строку.
 * @param unix_time Время Unix.
 * @return Форматированная строка даты и времени.
 */
String ConvertUnixTime(int unix_time) {
  time_t tm = unix_time;
  struct tm *now_tm = localtime(&tm);
  char output[40];
  if (Units == "M") {
    strftime(output, sizeof(output), "%H:%M %d/%m/%y", now_tm);
  }
  else {
    strftime(output, sizeof(output), "%I:%M%P %m/%d/%y", now_tm);
  }
  return output;
}

/**
 * @brief Запрашивает данные с сервера OpenWeatherMap через HTTP GET.
 * @param client Клиент Wi-Fi.
 * @param RequestType Тип запроса ("weather" или "forecast").
 * @return true В случае успещного получения и декодирования данных.
 */
bool obtainWeatherData(WiFiClient & client, const String & RequestType) {
  const String units = (Units == "M" ? "metric" : "imperial");
  client.stop(); 
  HTTPClient http;
  String uri = "/data/2.5/" + RequestType + "?q=" + City + "," + Country + "&APPID=" + apikey + "&mode=json&units=" + units + "&lang=" + Language;
  if (RequestType != "weather")
  {
    uri += "&cnt=" + String(max_readings);
  }
  http.begin(client, server, 80, uri); 
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    if (!DecodeWeather(http.getStream(), RequestType)) return false;
    client.stop();
    http.end();
    return true;
  }
  else
  {
    Serial.printf("connection failed, error: %s", http.errorToString(httpCode).c_str());
    client.stop();
    http.end();
    return false;
  }
  http.end();
  return true;
}

/**
 * @brief Перевод миллиметров в дюймы.
 * @param value_mm Значение в мм.
 * @return Значение в дюймах.
 */
float mm_to_inches(float value_mm) {
  return 0.0393701 * value_mm;
}

/**
 * @brief Перевод гектопаскалей (hPa) в дюймы ртутного столба (inHg).
 * @param value_hPa Давление в hPa.
 * @return Давление в inHg.
 */
float hPa_to_inHg(float value_hPa) {
  return 0.02953 * value_hPa;
}

/**
 * @brief Вычисляет Юлианскую дату по календарным значениям.
 * @param d День.
 * @param m Месяц.
 * @param y Год.
 * @return Число Юлианского дня.
 */
int JulianDate(int d, int m, int y) {
  int mm, yy, k1, k2, k3, j;
  yy = y - (int)((12 - m) / 10);
  mm = m + 9;
  if (mm >= 12) mm = mm - 12;
  k1 = (int)(365.25 * (yy + 4712));
  k2 = (int)(30.6001 * mm + 0.5);
  k3 = (int)((int)((yy / 100) + 49) * 0.75) - 38;
  j = k1 + k2 + d + 59 + 1;
  if (j > 2299160) j = j - k3; 
  return j;
}

/**
 * @brief Суммирует значение элементов массива осадков.
 * @param DataArray Массив значений.
 * @param readings Количество элементов.
 * @return Сумма осадков.
 */
float SumOfPrecip(float DataArray[], int readings) {
  float sum = 0;
  for (int i = 0; i <= readings; i++) {
    sum += DataArray[i];
  }
  return sum;
}

/**
 * @brief Преобразует первый символ строки в верхний регистр.
 * @param text Исходный текст.
 * @return Преобразованный текст.
 */
String TitleCase(String text) {
  if (text.length() > 0) {
    String temp_text = text.substring(0, 1);
    temp_text.toUpperCase();
    return temp_text + text.substring(1); 
  }
  else return text;
}

/**
 * @brief Рассчитывает нормализованную фазу Луны (от 0.0 до 1.0).
 * @param d День.
 * @param m Месяц.
 * @param y Год.
 * @return Значение фазы от 0.0 до 1.0.
 */
double NormalizedMoonPhase(int d, int m, int y) {
  int j = JulianDate(d, m, y);
  double Phase = (j + 4.867) / 29.53059;
  return (Phase - (int) Phase);
}

/** @brief Отображает секцию с базовой информацией (Город, Дата, Время). */
void DisplayGeneralInfoSection() {
  setFont(OpenSans10B);
  drawString(5, 2, City, LEFT);
  setFont(OpenSans8B);
  drawString(500, 2, Date_str + "  @   " + Time_str, LEFT);
}

/**
 * @brief Отображает главную иконку погоды.
 * @param x Позиция X.
 * @param y Позиция Y.
 */
void DisplayWeatherIcon(int x, int y) {
  DisplayConditionsSection(x, y, WxConditions[0].Icon, LargeIcon);
}

/**
 * @brief Отображает главную секцию погоды (температура, текст прогноза, давление).
 * @param x Позиция X.
 * @param y Позиция Y.
 */
void DisplayMainWeatherSection(int x, int y) {
  setFont(OpenSans8B);
  DisplayTemperatureSection(x, y - 40);
  DisplayForecastTextSection(x - 55, y + 25);
  DisplayPressureSection(x - 25, y + 90, WxConditions[0].Pressure, WxConditions[0].Trend);
}

/**
 * @brief Отображает компас и направление ветра.
 * @param x Позиция X центра.
 * @param y Позиция Y центра.
 * @param angle Угол ветра в градусах.
 * @param windspeed Скорость ветра.
 * @param Cradius Радиус компаса.
 */
void DisplayDisplayWindSection(int x, int y, float angle, float windspeed, int Cradius) {
  arrow(x, y, Cradius - 22, angle, 18, 33); 
  setFont(OpenSans8B);
  int dxo, dyo, dxi, dyi;
  drawCircle(x, y, Cradius, Black);       
  drawCircle(x, y, Cradius + 1, Black);   
  drawCircle(x, y, Cradius * 0.7, Black); 
  for (float a = 0; a < 360; a = a + 22.5) {
    dxo = Cradius * cos((a - 90) * PI / 180);
    dyo = Cradius * sin((a - 90) * PI / 180);
    if (a == 45)  drawString(dxo + x + 15, dyo + y - 18, TXT_NE, CENTER);
    if (a == 135) drawString(dxo + x + 20, dyo + y - 2,  TXT_SE, CENTER);
    if (a == 225) drawString(dxo + x - 20, dyo + y - 2,  TXT_SW, CENTER);
    if (a == 315) drawString(dxo + x - 15, dyo + y - 18, TXT_NW, CENTER);
    dxi = dxo * 0.9;
    dyi = dyo * 0.9;
    drawLine(dxo + x, dyo + y, dxi + x, dyi + y, Black);
    dxo = dxo * 0.7;
    dyo = dyo * 0.7;
    dxi = dxo * 0.9;
    dyi = dyo * 0.9;
    drawLine(dxo + x, dyo + y, dxi + x, dyi + y, Black);
  }
  drawString(x, y - Cradius - 20,     TXT_N, CENTER);
  drawString(x, y + Cradius + 10,     TXT_S, CENTER);
  drawString(x - Cradius - 15, y - 5, TXT_W, CENTER);
  drawString(x + Cradius + 10, y - 5, TXT_E, CENTER);
  drawString(x + 3, y + 50, String(angle, 0) + "°", CENTER);
  setFont(OpenSans12B);
  drawString(x, y - 50, WindDegToOrdinalDirection(angle), CENTER);
  setFont(OpenSans24B);
  drawString(x + 3, y - 18, String(windspeed, 1), CENTER);
  setFont(OpenSans12B);
  drawString(x, y + 25, (Units == "M" ? "m/s" : "mph"), CENTER);
}

/**
 * @brief Преобразует градусы направления ветра в текстовый румб (N, NE, E и т.д.).
 * @param winddirection Угол направления ветра.
 * @return Текстовое обозначение направления.
 */
String WindDegToOrdinalDirection(float winddirection) {
  if (winddirection >= 348.75 || winddirection < 11.25)  return TXT_N;
  if (winddirection >=  11.25 && winddirection < 33.75)  return TXT_NNE;
  if (winddirection >=  33.75 && winddirection < 56.25)  return TXT_NE;
  if (winddirection >=  56.25 && winddirection < 78.75)  return TXT_ENE;
  if (winddirection >=  78.75 && winddirection < 101.25) return TXT_E;
  if (winddirection >= 101.25 && winddirection < 123.75) return TXT_ESE;
  if (winddirection >= 123.75 && winddirection < 146.25) return TXT_SE;
  if (winddirection >= 146.25 && winddirection < 168.75) return TXT_SSE;
  if (winddirection >= 168.75 && winddirection < 191.25) return TXT_S;
  if (winddirection >= 191.25 && winddirection < 213.75) return TXT_SSW;
  if (winddirection >= 213.75 && winddirection < 236.25) return TXT_SW;
  if (winddirection >= 236.25 && winddirection < 258.75) return TXT_WSW;
  if (winddirection >= 258.75 && winddirection < 281.25) return TXT_W;
  if (winddirection >= 281.25 && winddirection < 303.75) return TXT_WNW;
  if (winddirection >= 303.75 && winddirection < 326.25) return TXT_NW;
  if (winddirection >= 326.25 && winddirection < 348.75) return TXT_NNW;
  return "?";
}

/**
 * @brief Отображает секцию текущей температуры и влажности.
 * @param x Позиция X.
 * @param y Позиция Y.
 */
void DisplayTemperatureSection(int x, int y) {
  setFont(OpenSans12B);
  if (sensors_available && local_temperature > -50) {
    drawString(x - 10, y + 25, String(local_temperature, 1) + "° | " + String(local_humidity, 0) + "% | " + String(local_pressure , 0) + " hPa", LEFT);
    setFont(OpenSans12B);
    drawString(x - 10, y, "LOCAL (INDOOR)", LEFT);
  } 

  setFont(OpenSans12B);
  drawString(x + 330, y + 36, String(WxConditions[0].High, 0) + "° | " + String(WxConditions[0].Low, 0) + "°", CENTER); 

  // OUTDOOR (ESP-NOW) - показуємо поруч, якщо дані від зовнішнього сенсора вже надходили хоч раз
  if (extendet_timestamp > 0) {
    setFont(OpenSans12B);
    drawString(x - 10, y + 65, "OUTDOOR", LEFT);
    setFont(OpenSans12B);
    drawString(x - 10, y + 85, String(extendet_temperature, 1) + "° | "  + String(extendet_humidity, 0) + "%", LEFT);
  }
}

/**
 * @brief Отображает текстовую описательную часть прогноза погоды.
 * @param x Позиция X.
 * @param y Позиция Y.
 */
void DisplayForecastTextSection(int x, int y) {
#define lineWidth 34
  setFont(OpenSans12B);
  String Wx_Description = WxConditions[0].Forecast0; 
  Wx_Description.replace(".", ""); 
  int spaceRemaining = 0, p = 0, charCount = 0, Width = lineWidth;
  while (p < Wx_Description.length()) {
    if (Wx_Description.substring(p, p + 1) == " ") spaceRemaining = p;
    if (charCount > Width - 1) { 
      Wx_Description = Wx_Description.substring(0, spaceRemaining) + "~" + Wx_Description.substring(spaceRemaining + 1);
      charCount = 0;
    }
    p++;
    charCount++;
  }
  if (WxForecast[0].Rainfall > 0) Wx_Description += " (" + String(WxForecast[0].Rainfall, 1) + String((Units == "M" ? "mm" : "in")) + ")";
  String Line1 = Wx_Description.substring(0, Wx_Description.indexOf("~"));
  String Line2 = Wx_Description.substring(Wx_Description.indexOf("~") + 1);
  drawString(x + 30, y + 5, TitleCase(Line1), LEFT);
  if (Line1 != Line2) drawString(x + 30, y + 30, Line2, LEFT);
}

/**
 * @brief Отображает давление и тренд давления, а также видимость и облачность.
 * @param x Позиция X.
 * @param y Позиция Y.
 * @param pressure Значение давления.
 * @param slope Символ тренда ("+", "-", "0").
 */
void DisplayPressureSection(int x, int y, float pressure, String slope) {
  setFont(OpenSans12B);
  if (sensors_available && local_pressure > 0) {
    DrawPressureAndTrend(x - 25, y + 10, local_pressure, slope);
  } else {
    DrawPressureAndTrend(x - 25, y + 10, pressure, slope);
  }
  if (extendet_timestamp > 0) {
    setFont(OpenSans8B);
    drawString(x - 25, y + 45, "OUTDOOR: " + String(extendet_pressure / 100.0f, 1) + " hPa", LEFT);
    setFont(OpenSans12B);
  }
  if (WxConditions[0].Visibility > 0) {
    Visibility(x + 145, y, String(WxConditions[0].Visibility) + "M");
    x += 150; 
  }
  if (WxConditions[0].Cloudcover > 0) CloudCover(x + 145, y, WxConditions[0].Cloudcover);
}

/**
 * @brief Рисует ячейку элемента прогноза погоды по индексу.
 * @param x Позиция X.
 * @param y Позиция Y.
 * @param index Индекс массива прогноза.
 */
void DisplayForecastWeather(int x, int y, int index) {
  int fwidth = 90;
  x = x + fwidth * index;
  DisplayConditionsSection(x + fwidth / 2, y + 90, WxForecast[index].Icon, SmallIcon);
  setFont(OpenSans10B);
  drawString(x + fwidth / 2, y + 30, String(ConvertUnixTime(WxForecast[index].Dt + WxConditions[0].Timezone).substring(0, 5)), CENTER);
  drawString(x + fwidth / 2, y + 125, String(WxForecast[index].High, 0) + "°/" + String(WxForecast[index].Low, 0) + "°", CENTER);
}

/**
 * @brief Отображает данные астрономии (рассвет, закат, фаза Луны).
 * @param x Позиция X.
 * @param y Позиция Y.
 */
void DisplayAstronomySection(int x, int y) {
  setFont(OpenSans10B);
  drawString(x + 5, y + 30, ConvertUnixTime(WxConditions[0].Sunrise).substring(0, 5) + " " + TXT_SUNRISE, LEFT);
  drawString(x + 5, y + 50, ConvertUnixTime(WxConditions[0].Sunset).substring(0, 5) + " " + TXT_SUNSET, LEFT);
  time_t now = time(NULL);
  struct tm * now_utc  = gmtime(&now);
  const int day_utc    = now_utc->tm_mday;
  const int month_utc  = now_utc->tm_mon + 1;
  const int year_utc   = now_utc->tm_year + 1900;
  drawString(x + 5, y + 70, MoonPhase(day_utc, month_utc, year_utc, Hemisphere), LEFT);
  DrawMoon(x + 160, y - 15, day_utc, month_utc, year_utc, Hemisphere);
}

/**
 * @brief Отрисовывает графическое изображение Луны с фазой.
 * @param x Позиция X.
 * @param y Позиция Y.
 * @param dd День.
 * @param mm Месяц.
 * @param yy Год.
 * @param hemisphere Полушарие ("north" / "south").
 */
void DrawMoon(int x, int y, int dd, int mm, int yy, String hemisphere) {
  const int diameter = 75;
  double Phase = NormalizedMoonPhase(dd, mm, yy);
  hemisphere.toLowerCase();
  if (hemisphere == "south") Phase = 1 - Phase;

  fillCircle(x + diameter - 1, y + diameter, diameter / 2 + 1, LightGrey);
  const int number_of_lines = 90;
  for (double Ypos = 0; Ypos <= number_of_lines / 2; Ypos++) {
    double Xpos = sqrt(number_of_lines / 2 * number_of_lines / 2 - Ypos * Ypos);
    double Rpos = 2 * Xpos;
    double Xpos1, Xpos2;
    if (Phase < 0.5) {
      Xpos1 = -Xpos;
      Xpos2 = Rpos - 2 * Phase * Rpos - Xpos;
    }
    else {
      Xpos1 = Xpos;
      Xpos2 = Xpos - 2 * Phase * Rpos + Rpos;
    }

    double pW1x = (Xpos1 + number_of_lines) / number_of_lines * diameter + x;
    double pW1y = (number_of_lines - Ypos)  / number_of_lines * diameter + y;
    double pW2x = (Xpos2 + number_of_lines) / number_of_lines * diameter + x;
    double pW2y = (number_of_lines - Ypos)  / number_of_lines * diameter + y;
    double pW3x = (Xpos1 + number_of_lines) / number_of_lines * diameter + x;
    double pW3y = (Ypos + number_of_lines)  / number_of_lines * diameter + y;
    double pW4x = (Xpos2 + number_of_lines) / number_of_lines * diameter + x;
    double pW4y = (Ypos + number_of_lines)  / number_of_lines * diameter + y;
    drawLine(pW1x, pW1y, pW2x, pW2y, White);
    drawLine(pW3x, pW3y, pW4x, pW4y, White);
  }
  drawCircle(x + diameter - 1, y + diameter, diameter / 2, Black);
}

/**
 * @brief Возвращает название текущей фазы Луны.
 * @param d День.
 * @param m Месяц.
 * @param y Год.
 * @param hemisphere Полушарие.
 * @return Текстовое описание фазы Луны.
 */
String MoonPhase(int d, int m, int y, String hemisphere) {
  int c, e;
  double jd;
  int b;
  if (m < 3) {
    y--;
    m += 12;
  }
  ++m;
  c   = 365.25 * y;
  e   = 30.6  * m;
  jd  = c + e + d - 694039.09;     
  jd /= 29.53059;                        
  b   = jd;                              
  jd -= b;                               
  b   = jd * 8 + 0.5;                
  b   = b & 7;                           
  if (hemisphere == "south") b = 7 - b;
  if (b == 0) return TXT_MOON_NEW;              
  if (b == 1) return TXT_MOON_WAXING_CRESCENT;  
  if (b == 2) return TXT_MOON_FIRST_QUARTER;    
  if (b == 3) return TXT_MOON_WAXING_GIBBOUS;   
  if (b == 4) return TXT_MOON_FULL;             
  if (b == 5) return TXT_MOON_WANING_GIBBOUS;   
  if (b == 6) return TXT_MOON_THIRD_QUARTER;    
  if (b == 7) return TXT_MOON_WANING_CRESCENT;  
  return "";
}

/**
 * @brief Отображает секцию прогнозов по часам и графики погоды.
 * @param x Позиция X.
 * @param y Позиция Y.
 */
void DisplayForecastSection(int x, int y) {
  int f = 0;
  do {
    DisplayForecastWeather(x, y, f);
    f++;
  } while (f < max_readings);
  int r = 0;
  do { 
    if (Units == "I") pressure_readings[r] = WxForecast[r].Pressure * 0.02953;   else pressure_readings[r] = WxForecast[r].Pressure;
    if (Units == "I") rain_readings[r]     = WxForecast[r].Rainfall * 0.0393701; else rain_readings[r]     = WxForecast[r].Rainfall;
    if (Units == "I") snow_readings[r]     = WxForecast[r].Snowfall * 0.0393701; else snow_readings[r]     = WxForecast[r].Snowfall;
    temperature_readings[r]                = WxForecast[r].Temperature;
    humidity_readings[r]                   = WxForecast[r].Humidity;
    r++;
  } while (r < max_readings);
  int gwidth = 175, gheight = 100;
  int gx = (SCREEN_WIDTH - gwidth * 4) / 5 + 8;
  int gy = (SCREEN_HEIGHT - gheight - 30);
  int gap = gwidth + gx;

  DrawGraph(gx + 0 * gap, gy, gwidth, gheight, 900, 1050, Units == "M" ? TXT_PRESSURE_HPA : TXT_PRESSURE_IN, pressure_readings, max_readings, autoscale_on, barchart_off);
  DrawGraph(gx + 1 * gap, gy, gwidth, gheight, 10, 30,    Units == "M" ? TXT_TEMPERATURE_C : TXT_TEMPERATURE_F, temperature_readings, max_readings, autoscale_on, barchart_off);
  DrawGraph(gx + 2 * gap, gy, gwidth, gheight, 0, 100,   TXT_HUMIDITY_PERCENT, humidity_readings, max_readings, autoscale_off, barchart_off);
  if (SumOfPrecip(rain_readings, max_readings) >= SumOfPrecip(snow_readings, max_readings))
    DrawGraph(gx + 3 * gap + 5, gy, gwidth, gheight, 0, 30, Units == "M" ? TXT_RAINFALL_MM : TXT_RAINFALL_IN, rain_readings, max_readings, autoscale_on, barchart_on);
  else
    DrawGraph(gx + 3 * gap + 5, gy, gwidth, gheight, 0, 30, Units == "M" ? TXT_SNOWFALL_MM : TXT_SNOWFALL_IN, snow_readings, max_readings, autoscale_on, barchart_on);
}

/**
 * @brief Маршрутизирует отрисовку соответствующей иконки погоды по коду OpenWeather.
 * @param x Позиция X.
 * @param y Позиция Y.
 * @param IconName Имя иконки (например, "01d").
 * @param IconSize Флаг размера (LargeIcon или SmallIcon).
 */
void DisplayConditionsSection(int x, int y, String IconName, bool IconSize) {
  Serial.println("Icon name: " + IconName);
  if      (IconName == "01d" || IconName == "01n")  Sunny(x, y, IconSize, IconName);
  else if (IconName == "02d" || IconName == "02n")  MostlySunny(x, y, IconSize, IconName);
  else if (IconName == "03d" || IconName == "03n")  Cloudy(x, y, IconSize, IconName);
  else if (IconName == "04d" || IconName == "04n")  MostlySunny(x, y, IconSize, IconName);
  else if (IconName == "09d" || IconName == "09n")  ChanceRain(x, y, IconSize, IconName);
  else if (IconName == "10d" || IconName == "10n")  Rain(x, y, IconSize, IconName);
  else if (IconName == "11d" || IconName == "11n")  Tstorms(x, y, IconSize, IconName);
  else if (IconName == "13d" || IconName == "13n")  Snow(x, y, IconSize, IconName);
  else if (IconName == "50d")                       Haze(x, y, IconSize, IconName);
  else if (IconName == "50n")                       Fog(x, y, IconSize, IconName);
  else                                              Nodata(x, y, IconSize, IconName);
}

/**
 * @brief Вспомогательная функция для рисования стрелки (для компаса).
 */
void arrow(int x, int y, int asize, float aangle, int pwidth, int plength) {
  float dx = (asize - 10) * cos((aangle - 90) * PI / 180) + x; 
  float dy = (asize - 10) * sin((aangle - 90) * PI / 180) + y; 
  float x1 = 0;         float y1 = plength;
  float x2 = pwidth / 2;  float y2 = pwidth / 2;
  float x3 = -pwidth / 2; float y3 = pwidth / 2;
  float angle = aangle * PI / 180 - 135;
  float xx1 = x1 * cos(angle) - y1 * sin(angle) + dx;
  float yy1 = y1 * cos(angle) + x1 * sin(angle) + dy;
  float xx2 = x2 * cos(angle) - y2 * sin(angle) + dx;
  float yy2 = y2 * cos(angle) + x2 * sin(angle) + dy;
  float xx3 = x3 * cos(angle) - y3 * sin(angle) + dx;
  float yy3 = y3 * cos(angle) + x3 * sin(angle) + dy;
  fillTriangle(xx1, yy1, xx3, yy3, xx2, yy2, Black);
}

/** @brief Вспомогательная функция отрисовки сегмента линии. */
void DrawSegment(int x, int y, int o1, int o2, int o3, int o4, int o11, int o12, int o13, int o14) {
  drawLine(x + o1,  y + o2,  x + o3,  y + o4,  Black);
  drawLine(x + o11, y + o12, x + o13, y + o14, Black);
}

/** @brief Рисует значение давления и стрелку его изменения (тренд). */
void DrawPressureAndTrend(int x, int y, float pressure, String slope) {
  drawString(x + 25, y - 10, String(pressure, (Units == "M" ? 0 : 1)) + (Units == "M" ? "hPa" : "in"), LEFT);
  if      (slope == "+") {
    DrawSegment(x, y, 0, 0, 8, -8, 8, -8, 16, 0);
    DrawSegment(x - 1, y, 0, 0, 8, -8, 8, -8, 16, 0);
  }
  else if (slope == "0") {
    DrawSegment(x, y, 8, -8, 16, 0, 8, 8, 16, 0);
    DrawSegment(x - 1, y, 8, -8, 16, 0, 8, 8, 16, 0);
  }
  else if (slope == "-") {
    DrawSegment(x, y, 0, 0, 8, 8, 8, 8, 16, 0);
    DrawSegment(x - 1, y, 0, 0, 8, 8, 8, 8, 16, 0);
  }
}

/** @brief Рисует блок статуса (Wi-Fi сигнал, заряд батареи). */
void DisplayStatusSection(int x, int y, int rssi) {
  setFont(OpenSans8B);
  DrawRSSI(x + 305, y + 15, rssi);
  DrawBattery(x + 150, y);
}

/** @brief Рисует иконку уровня Wi-Fi сигнала. */
void DrawRSSI(int x, int y, int rssi) {
  int WIFIsignal = 0;
  int xpos = 1;
  for (int _rssi = -100; _rssi <= rssi; _rssi = _rssi + 20) {
    if (_rssi <= -20)  WIFIsignal = 30; 
    if (_rssi <= -40)  WIFIsignal = 24; 
    if (_rssi <= -60)  WIFIsignal = 18; 
    if (_rssi <= -80)  WIFIsignal = 12; 
    if (_rssi <= -100) WIFIsignal = 6;  
    fillRect(x + xpos * 8, y - WIFIsignal, 6, WIFIsignal, Black);
    xpos++;
  }
}

/**
 * @brief Обновляет локальное системное время из NTP или фолбэка DS3231.
 * @return true В случае успеха, false если время не найдено.
 */
boolean UpdateLocalTime() {
  struct tm timeinfo;
  char   time_output[30], day_output[30], update_time[30];
  if (!getLocalTime(&timeinfo, 5000)) { 
    Serial.println("Failed to obtain time from NTP - пробуємо запасне джерело DS3231...");
    if (DS3231_GetTime(&timeinfo)) {
      time_t t = mktime(&timeinfo);
      struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
      settimeofday(&tv, NULL);
      Serial.println("ESP-NOW/час: відновлено з DS3231 (NTP недоступний цього циклу)");
    } else {
      Serial.println("DS3231 теж недоступний - часу нема");
      return false;
    }
  } else {
    DS3231_SetTime(&timeinfo);
  }
  CurrentHour = timeinfo.tm_hour;
  CurrentMin  = timeinfo.tm_min;
  CurrentSec  = timeinfo.tm_sec;
  Serial.println(&timeinfo, "%a %b %d %Y   %H:%M:%S");      
  if (Units == "M") {
    sprintf(day_output, "%s, %02u %s %04u", weekday_D[timeinfo.tm_wday], timeinfo.tm_mday, month_M[timeinfo.tm_mon], (timeinfo.tm_year) + 1900);
    strftime(update_time, sizeof(update_time), "%H:%M:%S", &timeinfo);  
    sprintf(time_output, "%s", update_time);
  }
  else
  {
    strftime(day_output, sizeof(day_output), "%a %b-%d-%Y", &timeinfo); 
    strftime(update_time, sizeof(update_time), "%r", &timeinfo);        
    sprintf(time_output, "%s", update_time);
  }
  Date_str = day_output;
  Time_str = time_output;
  return true;
}

/** @brief Рисует значок и процент встроенной батареи устройства. */
void DrawBattery(int x, int y) {
  uint8_t percentage = 100;
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
    vref = adc_chars.vref;
  }
  float voltage = analogRead(36) / 4096.0 * 6.566 * (vref / 1000.0);
  if (voltage > 1 ) { 
    Serial.println("\nVoltage = " + String(voltage));
    percentage = 2836.9625 * pow(voltage, 4) - 43987.4889 * pow(voltage, 3) + 255233.8134 * pow(voltage, 2) - 656689.7123 * voltage + 632041.7303;
    if (voltage >= 4.20) percentage = 100;
    if (voltage <= 3.20) percentage = 0;  
    drawRect(x + 25, y - 14, 40, 15, Black);
    fillRect(x + 65, y - 10, 4, 7, Black);
    fillRect(x + 27, y - 12, 36 * percentage / 100.0, 11, Black);
    drawString(x + 85, y - 14, String(percentage) + "%  " + String(voltage, 1) + "v", LEFT);
  }
}

/** @brief Функция рисовки облака на иконках погоды. */
void addcloud(int x, int y, int scale, int linesize) {
  fillCircle(x - scale * 3, y, scale, Black);                                                              
  fillCircle(x + scale * 3, y, scale, Black);                                                              
  fillCircle(x - scale, y - scale, scale * 1.4, Black);                                                    
  fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75, Black);                                       
  fillRect(x - scale * 3 - 1, y - scale, scale * 6, scale * 2 + 1, Black);                                 
  fillCircle(x - scale * 3, y, scale - linesize, White);                                                   
  fillCircle(x + scale * 3, y, scale - linesize, White);                                                   
  fillCircle(x - scale, y - scale, scale * 1.4 - linesize, White);                                         
  fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75 - linesize, White);                            
  fillRect(x - scale * 3 + 2, y - scale + linesize - 1, scale * 5.9, scale * 2 - linesize * 2 + 2, White); 
}

/** @brief Функция рисовки дождя на иконках погоды. */
void addrain(int x, int y, int scale, bool IconSize) {
  if (IconSize == SmallIcon) {
    setFont(OpenSans8B);
    drawString(x - 25, y + 12, "///////", LEFT);
  }
  else
  {
    setFont(OpenSans18B);
    drawString(x - 60, y + 25, "///////", LEFT);
  }
}

/** @brief Функция рисовки снега на иконках погоды. */
void addsnow(int x, int y, int scale, bool IconSize) {
  if (IconSize == SmallIcon) {
    setFont(OpenSans8B);
    drawString(x - 25, y + 15, "* * * *", LEFT);
  }
  else
  {
    setFont(OpenSans18B);
    drawString(x - 60, y + 30, "* * * *", LEFT);
  }
}

/** @brief Функция рисовки грозы на иконках погоды. */
void addtstorm(int x, int y, int scale) {
  y = y + scale / 2;
  for (int i = 0; i < 5; i++) {
    drawLine(x - scale * 4 + scale * i * 1.5 + 0, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 0, y + scale, Black);
    if (scale != Small) {
      drawLine(x - scale * 4 + scale * i * 1.5 + 1, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 1, y + scale, Black);
      drawLine(x - scale * 4 + scale * i * 1.5 + 2, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 2, y + scale, Black);
    }
    drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 0, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 0, Black);
    if (scale != Small) {
      drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 1, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 1, Black);
      drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 2, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 2, Black);
    }
    drawLine(x - scale * 3.5 + scale * i * 1.4 + 0, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5, Black);
    if (scale != Small) {
      drawLine(x - scale * 3.5 + scale * i * 1.4 + 1, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 1, y + scale * 1.5, Black);
      drawLine(x - scale * 3.5 + scale * i * 1.4 + 2, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 2, y + scale * 1.5, Black);
    }
  }
}

/** @brief Функция рисовки солнца на иконках погоды. */
void addsun(int x, int y, int scale, bool IconSize) {
  int linesize = 5;
  fillRect(x - scale * 2, y, scale * 4, linesize, Black);
  fillRect(x, y - scale * 2, linesize, scale * 4, Black);
  drawLine(x - scale * 1.3, y - scale * 1.3, x + scale * 1.3, y + scale * 1.3, Black);
  drawLine(x - scale * 1.3, y + scale * 1.3, x + scale * 1.3, y - scale * 1.3, Black);
  if (IconSize == LargeIcon) {
    drawLine(1 + x - scale * 1.3, y - scale * 1.3, 1 + x + scale * 1.3, y + scale * 1.3, Black);
    drawLine(2 + x - scale * 1.3, y - scale * 1.3, 2 + x + scale * 1.3, y + scale * 1.3, Black);
    drawLine(3 + x - scale * 1.3, y - scale * 1.3, 3 + x + scale * 1.3, y + scale * 1.3, Black);
    drawLine(1 + x - scale * 1.3, y + scale * 1.3, 1 + x + scale * 1.3, y - scale * 1.3, Black);
    drawLine(2 + x - scale * 1.3, y + scale * 1.3, 2 + x + scale * 1.3, y - scale * 1.3, Black);
    drawLine(3 + x - scale * 1.3, y + scale * 1.3, 3 + x + scale * 1.3, y - scale * 1.3, Black);
  }
  fillCircle(x, y, scale * 1.3, White);
  fillCircle(x, y, scale, Black);
  fillCircle(x, y, scale - linesize, White);
}

/** @brief Функция рисовки тумана на иконках погоды. */
void addfog(int x, int y, int scale, int linesize, bool IconSize) {
  if (IconSize == SmallIcon) {
    y -= 10;
    linesize = 1;
  }
  for (int i = 0; i < 6; i++) {
    fillRect(x - scale * 3, y + scale * 1.5, scale * 6, linesize, Black);
    fillRect(x - scale * 3, y + scale * 2.0, scale * 6, linesize, Black);
    fillRect(x - scale * 3, y + scale * 2.5, scale * 6, linesize, Black);
  }
}

/** @brief Отрисовка состояния погоды: Ясно / Солнечно. */
void Sunny(int x, int y, bool IconSize, String IconName) {
  int scale = Small, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  else y = y - 3; 
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  scale = scale * 1.6;
  addsun(x, y, scale, IconSize);
}

/** @brief Отрисовка состояния погоды: Малооблачно. */
void MostlySunny(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
  addcloud(x, y, scale, linesize);
}

/** @brief Отрисовка состояния погоды: Облачно с прояснениями. */
void MostlyCloudy(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addcloud(x, y, scale, linesize);
  addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
}

/** @brief Отрисовка состояния погоды: Пасмурно. */
void Cloudy(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addcloud(x + 15, y - 22, scale / 2, linesize); 
  addcloud(x - 10, y - 18, scale / 2, linesize); 
  addcloud(x, y, scale, linesize);             
}

/** @brief Отрисовка состояния погоды: Дождь. */
void Rain(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addcloud(x, y, scale, linesize);
  addrain(x, y, scale, IconSize);
}

/** @brief Отрисовка состояния погоды: Ожидается дождь. */
void ExpectRain(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
  addcloud(x, y, scale, linesize);
  addrain(x, y, scale, IconSize);
}

/** @brief Отрисовка состояния погоды: Возможен дождь. */
void ChanceRain(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
  addcloud(x, y, scale, linesize);
  addrain(x, y, scale, IconSize);
}

/** @brief Отрисовка состояния погоды: Гроза. */
void Tstorms(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addcloud(x, y, scale, linesize);
  addtstorm(x, y, scale);
}

/** @brief Отрисовка состояния погоды: Снег. */
void Snow(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addcloud(x, y, scale, linesize);
  addsnow(x, y, scale, IconSize);
}

/** @brief Отрисовка состояния погоды: Туман. */
void Fog(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addcloud(x, y - 5, scale, linesize);
  addfog(x, y - 5, scale, linesize, IconSize);
}

/** @brief Отрисовка состояния погоды: Дымка/Мгла. */
void Haze(int x, int y, bool IconSize, String IconName) {
  int scale = Small, linesize = 5, Offset = 10;
  if (IconSize == LargeIcon) {
    scale = Large;
    Offset = 35;
  }
  if (IconName.endsWith("n")) addmoon(x, y + Offset, scale, IconSize);
  addsun(x, y - 5, scale * 1.4, IconSize);
  addfog(x, y - 5, scale * 1.4, linesize, IconSize);
}

/** @brief Рисует процент облачности. */
void CloudCover(int x, int y, int CCover) {
  addcloud(x - 9, y + 2, Small * 0.3, 2); 
  addcloud(x + 3, y - 2, Small * 0.3, 2); 
  addcloud(x, y + 10, Small * 0.6, 2); 
  drawString(x + 20, y, String(CCover) + "%", LEFT);
}

/** @brief Рисует значок и показатель дальности видимости. */
void Visibility(int x, int y, String Visi) {
  float start_angle = 0.52, end_angle = 2.61, Offset = 8;
  int r = 14;
  for (float i = start_angle; i < end_angle; i = i + 0.05) {
    drawPixel(x + r * cos(i), y - r / 2 + r * sin(i) + Offset, Black);
    drawPixel(x + r * cos(i), 1 + y - r / 2 + r * sin(i) + Offset, Black);
  }
  start_angle = 3.61; end_angle = 5.78;
  for (float i = start_angle; i < end_angle; i = i + 0.05) {
    drawPixel(x + r * cos(i), y + r / 2 + r * sin(i) + Offset, Black);
    drawPixel(x + r * cos(i), 1 + y + r / 2 + r * sin(i) + Offset, Black);
  }
  fillCircle(x, y + Offset, r / 4, Black);
  drawString(x + 20, y, Visi, LEFT);
}

/** @brief Функция добавления ночного полумесяца к иконке погоды. */
void addmoon(int x, int y, int scale, bool IconSize) {
  if (IconSize == LargeIcon) {
    fillCircle(x - 85, y - 100, uint16_t(scale * 0.8), Black);
    fillCircle(x - 57, y - 100, uint16_t(scale * 1.6), White);
  }
  else
  {
    fillCircle(x - 28, y - 37, uint16_t(scale * 1.0), Black);
    fillCircle(x - 20, y - 37, uint16_t(scale * 1.6), White);
  }
}

/** @brief Отрисовка символа отсутствия данных. */
void Nodata(int x, int y, bool IconSize, String IconName) {
  if (IconSize == LargeIcon) setFont(OpenSans24B); else setFont(OpenSans12B);
  drawString(x - 3, y - 10, "?", CENTER);
}

/**
 * @brief Отрисовывает графики результатов измерений погоды.
 * @param x_pos Позиция начальная по X.
 * @param y_pos Позиция начальная по Y.
 * @param gwidth Ширина графика.
 * @param gheight Высота графика.
 * @param Y1Min Минимальное значение Y.
 * @param Y1Max Максимальное значение Y.
 * @param title Заголовок графика.
 * @param DataArray Массив данных значений.
 * @param readings Количество значений массива.
 * @param auto_scale Включение авто масштабирования оси Y.
 * @param barchart_mode Отображать в виде гистограммы (столбцов).
 */
void DrawGraph(int x_pos, int y_pos, int gwidth, int gheight, float Y1Min, float Y1Max, String title, float DataArray[], int readings, boolean auto_scale, boolean barchart_mode) {
#define auto_scale_margin 0 
#define y_minor_axis 5      
  setFont(OpenSans10B);
  int maxYscale = -10000;
  int minYscale =  10000;
  int last_x, last_y;
  float x2, y2;
  if (auto_scale == true) {
    for (int i = 1; i < readings; i++ ) {
      if (DataArray[i] >= maxYscale) maxYscale = DataArray[i];
      if (DataArray[i] <= minYscale) minYscale = DataArray[i];
    }
    maxYscale = round(maxYscale + auto_scale_margin); 
    Y1Max = round(maxYscale + 0.5);
    if (minYscale != 0) minYscale = round(minYscale - auto_scale_margin); 
    Y1Min = round(minYscale);
  }

  last_x = x_pos + 1;
  last_y = y_pos + (Y1Max - constrain(DataArray[1], Y1Min, Y1Max)) / (Y1Max - Y1Min) * gheight;
  drawRect(x_pos, y_pos, gwidth + 3, gheight + 2, Grey);
  drawString(x_pos - 20 + gwidth / 2, y_pos - 28, title, CENTER);
  for (int gx = 0; gx < readings; gx++) {
    x2 = x_pos + gx * gwidth / (readings - 1) - 1 ; 
    y2 = y_pos + (Y1Max - constrain(DataArray[gx], Y1Min, Y1Max)) / (Y1Max - Y1Min) * gheight + 1;
    if (barchart_mode) {
      fillRect(last_x + 2, y2, (gwidth / readings) - 1, y_pos + gheight - y2 + 2, Black);
    } else {
      drawLine(last_x, last_y - 1, x2, y2 - 1, Black); 
      drawLine(last_x, last_y, x2, y2, Black);
    }
    last_x = x2;
    last_y = y2;
  }

#define number_of_dashes 20
  for (int spacing = 0; spacing <= y_minor_axis; spacing++) {
    for (int j = 0; j < number_of_dashes; j++) { 
      if (spacing < y_minor_axis) drawFastHLine((x_pos + 3 + j * gwidth / number_of_dashes), y_pos + (gheight * spacing / y_minor_axis), gwidth / (2 * number_of_dashes), Grey);
    }
    if ((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing) < 5 || title == TXT_PRESSURE_IN) {
      drawString(x_pos - 10, y_pos + gheight * spacing / y_minor_axis - 5, String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 1), RIGHT);
    }
    else
    {
      if (Y1Min < 1 && Y1Max < 10) {
        drawString(x_pos - 3, y_pos + gheight * spacing / y_minor_axis - 5, String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 1), RIGHT);
      }
      else {
        drawString(x_pos - 7, y_pos + gheight * spacing / y_minor_axis - 5, String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 0), RIGHT);
      }
    }
  }
  for (int i = 0; i < 3; i++) {
    drawString(20 + x_pos + gwidth / 3 * i, y_pos + gheight + 10, String(i) + "d", LEFT);
    if (i < 2) drawFastVLine(x_pos + gwidth / 3 * i + gwidth / 3, y_pos, gheight, LightGrey);
  }
}

/**
 * @brief Отрисовывает текстовую строку в фреймбуфере дисплея.
 * @param x Позиция X.
 * @param y Позиция Y.
 * @param text Строка для печати.
 * @param align Тип выравнивания текста (LEFT, CENTER, RIGHT).
 */
void drawString(int x, int y, String text, alignment align) {
  char * data  = const_cast<char*>(text.c_str());
  int  x1, y1; 
  int w, h;
  int xx = x, yy = y;
  get_text_bounds(&currentFont, data, &xx, &yy, &x1, &y1, &w, &h, NULL);
  if (align == RIGHT)  x = x - w;
  if (align == CENTER) x = x - w / 2;
  int cursor_y = y + h;
  write_string(&currentFont, data, &x, &cursor_y, framebuffer);
}

/** @brief Рисует закрашенный круг в буфер кадров EPD. */
void fillCircle(int x, int y, int r, uint8_t color) {
  epd_fill_circle(x, y, r, color, framebuffer);
}

/** @brief Рисует быструю горизонтальную линию в буфер кадров EPD. */
void drawFastHLine(int16_t x0, int16_t y0, int length, uint16_t color) {
  epd_draw_hline(x0, y0, length, color, framebuffer);
}

/** @brief Рисует быструю вертикальную линию в буфер кадров EPD. */
void drawFastVLine(int16_t x0, int16_t y0, int length, uint16_t color) {
  epd_draw_vline(x0, y0, length, color, framebuffer);
}

/** @brief Рисует произвольную линию в буфер кадров EPD. */
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
  epd_write_line(x0, y0, x1, y1, color, framebuffer);
}

/** @brief Рисует контур окружности в буфер кадров EPD. */
void drawCircle(int x0, int y0, int r, uint8_t color) {
  epd_draw_circle(x0, y0, r, color, framebuffer);
}

/** @brief Рисует контур прямоугольника в буфер кадров EPD. */
void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  epd_draw_rect(x, y, w, h, color, framebuffer);
}

/** @brief Рисует закрашенный прямоугольник в буфер кадров EPD. */
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  epd_fill_rect(x, y, w, h, color, framebuffer);
}

/** @brief Рисует закрашенный треугольник в буфер кадров EPD. */
void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                  int16_t x2, int16_t y2, uint16_t color) {
  epd_fill_triangle(x0, y0, x1, y1, x2, y2, color, framebuffer);
}

/** @brief Рисует один пиксель в буфер кадров EPD. */
void drawPixel(int x, int y, uint8_t color) {
  epd_draw_pixel(x, y, color, framebuffer);
}

/** @brief Устанавливает шрифт для отрисовки текста. */
void setFont(GFXfont const &font) {
  currentFont = font;
}

/** @brief Отправляет содержимое фреймбуфера на EPD дисплей для физического обновления. */
void edp_update() {
  epd_draw_grayscale_image(epd_full_screen(), framebuffer); 
}

/** @brief Формирует главный экран (Страница 0) с онлайн прогнозом погоды. */
void DisplayWeather() {                          
  DisplayStatusSection(600, 20, wifi_signal);    
  DisplayGeneralInfoSection();                   
  DisplayDisplayWindSection(137, 150, WxConditions[0].Winddir, WxConditions[0].Windspeed, 100);
  DisplayAstronomySection(5, 255);               
  DisplayMainWeatherSection(320, 110);           
  DisplayWeatherIcon(810, 130);                  
  DisplayForecastSection(320, 220);              
}

/** @brief Формирует альтернативный экран (Страница 1) со сведениями локальных/внешних датчиков. */
void DisplayLocalWeather() {
  int x = 40;
  int y = 90;
  DisplayStatusSection(600, 20, wifi_signal);    
  DisplayGeneralInfoSection();                   

  drawLine(479, 25, 479, 540, Black);
  drawLine(481, 25, 481, 540, Black);              

  setFont(OpenSans24B);
  drawString( x + 160, y + 20, "OUTDOOR", CENTER);  

  setFont(OpenSans18B);
  drawString(x + 10, y + 150, "Temperature: " + String(extendet_temperature, 0) + " C", LEFT); 
  drawString(x + 10, y + 210, "Humidity: " + String(extendet_humidity, 1) + " %", LEFT); 
  drawString(x + 10, y + 285, "Pressure: " + String(extendet_pressure / 100.0f, 1) + " hPa", LEFT);
  drawString(x + 10, y + 345, "Battery: " + String(extendet_battery_percent) + " %", LEFT);
  drawString(x + 10, y + 405, "Solar: " + String(extendet_solar_percent) + " %", LEFT);
  
  setFont(OpenSans24B);
  drawString( x + 660, y + 10, "INDOOR", CENTER);   
  
  setFont(OpenSans18B);
  drawString( x + 500, y + 150, "Temperature: " + String(local_temperature, 1) + " C", LEFT); 
  drawString( x + 500, y + 210, "Humidity: " + String(local_humidity, 1) + " %", LEFT); 
  drawString( x + 500, y + 285, "Pressure: " + String(local_pressure, 1) + " mm", LEFT); 
}

/**
 * @brief Функция считывания тачскрина и переключения отображаемых страниц экрана.
 */
void DisplayPage(){
  uint16_t  x, y;
  leftTouch.x1 = 5;
  leftTouch.x2 = 250;
  leftTouch.y1 = 40;
  leftTouch.y2 = 500;

  rihgtTouch.x1 = 660;
  rihgtTouch.x2 = 960;
  rihgtTouch.y1 = 40;
  rihgtTouch.y2 = 500;
 
  if (digitalRead(TOUCH_INT)) { 
    if(touch.scanPoint()) {
      touch.getPoint(x, y, 0);
      Serial.printf("X:%d Y:%d\n", x, y);
      y = EPD_HEIGHT - y;
      if ((x > leftTouch.x1 && x < leftTouch.x2) && (y > leftTouch.y1  && y < leftTouch.y2)) {
        currentPage--;
      } else if ((x > rihgtTouch.x1 && x < rihgtTouch.x2) && (y > rihgtTouch.y1 && y < rihgtTouch.y2)) {
        currentPage++;
      } else {
        return;
      }
      currentPage %= page;
      Serial.printf("currentPageX:%d\n", currentPage);

      if(currentPage == 0){
          epd_poweron();      
          epd_clear();        
          memset(framebuffer, 0xFF, EPD_WIDTH * EPD_HEIGHT / 2);
          DisplayWeather();   
          edp_update();       
          epd_poweroff_light(); 
      }
      if(currentPage == 1){
          epd_poweron();      
          memset(framebuffer, 0xFF, EPD_WIDTH * EPD_HEIGHT / 2);
          epd_clear();        
          DisplayLocalWeather();
          edp_update();       
          epd_poweroff_light(); 
      }
      while (digitalRead(TOUCH_INT)) { }
      while (digitalRead(TOUCH_INT)) { }
    }
  }
}
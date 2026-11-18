#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_display_panel.hpp>
#include <math.h>
#include <time.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "Wire.h"
#include "ESP_IOExpander_Library.h"

#include <lvgl.h>
#include "lvgl_v8_port.h"

struct DailyStat;
struct ColumnInfo;

using namespace esp_panel::drivers;
using namespace esp_panel::board;

// ---------- SD card via CH422G IO expander (Waveshare built-in slot) ----------
#define I2C_MASTER_NUM      0
#define I2C_MASTER_SDA_IO   8
#define I2C_MASTER_SCL_IO   9

#define SD_CS   4
#define SD_MOSI 11
#define SD_CLK  12
#define SD_MISO 13
#define SD_SS   -1

ESP_IOExpander_CH422G *expander = NULL;
bool sdOk = false;   // true only if the card mounted AND a test file could be written

bool initSDCard(Board *board) {
    // The Board library already owns I2C port 0 and the CH422G expander.
    // Reuse it. Do NOT call Wire.begin() or create a second expander here.
    auto boardExpander = board->getIO_Expander();
    if (!boardExpander) {
        Serial.println("SD: board has no IO expander");
        return false;
    }

    auto base = boardExpander->getBase();   // underlying expander, has pinMode/digitalWrite
    if (!base) {
        Serial.println("SD: IO expander base not available");
        return false;
    }
    base->pinMode(SD_CS, OUTPUT);           // harmless if the board already set it up
    base->digitalWrite(SD_CS, LOW);

    SPI.setHwCs(false);
    SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_SS);

    bool mounted = false;
    for (int attempt = 1; attempt <= 3 && !mounted; attempt++) {
        mounted = SD.begin(SD_SS);
        if (!mounted) {
            Serial.printf("SD: mount attempt %d failed\n", attempt);
            SD.end();
            delay(300);
        }
    }
    if (!mounted) {
        Serial.println("SD: mount failed (check card is seated and formatted FAT32)");
        return false;
    }

    uint8_t type = SD.cardType();
    if (type == CARD_NONE) {
        Serial.println("SD: no card detected");
        return false;
    }
    Serial.printf("SD: card type %s, size %llu MB\n",
                  type == CARD_MMC ? "MMC" : type == CARD_SD ? "SDSC" : type == CARD_SDHC ? "SDHC" : "UNKNOWN",
                  (unsigned long long)(SD.cardSize() / (1024ULL * 1024ULL)));

    File f = SD.open("/sdtest.txt", FILE_WRITE);
    if (!f) {
        Serial.println("SD: mounted but cannot create files (wrong format, corrupt, or write-protected)");
        return false;
    }
    f.println("ok");
    f.close();
    SD.remove("/sdtest.txt");

    Serial.println("SD: mounted and writable");
    return true;
}

const char* ssid = "dlink-6D5C";
const char* password = "intentionally open ask for password";
const char* weatherURL = "http://192.168.0.100/weather";


// ---------- Layout constants ----------
const int COMPASS_DIAMETER   = 320;
const int COMPASS_CENTER_X   = 400;
const int COMPASS_CENTER_Y   = 210;
const float MAX_WIND_SPEED   = 20.0;
const int MIN_DISC_DIAMETER  = 40;
const int MAX_DISC_DIAMETER  = 260;

// ---------- Color palette ----------
#define COLOR_BG        lv_color_hex(0xF4F7FA)
#define COLOR_CARD      lv_color_hex(0xFFFFFF)
#define COLOR_ACCENT    lv_color_hex(0x2E86DE)
#define COLOR_ACCENT_LT lv_color_hex(0xAED6F1)
#define COLOR_TEXT      lv_color_hex(0x1B2631)
#define COLOR_SUBTEXT   lv_color_hex(0x5D6D7E)
#define COLOR_NEEDLE    lv_color_hex(0xE74C3C)
#define COLOR_ROSE      lv_color_hex(0x27AE60)

// ---------- LVGL objects ----------
lv_obj_t *main_screen;
lv_obj_t *label_temp_val, *label_humidity_val, *label_pressure_val, *label_radiation_val;
lv_obj_t *label_wind_speed_val, *label_wind_dir_val;
lv_obj_t *label_status;

lv_obj_t *compass_ring;
lv_obj_t *speed_disc;
lv_obj_t *needle_line;
static lv_point_t needle_points[2];

unsigned long lastFetch = 0;
const unsigned long fetchInterval = 10000; // 10 seconds

const char *LOG_PATH = "/weather_log.csv";

// ---------- Last good values (only ever updated from a fully valid fetch) ----------
static float lastTemp = 0, lastHum = 0, lastPres = 0, lastRad = 0, lastWindSpeed = 0, lastWindDir = 0;
static bool haveGood = false;
static int failCount = 0;

// ---------- Wind rose (daily direction/speed distribution) ----------
const int ROSE_SECTORS = 16;
const int ROSE_MAX_RADIUS = 130; // stays inside the compass ring, leaving room for the live needle on top
int roseCounts[ROSE_SECTORS] = {0};
float roseSpeedSum[ROSE_SECTORS] = {0};
char currentRoseDate[11] = "";
lv_obj_t *rose_lines[ROSE_SECTORS];
static lv_point_t rosePoints[ROSE_SECTORS][2];

// ---------- Column info for tap-to-view history ----------
struct ColumnInfo {
    int colIndex;      // index into the CSV row after timestamp: 1=temp,2=hum,3=pres,4=rad,5=windSpeed
    const char *title;
    const char *unit;
};
static const ColumnInfo tempInfo  = {1, "Temperature", "C"};
static const ColumnInfo humInfo   = {2, "Humidity", "%"};
static const ColumnInfo presInfo  = {3, "Pressure", "hPa"};
static const ColumnInfo radInfo   = {4, "Radiation", "W/m^2"};
static const ColumnInfo windInfo  = {5, "Wind Speed", "m/s"};

// ---------- Style helpers ----------
static lv_style_t style_card;
static lv_style_t style_card_pressed;

void init_styles() {
    lv_style_init(&style_card);
    lv_style_set_bg_color(&style_card, COLOR_CARD);
    lv_style_set_bg_opa(&style_card, LV_OPA_COVER);
    lv_style_set_radius(&style_card, 16);
    lv_style_set_shadow_width(&style_card, 12);
    lv_style_set_shadow_color(&style_card, lv_color_hex(0xB0BEC5));
    lv_style_set_shadow_opa(&style_card, LV_OPA_30);
    lv_style_set_pad_all(&style_card, 12);
    lv_style_set_border_width(&style_card, 0);

    lv_style_init(&style_card_pressed);
    lv_style_set_bg_color(&style_card_pressed, COLOR_ACCENT_LT);
}

// ---------- SD logging ----------
void logToSD(float temp, float hum, float pres, float rad, float windSpeed, float windDir) {
    if (!sdOk) return;

    File file = SD.open(LOG_PATH, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open log file for appending");
        return;
    }
    if (file.size() == 0) {
        file.println("timestamp,temperature,humidity,pressure,radiation,windSpeed,windDirection");
    }
    time_t now = time(nullptr);
    file.printf("%ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                (long)now, temp, hum, pres, rad, windSpeed, windDir);
    file.close();
}

// ---------- Daily min/max computation ----------
struct DailyStat {
    char date[11]; // "YYYY-MM-DD"
    float minVal;
    float maxVal;
};

int computeDailyMinMax(int columnIndex, DailyStat *results, int maxDays) {
    if (!sdOk) return 0;

    File file = SD.open(LOG_PATH, FILE_READ);
    if (!file) {
        Serial.println("Failed to open log file for reading");
        return 0;
    }

    file.readStringUntil('\n'); // skip header

    int count = 0;
    char currentDate[11] = "";
    float currentMin = 0, currentMax = 0;
    bool haveDay = false;

    while (file.available()) {
        String line = file.readStringUntil('\n');
        if (line.length() < 5) continue;

        long ts;
        float values[6];
        int parsed = sscanf(line.c_str(), "%ld,%f,%f,%f,%f,%f,%f",
                             &ts, &values[0], &values[1], &values[2],
                             &values[3], &values[4], &values[5]);
        if (parsed != 7) continue;

        float val = values[columnIndex - 1];

        struct tm timeinfo;
        time_t t = (time_t)ts;
        localtime_r(&t, &timeinfo);
        char dateBuf[11];
        strftime(dateBuf, sizeof(dateBuf), "%Y-%m-%d", &timeinfo);

        if (!haveDay) {
            strcpy(currentDate, dateBuf);
            currentMin = val;
            currentMax = val;
            haveDay = true;
        } else if (strcmp(dateBuf, currentDate) != 0) {
            if (count < maxDays) {
                strcpy(results[count].date, currentDate);
                results[count].minVal = currentMin;
                results[count].maxVal = currentMax;
                count++;
            }
            strcpy(currentDate, dateBuf);
            currentMin = val;
            currentMax = val;
        } else {
            if (val < currentMin) currentMin = val;
            if (val > currentMax) currentMax = val;
        }
    }
    file.close();

    if (haveDay && count < maxDays) {
        strcpy(results[count].date, currentDate);
        results[count].minVal = currentMin;
        results[count].maxVal = currentMax;
        count++;
    }

    return count;
}

// ---------- Wind rose logic ----------
void getTodayDateString(char *buf, size_t bufLen) {
    time_t now = time(nullptr);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    strftime(buf, bufLen, "%Y-%m-%d", &timeinfo);
}

// Saves the completed day's rose as its own file, one row per 22.5° sector
void saveWindRoseToSD(const char *dateStr) {
    if (!sdOk) return;

    char path[48];
    snprintf(path, sizeof(path), "/windrose_%s.csv", dateStr);
    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to create wind rose file");
        return;
    }
    file.println("directionDeg,count,avgSpeed");
    for (int i = 0; i < ROSE_SECTORS; i++) {
        float avgSpeed = (roseCounts[i] > 0) ? (roseSpeedSum[i] / roseCounts[i]) : 0.0f;
        file.printf("%d,%d,%.2f\n", i * (360 / ROSE_SECTORS), roseCounts[i], avgSpeed);
    }
    file.close();
    Serial.print("Saved wind rose for ");
    Serial.println(dateStr);
}

void updateWindRoseVisual() {
    int maxCount = 1; // avoid divide-by-zero before any data has come in
    for (int i = 0; i < ROSE_SECTORS; i++) {
        if (roseCounts[i] > maxCount) maxCount = roseCounts[i];
    }
    for (int i = 0; i < ROSE_SECTORS; i++) {
        float frac = (float)roseCounts[i] / (float)maxCount;
        float length = frac * ROSE_MAX_RADIUS;
        float angleRad = (i * (360.0f / ROSE_SECTORS)) * (float)M_PI / 180.0f;
        rosePoints[i][0] = {(lv_coord_t)COMPASS_CENTER_X, (lv_coord_t)COMPASS_CENTER_Y};
        rosePoints[i][1] = {
            (lv_coord_t)(COMPASS_CENTER_X + length * sinf(angleRad)),
            (lv_coord_t)(COMPASS_CENTER_Y - length * cosf(angleRad))
        };
        lv_line_set_points(rose_lines[i], rosePoints[i], 2);
    }
}

// Call once per FRESH reading, with the LVGL lock held. Bins the reading into a 22.5° sector,
// and rolls over to a fresh rose (saving the previous day's) whenever the date changes.
void updateWindRose(float windSpeed, float windDir) {
    char today[11];
    getTodayDateString(today, sizeof(today));

    if (currentRoseDate[0] == '\0') {
        strcpy(currentRoseDate, today);
    } else if (strcmp(today, currentRoseDate) != 0) {
        saveWindRoseToSD(currentRoseDate);
        for (int i = 0; i < ROSE_SECTORS; i++) {
            roseCounts[i] = 0;
            roseSpeedSum[i] = 0;
        }
        strcpy(currentRoseDate, today);
    }

    float sectorWidth = 360.0f / ROSE_SECTORS;
    int sector = ((int)((windDir + sectorWidth / 2.0f) / sectorWidth)) % ROSE_SECTORS;
    if (sector < 0) sector += ROSE_SECTORS;

    roseCounts[sector]++;
    roseSpeedSum[sector] += windSpeed;

    updateWindRoseVisual();
}

void create_wind_rose(lv_obj_t *parent) {
    for (int i = 0; i < ROSE_SECTORS; i++) {
        rose_lines[i] = lv_line_create(parent);
        lv_obj_set_style_line_width(rose_lines[i], 6, 0);
        lv_obj_set_style_line_color(rose_lines[i], COLOR_ROSE, 0);
        lv_obj_set_style_line_rounded(rose_lines[i], true, 0);
        lv_obj_set_style_line_opa(rose_lines[i], LV_OPA_70, 0);
        rosePoints[i][0] = {(lv_coord_t)COMPASS_CENTER_X, (lv_coord_t)COMPASS_CENTER_Y};
        rosePoints[i][1] = {(lv_coord_t)COMPASS_CENTER_X, (lv_coord_t)COMPASS_CENTER_Y};
        lv_line_set_points(rose_lines[i], rosePoints[i], 2);
    }
}

// ---------- History screen ----------
void back_btn_event_cb(lv_event_t *e) {
    lv_obj_t *history_screen = (lv_obj_t *)lv_event_get_user_data(e);
    lv_scr_load(main_screen);
    lv_obj_del_async(history_screen);
}

void showHistoryScreen(const ColumnInfo *info) {
    const int MAX_DAYS = 14;
    DailyStat results[MAX_DAYS];
    int count = computeDailyMinMax(info->colIndex, results, MAX_DAYS);

    lv_obj_t *history_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(history_screen, COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(history_screen, LV_OPA_COVER, LV_PART_MAIN);

    char titleBuf[48];
    snprintf(titleBuf, sizeof(titleBuf), "%s history (%s)", info->title, info->unit);
    lv_obj_t *title = lv_label_create(history_screen);
    lv_label_set_text(title, titleBuf);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, COLOR_TEXT, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 16);

    lv_obj_t *table = lv_table_create(history_screen);
    lv_obj_set_size(table, 500, 320);
    lv_obj_align(table, LV_ALIGN_TOP_MID, 0, 64);

    lv_table_set_col_cnt(table, 3);
    lv_table_set_col_width(table, 0, 200);
    lv_table_set_col_width(table, 1, 150);
    lv_table_set_col_width(table, 2, 150);

    lv_table_set_cell_value(table, 0, 0, "Date");
    lv_table_set_cell_value(table, 0, 1, "Min");
    lv_table_set_cell_value(table, 0, 2, "Max");

    if (count == 0) {
        lv_table_set_cell_value(table, 1, 0, sdOk ? "No data yet" : "No SD card");
        lv_table_set_cell_value(table, 1, 1, "-");
        lv_table_set_cell_value(table, 1, 2, "-");
    } else {
        for (int i = 0; i < count; i++) {
            int row = i + 1;
            int srcIdx = count - 1 - i;
            char minBuf[16], maxBuf[16];
            snprintf(minBuf, sizeof(minBuf), "%.1f", results[srcIdx].minVal);
            snprintf(maxBuf, sizeof(maxBuf), "%.1f", results[srcIdx].maxVal);
            lv_table_set_cell_value(table, row, 0, results[srcIdx].date);
            lv_table_set_cell_value(table, row, 1, minBuf);
            lv_table_set_cell_value(table, row, 2, maxBuf);
        }
    }

    lv_obj_t *back_btn = lv_btn_create(history_screen);
    lv_obj_set_size(back_btn, 120, 50);
    lv_obj_align(back_btn, LV_ALIGN_BOTTOM_MID, 0, -16);
    lv_obj_set_style_bg_color(back_btn, COLOR_ACCENT, 0);
    lv_obj_t *back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "Back");
    lv_obj_center(back_label);
    lv_obj_add_event_cb(back_btn, back_btn_event_cb, LV_EVENT_CLICKED, history_screen);

    lv_scr_load(history_screen);
}

void card_click_event_cb(lv_event_t *e) {
    lv_obj_t *card = lv_event_get_target(e);
    ColumnInfo *info = (ColumnInfo *)lv_obj_get_user_data(card);
    if (info != NULL) {
        showHistoryScreen(info);
    }
}

lv_obj_t* create_info_card(lv_obj_t *parent, const char *title, lv_align_t align, int x_ofs, int y_ofs, int width, const ColumnInfo *linkedInfo) {
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_remove_style_all(card);
    lv_obj_add_style(card, &style_card, 0);
    lv_obj_add_style(card, &style_card_pressed, LV_STATE_PRESSED);
    lv_obj_set_size(card, width, 80);
    lv_obj_align(card, align, x_ofs, y_ofs);
    lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);

    if (linkedInfo != NULL) {
        lv_obj_set_user_data(card, (void *)linkedInfo);
        lv_obj_add_event_cb(card, card_click_event_cb, LV_EVENT_CLICKED, NULL);
    }

    lv_obj_t *title_label = lv_label_create(card);
    lv_label_set_text(title_label, title);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(title_label, COLOR_SUBTEXT, 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *value_label = lv_label_create(card);
    lv_label_set_text(value_label, "N/A");
    lv_obj_set_style_text_font(value_label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(value_label, COLOR_TEXT, 0);
    lv_obj_align(value_label, LV_ALIGN_BOTTOM_LEFT, 0, 0);

    return value_label;
}

// ---------- Compass build ----------
void create_compass(lv_obj_t *parent) {
    compass_ring = lv_obj_create(parent);
    lv_obj_remove_style_all(compass_ring);
    lv_obj_set_size(compass_ring, COMPASS_DIAMETER, COMPASS_DIAMETER);
    lv_obj_set_pos(compass_ring, COMPASS_CENTER_X - COMPASS_DIAMETER / 2, COMPASS_CENTER_Y - COMPASS_DIAMETER / 2);
    lv_obj_set_style_radius(compass_ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(compass_ring, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(compass_ring, 4, 0);
    lv_obj_set_style_border_color(compass_ring, COLOR_ACCENT, 0);
    lv_obj_clear_flag(compass_ring, LV_OBJ_FLAG_SCROLLABLE);

    const char *dirs[4] = {"N", "E", "S", "W"};
    int dx[4] = {0, 1, 0, -1};
    int dy[4] = {-1, 0, 1, 0};
    int r = COMPASS_DIAMETER / 2 + 18;
    for (int i = 0; i < 4; i++) {
        lv_obj_t *lbl = lv_label_create(parent);
        lv_label_set_text(lbl, dirs[i]);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(lbl, COLOR_SUBTEXT, 0);
        lv_obj_set_pos(lbl,
            COMPASS_CENTER_X + dx[i] * r - 8,
            COMPASS_CENTER_Y + dy[i] * r - 10);
    }

    speed_disc = lv_obj_create(parent);
    lv_obj_remove_style_all(speed_disc);
    lv_obj_set_style_radius(speed_disc, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(speed_disc, COLOR_ACCENT_LT, 0);
    lv_obj_set_style_bg_opa(speed_disc, LV_OPA_70, 0);
    lv_obj_clear_flag(speed_disc, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(speed_disc, MIN_DISC_DIAMETER, MIN_DISC_DIAMETER);
    lv_obj_align(speed_disc, LV_ALIGN_DEFAULT, 0, 0);

    needle_line = lv_line_create(parent);
    lv_obj_set_style_line_width(needle_line, 6, 0);
    lv_obj_set_style_line_color(needle_line, COLOR_NEEDLE, 0);
    lv_obj_set_style_line_rounded(needle_line, true, 0);
    needle_points[0] = {(lv_coord_t)COMPASS_CENTER_X, (lv_coord_t)COMPASS_CENTER_Y};
    needle_points[1] = {(lv_coord_t)COMPASS_CENTER_X, (lv_coord_t)(COMPASS_CENTER_Y - COMPASS_DIAMETER / 2)};
    lv_line_set_points(needle_line, needle_points, 2);

    lv_obj_t *hub = lv_obj_create(parent);
    lv_obj_remove_style_all(hub);
    lv_obj_set_style_radius(hub, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(hub, COLOR_NEEDLE, 0);
    lv_obj_set_style_bg_opa(hub, LV_OPA_COVER, 0);
    lv_obj_clear_flag(hub, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(hub, 16, 16);
    lv_obj_set_pos(hub, COMPASS_CENTER_X - 8, COMPASS_CENTER_Y - 8);
}

void update_compass(float windSpeedMs, float windDirDeg) {
    float clampedSpeed = windSpeedMs;
    if (clampedSpeed < 0) clampedSpeed = 0;
    if (clampedSpeed > MAX_WIND_SPEED) clampedSpeed = MAX_WIND_SPEED;

    int diameter = MIN_DISC_DIAMETER + (int)((MAX_DISC_DIAMETER - MIN_DISC_DIAMETER) * (clampedSpeed / MAX_WIND_SPEED));
    lv_obj_set_size(speed_disc, diameter, diameter);
    lv_obj_set_pos(speed_disc, COMPASS_CENTER_X - diameter / 2, COMPASS_CENTER_Y - diameter / 2);

    float angleRad = windDirDeg * (float)M_PI / 180.0f;
    float length = COMPASS_DIAMETER / 2.0f - 6;
    needle_points[0] = {(lv_coord_t)COMPASS_CENTER_X, (lv_coord_t)COMPASS_CENTER_Y};
    needle_points[1] = {
        (lv_coord_t)(COMPASS_CENTER_X + length * sinf(angleRad)),
        (lv_coord_t)(COMPASS_CENTER_Y - length * cosf(angleRad))
    };
    lv_line_set_points(needle_line, needle_points, 2);
}

// ---------- Display of held values (call with the LVGL lock held) ----------
void showValues() {
    char buf[24];

    snprintf(buf, sizeof(buf), "%.1f C", lastTemp);
    lv_label_set_text(label_temp_val, buf);

    snprintf(buf, sizeof(buf), "%.1f %%", lastHum);
    lv_label_set_text(label_humidity_val, buf);

    snprintf(buf, sizeof(buf), "%.1f hPa", lastPres);
    lv_label_set_text(label_pressure_val, buf);

    snprintf(buf, sizeof(buf), "%.1f W/m^2", lastRad);
    lv_label_set_text(label_radiation_val, buf);

    snprintf(buf, sizeof(buf), "%.1f m/s", lastWindSpeed);
    lv_label_set_text(label_wind_speed_val, buf);

    snprintf(buf, sizeof(buf), "%.0f deg", lastWindDir);
    lv_label_set_text(label_wind_dir_val, buf);

    update_compass(lastWindSpeed, lastWindDir);
}

// A failed fetch never touches the values on screen, it only changes the status line
void showStale(const char *reason) {
    failCount++;
    char s[96];
    snprintf(s, sizeof(s), haveGood ? "Status: %s (%d in a row) - showing last good data"
                                    : "Status: %s (%d in a row) - waiting for data",
             reason, failCount);
    Serial.println(s);
    lvgl_port_lock(-1);
    lv_label_set_text(label_status, s);
    lvgl_port_unlock();
}

static bool inRange(float v, float lo, float hi) { return v >= lo && v <= hi; }

void fetchWeatherData() {
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.reconnect();
        showStale("WiFi disconnected");
        return;
    }

    
    String payload;
    int httpCode = -1;

    for (int attempt = 0; attempt < 2; attempt++) {
        HTTPClient http;
        http.setConnectTimeout(4000);
        http.setTimeout(7000);  // Longer than the sender's worst-case WU upload

        http.begin(weatherURL);
        httpCode = http.GET();

        if (httpCode == 200) {
            payload = http.getString();
            http.end();
            break;
        }

        Serial.printf(
            "HTTP failed (try %d): %d (%s), RSSI %d\n",
            attempt + 1,
            httpCode,
            HTTPClient::errorToString(httpCode).c_str(),
            WiFi.RSSI()
        );

        http.end();
        delay(500);
    }

    if (httpCode != 200) {
        char r[40];
        snprintf(r, sizeof(r), "HTTP error %d", httpCode);
        showStale(r);
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload);

    bool ok = !err &&
        doc["temperature"].is<float>() && doc["humidity"].is<float>() &&
        doc["pressure"].is<float>()    && doc["radiation"].is<float>() &&
        doc["windSpeed"].is<float>()   && doc["windDirection"].is<float>();

    float temp = 0, hum = 0, pres = 0, rad = 0, ws = 0, wd = 0;
    if (ok) {
        temp = doc["temperature"];
        hum  = doc["humidity"];
        pres = doc["pressure"];
        rad  = doc["radiation"];
        ws   = doc["windSpeed"];
        wd   = doc["windDirection"];

        // Plausibility check, rejects values that parse fine but can't be real
        ok = inRange(temp, -50, 60) && inRange(hum, -5, 105) && inRange(pres, 500, 1100) &&
             inRange(rad, 0, 1600)  && inRange(ws, 0, 75)   && inRange(wd, 0, 360);
        if (ok) hum = constrain(hum, 0.0f, 100.0f);
    }

    if (!ok) {
        showStale("bad data");
        return;
    }

    // Fresh, valid data: it becomes the new held set
    lastTemp = temp; lastHum = hum; lastPres = pres;
    lastRad = rad; lastWindSpeed = ws; lastWindDir = wd;
    haveGood = true;
    failCount = 0;

    lvgl_port_lock(-1);
    showValues();
    updateWindRose(ws, wd);   // touches LVGL objects, so it stays inside the lock
    char okBuf[40];
    snprintf(okBuf, sizeof(okBuf), "Status: OK (RSSI %d dBm)", WiFi.RSSI());
    lv_label_set_text(label_status, okBuf);
    lvgl_port_unlock();

    logToSD(temp, hum, pres, rad, ws, wd);   // slow SD I/O stays outside the lock
}

void setup() {
    Serial.begin(115200);

    Serial.println("Initializing board");
    Board *board = new Board();
    board->init();

#if LVGL_PORT_AVOID_TEARING_MODE
    auto lcd = board->getLCD();
    lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
    auto lcd_bus = lcd->getBus();
    if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
        static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
    }
#endif
#endif
    assert(board->begin());

    sdOk = initSDCard(board);
    if (!sdOk) Serial.println("Continuing without SD logging");

    Serial.println("Initializing LVGL");
    lvgl_port_init(board->getLCD(), board->getTouch());

    Serial.println("Creating UI");
    lvgl_port_lock(-1);

    init_styles();

    main_screen = lv_scr_act();
    lv_obj_set_style_bg_color(main_screen, COLOR_BG, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(main_screen, LV_OPA_COVER, LV_PART_MAIN);

    label_temp_val      = create_info_card(main_screen, "TEMPERATURE", LV_ALIGN_TOP_LEFT, 16, 16, 180, &tempInfo);
    label_humidity_val  = create_info_card(main_screen, "HUMIDITY",    LV_ALIGN_TOP_RIGHT,    -16, 16,  160, &humInfo);
    label_pressure_val  = create_info_card(main_screen, "PRESSURE",    LV_ALIGN_BOTTOM_LEFT,   16, -16, 160, &presInfo);
    label_radiation_val = create_info_card(main_screen, "RADIATION",   LV_ALIGN_BOTTOM_RIGHT, -16, -16, 180, &radInfo);

    create_wind_rose(main_screen); // drawn first so it sits BELOW the live needle/disc
    create_compass(main_screen);

    lv_obj_t *wind_readout = lv_obj_create(main_screen);
    lv_obj_remove_style_all(wind_readout);
    lv_obj_add_style(wind_readout, &style_card, 0);
    lv_obj_add_style(wind_readout, &style_card_pressed, LV_STATE_PRESSED);
    lv_obj_set_size(wind_readout, 260, 60);
    lv_obj_align(wind_readout, LV_ALIGN_TOP_MID, 0, COMPASS_CENTER_Y + COMPASS_DIAMETER / 2 + 30);
    lv_obj_clear_flag(wind_readout, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(wind_readout, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_user_data(wind_readout, (void *)&windInfo);
    lv_obj_add_event_cb(wind_readout, card_click_event_cb, LV_EVENT_CLICKED, NULL);

    label_wind_speed_val = lv_label_create(wind_readout);
    lv_label_set_text(label_wind_speed_val, "N/A");
    lv_obj_set_style_text_font(label_wind_speed_val, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(label_wind_speed_val, COLOR_ACCENT, 0);
    lv_obj_align(label_wind_speed_val, LV_ALIGN_LEFT_MID, 0, 0);

    label_wind_dir_val = lv_label_create(wind_readout);
    lv_label_set_text(label_wind_dir_val, "N/A");
    lv_obj_set_style_text_font(label_wind_dir_val, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(label_wind_dir_val, COLOR_TEXT, 0);
    lv_obj_align(label_wind_dir_val, LV_ALIGN_RIGHT_MID, 0, 0);

    label_status = lv_label_create(main_screen);
    lv_label_set_text(label_status, "Status: connecting...");
    lv_obj_set_style_text_font(label_status, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label_status, COLOR_SUBTEXT, 0);
    lv_obj_align(label_status, LV_ALIGN_BOTTOM_MID, 0, -6);

    lvgl_port_unlock();

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected. IP: ");
    Serial.println(WiFi.localIP());

    configTime(2 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    Serial.print("Waiting for NTP time sync");
    time_t now = time(nullptr);
    while (now < 8 * 3600 * 2) {
        delay(500);
        Serial.print(".");
        now = time(nullptr);
    }
    Serial.println();
    Serial.println("Time synced.");

    fetchWeatherData(); // initial fetch
    lastFetch = millis();
}

void loop() {
    if (millis() - lastFetch >= fetchInterval) {
        lastFetch = millis();
        fetchWeatherData();
    }
    delay(10);
}
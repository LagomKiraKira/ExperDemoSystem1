/*
 * DmdConfigIO.cpp
 *
 * JSON-based persistence for DMD calibration and UI configuration.
 * Uses hand-written JSON serialization (no external dependency).
 */

#include "DmdConfigIO.h"
#include "def_common.h"
#include <fstream>
#include <cstdio>
#include <cstring>
#include <windows.h>

// Simple JSON escape for strings
static std::string jsonEscape(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '\\') out += "\\\\";
        else if (c == '"') out += "\\\"";
        else if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else if (c == '\t') out += "\\t";
        else out += c;
    }
    return out;
}

// Write double array as JSON array
static void writeDoubleArray(std::ofstream& f, const char* key, const double* arr, int n) {
    f << "  \"" << key << "\": [";
    for (int i = 0; i < n; i++) {
        if (i > 0) f << ", ";
        f << arr[i];
    }
    f << "],\n";
}

bool DmdConfig_Save(const DmdPersistConfig& cfg, const std::string& filepath) {
    std::ofstream f(filepath);
    if (!f.is_open()) {
        printf("[DmdConfigIO] Failed to open %s for writing\n", filepath.c_str());
        return false;
    }

    f << "{\n";
    writeDoubleArray(f, "homoMatrix", cfg.homoMatrix, 9);
    f << "  \"channelBoundLeft\": " << cfg.channelBoundLeft << ",\n";
    f << "  \"channelBoundRight\": " << cfg.channelBoundRight << ",\n";
    f << "  \"deltaX\": " << cfg.deltaX << ",\n";
    f << "  \"deltaY\": " << cfg.deltaY << ",\n";
    f << "  \"calibrated\": " << (cfg.calibrated ? "true" : "false") << ",\n";
    f << "  \"patternType\": " << cfg.patternType << ",\n";
    f << "  \"spotRadius\": " << cfg.spotRadius << ",\n";
    f << "  \"segStart\": " << cfg.segStart << ",\n";
    f << "  \"segEnd\": " << cfg.segEnd << ",\n";
    f << "  \"kalmanComp\": " << cfg.kalmanComp << ",\n";
    f << "  \"sideSelect\": " << cfg.sideSelect << ",\n";
    f << "  \"refreshRate\": " << cfg.refreshRate << ",\n";
    f << "  \"ledIntensity\": " << cfg.ledIntensity << ",\n";
    f << "  \"simulationMode\": " << (cfg.simulationMode ? "true" : "false") << ",\n";
    f << "  \"lastVideoPath\": \"" << jsonEscape(cfg.lastVideoPath) << "\"\n";
    f << "}\n";
    f.close();

    printf("[DmdConfigIO] Configuration saved to %s\n", filepath.c_str());
    return true;
}

// Simple JSON parser for flat objects (single level, no arrays of objects)
static bool jsonGetBool(const std::string& json, const char* key, bool defaultVal) {
    std::string search = std::string("\"") + key + "\":";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return defaultVal;
    pos += search.length();
    while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    if (json.substr(pos, 5) == "true") return true;
    return false;
}

static int jsonGetInt(const std::string& json, const char* key, int defaultVal) {
    std::string search = std::string("\"") + key + "\":";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return defaultVal;
    pos += search.length();
    while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    return atoi(json.c_str() + pos);
}

static double jsonGetDouble(const std::string& json, const char* key, double defaultVal) {
    std::string search = std::string("\"") + key + "\":";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return defaultVal;
    pos += search.length();
    while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    return atof(json.c_str() + pos);
}

static void jsonGetDoubleArray(const std::string& json, const char* key, double* arr, int n) {
    std::string search = std::string("\"") + key + "\": [";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return;
    pos += search.length();
    for (int i = 0; i < n; i++) {
        while (pos < json.length() && (json[pos] == ' ' || json[pos] == '\t' || json[pos] == ',')) pos++;
        arr[i] = atof(json.c_str() + pos);
        while (pos < json.length() && json[pos] != ',' && json[pos] != ']') pos++;
        if (json[pos] == ',') pos++;
    }
}

static std::string jsonGetString(const std::string& json, const char* key, const char* defaultVal) {
    std::string search = std::string("\"") + key + "\": \"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return defaultVal;
    pos += search.length();
    std::string result;
    while (pos < json.length() && json[pos] != '"') {
        if (json[pos] == '\\' && pos + 1 < json.length()) {
            pos++;
            switch (json[pos]) {
                case 'n': result += '\n'; break;
                case 'r': result += '\r'; break;
                case 't': result += '\t'; break;
                case '\\': result += '\\'; break;
                case '"': result += '"'; break;
                default: result += json[pos]; break;
            }
        } else {
            result += json[pos];
        }
        pos++;
    }
    return result;
}

bool DmdConfig_Load(DmdPersistConfig& cfg, const std::string& filepath) {
    std::ifstream f(filepath);
    if (!f.is_open()) {
        printf("[DmdConfigIO] Config file not found: %s (using defaults)\n", filepath.c_str());
        return false;
    }

    std::string json((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    f.close();

    jsonGetDoubleArray(json, "homoMatrix", cfg.homoMatrix, 9);
    cfg.channelBoundLeft = jsonGetInt(json, "channelBoundLeft", 0);
    cfg.channelBoundRight = jsonGetInt(json, "channelBoundRight", 1920);
    cfg.deltaX = jsonGetDouble(json, "deltaX", 0.0);
    cfg.deltaY = jsonGetDouble(json, "deltaY", 0.0);
    cfg.calibrated = jsonGetBool(json, "calibrated", false);
    cfg.patternType = jsonGetInt(json, "patternType", 0);
    cfg.spotRadius = jsonGetInt(json, "spotRadius", 20);
    cfg.segStart = jsonGetInt(json, "segStart", 0);
    cfg.segEnd = jsonGetInt(json, "segEnd", 9);
    cfg.kalmanComp = jsonGetInt(json, "kalmanComp", 0);
    cfg.sideSelect = jsonGetInt(json, "sideSelect", 2);
    cfg.refreshRate = jsonGetInt(json, "refreshRate", 32);
    cfg.ledIntensity = jsonGetInt(json, "ledIntensity", 0);
    cfg.simulationMode = jsonGetBool(json, "simulationMode", false);

    std::string path = jsonGetString(json, "lastVideoPath", "");
    strncpy_s(cfg.lastVideoPath, sizeof(cfg.lastVideoPath), path.c_str(), _TRUNCATE);

    printf("[DmdConfigIO] Configuration loaded from %s\n", filepath.c_str());
    return true;
}

std::string DmdConfig_DefaultPath() {
    // Get executable directory
    char exePath[MAX_PATH];
    GetModuleFileNameA(NULL, exePath, MAX_PATH);
    std::string dir(exePath);
    size_t lastSlash = dir.find_last_of("\\/");
    if (lastSlash != std::string::npos) {
        dir = dir.substr(0, lastSlash + 1);
    }
    return dir + "dmd_config.json";
}

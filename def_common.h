#pragma once
#include <string>

#define EXP_ERROR    -1
#define EXP_SUCCESS  0
#define NSIZEX       1600     
#define NSIZEY       1100

/*
* 电场刺激电压的类型
*/
enum WaveType
{
    None,          //无电压
    Square,        //方波电压
    Sine,          //正弦电压
    DC             //直流电压
};
/* =================================================================
 *  DMD Persistence Configuration (JSON serializable)
 *  Stores calibration data and UI control values
 * ================================================================= */
#include <vector>

struct DmdPersistConfig {
    // Calibration data
    double homoMatrix[9];           // 3x3 homography matrix (row-major)
    int    channelBoundLeft;        // Left boundary of laser channel in DMD pixels
    int    channelBoundRight;       // Right boundary
    double deltaX;                  // Translation offset X (DMD pixels)
    double deltaY;                  // Translation offset Y (DMD pixels)
    bool   calibrated;              // Whether calibration has been performed

    // UI control values
    int    patternType;             // 0=circle, 1=arc along body
    int    spotRadius;              // 1-100 px
    int    segStart;                // 0-99
    int    segEnd;                  // 0-99
    int    kalmanComp;              // 0-50 px
    int    sideSelect;              // 0=left, 1=right, 2=both
    int    refreshRate;             // 30-60 Hz
    int    ledIntensity;            // 0-100%

    // Simulation mode
    bool   simulationMode;          // true=sim, false=realtime
    char   lastVideoPath[512];      // Last loaded AVI file path

    DmdPersistConfig() {
        for (int i = 0; i < 9; i++) homoMatrix[i] = (i % 4 == 0) ? 1.0 : 0.0;
        channelBoundLeft = 0;
        channelBoundRight = 1920;
        deltaX = 0.0;
        deltaY = 0.0;
        calibrated = false;
        patternType = 0;
        spotRadius = 20;
        segStart = 0;
        segEnd = 9;
        kalmanComp = 0;
        sideSelect = 2;
        refreshRate = 32;
        ledIntensity = 0;
        simulationMode = false;
        memset(lastVideoPath, 0, sizeof(lastVideoPath));
    }
};
;
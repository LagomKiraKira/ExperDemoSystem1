/*
 * Talk2ViALUX_DMD.h
 *
 * Created on June 2026
 * Interface for ViALUX DMD V-9501 (ALP API)
 * High-precision spatial optogenetic stimulation for C. elegans
 *
 * DMD V-9501 specs:
 *   - 1920 x 1080 micromirrors (Full HD)
 *   - Pattern rates up to 17.8 kHz (binary)
 *   - Multiple LED wavelength options
 */

#pragma once
#ifndef TALK2VIALUX_DMD_H_
#define TALK2VIALUX_DMD_H_

#include <windows.h>
#include <cstdint>
#include <vector>
#include <string>
#include <atomic>
#include <mutex>
#include <thread>

// Forward-declare ALP types (will link alp.h at build time)
#ifndef ALP_H
typedef long ALP_ID;
#endif

/* DMD-Camera Calibration Transform (affine, from 16-square grid) */
typedef struct DmdCalibTransform {
    double scaleX;          // DMD pixels per camera pixel
    double scaleY;
    double offsetX;         // DMD pixel X offset (translation)
    double offsetY;         // DMD pixel Y offset (translation)
    double rotationDeg;     // Rotation angle in degrees
    double camCenterX;      // Camera center X used during calibration
    double camCenterY;      // Camera center Y used during calibration
    bool   valid;           // Whether calibration has been performed
} DmdCalibTransform;

/* Channel offset (filter-induced translation shift) */
typedef struct DmdChannelOffset {
    double shiftX;          // Shift in camera pixels after filter insertion
    double shiftY;          // Shift in camera pixels after filter insertion
    bool   hasFilter;       // Whether filter is currently installed
    bool   measured;        // Whether channel offset has been measured
} DmdChannelOffset;

/* DMD Operation Result Codes */
#define DMD_OK       0
#define DMD_ERR     -1
#define DMD_NOT_INIT -2

/* DMD Pattern Types for Body-Region Stimulation */
enum class DmdPatternType : int {
    PAT_FULL_FRAME = 0,     // Full DMD illumination
    PAT_CIRCLE,             // Circular spot
    PAT_RECT,               // Rectangular ROI
    PAT_BODY_SEGMENT,       // Single body segment along centerline
    PAT_HEAD_REGION,        // Head region
    PAT_TAIL_REGION,        // Tail region
    PAT_MULTI_SEGMENTS,     // Multiple body segments
    PAT_CUSTOM_BITMAP,      // User-loaded custom bitmap
    PAT_OFF                 // All mirrors off
};

/* DMD LED Wavelength Selection */
enum class DmdLedWavelength : int {
    WL_405NM = 0,   // Violet
    WL_470NM,       // Blue
    WL_525NM,       // Green
    WL_590NM,       // Yellow/Amber (optimal for NpHR)
    WL_625NM,       // Red
    WL_WHITE        // White / broadband
};

/* DMD Configuration & Status */
typedef struct DmdConfig {
    ALP_ID  deviceId;           // ALP device handle
    long    dmdWidth;           // DMD mirror columns (1920)
    long    dmdHeight;          // DMD mirror rows (1080)
    long    bitDepth;           // Bit depth (1 for binary, 8 for grayscale)
    double  frameRateHz;        // Projection frame rate
    long    sequenceLength;     // Number of patterns in sequence
    double  illuminationTimeUs; // Illumination time per pattern (microseconds)
    double  darkTimeUs;         // Dark time between patterns
    bool    triggerInEnable;    // Use external trigger input
    bool    triggerOutEnable;   // Generate trigger output
    DmdLedWavelength wavelength;// Active LED wavelength
    int     ledIntensityPerc;   // LED intensity (0-100)
    DmdPatternType activePattern; // Current pattern type

    // Calibration & channel registration (auto-calibration workflow)
    DmdCalibTransform calib;     // Affine transform from camera to DMD coords
    DmdChannelOffset  chOffset;  // Filter-induced channel offset
} DmdConfig;

/* DMD Stimulation Parameters (body-region targeting) */
typedef struct DmdStimParams {
    DmdPatternType patternType;
    DmdLedWavelength wavelength;
    int     intensityPerc;          // 0-100
    int     spotRadiusPx;           // Radius for circle spot (in DMD pixels)
    int     rectWidthPx;            // Width for rectangle ROI
    int     rectHeightPx;           // Height for rectangle ROI
    int     targetBodySegment;      // Segment index (0..N-1) along centerline
    int     numTargetSegments;      // Number of segments to illuminate
    int     segmentWidthPx;         // Width per segment in DMD pixels
    bool    trackWorm;              // Real-time track & follow the worm
    double  pulseOnMs;              // Stimulation pulse ON duration
    double  pulseOffMs;             // Stimulation pulse OFF duration
    int     numPulses;              // Number of pulses (0 = continuous)
    double  stimDurationS;          // Total stimulation duration (0 = infinite)
    int     offsetFromCenterXPx;    // X offset from worm center
    int     offsetFromCenterYPx;    // Y offset from worm center
} DmdStimParams;

/* DMD Runtime State (thread-safe) */
typedef struct DmdState {
    std::atomic<bool>   initialized;
    std::atomic<bool>   projecting;
    std::atomic<bool>   stimulusActive;
    std::atomic<int>    currentPatternIndex;
    std::atomic<int>    pulsesDelivered;
    std::atomic<double> elapsedTimeS;

    // Real-time worm position for tracking
    std::mutex  posMutex;
    double      wormCenterX;
    double      wormCenterY;
    double      wormHeadX;
    double      wormHeadY;
    double      wormTailX;
    double      wormTailY;
    double      wormAngleRad;
    int         numSegments;
    std::vector<double> segX;
    std::vector<double> segY;

    std::thread stimThread;
    std::atomic<bool> stopStimThread;
} DmdState;

/* =================================================================
 *  DMD Core API -- Device Management
 * ================================================================= */

bool DMD_AllocConfig(DmdConfig** pCfg);
void DMD_FreeConfig(DmdConfig* pCfg);
bool DMD_AllocState(DmdState** pState);
void DMD_FreeState(DmdState* pState);
bool DMD_AllocStimParams(DmdStimParams** pParams);
void DMD_FreeStimParams(DmdStimParams* pParams);

/* =================================================================
 *  DMD Core API -- Device Lifecycle
 * ================================================================= */

int DMD_Init(DmdConfig* cfg, DmdState* state);
int DMD_Release(DmdConfig* cfg, DmdState* state);
int DMD_Inquire(DmdConfig* cfg, DmdState* state);

/* =================================================================
 *  DMD Pattern Generation & Projection
 * ================================================================= */

int DMD_GeneratePattern(
    DmdPatternType type,
    int width, int height,
    int posX, int posY,
    const DmdStimParams* params,
    unsigned char** pBits);

int DMD_GenerateBodySegmentPattern(
    const double* segX, const double* segY,
    int numSegs, int startSeg, int endSeg, int segWidth,
    int width, int height,
    unsigned char** pBits);

int DMD_LoadAndProject(
    const unsigned char* patternBits,
    DmdConfig* cfg,
    DmdState* state);

int DMD_LoadPulsedSequence(
    const unsigned char* onPattern,
    int numPulses,
    double onTimeUs, double offTimeUs,
    DmdConfig* cfg,
    DmdState* state);

int DMD_HaltProjection(DmdConfig* cfg, DmdState* state);
int DMD_StartContinuous(DmdConfig* cfg, DmdState* state);

/* =================================================================
 *  High-Level Stimulation Protocols
 * ================================================================= */

int DMD_SetLedWavelength(DmdConfig* cfg, DmdLedWavelength wl, int intensityPerc);
int DMD_StartStimulation(DmdConfig* cfg, DmdState* state, const DmdStimParams* params);
int DMD_StopStimulation(DmdConfig* cfg, DmdState* state);

void DMD_UpdateWormPosition(
    DmdState* state,
    double centerX, double centerY,
    double headX, double headY,
    double tailX, double tailY,
    double angleRad,
    const double* segX, const double* segY,
    int numSegs);

void DMD_CameraToDmdCoords(
    double camX, double camY,
    int camW, int camH,
    int* dmdX, int* dmdY,
    const DmdConfig* cfg);

/* =================================================================
 *  DMD Auto-Calibration (16-square-pattern grid, mirror-based)
 * ================================================================= */

// Generate bit-packed square pattern at grid position (row,col out of 4x4)
int DMD_GenerateCalibPattern(
    int gridRow, int gridCol,
    int gridCols, int gridRows,
    int squareW, int squareH,
    int dmdWidth, int dmdHeight,
    unsigned char** pBits);

// Project a single calibration square pattern
int DMD_ProjectSingleCalibPattern(
    int gridRow, int gridCol,
    DmdConfig* cfg, DmdState* state);

// Run full auto-calibration: project 16 squares, detect in camera, compute affine
// detectedCamX/Y arrays must be pre-allocated with 16 elements each
int DMD_RunAutoCalibration(
    DmdConfig* cfg, DmdState* state,
    int camWidth, int camHeight,
    void (*grabFrameFn)(void*),     // Callback to grab a fresh camera frame
    void* grabCtx,                  // Context for the grab callback
    int (*detectSquareFn)(void*, int camW, int camH, double* cx, double* cy),
                                    // Callback to detect brightest square in camera frame
    void* detectCtx,                // Context for detection callback
    double* outDetectedCamX,        // Output: 16 camera X positions of detected squares
    double* outDetectedCamY);       // Output: 16 camera Y positions of detected squares

// Compute and store affine calibration from 16 matched point pairs
int DMD_SetCalibrationFromPoints(
    DmdConfig* cfg,
    const double* dmdX, const double* dmdY,   // 16 known DMD positions
    const double* camX, const double* camY,   // 16 detected camera positions
    int numPoints);

// Set the channel offset (translation shift due to filter)
void DMD_SetChannelOffset(DmdConfig* cfg, double shiftX, double shiftY, bool hasFilter);

// Compute calibration quality metric (RMS error in DMD pixels)
double DMD_CalibrationRMSError(const DmdConfig* cfg);

// Draw calibration overlay on camera image (crosshairs at expected projection positions)
void DMD_DrawCalibOverlay(unsigned char* imgData, int imgW, int imgH, int channels, const DmdConfig* cfg);

// Built-in bright square detector for calibration (can be used as detectSquareFn callback)
int DMD_DetectBrightSquare(
    const unsigned char* imgData, int camW, int camH, int channels,
    double* outCx, double* outCy);

inline bool DMD_IsInitialized(const DmdState* state) {
    return state != nullptr && state->initialized.load();
}

inline bool DMD_IsStimulating(const DmdState* state) {
    return state != nullptr && state->stimulusActive.load();
}

inline int DMD_GetPulseCount(const DmdState* state) {
    return state ? state->pulsesDelivered.load() : 0;
}

#endif // TALK2VIALUX_DMD_H_
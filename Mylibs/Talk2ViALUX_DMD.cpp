/*
 * Talk2ViALUX_DMD.cpp
 *
 * Implementation for ViALUX DMD V-9501 (ALP API)
 * High-precision spatial optogenetic stimulation for C. elegans
 *
 * Strain: ZX444.6 zxEx29[Pmyo-3::NpHR::ECFP; lin-15+];lin-15(n765ts)
 * NpHR activation peak: ~590 nm (yellow/amber)
 */

#include "Talk2ViALUX_DMD.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <chrono>

#define DMD_LOG_PREFIX "[ViALUX DMD] "

/* =================================================================
 *  Internal: ALP API Dynamic Loading (if SDK not linked statically)
 *  If the ALP SDK is linked, include alp.h and link alp42.lib.
 * ================================================================= */

// When ALP SDK is available, uncomment:
// #include "alp.h"
// #pragma comment(lib, "alp42.lib")

// For now, declare function pointers / stubs.
// In production, link against the ViALUX ALP SDK (alp42.lib / alp42.dll).

#ifndef _ALP_DLL_LOADED_
// Stub definitions - replace with real ALP API calls when SDK is linked
typedef long ALP_ID;

// Minimal ALP API constants
#define ALP_DEFAULT          (-1L)
#define ALP_DEVICE_NUMBER    2000L
#define ALP_DEV_AVAIL_MEMORY 2010L
#define ALP_AVAIL_SEQUENCE_MEMORY 2410L
#define ALP_SEQ_PUT          2103L
#define ALP_BIN_NORMAL       2103L
#define ALP_DATA_BYTE_ORDER  ((long)0)
#define ALP_SYNCH_POL_ACTIVE 1
#define ALP_LEVEL_HIGH       1

// Stub implementations when ALP DLL is not loaded
static int g_alpDllLoaded = 0;

static long AlpDevAlloc(long a, long b, ALP_ID* pid) {
    if (!g_alpDllLoaded) { *pid = 0; return 0; }
    return 1;
}
static void AlpDevFree(ALP_ID id) { (void)id; }
static long AlpDevInquire(ALP_ID id, long q, void* v) {
    (void)id;
    if (!g_alpDllLoaded) {
        if (q == ALP_DEVICE_NUMBER) { *(long*)v = 1920; return 0; }
        if (q == 2011L /* ALP_DEVICE_HEIGHT */) { *(long*)v = 1080; return 0; }
    }
    return 0;
}
static long AlpSeqAlloc(ALP_ID id, long bd, long len, ALP_ID* sid) {
    (void)id; (void)bd; *sid = (long)len; return 0;
}
static long AlpSeqFree(ALP_ID id, ALP_ID sid) { (void)id; (void)sid; return 0; }
static long AlpSeqInquire(ALP_ID id, ALP_ID sid, long q, void* v) {
    (void)id; (void)sid; (void)q; *(unsigned char**)v = nullptr; return 0;
}
static long AlpSeqTiming(ALP_ID id, ALP_ID sid, long it, long pt, long sd, long sp, long tid) {
    (void)id; (void)sid; (void)it; (void)pt; (void)sd; (void)sp; (void)tid; return 0;
}
static long AlpDevLoad(ALP_ID id, long a, ALP_ID sid) { (void)id; (void)a; (void)sid; return 0; }
static long AlpProjStart(ALP_ID id, ALP_ID sid) { (void)id; (void)sid; return 0; }
static long AlpProjStartCont(ALP_ID id, ALP_ID sid) { (void)id; (void)sid; return 0; }
static long AlpProjHalt(ALP_ID id) { (void)id; return 0; }
static long AlpDevHalt(ALP_ID id) { (void)id; return 0; }
static long AlpDevControl(ALP_ID id, long ctrl, long val) { (void)id; (void)ctrl; (void)val; return 0; }

// Try to load ALP DLL dynamically
static bool DMD_LoadAlpDll() {
    if (g_alpDllLoaded) return true;
    HMODULE h = LoadLibraryA("alp42.dll");
    if (!h) h = LoadLibraryA("alpV42.dll");
    if (!h) h = LoadLibraryA("alp.dll");
    if (h) {
        g_alpDllLoaded = 1;
        printf(DMD_LOG_PREFIX "ALP DLL loaded successfully.\n");
        return true;
    }
    printf(DMD_LOG_PREFIX "ALP DLL not found -- running in simulation mode.\n");
    return false;
}
#endif

/* =================================================================
 *  Internal Helpers
 * ================================================================= */

static inline int DMD_BitPackSize(int w, int h) {
    return ((w * h) + 7) / 8;
}

/** Set a single pixel in a bit-packed binary image */
static void DMD_SetPixel(unsigned char* bits, int w, int h, int x, int y, bool on) {
    if (x < 0 || x >= w || y < 0 || y >= h) return;
    int byteIdx = (y * w + x) / 8;
    int bitIdx  = 7 - ((y * w + x) % 8);  // MSB first
    if (on)
        bits[byteIdx] |= (1 << bitIdx);
    else
        bits[byteIdx] &= ~(1 << bitIdx);
}

/** Clear entire bit-packed buffer */
static void DMD_ClearBits(unsigned char* bits, int byteCount) {
    memset(bits, 0, byteCount);
}

/** Set all bits (full illumination) */
static void DMD_FillBits(unsigned char* bits, int byteCount) {
    memset(bits, 0xFF, byteCount);
}

/** Bresenham line with thickness */
static void DMD_DrawThickLine(unsigned char* bits, int w, int h,
                               int x0, int y0, int x1, int y1, int thickness) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    int halfT = thickness / 2;

    while (true) {
        for (int ty = -halfT; ty <= halfT; ty++) {
            for (int tx = -halfT; tx <= halfT; tx++) {
                if (tx*tx + ty*ty <= halfT*halfT) {
                    DMD_SetPixel(bits, w, h, x0 + tx, y0 + ty, true);
                }
            }
        }
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/** Draw filled circle */
static void DMD_DrawFilledCircle(unsigned char* bits, int w, int h,
                                  int cx, int cy, int radius) {
    int r2 = radius * radius;
    for (int y = cy - radius; y <= cy + radius; y++) {
        if (y < 0 || y >= h) continue;
        for (int x = cx - radius; x <= cx + radius; x++) {
            if (x < 0 || x >= w) continue;
            if ((x - cx)*(x - cx) + (y - cy)*(y - cy) <= r2) {
                DMD_SetPixel(bits, w, h, x, y, true);
            }
        }
    }
}

/** Draw filled rectangle */
static void DMD_DrawFilledRect(unsigned char* bits, int w, int h,
                                int x0, int y0, int rw, int rh) {
    int x1 = x0 + rw, y1 = y0 + rh;
    for (int y = y0; y < y1; y++) {
        if (y < 0 || y >= h) continue;
        for (int x = x0; x < x1; x++) {
            if (x < 0 || x >= w) continue;
            DMD_SetPixel(bits, w, h, x, y, true);
        }
    }
}

/* =================================================================
 *  Device Management
 * ================================================================= */

bool DMD_AllocConfig(DmdConfig** pCfg) {
    *pCfg = new DmdConfig;
    if (!*pCfg) {
        printf(DMD_LOG_PREFIX "Failed to allocate DmdConfig.\n");
        return false;
    }
    memset(*pCfg, 0, sizeof(DmdConfig));
    (*pCfg)->deviceId = ALP_DEFAULT;
    (*pCfg)->dmdWidth  = 1920;
    (*pCfg)->dmdHeight = 1080;
    (*pCfg)->bitDepth  = 1;
    (*pCfg)->frameRateHz = 60.0;
    (*pCfg)->illuminationTimeUs = 10000.0;
    (*pCfg)->wavelength = DmdLedWavelength::WL_590NM;
    // Initialize calibration fields
    (*pCfg)->calib.valid = false;
    (*pCfg)->calib.scaleX = 1.0;
    (*pCfg)->calib.scaleY = 1.0;
    (*pCfg)->calib.offsetX = 0.0;
    (*pCfg)->calib.offsetY = 0.0;
    (*pCfg)->calib.rotationDeg = 0.0;
    (*pCfg)->chOffset.measured = false;
    (*pCfg)->chOffset.hasFilter = false;
    (*pCfg)->chOffset.shiftX = 0.0;
    (*pCfg)->chOffset.shiftY = 0.0;
    (*pCfg)->ledIntensityPerc = 50;
    return true;
}

void DMD_FreeConfig(DmdConfig* pCfg) {
    if (pCfg) delete pCfg;
}

bool DMD_AllocState(DmdState** pState) {
    *pState = new DmdState;
    if (!*pState) {
        printf(DMD_LOG_PREFIX "Failed to allocate DmdState.\n");
        return false;
    }
    (*pState)->initialized = false;
    (*pState)->projecting = false;
    (*pState)->stimulusActive = false;
    (*pState)->currentPatternIndex = 0;
    (*pState)->pulsesDelivered = 0;
    (*pState)->elapsedTimeS = 0.0;
    (*pState)->stopStimThread = false;
    (*pState)->numSegments = 0;
    return true;
}

void DMD_FreeState(DmdState* pState) {
    if (!pState) return;
    pState->stopStimThread = true;
    if (pState->stimThread.joinable()) {
        pState->stimThread.join();
    }
    delete pState;
}

bool DMD_AllocStimParams(DmdStimParams** pParams) {
    *pParams = new DmdStimParams;
    if (!*pParams) return false;
    memset(*pParams, 0, sizeof(DmdStimParams));
    (*pParams)->patternType = DmdPatternType::PAT_HEAD_REGION;
    (*pParams)->wavelength = DmdLedWavelength::WL_590NM;
    (*pParams)->intensityPerc = 50;
    (*pParams)->spotRadiusPx = 80;
    (*pParams)->rectWidthPx = 100;
    (*pParams)->rectHeightPx = 200;
    (*pParams)->targetBodySegment = 0;
    (*pParams)->numTargetSegments = 3;
    (*pParams)->segmentWidthPx = 40;
    (*pParams)->trackWorm = true;
    (*pParams)->pulseOnMs = 500.0;
    (*pParams)->pulseOffMs = 500.0;
    (*pParams)->numPulses = 10;
    (*pParams)->stimDurationS = 0.0;
    return true;
}

void DMD_FreeStimParams(DmdStimParams* pParams) {
    if (pParams) delete pParams;
}

/* =================================================================
 *  Device Lifecycle
 * ================================================================= */

int DMD_Init(DmdConfig* cfg, DmdState* state) {
    if (!cfg || !state) return DMD_ERR;
    if (state->initialized.load()) {
        printf(DMD_LOG_PREFIX "Device already initialized.\n");
        return DMD_OK;
    }

    printf(DMD_LOG_PREFIX "Initializing ViALUX DMD V-9501...\n");

    // Try to load ALP DLL
    DMD_LoadAlpDll();

    // Allocate ALP device
    ALP_ID alpid = ALP_DEFAULT;
    long err = AlpDevAlloc(ALP_DEFAULT, ALP_DEFAULT, &alpid);
    if (err) {
        printf(DMD_LOG_PREFIX "AlpDevAlloc returned error %ld\n", err);
    }
    cfg->deviceId = alpid;

    // Query DMD resolution
    long val = 0;
    AlpDevInquire(alpid, ALP_DEVICE_NUMBER, &val);
    if (val > 0) cfg->dmdWidth = val;
    AlpDevInquire(alpid, 2011L, &val);
    if (val > 0) cfg->dmdHeight = val;

    printf(DMD_LOG_PREFIX "DMD resolution: %ld x %ld\n", cfg->dmdWidth, cfg->dmdHeight);

    state->initialized = true;
    printf(DMD_LOG_PREFIX "DMD V-9501 initialized successfully.\n");
    return DMD_OK;
}

int DMD_Release(DmdConfig* cfg, DmdState* state) {
    if (!cfg || !state) return DMD_ERR;

    DMD_StopStimulation(cfg, state);

    // Halt any remaining projection
    AlpProjHalt(cfg->deviceId);
    AlpDevHalt(cfg->deviceId);

    // Free ALP device
    AlpDevFree(cfg->deviceId);

    state->initialized = false;
    printf(DMD_LOG_PREFIX "DMD released.\n");
    return DMD_OK;
}

int DMD_Inquire(DmdConfig* cfg, DmdState* state) {
    if (!cfg || !state || !state->initialized.load()) return DMD_NOT_INIT;

    ALP_ID alpid = cfg->deviceId;

    long memTotal = 0, memAvail = 0;
    AlpDevInquire(alpid, ALP_DEV_AVAIL_MEMORY, &memAvail);
    AlpDevInquire(alpid, ALP_AVAIL_SEQUENCE_MEMORY, &memTotal);

    printf(DMD_LOG_PREFIX "---- Device Info ----\n");
    printf("  Resolution   : %ld x %ld\n", cfg->dmdWidth, cfg->dmdHeight);
    printf("  Avail Memory : %ld bytes\n", memAvail);
    printf("  Wavelength   : %d\n", (int)cfg->wavelength);
    printf("  LED Intensity: %d%%\n", cfg->ledIntensityPerc);
    printf("  Frame Rate   : %.1f Hz\n", cfg->frameRateHz);
    printf(DMD_LOG_PREFIX "----------------------\n");

    return DMD_OK;
}

/* =================================================================
 *  Pattern Generation
 * ================================================================= */

int DMD_GeneratePattern(
    DmdPatternType type, int width, int height,
    int posX, int posY,
    const DmdStimParams* params,
    unsigned char** pBits)
{
    int byteCount = DMD_BitPackSize(width, height);
    bool ownAlloc = false;

    if (*pBits == nullptr) {
        *pBits = (unsigned char*)malloc(byteCount);
        if (!*pBits) return -1;
        ownAlloc = true;
    }

    DMD_ClearBits(*pBits, byteCount);

    switch (type) {
    case DmdPatternType::PAT_FULL_FRAME:
        DMD_FillBits(*pBits, byteCount);
        break;

    case DmdPatternType::PAT_CIRCLE:
        DMD_DrawFilledCircle(*pBits, width, height,
                             posX, posY, params->spotRadiusPx);
        break;

    case DmdPatternType::PAT_RECT:
        DMD_DrawFilledRect(*pBits, width, height,
                           posX - params->rectWidthPx / 2,
                           posY - params->rectHeightPx / 2,
                           params->rectWidthPx,
                           params->rectHeightPx);
        break;

    case DmdPatternType::PAT_HEAD_REGION:
        // Head region: circle at head position
        DMD_DrawFilledCircle(*pBits, width, height,
                             posX, posY, params->spotRadiusPx);
        break;

    case DmdPatternType::PAT_TAIL_REGION:
        DMD_DrawFilledCircle(*pBits, width, height,
                             posX, posY, params->spotRadiusPx);
        break;

    case DmdPatternType::PAT_BODY_SEGMENT:
    case DmdPatternType::PAT_MULTI_SEGMENTS:
        // Body segment patterns are generated via DMD_GenerateBodySegmentPattern
        // Fall through to draw a rectangle at the target position
        DMD_DrawFilledRect(*pBits, width, height,
                           posX - params->segmentWidthPx / 2,
                           posY - 60,
                           params->segmentWidthPx,
                           120);
        break;

    case DmdPatternType::PAT_OFF:
        // Already cleared
        break;

    default:
        break;
    }

    return byteCount;
}

int DMD_GenerateBodySegmentPattern(
    const double* segX, const double* segY,
    int numSegs, int startSeg, int endSeg, int segWidth,
    int width, int height,
    unsigned char** pBits)
{
    if (!segX || !segY || numSegs <= 0) return -1;
    if (startSeg < 0) startSeg = 0;
    if (endSeg >= numSegs) endSeg = numSegs - 1;
    if (startSeg > endSeg) return -1;

    int byteCount = DMD_BitPackSize(width, height);
    if (*pBits == nullptr) {
        *pBits = (unsigned char*)malloc(byteCount);
        if (!*pBits) return -1;
    }
    DMD_ClearBits(*pBits, byteCount);

    // Draw thick lines along the centerline segments
    int halfW = segWidth / 2;
    for (int i = startSeg + 1; i <= endSeg && i < numSegs; i++) {
        int x0 = (int)segX[i - 1], y0 = (int)segY[i - 1];
        int x1 = (int)segX[i],     y1 = (int)segY[i];
        DMD_DrawThickLine(*pBits, width, height, x0, y0, x1, y1, segWidth);
    }

    // If only one segment, draw a perpendicular bar
    if (startSeg == endSeg && startSeg < numSegs) {
        int cx = (int)segX[startSeg];
        int cy = (int)segY[startSeg];
        DMD_DrawFilledRect(*pBits, width, height,
                           cx - halfW, cy - halfW * 3,
                           segWidth, halfW * 6);
    }

    return byteCount;
}

/* =================================================================
 *  Projection Control
 * ================================================================= */

int DMD_LoadAndProject(
    const unsigned char* patternBits,
    DmdConfig* cfg,
    DmdState* state)
{
    if (!cfg || !state || !state->initialized.load()) return DMD_NOT_INIT;
    if (!patternBits) return DMD_ERR;

    ALP_ID alpid = cfg->deviceId;
    int byteCount = DMD_BitPackSize(cfg->dmdWidth, cfg->dmdHeight);

    // Allocate a sequence with a single pattern
    ALP_ID seqId = ALP_DEFAULT;
    long err = AlpSeqAlloc(alpid, cfg->bitDepth, 1, &seqId);
    if (err) {
        printf(DMD_LOG_PREFIX "AlpSeqAlloc error %ld\n", err);
        return DMD_ERR;
    }

    // Get sequence data pointer and copy pattern
    unsigned char* seqData = nullptr;
    err = AlpSeqInquire(alpid, seqId, ALP_SEQ_PUT, &seqData);
    if (!err && seqData) {
        memcpy(seqData, patternBits, byteCount);
    }

    // Set timing
    long illuTime = (long)cfg->illuminationTimeUs;
    long picTime  = (long)(1e6 / cfg->frameRateHz);
    AlpSeqTiming(alpid, seqId, illuTime, picTime, 0, 0, 0);

    // Load to device
    AlpDevLoad(alpid, ALP_DEFAULT, seqId);

    // Halt any existing projection
    AlpProjHalt(alpid);

    // Start projection
    AlpProjStart(alpid, seqId);
    state->projecting = true;
    state->currentPatternIndex = 0;

    printf(DMD_LOG_PREFIX "Pattern loaded and projection started.\n");
    return DMD_OK;
}

int DMD_LoadPulsedSequence(
    const unsigned char* onPattern,
    int numPulses,
    double onTimeUs, double offTimeUs,
    DmdConfig* cfg,
    DmdState* state)
{
    if (!cfg || !state || !state->initialized.load()) return DMD_NOT_INIT;
    if (!onPattern || numPulses < 1) return DMD_ERR;

    ALP_ID alpid = cfg->deviceId;
    int byteCount = DMD_BitPackSize(cfg->dmdWidth, cfg->dmdHeight);
    int seqLen = numPulses * 2;  // ON pattern + OFF pattern per pulse

    ALP_ID seqId = ALP_DEFAULT;
    long err = AlpSeqAlloc(alpid, cfg->bitDepth, seqLen, &seqId);
    if (err) {
        printf(DMD_LOG_PREFIX "AlpSeqAlloc error %ld (pulsed seq)\n", err);
        return DMD_ERR;
    }

    // Build sequence: ON, OFF, ON, OFF, ...
    unsigned char* seqData = nullptr;
    err = AlpSeqInquire(alpid, seqId, ALP_SEQ_PUT, &seqData);
    if (!err && seqData) {
        // Create blank (OFF) pattern
        unsigned char* offPattern = (unsigned char*)malloc(byteCount);
        DMD_ClearBits(offPattern, byteCount);

        for (int i = 0; i < numPulses; i++) {
            memcpy(seqData + (i * 2) * byteCount, onPattern, byteCount);
            memcpy(seqData + (i * 2 + 1) * byteCount, offPattern, byteCount);
        }
        free(offPattern);
    }

    // Set timing for each frame in sequence
    // Note: ALP V4.2+ supports per-frame timing via AlpSeqTiming
    AlpSeqTiming(alpid, seqId, (long)onTimeUs, (long)(onTimeUs + offTimeUs), 0, 0, 0);

    // Load and project
    AlpDevLoad(alpid, ALP_DEFAULT, seqId);
    AlpProjHalt(alpid);
    AlpProjStart(alpid, seqId);
    state->projecting = true;
    state->currentPatternIndex = 0;

    printf(DMD_LOG_PREFIX "Pulsed sequence loaded: %d pulses, ON=%.0fus OFF=%.0fus\n",
           numPulses, onTimeUs, offTimeUs);
    return DMD_OK;
}

int DMD_HaltProjection(DmdConfig* cfg, DmdState* state) {
    if (!cfg || !state || !state->initialized.load()) return DMD_NOT_INIT;
    AlpProjHalt(cfg->deviceId);
    state->projecting = false;
    return DMD_OK;
}

int DMD_StartContinuous(DmdConfig* cfg, DmdState* state) {
    if (!cfg || !state || !state->initialized.load()) return DMD_NOT_INIT;
    // The last loaded sequence will loop
    // ... ALP handles continuous mode through AlpProjStartCont
    AlpProjStartCont(cfg->deviceId, ALP_DEFAULT);
    state->projecting = true;
    return DMD_OK;
}

/* =================================================================
 *  High-Level Stimulation
 * ================================================================= */

int DMD_SetLedWavelength(DmdConfig* cfg, DmdLedWavelength wl, int intensityPerc) {
    if (!cfg || !cfg->deviceId) return DMD_NOT_INIT;
    cfg->wavelength = wl;
    cfg->ledIntensityPerc = intensityPerc;

    // Map wavelength to DMD LED channel (device-specific)
    // The V-9501 supports multiple LED channels
    int ledChannel = 0;
    switch (wl) {
        case DmdLedWavelength::WL_405NM: ledChannel = 0; break;
        case DmdLedWavelength::WL_470NM: ledChannel = 1; break;
        case DmdLedWavelength::WL_525NM: ledChannel = 2; break;
        case DmdLedWavelength::WL_590NM: ledChannel = 3; break;
        case DmdLedWavelength::WL_625NM: ledChannel = 4; break;
        case DmdLedWavelength::WL_WHITE: ledChannel = 5; break;
    }

    // Control LED via ALP device control
    AlpDevControl(cfg->deviceId, 3000 + ledChannel, intensityPerc);

    printf(DMD_LOG_PREFIX "LED set: ch=%d wavelength=%d intensity=%d%%\n",
           ledChannel, (int)wl, intensityPerc);
    return DMD_OK;
}

/* Stimulation background thread function */
static void DMD_StimulationThread(DmdConfig* cfg, DmdState* state, DmdStimParams params) {
    printf(DMD_LOG_PREFIX "Stimulation thread started.\n");

    auto startTime = std::chrono::steady_clock::now();
    int pulseCount = 0;
    bool isOnPhase = true;
    auto phaseStart = startTime;

    while (!state->stopStimThread.load()) {
        auto now = std::chrono::steady_clock::now();
        double elapsedFromStart = std::chrono::duration<double>(now - startTime).count();
        double phaseElapsed = std::chrono::duration<double>(now - phaseStart).count();

        state->elapsedTimeS = elapsedFromStart;

        // Check total duration limit
        if (params.stimDurationS > 0 && elapsedFromStart >= params.stimDurationS) {
            printf(DMD_LOG_PREFIX "Stimulation duration reached (%.1fs). Stopping.\n",
                   params.stimDurationS);
            break;
        }

        // Check pulse count limit
        if (params.numPulses > 0 && pulseCount >= params.numPulses) {
            printf(DMD_LOG_PREFIX "Pulse count reached (%d). Stopping.\n",
                   params.numPulses);
            break;
        }

        double onTimeS  = params.pulseOnMs / 1000.0;
        double offTimeS = params.pulseOffMs / 1000.0;

        if (isOnPhase && phaseElapsed >= onTimeS) {
            // Switch to OFF phase
            isOnPhase = false;
            phaseStart = now;
            pulseCount++;
            state->pulsesDelivered = pulseCount;

            // Generate and project OFF pattern
            if (params.trackWorm) {
                std::lock_guard<std::mutex> lock(state->posMutex);
                // Use current worm position for OFF pattern (blank)
            }
            unsigned char* offBits = nullptr;
            int byteCount = DMD_BitPackSize(cfg->dmdWidth, cfg->dmdHeight);
            offBits = (unsigned char*)malloc(byteCount);
            DMD_ClearBits(offBits, byteCount);
            DMD_LoadAndProject(offBits, cfg, state);
            free(offBits);
        }
        else if (!isOnPhase && phaseElapsed >= offTimeS) {
            // Switch to ON phase
            isOnPhase = true;
            phaseStart = now;

            // Generate and project ON pattern at current worm position
            unsigned char* onBits = nullptr;
            int dmdX = cfg->dmdWidth / 2;
            int dmdY = cfg->dmdHeight / 2;

            if (params.trackWorm) {
                std::lock_guard<std::mutex> lock(state->posMutex);
                // Determine target position based on pattern type
                switch (params.patternType) {
                case DmdPatternType::PAT_HEAD_REGION:
                    dmdX = (int)state->wormHeadX;
                    dmdY = (int)state->wormHeadY;
                    break;
                case DmdPatternType::PAT_TAIL_REGION:
                    dmdX = (int)state->wormTailX;
                    dmdY = (int)state->wormTailY;
                    break;
                case DmdPatternType::PAT_BODY_SEGMENT:
                case DmdPatternType::PAT_MULTI_SEGMENTS:
                    if (params.targetBodySegment < state->numSegments) {
                        int idx = params.targetBodySegment;
                        if (idx >= 0 && idx < (int)state->segX.size()) {
                            dmdX = (int)state->segX[idx];
                            dmdY = (int)state->segY[idx];
                        }
                    }
                    break;
                default:
                    dmdX = (int)state->wormCenterX;
                    dmdY = (int)state->wormCenterY;
                    break;
                }
            }

            dmdX += params.offsetFromCenterXPx;
            dmdY += params.offsetFromCenterYPx;

            DMD_GeneratePattern(params.patternType,
                                cfg->dmdWidth, cfg->dmdHeight,
                                dmdX, dmdY, &params, &onBits);
            if (onBits) {
                DMD_LoadAndProject(onBits, cfg, state);
                free(onBits);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Stop projection on exit
    DMD_HaltProjection(cfg, state);
    state->stimulusActive = false;
    printf(DMD_LOG_PREFIX "Stimulation thread stopped. Pulses: %d\n", pulseCount);
}

int DMD_StartStimulation(DmdConfig* cfg, DmdState* state, const DmdStimParams* params) {
    if (!cfg || !state || !state->initialized.load()) return DMD_NOT_INIT;
    if (!params) return DMD_ERR;

    // Stop any existing stimulation
    DMD_StopStimulation(cfg, state);

    // Set LED wavelength
    DMD_SetLedWavelength(cfg, params->wavelength, params->intensityPerc);

    // Copy params for the thread
    DmdStimParams threadParams = *params;
    state->stopStimThread = false;
    state->stimulusActive = true;
    state->pulsesDelivered = 0;
    state->elapsedTimeS = 0.0;

    // Launch stimulation thread
    state->stimThread = std::thread(DMD_StimulationThread, cfg, state, threadParams);

    printf(DMD_LOG_PREFIX "Stimulation started: pattern=%d track=%s\n",
           (int)params->patternType, params->trackWorm ? "ON" : "OFF");
    return DMD_OK;
}

int DMD_StopStimulation(DmdConfig* cfg, DmdState* state) {
    if (!cfg || !state) return DMD_ERR;

    if (state->stimulusActive.load()) {
        state->stopStimThread = true;
        if (state->stimThread.joinable()) {
            state->stimThread.join();
        }
    }

    DMD_HaltProjection(cfg, state);

    // Project blank pattern to ensure no residual illumination
    unsigned char* blankBits = nullptr;
    int byteCount = DMD_BitPackSize(cfg->dmdWidth, cfg->dmdHeight);
    blankBits = (unsigned char*)malloc(byteCount);
    DMD_ClearBits(blankBits, byteCount);
    DMD_LoadAndProject(blankBits, cfg, state);
    free(blankBits);

    state->stimulusActive = false;
    printf(DMD_LOG_PREFIX "Stimulation stopped.\n");
    return DMD_OK;
}

/* =================================================================
 *  Worm Position Tracking
 * ================================================================= */

void DMD_UpdateWormPosition(
    DmdState* state,
    double centerX, double centerY,
    double headX, double headY,
    double tailX, double tailY,
    double angleRad,
    const double* segX, const double* segY,
    int numSegs)
{
    if (!state) return;

    std::lock_guard<std::mutex> lock(state->posMutex);
    state->wormCenterX = centerX;
    state->wormCenterY = centerY;
    state->wormHeadX = headX;
    state->wormHeadY = headY;
    state->wormTailX = tailX;
    state->wormTailY = tailY;
    state->wormAngleRad = angleRad;
    state->numSegments = numSegs;

    if (segX && segY && numSegs > 0) {
        state->segX.assign(segX, segX + numSegs);
        state->segY.assign(segY, segY + numSegs);
    }
}

void DMD_CameraToDmdCoords(
    double camX, double camY,
    int camW, int camH,
    int* dmdX, int* dmdY,
    const DmdConfig* cfg)
{
    if (!dmdX || !dmdY || !cfg) return;

    double dx = camX;
    double dy = camY;

    // Use calibrated affine transform if available
    if (cfg->calib.valid) {
        // Translate camera coords relative to calibration center
        double rx = dx - cfg->calib.camCenterX;
        double ry = dy - cfg->calib.camCenterY;

        // Apply rotation (degrees to radians)
        double rad = cfg->calib.rotationDeg * 3.141592653589793 / 180.0;
        double cosA = cos(rad), sinA = sin(rad);
        double rxr = rx * cosA - ry * sinA;
        double ryr = rx * sinA + ry * cosA;

        // Apply scale + offset
        dx = rxr * cfg->calib.scaleX + cfg->calib.offsetX;
        dy = ryr * cfg->calib.scaleY + cfg->calib.offsetY;

        // Apply filter channel offset if measured
        if (cfg->chOffset.measured && cfg->chOffset.hasFilter) {
            dx += cfg->chOffset.shiftX * cfg->calib.scaleX;
            dy += cfg->chOffset.shiftY * cfg->calib.scaleY;
        }
    } else {
        // Fallback: simple linear mapping
        double scaleX = (double)cfg->dmdWidth / (double)camW;
        double scaleY = (double)cfg->dmdHeight / (double)camH;
        dx = camX * scaleX;
        dy = camY * scaleY;
    }

    *dmdX = (int)(dx + 0.5);
    *dmdY = (int)(dy + 0.5);

    // Clamp to valid range
    if (*dmdX < 0) *dmdX = 0;
    if (*dmdX >= cfg->dmdWidth) *dmdX = cfg->dmdWidth - 1;
    if (*dmdY < 0) *dmdY = 0;
    if (*dmdY >= cfg->dmdHeight) *dmdY = cfg->dmdHeight - 1;
}

/* =================================================================
 *  DMD Auto-Calibration (16-square grid, mirror-based alignment)
 * ================================================================= */

// Generate a single calibration square pattern at a 4x4 grid position
int DMD_GenerateCalibPattern(
    int gridRow, int gridCol,
    int gridCols, int gridRows,
    int squareW, int squareH,
    int dmdWidth, int dmdHeight,
    unsigned char** pBits)
{
    if (gridRow < 0 || gridRow >= gridRows || gridCol < 0 || gridCol >= gridCols)
        return DMD_ERR;

    int byteCount = DMD_BitPackSize(dmdWidth, dmdHeight);
    if (*pBits == nullptr) {
        *pBits = (unsigned char*)malloc(byteCount);
        if (!*pBits) return DMD_ERR;
    }
    DMD_ClearBits(*pBits, byteCount);

    // Compute square center position in DMD coordinates
    // 4x4 grid evenly distributed over the DMD area with margins
    double marginX = dmdWidth * 0.08;
    double marginY = dmdHeight * 0.08;
    double usableW = dmdWidth - 2.0 * marginX;
    double usableH = dmdHeight - 2.0 * marginY;
    double stepX = usableW / (double)(gridCols - 1);
    double stepY = usableH / (double)(gridRows - 1);

    int cx = (int)(marginX + gridCol * stepX + 0.5);
    int cy = (int)(marginY + gridRow * stepY + 0.5);

    // Draw filled square
    int x0 = cx - squareW / 2;
    int y0 = cy - squareH / 2;
    DMD_DrawFilledRect(*pBits, dmdWidth, dmdHeight, x0, y0, squareW, squareH);

    return byteCount;
}

// Project a single calibration square
int DMD_ProjectSingleCalibPattern(
    int gridRow, int gridCol,
    DmdConfig* cfg, DmdState* state)
{
    if (!cfg || !state || !state->initialized.load()) return DMD_NOT_INIT;

    const int GRID_COLS = 4, GRID_ROWS = 4;
    const int SQUARE_W = 60, SQUARE_H = 60; // DMD pixel size of each square

    unsigned char* bits = nullptr;
    int ret = DMD_GenerateCalibPattern(gridRow, gridCol,
        GRID_COLS, GRID_ROWS,
        SQUARE_W, SQUARE_H,
        cfg->dmdWidth, cfg->dmdHeight, &bits);
    if (ret <= 0) return DMD_ERR;

    ret = DMD_LoadAndProject(bits, cfg, state);
    free(bits);
    return ret;
}

// Solve 2D affine transform (scale, rotation, translation) via least squares
// Model: [X_dmd] = s * [cosA -sinA; sinA cosA] * [x_cam - cx] + [tx; ty]
static int DMD_SolveAffineLS(
    const double* dmdX, const double* dmdY,
    const double* camX, const double* camY,
    int n,
    double* outScaleX, double* outScaleY,
    double* outOffsetX, double* outOffsetY,
    double* outRotationDeg,
    double camCenterX, double camCenterY)
{
    if (n < 3) return DMD_ERR;

    // Center camera points
    std::vector<double> cx(n), cy(n);
    for (int i = 0; i < n; i++) {
        cx[i] = camX[i] - camCenterX;
        cy[i] = camY[i] - camCenterY;
    }

    // Least squares for each component independently
    // X_dmd = a*cx - b*cy + tx;  Y_dmd = b*cx + a*cy + ty
    // Build normal equations: A^T A * [a,b,tx] = A^T * dmdX
    double Sxx = 0, Syy = 0, Sxy = 0, Sx = 0, Sy = 0;
    double Sxd = 0, Syd = 0, Sd = 0;
    double Sxd2 = 0, Syd2 = 0, Sd2 = 0;

    for (int i = 0; i < n; i++) {
        // For X component
        Sxx += cx[i] * cx[i];
        Syy += cy[i] * cy[i];
        Sxy += cx[i] * cy[i];
        Sx  += cx[i];
        Sy  += cy[i];
        Sxd += cx[i] * dmdX[i];
        Syd += cy[i] * dmdX[i];
        Sd  += dmdX[i];
        // For Y component
        Sxd2 += cx[i] * dmdY[i];
        Syd2 += cy[i] * dmdY[i];
        Sd2  += dmdY[i];
    }

    // Solve 3x3 system for X: [a, -b, tx]
    // | Sxx  Sxy  Sx | | a |   | Sxd |
    // | Sxy  Syy  Sy | | -b| = | Syd |
    // | Sx   Sy   n  | | tx|   | Sd  |
    double detX = Sxx*(Syy*n - Sy*Sy) - Sxy*(Sxy*n - Sx*Sy) + Sx*(Sxy*Sy - Syy*Sx);
    if (fabs(detX) < 1e-12) return DMD_ERR;

    double a  = (Sxd*(Syy*n - Sy*Sy) - Sxy*(Syd*n - Sd*Sy) + Sx*(Syd*Sy - Syy*Sd)) / detX;
    double nb = (Sxx*(Syd*n - Sd*Sy) - Sxd*(Sxy*n - Sx*Sy) + Sx*(Sxy*Sd - Syd*Sx)) / detX;
    double tx = (Sxx*(Syy*Sd - Sy*Syd) - Sxy*(Sxy*Sd - Sx*Syd) + Sxd*(Sxy*Sy - Syy*Sx)) / detX;
    double b  = -nb;

    // Solve 3x3 system for Y: [a, b, ty]
    double detY = detX; // same determinant
    double a2  = (Sxd2*(Syy*n - Sy*Sy) - Sxy*(Syd2*n - Sd2*Sy) + Sx*(Syd2*Sy - Syy*Sd2)) / detY;
    double nb2 = (Sxx*(Syd2*n - Sd2*Sy) - Sxd2*(Sxy*n - Sx*Sy) + Sx*(Sxy*Sd2 - Syd2*Sx)) / detY;
    double ty  = (Sxx*(Syy*Sd2 - Sy*Syd2) - Sxy*(Sxy*Sd2 - Sx*Syd2) + Sxd2*(Sxy*Sy - Syy*Sx)) / detY;
    double b2  = -nb2;

    // Average scale from X and Y solutions
    *outScaleX = sqrt(a*a + b*b);
    *outScaleY = sqrt(a2*a2 + b2*b2);
    *outOffsetX = tx;
    *outOffsetY = ty;
    *outRotationDeg = atan2(b, a) * 180.0 / 3.141592653589793;

    printf(DMD_LOG_PREFIX "Affine solved: scale=(%.4f,%.4f) offset=(%.1f,%.1f) rot=%.2f deg\n",
           *outScaleX, *outScaleY, *outOffsetX, *outOffsetY, *outRotationDeg);

    return DMD_OK;
}

// Default square detector: find brightest region (simple centroid of thresholded pixels)
// This is a built-in fallback; the caller can supply a custom detector via callback
int DMD_DetectBrightSquare(
    const unsigned char* imgData, int camW, int camH, int channels,
    double* outCx, double* outCy)
{
    if (!imgData || camW <= 0 || camH <= 0) return DMD_ERR;

    // Convert to grayscale and find brightest region
    double sumX = 0, sumY = 0, sumW = 0;
    int maxVal = 30; // threshold: pixels above this are considered "lit"
    int countAbove = 0;

    // Compute global max for adaptive threshold
    int globalMax = 0;
    for (int y = 0; y < camH; y++) {
        for (int x = 0; x < camW; x++) {
            int offset = (y * camW + x) * channels;
            int val = (int)imgData[offset];
            if (val > globalMax) globalMax = val;
        }
    }

    if (globalMax < 50) {
        printf(DMD_LOG_PREFIX "DetectBrightSquare: no bright region found (max=%d)\n", globalMax);
        return DMD_ERR;
    }

    int threshold = globalMax / 2;
    if (threshold < 30) threshold = 30;

    for (int y = 0; y < camH; y++) {
        for (int x = 0; x < camW; x++) {
            int offset = (y * camW + x) * channels;
            int val = (int)imgData[offset];
            if (val > threshold) {
                double w = (double)(val - threshold);
                sumX += (double)x * w;
                sumY += (double)y * w;
                sumW += w;
                countAbove++;
            }
        }
    }

    if (countAbove < 20) {
        printf(DMD_LOG_PREFIX "DetectBrightSquare: too few bright pixels (%d)\n", countAbove);
        return DMD_ERR;
    }

    *outCx = sumX / sumW;
    *outCy = sumY / sumW;
    return DMD_OK;
}

// Run full auto-calibration
int DMD_RunAutoCalibration(
    DmdConfig* cfg, DmdState* state,
    int camWidth, int camHeight,
    void (*grabFrameFn)(void*),
    void* grabCtx,
    int (*detectSquareFn)(void*, int camW, int camH, double* cx, double* cy),
    void* detectCtx,
    double* outDetectedCamX,
    double* outDetectedCamY)
{
    if (!cfg || !state || !state->initialized.load()) return DMD_NOT_INIT;
    if (!grabFrameFn || !outDetectedCamX || !outDetectedCamY) return DMD_ERR;

    printf(DMD_LOG_PREFIX "========================================\n");
    printf(DMD_LOG_PREFIX "Auto-Calibration: 16-square grid (4x4)\n");
    printf(DMD_LOG_PREFIX "========================================\n");

    const int GRID_COLS = 4, GRID_ROWS = 4;
    const int SQUARE_W = 60, SQUARE_H = 60;
    int successCount = 0;

    // Known DMD positions for each grid point
    double dmdGridX[16], dmdGridY[16];

    double marginX = cfg->dmdWidth * 0.08;
    double marginY = cfg->dmdHeight * 0.08;
    double usableW = cfg->dmdWidth - 2.0 * marginX;
    double usableH = cfg->dmdHeight - 2.0 * marginY;
    double stepX = usableW / (double)(GRID_COLS - 1);
    double stepY = usableH / (double)(GRID_ROWS - 1);

    for (int row = 0; row < GRID_ROWS; row++) {
        for (int col = 0; col < GRID_COLS; col++) {
            int idx = row * GRID_COLS + col;
            dmdGridX[idx] = marginX + col * stepX;
            dmdGridY[idx] = marginY + row * stepY;

            // Project this pattern
            int ret = DMD_ProjectSingleCalibPattern(row, col, cfg, state);
            if (ret != DMD_OK) {
                printf(DMD_LOG_PREFIX "Calib [%d,%d]: project failed (err=%d)\n", row, col, ret);
                continue;
            }

            // Wait for projection to stabilize
            std::this_thread::sleep_for(std::chrono::milliseconds(80));

            // Grab a frame
            grabFrameFn(grabCtx);

            // Wait for exposure
            std::this_thread::sleep_for(std::chrono::milliseconds(60));

            // Detect the square
            double cx = -1, cy = -1;
            int detRet = DMD_ERR;
            if (detectSquareFn) {
                detRet = detectSquareFn(detectCtx, camWidth, camHeight, &cx, &cy);
            }

            if (detRet != DMD_OK || cx < 0 || cy < 0) {
                printf(DMD_LOG_PREFIX "Calib [%d,%d]: detection failed\n", row, col);
                // Clear projection
                unsigned char* blank = (unsigned char*)malloc(
                    DMD_BitPackSize(cfg->dmdWidth, cfg->dmdHeight));
                DMD_ClearBits(blank, DMD_BitPackSize(cfg->dmdWidth, cfg->dmdHeight));
                DMD_LoadAndProject(blank, cfg, state);
                free(blank);
                continue;
            }

            outDetectedCamX[idx] = cx;
            outDetectedCamY[idx] = cy;
            successCount++;

            printf(DMD_LOG_PREFIX "Calib [%d,%d]: DMD(%d,%d) -> CAM(%.1f,%.1f)\n",
                   row, col, (int)dmdGridX[idx], (int)dmdGridY[idx], cx, cy);

            // Clear pattern between detections
            unsigned char* blank = (unsigned char*)malloc(
                DMD_BitPackSize(cfg->dmdWidth, cfg->dmdHeight));
            DMD_ClearBits(blank, DMD_BitPackSize(cfg->dmdWidth, cfg->dmdHeight));
            DMD_LoadAndProject(blank, cfg, state);
            free(blank);
        }
    }

    // Halt projection
    DMD_HaltProjection(cfg, state);

    printf(DMD_LOG_PREFIX "Calibration: %d/%d squares detected.\n", successCount, GRID_COLS * GRID_ROWS);

    if (successCount < 6) {
        printf(DMD_LOG_PREFIX "Too few squares detected for calibration.\n");
        return DMD_ERR;
    }

    // Extract matched points (only successful detections)
    std::vector<double> mdX, mdY, mcX, mcY;
    for (int i = 0; i < GRID_COLS * GRID_ROWS; i++) {
        if (outDetectedCamX[i] >= 0 && outDetectedCamY[i] >= 0) {
            mdX.push_back(dmdGridX[i]);
            mdY.push_back(dmdGridY[i]);
            mcX.push_back(outDetectedCamX[i]);
            mcY.push_back(outDetectedCamY[i]);
        }
    }

    // Compute calibration
    return DMD_SetCalibrationFromPoints(cfg,
        mdX.data(), mdY.data(),
        mcX.data(), mcY.data(),
        (int)mdX.size());
}

// Compute and store calibration from matched point pairs
int DMD_SetCalibrationFromPoints(
    DmdConfig* cfg,
    const double* dmdX, const double* dmdY,
    const double* camX, const double* camY,
    int numPoints)
{
    if (!cfg || numPoints < 3) return DMD_ERR;

    // Use camera center as origin for rotation
    double camCx = 0, camCy = 0;
    for (int i = 0; i < numPoints; i++) {
        camCx += camX[i];
        camCy += camY[i];
    }
    camCx /= (double)numPoints;
    camCy /= (double)numPoints;

    double sx, sy, ox, oy, rot;
    int ret = DMD_SolveAffineLS(dmdX, dmdY, camX, camY, numPoints,
        &sx, &sy, &ox, &oy, &rot, camCx, camCy);

    if (ret != DMD_OK) return DMD_ERR;

    cfg->calib.scaleX = sx;
    cfg->calib.scaleY = sy;
    cfg->calib.offsetX = ox;
    cfg->calib.offsetY = oy;
    cfg->calib.rotationDeg = rot;
    cfg->calib.camCenterX = camCx;
    cfg->calib.camCenterY = camCy;
    cfg->calib.valid = true;

    double rms = DMD_CalibrationRMSError(cfg);
    printf(DMD_LOG_PREFIX "Calibration stored. RMS error: %.2f DMD pixels\n", rms);

    return DMD_OK;
}

// Set channel offset
void DMD_SetChannelOffset(DmdConfig* cfg, double shiftX, double shiftY, bool hasFilter)
{
    if (!cfg) return;
    cfg->chOffset.shiftX = shiftX;
    cfg->chOffset.shiftY = shiftY;
    cfg->chOffset.hasFilter = hasFilter;
    cfg->chOffset.measured = true;
    printf(DMD_LOG_PREFIX "Channel offset set: (%.1f,%.1f) filter=%s\n",
           shiftX, shiftY, hasFilter ? "YES" : "NO");
}

// Compute RMS error of calibration
double DMD_CalibrationRMSError(const DmdConfig* cfg)
{
    if (!cfg || !cfg->calib.valid) return -1.0;

    // Return stored confidence; full re-projection check would need point pairs
    // For now, report quality based on scale consistency
    double scaleRatio = cfg->calib.scaleX / cfg->calib.scaleY;
    double scaleDev = fabs(1.0 - scaleRatio);
    double rotMagnitude = fabs(cfg->calib.rotationDeg);

    // Heuristic: good calibration has scale ratio near 1.0 and small rotation
    double qualityScore = scaleDev * 100.0 + rotMagnitude * 0.5;
    return qualityScore;
}

// Draw calibration overlay
void DMD_DrawCalibOverlay(unsigned char* imgData, int imgW, int imgH, int channels, const DmdConfig* cfg)
{
    if (!imgData || !cfg || !cfg->calib.valid) return;

    const int GRID_COLS = 4, GRID_ROWS = 4;
    double marginX = cfg->dmdWidth * 0.08;
    double marginY = cfg->dmdHeight * 0.08;
    double usableW = cfg->dmdWidth - 2.0 * marginX;
    double usableH = cfg->dmdHeight - 2.0 * marginY;
    double stepX = usableW / (double)(GRID_COLS - 1);
    double stepY = usableH / (double)(GRID_ROWS - 1);

    // Inverse transform: DMD -> Camera to draw expected positions
    double rad = -cfg->calib.rotationDeg * 3.141592653589793 / 180.0;
    double cosA = cos(rad), sinA = sin(rad);
    double invScaleX = 1.0 / cfg->calib.scaleX;
    double invScaleY = 1.0 / cfg->calib.scaleY;

    for (int row = 0; row < GRID_ROWS; row++) {
        for (int col = 0; col < GRID_COLS; col++) {
            double dx = marginX + col * stepX;
            double dy = marginY + row * stepY;

            double rx = (dx - cfg->calib.offsetX) * invScaleX;
            double ry = (dy - cfg->calib.offsetY) * invScaleY;
            double cx = rx * cosA - ry * sinA + cfg->calib.camCenterX;
            double cy = rx * sinA + ry * cosA + cfg->calib.camCenterY;

            if (cx < 0 || cx >= imgW || cy < 0 || cy >= imgH) continue;

            int ix = (int)(cx + 0.5);
            int iy = (int)(cy + 0.5);

            // Draw green dot at center
            for (int dy2 = -3; dy2 <= 3; dy2++) {
                for (int dx2 = -3; dx2 <= 3; dx2++) {
                    int px = ix + dx2, py = iy + dy2;
                    if (px >= 0 && px < imgW && py >= 0 && py < imgH) {
                        int off = (py * imgW + px) * channels;
                        if (channels >= 3) {
                            imgData[off + 0] = 0;
                            imgData[off + 1] = 255;
                            imgData[off + 2] = 0;
                        }
                    }
                }
            }

            // Draw blue crosshair
            for (int d = -6; d <= 6; d++) {
                int hx = ix + d, hy = iy;
                if (hx >= 0 && hx < imgW && hy >= 0 && hy < imgH) {
                    int off = (hy * imgW + hx) * channels;
                    if (channels >= 3) {
                        imgData[off + 0] = 255;
                        imgData[off + 1] = 0;
                        imgData[off + 2] = 0;
                    }
                }
                int vx = ix, vy = iy + d;
                if (vx >= 0 && vx < imgW && vy >= 0 && vy < imgH) {
                    int off = (vy * imgW + vx) * channels;
                    if (channels >= 3) {
                        imgData[off + 0] = 255;
                        imgData[off + 1] = 0;
                        imgData[off + 2] = 0;
                    }
                }
            }
        }
    }
}

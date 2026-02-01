#ifndef DISPLAY_H
#define DISPLAY_H

// Display resolution
#define TFT_HOR_RES 480
#define TFT_VER_RES 480

// Debug flag
#ifndef DEBUG
#define DEBUG true
#endif

void initDisplay();
void initLVGL();
void initTouch();
uint32_t getAndResetFlushCount();

#endif // DISPLAY_H

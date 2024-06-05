#ifndef DISPLAY_H
#define DISPLAY_H

/*Set to your screen resolution*/
#define TFT_HOR_RES 480
#define TFT_VER_RES 480

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))

void initDisplay();
void flushDisplay();

#endif // DISPLAY_H
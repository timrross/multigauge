#ifndef NEEDLE_GLIDE_H
#define NEEDLE_GLIDE_H

// Needle glide: pure logic with no Arduino or LVGL calls, so it can be checked on a PC.
//
// The sensor node sends boost about 9 times a second. Jumping the needle to each
// reading makes it move in 9 steps a second, however fast the screen draws. This
// moves it a little on every screen update instead, at a steady speed, timed to
// reach each reading shortly after the next one is due.
//
// The cost is lag: the needle trails the readings by one reading interval plus
// GLIDE_MARGIN_MS, about 0.15 s at 9 readings a second.
//
// The time between readings is measured, not fixed, so the lag shrinks by itself
// if the sensor node is changed to send boost more often.

#include <stdint.h>

// A reading has to differ from where the needle is heading by this much before the
// needle moves. Keeps sensor noise from making it shiver (and redrawing the screen).
#define GLIDE_DEADBAND_MBAR         5
// The needle is timed to reach a reading this long after the next reading is due.
// It covers a reading turning up late and the wait for the next screen update
// (about 25 ms). Too small and the needle arrives early and stops to wait, which
// shows as a stutter. Larger only adds lag.
#define GLIDE_MARGIN_MS             45.0f
// Assumed time between readings until some have been timed.
#define GLIDE_DEFAULT_INTERVAL_MS   110.0f
// Gaps outside this range are not readings arriving normally (a lost link, a
// restart), so they are left out of the timing.
#define GLIDE_MIN_INTERVAL_MS       10
#define GLIDE_MAX_INTERVAL_MS       300
// Shortest glide, for a reading that was noticed very late.
#define GLIDE_MIN_MS                10.0f

class NeedleGlide {
 public:
  // Call on every screen update.
  //   reading:   the latest boost reading, in mbar
  //   readingMs: when that reading arrived, on the same clock as nowMs. It is only
  //              used for timing, so it may stay unchanged if no readings arrive.
  //   nowMs:     the current time
  // Returns where to draw the needle, in mbar.
  float update(int reading, uint32_t readingMs, uint32_t nowMs) {
    if (!started_) {
      started_ = true;
      target_ = reading;
      from_ = pos_ = (float)reading;
      lastReadingMs_ = readingMs;
      return pos_;
    }

    // Carry on along the current glide. This comes first: if a new reading turns up
    // below, the needle sets off for it from here, and this update still moves.
    if (pos_ != (float)target_) {
      float done = (float)(nowMs - startMs_) / glideMs_;
      pos_ = (done >= 1.0f) ? (float)target_ : from_ + ((float)target_ - from_) * done;
    }

    // Time the readings as they arrive, as a running average.
    bool fresh = readingMs != lastReadingMs_;
    if (fresh) {
      uint32_t gap = readingMs - lastReadingMs_;
      if (gap >= GLIDE_MIN_INTERVAL_MS && gap <= GLIDE_MAX_INTERVAL_MS) {
        intervalMs_ += ((float)gap - intervalMs_) * 0.25f;
      }
      lastReadingMs_ = readingMs;
    }

    // Head for a new reading, starting from wherever the needle has got to.
    int change = reading - target_;
    if (change >= GLIDE_DEADBAND_MBAR || change <= -GLIDE_DEADBAND_MBAR) {
      from_ = pos_;
      target_ = reading;
      startMs_ = nowMs;
      // The arrival time is counted from when the reading came in, not from now.
      // It was noticed a little later than that (this runs once per screen update),
      // and counting from now would make the needle's speed wander.
      int32_t sinceArrival = fresh ? (int32_t)(nowMs - readingMs) : 0;
      if (sinceArrival < 0) sinceArrival = 0;  // a reading stamped later than now: ignore the stamp
      glideMs_ = intervalMs_ + GLIDE_MARGIN_MS - (float)sinceArrival;
      if (glideMs_ < GLIDE_MIN_MS) glideMs_ = GLIDE_MIN_MS;
    }
    return pos_;
  }

  // The measured time between readings.
  float intervalMs() const { return intervalMs_; }

 private:
  bool started_ = false;
  int target_ = 0;                 // the reading the needle is heading for
  float from_ = 0;                 // where the needle was when it set off
  float pos_ = 0;                  // where the needle is now
  uint32_t startMs_ = 0;           // when it set off
  float glideMs_ = GLIDE_DEFAULT_INTERVAL_MS + GLIDE_MARGIN_MS;
  float intervalMs_ = GLIDE_DEFAULT_INTERVAL_MS;
  uint32_t lastReadingMs_ = 0;
};

#endif // NEEDLE_GLIDE_H

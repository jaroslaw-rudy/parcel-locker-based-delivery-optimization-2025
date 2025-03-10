#pragma once
#include <ctime>

double timer(struct timespec t) {
  struct timespec s;
  double ta, tb;

  clock_gettime(CLOCK_REALTIME, & s);

  ta = t.tv_sec + 0.000000001 * t.tv_nsec;
  tb = s.tv_sec + 0.000000001 * s.tv_nsec;

  return tb - ta;
}
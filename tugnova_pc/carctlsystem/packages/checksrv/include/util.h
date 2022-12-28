#ifndef util_h
#define util_h
//複数のソースで使用する処理を記載する
#include <ctime>

// 現在時刻をミリ秒で取得する
double calc_time()
{
  struct ::timespec getTime;
  clock_gettime(CLOCK_MONOTONIC, &getTime);
  return (getTime.tv_sec + getTime.tv_nsec * 1e-9) * 1000;
}

#endif // util
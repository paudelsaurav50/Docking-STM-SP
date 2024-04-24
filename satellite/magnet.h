#ifndef _TAMARIW_MAGNET_H_
#define _TAMARIW_MAGNET_H_

enum magnet_idx
{
  MAGNET_IDX_0,
  MAGNET_IDX_1,
  MAGNET_IDX_2,
  MAGNET_IDX_3,
  MAGNET_IDX_ALL,
};

namespace magnet
{
  void init(void);
  void stop(const magnet_idx idx);
  void actuate(const magnet_idx idx, const float dc);
  float get_current(const magnet_idx);
  void get_current(float current[4]);
}

#endif // magnet.h

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
}

#endif // magnet.h

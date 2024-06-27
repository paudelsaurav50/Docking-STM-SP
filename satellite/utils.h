// Generally useful utility files for TAMARIW mission
// 2024-01-19

#ifndef _TAMAWIW_UTILS_H_
#define _TAMARIW_UTILS_H_

#define R2D 57.2957795131
#define D2R 0.01745329251

int sign(const float in);

inline void swap(float *a, float *b)
{
  float temp = *a;
  *a = *b;
  *b = temp;
}

#endif // utils.h

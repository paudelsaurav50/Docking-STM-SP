// Generally useful utility functions for TAMARIW mission
// 2024-01-19

#ifndef _TAMAWIW_UTILS_H_
#define _TAMARIW_UTILS_H_

#define R2D 57.2957795131
#define D2R 0.01745329251

int sign(const float in);
bool float_to_bool(float val);
inline void swap(float *a, float *b);
float winsorized_mean(const float x[4]);
float winsorized_mean(const int x[4]);

#endif // utils.h

#ifndef __ULTRASONICSENSOR_H
#define __ULTRASONICSENSOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ULTRASONICSENSOR_Init(void);
void ULTRASONICSENSOR_App(void);
void ULTRASONICSENSOR_ReceiceIT(void);

#ifdef __cplusplus
}
#endif

#endif  /* __ULTRASONICSENSOR_H */
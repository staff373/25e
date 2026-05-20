#ifndef APP_TURN_H
#define APP_TURN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    TURN_DIR_LEFT = -1,
    TURN_DIR_NONE = 0,
    TURN_DIR_RIGHT = 1
} Turn_Direction_t;

void Turn_Init(void);
void Turn_Poll(void);
uint8_t Turn_Start(int8_t direction);
void Turn_Stop(void);
uint8_t Turn_IsActive(void);
uint8_t Turn_WasLastTimeout(void);
int8_t Turn_GetDirection(void);
float Turn_GetProgressDeg(void);
float Turn_GetTargetAngleDeg(void);
uint8_t Turn_SetParam(const char *name, float value);
uint8_t Turn_GetParam(const char *name, float *value);
void Turn_FormatStatus(char *buffer, size_t buffer_size, const char *prefix);

#ifdef __cplusplus
}
#endif

#endif /* APP_TURN_H */

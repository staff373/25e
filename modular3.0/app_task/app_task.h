#ifndef APP_TASK_H
#define APP_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

typedef enum
{
    TASK_MODE_NONE = 0,
    TASK_MODE_Q1_TRACK = 1
} Task_Mode_t;

typedef enum
{
    TASK_STATE_IDLE = 0,
    TASK_STATE_SELECTED,
    TASK_STATE_RUNNING,
    TASK_STATE_FINISHED,
    TASK_STATE_STOPPED,
    TASK_STATE_ERROR
} Task_State_t;

typedef enum
{
    TASK_STOP_REASON_NONE = 0,
    TASK_STOP_REASON_USER,
    TASK_STOP_REASON_COMPLETE,
    TASK_STOP_REASON_START_FAIL,
    TASK_STOP_REASON_CHILD_STOPPED,
    TASK_STOP_REASON_UNSUPPORTED
} Task_StopReason_t;

void Task_Init(void);
void Task_Poll(void);

uint8_t Task_Select(Task_Mode_t mode);
uint8_t Task_SelectQuestion(uint8_t question_id);
uint8_t Task_Start(void);
void Task_Stop(void);
void Task_Reset(void);

Task_Mode_t Task_GetMode(void);
const char *Task_GetModeName(void);
Task_State_t Task_GetState(void);
const char *Task_GetStateName(void);
Task_StopReason_t Task_GetStopReason(void);
const char *Task_GetStopReasonName(void);
uint32_t Task_GetElapsedMs(void);
void Task_FormatStatus(char *buffer, size_t buffer_size, const char *prefix);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASK_H */

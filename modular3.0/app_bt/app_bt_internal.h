#ifndef APP_BT_INTERNAL_H
#define APP_BT_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

void BT_ProcessLine(char *line);
void BT_SendCaptureNotices(void);
void BT_SendVisionStream(void);
void BT_WriteText(const char *text);
void BT_WriteLine(const char *text);

#ifdef __cplusplus
}
#endif

#endif /* APP_BT_INTERNAL_H */

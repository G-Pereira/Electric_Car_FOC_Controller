#ifndef _SD_WR_H_
#define _SD_WR_H_


int bufsize (char *buf);
void bufclear (char *buffer);
FRESULT mount_card (FATFS *fs);
FRESULT unmount_card (FATFS *fs);
void card_capacity (DWORD *free_space, DWORD *total_space);
FRESULT create_file (char *filename, char *data, FIL *fil, UINT *bw);
FRESULT read_file (char *buffer, char *filename, FIL *fil, UINT *br);
FRESULT update_file(char *filename, char *data, char *timestamp, char *msec_stamp, FIL *fil, UINT *bw);
char *get_timestamp(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *currentTime, RTC_DateTypeDef *currentDate);

#endif

#define TRUE  1
#define FALSE 0
#define bool BYTE

#include "fatfs.h"
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"
#include "time.h"


/* to find the size of data in the buffer */
int bufsize (char *buf)
{
	int i=0;
	while (*buf++ != '\0') i++;
	return i;
}

void bufclear (char *buffer)  // clear buffer
{
	for (int i=0; i<512; i++)
	{
		buffer[i] = '\0';
	}
}

/* Mount SD Card */
FRESULT mount_card (FATFS *fs)
{
      return f_mount(fs, "", 0);
      //if fresult <>
}

FRESULT unmount_card (FATFS *fs)
{
      return f_mount(fs, "", 1);
}

/*************** Card capacity details ********************/

void card_capacity (DWORD *free_space, DWORD *total_space)//, DWORD *fre_clust)
{
	FATFS *pfs;
	DWORD fre_clust;

	/* Check free space */
	f_getfree("", &fre_clust, &pfs);
	*total_space = ((pfs->n_fatent - 2) * (pfs->csize)); //total number of sectors
	*free_space = (fre_clust * (pfs->csize)); //number of free sectors
}

/**************** The following operation is using f_write and f_read **************************/

FRESULT create_file (char *filename, char *data, FIL *fil, UINT *bw){

	FRESULT fresult;

	/* Create second file with read write access and open it */
	fresult = f_open(fil, filename , FA_CREATE_ALWAYS | FA_WRITE | FA_READ);

	fresult = f_write(fil, data, bufsize(data), bw);

	/* Close file */
	f_close(fil);

	return fresult;
}

FRESULT read_file (char *buffer, char *filename, FIL *fil, UINT *br){

	FRESULT fresult;

	/* Open second file to read */
	fresult = f_open(fil, filename, FA_READ);

	/* Read data from the file
	 * Please see the function details for the arguments */
	f_read (fil, buffer, fil->fsize, br);

	/* Close file */
	f_close(fil);

	//bufclear(buffer);//é capaz de nem fazer sentido isto

	return fresult;
}

/*********************UPDATING an existing file ***************************/
FRESULT update_file(char *filename, char *data, char *timestamp, FIL *fil, UINT *bw)
{
	FRESULT fresult;

	strcat(data,timestamp);
	strcat(data,"\n");

	/* Open the file with write access */
	fresult = f_open(fil, filename, FA_OPEN_ALWAYS | FA_WRITE | FA_READ);


	/* Move to offset to the end of the file */
	fresult = f_lseek(fil, fil->fsize);

	/* write the string to the file */
	fresult = f_puts(data, fil); //pode precisar de um "\n"

	f_close (fil);
	return fresult;
}

char *get_timestamp(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *currentTime, RTC_DateTypeDef *currentDate){

	time_t timestamp;
	struct tm currTime;

	HAL_RTC_GetTime(hrtc, currentTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, currentDate, RTC_FORMAT_BIN);


	currTime.tm_year = currentDate->Year-16;
	currTime.tm_mday = currentDate->Date-6;
	currTime.tm_mon  = currentDate->Month-2;
	currTime.tm_hour = currentTime->Hours+8;
	currTime.tm_min  = currentTime->Minutes-23;
	currTime.tm_sec  = currentTime->Seconds;

	timestamp = mktime(&currTime);
	return asctime(gmtime(&timestamp));

}


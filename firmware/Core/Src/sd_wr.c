#define TRUE  1
#define FALSE 0
#define bool BYTE

#include "fatfs.h"
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"


/* to find the size of data in the buffer */
int bufsize (char *buf)
{
	int i=0;
	while (*buf++ != '\0') i++;
	return i;
}

void bufclear (char *buffer)  // clear buffer
{
	for (int i=0; i<1024; i++)
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

/*************** Card capacity details ********************/
void card_capacity (char *buffer, uint32_t *free_space, uint32_t *total_space, FATFS **pfs, DWORD *fre_clust)
{
	/* Check free space */
	f_getfree("", fre_clust, &(*pfs));

	*total_space = (uint32_t)(((*pfs)->n_fatent - 2) * ((*pfs)->csize) * 0.5);
	sprintf (buffer, "SD CARD Total Size: \t%lu\n", *total_space);
	bufclear(buffer);
	*free_space = (uint32_t)(*fre_clust * ((*pfs)->csize) * 0.5);
	sprintf (buffer, "SD CARD Free Space: \t%lu\n",*free_space);
	//bufclear(buffer);
}

/**************** The following operation is using f_write and f_read **************************/

FRESULT create_file (char *buffer, char *filename, char *data, FIL *fil, UINT *bw){

	FRESULT fresult;

	/* Create second file with read write access and open it */
	fresult = f_open(fil, filename , FA_CREATE_ALWAYS | FA_WRITE | FA_READ);

	/* Writing text */
	strcpy (buffer, data); //pode precisar de um "\n"

	fresult = f_write(fil, buffer, bufsize(buffer), bw);


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
FRESULT update_file(char *buffer, char *filename, char *data, FIL *fil, UINT *bw)
{
	FRESULT fresult;

	/* Open the file with write access */
	fresult = f_open(fil, filename, FA_OPEN_ALWAYS | FA_WRITE);

	/* Move to offset to the end of the file */
	fresult = f_lseek(fil, fil->fsize);

	/* write the string to the file */
	fresult = f_puts(data, fil); //pode precisar de um "\n"

	f_close (fil);

	//bufclear(buffer);

	return fresult;
}

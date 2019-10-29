#ifndef _SD_WR_H_
#define _SD_WR_H_


int bufsize (char *buf);
void bufclear (char *buffer);
FRESULT mount_card (FATFS *fs);
void card_capacity (uint32_t *free_space, uint32_t *total_space, FATFS *pfs, DWORD *fre_clust);
FRESULT create_file (char *filename, char *data, FIL *fil, UINT *bw);
FRESULT read_file (char *buffer, char *filename, FIL *fil, UINT *br);
FRESULT update_file(char *filename, char *data, FIL *fil, UINT *bw);

#endif

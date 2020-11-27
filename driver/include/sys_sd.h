#ifndef _SD_F1C_H
#define _SD_F1C_H

#define SD0 0
#define SD1 1
/* we allocate one of these for every device that we remember */
#define STOR_STRING_LEN 32
typedef struct disk_data_t
{
    struct disk_data_t *next; /* next device */

    /* information about the device -- always good */
    unsigned int totalSectorN;
    unsigned int diskSize; /* disk size in Kbytes */
    int sectorSize;
    char vendor[STOR_STRING_LEN];
    char product[STOR_STRING_LEN];
    char serial[STOR_STRING_LEN];
} DISK_DATA_T;
extern DISK_DATA_T SD_INFO;

int SD_Init(int SDinx);
int SD_Read_in(int SDinx, unsigned int sector, unsigned int count, unsigned char *buff);
int SD_Write_out(int SDinx, unsigned int sector, unsigned int count, unsigned char *buff);
void SD_Demo(void);

#endif

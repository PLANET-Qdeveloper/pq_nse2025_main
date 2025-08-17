#ifndef __FLASH_H__
#define __FLASH_H__

#include "main.h"
#include "w25qx.h"
#include "lfs.h"
#include "logger.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

#define BOOT_COUNT_FILE_NAME "boot_count.txt"
#define LOG_FILE_NAME "log.txt"

#define LFS_THREADSAFE

#define NUMBER_OF_FILES        8                                       // max 32
#define FILE_SIZE            8192
#define FILE_DEBUG            1                                        // Show test file messages, disable for benchmark



typedef struct {
    lfs_file_t file;
    uint32_t pos;
    uint32_t size;
} flash_stream_t;

int init_flash();
int write_flash_log(uint8_t *data, uint32_t size);


int flash_write(uint8_t *data, uint32_t size, const char *filename);
int flash_read(uint8_t *data, uint32_t size, const char *filename);
int flash_filelist(const char *path, char *buffer, uint32_t buffer_size);
int flash_mkdir(const char *path);
int flash_rm(const char *path);
int flash_rmdir(const char *path);
int flash_touch(const char *filename);
int flash_set_log_file(const char *filename);
int flash_stream_open(flash_stream_t *stream, const char *filename);
int flash_stream_read(flash_stream_t *stream, uint8_t *buffer, uint32_t size);
int flash_stream_close(flash_stream_t *stream);
uint32_t flash_get_free_size(void);

#endif



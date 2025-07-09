#include "flash.h"
#include <string.h>
#include <stdio.h>

extern lfs_t lfs;
uint32_t boot_count = 0;
const char* log_file_name = LOG_FILE_NAME;

char fn[32], fn2[32];
uint8_t buffer[FILE_SIZE]={0};
lfs_file_t fp_log;
lfs_file_t fp;



int init_flash(){
    // Initialize XSPI Flash
  if (CSP_XSPI_Init() != HAL_OK) {
        output_log(LOG_LEVEL_ERROR, "XSPI init failed");
        Error_Handler();
    }
    // Read Flash ID
    uint32_t id=0;
    XSPI_ReadID(&id);

    // Read Unique ID
    uint8_t uid[8]={0};
    XSPI_ReadUniqueID(uid);
    int err;
    // Mount filesystem without format
    err = stmlfs_mount(false);
    if (err) {
        stmlfs_mount(true);
    }


    struct lfs_info info;
    stmlfs_stat(BOOT_COUNT_FILE_NAME, &info);


    if ((err = stmlfs_file_open(&fp, BOOT_COUNT_FILE_NAME, LFS_O_RDWR | LFS_O_CREAT)) < 0) {
            output_log(LOG_LEVEL_ERROR, "open failed %d\n", err);
            Error_Handler();
    }

    if ((err = stmlfs_file_open(&fp_log, LOG_FILE_NAME, LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND)) < 0) {
            output_log(LOG_LEVEL_ERROR, "open failed %d\n", err);
            Error_Handler();
    }

    if ((err = stmlfs_file_read(&fp, &boot_count, sizeof(boot_count))) < 0) {
        output_log(LOG_LEVEL_ERROR, "read failed %d\n", err);
        Error_Handler();
    }
    boot_count++;
    stmlfs_file_rewind(&fp);

    stmlfs_file_write(&fp, &boot_count, sizeof(boot_count));

    if ((err = stmlfs_file_close(&fp)) < 0) {  // flush and close the file
        output_log(LOG_LEVEL_ERROR, "close failed %d\n", err);
        Error_Handler();
    }




    struct littlfs_fsstat_t stat;  // Display file system sizes
    stmlfs_fsstat(&stat);

    size_t line_length = sprintf(buffer, "Flash init. id: %08lx, uid: %02x%02x%02x%02x%02x%02x%02x%02x, boot_count: %d\n", id, uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7], boot_count);
    if((err = stmlfs_file_write(&fp_log, buffer, line_length)) < 0) {
        output_log(LOG_LEVEL_ERROR, "write failed %d\n", err);
        Error_Handler();
    }
    line_length = sprintf(buffer, "Flash: blocks %d, block size %d, used %d\n", (int)stat.block_count, (int)stat.block_size, (int)stat.blocks_used);
    if((err = stmlfs_file_write(&fp_log, buffer, line_length)) < 0) {
        output_log(LOG_LEVEL_ERROR, "write failed %d\n", err);
        Error_Handler();
    }

    if ((err = stmlfs_file_close(&fp_log)) < 0) {
        output_log(LOG_LEVEL_ERROR, "close failed %d\n", err);
        Error_Handler();
    }
}

int write_flash_log(uint8_t *data, uint32_t size){
    int comres = stmlfs_file_open(&fp_log, LOG_FILE_NAME, LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND);
    comres += stmlfs_file_write(&fp, data, size);
    comres += stmlfs_file_close(&fp);
    return comres;
}



int flash_write(uint8_t *data, uint32_t size, const char *filename)
{
    lfs_file_t file;
    int err = lfs_file_open(&lfs, &file, filename, LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND);
    if (err < 0) return err;
    
    lfs_ssize_t written = lfs_file_write(&lfs, &file, data, size);
    lfs_file_close(&lfs, &file);
    
    return (written < 0) ? written : 0;
}

int flash_read(uint8_t *data, uint32_t size, const char *filename)
{
    lfs_file_t file;
    int err = lfs_file_open(&lfs, &file, filename, LFS_O_RDONLY);
    if (err < 0) return err;
    
    lfs_ssize_t bytes_read = lfs_file_read(&lfs, &file, data, size);
    lfs_file_close(&lfs, &file);
    
    return (bytes_read < 0) ? bytes_read : 0;
}

int flash_filelist(const char *path, char *buffer, uint32_t buffer_size)
{
    lfs_dir_t dir;
    struct lfs_info info;
    int err = lfs_dir_open(&lfs, &dir, path);
    if (err < 0) return err;
    
    uint32_t offset = 0;
    while (true) {
        err = lfs_dir_read(&lfs, &dir, &info);
        if (err <= 0) break;
        
        uint32_t name_len = strlen(info.name);
        if (offset + name_len + 2 >= buffer_size) break;
        
        strcpy(buffer + offset, info.name);
        offset += name_len;
        buffer[offset++] = '\n';
    }
    
    if (offset > 0) buffer[offset - 1] = '\0';
    else buffer[0] = '\0';
    
    lfs_dir_close(&lfs, &dir);
    return 0;
}

int flash_mkdir(const char *path)
{
    return lfs_mkdir(&lfs, path);
}

int flash_rm(const char *path)
{
    return lfs_remove(&lfs, path);
}

int flash_rmdir(const char *path)
{
    return lfs_remove(&lfs, path);
}

int flash_touch(const char *filename)
{
    lfs_file_t file;
    int err = lfs_file_open(&lfs, &file, filename, LFS_O_CREAT | LFS_O_RDWR);
    if (err < 0) return err;
    
    lfs_file_close(&lfs, &file);
    return 0;
}

static char current_log_file[256] = LOG_FILE_NAME;

int flash_set_log_file(const char *filename)
{
    if (strlen(filename) >= sizeof(current_log_file)) {
        return LFS_ERR_NAMETOOLONG;
    }
    strcpy(current_log_file, filename);
    return 0;
}

int flash_stream_open(flash_stream_t *stream, const char *filename)
{
    int err = lfs_file_open(&lfs, &stream->file, filename, LFS_O_RDONLY);
    if (err < 0) return err;
    
    stream->pos = 0;
    lfs_soff_t size = lfs_file_size(&lfs, &stream->file);
    stream->size = (size < 0) ? 0 : size;
    
    return 0;
}

int flash_stream_read(flash_stream_t *stream, uint8_t *buffer, uint32_t size)
{
    if (stream->pos >= stream->size) {
        return 0;
    }
    
    uint32_t to_read = (stream->pos + size > stream->size) ? 
                       (stream->size - stream->pos) : size;
    
    lfs_ssize_t bytes_read = lfs_file_read(&lfs, &stream->file, buffer, to_read);
    if (bytes_read > 0) {
        stream->pos += bytes_read;
    }
    
    return bytes_read;
}

int flash_stream_close(flash_stream_t *stream)
{
    return lfs_file_close(&lfs, &stream->file);
}


uint32_t flash_get_free_size(void)
{
    struct lfs_fsinfo fsinfo;
    int err = lfs_fs_stat(&lfs, &fsinfo);
    if (err < 0) return 0;
    
    lfs_ssize_t used_blocks = lfs_fs_size(&lfs);
    if (used_blocks < 0) return 0;
    
    uint32_t free_blocks = fsinfo.block_count - used_blocks;
    return free_blocks * fsinfo.block_size;
}
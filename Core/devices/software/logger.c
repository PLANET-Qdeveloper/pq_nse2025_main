#include "logger.h"


int output_log(log_level_t level, const char *format, ...){
    char buffer[256];
    va_list args;
    
    // 可変引数リストを初期化
    va_start(args, format);
    
    // vsnprintfを使って文字列をフォーマット
    int size = vsnprintf(buffer, sizeof(buffer), format, args);
    
    // 終了処理
    va_end(args);
    
    if(level == LOG_LEVEL_DEBUG){
        write_flash_log(buffer, size);
    }else if(level == LOG_LEVEL_INFO){
        write_flash_log(buffer, size);
    }else if(level == LOG_LEVEL_IMPORTANT){
        write_flash_log(buffer, size);
        send_data(buffer, size);
    }else if(level == LOG_LEVEL_WARN){
        write_flash_log(buffer, size);
        send_data(buffer, size);
    }else if(level == LOG_LEVEL_ERROR){
        write_flash_log(buffer, size);
        send_data(buffer, size);
    }
}









#include <stdint.h>

#ifndef ZYCAP_H
#define ZYCAP_H

typedef struct {
        int memfd;
        int fd;
} t_zycap;

typedef struct
{
        char * name;
        char * data;
        uint32_t size;
        uint32_t slots;         //bit x = 1 for slot x 
} t_bitstream;

int Zycap_Init                  (t_zycap * zycap );
int Zycap_Deinit                (t_zycap * zycap );      
int Zycap_Write_Bitstream       (t_zycap * zycap, t_bitstream * bitstream);
int Zycap_Prefetch_Bitstream    (char * bs_name, t_bitstream * bitstream);


#endif
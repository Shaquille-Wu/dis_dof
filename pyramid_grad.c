#include "pyramid_grad.h"
#include <string.h>

void  pyramid_grad_xy_iu8_os16(DIS_PYRAMID const* A,
                               DIS_PYRAMID*       B){
  unsigned int l = 0;
  unsigned int levels = A->level;
  int          i = 0, j = 0;
  memset(B->mem, 0, B->total_buf_size);
  for(l = 0 ; l < levels ; l ++){ 
    int                   src_line_size = A->line_size[l];
    unsigned char const*  src           = A->buf[l] + 
                                          A->pad + 
                                          A->pad * A->line_size[l];
    unsigned int*         dst           = (unsigned int*)(B->buf[l] + 
                                           B->pad * 4 + 
                                           B->pad * B->line_size[l]);
    unsigned int          cur_width     = A->width[l];
    unsigned int          cur_height    = A->height[l];
    for(i = 0 ; i < cur_height ; i ++){
      for(j = 0 ; j < cur_width ; j ++){
        int tl   = src[j - 1 - src_line_size];
        int t    = src[j - src_line_size];
        int tr   = src[j + 1 - src_line_size];
        int r    = src[j + 1];
        int br   = src[j + 1 + src_line_size];
        int b    = src[j + src_line_size];
        int bl   = src[j - 1 + src_line_size];
        int l    = src[j - 1];
        int c    = src[j];
        int x    = 3 * tr + 10 * r + 3 * br -
                   (3 * tl + 10 * l + 3 * bl);
        int y    = 3 * bl + 10 * b + 3 * br -
                   (3 * tl + 10 * t + 3 * tr);
        x        = x >> 4;
        y        = y >> 4;
        unsigned int res  = (x & 0x0000FFFF) | (y << 16);
        dst[j]   = res;
      }
      src += A->line_size[l];
      dst += (B->line_size[l] >> 2);
    }
  }
}
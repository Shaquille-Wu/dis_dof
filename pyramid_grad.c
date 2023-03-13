#include "pyramid_grad.h"
#include <string.h>

void  pyramid_grad_xy_iu8_os16(DIS_PYRAMID const* A,
                               DIS_PYRAMID*       B){
  unsigned int level  = 0;
  unsigned int levels = A->level;
  int          i = 0, j = 0;
  memset(B->mem, 0, B->total_buf_size);
  for(level = 0 ; level < levels ; level ++){ 
    int                   src_line_size = A->line_size[level];
    int                   dst_line_size = B->line_size[level] >> 2;
    unsigned char const*  src           = A->buf[level] + 
                                          A->pad + 
                                          A->pad * A->line_size[level];
    unsigned int*         dst           = (unsigned int*)(B->buf[level] + 
                                           B->pad * 4 + 
                                           B->pad * B->line_size[level]);
    unsigned int          cur_width     = A->width[level];
    unsigned int          cur_height    = A->height[level];
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
#if 0
        int x    = 3 * tr + 10 * r + 3 * br -
                   (3 * tl + 10 * l + 3 * bl);
        int y    = 3 * bl + 10 * b + 3 * br -
                   (3 * tl + 10 * t + 3 * tr);
        x        = x >> 4;
        y        = y >> 4;
#else
        int x    = tr + 2 * r + br -
                   (tl + 2 * l + bl);
        int y    = bl + 2 * b + br -
                   (tl + 2 * t + tr);
        x        = x >> 2;
        y        = y >> 2;
#endif
        unsigned int res  = (x & 0x0000FFFF) | (y << 16);
        dst[j]   = res;
      }
      src += src_line_size;
      dst += dst_line_size;
    }
  }
}
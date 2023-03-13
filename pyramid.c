#include <string.h>
#include "pyramid.h"
#include <align_mem.h>
#include <dis_def.h>
#include "pyramid_add_sub.h"
#include "pyramid_grad.h"
#include "pyramid_integral.h"

static const unsigned int kPyrPixelDepth[PYRAMID_PIXEL_TYPE_SUM] = {
  8,
  16,
  32,
  64,
  8,
  16,
  32,
  64,
  32
};

int  create_pyramid(int            src_width, 
                    int            src_height, 
                    int            pixel_type,
                    int            pyramid_level,
                    int            pad,
                    DIS_PYRAMID*   pyramid){
  int          i          = 0;
  int          levels     = 0;
  int          cur_w      = src_width;
  int          cur_h      = src_height;
  unsigned int total_size = 0;
  DIS_PYRAMID  cur_pyr    = { 0 };
  memset(&cur_pyr, 0, sizeof(DIS_PYRAMID));
  while(cur_w >= DIS_PYRAMID_MIN_SIZE && cur_h >= DIS_PYRAMID_MIN_SIZE){
    cur_pyr.width[levels]     = cur_w;
    cur_pyr.height[levels]    = cur_h;
    cur_pyr.line_size[levels] = IMG_LINE_ALIGNED(((cur_w + 2 * pad) * (kPyrPixelDepth[pixel_type] >> 3)));
    cur_w = src_width  >> (levels + 1);
    cur_h = src_height >> (levels + 1);
    total_size += cur_pyr.line_size[levels] * (cur_pyr.height[levels] + 2 * pad);
    levels ++;
    if(levels == pyramid_level){
      break;
    }
  }
  if(levels < pyramid_level){
    return -1;
  }

  unsigned char*  buf     = alloc_mem_align(total_size);
  unsigned char*  cur_buf = buf;
  for(i = 0 ; i < levels ; i ++){
    cur_pyr.buf[i] = cur_buf;
    cur_buf       += cur_pyr.line_size[i] * (cur_pyr.height[i] + 2 * pad);
  }
  cur_pyr.mem            = buf;
  cur_pyr.raw_width      = src_width;
  cur_pyr.raw_height     = src_height;
  cur_pyr.total_buf_size = total_size;
  cur_pyr.pixel_type     = pixel_type;
  cur_pyr.level          = levels;
  cur_pyr.pad            = pad;
  memcpy(pyramid, &cur_pyr, sizeof(DIS_PYRAMID));

  return 0;
}

int  create_pyramid_with_size(unsigned int const*    src_width, 
                              unsigned int const*    src_height, 
                              int                    pixel_type,
                              int                    pyramid_level,
                              int                    pad,
                              DIS_PYRAMID*           pyramid){
  int          i          = 0;
  unsigned int total_size = 0;
  DIS_PYRAMID  cur_pyr    = { 0 };
  memset(&cur_pyr, 0, sizeof(DIS_PYRAMID));
  for(i = 0 ; i < pyramid_level ; i ++){
    cur_pyr.width[i]     = src_width[i];
    cur_pyr.height[i]    = src_height[i];
    cur_pyr.line_size[i] = IMG_LINE_ALIGNED(((cur_pyr.width[i] + 2 * pad) * (kPyrPixelDepth[pixel_type] >> 3)));
    total_size          += cur_pyr.line_size[i] * 
                           (cur_pyr.height[i] + 2 * pad);
  }

  unsigned char*  buf     = alloc_mem_align(total_size);
  unsigned char*  cur_buf = buf;
  for(i = 0 ; i < pyramid_level ; i ++){
    cur_pyr.buf[i] = cur_buf;
    cur_buf       += cur_pyr.line_size[i] * (cur_pyr.height[i] + 2 * pad);
  }
  cur_pyr.mem            = buf;
  cur_pyr.raw_width      = src_width[0];
  cur_pyr.raw_height     = src_height[0];
  cur_pyr.total_buf_size = total_size;
  cur_pyr.pixel_type     = pixel_type;
  cur_pyr.level          = pyramid_level;
  cur_pyr.pad            = pad;
  memcpy(pyramid, &cur_pyr, sizeof(DIS_PYRAMID));
}

int  build_gray_pyramid(unsigned char const*  src_image,
                        int                   src_line_size,
                        int                   padding_zero,
                        DIS_PYRAMID*          pyramid){
  if(NULL == pyramid)  return -1;
  if(NULL == pyramid->buf[0] || pyramid->level < 1) return -1;
  int l = 0, i = 0, j = 0, k = 0;

  unsigned int  bytes_per_pixel = (kPyrPixelDepth[pyramid->pixel_type] >> 3);
  if((pyramid->line_size[0] == src_line_size) && 0 == pyramid->pad){
    memcpy(pyramid->buf[0], src_image, src_line_size * pyramid->raw_height);
  }else{
    unsigned char const* src             = src_image;
    unsigned char*       dst             = pyramid->buf[0] + 
                            pyramid->pad * pyramid->line_size[0] +
                            pyramid->pad * bytes_per_pixel;
    unsigned int         bytes_per_line  = bytes_per_pixel * pyramid->raw_width;
    for(i = 0 ; i < pyramid->raw_height ; i ++){
      memcpy(dst, src, bytes_per_line);
      src += src_line_size;
      dst += pyramid->line_size[0];
    }
  }

  int cur_width  = pyramid->raw_width >> 1;
  int cur_height = pyramid->raw_height >> 1;
  for(l = 1 ; l < pyramid->level ; l ++){
    unsigned int   last_leve_line_size = pyramid->line_size[l - 1];
    unsigned char* src                 = pyramid->buf[l - 1] + 
                                         pyramid->pad * bytes_per_pixel +
                                         pyramid->pad * pyramid->line_size[l - 1];
    unsigned char* dst                 = pyramid->buf[l] +
                                         pyramid->pad * bytes_per_pixel +
                                         pyramid->pad * pyramid->line_size[l];
    for(i = 0 ; i < cur_height ; i ++){
      for(j = 0 ; j < cur_width ; j ++){
        dst[j] = (src[2 * j] + 
                  src[2 * j + 1] +
                  src[last_leve_line_size + 2 * j] + 
                  src[last_leve_line_size + 2 * j + 1] + 2) >> 2;
      }
      src  += 2 * last_leve_line_size;
      dst  += pyramid->line_size[l];
    }
    cur_width  = pyramid->raw_width  >> (l + 1);
    cur_height = pyramid->raw_height >> (l + 1);
  }

  if(pyramid->pad > 0 && 0 == padding_zero){
    unsigned int  bytes_per_pixel = (kPyrPixelDepth[pyramid->pixel_type] >> 3);
    for(l = 0 ; l < pyramid->level ; l ++){
      unsigned int    bytes_per_line = bytes_per_pixel * pyramid->width[l];
      //top and bottom
      unsigned char*  padding_top    = pyramid->buf[l] + 
                                       pyramid->pad * bytes_per_pixel;
      unsigned char*  src_top        = padding_top + pyramid->pad * pyramid->line_size[l];
      unsigned char*  padding_btm    = padding_top +
                                        (pyramid->pad + pyramid->height[l]) * pyramid->line_size[l];
      unsigned char*  src_btm        = padding_top +
                                        (pyramid->pad + pyramid->height[l] - 1) * pyramid->line_size[l];
      for(i = 0 ; i < pyramid->pad ; i ++){
        memcpy(padding_top, src_top, bytes_per_line);
        memcpy(padding_btm, src_btm, bytes_per_line);
        padding_top += pyramid->line_size[l];
        padding_btm += pyramid->line_size[l];
      }
      //left and right
      unsigned char*  padding_left   = pyramid->buf[l] + pyramid->pad * pyramid->line_size[l];
      unsigned char*  src_left       = padding_left + pyramid->pad * bytes_per_pixel;
      unsigned char*  padding_right  = padding_left +
                                        (pyramid->pad + pyramid->width[l]) * bytes_per_pixel;
      unsigned char*  src_right      = padding_left +
                                        (pyramid->pad + pyramid->width[l] - 1) * bytes_per_pixel;
      for(i = 0 ; i < pyramid->height[l] ; i ++){
        for(j = 0 ; j < pyramid->pad ; j ++){
          padding_left[j]  = src_left[0];
          padding_right[j] = src_right[0];
        }
        padding_left  += pyramid->line_size[l];
        padding_right += pyramid->line_size[l];
        src_left      += pyramid->line_size[l];
        src_right     += pyramid->line_size[l];
      }
      //top left, top right, btm left, btm right
      unsigned char*  padding_tl  = pyramid->buf[l];
      unsigned char*  padding_tr  = pyramid->buf[l] + 
                                    (pyramid->pad + pyramid->width[l]) * bytes_per_pixel;
      unsigned char*  padding_bl  = pyramid->buf[l] + 
                                    (pyramid->pad + pyramid->height[l]) * pyramid->line_size[l];
      unsigned char*  padding_br  = pyramid->buf[l] + 
                                    (pyramid->pad + pyramid->height[l]) * pyramid->line_size[l] +
                                    (pyramid->pad + pyramid->width[l]) * bytes_per_pixel;
      unsigned char*  src_tl      = pyramid->buf[l] + 
                                    pyramid->pad * pyramid->line_size[l] +
                                    pyramid->pad * bytes_per_pixel;
      unsigned char*  src_tr      = pyramid->buf[l] + 
                                    pyramid->pad * pyramid->line_size[l] +
                                    (pyramid->pad + pyramid->width[l] - 1) * bytes_per_pixel;
      unsigned char*  src_bl      = pyramid->buf[l] + 
                                    (pyramid->pad + pyramid->height[l] - 1) * pyramid->line_size[l] +
                                    pyramid->pad  * bytes_per_pixel;
      unsigned char*  src_br      = pyramid->buf[l] + 
                                    (pyramid->pad + pyramid->height[l] - 1) * pyramid->line_size[l] +
                                    (pyramid->pad + pyramid->width[l] - 1)  * bytes_per_pixel;
      for(i = 0 ; i < pyramid->pad ; i ++){
        for(j = 0 ; j < pyramid->pad ; j ++){
          padding_tl[j] = src_tl[0];
          padding_tr[j] = src_tr[0];
          padding_bl[j] = src_bl[0];
          padding_br[j] = src_br[0];
        }
        padding_tl += pyramid->line_size[l];
        padding_tr += pyramid->line_size[l];
        padding_bl += pyramid->line_size[l];
        padding_br += pyramid->line_size[l];
      }
    }
  }
}

int  add_pyramid(DIS_PYRAMID const* A,
                 DIS_PYRAMID const* B,
                 DIS_PYRAMID*       C){
  if(A->pixel_type != B->pixel_type){
    return -1;
  }
  if((PYRAMID_PIXEL_U8 == A->pixel_type)
     && PYRAMID_PIXEL_U16 != C->pixel_type){
    return -1;
  }
  pyramid_add_iu8_ou16(A, B, C);

  return 0;
}

int  sub_pyramid(DIS_PYRAMID const* A,
                 DIS_PYRAMID const* B,
                 DIS_PYRAMID*       C){
  if(A->pixel_type != B->pixel_type){
    return -1;
  }
  if((PYRAMID_PIXEL_U8 == A->pixel_type)
     && PYRAMID_PIXEL_S16 != C->pixel_type){
    return -1;
  }
  pyramid_sub_iu8_os16(A, B, C);

  return 0;
}

int grad_xy_pyramid(DIS_PYRAMID const* A,
                    DIS_PYRAMID*       B){
  if(PYRAMID_PIXEL_U8 != A->pixel_type ||
     (PYRAMID_PIXEL_U32 != B->pixel_type &&
      PYRAMID_PIXEL_S32 != B->pixel_type)){
    return -1;
  }

  pyramid_grad_xy_iu8_os16(A, B);

  return 0;
}

int  integral_pyramid(DIS_PYRAMID const* A,
                      DIS_PYRAMID*       B){
  if(PYRAMID_PIXEL_U8 != A->pixel_type ||
     PYRAMID_PIXEL_U32 != B->pixel_type){
    return -1;
  }

  pyramid_integral_iu8_ou32(A, B);

  return 0;
}

int  integral_sqr_pyramid(DIS_PYRAMID const* A,
                          DIS_PYRAMID*       B){
  if(PYRAMID_PIXEL_U8 != A->pixel_type ||
     PYRAMID_PIXEL_U64 != B->pixel_type){
    return -1;
  }

  pyramid_integral_sqr_iu8_ou64(A, B);

  return 0;
}

int  destroy_pyramid(DIS_PYRAMID*  pyramid){
  if(NULL != pyramid){
    if(NULL != pyramid->mem){
      free_mem_align(pyramid->mem);
    }
  }

  return 0;
}
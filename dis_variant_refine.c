#include "dis_variant_refine.h"
#include <math.h>
#include <string.h>
#include <bmp.h>
#include <align_mem.h>
#include <stdio.h>
#include "draw_output.h"

#define  SMOOTH_EPS     1e-6f
#define  DATA_BRI_EPS   1e-6f
#define  DATA_GRAD_EPS  1e-6f
#define  DATA_NORM      0.01f

static void clear_dense_flow_border(DIS_INSTANCE* dis, unsigned int cur_level){
  unsigned long long*  dense_flow           = (unsigned long long*)(dis->dense_flow_pyramid.buf[cur_level]);
  unsigned int         dense_flow_line_size = dis->dense_flow_pyramid.line_size[cur_level] >> 3;
  unsigned int         w                    = dis->dense_flow_pyramid.width[cur_level];
  unsigned int         h                    = dis->dense_flow_pyramid.height[cur_level];
  unsigned int         i                    = 0;
  unsigned int         j                    = 0;
  memset(dense_flow,    
         0, 
         dis->dense_flow_pyramid.pad * (dense_flow_line_size << 3));
  memset(dense_flow + 
         (dis->dense_flow_pyramid.pad + h) * dense_flow_line_size, 
         0, 
         dis->dense_flow_pyramid.pad * (dense_flow_line_size << 3));
  dense_flow = dense_flow +
               dis->dense_flow_pyramid.pad * dense_flow_line_size;
  unsigned long long*  dense_flow1 = dense_flow + w + dis->dense_flow_pyramid.pad;
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < dis->dense_flow_pyramid.pad ; j ++){
      *dense_flow   = 0;
      *dense_flow1  = 0;
    }
    dense_flow   += dense_flow_line_size;
    dense_flow1  += dense_flow_line_size;
  }
}

static void shift_image(DIS_INSTANCE* dis, unsigned int cur_level){
  int    w             = dis->ref_gray_pyramid.width[cur_level];
  int    h             = dis->ref_gray_pyramid.height[cur_level];
  int    shift_img_pad           = dis->refine_pad;
  int    shift_img_pad_line_size = IMG_LINE_ALIGNED((w + 2 * shift_img_pad) * 4);
  int    shift_img_line_size     = IMG_LINE_ALIGNED(w * 4);

  unsigned char const*  src_image     = dis->track_gray_pyramid.buf[cur_level] +
                                  dis->track_gray_pyramid.pad * dis->track_gray_pyramid.line_size[cur_level] +
                                  dis->track_gray_pyramid.pad;
  float const*          flow_image     = (float const*)(dis->dense_flow_pyramid.buf[cur_level] +
                                  dis->dense_flow_pyramid.pad * dis->dense_flow_pyramid.line_size[cur_level] +
                                  dis->dense_flow_pyramid.pad * 8);
  float* dst_img       = dis->shift_image + 
                              shift_img_pad * (shift_img_pad_line_size >> 2) +
                              shift_img_pad;
  float* mask_img      = dis->shift_image_mask;
  memset(dis->shift_image, 0, shift_img_pad_line_size * (2 * shift_img_pad + h));

  int    src_img_line_size      = dis->ref_gray_pyramid.line_size[cur_level];
  int    flow_img_line_size     = (dis->dense_flow_pyramid.line_size[cur_level] >> 2);
  int                   i             = 0;
  int                   j             = 0;
  shift_img_line_size = shift_img_line_size >> 2;
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < w ; j ++){
      float  delta_x_f = flow_image[2 * j];
      float  delta_y_f = flow_image[2 * j + 1];
      float  x_f       = j + delta_x_f;
      float  y_f       = i + delta_y_f;
      int    valid     = (x_f >= 0.0f) & (x_f <= ((float)(w - 1))) & (y_f >= 0.0f) & (y_f <= ((float)(h - 1)));
      x_f              = (x_f < -1.0f ? -1.0f : x_f);
      y_f              = (y_f < -1.0f ? -1.0f : y_f);
      x_f              = (x_f >= ((float)(w)) ? ((float)(w)) : x_f);
      y_f              = (y_f >= ((float)(h)) ? ((float)(h)) : y_f);
      int    y_i       = floorf(y_f);
      int    x_i       = floorf(x_f);
      float  u         = x_f - x_i;
      float  v         = y_f - y_i;
      float  d00       = src_image[     y_i  * src_img_line_size + x_i];
      float  d01       = src_image[     y_i  * src_img_line_size + x_i + 1];
      float  d10       = src_image[(y_i + 1) * src_img_line_size + x_i];
      float  d11       = src_image[(y_i + 1) * src_img_line_size + x_i + 1];
      float  res       = d00 * (1.0f - u) * (1.0f - v) +
                         d01 *          u * (1.0f - v) +
                         d10 * (1.0f - u) *         v  +
                         d11 *          u *         v;
      dst_img[j]       = res;
    }
    flow_image += flow_img_line_size;
    dst_img    += (shift_img_pad_line_size >> 2);
    mask_img   += shift_img_line_size;
  }
}

static void fill_uv_border(long long* uv,
                           int        uv_pad,
                           int        uv_w,
                           int        uv_h){
  int  w     = uv_w;
  int  h     = uv_h;
  int  i     = 0;
  int  j     = 0;
  int  refine_uv_pad_line_size = IMG_LINE_ALIGNED((w + 2 * uv_pad) * 8) >> 3;
  long long* dst0 = uv + uv_pad;
  long long* src0 = uv + 
                uv_pad * refine_uv_pad_line_size +
                uv_pad;
  long long* dst1 = dst0 + (uv_pad + h) * refine_uv_pad_line_size;
  long long* src1 = src0 + (h - 1) * refine_uv_pad_line_size;
  for(i = 0 ; i < uv_pad ; i ++){
    memcpy(dst0, src0, w * 8);
    memcpy(dst1, src1, w * 8);
    dst0 += refine_uv_pad_line_size;
    dst1 += refine_uv_pad_line_size;
  }
  src0 = uv + 
         uv_pad * refine_uv_pad_line_size +
         uv_pad;
  src1 = src0 + w - 1;
  dst0 = uv + uv_pad * refine_uv_pad_line_size;
  dst1 = dst0 + (uv_pad + w);
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < uv_pad ; j ++){
      dst0[j] = src0[0];
      dst1[j] = src1[0];
    }
    src0 += refine_uv_pad_line_size;
    src1 += refine_uv_pad_line_size;
    dst0 += refine_uv_pad_line_size;
    dst1 += refine_uv_pad_line_size;
  }
  src0 = uv + 
         uv_pad * refine_uv_pad_line_size +
         uv_pad;
  dst0  = src0;
  dst0[0 - 1 - refine_uv_pad_line_size] = src0[0];
  dst0  = src0 + w;
  dst0[0 - refine_uv_pad_line_size]     = src0[w - 1];
  dst0  = src0 + h * refine_uv_pad_line_size;
  dst0[-1]                              = src0[(h - 1) * refine_uv_pad_line_size];
  dst0[w]                               = src0[(h - 1) * refine_uv_pad_line_size + w - 1];
}

static void fill_refine_border(float*     refine,
                               int        refine_pad,
                               int        refine_w,
                               int        refine_h){
  int  w     = refine_w;
  int  h     = refine_h;
  int  i     = 0;
  int  j     = 0;
  int  refine_pad_line_size = IMG_LINE_ALIGNED((w + 2 * refine_pad) * 4) >> 2;
  float* dst0 = refine + refine_pad;
  float* src0 = refine + 
                refine_pad * refine_pad_line_size +
                refine_pad;
  float* dst1 = dst0 + (refine_pad + h) * refine_pad_line_size;
  float* src1 = src0 + (h - 1) * refine_pad_line_size;
  for(i = 0 ; i < refine_pad ; i ++){
    memcpy(dst0, src0, w * 4);
    memcpy(dst1, src1, w * 4);
    dst0 += refine_pad_line_size;
    dst1 += refine_pad_line_size;
  }
  src0 = refine + 
         refine_pad * refine_pad_line_size +
         refine_pad;
  src1 = src0 + w - 1;
  dst0 = refine + refine_pad * refine_pad_line_size;
  dst1 = dst0 + (refine_pad + w);
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < refine_pad ; j ++){
      dst0[j] = src0[0];
      dst1[j] = src1[0];
    }
    src0 += refine_pad_line_size;
    src1 += refine_pad_line_size;
    dst0 += refine_pad_line_size;
    dst1 += refine_pad_line_size;
  }
  src0 = refine + 
         refine_pad * refine_pad_line_size +
         refine_pad;
  dst0  = src0;
  dst0[0 - 1 - refine_pad_line_size] = src0[0];
  dst0  = src0 + w;
  dst0[0 - refine_pad_line_size]     = src0[w - 1];
  dst0  = src0 + h * refine_pad_line_size;
  dst0[-1]                           = src0[(h - 1) * refine_pad_line_size];
  dst0[w]                            = src0[(h - 1) * refine_pad_line_size + w - 1];
}

static void prepare_terms(DIS_INSTANCE* dis, unsigned int cur_level){
  shift_image(dis, cur_level);
  
  int     w                = dis->ref_gray_pyramid.width[cur_level];
  int     h                = dis->ref_gray_pyramid.height[cur_level];
  int     refine_pad_line_size = IMG_LINE_ALIGNED(((w + 2 * dis->refine_pad) * 4)) >> 2;
  int     refine_line_size     = IMG_LINE_ALIGNED((w * 4)) >> 2;
  float*  ref_track_avg    = dis->ref_track_avg;
  float*  Ix               = dis->Ix;
  float*  Iy               = dis->Iy;
  float*  It               = dis->It;
  float*  Ixx              = dis->Ixx;
  float*  Ixy              = dis->Ixy;
  float*  Iyy              = dis->Iyy;
  float*  Ixt              = dis->Ixt;
  float*  Iyt              = dis->Iyt;
  int*    ref_Ixy          = (int*)(dis->ref_grad_xy_pyramid.buf[cur_level] + 
                                    dis->ref_grad_xy_pyramid.pad * dis->ref_grad_xy_pyramid.line_size[cur_level] +
                                    dis->ref_grad_xy_pyramid.pad * 4);
  int     ref_Ixy_line_size= dis->ref_grad_xy_pyramid.line_size[cur_level] >> 2;
  unsigned char* ref_img   = dis->ref_gray_pyramid.buf[cur_level] +
                             dis->ref_gray_pyramid.pad * dis->ref_gray_pyramid.line_size[cur_level] +
                             dis->ref_gray_pyramid.pad;
  float*         shift_img = dis->shift_image + 
                             refine_pad_line_size * dis->refine_pad +
                             dis->refine_pad;
  int     ref_img_line_size = dis->ref_gray_pyramid.line_size[cur_level];
  int     i                 = 0;
  int     j                 = 0;
  //clear image border for 1 order derivate and ref_track_avg, start
  memset(ref_track_avg, 0, dis->refine_pad * (refine_pad_line_size << 2));
  memset(Ix,            0, dis->refine_pad * (refine_pad_line_size << 2));
  memset(Iy,            0, dis->refine_pad * (refine_pad_line_size << 2));
  memset(It,            0, dis->refine_pad * (refine_pad_line_size << 2));
  memset(ref_track_avg + (dis->refine_pad + h) * refine_pad_line_size,
         0,
         dis->refine_pad * (refine_pad_line_size << 2));
  memset(Ix + (dis->refine_pad + h) * refine_pad_line_size,
         0,
         dis->refine_pad * (refine_pad_line_size << 2));
  memset(Iy + (dis->refine_pad + h) * refine_pad_line_size,
         0,
         dis->refine_pad * (refine_pad_line_size << 2));
  memset(It + (dis->refine_pad + h) * refine_pad_line_size,
         0,
         dis->refine_pad * (refine_pad_line_size << 2));
  ref_track_avg  = ref_track_avg + 
                   dis->refine_pad * refine_pad_line_size;
  Ix             = Ix + 
                   dis->refine_pad * refine_pad_line_size;
  Iy             = Iy + 
                   dis->refine_pad * refine_pad_line_size;
  It             = It + 
                   dis->refine_pad * refine_pad_line_size;
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < dis->refine_pad ; j ++){
      ref_track_avg[j]     = 0.0f;
      Ix[j]                = 0.0f;
      Iy[j]                = 0.0f;
      It[j]                = 0.0f;
      ref_track_avg[j + w] = 0.0f;
      Ix[j + w]            = 0.0f;
      Iy[j + w]            = 0.0f;
      It[j + w]            = 0.0f;
    }
    ref_track_avg += refine_pad_line_size;
    Ix            += refine_pad_line_size;
    Iy            += refine_pad_line_size;
    It            += refine_pad_line_size;
  }
  //clear image border for 1 order derivate and ref_track_avg, end

  ref_track_avg -= h * refine_pad_line_size;
  Ix            -= h * refine_pad_line_size;
  Iy            -= h * refine_pad_line_size;
  It            -= h * refine_pad_line_size;

  ref_track_avg += dis->refine_pad;
  Ix            += dis->refine_pad;
  Iy            += dis->refine_pad;
  It            += dis->refine_pad;

  {
    int            cur_shift_line_size = ((w + 3) >> 2) << 2;
    unsigned char* cur_shift_img       = (unsigned char*)alloc_mem_align(cur_shift_line_size * h);
    for(i = 0 ; i < h ; i ++){
      for(j = 0 ; j < w ; j ++){
        cur_shift_img[i * cur_shift_line_size + j] = shift_img[i * refine_pad_line_size + j];
      }
    }
    char           file_name[256] = { 0 };
    sprintf(file_name, "/home/icework/adas_alg_ref/dis_dof_ref/dis_dof/data/output/warp_img_%d.bmp", cur_level);
    SaveBMP(file_name,
            cur_shift_img,
            w,
            h,
            cur_shift_line_size,
            8,
            100);
  }

  //fill_refine_border(dis->shift_image, dis->refine_pad, w, h);
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < w ; j ++){
      ref_track_avg[j] = 0.5f * (shift_img[j] + ref_img[j]);
      It[j]            = shift_img[j] - ref_img[j];
    }
    ref_img       += ref_img_line_size;
    shift_img     += refine_pad_line_size;
    ref_track_avg += refine_pad_line_size;
    It            += refine_pad_line_size;
  }

  ref_track_avg -= h * refine_pad_line_size;
  It            -= h * refine_pad_line_size;
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < w ; j ++){
      float tl   = ref_track_avg[j - 1 - refine_pad_line_size];
      float t    = ref_track_avg[j - refine_pad_line_size];
      float tr   = ref_track_avg[j + 1 - refine_pad_line_size];
      float r    = ref_track_avg[j + 1];
      float br   = ref_track_avg[j + 1 + refine_pad_line_size];
      float b    = ref_track_avg[j + refine_pad_line_size];
      float bl   = ref_track_avg[j - 1 + refine_pad_line_size];
      float l    = ref_track_avg[j - 1];
      float x    = tr + 2.0f * r + br -
                   (tl + 2.0f * l + bl);
      float y    = bl + 2.0f * b + br -
                   (tl + 2.0f * t + tr);
      Ix[j]      = x * 0.25f;
      Iy[j]      = y * 0.25f;
#if 0      
      int ref_x = ((ref_Ixy[j] << 16) >> 16);
      int ref_y = (ref_Ixy[j]         >> 16);
      Ixt[j]   = Ix[j] - (float)ref_x;
      Iyt[j]   = Iy[j] - (float)ref_y;
#else
      tl   = It[j - 1 - refine_pad_line_size];
      t    = It[j - refine_pad_line_size];
      tr   = It[j + 1 - refine_pad_line_size];
      r    = It[j + 1];
      br   = It[j + 1 + refine_pad_line_size];
      b    = It[j + refine_pad_line_size];
      bl   = It[j - 1 + refine_pad_line_size];
      l    = It[j - 1];
      x    = tr + 2.0f * r + br -
                   (tl + 2.0f * l + bl);
      y    = bl + 2.0f * b + br -
                   (tl + 2.0f * t + tr);
      Ixt[j] = x * 0.25f;
      Iyt[j] = y * 0.25f;
#endif
    }
    Ix      += refine_pad_line_size;
    Iy      += refine_pad_line_size;
    It      += refine_pad_line_size;
    Ixt     += refine_line_size;
    Iyt     += refine_line_size;
  }
  Ix -= h * refine_pad_line_size;
  Iy -= h * refine_pad_line_size;
  It -= h * refine_pad_line_size;

  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < w ; j ++){
      float tl   = Ix[j - 1 - refine_pad_line_size];
      float t    = Ix[j - refine_pad_line_size];
      float tr   = Ix[j + 1 - refine_pad_line_size];
      float r    = Ix[j + 1];
      float br   = Ix[j + 1 + refine_pad_line_size];
      float b    = Ix[j + refine_pad_line_size];
      float bl   = Ix[j - 1 + refine_pad_line_size];
      float l    = Ix[j - 1];

      float tl_y = Iy[j - 1 - refine_pad_line_size];
      float t_y  = Iy[j - refine_pad_line_size];
      float tr_y = Iy[j + 1 - refine_pad_line_size];
      float bl_y = Iy[j - 1 + refine_pad_line_size];
      float b_y  = Iy[j + refine_pad_line_size];
      float br_y = Iy[j + 1 + refine_pad_line_size];
      float xx   = tr + 2.0f * r + br -
                   (tl + 2.0f * l + bl);
      float xy   = bl + 2.0f * b + br -
                   (tl + 2.0f * t + tr);
      float yy   = bl_y + 2.0f * b_y + br_y -
                   (tl_y + 2.0f * t_y + tr_y);
      Ixx[j]   = xx * 0.25f;
      Ixy[j]   = xy * 0.25f;
      Iyy[j]   = yy * 0.25f;
    }
    Ix  += refine_pad_line_size;
    Iy  += refine_pad_line_size;
    Ixx += refine_line_size;
    Ixy += refine_line_size;
    Iyy += refine_line_size;
  }

  int  refine_uv_pad_line_size = IMG_LINE_ALIGNED(((w + 2 * dis->refine_uv_pad) * 8)) >> 3;
  memset(dis->dudv, 0, (refine_uv_pad_line_size << 3) * (h + 2 * dis->refine_uv_pad));
  if(dis->dense_flow_pyramid.pad == dis->refine_uv_pad &&
     dis->dense_flow_pyramid.line_size[cur_level] == refine_uv_pad_line_size * 8){
    memcpy(dis->uv, 
           dis->dense_flow_pyramid.buf[cur_level], 
           (refine_uv_pad_line_size << 3) * (h + 2 * dis->refine_uv_pad));
  }else{
    unsigned long long*  src_flow = (unsigned long long*)(dis->dense_flow_pyramid.buf[cur_level] +
                                    dis->dense_flow_pyramid.pad * dis->dense_flow_pyramid.line_size[cur_level] +
                                    dis->dense_flow_pyramid.pad * 8);
    unsigned long long*  dst_uv   = (unsigned long long*)(dis->dudv +
                                    dis->refine_uv_pad * refine_uv_pad_line_size);
    memset(dis->uv, 
           0, 
           (refine_uv_pad_line_size << 3) * dis->refine_uv_pad);
    memset(dis->uv + (h + dis->refine_uv_pad) * refine_uv_pad_line_size, 
           0, 
           (refine_uv_pad_line_size << 3) * dis->refine_uv_pad);
    for(i = 0 ; i < h ; i ++){
      for(j = 0 ; j < dis->refine_pad ; j ++){
        dst_uv[j]                       = 0;
        dst_uv[dis->refine_pad + w + j] = 0;
      }
      dst_uv += refine_uv_pad_line_size;
    }
    dst_uv  = (unsigned long long*)(dis->dudv +
                                    dis->refine_pad * refine_uv_pad_line_size +
                                    dis->refine_pad);
    for(i = 0 ; i < h ; i ++){
      memcpy(dst_uv, src_flow, 8 * w);
      src_flow += (dis->dense_flow_pyramid.line_size[cur_level] >> 3);
      dst_uv   += refine_uv_pad_line_size;
    }
  }
  fill_uv_border((long long*)(dis->dense_flow_pyramid.buf[cur_level]), 
                 dis->dense_flow_pyramid.pad, w, h);
  fill_uv_border(dis->uv, dis->refine_uv_pad, w, h);
}

static void cacl_data_term(DIS_INSTANCE* dis, unsigned int cur_level){
  int     w                    = dis->ref_gray_pyramid.width[cur_level];
  int     h                    = dis->ref_gray_pyramid.height[cur_level];
  int     refine_line_size     = IMG_LINE_ALIGNED((w * 4)) >> 2;
  int     refine_pad_line_size    = IMG_LINE_ALIGNED(((w + 2 * dis->refine_pad) * 4)) >> 2;
  int     refine_uv_pad_line_size = IMG_LINE_ALIGNED(((w + 2 * dis->refine_uv_pad) * 8)) >> 3;
  float*  mask             = dis->shift_image_mask;
  float*  Ix               = dis->Ix + 
                             dis->refine_pad * refine_pad_line_size +
                             dis->refine_pad;
  float*  Iy               = dis->Iy + 
                             dis->refine_pad * refine_pad_line_size +
                             dis->refine_pad;
  float*  It               = dis->It + 
                             dis->refine_pad * refine_pad_line_size +
                             dis->refine_pad;
  float*  Ixx              = dis->Ixx;
  float*  Ixy              = dis->Ixy;
  float*  Iyy              = dis->Iyy;
  float*  Ixt              = dis->Ixt;
  float*  Iyt              = dis->Iyt;
  float*  A00              = dis->A00 + 
                             dis->refine_pad * refine_pad_line_size +
                             dis->refine_pad;
  float*  A01              = dis->A01 + 
                             dis->refine_pad * refine_pad_line_size +
                             dis->refine_pad;
  float*  A11              = dis->A11 + 
                             dis->refine_pad * refine_pad_line_size +
                             dis->refine_pad;
  float*  b0               = dis->b0;
  float*  b1               = dis->b1;
  float*  dudv             = (float*)(dis->dudv + 
                                      dis->refine_uv_pad * refine_uv_pad_line_size +
                                      dis->refine_uv_pad);
  float   weight_delta     = dis->tv_delta;
  float   weight_gamma     = dis->tv_gamma;
  int     i = 0, j = 0;

  memset(A00, 0, (refine_pad_line_size << 2) * h);
  memset(A01, 0, (refine_pad_line_size << 2) * h);
  memset(A11, 0, (refine_pad_line_size << 2) * h);
  memset(b0,  0, (refine_line_size << 2) * h);
  memset(b1,  0, (refine_line_size << 2) * h);

  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < w ; j ++){
      float norm0  = Ix[j] * Ix[j] + Iy[j] * Iy[j] + DATA_NORM;
      float tmp0   = It[j] + Ix[j] * dudv[2 * j] + Iy[j] * dudv[2 * j + 1];
      tmp0         = weight_delta / sqrtf((tmp0 * tmp0) / norm0 + DATA_BRI_EPS) / norm0;
      A00[j] += tmp0  * Ix[j] * Ix[j] + DATA_NORM;
      A01[j] += tmp0  * Ix[j] * Iy[j];
      A11[j] += tmp0  * Iy[j] * Iy[j] + DATA_NORM;
      b0[j]  -= tmp0  * It[j] * Ix[j];
      b1[j]  -= tmp0  * It[j] * Iy[j];

      norm0       = Ixx[j] * Ixx[j] + Ixy[j] * Ixy[j] + DATA_NORM;
      float norm1 = Iyy[j] * Iyy[j] + Ixy[j] * Ixy[j] + DATA_NORM;
      tmp0        = Ixt[j] + Ixx[j] * dudv[2 * j] + Ixy[j] * dudv[2 * j + 1];
      float tmp1  = Iyt[j] + Ixy[j] * dudv[2 * j] + Iyy[j] * dudv[2 * j + 1];
      tmp0        = weight_gamma / sqrtf(tmp0 * tmp0/norm0 + tmp1*tmp1/norm1 + DATA_GRAD_EPS);
      tmp0        = tmp0/norm0;
      tmp1        = tmp0/norm1;
      A00[j] += tmp0 * Ixx[j] * Ixx[j] + tmp1 * Ixy[j] * Ixy[j];
      A01[j] += tmp0 * Ixx[j] * Ixy[j] + tmp1 * Ixy[j] * Iyy[j];
      A11[j] += tmp0 * Ixy[j] * Ixy[j] + tmp1 * Iyy[j] * Iyy[j];
      b0[j]  -= tmp0 * Ixx[j] * Ixt[j] + tmp1 * Ixy[j] * Iyt[j];
      b1[j]  -= tmp0 * Ixy[j] * Ixt[j] + tmp1 * Iyy[j] * Iyt[j];

      //tmp0          = Ixt[j] + Ixx[j] * dudv[2 * j] + Ixy[j] * dudv[2 * j + 1];
      //float tmp1    = Iyt[j] + Ixy[j] * dudv[2 * j] + Iyy[j] * dudv[2 * j + 1];
      //tmp0          = mask[j] * weight_gamma / sqrtf(tmp0 * tmp0 + tmp1 * tmp1 + DATA_GRAD_EPS);

      //A00[j] += tmp0 * (Ixx[j] * Ixx[j] + Ixy[j] * Ixy[j]);
      //A01[j] += tmp0 * (Ixx[j] * Ixy[j] + Ixy[j] * Iyy[j]);
      //A11[j] += tmp0 * (Iyy[j] * Iyy[j] + Ixy[j] * Ixy[j]);
      //b0[j]  -= tmp0 * (Ixx[j] * Ixt[j] + Ixy[j] * Iyt[j]);
      //b1[j]  -= tmp0 * (Iyy[j] * Iyt[j] + Ixy[j] * Ixt[j]);
    }
    Ix   += refine_pad_line_size;
    Iy   += refine_pad_line_size;
    It   += refine_pad_line_size;
    Ixx  += refine_line_size;
    Ixy  += refine_line_size;
    Iyy  += refine_line_size;
    Ixt  += refine_line_size;
    Iyt  += refine_line_size;
    A00  += refine_pad_line_size;
    A01  += refine_pad_line_size;
    A11  += refine_pad_line_size;
    b0   += refine_line_size;
    b1   += refine_line_size;
    mask += refine_line_size;
  }
}

static void cacl_smooth_term(DIS_INSTANCE* dis, unsigned int cur_level){
  int       w                   = dis->dense_flow_pyramid.width[cur_level];
  int       h                   = dis->dense_flow_pyramid.height[cur_level];
  int       refine_uv_pad_line_size = IMG_LINE_ALIGNED((w + 2 * dis->refine_uv_pad) * 8) >> 3;
  int       refine_line_size        = IMG_LINE_ALIGNED(w * 4) >> 2;
  int       refine_pad_line_size    = IMG_LINE_ALIGNED((w + 2 * dis->refine_pad) * 4) >> 2;
  float const*  uv = (float const*)(dis->uv +
                                    dis->refine_uv_pad * refine_uv_pad_line_size +
                                    dis->refine_uv_pad);
  float const*  dense_uv = (float const*)(dis->dense_flow_pyramid.buf[cur_level] +
                                    dis->dense_flow_pyramid.pad * dis->dense_flow_pyramid.line_size[cur_level] +
                                    dis->dense_flow_pyramid.pad * 8);
  int           dense_uv_line_size = dis->dense_flow_pyramid.line_size[cur_level] >> 2;
  float*        smooth_uv  = (float*)(dis->smooth_uv +
                                      dis->refine_uv_pad * refine_uv_pad_line_size +
                                      dis->refine_uv_pad);
  float*        smoothness = dis->smoothness;
  float*        mask       = dis->shift_image_mask;
  memset(smooth_uv,  0, (refine_uv_pad_line_size << 3) * (h + 2 * dis->refine_uv_pad));
  memset(smoothness, 0, (refine_pad_line_size << 2)    * (h + 2 * dis->refine_pad));
  smoothness = (float*)(dis->smoothness +
                        dis->refine_pad * refine_pad_line_size +
                        dis->refine_pad);
  int       uv_line_size        = refine_uv_pad_line_size << 1;
  int       i = 0, j = 0;
  float     tv_alpha            = dis->tv_alpha;
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < w ; j ++){
#if 1
      float  r_u   = uv[2 * (j + 1)];
      float  r_v   = uv[2 * (j + 1) + 1];
      float  b_u   = uv[2 * j           + uv_line_size];
      float  b_v   = uv[2 * j       + 1 + uv_line_size];
      float  c_u   = uv[2 * j];
      float  c_v   = uv[2 * j       + 1];
      float  ux    = r_u - c_u;
      float  vx    = r_v - c_v;
      float  uy    = b_u - c_u;
      float  vy    = b_v - c_v;
      float  uv_norm = (ux * ux + uy * uy + vx * vx + vy * vy) + SMOOTH_EPS;
#else
      float  tl_u  = uv[2 * (j - 1)     - uv_line_size];
      float  tl_v  = uv[2 * (j - 1) + 1 - uv_line_size];
      float  t_u   = uv[2 * j           - uv_line_size];
      float  t_v   = uv[2 * j       + 1 - uv_line_size];
      float  tr_u  = uv[2 * (j + 1)     - uv_line_size];
      float  tr_v  = uv[2 * (j + 1) + 1 - uv_line_size];
      float  r_u   = uv[2 * (j + 1)];
      float  r_v   = uv[2 * (j + 1) + 1];
      float  br_u  = uv[2 * (j + 1)     + uv_line_size];
      float  br_v  = uv[2 * (j + 1) + 1 + uv_line_size];
      float  b_u   = uv[2 * j           + uv_line_size];
      float  b_v   = uv[2 * j       + 1 + uv_line_size];
      float  bl_u  = uv[2 * (j - 1)     + uv_line_size];
      float  bl_v  = uv[2 * (j - 1) + 1 + uv_line_size];
      float  l_u   = uv[2 * (j - 1)];
      float  l_v   = uv[2 * (j - 1) + 1];
      float  c_u   = uv[2 * j];
      float  c_v   = uv[2 * j       + 1];
      float  x_u   = tr_u + 2 * r_u + br_u -
                     (tl_u + 2 * l_u + bl_u);
      float  y_u   = bl_u + 2 * b_u + br_u -
                     (tl_u + 2 * t_u + tr_u);
      float  x_v   = tr_v + 2 * r_v + br_v -
                     (tl_v + 2 * l_v + bl_v);
      float  y_v   = bl_v + 2 * b_v + br_v -
                     (tl_v + 2 * t_v + tr_v);
      float  uv_norm = 0.0625f * (x_u * x_u + y_u * y_u + x_v * x_v + y_v * y_v) + SMOOTH_EPS;
#endif
      uv_norm        = tv_alpha / sqrtf(uv_norm);
      smoothness[j]  = uv_norm;
    }
    for(j = 0 ; j < w ; j ++){
      float  r_u   = dense_uv[2 * (j + 1)];
      float  r_v   = dense_uv[2 * (j + 1) + 1];
      float  c_u   = dense_uv[2 * j];
      float  c_v   = dense_uv[2 * j       + 1];
      float  ux    = smoothness[j] * (r_u - c_u);
      float  vx    = smoothness[j] * (r_v - c_v);
      smooth_uv[2 * j]     += ux;
      smooth_uv[2 * j + 1] += vx;
      smooth_uv[2 * j + 2] -= ux;
      smooth_uv[2 * j + 3] -= vx;
    }
    smooth_uv  += uv_line_size;
    uv         += uv_line_size;
    smoothness += refine_pad_line_size;
    mask       += refine_line_size;
    dense_uv   += dense_uv_line_size;
  }
  smooth_uv    -= h * uv_line_size;
  smoothness   -= h * refine_pad_line_size;
  dense_uv     -= h * dense_uv_line_size;
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < w ; j ++){
      float  b_u            = dense_uv[2 * j     + dense_uv_line_size];
      float  b_v            = dense_uv[2 * j + 1 + dense_uv_line_size];
      float  c_u            = dense_uv[2 * j];
      float  c_v            = dense_uv[2 * j + 1];
      float  uy             = smoothness[j] * (b_u - c_u);
      float  vy             = smoothness[j] * (b_v - c_v);
      smooth_uv[2 * j]     += uy;
      smooth_uv[2 * j + 1] += vy;
      smooth_uv[2 * j     + uv_line_size] -= uy;
      smooth_uv[2 * j + 1 + uv_line_size] -= vy;
    }
    smooth_uv  += uv_line_size;
    smoothness += refine_pad_line_size;
    dense_uv   += dense_uv_line_size;
  }
  /*
  smoothness   -= refine_pad_line_size;
  smoothness   -= dis->refine_pad;
  memset(smoothness, 0, 2 * 4 * refine_pad_line_size);
  smoothness = (float*)(dis->smoothness +
                        dis->refine_pad * refine_pad_line_size +
                        dis->refine_pad + w);
  for(i = 0 ; i < h ; i ++){
    smoothness[0]  = 0.0f; 
    smoothness    += refine_pad_line_size;
  }
  */
  //fill_refine_border(dis->smoothness, dis->refine_pad, w, h);
}

static void move_smooth_term(DIS_INSTANCE* dis, unsigned int cur_level){
  int           w                       = dis->dense_flow_pyramid.width[cur_level];
  int           h                       = dis->dense_flow_pyramid.height[cur_level];
  int           refine_uv_pad_line_size = IMG_LINE_ALIGNED((w + 2 * dis->refine_uv_pad) * 8) >> 3;
  int           refine_line_size        = IMG_LINE_ALIGNED(w * 4) >> 2;
  float*        smooth_uv  = (float*)(dis->smooth_uv + 
                                      dis->refine_uv_pad * refine_uv_pad_line_size +
                                      dis->refine_uv_pad);
  float*        b0         = dis->b0;
  float*        b1         = dis->b1;
  int           i = 0, j = 0;
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < w ; j ++){
      b0[j] += smooth_uv[2 * j];
      b1[j] += smooth_uv[2 * j + 1];
    }
    smooth_uv += (refine_uv_pad_line_size << 1);
    b0        += refine_line_size;
    b1        += refine_line_size;
  }
}

static void solve_sor(DIS_INSTANCE*   dis, 
                      unsigned int    cur_level,
                      unsigned int    sor_iter){
  unsigned int  k  = 0 ;
  int           i  = 0 ;
  int           j  = 0 ;
  int           w  = dis->dense_flow_pyramid.width[cur_level];
  int           h  = dis->dense_flow_pyramid.height[cur_level];
  int           refine_uv_pad_line_size = IMG_LINE_ALIGNED((w + 2 * dis->refine_uv_pad) * 8) >> 3;
  int           refine_pad_line_size    = IMG_LINE_ALIGNED((w + 2 * dis->refine_pad) * 4) >> 2;
  int           refine_line_size        = IMG_LINE_ALIGNED(w * 4) >> 2;
  float*       dense_flow = (float*)(dis->dense_flow_pyramid.buf[cur_level] +
                                dis->dense_flow_pyramid.pad * dis->dense_flow_pyramid.line_size[cur_level] +
                                dis->dense_flow_pyramid.pad * 8);
  float*       dudv  = (float*)(dis->dudv +
                                dis->refine_uv_pad * refine_uv_pad_line_size +
                                dis->refine_uv_pad);
  float*       A00   = dis->A00 + 
                       dis->refine_pad * refine_pad_line_size +
                       dis->refine_pad;
  float*       A01   = dis->A01 + 
                       dis->refine_pad * refine_pad_line_size +
                       dis->refine_pad;
  float*       A11   = dis->A11 + 
                       dis->refine_pad * refine_pad_line_size +
                       dis->refine_pad;
  float*       psi   = (dis->smoothness +
                        dis->refine_pad * refine_pad_line_size +
                        dis->refine_pad);
  float*       b0    = dis->b0;
  float*       b1    = dis->b1;
  float        omega = dis->sor_omega;
  for(k = 0 ; k < sor_iter ; k ++){
    for(i = 0 ; i < h ; i ++){
      for(j = 0 ; j < w ; j ++){
        float a00 = 0.0f;
        float a01 = 0.0f;
        float a11 = 0.0f;  
        float b00 = 0.0f;
        float b01 = 0.0f;    
#if 0  
        a00       = (A00[j] + 2.0f * psi[j] + psi[j - 1] + psi[j - refine_pad_line_size]);
        a01       = (dudv[2 * (j - 1)] * psi[j - 1] +
                     dudv[2 * (j + 1)] * psi[j] +
                     dudv[2 * j - (refine_uv_pad_line_size << 1)] * psi[j - refine_pad_line_size] +
                     dudv[2 * j + (refine_uv_pad_line_size << 1)] * psi[j]) -
                     A01[j] * dudv[2 * j + 1];
        b00       = b0[j]  + a01;
        dudv[2 * j]     += omega * (b00 / a00 - dudv[2 * j]);
        a11       = (A11[j] + 2.0f * psi[j] + psi[j - 1] + psi[j - refine_pad_line_size]);
        a01       = (dudv[2 * (j - 1) + 1] * psi[j - 1] +
                     dudv[2 * (j + 1) + 1] * psi[j] +
                     dudv[2 * j - (refine_uv_pad_line_size << 1) + 1] * psi[j - refine_pad_line_size] +
                     dudv[2 * j + (refine_uv_pad_line_size << 1) + 1] * psi[j]) -
                     A01[j] * dudv[2 * j];
        b01       = b1[j] + a01;
        dudv[2 * j + 1] += omega * (b01 / a11 - dudv[2 * j + 1]);
#else
        float sigma_u = 0.0f, sigma_v = 0.0f, coeff = 0.0f, weight = 0.0f;
        if(j > 0)
        {
          weight    = psi[j - 1];
          sigma_u  += weight * dudv[2 * (j - 1)];
          sigma_v  += weight * dudv[2 * (j - 1) + 1];
          coeff    += weight;
        }
        if(j < w - 1)
        {
          weight    = psi[j];
          sigma_u  += weight * dudv[2 * (j + 1)];
          sigma_v  += weight * dudv[2 * (j + 1) + 1];
          coeff    += weight;
        }
        if(i > 0)
        {
          weight    = psi[j - refine_pad_line_size];
          sigma_u  += weight * dudv[2 * j - (refine_uv_pad_line_size << 1)];
          sigma_v  += weight * dudv[2 * j - (refine_uv_pad_line_size << 1) + 1];
          coeff    += weight;
        }
        if(i < h - 1)
        {
          weight    = psi[j];
          sigma_u  += weight * dudv[2 * j + (refine_uv_pad_line_size << 1)];
          sigma_v  += weight * dudv[2 * j + (refine_uv_pad_line_size << 1) + 1];
          coeff    += weight;
        }
        sigma_u         -= A01[j] * dudv[2 * j + 1];
        a00              = A00[j] + coeff;
        b00              = b0[j]  + sigma_u;
        dudv[2 * j]     += omega * (b00 / a00 - dudv[2 * j]);
        sigma_v         -= A01[j] * dudv[2 * j];
        a11              = A11[j] + coeff;
        b01              = b1[j]  + sigma_v;
        dudv[2 * j + 1] += omega * (b01 / a11 - dudv[2 * j + 1]);
#endif
      }
      dudv += (refine_uv_pad_line_size << 1);
      A00  += refine_pad_line_size;
      A01  += refine_pad_line_size;
      A11  += refine_pad_line_size;
      b0   += refine_line_size;
      b1   += refine_line_size;
      psi  += refine_pad_line_size;
    }
    dudv  = dudv - (refine_uv_pad_line_size << 1) * h;
    A00   = A00  - refine_pad_line_size * h;
    A01   = A01  - refine_pad_line_size * h;
    A11   = A11  - refine_pad_line_size * h;
    b0    = b0   - refine_line_size * h;
    b1    = b1   - refine_line_size * h;
    psi   = psi  - refine_pad_line_size * h;
  }
}

static void recompute_uv(DIS_INSTANCE*   dis, 
                         unsigned int    cur_level){
  int  i  = 0 ;
  int  j  = 0 ;
  int  w  = dis->dense_flow_pyramid.width[cur_level];
  int  h  = dis->dense_flow_pyramid.height[cur_level];
  int  refine_uv_pad_line_size = IMG_LINE_ALIGNED((w + 2 * dis->refine_uv_pad) * 8) >> 3;
  int     dense_flow_line_size = dis->dense_flow_pyramid.line_size[cur_level];
  float*  dense_flow           = (float*)(dis->dense_flow_pyramid.buf[cur_level] +
                                          dis->dense_flow_pyramid.pad * dense_flow_line_size +
                                          dis->dense_flow_pyramid.pad * 8);
  float*  uv                   = (float*)(dis->uv + 
                                          dis->refine_uv_pad * refine_uv_pad_line_size +
                                          dis->refine_uv_pad);
  float*  dudv                 = (float*)(dis->dudv + 
                                          dis->refine_uv_pad * refine_uv_pad_line_size +
                                          dis->refine_uv_pad);
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < w ; j ++){
      uv[2 * j]     = dense_flow[2 * j]     + dudv[2 * j];
      uv[2 * j + 1] = dense_flow[2 * j + 1] + dudv[2 * j + 1];
    }
    uv         += (refine_uv_pad_line_size << 1);
    dudv       += (refine_uv_pad_line_size << 1);
    dense_flow += (dense_flow_line_size >> 2);
  }
  fill_uv_border((long long*)(dis->dense_flow_pyramid.buf[cur_level]),   
                 dis->dense_flow_pyramid.pad, w, h);
  fill_uv_border(dis->uv,   dis->refine_uv_pad, w, h);
}

static void copy_back_dense_flow(DIS_INSTANCE*   dis, 
                                 unsigned int    cur_level){
  int  i  = 0 ;
  int  j  = 0 ;
  int  w  = dis->dense_flow_pyramid.width[cur_level];
  int  h  = dis->dense_flow_pyramid.height[cur_level];
  int  refine_uv_pad_line_size = IMG_LINE_ALIGNED(((w + 2 * dis->refine_uv_pad) * 8)) >> 3;
  if(dis->dense_flow_pyramid.pad == dis->refine_uv_pad &&
     dis->dense_flow_pyramid.line_size[cur_level] == refine_uv_pad_line_size * 8){
    memcpy(dis->dense_flow_pyramid.buf[cur_level], 
           dis->uv, 
           (refine_uv_pad_line_size << 3) * (h + 2 * dis->refine_uv_pad));
  }else{
    unsigned long long*  dst_flow = (unsigned long long*)(dis->dense_flow_pyramid.buf[cur_level] +
                                    dis->dense_flow_pyramid.pad * dis->dense_flow_pyramid.line_size[cur_level] +
                                    dis->dense_flow_pyramid.pad * 8);
    unsigned long long*  src_uv   = (unsigned long long*)(dis->dudv +
                                    dis->refine_uv_pad * refine_uv_pad_line_size +
                                    dis->refine_uv_pad);
    for(i = 0 ; i < h ; i ++){
      memcpy(dst_flow, src_uv, 8 * w);
      dst_flow += (dis->dense_flow_pyramid.line_size[cur_level] >> 3);
      src_uv   += refine_uv_pad_line_size;
    }
  }
}

int     variant_refine(DIS_INSTANCE*   dis, 
                       unsigned int    cur_level, 
                       unsigned int    fixed_point_iter,
                       unsigned int    sor_iter){
  clear_dense_flow_border(dis, cur_level);
  prepare_terms(dis, cur_level);
  int i= 0 ;
  for(i = 0 ; i < fixed_point_iter ; i ++){
    cacl_data_term(dis, cur_level);
    cacl_smooth_term(dis, cur_level);
    move_smooth_term(dis, cur_level);
    solve_sor(dis, cur_level, sor_iter);
    recompute_uv(dis, cur_level);
    {
      char           file_name[256] = { 0 };
      sprintf(file_name, "%s/refine_flow_%d_%d.bmp", DEBUG_IMG_PREFIX, cur_level, i);
      draw_flow(file_name, 
                dis->dense_flow_pyramid.width[cur_level],
                dis->dense_flow_pyramid.height[cur_level],
                (float const*)(dis->uv + 
                dis->refine_uv_pad * (IMG_LINE_ALIGNED((dis->dense_flow_pyramid.width[cur_level] * 8 + 2 * dis->refine_uv_pad * 8)) >> 3) + 
                dis->refine_uv_pad),
                IMG_LINE_ALIGNED((dis->dense_flow_pyramid.width[cur_level] * 8 + 2 * 8 * dis->refine_uv_pad)),
                20.0f);
    }
  }
  copy_back_dense_flow(dis, cur_level);
  clear_dense_flow_border(dis, cur_level);
  {
    char           file_name[256] = { 0 };
    sprintf(file_name, "%s/dense_flow_%d.bmp", DEBUG_IMG_PREFIX, cur_level);
    draw_flow(file_name, 
              dis->dense_flow_pyramid.width[cur_level],
              dis->dense_flow_pyramid.height[cur_level],
              (float const*)(dis->dense_flow_pyramid.buf[cur_level] + 
              dis->dense_flow_pyramid.pad * dis->dense_flow_pyramid.line_size[cur_level] + 
              dis->dense_flow_pyramid.pad * 8),
              dis->dense_flow_pyramid.line_size[cur_level],
              20.0f);
  }
  return 0;
}
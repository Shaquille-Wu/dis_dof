#include "dis_variant_refine.h"
#include <math.h>
#include <string.h>

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
               dis->dense_flow_pyramid.pad * (dense_flow_line_size >> 3);
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
  memset(dst_img, 0, shift_img_pad_line_size * (2 * shift_img_pad + h));

  int    src_img_line_size      = dis->ref_gray_pyramid.line_size[cur_level];
  int    flow_img_line_size     = (dis->dense_flow_pyramid.line_size[cur_level] >> 2);
  int                   i             = 0;
  int                   j             = 0;
  unsigned char const*  src_ptr0      = src_image;
  unsigned char const*  src_ptr1      = src_image + src_img_line_size;
  shift_img_line_size = shift_img_line_size >> 2;
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < w ; j ++){
      float  delta_x_f = flow_image[2 * j];
      float  delta_y_f = flow_image[2 * j + 1];
      float  x_f       = j + delta_x_f;
      float  y_f       = i + delta_y_f;
      int    valid     = (x_f >= 0.0f) & (x_f <= ((float)(w - 1))) & (y_f >= 0.0f) & (y_f <= ((float)(h - 1)));
      mask_img[j]      = (float)valid;
      x_f              = (x_f < 0.0f ? 0.0f : x_f);
      y_f              = (y_f < 0.0f ? 0.0f : y_f);
      x_f              = (x_f > ((float)(w - 1)) ? ((float)(w - 1)) : x_f);
      y_f              = (y_f > ((float)(h - 1)) ? ((float)(h - 1)) : y_f);
      float  u         = x_f - ((int)x_f);
      float  v         = y_f - ((int)y_f);
      float  res       = src_ptr0[j]     * (1.0f - u) * (1.0f - v) +
                         src_ptr0[j + 1] *          u * (1.0f - v) +
                         src_ptr1[j]     * (1.0f - u) *         v +
                         src_ptr1[j + 1] *          u *         v;
      dst_img[j]       = res;
    }
    src_image  += src_img_line_size;
    flow_image += flow_img_line_size;
    dst_img    += shift_img_pad_line_size;
    mask_img   += shift_img_line_size;
  }
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
      Ix[j]    = x * 0.25f;
      Iy[j]    = y * 0.25f;
      int ref_x = ((ref_Ixy[j] << 16) >> 16);
      int ref_y = (ref_Ixy[j]         >> 16);
      Ixt[j]   = Ix[j] - (float)ref_x;
      Iyt[j]   = Iy[j] - (float)ref_y;
    }
    Ix      += refine_pad_line_size;
    Iy      += refine_pad_line_size;
    Ixt     += refine_line_size;
    Iyt     += refine_line_size;
    ref_Ixy += ref_Ixy_line_size;
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
      float br_y = Ix[j + 1 + refine_pad_line_size];
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
                                      dis->refine_pad * refine_uv_pad_line_size +
                                      dis->refine_pad);
  float   weight_delta     = dis->tv_delta;
  float   weight_gamma     = dis->tv_gamma;
  int     i = 0, j = 0;
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < w ; j ++){
      float norm0  = Ix[j] * Ix[j] + Iy[j] * Iy[j] + DATA_NORM;
      float tmp0   = It[j] + Ix[j] * dudv[2 * j] + Iy[j] * dudv[2 * j + 1];
      tmp0         = mask[j] * weight_delta / sqrtf((tmp0 * tmp0) / norm0 + DATA_BRI_EPS);
      tmp0        /= norm0;
      A00[j] += tmp0  * Ix[j] * Ix[j];
      A01[j] += tmp0  * Ix[j] * Iy[j];
      A11[j] += tmp0  * Iy[j] * Iy[j];
      b0[j]  -= tmp0  * It[j] * Ix[j];
      b1[j]  -= tmp0  * It[j] * Ix[j];

      norm0       = Ixx[j] * Ixx[j] + Ixy[j] * Ixy[j] + DATA_NORM;
      float norm1 = Iyy[j] * Iyy[j] + Ixy[j] * Ixy[j] + DATA_NORM;
      tmp0        = Ixt[j] + Ixx[j] * dudv[2 * j] + Ixy[j] * dudv[2 * j + 1];
      float tmp1  = Iyt[j] + Ixy[j] * dudv[2 * j] + Iyy[j] * dudv[2 * j + 1];
      tmp0        = mask[j] * weight_gamma / sqrtf(tmp0 * tmp0/norm0 + tmp1*tmp1/norm1 + DATA_GRAD_EPS);
      tmp0        = tmp0/norm0;
      tmp1        = tmp0/norm1;

      A00[j] += tmp0 * Ixx[j] * Ixx[j] + tmp1 * Ixy[j] * Ixy[j];
      A01[j] += tmp0 * Ixx[j] * Ixy[j] + tmp1 * Ixy[j] * Iyy[j];
      A11[j] += tmp1 * Iyy[j] * Iyy[j] + tmp0 * Ixy[j] * Ixy[j];
      b0[j]  -= tmp0 * Ixx[j] * Ixt[j] + tmp1 * Ixy[j] * Iyt[j];
      b1[j]  -= tmp1 * Iyy[j] * Iyt[j] + tmp0 * Ixy[j] * Ixt[j];
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
  int       refine_pad_line_size    = IMG_LINE_ALIGNED((w + 2 * dis->refine_pad) * 4) >> 2;
  float const*  uv = (float const*)(dis->uv +
                                    dis->refine_uv_pad * refine_uv_pad_line_size +
                                    dis->refine_uv_pad);
  float*        smooth_uv  = (float*)(dis->smooth_uv);
  float*        smoothness = dis->smoothness;
  memset(smooth_uv,  0, (refine_uv_pad_line_size << 3) * (h + 2 * dis->refine_uv_pad));
  memset(smoothness, 0, (refine_pad_line_size << 2)    * (h + 2 * dis->refine_pad));

  int       uv_line_size        = refine_uv_pad_line_size << 1;
  int       i = 0, j = 0;
  float     tv_alpha            = dis->tv_alpha;
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < w ; j ++){
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
      uv_norm        = tv_alpha / sqrtf(uv_norm);
      float  div_u   = tl_u + t_u + tr_u + l_u + r_u + bl_u + b_u + br_u;
      float  div_v   = tl_v + t_v + tr_v + l_v + r_v + bl_v + b_v + br_v;
      smoothness[j]        = uv_norm;
      smooth_uv[2 * j]     = uv_norm * (div_u - 8.0f * c_u) * 0.5f;
      smooth_uv[2 * j + 1] = uv_norm * (div_v - 8.0f * c_v) * 0.5f;
    }
    smooth_uv  += uv_line_size;
    uv         += uv_line_size;
    smoothness += refine_pad_line_size;
  }
}

static void move_smooth_term(DIS_INSTANCE* dis, unsigned int cur_level){
  int           w                       = dis->dense_flow_pyramid.width[cur_level];
  int           h                       = dis->dense_flow_pyramid.height[cur_level];
  int           refine_uv_pad_line_size = IMG_LINE_ALIGNED((w + 2 * dis->refine_uv_pad) * 8) >> 3;
  int           refine_line_size        = IMG_LINE_ALIGNED(w * 4) >> 2;
  float const*  uv                      = (float const*)(dis->uv +
                                           dis->refine_uv_pad * refine_uv_pad_line_size +
                                           dis->refine_uv_pad);
  float*        smooth_uv  = (float*)(dis->smooth_uv + 
                                      dis->refine_uv_pad * refine_uv_pad_line_size +
                                      dis->refine_uv_pad);
  float*        b0         = dis->b0;
  float*        b1         = dis->b1;
  int           i = 0, j = 0;
  for(i = 0 ; i < h ; i ++){
    for(j = 0 ; j < w ; j ++){
      b0[j] = b0[j] + smooth_uv[2 * j];
      b1[j] = b1[j] + smooth_uv[2 * j + 1];
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
  float*       b0    = dis->b0;
  float*       b1    = dis->b1;
  float        omega = dis->sor_omega;
  for(k = 0 ; k < sor_iter ; k ++){
    for(i = 0 ; i < h ; i ++){
      for(j = 0 ; j < w ; j ++){
        float sigma_u   = 0.0f;
        float sigma_v   = 0.0f;

        sigma_u = dudv[2 * (j - 1)] + dudv[2 * (j + 1)] + dudv[2 * j - (refine_uv_pad_line_size << 1)] +
                  dudv[2 * j + (refine_uv_pad_line_size << 1)];
        sigma_v = dudv[2 * (j - 1) + 1] + dudv[2 * (j + 1) + 1] + dudv[2 * j - (refine_uv_pad_line_size << 1) + 1] +
                  dudv[2 * j + (refine_uv_pad_line_size << 1) + 1];
        dudv[2 * j]     += omega * ((sigma_u + b0[j] - dudv[2 * j]     * A01[j]) / A00[j] - dudv[2 * j]);
        dudv[2 * j + 1] += omega * ((sigma_v + b1[j] - dudv[2 * j + 1] * A01[j]) / A11[j] - dudv[2 * j + 1]);
      }
      dudv += (refine_uv_pad_line_size << 1);
      A00  += refine_pad_line_size;
      A01  += refine_pad_line_size;
      A11  += refine_pad_line_size;
      b0   += refine_line_size;
      b1   += refine_line_size;
    }
    dudv  = dudv - (refine_uv_pad_line_size << 1) * h;
    A00   = A00  - refine_pad_line_size * h;
    A01   = A01  - refine_pad_line_size * h;
    A01   = A01  - refine_pad_line_size * h;
    b0    = b0   - refine_line_size * h;
    b1    = b1   - refine_line_size * h;
  }
}

static void recompute_uv(DIS_INSTANCE*   dis, 
                         unsigned int    cur_level){
  int  i  = 0 ;
  int  j  = 0 ;
  int  w  = dis->dense_flow_pyramid.width[cur_level];
  int  h  = dis->dense_flow_pyramid.height[cur_level];
  int  refine_uv_pad_line_size = IMG_LINE_ALIGNED((w + 2 * dis->refine_uv_pad) * 8) >> 3;
  int  dense_flow_line_size    = dis->dense_flow_pyramid.line_size[cur_level];
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
    dense_flow += (dense_flow_line_size >> 2);
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
  }
  return 0;
}
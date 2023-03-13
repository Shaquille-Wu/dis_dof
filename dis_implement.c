#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <align_mem.h>
#include <cv_preproc.h>
#include <bmp.h>
#include "dis_implement.h"
#include "dis_internal_def.h"
#include "pyramid.h"

#define DEBUG_IMG_PREFIX    "/home/icework/adas_alg_ref/dof_dis/data/output"

#define MAX_F32(x, y)    (x) < (y) ? (y) : (x)

#define DET_EPS 0.001F

void*   create_dis_instance(unsigned int  img_width, 
                            unsigned int  img_height){
  
  unsigned int    total_mem_size = 0;
  unsigned int    i              = 0;
  DIS_INSTANCE*   dis = (DIS_INSTANCE*)alloc_mem_align(sizeof(DIS_INSTANCE));
  memset(dis, 0, sizeof(DIS_INSTANCE));
  unsigned int  pyr_level        = 6;
  unsigned int  btm_width        = (img_width + 3) >> 2;
  unsigned int  btm_height       = (img_height + 3) >> 2;
  dis->src_width  = img_width;
  dis->src_height = img_height;
  dis->pyr_level  = pyr_level - 2;

  btm_width  = (((btm_width + (1 << (dis->pyr_level - 1)) - 1) >> 
                (dis->pyr_level - 1))
                << (dis->pyr_level - 1));
  btm_height = (((btm_height + (1 << (dis->pyr_level - 1)) - 1) >> 
                (dis->pyr_level - 1))
                << (dis->pyr_level - 1));
  create_pyramid(btm_width, 
                 btm_height, 
                 PYRAMID_PIXEL_U8, 
                 dis->pyr_level, 
                 DIS_DOF_WND_SIZE,
                 &(dis->ref_gray_pyramid));
  total_mem_size += dis->ref_gray_pyramid.total_buf_size;
  create_pyramid(btm_width, 
                 btm_height, 
                 PYRAMID_PIXEL_U8, 
                 dis->pyr_level, 
                 DIS_DOF_WND_SIZE,
                 &(dis->track_gray_pyramid));
  total_mem_size += dis->track_gray_pyramid.total_buf_size;
  create_pyramid(btm_width, 
                 btm_height, 
                 PYRAMID_PIXEL_U64, 
                 dis->pyr_level - 1, 
                 DIS_DOF_WND_SIZE,
                 &(dis->init_flow_pyramid));
  total_mem_size += dis->init_flow_pyramid.total_buf_size;
  create_pyramid(btm_width,
                 btm_height,
                 PYRAMID_PIXEL_U32,
                 dis->pyr_level,
                 4,
                 &(dis->ref_grad_xy_pyramid));
  total_mem_size += dis->ref_grad_xy_pyramid.total_buf_size;
  create_pyramid(btm_width * 4,
                 btm_height,
                 PYRAMID_PIXEL_U64,
                 dis->pyr_level,
                 2,
                 &(dis->grad_xy_integral_pyramid));
  total_mem_size += dis->grad_xy_integral_pyramid.total_buf_size;
  create_pyramid(btm_width,
                 btm_height,
                 PYRAMID_PIXEL_U64,
                 dis->pyr_level,
                 2,
                 &(dis->dense_flow_pyramid));

  unsigned int btm_img_size  = dis->ref_gray_pyramid.line_size[0] * 
                              dis->ref_gray_pyramid.height[0];
  dis->ref_btm_resize_img    = (unsigned char*)alloc_mem_align(2 * btm_img_size);
  dis->track_btm_resize_img  = dis->ref_btm_resize_img + btm_img_size;
  total_mem_size += 2 * btm_img_size;

  unsigned int patch_stride_blk_w[DIS_PYRAMID_MAX] = { 0, 0, 0, 0,
                                                       0, 0, 0, 0
                                                     };
  unsigned int patch_stride_blk_h[DIS_PYRAMID_MAX] = { 0, 0, 0, 0,
                                                       0, 0, 0, 0
                                                     };
  for(i = 0 ; i < dis->pyr_level ; i ++){
    unsigned int cur_width  = dis->ref_gray_pyramid.width[i];
    unsigned int cur_height = dis->ref_gray_pyramid.height[i];
    patch_stride_blk_w[i] = ((cur_width + DIS_PATCH_STRIDE - 1) / 
                                   DIS_PATCH_STRIDE) - 1;
    patch_stride_blk_h[i] = ((cur_height + DIS_PATCH_STRIDE - 1) / 
                                   DIS_PATCH_STRIDE) - 1;
  }
  create_pyramid_with_size(patch_stride_blk_w,
                           patch_stride_blk_h,
                           PYRAMID_PIXEL_U64,
                           dis->pyr_level,
                           2,
                           &(dis->patch_stride_flow_pyramid));
  total_mem_size += dis->patch_stride_flow_pyramid.total_buf_size;
  create_pyramid_with_size(patch_stride_blk_w,
                           patch_stride_blk_h,
                           PYRAMID_PIXEL_U64,
                           dis->pyr_level,
                           4,
                           &(dis->grad_xy_stride_integral_pyramid));
  total_mem_size += dis->grad_xy_stride_integral_pyramid.total_buf_size;
  for(i = 0 ; i < dis->pyr_level ; i ++){
    patch_stride_blk_w[i] = patch_stride_blk_w[i] * 4;
  }
  create_pyramid_with_size(patch_stride_blk_w,
                           patch_stride_blk_h,
                           PYRAMID_PIXEL_U32,
                           dis->pyr_level,
                           0,
                           &(dis->hessian_inv_pyramid));
  total_mem_size += dis->hessian_inv_pyramid.total_buf_size;

  dis->mem_size          = total_mem_size;
  dis->patch_size        = DIS_PATCH_SIZE;
  dis->patch_stride_size = DIS_PATCH_STRIDE;

  return dis;
}

int     destroy_dis_instance(void* dis_instance){
  DIS_INSTANCE*  dis = (DIS_INSTANCE*)dis_instance;
  destroy_pyramid(&(dis->ref_gray_pyramid));
  destroy_pyramid(&(dis->track_gray_pyramid));
  destroy_pyramid(&(dis->patch_stride_flow_pyramid));
  destroy_pyramid(&(dis->init_flow_pyramid));
  destroy_pyramid(&(dis->ref_grad_xy_pyramid));
  destroy_pyramid(&(dis->grad_xy_stride_integral_pyramid));
  destroy_pyramid(&(dis->hessian_inv_pyramid));
  destroy_pyramid(&(dis->dense_flow_pyramid));
  if(NULL != dis->ref_btm_resize_img){
    free_mem_align(dis->ref_btm_resize_img);
    dis->ref_btm_resize_img = NULL;
  }
  return 0;
}

static void  build_grad_xy_integral_pyr(DIS_PYRAMID const*  grad_xy_pyramid,
                                        int                 patch_size,
                                        int                 stride_size,
                                        unsigned char*      work_buf,
                                        unsigned int        work_buf_size,
                                        DIS_PYRAMID*        grad_xy_stride_integral_pyramid,
                                        DIS_PYRAMID*        grad_xy_integral_pyramid){
  unsigned int integral_pad                = 2;
  unsigned int expected_work_buf_line_size = (((grad_xy_pyramid->width[0] + 2 * integral_pad) * 8 + 15) >> 4) << 4;   
  unsigned int expected_work_buf_size      = expected_work_buf_line_size * (grad_xy_pyramid->height[0] + integral_pad);
  if(expected_work_buf_size > work_buf_size){
    return;
  }
  memset(work_buf, 
         0, 
         expected_work_buf_size);
  memset(grad_xy_stride_integral_pyramid->mem, 
         0, 
         grad_xy_stride_integral_pyramid->total_buf_size);
  memset(grad_xy_integral_pyramid->mem, 
         0, 
         grad_xy_integral_pyramid->total_buf_size);
  for(int l = 0 ; l < grad_xy_pyramid->level ; l ++){
    int                  src_line_size = grad_xy_pyramid->line_size[l] >> 2;
    unsigned int const*  src_ptr       = (unsigned int const*)(grad_xy_pyramid->buf[l] + 
                                     grad_xy_pyramid->pad * grad_xy_pyramid->line_size[l] +
                                     grad_xy_pyramid->pad * 4);
    unsigned int         width         = grad_xy_pyramid->width[l];
    unsigned int         height        = grad_xy_pyramid->height[l];
    int                  dst_line_size = ((((width + 2 * integral_pad) * 8 + 15) >> 4) << 4) >> 2;   
    int*                 dst_ptr       = (int*)(work_buf + 
                                                integral_pad * (dst_line_size << 2) +
                                                8 * integral_pad);
    unsigned int         grad_xy_integral_line_size = grad_xy_integral_pyramid->line_size[l] >> 3;
    long long int*       grad_xy_integral_ptr = (long long int*)(grad_xy_integral_pyramid->buf[l] + 
                                                grad_xy_integral_pyramid->pad * 
                                                (grad_xy_integral_line_size << 3) +
                                                32 * integral_pad);
    int i = 0, j = 0;
    int                  cur_int    = 0;
    int                  cur_val_x  = 0;
    int                  cur_val_y  = 0;
    int                  cur_val_xx = 0;
    int                  cur_val_yy = 0;
    int                  cur_val_xy = 0;
    for (j = 0; j < width ; j++){
      int cur_src        = src_ptr[j];
      int cur_x          = (cur_src << 16) >> 16;
      int cur_y          = cur_src >> 16;
      cur_val_x         += cur_x;
      cur_val_y         += cur_y;
      cur_val_xx        += cur_x * cur_x;
      cur_val_yy        += cur_y * cur_y;
      cur_val_xy        += cur_x * cur_y;
      dst_ptr[2 * j]     = cur_val_x ;
      dst_ptr[2 * j + 1] = cur_val_y ;
      grad_xy_integral_ptr[4 * j]     = cur_val_xx;
      grad_xy_integral_ptr[4 * j + 1] = cur_val_yy;
      grad_xy_integral_ptr[4 * j + 2] = cur_val_xy;
    }
    cur_val_x = 0;
    cur_val_y = 0;
    cur_val_xx = 0;
    cur_val_yy = 0;
    cur_val_xy = 0;
    for (i = 1; i < height ; i++){
      for (j = 0; j < width ; j++){
        int cur_src        = src_ptr[i * src_line_size + j];
        int cur_x          = (cur_src << 16) >> 16;
        int cur_y          = cur_src >> 16;
        cur_val_x  += cur_x;
        cur_val_y  += cur_y;
        cur_val_xx += cur_x * cur_x;
        cur_val_yy += cur_y * cur_y;
        cur_val_xy += cur_x * cur_y;
        dst_ptr[i * dst_line_size + 2 * j]     = cur_val_x + dst_ptr[(i - 1) * dst_line_size + 2 * j];
        dst_ptr[i * dst_line_size + 2 * j + 1] = cur_val_y + dst_ptr[(i - 1) * dst_line_size + 2 * j + 1];

        grad_xy_integral_ptr[i * grad_xy_integral_line_size + 4 * j]     = ((long long int)cur_val_xx) + grad_xy_integral_ptr[(i - 1) * grad_xy_integral_line_size + 4 * j];
        grad_xy_integral_ptr[i * grad_xy_integral_line_size + 4 * j + 1] = ((long long int)cur_val_yy) + grad_xy_integral_ptr[(i - 1) * grad_xy_integral_line_size + 4 * j + 1];
        grad_xy_integral_ptr[i * grad_xy_integral_line_size + 4 * j + 2] = ((long long int)cur_val_xy) + grad_xy_integral_ptr[(i - 1) * grad_xy_integral_line_size + 4 * j + 2];
      }
      cur_val_x = 0;
      cur_val_y = 0;
    }
    int   dst_width        = grad_xy_stride_integral_pyramid->width[l];
    int   dst_height       = grad_xy_stride_integral_pyramid->height[l];
    int*  stride_integral  = (int*)(grad_xy_stride_integral_pyramid->buf[l] + 
                                    grad_xy_stride_integral_pyramid->pad * 
                                    grad_xy_stride_integral_pyramid->line_size[l] +
                                    grad_xy_stride_integral_pyramid->pad * 8);
    int   stride_line_size = grad_xy_stride_integral_pyramid->line_size[l] >> 2;
    for(i = 0 ; i < dst_height ; i ++){
      int top  = i * stride_size - 1;
      int btm  = (i + 1) * patch_size - 1;
      btm      = btm > height ? height - 1 : btm;
      for(j = 0 ; j < dst_width ; j ++){
        int left   = j * stride_size - 1;
        int right  = (j + 1) * patch_size - 1;
        right      = right > width ? width - 1 : right;
        int Ax     = dst_ptr[top * dst_line_size + 2 * left];
        int Ay     = dst_ptr[top * dst_line_size + 2 * left + 1];
        int Bx     = dst_ptr[top * dst_line_size + 2 * right];
        int By     = dst_ptr[top * dst_line_size + 2 * right + 1];
        int Cx     = dst_ptr[btm * dst_line_size + 2 * left];
        int Cy     = dst_ptr[btm * dst_line_size + 2 * left + 1];
        int Dx     = dst_ptr[btm * dst_line_size + 2 * right];
        int Dy     = dst_ptr[btm * dst_line_size + 2 * right + 1];
        stride_integral[2 * j]     = Dx - Bx - Cx + Ax;
        stride_integral[2 * j + 1] = Dy - By - Cy + Ay;
      }
      stride_integral += stride_line_size;
    }
  }
}

static void  build_hessian_inv_pyr(DIS_PYRAMID const*  grad_xy_integral_pyramid,
                                   int                 patch_size,
                                   int                 stride_size,
                                   DIS_PYRAMID*        hessian_inv_pyramid){
  memset(hessian_inv_pyramid->mem, 
         0, 
         hessian_inv_pyramid->total_buf_size);
  for(int l = 0 ; l < hessian_inv_pyramid->level ; l ++){
    int             src_width  = grad_xy_integral_pyramid->width[l] >> 2;
    int             src_height = grad_xy_integral_pyramid->height[l] >> 2;
    long long int*  integral   = (long long int*)(grad_xy_integral_pyramid->buf[l] + 
                                    grad_xy_integral_pyramid->pad * 
                                    grad_xy_integral_pyramid->line_size[l] +
                                    grad_xy_integral_pyramid->pad * 32);
    int             integral_line_size = (grad_xy_integral_pyramid->line_size[l] >> 3);
    int             dst_width   = hessian_inv_pyramid->width[l] >> 2;
    int             dst_height  = hessian_inv_pyramid->height[l];
    float*          hessian_inv = (float*)(hessian_inv_pyramid->buf[l]);
    int             hessian_line_size = hessian_inv_pyramid->line_size[l] >> 2;
    int   i = 0, j = 0;
    for(i = 0 ; i < dst_height ; i ++){
      int top  = i * stride_size - 1;
      int btm  = (i + 1) + patch_size - 1;
      btm      = btm > src_height ? src_height - 1 : btm;
      for(j = 0 ; j < dst_width ; j ++){
        int left   = j * stride_size - 1;
        int right  = (j + 1) * patch_size - 1;
        right      = right > src_width ? src_width - 1 : right;
        long long int Ax    = integral[top * integral_line_size + 4 * left];
        long long int Ay    = integral[top * integral_line_size + 4 * left + 1];
        long long int Az    = integral[top * integral_line_size + 4 * left + 2];
        long long int Bx    = integral[top * integral_line_size + 4 * right];
        long long int By    = integral[top * integral_line_size + 4 * right + 1];
        long long int Bz    = integral[top * integral_line_size + 4 * right + 2];
        long long int Cx    = integral[btm * integral_line_size + 4 * left];
        long long int Cy    = integral[btm * integral_line_size + 4 * left + 1];
        long long int Cz    = integral[btm * integral_line_size + 4 * left + 2];
        long long int Dx    = integral[btm * integral_line_size + 4 * right];
        long long int Dy    = integral[btm * integral_line_size + 4 * right + 1];
        long long int Dz    = integral[btm * integral_line_size + 4 * right + 2];
        float         Ixx   = Dx - Bx - Cx + Ax;
        float         Iyy   = Dy - By - Cy + Ay;
        float         Ixy   = Dz - Bz - Cz + Az;
        float         det_h = Ixx * Iyy - Ixy * Ixy;
        if(fabsf(det_h) < DET_EPS){
          hessian_inv[4 * j]     = 0.0f;
          hessian_inv[4 * j + 1] = 0.0f;
          hessian_inv[4 * j + 2] = 0.0f;
        }else{
          hessian_inv[4 * j]     =  Iyy / det_h;
          hessian_inv[4 * j + 1] = -Ixy / det_h;
          hessian_inv[4 * j + 2] =  Ixx / det_h;
        }
      }
      hessian_inv += hessian_line_size;
    }
  }
}

static float calc_patch_ssd(unsigned char const* ref,
                            unsigned char const* track,
                            int const*           ref_xy,
                            float const*         flow_xy,
                            unsigned int         patch_size,
                            int                  line_size,
                            int                  track_max_x,
                            int                  track_max_y){
  float track_x_f = ref_xy[0] + flow_xy[0];
  float track_y_f = ref_xy[1] + flow_xy[1];
  int   track_x_i = track_x_f < 0.0f ? (int)(track_x_f - 1 - 0.5f) : (int)(track_x_f + 0.5f);
  int   track_y_i = track_y_f < 0.0f ? (int)(track_y_f - 1 - 0.5f) : (int)(track_y_f + 0.5f);
  float u         = track_x_f - track_x_i;
  float v         = track_y_f - track_y_i;
  track_x_i       = track_x_i < -(DIS_DOF_WND_SIZE - 1) ? (DIS_DOF_WND_SIZE - 1) : track_x_i;
  track_x_i       = track_x_i > track_max_x ? track_max_x : track_x_i;
  track_y_i       = track_y_i < -(DIS_DOF_WND_SIZE - 1) ? (DIS_DOF_WND_SIZE - 1) : track_y_i;
  track_y_i       = track_y_i > track_max_y ? track_max_y : track_y_i;

  unsigned char const*  ref_ptr    = ref + 
                                     ref_xy[0] * line_size + ref_xy[1];
  unsigned char const*  track_ptr0 = track + 
                                    track_x_i * line_size + track_y_i;
  unsigned char const*  track_ptr1 = track_ptr0 + 
                                    line_size;
  int    i = 0, j = 0;
  float  ssd  = 0.0f;
  float  u0v0 = (1.0f - u) * (1.0f - v);
  float  u1v0 = u          * (1.0f - v);
  float  u0v1 = (1.0f - u) * v;
  float  u1v1 = u          * v;
  float  diff_sum  = 0.0f;
  float  diff_sum2 = 0.0f;
  for(i = 0 ; i < patch_size ; i ++){
    for(j = 0 ; j < patch_size ; j ++){
      float  ref_val   = ref_ptr[j];
      float  track_val = u0v0 * track_ptr0[j] + 
                         u1v0 * track_ptr0[j + 1] +
                         u0v1 * track_ptr1[j] +
                         u1v1 * track_ptr1[j + 1];
      float  diff      = track_val - ref_val;
      diff_sum  += diff;
      diff_sum2 += diff * diff;
    }
    ref_ptr    += line_size;
    track_ptr0 += line_size;
    track_ptr1 += line_size;
  }
  return diff_sum2 - ((diff_sum * diff_sum) / (patch_size * patch_size));
}

static float dis_patch(int const*             ref_xy,
                       float*                 flow_xy,
                       unsigned char const*   ref_img,
                       unsigned char const*   track_img,
                       int                    img_line_size,
                       int const*             ref_grad_xy,
                       unsigned int           ref_grad_xy_line_size,
                       int                    grad_sum_x,
                       int                    grad_sum_y,
                       int                    patch_size,
                       int                    track_max_x,
                       int                    track_max_y){
  float track_x_f = ref_xy[0] + flow_xy[0];
  float track_y_f = ref_xy[1] + flow_xy[1];
  int   track_x_i = track_x_f < 0.0f ? (int)(track_x_f - 1 - 0.5f) : (int)(track_x_f + 0.5f);
  int   track_y_i = track_y_f < 0.0f ? (int)(track_y_f - 1 - 0.5f) : (int)(track_y_f + 0.5f);
  float u         = track_x_f - track_x_i;
  float v         = track_y_f - track_y_i;
  track_x_i       = track_x_i < -(DIS_DOF_WND_SIZE - 1) ? (DIS_DOF_WND_SIZE - 1) : track_x_i;
  track_x_i       = track_x_i > track_max_x ? track_max_x : track_x_i;
  track_y_i       = track_y_i < -(DIS_DOF_WND_SIZE - 1) ? (DIS_DOF_WND_SIZE - 1) : track_y_i;
  track_y_i       = track_y_i > track_max_y ? track_max_y : track_y_i;

  unsigned char const*  track_ptr0 = track_img + 
                                     track_x_i * img_line_size + track_y_i;
  unsigned char const*  track_ptr1 = track_ptr0 + img_line_size;
  int    i = 0, j = 0;
  float  ssd  = 0.0f;
  float  u0v0 = (1.0f - u) * (1.0f - v);
  float  u1v0 = u          * (1.0f - v);
  float  u0v1 = (1.0f - u) * v;
  float  u1v1 = u          * v;
  float  diff_sum  = 0.0f;
  float  diff_sum2 = 0.0f;
  float  sum_x_mul = 0.0, sum_y_mul = 0.0;
  for(i = 0 ; i < patch_size ; i ++){
    for(j = 0 ; j < patch_size ; j ++){
      float  ref_val   = ref_img[j];
      float  track_val = u0v0 * track_ptr0[j] + 
                         u1v0 * track_ptr0[j + 1] +
                         u0v1 * track_ptr1[j] +
                         u1v1 * track_ptr1[j + 1];
      float  diff      = track_val - ref_val;
      diff_sum  += diff;
      diff_sum2 += diff * diff;

      int    ref_grad   = ref_grad_xy[j];
      int    ref_grad_x = (ref_grad << 16) >> 16;
      int    ref_grad_y = ref_grad >> 16;

      sum_x_mul += diff * ref_grad_x;
      sum_y_mul += diff * ref_grad_y;
    }
    ref_img     += img_line_size;
    track_ptr0  += img_line_size;
    track_ptr1  += img_line_size;
    ref_grad_xy += ref_grad_xy_line_size;
  }
  float n    = (patch_size * patch_size);
  flow_xy[0] = sum_x_mul - diff_sum * grad_sum_x / n;
  flow_xy[1] = sum_y_mul - diff_sum * grad_sum_y / n;
  return diff_sum2 - ((diff_sum * diff_sum) / n);
}

static int iterate_dis(unsigned char const*   ref_img,
                       unsigned char const*   track_img,
                       unsigned int           img_line_size,
                       int const*             ref_grad_xy,
                       unsigned int           ref_grad_xy_line_size,
                       float*                 patch_stride,
                       int                    stride_x,
                       int                    stride_y,
                       int const*             Ixy,
                       int                    Ixy_line_size,
                       float const*           hessian_inv,
                       int                    hessian_line_size,
                       int                    iterate_num,
                       int                    patch_size,
                       int                    stride_size,
                       int                    track_max_x,
                       int                    track_max_y){
  int   start_xy[2] = { stride_x * stride_size, stride_y * stride_size };
  float cur_u[2]    = { patch_stride[0], patch_stride[1] };
  float invH11      = hessian_inv[stride_y * hessian_line_size + 4 * stride_x];
  float invH12      = hessian_inv[stride_y * hessian_line_size + 4 * stride_x + 1];
  float invH22      = hessian_inv[stride_y * hessian_line_size + 4 * stride_x + 2];
  int   x_grad_sum  = Ixy[stride_y * Ixy_line_size + 2 * stride_x];
  int   y_grad_sum  = Ixy[stride_y * Ixy_line_size + 2 * stride_x + 1];
  float ssd         = 0.0f;
  int   i           = 0;
  unsigned char const*  ref_ptr         = ref_img + 
                                  start_xy[1] * img_line_size +
                                  start_xy[0];
  int const*            ref_grad_xy_ptr = (int const*)(ref_grad_xy + 
                                  start_xy[1] * ref_grad_xy_line_size +
                                  2 * start_xy[0]);
  float prev_ssd = 1e20;
  for(i = 0 ; i < iterate_num ; i ++){
    float delta_u[2] = { 0.0f, 0.0f } ;
    ssd = dis_patch(start_xy,
                    delta_u,
                    ref_ptr, 
                    track_img,
                    img_line_size,
                    ref_grad_xy_ptr,
                    ref_grad_xy_line_size,
                    x_grad_sum, 
                    y_grad_sum,
                    patch_size,
                    track_max_x,
                    track_max_y);
    float dx = invH11 * delta_u[0] + invH12 * delta_u[1];
    float dy = invH12 * delta_u[0] + invH22 * delta_u[1];
    cur_u[0] -= dx;
    cur_u[1] -= dy;

    if (ssd >= prev_ssd)
        break;
    prev_ssd = ssd;
  }

  float dx   = cur_u[0] - patch_stride[0];
  float dy   = cur_u[1] - patch_stride[1];
  float dist = dx * dx + dy * dy;
  if (dist <= (patch_size * patch_size)){
      patch_stride[0] = cur_u[0];
      patch_stride[1] = cur_u[1];
  }

  return 0;
}

static int patch_inv_search(DIS_INSTANCE*  dis,
                            unsigned int   level,
                            int            internal_iter_num){
  unsigned int    level_iter             = 0;
  int             dir[2]                 = { 1, -1 };
  int             i                      = 0;
  int             j                      = 0;
  DIS_PYRAMID*    patch_stride_pyr       = &(dis->patch_stride_flow_pyramid); 
  int             patch_stride_w         = patch_stride_pyr->width[level];
  int             patch_stride_h         = patch_stride_pyr->height[level];
  int             patch_stride_line_size = patch_stride_pyr->line_size[level] >> 2;
  float*          patch_stride_ptr       = (float*)(patch_stride_pyr->buf[level] + 
                                                    patch_stride_pyr->pad * patch_stride_pyr->line_size[level] +
                                                    8 * patch_stride_pyr->pad);
  unsigned char*  ref_img                = dis->ref_gray_pyramid.buf[level];
  unsigned char*  track_img              = dis->track_gray_pyramid.buf[level];
  unsigned int    gray_img_line_size     = dis->ref_gray_pyramid.line_size[level];
  unsigned int    gray_x_max             = dis->track_gray_pyramid.width[level] - 1;
  unsigned int    gray_y_max             = dis->track_gray_pyramid.height[level] - 1;
  int const*      ref_grad_xy            = (int const*)(dis->ref_grad_xy_pyramid.buf[level] + 
                                            dis->ref_grad_xy_pyramid.pad * dis->ref_grad_xy_pyramid.line_size[level] +
                                            4 * dis->ref_grad_xy_pyramid.pad);
  int             ref_grad_xy_line_size  = dis->ref_grad_xy_pyramid.line_size[level] >> 2;
  float const*    init_flow_ptr          = level < (dis->pyr_level - 1) ? 
                                           (float const*)(dis->init_flow_pyramid.buf[level]) :
                                           NULL;
  int             init_flow_line_size    = dis->init_flow_pyramid.line_size[level] >> 2;
  int             patch_size             = dis->patch_size;
  int             patch_stride_size      = dis->patch_stride_size;
  int const*      stride_grad_xy_integral      = (int const*)(dis->grad_xy_stride_integral_pyramid.buf[level] +
                                                              dis->grad_xy_stride_integral_pyramid.pad * dis->grad_xy_stride_integral_pyramid.line_size[level] +
                                                              8 * dis->grad_xy_stride_integral_pyramid.pad);
  int             stride_grad_xy_int_line_size = dis->grad_xy_stride_integral_pyramid.line_size[level] >> 2;
  float const*    hessian_inv                  = (float const*)(dis->hessian_inv_pyramid.buf[level]);
  int             hessian_inv_line_size        = dis->hessian_inv_pyramid.line_size[level] >> 2;
  float const*    flow_out_ptr                 = (float const*)(dis->dense_flow_pyramid.buf[level] + 
                                                 dis->dense_flow_pyramid.pad * dis->dense_flow_pyramid.line_size[level] +
                                                 8 * dis->dense_flow_pyramid.pad);
  int             flow_out_line_size           = dis->dense_flow_pyramid.line_size[level] >> 2;
  ref_img   = ref_img + 
            dis->ref_gray_pyramid.pad * gray_img_line_size +
            dis->ref_gray_pyramid.pad;
  track_img = track_img + 
            dis->ref_gray_pyramid.pad * gray_img_line_size +
            dis->ref_gray_pyramid.pad;
  init_flow_ptr = init_flow_ptr + 
            (dis->init_flow_pyramid.pad + patch_stride_size) * init_flow_line_size +
            2 * (dis->init_flow_pyramid.pad + patch_stride_size);
  flow_out_ptr  = flow_out_ptr + 
                  patch_stride_size * flow_out_line_size +
                  2 * patch_stride_size;
  float*       patch_stride_ptr0 = patch_stride_ptr;
  float const* flow_out_ptr0     = flow_out_ptr;
  float const* init_flow_ptr0    = init_flow_ptr;
  for(level_iter = 0 ; level_iter < 2 ; level_iter ++){
    int start_i = 0;
    int start_j = 0;
    int ii      = 0;
    int jj      = 0;
    if(1 == level_iter){
      start_i          = patch_stride_h - 1;
      start_j          = patch_stride_w - 1;
      patch_stride_ptr = patch_stride_ptr0 + 
                         (patch_stride_h - 1) * patch_stride_line_size;
      flow_out_ptr     = flow_out_ptr0 + 
                         (patch_stride_h - 1) * patch_stride_size * flow_out_line_size;
      init_flow_ptr    = init_flow_ptr0 + 
                         (patch_stride_h - 1) * patch_stride_size * init_flow_line_size;
    }
    ii = start_i;
    jj = start_j;
    for(i = 0 ; i < patch_stride_h ; i ++){
      for(j = 0 ; j < patch_stride_w ; j ++){
        int   ref_xy[2]  = { 
          jj * patch_stride_size, 
          ii * patch_stride_size
        };
        if (0 == level_iter){
          *((unsigned long long*)(patch_stride_ptr + 2 * jj)) = 
          *((unsigned long long*)(flow_out_ptr + 2 * ref_xy[0]));
        }
        float min_ssd    = calc_patch_ssd(ref_img, 
                                          track_img, 
                                          ref_xy, 
                                          patch_stride_ptr + 2 * jj, 
                                          patch_size, 
                                          gray_img_line_size,
                                          gray_x_max,
                                          gray_y_max);
        float cur_ssd = 0.0f;
        if(level < dis->pyr_level - 1){
          cur_ssd = calc_patch_ssd(ref_img, 
                                   track_img,
                                   ref_xy,
                                   init_flow_ptr + 2 * (jj * patch_stride_size),
                                   patch_size,
                                   gray_img_line_size,
                                   gray_x_max,
                                   gray_y_max);
          if(cur_ssd < min_ssd){
            *((unsigned long long*)(patch_stride_ptr + 2 * jj)) = 
            *((unsigned long long*)(init_flow_ptr + 2 * (jj * patch_stride_size)));
            min_ssd = cur_ssd;
          }
        }
        cur_ssd = calc_patch_ssd(ref_img, 
                                 track_img,
                                 ref_xy,
                                 patch_stride_ptr + 2 * (jj - dir[level_iter]),
                                 patch_size,
                                 gray_img_line_size,
                                 gray_x_max,
                                 gray_y_max);
        if(cur_ssd < min_ssd){
          *((unsigned long long*)(patch_stride_ptr + 2 * jj)) = 
          *((unsigned long long*)(patch_stride_ptr + 2 * (jj - dir[level_iter])));
          min_ssd = cur_ssd;
        }
        cur_ssd = calc_patch_ssd(ref_img, 
                                 track_img,
                                 ref_xy,
                                 patch_stride_ptr + 2 * jj - dir[level_iter] * patch_stride_line_size,
                                 patch_size,
                                 gray_img_line_size,
                                 gray_x_max,
                                 gray_y_max);
        if(cur_ssd < min_ssd){
          *((unsigned long long*)(patch_stride_ptr + 2 * jj)) = 
          *((unsigned long long*)(patch_stride_ptr + 2 * jj - dir[level_iter] * patch_stride_line_size));
          min_ssd = cur_ssd;
        }
        iterate_dis(ref_img,
                    track_img,
                    gray_img_line_size,
                    ref_grad_xy,
                    ref_grad_xy_line_size,
                    patch_stride_ptr + 2 * jj,
                    jj, ii,
                    stride_grad_xy_integral,
                    stride_grad_xy_int_line_size,
                    hessian_inv,
                    hessian_inv_line_size,
                    internal_iter_num,
                    patch_size,
                    patch_stride_size,
                    gray_x_max,
                    gray_y_max);
        jj += dir[level_iter];
      }
      patch_stride_ptr += dir[level_iter] * patch_stride_line_size;
      flow_out_ptr     += dir[level_iter] * patch_stride_size * flow_out_line_size;
      init_flow_ptr    += dir[level_iter] * patch_stride_size * init_flow_line_size;
      ii               += dir[level_iter];
      jj                = start_j;
    }
  }

  return 0;
}

static int densification(DIS_INSTANCE*  dis,
                         unsigned int   level){
  float*         dense_flow       = (float*)(dis->dense_flow_pyramid.buf[level] +
                                     dis->dense_flow_pyramid.pad * dis->dense_flow_pyramid.line_size[level] +
                                     dis->dense_flow_pyramid.pad * 8);
  float const*   sparse_flow      = (float const*)(dis->patch_stride_flow_pyramid.buf[level] +
                                     dis->patch_stride_flow_pyramid.pad * dis->patch_stride_flow_pyramid.line_size[level] +
                                     dis->patch_stride_flow_pyramid.pad * 8);
  unsigned char const* ref_img    = dis->ref_gray_pyramid.buf[level] + 
                                    dis->ref_gray_pyramid.pad * dis->ref_gray_pyramid.line_size[level] +
                                    dis->ref_gray_pyramid.pad;
  unsigned char const* track_img  = dis->track_gray_pyramid.buf[level] + 
                                    dis->track_gray_pyramid.pad * dis->track_gray_pyramid.line_size[level] +
                                    dis->track_gray_pyramid.pad;
  int    img_line_size    = dis->track_gray_pyramid.line_size[level];
  int    w                = dis->dense_flow_pyramid.width[level];
  int    h                = dis->dense_flow_pyramid.height[level];
  int    dense_line_size  = dis->dense_flow_pyramid.line_size[level] >> 2;
  int    sparse_line_size = dis->patch_stride_flow_pyramid.line_size[level] >> 2;
  int    stride_size      = dis->patch_stride_size;
  int    I = 0, J = 0, is = 0, js = 0, ii = 0, jj = 0;
  int    start_y = 0, start_x = 0, end_y = 0, end_x = 0;
  int    sparse_w = dis->patch_stride_flow_pyramid.width[level];
  int    sparse_h = dis->patch_stride_flow_pyramid.height[level];
  {
    //sparse top-left,  only 1 cross-block
    end_y = h < stride_size ? h : stride_size;
    end_x = w < stride_size ? w : stride_size;
    for(ii = 0 ; ii < end_y ; ii ++){
      for(jj = 0 ; jj < end_x ; jj ++){
        dense_flow[ii * dense_line_size + 2 * jj]     = sparse_flow[0];
        dense_flow[ii * dense_line_size + 2 * jj + 1] = sparse_flow[1];
      }
    }
    //sparse top-right, only 1 cross-block
    start_x = stride_size * sparse_w;
    end_x   = w;
    for(ii = 0 ; ii < end_y ; ii ++){
      for(jj = start_x ; jj < end_x ; jj ++){
        dense_flow[ii * dense_line_size + 2 * jj]     = sparse_flow[2 * (sparse_w - 1)];
        dense_flow[ii * dense_line_size + 2 * jj + 1] = sparse_flow[2 * (sparse_w - 1) + 1];
      }
    }
    //sparse btm-left,  only 1 cross-block
    start_y = stride_size * sparse_h;
    end_y   = h < stride_size ? h : stride_size;
    end_x   = w < stride_size ? w : stride_size;
    for(ii = start_y ; ii < end_y ; ii ++){
      for(jj = 0 ; jj < end_x ; jj ++){
        dense_flow[ii * dense_line_size + 2 * jj]     = sparse_flow[(sparse_h - 1) * sparse_line_size];
        dense_flow[ii * dense_line_size + 2 * jj + 1] = sparse_flow[(sparse_h - 1) * sparse_line_size + 1];
      }
    }

    //sparse btm-right,  only 1 cross-block
    start_x = stride_size * sparse_w;
    end_x   = w;
    for(ii = start_y ; ii < end_y ; ii ++){
      for(jj = start_x ; jj < end_x ; jj ++){
        dense_flow[ii * dense_line_size + 2 * jj]     = sparse_flow[(sparse_h - 1) * sparse_line_size + 2 * (sparse_w - 1)];
        dense_flow[ii * dense_line_size + 2 * jj + 1] = sparse_flow[(sparse_h - 1) * sparse_line_size + 2 * (sparse_w - 1) + 1];
      }
    }
  }

  {
    //sparse top, 2 cross-blocks
    start_y = 0;
    end_y   = h < stride_size ? h : stride_size;
    start_x = stride_size;
    end_x   = stride_size * sparse_w;
    I       = 0;
    for(J = start_x, js = 1 ; J < end_x ; J += stride_size, js ++){
      float flow_x0 = sparse_flow[2 * (js - 1)];
      float flow_y0 = sparse_flow[2 * (js - 1) + 1];
      float flow_x1 = sparse_flow[2 * js];
      float flow_y1 = sparse_flow[2 * js + 1];
      for(ii = 0 ; ii < end_y ; ii ++){
        int     ref_y       = I + ii;
        float   track_y0_f  = ref_y + flow_y0;
        float   track_y1_f  = ref_y + flow_y1;
        int     track_y0_i  = track_y0_f < 0.0f ? (int)(track_y0_f - 1 - 0.5f) : (int)(track_y0_f + 0.5f);
        int     track_y1_i  = track_y1_f < 0.0f ? (int)(track_y1_f - 1 - 0.5f) : (int)(track_y1_f + 0.5f);
        for(jj = 0 ; jj < stride_size ; jj ++){
          int   ref_x       = J + jj;
          float track_x0_f  = ref_x + flow_x0;
          float track_x1_f  = ref_x + flow_x1;
          int   track_x0_i  = track_x0_f < 0.0f ? (int)(track_x0_f - 1 - 0.5f) : (int)(track_x0_f + 0.5f);
          int   track_x1_i  = track_x1_f < 0.0f ? (int)(track_x1_f - 1 - 0.5f) : (int)(track_x1_f + 0.5f);
          float u0          = track_x0_f - track_x0_i;
          float v0          = track_y0_f - track_y0_i;
          float u1          = track_x1_f - track_x1_i;
          float v1          = track_y1_f - track_y1_i;
          float ref_val     = ref_img[ref_y * img_line_size + ref_x];
          float track_val0  = (1.0f - u0) * (1.0f - v0) * track_img[      track_y0_i  * img_line_size + track_x0_i] +
                                      u0  * (1.0f - v0) * track_img[      track_y0_i  * img_line_size + track_x0_i + 1] +
                              (1.0f - u0) *         v0  * track_img[ (track_y0_i + 1) * img_line_size + track_x0_i] +
                                      u0  *         v0  * track_img[ (track_y0_i + 1) * img_line_size + track_x0_i + 1];
          float track_val1  = (1.0f - u1) * (1.0f - v1) * track_img[      track_y1_i  * img_line_size + track_x1_i] +
                                      u1  * (1.0f - v1) * track_img[      track_y1_i  * img_line_size + track_x1_i + 1] +
                              (1.0f - u1) *         v1  * track_img[ (track_y1_i + 1) * img_line_size + track_x1_i] +
                                      u1  *         v1  * track_img[ (track_y1_i + 1) * img_line_size + track_x1_i + 1];
          float coef, sum_coef = 0.0f;
          float sum_x   = 0.0f;
          float sum_y   = 0.0f;
          coef      = 1.0f / MAX_F32(1.0f, fabsf(ref_val - track_val0));
          sum_x    += coef * flow_x0;
          sum_y    += coef * flow_y0;
          sum_coef += coef;
          coef      = 1.0f / MAX_F32(1.0f, fabsf(ref_val - track_val1));
          sum_x    += coef * flow_x1;
          sum_y    += coef * flow_y1;
          sum_coef += coef;
          dense_flow[ref_y * dense_line_size + 2 * ref_x]     = sum_x / sum_coef;
          dense_flow[ref_y * dense_line_size + 2 * ref_x + 1] = sum_y / sum_coef;
        }
      }
    }
    //sparse btm, 2 cross-blocks
    if(h > stride_size){
      start_y = stride_size * sparse_h;
      end_y   = h - start_y;
      is      = sparse_h;
      I       = start_y;
      for(J = start_x, js = 1 ; J < end_x ; J += stride_size, js ++){
        float flow_x0 = sparse_flow[(is - 1) * sparse_line_size + 2 * (js - 1)];
        float flow_y0 = sparse_flow[(is - 1) * sparse_line_size + 2 * (js - 1) + 1];
        float flow_x1 = sparse_flow[(is - 1) * sparse_line_size + 2 * js];
        float flow_y1 = sparse_flow[(is - 1) * sparse_line_size + 2 * js + 1];
        for(ii = 0 ; ii < end_y ; ii ++){
          int   ref_y      = I + ii;
          float track_y0_f = ref_y + flow_y0;
          float track_y1_f = ref_y + flow_y1;
          int   track_y0_i = track_y0_f < 0.0f ? (int)(track_y0_f - 1 - 0.5f) : (int)(track_y0_f + 0.5f);
          int   track_y1_i = track_y1_f < 0.0f ? (int)(track_y1_f - 1 - 0.5f) : (int)(track_y1_f + 0.5f);
          float v0         = track_y0_f - track_y0_i;
          float v1         = track_y1_f - track_y1_i;
          for(jj = 0 ; jj < stride_size ; jj ++){
            int   ref_x       = J + jj;
            float track_x0_f  = ref_x + flow_x0;
            float track_x1_f  = ref_x + flow_x1;
            int   track_x0_i  = track_x0_f < 0.0f ? (int)(track_x0_f - 1 - 0.5f) : (int)(track_x0_f + 0.5f);
            int   track_x1_i  = track_x1_f < 0.0f ? (int)(track_x1_f - 1 - 0.5f) : (int)(track_x1_f + 0.5f);
            float u0          = track_x0_f - track_x0_i;
            float u1          = track_x1_f - track_x1_i;
            float ref_val     = ref_img[ref_y * img_line_size + ref_x];
            float track_val0  = (1.0f - u0) * (1.0f - v0) * track_img[      track_y0_i  * img_line_size + track_x0_i] +
                                        u0  * (1.0f - v0) * track_img[      track_y0_i  * img_line_size + track_x0_i + 1] +
                                (1.0f - u0) *         v0  * track_img[ (track_y0_i + 1) * img_line_size + track_x0_i] +
                                        u0  *         v0  * track_img[ (track_y0_i + 1) * img_line_size + track_x0_i + 1];
            float track_val1  = (1.0f - u1) * (1.0f - v1) * track_img[      track_y1_i  * img_line_size + track_x1_i] +
                                        u1  * (1.0f - v1) * track_img[      track_y1_i  * img_line_size + track_x1_i + 1] +
                                (1.0f - u1) *         v1  * track_img[ (track_y1_i + 1) * img_line_size + track_x1_i] +
                                        u1  *         v1  * track_img[ (track_y1_i + 1) * img_line_size + track_x1_i + 1];
            float coef, sum_coef = 0.0f;
            float sum_x   = 0.0f;
            float sum_y   = 0.0f;
            coef      = 1.0f / MAX_F32(1.0f, fabsf(ref_val - track_val0));
            sum_x    += coef * flow_x0;
            sum_y    += coef * flow_y0;
            sum_coef += coef;
            coef      = 1.0f / MAX_F32(1.0f, fabsf(ref_val - track_val1));
            sum_x    += coef * flow_x1;
            sum_y    += coef * flow_y1;
            sum_coef += coef;
            dense_flow[ref_y * dense_line_size + 2 * ref_x]     = sum_x / sum_coef;
            dense_flow[ref_y * dense_line_size + 2 * ref_x + 1] = sum_y / sum_coef;
          }
        }
      }
    }
    //sparse left, 2 cross-blocks
    if(h > 2 * stride_size){
      start_x = 0;
      end_x   = w < stride_size ? w : stride_size;
      start_y = stride_size;
      end_y   = stride_size * sparse_h;
      J       = 0;
      for(I = start_y, is = 1 ; I < end_y ; I += stride_size, is ++){
        float flow_x0 = sparse_flow[(is - 1) * sparse_line_size];
        float flow_y0 = sparse_flow[(is - 1) * sparse_line_size + 1];
        float flow_x1 = sparse_flow[is       * sparse_line_size];
        float flow_y1 = sparse_flow[is       * sparse_line_size + 1];
        for(ii = 0 ; ii < stride_size ; ii ++){
          int   ref_y      = I + ii;
          float track_y0_f = ref_y + flow_y0;
          float track_y1_f = ref_y + flow_y1;
          int   track_y0_i = track_y0_f < 0.0f ? (int)(track_y0_f - 1 - 0.5f) : (int)(track_y0_f + 0.5f);
          int   track_y1_i = track_y1_f < 0.0f ? (int)(track_y1_f - 1 - 0.5f) : (int)(track_y1_f + 0.5f);
          float v0         = track_y0_f - track_y0_i;
          float v1         = track_y1_f - track_y1_i;
          for(jj = 0 ; jj < end_x ; jj ++){
            int   ref_x       = J + jj;
            float track_x0_f  = ref_x + flow_x0;
            float track_x1_f  = ref_x + flow_x1;
            int   track_x0_i  = track_x0_f < 0.0f ? (int)(track_x0_f - 1 - 0.5f) : (int)(track_x0_f + 0.5f);
            int   track_x1_i  = track_x1_f < 0.0f ? (int)(track_x1_f - 1 - 0.5f) : (int)(track_x1_f + 0.5f);
            float u0          = track_x0_f - track_x0_i;
            float u1          = track_x1_f - track_x1_i;
            float ref_val     = ref_img[ref_y * img_line_size + ref_x];
            float track_val0  = (1.0f - u0) * (1.0f - v0) * track_img[      track_y0_i  * img_line_size + track_x0_i] +
                                        u0  * (1.0f - v0) * track_img[      track_y0_i  * img_line_size + track_x0_i + 1] +
                                (1.0f - u0) *         v0  * track_img[ (track_y0_i + 1) * img_line_size + track_x0_i] +
                                        u0  *         v0  * track_img[ (track_y0_i + 1) * img_line_size + track_x0_i + 1];
            float track_val1  = (1.0f - u1) * (1.0f - v1) * track_img[      track_y1_i  * img_line_size + track_x1_i] +
                                        u1  * (1.0f - v1) * track_img[      track_y1_i  * img_line_size + track_x1_i + 1] +
                                (1.0f - u1) *         v1  * track_img[ (track_y1_i + 1) * img_line_size + track_x1_i] +
                                        u1  *         v1  * track_img[ (track_y1_i + 1) * img_line_size + track_x1_i + 1];
            float coef, sum_coef = 0.0f;
            float sum_x   = 0.0f;
            float sum_y   = 0.0f;
            coef      = 1.0f / MAX_F32(1.0f, fabsf(ref_val - track_val0));
            sum_x    += coef * flow_x0;
            sum_y    += coef * flow_y0;
            sum_coef += coef;
            coef      = 1.0f / MAX_F32(1.0f, fabsf(ref_val - track_val1));
            sum_x    += coef * flow_x1;
            sum_y    += coef * flow_y1;
            sum_coef += coef;
            dense_flow[ref_y * dense_line_size + 2 * ref_x]     = sum_x / sum_coef;
            dense_flow[ref_y * dense_line_size + 2 * ref_x + 1] = sum_y / sum_coef;
          }
        }
      }
      //sparse right, 2 cross-blocks
      start_x = stride_size * sparse_w;
      end_x   = w;
      int stop_jj = end_x - start_x;
      start_y = stride_size;
      end_y   = stride_size * sparse_h;
      J       = start_x;
      for(I = start_y, is = 1 ; I < end_y ; I += stride_size, is ++){
        float flow_x0 = sparse_flow[(is - 1) * sparse_line_size];
        float flow_y0 = sparse_flow[(is - 1) * sparse_line_size + 1];
        float flow_x1 = sparse_flow[is       * sparse_line_size];
        float flow_y1 = sparse_flow[is       * sparse_line_size + 1];
        for(ii = 0 ; ii < stride_size ; ii ++){
          int   ref_y      = I + ii;
          float track_y0_f = ref_y + flow_y0;
          float track_y1_f = ref_y + flow_y1;
          int   track_y0_i = track_y0_f < 0.0f ? (int)(track_y0_f - 1 - 0.5f) : (int)(track_y0_f + 0.5f);
          int   track_y1_i = track_y1_f < 0.0f ? (int)(track_y1_f - 1 - 0.5f) : (int)(track_y1_f + 0.5f);
          float v0         = track_y0_f - track_y0_i;
          float v1         = track_y1_f - track_y1_i;
          for(jj = 0 ; jj < stop_jj ; jj ++){
            int   ref_x       = J + jj;
            float track_x0_f  = ref_x + flow_x0;
            float track_x1_f  = ref_x + flow_x1;
            int   track_x0_i  = track_x0_f < 0.0f ? (int)(track_x0_f - 1 - 0.5f) : (int)(track_x0_f + 0.5f);
            int   track_x1_i  = track_x1_f < 0.0f ? (int)(track_x1_f - 1 - 0.5f) : (int)(track_x1_f + 0.5f);
            float u0          = track_x0_f - track_x0_i;
            float u1          = track_x1_f - track_x1_i;
            float ref_val     = ref_img[ref_y * img_line_size + ref_x];
            float track_val0  = (1.0f - u0) * (1.0f - v0) * track_img[      track_y0_i  * img_line_size + track_x0_i] +
                                        u0  * (1.0f - v0) * track_img[      track_y0_i  * img_line_size + track_x0_i + 1] +
                                (1.0f - u0) *         v0  * track_img[ (track_y0_i + 1) * img_line_size + track_x0_i] +
                                        u0  *         v0  * track_img[ (track_y0_i + 1) * img_line_size + track_x0_i + 1];
            float track_val1  = (1.0f - u1) * (1.0f - v1) * track_img[      track_y1_i  * img_line_size + track_x1_i] +
                                        u1  * (1.0f - v1) * track_img[      track_y1_i  * img_line_size + track_x1_i + 1] +
                                (1.0f - u1) *         v1  * track_img[ (track_y1_i + 1) * img_line_size + track_x1_i] +
                                        u1  *         v1  * track_img[ (track_y1_i + 1) * img_line_size + track_x1_i + 1];
            float coef, sum_coef = 0.0f;
            float sum_x   = 0.0f;
            float sum_y   = 0.0f;
            coef      = 1.0f / MAX_F32(1.0f, fabsf(ref_val - track_val0));
            sum_x    += coef * flow_x0;
            sum_y    += coef * flow_y0;
            sum_coef += coef;
            coef      = 1.0f / MAX_F32(1.0f, fabsf(ref_val - track_val1));
            sum_x    += coef * flow_x1;
            sum_y    += coef * flow_y1;
            sum_coef += coef;
            dense_flow[ref_y * dense_line_size + 2 * ref_x]     = sum_x / sum_coef;
            dense_flow[ref_y * dense_line_size + 2 * ref_x + 1] = sum_y / sum_coef;
          }
        }
      }
    }
  }

  //center, 4 cross-blocks
  start_x = stride_size;
  start_y = stride_size;
  end_x   = stride_size * dis->patch_stride_flow_pyramid.width[level];
  end_y   = stride_size * dis->patch_stride_flow_pyramid.height[level];
  for(I = start_y, is = 1 ; I < end_y ; I += stride_size, is ++){
    for(J = start_x, js = 1 ; J < end_x ; J += stride_size, js ++){
      float flow_x0 = sparse_flow[(is - 1) * sparse_line_size + 2 * (js - 1)];
      float flow_y0 = sparse_flow[(is - 1) * sparse_line_size + 2 * (js - 1) + 1];
      float flow_x1 = sparse_flow[(is - 1) * sparse_line_size + 2 * js];
      float flow_y1 = sparse_flow[(is - 1) * sparse_line_size + 2 * js + 1];
      float flow_x2 = sparse_flow[is       * sparse_line_size + 2 * (js - 1)];
      float flow_y2 = sparse_flow[is       * sparse_line_size + 2 * (js - 1) + 1];
      float flow_x3 = sparse_flow[is       * sparse_line_size + 2 * js];
      float flow_y3 = sparse_flow[is       * sparse_line_size + 2 * js + 1];
      for(ii = 0 ; ii < stride_size ; ii ++){
        int   ref_y       = I + ii;
        float track_y0_f  = ref_y + flow_y0;
        float track_y1_f  = ref_y + flow_y1;
        float track_y2_f  = ref_y + flow_y2;
        float track_y3_f  = ref_y + flow_y3;
        int   track_y0_i  = track_y0_f < 0.0f ? (int)(track_y0_f - 1 - 0.5f) : (int)(track_y0_f + 0.5f);
        int   track_y1_i  = track_y1_f < 0.0f ? (int)(track_y1_f - 1 - 0.5f) : (int)(track_y1_f + 0.5f);
        int   track_y2_i  = track_y2_f < 0.0f ? (int)(track_y2_f - 1 - 0.5f) : (int)(track_y2_f + 0.5f);
        int   track_y3_i  = track_y3_f < 0.0f ? (int)(track_y3_f - 1 - 0.5f) : (int)(track_y3_f + 0.5f);
        float v0          = track_y0_f - track_y0_i;
        float v1          = track_y1_f - track_y1_i;
        float v2          = track_y2_f - track_y2_i;
        float v3          = track_y3_f - track_y3_i;
        for(jj = 0 ; jj < stride_size ; jj ++){
          int   ref_x       = J + jj;
          float track_x0_f  = ref_x + flow_x0;
          float track_x1_f  = ref_x + flow_x1;
          float track_x2_f  = ref_x + flow_x2;
          float track_x3_f  = ref_x + flow_x3;
          int   track_x0_i  = track_x0_f < 0.0f ? (int)(track_x0_f - 1 - 0.5f) : (int)(track_x0_f + 0.5f);
          int   track_x1_i  = track_x1_f < 0.0f ? (int)(track_x1_f - 1 - 0.5f) : (int)(track_x1_f + 0.5f);
          int   track_x2_i  = track_x2_f < 0.0f ? (int)(track_x2_f - 1 - 0.5f) : (int)(track_x2_f + 0.5f);
          int   track_x3_i  = track_x3_f < 0.0f ? (int)(track_x3_f - 1 - 0.5f) : (int)(track_x3_f + 0.5f);
          float u0          = track_x0_f - track_x0_i;
          float u1          = track_x1_f - track_x1_i;
          float u2          = track_x2_f - track_x2_i;
          float u3          = track_x3_f - track_x3_i;
          float ref_val     = ref_img[ref_y * img_line_size + ref_x];
          float track_val0  = (1.0f - u0) * (1.0f - v0) * track_img[      track_y0_i  * img_line_size + track_x0_i] +
                                      u0  * (1.0f - v0) * track_img[      track_y0_i  * img_line_size + track_x0_i + 1] +
                              (1.0f - u0) *         v0  * track_img[ (track_y0_i + 1) * img_line_size + track_x0_i] +
                                      u0  *         v0  * track_img[ (track_y0_i + 1) * img_line_size + track_x0_i + 1];
          float track_val1  = (1.0f - u1) * (1.0f - v1) * track_img[      track_y1_i  * img_line_size + track_x1_i] +
                                      u1  * (1.0f - v1) * track_img[      track_y1_i  * img_line_size + track_x1_i + 1] +
                              (1.0f - u1) *         v1  * track_img[ (track_y1_i + 1) * img_line_size + track_x1_i] +
                                      u1  *         v1  * track_img[ (track_y1_i + 1) * img_line_size + track_x1_i + 1];
          float track_val2  = (1.0f - u2) * (1.0f - v2) * track_img[      track_y2_i  * img_line_size + track_x2_i] +
                                      u2  * (1.0f - v2) * track_img[      track_y2_i  * img_line_size + track_x2_i + 1] +
                              (1.0f - u2) *         v2  * track_img[ (track_y2_i + 1) * img_line_size + track_x2_i] +
                                      u2  *         v2  * track_img[ (track_y2_i + 1) * img_line_size + track_x2_i + 1];
          float track_val3  = (1.0f - u3) * (1.0f - v3) * track_img[      track_y3_i  * img_line_size + track_x3_i] +
                                      u3  * (1.0f - v3) * track_img[      track_y3_i  * img_line_size + track_x3_i + 1] +
                              (1.0f - u3) *         v3  * track_img[ (track_y3_i + 1) * img_line_size + track_x3_i] +
                                      u3  *         v3  * track_img[ (track_y3_i + 1) * img_line_size + track_x3_i + 1];
          float coef, sum_coef = 0.0f;
          float sum_x   = 0.0f;
          float sum_y   = 0.0f;
          coef      = 1.0f / MAX_F32(1.0f, fabsf(ref_val - track_val0));
          sum_x    += coef * flow_x0;
          sum_y    += coef * flow_y0;
          sum_coef += coef;
          coef      = 1.0f / MAX_F32(1.0f, fabsf(ref_val - track_val1));
          sum_x    += coef * flow_x1;
          sum_y    += coef * flow_y1;
          sum_coef += coef;
          coef      = 1.0f / MAX_F32(1.0f, fabsf(ref_val - track_val2));
          sum_x    += coef * flow_x2;
          sum_y    += coef * flow_y2;
          sum_coef += coef;
          coef      = 1.0f / MAX_F32(1.0f, fabsf(ref_val - track_val3));
          sum_x    += coef * flow_x3;
          sum_y    += coef * flow_y3;
          sum_coef += coef;
          dense_flow[ref_y * dense_line_size + 2 * ref_x]     = sum_x / sum_coef;
          dense_flow[ref_y * dense_line_size + 2 * ref_x + 1] = sum_y / sum_coef;
        }
      }
    }
  }
  return 0;
}

static int upsample_flow_to_next_level(DIS_INSTANCE*  dis,
                                       unsigned int   level){
  float const*    cur_flow  = (float const*)(dis->dense_flow_pyramid.buf[level] +
                  dis->dense_flow_pyramid.pad * dis->dense_flow_pyramid.line_size[level] +
                                             8 * dis->dense_flow_pyramid.pad);
  float*          next_flow = (float*)(dis->dense_flow_pyramid.buf[level - 1] +
                  dis->dense_flow_pyramid.pad * dis->dense_flow_pyramid.line_size[level - 1] +
                                             8 * dis->dense_flow_pyramid.pad);
  int             cur_w     = dis->dense_flow_pyramid.width[level];
  int             cur_h     = dis->dense_flow_pyramid.height[level];
  int             next_w    = dis->dense_flow_pyramid.width[level - 1];
  int             next_h    = dis->dense_flow_pyramid.height[level - 1];
  int             cur_line_size  = dis->dense_flow_pyramid.line_size[level] >> 2;
  int             next_line_size = dis->dense_flow_pyramid.line_size[level - 1] >> 2;
  float           ratio_x    = (float)cur_w / (float)next_w;
  float           ratio_y    = (float)cur_h / (float)next_h;
  int             i          = 0;
  int             j          = 0;
  float*          next_ptr0  = next_flow;
  int             max_x      = ((int)((cur_w - 1) / ratio_x + 0.5f));
  for(i = 0 ; i < next_h ; i ++){
    float        cur_y_pos_f = i * ratio_y;
    int          cur_y_pos   = (int)(cur_y_pos_f);
    float        v           = cur_y_pos_f - cur_y_pos;
    float const* cur_ptr0    = cur_flow + cur_y_pos * cur_line_size;
    float const* cur_ptr1    = cur_ptr0 + (cur_y_pos > (cur_h - 1) ? 0 : cur_line_size);
    float*       next_ptr1   = next_ptr0;
    for(j = 0 ; j < max_x ; j ++){
      float  cur_x_pos_f = j * ratio_y;
      int    cur_x_pos   = (int)(cur_x_pos_f);
      float  u           = cur_x_pos_f - cur_x_pos;
      float  p00         = cur_ptr0[2 * cur_x_pos];
      float  p01         = cur_ptr0[2 * cur_x_pos + 1];
      float  p10         = cur_ptr0[2 * cur_x_pos + 2];
      float  p11         = cur_ptr0[2 * cur_x_pos + 3];
      float  p20         = cur_ptr1[2 * cur_x_pos];
      float  p21         = cur_ptr1[2 * cur_x_pos + 1];
      float  p30         = cur_ptr1[2 * cur_x_pos + 2];
      float  p31         = cur_ptr1[2 * cur_x_pos + 3];
      float  res0        = (1.0f - u) * (1.0f - v) * p00 +
                                   u  * (1.0f - v) * p10 +
                           (1.0f - u) *         v  * p20 +
                                   u  *         v  * p30 ;
      float  res1        = (1.0f - u) * (1.0f - v) * p01 +
                                   u  * (1.0f - v) * p11 +
                           (1.0f - u) *         v  * p21 +
                                   u  *         v  * p31 ;
      next_ptr1[0]  = res0;
      next_ptr1[1]  = res1;
      next_ptr1    += 2;
    }
    for(; j < next_w ; j ++){
      float  cur_x_pos_f = j * ratio_y;
      int    cur_x_pos   = (int)(cur_x_pos_f);
      float  u           = cur_x_pos_f - cur_x_pos;
      float  p00         = cur_ptr0[2 * cur_x_pos];
      float  p01         = cur_ptr0[2 * cur_x_pos + 1];
      float  p10         = p00;
      float  p11         = p01;
      float  p20         = cur_ptr1[2 * cur_x_pos];
      float  p21         = cur_ptr1[2 * cur_x_pos + 1];
      float  p30         = p20;
      float  p31         = p21;
      float  res0        = (1.0f - u) * (1.0f - v) * p00 +
                                   u  * (1.0f - v) * p10 +
                           (1.0f - u) *         v  * p20 +
                                   u  *         v  * p30 ;
      float  res1        = (1.0f - u) * (1.0f - v) * p01 +
                                   u  * (1.0f - v) * p11 +
                           (1.0f - u) *         v  * p21 +
                                   u  *         v  * p31 ;
      next_ptr1[0]  = res0;
      next_ptr1[1]  = res1;
      next_ptr1    += 2;
    }
    next_ptr0 += next_line_size;
  }
}

int     dis_dof(void*                  dis_instance, 
                unsigned char const*   ref_img,
                unsigned int           ref_img_line_bytes,
                unsigned char const*   track_img,
                unsigned int           track_img_line_bytes,
                DOF_MAP*               dof_map){
  unsigned int   i   = 0;
  DIS_INSTANCE*  dis = (DIS_INSTANCE*)dis_instance;
  cv_resize_image_uc(ref_img,
                     dis->ref_btm_resize_img,
                     dis->src_width,
                     dis->src_height,
                     ref_img_line_bytes,
                     dis->ref_gray_pyramid.width[0],
                     dis->ref_gray_pyramid.height[0],
                     dis->ref_gray_pyramid.line_size[0],
                     1,
                     0);
  cv_resize_image_uc(track_img,
                     dis->track_btm_resize_img,
                     dis->src_width,
                     dis->src_height,
                     ref_img_line_bytes,
                     dis->ref_gray_pyramid.width[0],
                     dis->ref_gray_pyramid.height[0],
                     dis->ref_gray_pyramid.line_size[0],
                     1,
                     0);
  build_gray_pyramid(dis->ref_btm_resize_img,
                     dis->ref_gray_pyramid.line_size[0],
                     1,
                     &(dis->ref_gray_pyramid));
  build_gray_pyramid(dis->track_btm_resize_img,
                     dis->track_gray_pyramid.line_size[0],
                     1,
                     &(dis->track_gray_pyramid));
  {
    unsigned int pyr_level = dis->ref_gray_pyramid.level;
    char output_file_prefix[256] ;
    char output_file_name[256] ;
    sprintf(output_file_prefix, "%s/ref_pyr", DEBUG_IMG_PREFIX);
    for(i = 0 ; i < pyr_level ; i ++){
      sprintf(output_file_name, "%s/%d.bmp", output_file_prefix, i);
      SaveBMP(output_file_name,
              dis->ref_gray_pyramid.buf[i], 
              dis->ref_gray_pyramid.width[i]  + 2 * dis->ref_gray_pyramid.pad, 
              dis->ref_gray_pyramid.height[i] + 2 * dis->ref_gray_pyramid.pad, 
              dis->ref_gray_pyramid.line_size[i], 
              8,
              100);
    }
    sprintf(output_file_prefix, "%s/track_pyr", DEBUG_IMG_PREFIX);
    for(i = 0 ; i < pyr_level ; i ++){
      sprintf(output_file_name, "%s/%d.bmp", output_file_prefix, i);
      SaveBMP(output_file_name,
              dis->ref_gray_pyramid.buf[i], 
              dis->ref_gray_pyramid.width[i]  + 2 * dis->ref_gray_pyramid.pad, 
              dis->ref_gray_pyramid.height[i] + 2 * dis->ref_gray_pyramid.pad, 
              dis->ref_gray_pyramid.line_size[i], 
              8,
              100);
    }
  }
  grad_xy_pyramid(&(dis->ref_gray_pyramid),
                  &(dis->ref_grad_xy_pyramid));
  build_grad_xy_integral_pyr(&(dis->ref_grad_xy_pyramid),
                             dis->patch_size,
                             dis->patch_stride_size,
                             dis->init_flow_pyramid.mem,
                             dis->init_flow_pyramid.total_buf_size,
                             &(dis->grad_xy_stride_integral_pyramid),
                             &(dis->grad_xy_integral_pyramid));
  build_hessian_inv_pyr(&(dis->grad_xy_integral_pyramid),
                        dis->patch_size,
                        dis->patch_stride_size,
                        &(dis->hessian_inv_pyramid));
  memset(dis->patch_stride_flow_pyramid.mem,
         0,
         dis->patch_stride_flow_pyramid.total_buf_size);
  memset(dis->init_flow_pyramid.mem,
         0,
         dis->init_flow_pyramid.total_buf_size);
  memset(dis->dense_flow_pyramid.mem,
         0,
         dis->dense_flow_pyramid.total_buf_size);
  int          cur_level              = 0;
  int          top_level              = dis->ref_gray_pyramid.level - 1;
  unsigned int kInvSearchInternalIter = 8;
  for(cur_level = top_level; cur_level >= 0 ; cur_level --){
    patch_inv_search(dis, cur_level, kInvSearchInternalIter);
    densification(dis, cur_level);
    if(cur_level > 0){
      upsample_flow_to_next_level(dis, cur_level);
    }
  }

  return 0;
}
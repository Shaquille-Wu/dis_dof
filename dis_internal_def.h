#ifndef __DIS_INTERNAL_DEF_H__
#define __DIS_INTERNAL_DEF_H__

#include <dis_def.h>
#include <dis_types.h>

#define DIS_PATCH_SIZE     DIS_DOF_WND_SIZE
#define DIS_PATCH_STRIDE   (DIS_PATCH_SIZE >> 1)

typedef struct tag_dis_instance{
  unsigned int      pyr_level;
  unsigned int      src_width;
  unsigned int      src_height;
  unsigned char*    ref_btm_resize_img;
  unsigned char*    track_btm_resize_img;
  DIS_PYRAMID       ref_gray_pyramid;
  DIS_PYRAMID       track_gray_pyramid;
  DIS_PYRAMID       patch_stride_flow_pyramid;
  DIS_PYRAMID       ref_grad_xy_pyramid;
  DIS_PYRAMID       grad_xy_stride_integral_pyramid;
  DIS_PYRAMID       grad_xy_integral_pyramid;
  DIS_PYRAMID       hessian_inv_pyramid;
  DIS_PYRAMID       init_flow_pyramid;
  DIS_PYRAMID       dense_flow_pyramid;
  unsigned int      patch_stride_blk_w[DIS_PYRAMID_MAX];
  unsigned int      patch_stride_blk_h[DIS_PYRAMID_MAX];
  unsigned int      patch_size;
  unsigned int      patch_stride_size;
  unsigned int      mem_size;
}DIS_INSTANCE, *PDIS_INSTANCE;

#endif  //__DIS_INTERNAL_DEF_H__
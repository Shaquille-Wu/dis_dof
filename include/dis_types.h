#ifndef  __DIS_TYPES_H__
#define  __DIS_TYPES_H__

#define DIS_PYRAMID_MAX         8
#define DIS_PYRAMID_MIN_SIZE    4

typedef enum tag_pyramid_pixel_type{
  PYRAMID_PIXEL_S8 = 0,
  PYRAMID_PIXEL_S16,
  PYRAMID_PIXEL_S32,
  PYRAMID_PIXEL_S64,
  PYRAMID_PIXEL_U8,
  PYRAMID_PIXEL_U16,
  PYRAMID_PIXEL_U32,
  PYRAMID_PIXEL_U64,
  PYRAMID_PIXEL_F32,
  PYRAMID_PIXEL_TYPE_SUM
}PYRAMID_PIXEL_TYPE;

typedef struct tag_dis_pyramid{
  unsigned int        raw_width;
  unsigned int        raw_height;
  unsigned char*      mem;
  unsigned char*      buf[DIS_PYRAMID_MAX];
  unsigned int        width[DIS_PYRAMID_MAX];
  unsigned int        height[DIS_PYRAMID_MAX];
  unsigned int        line_size[DIS_PYRAMID_MAX];
  unsigned int        pixel_type;
  unsigned int        level;
  unsigned int        pad;
  unsigned int        total_buf_size;
}DIS_PYRAMID, *PDIS_PYRAMID;

typedef struct tag_dof_map{
  unsigned int        width;
  unsigned int        height;
  unsigned int        line_size;
  float*              vector;   //4 floats in each vector: x, y, scroe and reserve
}DOF_MAP, *PDOF_MAP;

#endif   //__DIS_DOF_TYPES_H__
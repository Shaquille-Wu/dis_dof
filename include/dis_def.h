#ifndef  __DIS_DEF_H__
#define  __DIS_DEF_H__

#define DIS_TRACK_MAX           6

#define DIS_DOF_WND_SIZE        8
#define DIS_DOF_PAD_SIZE        DIS_DOF_WND_SIZE

#define IMG_LINE_ALIGN_BITS     4
#define IMG_LINE_ALIGN_SIZE     (1 << 4)
#define IMG_LINE_ALIGNED(x)     (((x) + IMG_LINE_ALIGN_SIZE - 1) >> IMG_LINE_ALIGN_BITS) << IMG_LINE_ALIGN_BITS

#endif   //__SFM_DEF_H__
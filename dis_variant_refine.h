#ifndef __DIS_VARIANT_REFINE_H__
#define __DIS_VARIANT_REFINE_H__

#include "dis_internal_def.h"

#ifdef __cplusplus
extern "C"{
#endif

int     variant_refine(DIS_INSTANCE*   dis, 
                       unsigned int    cur_level, 
                       unsigned int    fixed_point_iter,
                       unsigned int    sor_iter);

#ifdef __cplusplus
}
#endif

#endif  //__DIS_IMPLEMENT_H__
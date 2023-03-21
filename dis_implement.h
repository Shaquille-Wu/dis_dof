#ifndef __DIS_IMPLEMENT_H__
#define __DIS_IMPLEMENT_H__

#include <dis_def.h>
#include <dis_types.h>

void*   create_dis_instance(unsigned int  img_width, 
                            unsigned int  img_height,
                            float         tv_alpha,
                            float         tv_gamma,
                            float         tv_delta);

int     destroy_dis_instance(void* dis_instance);

int     dis_dof(void*                  dis_instance, 
                unsigned char const*   ref_img,
                unsigned int           ref_img_line_bytes,
                unsigned char const*   track_img,
                unsigned int           track_img_line_bytes,
                DOF_MAP*               dof_map);
            
#endif  //__DIS_IMPLEMENT_H__
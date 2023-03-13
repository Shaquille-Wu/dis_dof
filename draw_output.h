#ifndef __DRAW_OUTPUT_H__
#define __DRAW_OUTPUT_H__

#include <dis_types.h>

int draw_dof_match(char const*           image_file_name,
                   unsigned char const*  ref_img,
                   unsigned int          width,
                   unsigned int          height,
                   unsigned int          ref_line_size,
                   unsigned char const*  track_img,
                   unsigned int          track_line_size,
                   DOF_MAP const const*  dof_map,
                   int                   match_dir);


#endif
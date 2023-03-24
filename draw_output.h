#ifndef __DRAW_OUTPUT_H__
#define __DRAW_OUTPUT_H__

#include <dis_types.h>

int draw_dof_match(char const*           match_file_name,
                   char const*           flow_file_name,
                   unsigned char const*  ref_img,
                   unsigned int          width,
                   unsigned int          height,
                   unsigned int          ref_line_size,
                   unsigned char const*  track_img,
                   unsigned int          track_line_size,
                   DOF_MAP const const*  dof_map,
                   int                   match_dir,
                   float                 flow_color_scale);

int draw_flow(char const*           flow_file_name,
              unsigned int          width,
              unsigned int          height,
              float const*          flow,
              int                   flow_line_size,
              float                 scale);

#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cv_preproc.h>
#include <align_mem.h>
#include <bmp.h>
#include "dis_implement.h"
#include "draw_output.h"

#define INPUT_IMG_PREFIX    "/home/icework/adas_alg_ref/dis_dof_ref/dis_dof/data/input"
#define INPUT_IMG_CNT       6
#define DEBUG_IMG_PREFIX    "/home/icework/adas_alg_ref/dis_dof_ref/dis_dof/data/output"

int main(int argc, char *argv[]){
  int  i = 0;
  unsigned char*  src_img_data[INPUT_IMG_CNT] = {
    NULL, NULL, NULL, NULL, NULL, NULL
  };
  unsigned char*  src_gray_img_data[INPUT_IMG_CNT] = {
    NULL, NULL, NULL, NULL, NULL, NULL
  };
  char img_file[256]      = { 0 };
  int  src_width          = 0;
  int  src_height         = 0;
  int  src_line_size      = 0;
  int  src_depth          = 0;
  int  src_gray_width     = 0;
  int  src_gray_height    = 0;
  int  src_gray_line_size = 0;
  int  src_gray_depth     = 0;
  DOF_MAP      dof_map;
  memset(&dof_map, 0, sizeof(DOF_MAP));

  for(i = 0 ; i < INPUT_IMG_CNT ; i ++){
    sprintf(img_file, "%s/%010d.bmp", INPUT_IMG_PREFIX, i);
    LoadBMP(img_file, src_img_data + i, &src_width, &src_height, &src_line_size, &src_depth, 1);
    src_gray_width        = src_width;
    src_gray_height       = src_height;
    src_gray_line_size    = IMG_LINE_ALIGNED(src_width);
    src_gray_depth        = 8;
    src_gray_img_data[i]  = (unsigned char*)alloc_mem_align(src_gray_line_size * src_gray_height);
    cv_image_togray_uc(src_img_data[i],
                       src_gray_img_data[i],
                       src_width,
                       src_height,
                       src_line_size,
                       src_gray_line_size,
                       src_depth >> 3,   //just 3 or 4
                       0);
  }

  dof_map.line_size = src_gray_width * sizeof(float) * 2;
  dof_map.width     = src_gray_width;
  dof_map.height    = src_gray_height;
  dof_map.vector    = (float*)alloc_mem_align(dof_map.line_size * src_gray_height);
  memset(dof_map.vector, 0, dof_map.line_size * src_gray_height);

  void* dis = create_dis_instance(src_gray_width, 
                                  src_gray_height,
                                  20.0f, 10.0f, 5.0f);

  int   res = dis_dof(dis, 
                      src_gray_img_data[0], 
                      src_gray_line_size, 
                      src_gray_img_data[1], 
                      src_gray_line_size, 
                      &dof_map);

  draw_dof_match("/home/icework/adas_alg_ref/dis_dof_ref/dis_dof/data/output/match.bmp",
                 "/home/icework/adas_alg_ref/dis_dof_ref/dis_dof/data/output/flow.bmp",
                 src_gray_img_data[0],
                 src_gray_width,
                 src_gray_height,
                 src_gray_line_size,
                 src_gray_img_data[1],
                 src_gray_line_size,
                 &dof_map,
                 1,
                 5.0f);

  destroy_dis_instance(dis);
  free_mem_align(dof_map.vector);

  for(i = 0 ; i < INPUT_IMG_CNT ; i ++){
    if(NULL != src_img_data[i]){
      ReleaseBMPData(src_img_data[i]);
      free_mem_align(src_gray_img_data[i]);
    }
  }

  return 0;
}
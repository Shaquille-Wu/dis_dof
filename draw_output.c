#include <dis_def.h>
#include "draw_output.h"
#include <stdlib.h>
#include <bmp.h>
#include <align_mem.h>
#include <string.h>
#include <math.h>

int draw_line_rgb32(unsigned int*  match_img,
                    int            width,
                    int            height,
                    int            line_size,
                    int const*     start,
                    int const*     stop,
                    unsigned int   color,
                    int            radius){
  int delta_x     = stop[0] - start[0];
  int delta_y     = stop[1] - start[1];
  int delta_x_abs = abs(delta_x);
  int delta_y_abs = abs(delta_y);
  int i           = 0;
  int j           = 0;
  int k           = 0;
  if(0 == delta_x ||
     0 == delta_y){
    if(0 == delta_x && 0 == delta_y){
      if(start[0] >= 0 && start[0] < width &&
         start[1] >= 0 && start[1] < height){
        for(i = start[1] - radius ; i < start[1] + radius ; i ++){
          if(i < 0 || i >= height){
            continue;
          }
          for(j = start[0] - radius ; j < start[0] + radius ; j ++){
            if(j < 0 || j >= width){
              continue;
            }
            match_img[i * line_size + j] = color;
          }
        }
      }
    }else if(0 == delta_x){
      for(i = start[1] - radius ; i < stop[1] + radius ; i ++){
        if(i < 0 || i >= height){
          continue;
        }
        for(j = start[0] - radius ; j < start[0] + radius ; j ++){
          if(j < 0 || j >= width){
            continue;
          }
          match_img[i * line_size + j] = color;
        }
      }
    }else if(0 == delta_y){
      for(i = start[1] - radius ; i < start[1] + radius ; i ++){
        if(i < 0 || i >= height){
          continue;
        }
        for(j = start[0] - radius ; j < stop[1] + radius ; j ++){
          if(j < 0 || j >= width){
            continue;
          }
          match_img[i * line_size + j] = color;
        }
      }
    }
  }else{
    int   x_dir       = delta_x > 0 ? 1 : -1;
    int   y_dir       = delta_y > 0 ? 1 : -1;
    if(delta_x_abs > delta_y_abs){
      float y_2_x_ratio = (float)(delta_y_abs) / (float)(delta_x_abs);
      for(i = 0 ; i < delta_x_abs ; i ++){
        int cur_x = start[0] + x_dir * i;
        int cur_y = start[1] + (y_2_x_ratio * (y_dir * i));
        for(j = cur_y - radius ; j < cur_y + radius ; j ++){
          if(j < 0 || j >= height){
            continue;
          }
          for(k = cur_x - radius ; k < cur_x + radius ; k ++){
            if(k < 0 || k >= width){
              continue;
            }
            match_img[j * line_size + k] = color;
          }
        }
      }
    }else{
      float x_2_y_ratio = (float)(delta_x_abs) / (float)(delta_y_abs);
      for(i = 0 ; i < delta_y_abs ; i ++){
        int cur_x = start[0] + (x_2_y_ratio * (x_dir * i));
        int cur_y = start[1] + y_dir * i;
        for(j = cur_y - radius ; j < cur_y + radius ; j ++){
          if(j < 0 || j >= height){
            continue;
          }
          for(k = cur_x - radius ; k < cur_x + radius ; k ++){
            if(k < 0 || k >= width){
              continue;
            }
            match_img[j * line_size + k] = color;
          }
        }
      }
    }
  }
  return 0;
}

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
                   float                 flow_color_scale){
  unsigned int    match_width     = 2 * width;
  unsigned int    match_height    = height;
  unsigned int    i = 0 , j = 0, k = 0 ;
  if(0 != match_dir){
    match_width  = width;
    match_height = 2 * height;
  }
  unsigned int    match_line_size = IMG_LINE_ALIGNED(4 * match_width);
  unsigned int    flow_line_size  = IMG_LINE_ALIGNED(4 * width);
  unsigned char*  match_image     = (unsigned char*)alloc_mem_align(match_line_size * match_height);
  unsigned char*  flow_image      = (unsigned char*)alloc_mem_align(flow_line_size * height);
  memset(match_image, 0, match_line_size * match_height);
  memset(flow_image,  0, flow_line_size  * height);
  static const unsigned int  kColors[] = {
    0x000000FF,
    0x0000FF00,
    0x00FF0000,
    0x0000FFFF,
    0x00FF00FF,
    0x00FFFF00
  };
  static const unsigned int  kColorCnt = sizeof(kColors) / sizeof(kColors[0]);
  float const*               flow_vec  = dof_map->vector;
  if(0 == match_dir){
    for(i = 0 ; i < height ; i ++){
      for(j = 0 ; j < width ; j ++){
        match_image[i * match_line_size + 4 * j]     = ref_img[i * ref_line_size + j];
        match_image[i * match_line_size + 4 * j + 1] = ref_img[i * ref_line_size + j];
        match_image[i * match_line_size + 4 * j + 2] = ref_img[i * ref_line_size + j];
        match_image[i * match_line_size + 4 * width + 4 * j]     = track_img[i * ref_line_size + j];
        match_image[i * match_line_size + 4 * width + 4 * j + 1] = track_img[i * ref_line_size + j];
        match_image[i * match_line_size + 4 * width + 4 * j + 2] = track_img[i * ref_line_size + j];
      }
    }
  }else{
    for(i = 0 ; i < height ; i ++){
      for(j = 0 ; j < width ; j ++){
        match_image[i * match_line_size + 4 * j]     = ref_img[i * ref_line_size + j];
        match_image[i * match_line_size + 4 * j + 1] = ref_img[i * ref_line_size + j];
        match_image[i * match_line_size + 4 * j + 2] = ref_img[i * ref_line_size + j];
        match_image[(i + height) * match_line_size + 4 * j]     = track_img[i * ref_line_size + j];
        match_image[(i + height) * match_line_size + 4 * j + 1] = track_img[i * ref_line_size + j];
        match_image[(i + height) * match_line_size + 4 * j + 2] = track_img[i * ref_line_size + j];
      }
    }
  }

  for(i = 0 ; i < height ; i ++){
    for(j = 0 ; j < width ; j ++){
      float  flow_x     = flow_color_scale * flow_vec[i * (dof_map->line_size >> 2) + 2 * j];
      float  flow_y     = flow_color_scale * flow_vec[i * (dof_map->line_size >> 2) + 2 * j + 1];
      float  ofs        = sqrtf(flow_x * flow_x + flow_y * flow_y);
      unsigned int  ofs_i = (unsigned int)ofs;
      if(ofs_i > 255){
        ofs_i = 255;
      }
      unsigned int color = kColors[0];
      if(flow_x >= 0.0f && flow_y >= 0.0f){
        color = ofs_i;
      }else if(flow_x >= 0.0f && flow_y < 0.0f){
        color = ofs_i << 8;
      }else if(flow_x < 0.0f && flow_y >= 0.0f){
        color = ofs_i << 16;
      }else{
        color = (ofs_i << 16) | ofs_i;
      }
      ((unsigned int*)flow_image)[i * (flow_line_size >> 2) + j] = color;
    }
  }

  int color_idx = 0;
  srand(0);
  for(i = 0 ; i < height ; i += 16){
    for(j = 0 ; j < width ; j += 16){
      float  flow_x     = flow_vec[i * (dof_map->line_size >> 2) + 2 * j];
      float  flow_y     = flow_vec[i * (dof_map->line_size >> 2) + 2 * j + 1];
      float  ofs        = sqrtf(flow_x * flow_x + flow_y * flow_y);
      if(ofs < 32.0f){
        continue;
      }

      if(j < 925 || j > (925 + 305) ||
         i < 235 || i > (235 + 136)){
        continue;
      }

      //float  flow_score = flow_vec[i * (dof_map->line_size >> 2) + 4 * j + 2];
      //if(flow_score < 80.0f){
      //  continue;
      //}
      int    start[2]   = { j, i };
      int    stop[2]    = { width + j + flow_x, i + flow_y };
      if(0 != match_dir){
        stop[0] = j      + flow_x;
        stop[1] = height + i + flow_y;
      }
      color_idx = rand() % kColorCnt;
      draw_line_rgb32((unsigned int*)match_image, 
                      match_width, 
                      match_height, 
                      match_line_size >> 2, 
                      start, 
                      stop, 
                      kColors[color_idx], 1);
    }
  }
  SaveBMP(match_file_name,
          match_image, 
          match_width, 
          match_height, 
          match_line_size, 
          32,
          100);
  SaveBMP(flow_file_name,
          flow_image, 
          width, 
          height, 
          flow_line_size, 
          32,
          100);
  free_mem_align(match_image);
  free_mem_align(flow_image);
}

int draw_flow(char const*           flow_file_name,
              unsigned int          width,
              unsigned int          height,
              float const*          flow,
              int                   src_line_size,
              float                 scale){
  unsigned int    i = 0 , j = 0, k = 0 ;
  unsigned int    flow_line_size  = IMG_LINE_ALIGNED(4 * width);
  unsigned char*  flow_image      = (unsigned char*)alloc_mem_align(flow_line_size * height);
  memset(flow_image,  0, flow_line_size  * height);
  static const unsigned int  kColors[] = {
    0x000000FF,
    0x0000FF00,
    0x00FF0000,
    0x0000FFFF,
    0x00FF00FF,
    0x00FFFF00
  };
  static const unsigned int  kColorCnt = sizeof(kColors) / sizeof(kColors[0]);
  float const*               flow_vec  = flow;

  for(i = 0 ; i < height ; i ++){
    for(j = 0 ; j < width ; j ++){
      float  flow_x     = scale * flow_vec[i * (src_line_size >> 2) + 2 * j];
      float  flow_y     = scale * flow_vec[i * (src_line_size >> 2) + 2 * j + 1];
      float  ofs        = sqrtf(flow_x * flow_x + flow_y * flow_y);
      unsigned int  ofs_i = (unsigned int)ofs;
      if(ofs_i > 255){
        ofs_i = 255;
      }
      unsigned int color = kColors[0];
      if(flow_x >= 0.0f && flow_y >= 0.0f){
        color = ofs_i;
      }else if(flow_x >= 0.0f && flow_y < 0.0f){
        color = ofs_i << 8;
      }else if(flow_x < 0.0f && flow_y >= 0.0f){
        color = ofs_i << 16;
      }else{
        color = (ofs_i << 16) | ofs_i;
      }
      ((unsigned int*)flow_image)[i * (flow_line_size >> 2) + j] = color;
    }
  }

  SaveBMP(flow_file_name,
          flow_image, 
          width, 
          height, 
          flow_line_size, 
          32,
          100);
  free_mem_align(flow_image);
}
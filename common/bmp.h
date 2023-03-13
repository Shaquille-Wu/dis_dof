#ifndef BMP_H_
#define BMP_H_

#ifdef __cplusplus
extern "C" {
#endif

int LoadBMP(char const*       file_name,
            unsigned char**   src_img, 
            int*              width, 
            int*              height, 
            int*              line_size,
            int*              depth,
            int               line_size_as_bmp);

void ReleaseBMPData(unsigned char* src_img);

void SaveBMP(char const*           szFileName,
             unsigned char const*  pSrcImg, 
             int                   iWidth, 
             int                   iHeight, 
             int                   iLineSize,
             int                   iDepth,
             int                   iQuality);

#ifdef __cplusplus
}
#endif

#endif
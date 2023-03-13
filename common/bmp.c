#include <stdio.h>
#include <string.h>
#include "bmp.h"
#include <assert.h>
#include <stdlib.h>

#ifdef _WINDOWS
#include <Windows.h>
#else
#ifndef WIN32
#pragma pack(push, 2)
typedef struct tagBITMAPFILEHEADER {
    unsigned short int    bfType ;
    unsigned int          bfSize ;
    unsigned short int    bfReserved1 ;
    unsigned short int    bfReserved2 ;
    unsigned int          bfOffBits ;
}BITMAPFILEHEADER, *PBITMAPFILEHEADER ;
#pragma pack(pop)

typedef struct tagBITMAPINFOHEADER{
    unsigned int          biSize ;
    int                   biWidth ;
    int                   biHeight;
    unsigned short int    biPlanes;
    unsigned short int    biBitCount;
    unsigned int      	  biCompression;
    unsigned int      	  biSizeImage;
    int                   biXPelsPerMeter;
    int                   biYPelsPerMeter;
    unsigned int      	  biClrUsed;
    unsigned int      	  biClrImportant;
} BITMAPINFOHEADER, *PBITMAPINFOHEADER;

typedef struct tagRGBQUAD {
    unsigned char         rgbBlue;
    unsigned char         rgbGreen;
    unsigned char         rgbRed;
    unsigned char         rgbReserved;
}RGBQUAD, *PRGBQUAD;

#define BI_RGB        0L
#define BI_RLE8       1L
#define BI_RLE4       2L
#define BI_BITFIELDS  3L
#define BI_JPEG       4L
#define BI_PNG        5L

#endif

#endif

int LoadBMP(char const*       file_name,
            unsigned char**   dst_img, 
            int*              width, 
            int*              height, 
            int*              line_size,
            int*              depth,
            int               line_size_as_bmp)
{
  int i = 0;
  BITMAPFILEHEADER    stBmpFileHeader ;
  BITMAPINFOHEADER    stBmpInfoHeader ;
  RGBQUAD            *pQuad          = (RGBQUAD*)malloc(256 * sizeof(RGBQUAD)) ;
  unsigned char      *src_ptr = NULL, *dst_ptr = NULL;
  unsigned char*      buf     = NULL;
  memset(&stBmpFileHeader, 0, sizeof(BITMAPFILEHEADER)) ;
  memset(&stBmpInfoHeader, 0, sizeof(BITMAPINFOHEADER)) ;
  memset(pQuad, 0, 256 * sizeof(RGBQUAD)) ;

  int img_width     = 0;
  int img_height    = 0;
  int img_depth     = 0;
  int img_line_size = 0;
  unsigned char* dst_buf = NULL;

  FILE   *fp = NULL ;
  size_t ofs = 0;
  size_t bmp_line_size = 0;
  size_t buf_len       = 0;
#ifdef _WINDOWS
  fopen_s(&fp, file_name, "rb") ;
#else
  fp = fopen(file_name, "rb") ;
#endif
  if(NULL == fp){
    goto error_leave;
  }
  ofs = fread(&stBmpFileHeader, sizeof(BITMAPFILEHEADER), 1, fp);
  if(1 != ofs){
    goto error_leave;
  }
  ofs = fread(&stBmpInfoHeader, sizeof(BITMAPINFOHEADER), 1, fp);
  if(1 != ofs){
    goto error_leave;
  }

  assert(BI_RGB == stBmpInfoHeader.biCompression);
  bmp_line_size = stBmpInfoHeader.biWidth * (stBmpInfoHeader.biBitCount >> 3);
  bmp_line_size = (((bmp_line_size + 3) >> 2) << 2);
  buf_len       = bmp_line_size * (size_t)(stBmpInfoHeader.biHeight);
  
  if(8 == stBmpInfoHeader.biBitCount){
    ofs = fread(pQuad, sizeof(RGBQUAD), 256, fp);
    if(1 != ofs){
      goto error_leave;
    }
  }else{
    assert(24 == stBmpInfoHeader.biBitCount || 32 == stBmpInfoHeader.biBitCount);
  }
  buf = (unsigned char*)malloc(buf_len);
  for(i = 0 ; i < stBmpInfoHeader.biHeight ; i ++){
    ofs = fread(buf + (stBmpInfoHeader.biHeight - 1 - i) * bmp_line_size, 1, bmp_line_size, fp);
    if(bmp_line_size != ofs){
      goto error_leave;
    }
  }

  img_width   = stBmpInfoHeader.biWidth;
  img_height  = stBmpInfoHeader.biHeight;
  img_depth   = stBmpInfoHeader.biBitCount;
  if(0 == line_size_as_bmp &&
     ((size_t)((img_depth >> 3) * img_width)) != bmp_line_size){
    dst_buf = (unsigned char*)malloc(img_width * img_height * (img_depth >> 3));
    src_ptr = buf;
    dst_ptr = dst_buf;
    for(i = 0 ; i < img_height ; i ++){
      memcpy(dst_ptr, src_ptr, (img_depth >> 3) * img_width);
      src_ptr += bmp_line_size;
      dst_ptr += (img_depth >> 3) * img_width;
    }
    bmp_line_size = (img_depth >> 3) * img_width;
    free(buf);
  }else{
    dst_buf = buf;
  }
  img_line_size = bmp_line_size;
  free(pQuad);

  *dst_img     = dst_buf;
  *width       = img_width;
  *height      = img_height;
  *line_size   = img_line_size;
  *depth       = img_depth;

  return 1;

error_leave:
  if(NULL != fp){
    fflush(fp) ;
    fclose(fp) ;
  }
  if(NULL != buf){
    free(buf);
  }
  free(pQuad);
  return 0;
}

void ReleaseBMPData(unsigned char* src_img){
  free(src_img);
}

void SaveBMP(char const*           szFileName,
             unsigned char const*  pSrcImg, 
             int                   iWidth, 
             int                   iHeight, 
             int                   iLineSize,
             int                   iDepth,
             int                   iQuality)
{
  int              i              = 0 ;
  int              iQuaLen        = 0 ;
  int              iBMPLineSize   = 0 ;
  unsigned char*   pBMPImg        = (unsigned char*)pSrcImg ;

  BITMAPFILEHEADER        stBmpFileHeader ;
  BITMAPINFOHEADER        stBmpInfoHeader ;
  RGBQUAD                *pQuad          = (RGBQUAD*)malloc(256 * sizeof(RGBQUAD)) ;
  memset(&stBmpFileHeader, 0, sizeof(BITMAPFILEHEADER)) ;
  memset(&stBmpInfoHeader, 0, sizeof(BITMAPINFOHEADER)) ;
  memset(pQuad, 0, 256 * sizeof(RGBQUAD)) ;

  iBMPLineSize = (((iWidth * (iDepth >> 3)) + 3) >> 2) << 2;

  if(8 == iDepth)
  {
        for (i = 0 ; i < 256 ; i++)
        {
            pQuad[i].rgbRed = pQuad[i].rgbGreen = pQuad[i].rgbBlue = i ;
            pQuad[i].rgbReserved = 0 ;
        }
        iQuaLen = 256 * sizeof(RGBQUAD) ;
  }

  stBmpFileHeader.bfType                  = 0x4D42 ; //BM
  stBmpFileHeader.bfOffBits               = sizeof(BITMAPFILEHEADER) +  sizeof(BITMAPINFOHEADER) + iQuaLen ;
  stBmpFileHeader.bfSize                  = stBmpFileHeader.bfOffBits + iBMPLineSize * iHeight ;
  stBmpInfoHeader.biSize                  = sizeof(BITMAPINFOHEADER) ;
  stBmpInfoHeader.biWidth                 = iWidth ;
  stBmpInfoHeader.biHeight                = iHeight ;
  stBmpInfoHeader.biPlanes                = 1 ;
  stBmpInfoHeader.biBitCount              = iDepth ;
  stBmpInfoHeader.biCompression           = BI_RGB ;
  stBmpInfoHeader.biSizeImage             = iBMPLineSize * iHeight ;
  stBmpInfoHeader.biXPelsPerMeter         = 7872 ;
  stBmpInfoHeader.biYPelsPerMeter         = 7872 ;
  stBmpInfoHeader.biClrUsed               = 0 ;
  stBmpInfoHeader.biClrImportant          = 0 ;

  if(iBMPLineSize != iLineSize)
  {
    pBMPImg = (unsigned char*)malloc(stBmpInfoHeader.biSizeImage);
    for(i = 0 ; i < iHeight ; i ++)
      memcpy(pBMPImg + i * iBMPLineSize, pSrcImg + i * iLineSize, (iDepth >> 3) * iWidth) ;
  }

  FILE *fp = NULL ;
#ifdef _WINDOWS
  fopen_s(&fp, szFileName, "wb") ;
#else
  fp = fopen(szFileName, "wb") ;
#endif
  if(NULL != fp)
  {
    fwrite(&stBmpFileHeader, sizeof(BITMAPFILEHEADER), 1, fp) ;
    fwrite(&stBmpInfoHeader, sizeof(BITMAPINFOHEADER), 1, fp) ;
    if(8 == stBmpInfoHeader.biBitCount)
      fwrite(pQuad, sizeof(RGBQUAD), 256, fp) ;

    for(i = 0 ; i < iHeight ; i ++)
      fwrite(pBMPImg + (iHeight - 1 - i) * iBMPLineSize, 1, iBMPLineSize, fp) ;

    fflush(fp) ;
    fclose(fp) ;
  }

  if(NULL != pBMPImg && pBMPImg != pSrcImg)
    free(pBMPImg);
  pBMPImg = NULL ;

  free(pQuad);
}
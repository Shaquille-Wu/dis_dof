#include "align_mem.h"
#include <stdint.h>

#define  MEM_ALIGNED                 32

#ifndef  ALLOC_MEM_MAX
#define  ALLOC_MEM_MAX  0x7FFFFFFF
#endif

void*  alloc_mem_align(size_t uMemSize)
{
  void *ptr = 0 ;

  size_t diff = 0 ;

  if (uMemSize > (ALLOC_MEM_MAX - MEM_ALIGNED) || uMemSize <= 0)
    return NULL;

  ptr = malloc(uMemSize + MEM_ALIGNED) ;
  if (!ptr)
    return ptr;

  diff = ((~(size_t)ptr)&(MEM_ALIGNED - 1)) + 1 ;

  ptr = (unsigned char *)ptr + diff ;
  ((unsigned char *)ptr)[-1] = (unsigned char)diff ;

  return ptr;
}

void   free_mem_align(void* pMem)
{
  if (pMem)
  {
    int v = ((unsigned char *)pMem)[-1];
    if (v <= 0 || v > MEM_ALIGNED)   return;
    //assert(v > 0 && v <= MEM_ALIGNED) ;
    free((unsigned char *)pMem - v);
  }
}
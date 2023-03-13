#ifndef __ALIGN_MEM_H__
#define __ALIGN_MEM_H__

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

  void*  alloc_mem_align(size_t mem_size) ;

  void   free_mem_align(void* mem_ptr) ;

#ifdef __cplusplus
}
#endif

#endif
CC=gcc

TOP_PATH= $(abspath ../)
CV_PREPROC_INSTALL_DIR=${TOP_PATH}/build/Release/x86_64/cv_preproc/install
INC_DIR=./include
COMMON_DIR=./common
CC_FLAGS  = -O0 -g -I${CV_PREPROC_INSTALL_DIR}/include -I${INC_DIR} -I${COMMON_DIR} -L${CV_PREPROC_INSTALL_DIR}/lib

all: dis_dof_test

SRC= main.c \
     dis_implement.c \
     pyramid.c \
	 pyramid_add_sub.c \
	 pyramid_grad.c \
	 pyramid_integral.c \
     ${COMMON_DIR}/bmp.c \
	 ${COMMON_DIR}/align_mem.c

dis_dof_test: ${SRC}
	$(CC) ${CC_FLAGS} ${SRC} -o dis_dof_test -lcv_preproc -lm

clean:
	rm dis_dof_test

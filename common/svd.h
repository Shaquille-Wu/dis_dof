#ifndef _SVD_H_
#define _SVD_H_

int svd(double const*  A,
        const int      rows,
        const int      cols,
        double*        U,
        double*        V,
        double*        U1,
        double*        diag,
        double*        superdiag);

#endif /* _SVD_H_ */


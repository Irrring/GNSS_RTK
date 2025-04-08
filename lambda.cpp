#include <Eigen/Dense>
#include <math.h>
#include <memory.h>
#include "my_rtk.hpp"

/* constants/macros ----------------------------------------------------------*/

#define LOOPMAX     1000000           /* maximum count of search loop */

#define SGN(x)      ((x)<=0.0?-1.0:1.0)
#define ROUND(x)    (floor((x)+0.5))
#define SWAP(x,y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)


/* LD factorization (Q=L'*diag(D)*L) -----------------------------------------*/
int LD(int n, const Eigen::MatrixXd & Q, Eigen::MatrixXd & L, Eigen::VectorXd & D)
{
    int i, j, k, info = 0;
    Eigen::MatrixXd A = Q;
    L.setZero(); // 确保L矩阵初始化为0

    for (i = n - 1; i >= 0; i--) {
        D(i) = A(i, i);
        if (D(i) <= 0.0) {
            info = -1; break;
        }
        double a = sqrt(D(i));
        for (j = 0; j <= i; j++) L(i, j) = A(i, j) / a;
        for (j = 0; j <= i - 1; j++) for (k = 0; k <= j; k++) A(j, k) -= L(i, k) * L(i, j);
        for (j = 0; j <= i; j++) L(i, j) /= L(i, i);
    }

    return info;
}

/* integer gauss transformation ----------------------------------------------*/
void gauss(int n, Eigen::MatrixXd& L, Eigen::MatrixXd& Z, int i, int j)
{
    int k, mu;

    if ((mu = (int)ROUND(L(i, j))) != 0) {
        for (k = i; k < n; k++) L(k, j) -= (double)mu * L(k, i);
        for (k = 0; k < n; k++) Z(k, j) -= (double)mu * Z(k, i);
    }
}

/* permutations --------------------------------------------------------------*/
void perm(int n, Eigen::MatrixXd& L, Eigen::VectorXd& D, int j, double del, Eigen::MatrixXd& Z)
{
    int k;
    double eta, lam, a0, a1;

    eta = D(j) / del;
    lam = D(j + 1) * L(j + 1, j) / del;
    D(j) = eta * D(j + 1); D(j + 1) = del;
    for (k = 0; k <= j - 1; k++) {
        a0 = L(j, k); a1 = L(j + 1, k);
        L(j, k) = -L(j + 1, j) * a0 + a1;
        L(j + 1, k) = eta * a0 + lam * a1;
    }
    L(j + 1, j) = lam;
    for (k = j + 2; k < n; k++) std::swap(L(k, j), L(k, j + 1));
    for (k = 0; k < n; k++) std::swap(Z(k, j), Z(k, j + 1));
}

/* lambda reduction (z=Z'*a, Qz=Z'*Q*Z=L'*diag(D)*L) (ref.[1]) ---------------*/
void reduction(int n, Eigen::MatrixXd& L, Eigen::VectorXd& D, Eigen::MatrixXd& Z)
{
    int i, j, k;
    double del;

    j = n - 2; k = n - 2;
    while (j >= 0) {
        if (j <= k) for (i = j + 1; i < n; i++) gauss(n, L, Z, i, j);
        del = D(j) + L(j + 1, j) * L(j + 1, j) * D(j + 1);
        if (del + 1E-6 < D(j + 1)) { /* compared considering numerical error */
            perm(n, L, D, j, del, Z);
            k = j; j = n - 2;
        }
        else j--;
    }
}

/* modified lambda (mlambda) search (ref. [2]) -------------------------------*/
int search(int n, int m, const Eigen::MatrixXd& L, const Eigen::VectorXd& D, const Eigen::VectorXd& zs, Eigen::MatrixXd& zn, Eigen::VectorXd& s)
{
    int i, j, k, c, nn = 0, imax = 0;
    double newdist, maxdist = 1E99, y;
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd dist = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd zb = zs;
    Eigen::VectorXd z = zs.unaryExpr([](double v) { return ROUND(v); });
    Eigen::VectorXd step = z.unaryExpr([](double v) { return SGN(v); });

    k = n - 1; dist(k) = 0.0;
    y = zb(k) - z(k);

    for (c = 0; c < LOOPMAX; c++)
    {
        newdist = dist(k) + y * y / D(k);
        if (newdist < maxdist)
        {
            if (k != 0)
            {
                dist(--k) = newdist;
                for (i = 0; i <= k; i++) S(k, i) = S(k + 1, i) + (z(k + 1) - zb(k + 1)) * L(k + 1, i);
                zb(k) = zs(k) + S(k, k);
                z(k) = ROUND(zb(k)); y = zb(k) - z(k); step(k) = SGN(y);
            }
            else {
                if (nn < m)
                {
                    if (nn == 0 || newdist > s(imax)) imax = nn;
                    zn.col(nn) = z;
                    s(nn++) = newdist;
                }
                else {
                    if (newdist < s(imax))
                    {
                        zn.col(imax) = z;
                        s(imax) = newdist;
                        for (i = imax = 0; i < m; i++) if (s(imax) < s(i)) imax = i;
                    }
                    maxdist = s(imax);
                }
                z(0) += step(0); y = zb(0) - z(0); step(0) = -step(0) - SGN(step(0));
            }
        }
        else {
            if (k == n - 1) break;
            else {
                k++;
                z(k) += step(k); y = zb(k) - z(k); step(k) = -step(k) - SGN(step(k));
            }
        }
    }
    for (i = 0; i < m - 1; i++) { /* sort by s */
        for (j = i + 1; j < m; j++) {
            if (s(i) < s(j)) continue;
            std::swap(s(i), s(j));
            zn.col(i).swap(zn.col(j));
        }
    }

    if (c >= LOOPMAX) {
        return -1;
    }

    return 0;
}



/* lambda/mlambda integer least-square estimation ------------------------------
* integer least-square estimation. reduction is performed by lambda (ref.[1]),
* and search by mlambda (ref.[2]).
* args   : int    n      I  number of float parameters
*          int    m      I  number of fixed solutions
*          Eigen::VectorXd& a     I  float parameters (n x 1)
*          Eigen::MatrixXd& Q     I  covariance matrix of float parameters (n x n)
*          Eigen::MatrixXd& F     O  fixed solutions (n x m)
*          Eigen::VectorXd& s     O  sum of squared residulas of fixed solutions (1 x m)
* return : status (0:ok,other:error)
* notes  : matrix stored by column-major order (fortran convension)
*-----------------------------------------------------------------------------*/
int lambda(int n, int m, const Eigen::VectorXd& a, const Eigen::MatrixXd& Q, Eigen::MatrixXd& F, Eigen::VectorXd& s)
{
    int i, info;
    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd D = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd Z = Eigen::MatrixXd::Identity(n, n);
    Eigen::VectorXd z = Eigen::VectorXd::Zero(n);
    Eigen::MatrixXd E = Eigen::MatrixXd::Zero(n, m);

    /* LD factorization */
    if (!(info = LD(n, Q, L, D)))
    {
        /* lambda reduction */
        reduction(n, L, D, Z);

        /* Z-Transformation */
        z = Z.transpose() * a;

        /* mlambda search */
        if (!(info = search(n, m, L, D, z, E, s)))
        {
            F = Z.transpose().inverse() * E;
        }
    }

    return info;
}

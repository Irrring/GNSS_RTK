#include <math.h>
#include <memory.h>
#include "my_rtk.hpp"

/* constants/macros ----------------------------------------------------------*/

#define LOOPMAX     1000000           /* maximum count of search loop */

#define SGN(x)      ((x)<=0.0?-1.0:1.0)
#define ROUND(x)    (floor((x)+0.5))
#define SWAP(x,y)   do {double tmp_; tmp_=x; x=y; y=tmp_;} while (0)

/* 矩阵乘法函数 ------------------------------------------------------------*/
void MatrixMultiply(int n, int m, int p, const double* A, const double* B, double* C)
{
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < p; j++) {
            C[i * p + j] = 0;
            for (int k = 0; k < m; k++) {
                C[i * p + j] += A[i * m + k] * B[k * p + j];
            }
        }
    }
}

/* 矩阵转置函数 ------------------------------------------------------------*/
void MatrixTranspose(int n, int m, const double* A, double* AT)
{
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            AT[j * n + i] = A[i * m + j];
        }
    }
}

/* 矩阵求逆函数（使用高斯消去法） ----------------------------------------*/
int MatrixInverse(int n, double* A, double* A_inv)
{
    int i, j, k;
    double temp;

    // 创建单位矩阵
    double* I = new double[n * n];
    memset(I, 0, sizeof(double) * n * n);
    for (i = 0; i < n; i++) {
        I[i * n + i] = 1.0;
    }

    // 高斯消去法求解矩阵逆
    for (i = 0; i < n; i++) {
        // 选取主元素
        temp = A[i * n + i];
        if (temp == 0.0) {
            delete[] I;
            return -1; // 如果主元素为0，矩阵不可逆
        }

        for (j = 0; j < n; j++) {
            A[i * n + j] /= temp;
            I[i * n + j] /= temp;
        }

        for (j = 0; j < n; j++) {
            if (i != j) {
                temp = A[j * n + i];
                for (k = 0; k < n; k++) {
                    A[j * n + k] -= A[i * n + k] * temp;
                    I[j * n + k] -= I[i * n + k] * temp;
                }
            }
        }
    }

    memcpy(A_inv, I, sizeof(double) * n * n);
    delete[] I;
    return 0; // 成功
}

/* LD分解 (Q = L' * diag(D) * L) -------------------------------------------*/
int LD(int n, const double* Q, double* L, double* D)
{
    int i, j, k, info = 0;
    double* A = new double[n * n];
    memcpy(A, Q, sizeof(double) * n * n);
    for (i = n - 1; i >= 0; i--) {
        if ((D[i] = A[i + i * n]) <= 0.0) {
            info = -1;
            break;
        }
        double a = sqrt(D[i]);
        for (j = 0; j <= i; j++) L[i + j * n] = A[i + j * n] / a;
        for (j = 0; j <= i - 1; j++) for (k = 0; k <= j; k++) A[j + k * n] -= L[i + k * n] * L[i + j * n];
        for (j = 0; j <= i; j++) L[i + j * n] /= L[i + i * n];
    }

    delete[] A;
    return info;
}


/* permutations --------------------------------------------------------------*/
void perm(int n, double* L, double* D, int j, double del, double* Z)
{
    int k;
    double eta, lam, a0, a1;

    eta = D[j] / del;
    lam = D[j + 1] * L[j + 1 + j * n] / del;
    D[j] = eta * D[j + 1]; D[j + 1] = del;
    for (k = 0; k <= j - 1; k++) {
        a0 = L[j + k * n]; a1 = L[j + 1 + k * n];
        L[j + k * n] = -L[j + 1 + j * n] * a0 + a1;
        L[j + 1 + k * n] = eta * a0 + lam * a1;
    }
    L[j + 1 + j * n] = lam;
    for (k = j + 2; k < n; k++) SWAP(L[k + j * n], L[k + (j + 1) * n]);
    for (k = 0; k < n; k++) SWAP(Z[k + j * n], Z[k + (j + 1) * n]);
}



/* Gaussian变换 ------------------------------------------------------------*/
void gauss(int n, double* L, double* Z, int i, int j)
{
    int k, mu;

    if ((mu = (int)ROUND(L[i + j * n])) != 0) {
        for (k = i; k < n; k++) L[k + j * n] -= (double)mu * L[k + i * n];
        for (k = 0; k < n; k++) Z[k + j * n] -= (double)mu * Z[k + i * n];
    }
}

/* Lambda约简 ------------------------------------------------------------*/
void reduction(int n, double* L, double* D, double* Z)
{
    int i, j, k;
    double del;

    j = n - 2;
    k = n - 2;
    while (j >= 0) {
        if (j <= k) for (i = j + 1; i < n; i++) gauss(n, L, Z, i, j);
        del = D[j] + L[j + 1 + j * n] * L[j + 1 + j * n] * D[j + 1];
        if (del + 1E-6 < D[j + 1]) {
            perm(n, L, D, j, del, Z);
            k = j;
            j = n - 2;
        }
        else {
            j--;
        }
    }
}

/* Lambda搜索 ------------------------------------------------------------*/
int search(int n, int m, const double* L, const double* D, const double* zs, double* zn, double* s)
{
    int nn = 0, imax = 0;
    double newdist, maxdist = 1E99, y;
    double* S = new double[n * n];
    double* dist = new double[n];
    double* zb = new double[n];
    double* z = new double[n];
    double* step = new double[n];

    memset(S, 0, sizeof(double) * n * n);

    int c = 0;
    int k = n - 1;
    dist[k] = 0.0;
    zb[k] = zs[k];
    z[k] = ROUND(zb[k]);
    y = zb[k] - z[k];
    step[k] = SGN(y);

    for (c ; c < LOOPMAX; c++) {
        newdist = dist[k] + y * y / D[k];
        if (newdist < maxdist) {
            if (k != 0) {
                dist[--k] = newdist;
                for (int i = 0; i <= k; i++) S[k + i * n] = S[k + 1 + i * n] + (z[k + 1] - zb[k + 1]) * L[k + 1 + i * n];
                zb[k] = zs[k] + S[k + k * n];
                z[k] = ROUND(zb[k]);
                y = zb[k] - z[k];
                step[k] = SGN(y);
            }
            else {
                if (nn < m) {
                    if (nn == 0 || newdist > s[imax]) imax = nn;
                    memcpy(zn + nn * n, z, sizeof(double) * n);
                    s[nn++] = newdist;
                }
                else {
                    if (newdist < s[imax]) {
                        memcpy(zn + imax * n, z, sizeof(double) * n);
                        s[imax] = newdist;
                        for (int i = imax = 0; i < m; i++) if (s[imax] < s[i]) imax = i;
                    }
                    maxdist = s[imax];
                }
                z[0] += step[0];
                y = zb[0] - z[0];
                step[0] = -step[0] - SGN(step[0]);
            }
        }
        else {
            if (k == n - 1) break;
            else {
                k++;
                z[k] += step[k];
                y = zb[k] - z[k];
                step[k] = -step[k] - SGN(step[k]);
            }
        }
    }

    for (int i = 0; i < m - 1; i++) {
        for (int j = i + 1; j < m; j++) {
            if (s[i] < s[j]) continue;
            SWAP(s[i], s[j]);
            for (int k = 0; k < n; k++) SWAP(zn[k + i * n], zn[k + j * n]);
        }
    }

    if (c >= LOOPMAX) {
        delete[] S;
        delete[] dist;
        delete[] zb;
        delete[] z;
        delete[] step;
        return -1;
    }

    delete[] S;
    delete[] dist;
    delete[] zb;
    delete[] z;
    delete[] step;

    return 0;
}

/* Lambda算法的实现 --------------------------------------------------------*/
bool my_lambda(const double* floatAmb, const double* Q, double* fixedAmb, double& ratio, int n, int m) {
    // 调用LD分解进行矩阵分解
    double* L = new double[n * n];
    double* D = new double[n];
    double* Z = new double[n * n];
    double* aflt = new double[n];
    double* afix = new double[n * m];
    double* sqnorm = new double[m];

    for (int i = 0; i < n; i++) {
        aflt[i] = floatAmb[i];
    }

    // 进行LD分解
    int info = LD(n, Q, L, D);
    if (info != 0) {
        delete[] L;
        delete[] D;
        delete[] Z;
        delete[] aflt;
        delete[] afix;
        delete[] sqnorm;
        return false;
    }

    // 进行Lambda约简
    reduction(n, L, D, Z);

    // 使用Lambda算法进行搜索
    info = search(n, m, L, D, aflt, afix, sqnorm);
    if (info != 0) {
        delete[] L;
        delete[] D;
        delete[] Z;
        delete[] aflt;
        delete[] afix;
        delete[] sqnorm;
        return false;
    }

    ratio = sqnorm[1] / sqnorm[0];
    memcpy(fixedAmb, afix, sizeof(double) * n);

    delete[] L;
    delete[] D;
    delete[] Z;
    delete[] aflt;
    delete[] afix;
    delete[] sqnorm;

    return true;
}

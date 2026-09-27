package com.github.guoyyit.poseestimationjava.core;

/**
 * Minimal dense matrix helpers for the small fixed-size EKF algebra.
 * Matrices are row-major double[n][n]. Not intended for large problems.
 */
final class MatrixOps {

    private MatrixOps() {
    }

    static double[][] identity(int n) {
        double[][] m = new double[n][n];
        for (int i = 0; i < n; i++) {
            m[i][i] = 1.0;
        }
        return m;
    }

    static double[][] multiply(double[][] a, double[][] b) {
        int n = a.length;
        int k = b.length;
        int m = b[0].length;
        double[][] r = new double[n][m];
        for (int i = 0; i < n; i++) {
            for (int kk = 0; kk < k; kk++) {
                double aik = a[i][kk];
                if (aik == 0.0) {
                    continue;
                }
                for (int j = 0; j < m; j++) {
                    r[i][j] += aik * b[kk][j];
                }
            }
        }
        return r;
    }

    static double[][] transpose(double[][] a) {
        int n = a.length;
        int m = a[0].length;
        double[][] r = new double[m][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                r[j][i] = a[i][j];
            }
        }
        return r;
    }

    static double[][] add(double[][] a, double[][] b) {
        int n = a.length;
        int m = a[0].length;
        double[][] r = new double[n][m];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                r[i][j] = a[i][j] + b[i][j];
            }
        }
        return r;
    }

    static double[][] subtract(double[][] a, double[][] b) {
        int n = a.length;
        int m = a[0].length;
        double[][] r = new double[n][m];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < m; j++) {
                r[i][j] = a[i][j] - b[i][j];
            }
        }
        return r;
    }

    /** Symmetrizes in place: a <- (a + a^T) / 2. Guards against asymmetry drift. */
    static void symmetrize(double[][] a) {
        int n = a.length;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double v = 0.5 * (a[i][j] + a[j][i]);
                a[i][j] = v;
                a[j][i] = v;
            }
        }
    }

    /** Ensures each diagonal element is at least floor (variance floor). */
    static void floorDiagonal(double[][] a, double floor) {
        for (int i = 0; i < a.length; i++) {
            if (a[i][i] < floor) {
                a[i][i] = floor;
            }
        }
    }

    /** Flattens row-major into a fresh array. */
    static double[] flatten(double[][] a) {
        int n = a.length;
        int m = a[0].length;
        double[] r = new double[n * m];
        for (int i = 0; i < n; i++) {
            System.arraycopy(a[i], 0, r, i * m, m);
        }
        return r;
    }

    /** Inverse of a small positive-definite matrix via Gauss-Jordan with pivoting. */
    static double[][] invert(double[][] a) {
        int n = a.length;
        double[][] aug = new double[n][2 * n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(a[i], 0, aug[i], 0, n);
            aug[i][n + i] = 1.0;
        }
        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int row = col + 1; row < n; row++) {
                if (Math.abs(aug[row][col]) > Math.abs(aug[pivot][col])) {
                    pivot = row;
                }
            }
            double[] tmp = aug[col];
            aug[col] = aug[pivot];
            aug[pivot] = tmp;
            double d = aug[col][col];
            if (Math.abs(d) < 1e-15) {
                d = d < 0 ? -1e-15 : 1e-15;
            }
            for (int j = 0; j < 2 * n; j++) {
                aug[col][j] /= d;
            }
            for (int row = 0; row < n; row++) {
                if (row == col) {
                    continue;
                }
                double f = aug[row][col];
                if (f == 0.0) {
                    continue;
                }
                for (int j = 0; j < 2 * n; j++) {
                    aug[row][j] -= f * aug[col][j];
                }
            }
        }
        double[][] inv = new double[n][n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(aug[i], n, inv[i], 0, n);
        }
        return inv;
    }
}

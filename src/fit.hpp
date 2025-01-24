// fit.hpp
#ifndef FIT_HPP
#define FIT_HPP

#include <string>
#include <vector>
#include <cmath>
#include <sstream>
#include <iomanip>

class FloatPoint
{
  public:
    float x, y;
    FloatPoint(float x, float y)
    {
        this->x = x;
        this->y = y;
    }
};

class Equation
{
  private:
    float *c;
    int deg;

void gaussEliminationLS(int m, int n, float **a, float *x)
{
    // Gaussian elimination with partial pivoting
    for (int i = 0; i < m; i++) {
        // Find pivot
        int max_row = i;
        for (int k = i + 1; k < m; k++) {
            if (std::abs(a[k][i]) > std::abs(a[max_row][i])) {
                max_row = k;
            }
        }

        // Swap rows
        for (int k = i; k < n; k++) {
            std::swap(a[max_row][k], a[i][k]);
        }

        // Eliminate column
        for (int k = i + 1; k < m; k++) {
            float factor = a[k][i] / a[i][i];
            for (int j = i; j < n; j++) {
                a[k][j] -= factor * a[i][j];
            }
        }
    }

    // Back-substitution
    for (int i = m - 1; i >= 0; i--) {
        x[i] = a[i][n-1];
        for (int j = i + 1; j < m; j++) {
            x[i] -= a[i][j] * x[j];
        }
        x[i] /= a[i][i];
    }
}

public:
    Equation() : c(nullptr), deg(0) {}

    ~Equation() {
        if (c) delete[] c;
    }

    float evaluate(float x) {
        if (!c) return 0;

        float result = 0;
        for (int i = 0; i <= deg; i++) {
            result += c[i] * std::pow(x, i);
        }
        return result;
    }

    void fit(int n, std::vector<FloatPoint> &data) {
        // Cleanup previous coefficients
        if (c) delete[] c;

        deg = n;
        c = new float[deg + 1]();

        // Prepare matrices for least squares
        float **a = new float*[n+1];
        for (int i = 0; i <= n; i++) {
            a[i] = new float[n+2]();
        }

        // Build normal equations matrix
        for (size_t i = 0; i < data.size(); i++) {
            for (int j = 0; j <= n; j++) {
                for (int k = 0; k <= n; k++) {
                    a[j][k] += std::pow(data[i].x, j+k);
                }
                a[j][n+1] += data[i].y * std::pow(data[i].x, j);
            }
        }

        // Solve using Gaussian elimination
        float *x = new float[n+1];
        gaussEliminationLS(n+1, n+2, a, x);

        // Store coefficients
        for (int i = 0; i <= n; i++) {
            c[i] = x[i];
        }

        // Cleanup
        delete[] x;
        for (int i = 0; i <= n; i++) {
            delete[] a[i];
        }
        delete[] a;
    }

    std::string toString() {
        if (!c) return "No equation";

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4);
        bool first = true;

        for (int i = deg; i >= 0; i--) {
            if (c[i] != 0) {
                if (!first && c[i] > 0) oss << "+ ";
                if (c[i] != 1 || i == 0) oss << c[i];
                if (i > 1) oss << "x^" << i << " ";
                else if (i == 1) oss << "x ";
                first = false;
            }
        }
        return oss.str();
    }
};

#endif // FIT_HPP
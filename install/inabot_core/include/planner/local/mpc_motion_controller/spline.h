// spline.h
#ifndef TK_SPLINE_H
#define TK_SPLINE_H

#include <vector>
#include <cassert>
#include <cmath>

namespace tk
{
class spline
{
public:
    enum class boundary_type {
        first_derivative = 1,
        second_derivative = 2
    };

private:
    std::vector<double> m_x, m_y, m_a, m_b, m_c, m_d;

public:
    spline() {}

    void set_points(const std::vector<double>& x, const std::vector<double>& y)
    {
        assert(x.size() == y.size());
        assert(x.size() > 2);

        m_x = x;
        m_y = y;
        int n = x.size();

        m_a = y;

        std::vector<double> h(n - 1), alpha(n - 1), l(n), mu(n), z(n);
        for (int i = 0; i < n - 1; ++i) h[i] = x[i + 1] - x[i];

        for (int i = 1; i < n - 1; ++i)
        {
            alpha[i] = (3.0 / h[i]) * (m_a[i + 1] - m_a[i]) - (3.0 / h[i - 1]) * (m_a[i] - m_a[i - 1]);
        }

        l[0] = 1.0;
        mu[0] = z[0] = 0.0;

        for (int i = 1; i < n - 1; ++i)
        {
            l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[n - 1] = 1.0;
        z[n - 1] = m_c[n - 1] = 0.0;

        m_c.resize(n);
        m_b.resize(n - 1);
        m_d.resize(n - 1);

        for (int j = n - 2; j >= 0; --j)
        {
            m_c[j] = z[j] - mu[j] * m_c[j + 1];
            m_b[j] = (m_a[j + 1] - m_a[j]) / h[j] - h[j] * (m_c[j + 1] + 2.0 * m_c[j]) / 3.0;
            m_d[j] = (m_c[j + 1] - m_c[j]) / (3.0 * h[j]);
        }
    }

    double operator()(double x) const
    {
        auto it = std::lower_bound(m_x.begin(), m_x.end(), x);
        size_t i = std::max(int(it - m_x.begin()) - 1, 0);

        double dx = x - m_x[i];
        return m_a[i] + m_b[i] * dx + m_c[i] * dx * dx + m_d[i] * dx * dx * dx;
    }
};
}

#endif
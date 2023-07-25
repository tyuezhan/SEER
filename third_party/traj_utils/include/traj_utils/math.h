/**
 * @file math.h
 * @brief Polynomial roots solver

 * Solving real roots for n-th order polynomial:
    if n < 5, the closed form solution will be calculated;
    if n >= 5, using Eigen Polynomials solver which is slower but correct.
 */
#pragma once

#include <iostream>
#include <unsupported/Eigen/Polynomials>

inline double normalize_angle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

/// Quadratic equation: \f$b*t^2+c*t+d = 0\f$
inline std::vector<double> quad(double b, double c, double d) {
  std::vector<double> dts;
  double p = c * c - 4 * b * d;
  if (p < 0)
    return dts;
  else {
    dts.push_back((-c - sqrt(p)) / (2 * b));
    dts.push_back((-c + sqrt(p)) / (2 * b));
    return dts;
  }
}

/// Cubic equation: \f$a*t^3+b*t^2+c*t+d = 0\f$
inline std::vector<double> cubic(double a, double b, double c,
                                    double d) {
  std::vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;
  // printf("a: %f, b: %f, c: %f, d: %f\n", a, b, c, d);

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  // printf("R: %f, Q: %f, D: %f\n", R, Q, D);
  if (D > 0) {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    // printf("S: %f, T: %f\n", S, T);
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  } else if (D == 0) {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  } else {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

/// Quartic equation: \f$a*t^4+b*t^3+c*t^2+d*t+e = 0\f$
inline std::vector<double> quartic(double a, double b, double c,
                                   double d, double e) {
  std::vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  std::vector<double> ys =
      cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  // printf("y1: %f\n", y1);
  double r = a3 * a3 / 4 - a2 + y1;
  // printf("r: %f\n", r);

  // printf("a = %f, b = %f, c = %f, d = %f, e = %f\n", a, b, c, d, e);
  if (r < 0) return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0) {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 +
             0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 -
             0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  } else {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D)) {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E)) {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

/*! \brief General solver for \f$a*t^4+b*t^3+c*t^2+d*t+e = 0\f$

  \f$a, b, c\f$ can be zero. The function itself checks the highest order of the
  polynomial.
  */
inline std::vector<double> solve(double a, double b, double c,
                                    double d, double e) {
  std::vector<double> ts;
  if (a != 0)
    return quartic(a, b, c, d, e);
  else if (b != 0)
    return cubic(b, c, d, e);
  else if (c != 0)
    return quad(c, d, e);
  else if (d != 0) {
    ts.push_back(-e / d);
    return ts;
  } else
    return ts;
}

/// General solver for \f$a*t^5+b*t^4+c*t^3+d*t^2+e*t+f = 0\f$
inline std::vector<double> solve(double a, double b, double c,
                                 double d, double e, double f) {
  std::vector<double> ts;
  if (a == 0)
    return solve(b, c, d, e, f);
  else {
    Eigen::VectorXd coeff(6);
    coeff << f, e, d, c, b, a;
    Eigen::PolynomialSolver<double, 5> solver;
    solver.compute(coeff);

    const Eigen::PolynomialSolver<double, 5>::RootsType &r = solver.roots();
    std::vector<double> ts;
    // std::cout << coeff.transpose() << std::endl;
    for (int i = 0; i < r.rows(); ++i) {
      if (r[i].imag() == 0) {
        // std::cout << r[i] << std::endl;
        ts.push_back(r[i].real());
      }
    }

    return ts;
  }
}

/// General solver for \f$a*t^6+b*t^5+c*t^4+d*t^3+e*t^2+f*t+g = 0\f$
inline std::vector<double> solve(double a, double b, double c,
                                 double d, double e, double f,
                                 double g) {
  std::vector<double> ts;
  if (a == 0 && b == 0)
    return solve(c, d, e, f, g);
  else {
    Eigen::VectorXd coeff(7);
    coeff << g, f, e, d, c, b, a;
    Eigen::PolynomialSolver<double, 6> solver;
    solver.compute(coeff);

    const Eigen::PolynomialSolver<double, 6>::RootsType &r = solver.roots();
    std::vector<double> ts;
    // std::cout << coeff.transpose() << std::endl;
    for (int i = 0; i < r.rows(); ++i) {
      if (r[i].imag() == 0) {
        // std::cout << r[i] << std::endl;
        ts.push_back(r[i].real());
      }
    }

    return ts;
  }
}

/// General solver for \f$a*t^7+b*t^6+c*t^5+d*t^4+e*t^3+f*t^2+g*t+h = 0\f$
inline std::vector<double> solve(double a, double b, double c,
                                 double d, double e, double f,
                                 double g, double h) {
  std::vector<double> ts;
  if (a == 0 && b == 0 && c == 0){
    return solve(d, e, f, g, h);
  }
  else if (a == 0 && b == 0){
    return solve(c, d, e, f, g, h);
  }
  else if (a == 0){
    return solve(b, c, d, e, f, g, h);
  }
  else {
    Eigen::VectorXd coeff(8);
    coeff << h, g, f, e, d, c, b, a;
    Eigen::PolynomialSolver<double, 7> solver;
    solver.compute(coeff);

    const Eigen::PolynomialSolver<double, 7>::RootsType &r = solver.roots();
    std::vector<double> ts;
    // std::cout << coeff.transpose() << std::endl;
    for (int i = 0; i < r.rows(); ++i) {
      if (r[i].imag() == 0) {
        // std::cout << r[i] << std::endl;
        ts.push_back(r[i].real());
      }
    }

    return ts;
  }
}



/// Return \f$n!\f$
inline int factorial(int n) {
  int nf = 1;
  while (n > 0) {
    nf *= n;
    n--;
  }
  return nf;
}

/// Return \f$t^n\f$
inline double power(double t, int n) {
  double tn = 1;
  while (n > 0) {
    tn *= t;
    n--;
  }
  return tn;
  // return n <= 0 ? 1 : power(t, n-1);
}

template <typename Derived>
typename Derived::PlainObject pseudoInverse(
    Eigen::MatrixBase<Derived> const &m) {
  // JacobiSVD: thin U and V are only available when your matrix has a dynamic
  // number of columns.
  constexpr auto flags = (Derived::ColsAtCompileTime == Eigen::Dynamic)
                             ? (Eigen::ComputeThinU | Eigen::ComputeThinV)
                             : (Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<typename Derived::PlainObject> m_svd(m, flags);
  // std::cout << "singular values: " << m_svd.singularValues().transpose()
  //           << "\n";
  return m_svd.solve(Derived::PlainObject::Identity(m.rows(), m.rows()));
}

template <typename Derived>
typename Derived::PlainObject matrixSquareRoot(
    Eigen::MatrixBase<Derived> const &mat, bool semidefinite_mat = false) {
  if (!semidefinite_mat) {
    Eigen::LLT<typename Derived::PlainObject> cov_chol{mat};
    if (cov_chol.info() == Eigen::Success) return cov_chol.matrixL();
  }
  Eigen::LDLT<typename Derived::PlainObject> cov_chol{mat};
  if (cov_chol.info() == Eigen::Success) {
    typename Derived::PlainObject const L = cov_chol.matrixL();
    auto const P = cov_chol.transpositionsP();
    auto const D_sqrt = cov_chol.vectorD().array().sqrt().matrix().asDiagonal();
    return P.transpose() * L * D_sqrt;
  }
  return Derived::PlainObject::Zero(mat.rows(), mat.cols());
}


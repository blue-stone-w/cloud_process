// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#ifndef THEIA_MATH_CLOSED_FORM_POLYNOMIAL_SOLVER_H_
#define THEIA_MATH_CLOSED_FORM_POLYNOMIAL_SOLVER_H_

// These functions provides closed form solutions for n = {2, 3, 4} degree
// polynomials. Solve___Reals returns only the real solutions to the polynomial
// (i.e. solutions with complex components less than the passed in
// tolerance). All methods return the number of solutions obtained and output
// the solutions in the roots array (for m solutions, the first m elements of
// the array will be filled with the roots).

#include <complex>

namespace theia {
// Provides solutions to the equation a*x^2 + b*x + c = 0.
// Returns the real component of all roots by ignoring the imaginary part.
int SolveQuadraticReals(const double a, const double b, const double c,
                        double* roots);
// Returns all real roots where the magnitude of the imaginary component is less
// than a tolerance.
int SolveQuadraticReals(const double a, const double b, const double c,
                        const double tolerance, double* roots);
int SolveQuadratic(const double a, const double b, const double c,
                   std::complex<double>* roots);

// Provides solutions to the equation a*x^3 + b*x^2 + c*x + d = 0 using Cardan's
// method. Returns real roots by simply ignoring the imaginary component.
int SolveCubicReals(const double a, const double b, const double c,
                    const double d, double* roots);

// Returns all real roots where the magnitude of the imaginary component is less
// than a tolerance.
int SolveCubicReals(const double a, const double b, const double c,
                    const double d, const double tolerance, double* roots);

int SolveCubic(const double a, const double b, const double c, const double d,
               std::complex<double>* roots);

// Provides solutions to the equation a*x^4 + b*x^3 + c*x^2 + d*x + e = 0 using
// Ferrari's method to reduce to problem to a cubic.
int SolveQuarticReals(const long double a, const long double b,
                      const long double c, const long double d,
                      const long double e, long double* roots);

// Returns all real roots where the magnitude of the imaginary component is less
// than a tolerance.
int SolveQuarticReals(const long double a, const long double b,
                      const long double c, const long double d,
                      const long double e, const long double tolerance,
                      long double* roots);

int SolveQuartic(const long double a, const long double b, const long double c,
                 const long double d, const long double e,
                 std::complex<long double>* roots);

}       // namespace theia

#endif  // THEIA_MATH_CLOSED_FORM_POLYNOMIAL_SOLVER_H_

//[]---------------------------------------------------------------[]
//|                                                                 |
//| Copyright (C) 2025 Paulo Pagliosa.                              |
//|                                                                 |
//| This software is provided 'as-is', without any express or       |
//| implied warranty. In no event will the authors be held liable   |
//| for any damages arising from the use of this software.          |
//|                                                                 |
//| Permission is granted to anyone to use this software for any    |
//| purpose, including commercial applications, and to alter it and |
//| redistribute it freely, subject to the following restrictions:  |
//|                                                                 |
//| 1. The origin of this software must not be misrepresented; you  |
//| must not claim that you wrote the original software. If you use |
//| this software in a product, an acknowledgment in the product    |
//| documentation would be appreciated but is not required.         |
//|                                                                 |
//| 2. Altered source versions must be plainly marked as such, and  |
//| must not be misrepresented as being the original software.      |
//|                                                                 |
//| 3. This notice may not be removed or altered from any source    |
//| distribution.                                                   |
//|                                                                 |
//[]---------------------------------------------------------------[]
//
// OVERVIEW: Matrix2x2.h
// ========
// Class definition for 2x2 matrix.
//
// Author: Paulo Pagliosa
// Last revision: 28/10/2025

#ifndef __Matrix2x2_h
#define __Matrix2x2_h

#include "math/Vector2.h"

namespace cg
{ // begin namespace cg

template <typename real, int M, int N> class Matrix;


/////////////////////////////////////////////////////////////////////
//
// Matrix2x2: 2x2 matrix class (column-major format)
// =========
template <typename real>
class Matrix<real, 2, 2>
{
public:
  ASSERT_REAL(real, "Matrix2x2: floating point type expected");

  using vec2 = Vector2<real>;
  using mat2 = Matrix<real, 2, 2>;
  using value_type = real;

  /// Default constructor.
  HOST DEVICE
  Matrix()
  {
    // do nothing
  }

  /// Constructs a Matrix2x2 object from [v0; v1].
  HOST DEVICE
  Matrix(const vec2& v0, const vec2& v1)
  {
    set(v0, v1);
  }

  /// Constructs a Matrix4x4 object from v[4].
  HOST DEVICE
  explicit Matrix(const real v[])
  {
    set(v);
  }

  /// Constructs a Matrix2x2 object as a multiple s of the identity matrix.
  HOST DEVICE
  explicit Matrix(real s)
  {
    set(s);
  }

  /// Constructs a Matrix2x2 object from the diagonal d.
  HOST DEVICE
  explicit Matrix(const vec2& d)
  {
    set(d);
  }

  /// Sets the columns of this object to [v0; v1].
  HOST DEVICE
  void set(const vec2& v0, const vec2& v1)
  {
    this->v0 = v0;
    this->v1 = v1;
  }

  /// Sets the elements of this object from v[4].
  HOST DEVICE
  void set(const real v[])
  {
    v0.set(&v[0]);
    v1.set(&v[2]);
  }

  /// Sets this object to a multiple s of the identity matrix.
  HOST DEVICE
  void set(real s)
  {
    v0.set(s, 0);
    v1.set(0, s);
  }

  /// Sets this object to a diagonal matrix d.
  HOST DEVICE
  void set(const vec2& d)
  {
    v0.set(d.x, 0);
    v1.set(0, d.y);
  }

  /// Returns a zero matrix.
  HOST DEVICE
  static auto zero()
  {
    return mat2{(real)0};
  }

  /// Returns an identity matrix.
  HOST DEVICE
  static auto identity()
  {
    return mat2{(real)1};
  }

  /// Returns a diagonal matrix d.
  HOST DEVICE
  static auto diagonal(const vec2& d)
  {
    return mat2{d};
  }

  /// Returns the diagonal of this object.
  HOST DEVICE
  auto diagonal() const
  {
    return vec2{v0.x, v1.y};
  }

  /// Returns the trace of this object.
  HOST DEVICE
  auto trace() const
  {
    return v0.x + v1.y;
  }

  /// Returns a reference to the j-th column of this object.
  HOST DEVICE
  auto& operator [](int j)
  {
    return (&v0)[j];
  }

  /// Returns the j-th column of this object.
  HOST DEVICE
  const auto& operator [](int j) const
  {
    return (&v0)[j];
  }

  /// Returns a reference to the element (i, j) of this object.
  HOST DEVICE
  auto& operator ()(int i, int j)
  {
    return (*this)[j][i];
  }

  /// Returns the element (i, j) of this object.
  HOST DEVICE
  const auto& operator ()(int i, int j) const
  {
    return (*this)[j][i];
  }

  /// Returns this object * s.
  HOST DEVICE
  auto operator *(real s) const
  {
    return mat2{v0 * s, v1 * s};
  }

  /// Returns a reference to this object *= s.
  HOST DEVICE
  auto& operator *=(real s)
  {
    v0 *= s;
    v1 *= s;
    return *this;
  }

  /// Returns this object * m.
  HOST DEVICE
  auto operator *(const mat2& m) const
  {
    const auto b0 = transform(m.v0);
    const auto b1 = transform(m.v1);

    return mat2{b0, b1};
  }

  /// Returns a reference to this object *= m.
  HOST DEVICE
  auto& operator *=(const mat2& m)
  {
    return *this = operator *(m);
  }

  /// Returns this object * v.
  HOST DEVICE
  auto operator *(const vec2& v) const
  {
    return transform(v);
  }

  /// Returns the transposed of this object.
  HOST DEVICE
  auto transposed() const
  {
    const vec2 b0{v0.x, v1.x};
    const vec2 b1{v0.y, v1.y};

    return mat2{b0, b1};
  }

  /// Transposes and returns a reference to this object.
  HOST DEVICE
  auto& transpose()
  {
    return *this = transposed();
  }

  /// \brief Tries to invert this object and returns true on success;
  /// otherwise, leaves this object unchanged and returns false.
  HOST DEVICE
  bool invert(real eps = math::Limits<real>::eps())
  {
    auto d = v0[0] * v1[1] - v0[1] * v1[0];

    if (math::isZero(d, eps))
      return false;
    d = real(1 / d);

    auto b0 = vec2{+v1[1], -v0[1]} * d;
    auto b1 = vec2{-v1[0], +v0[0]} * d;

    v0 = b0;
    v1 = b1;
    return true;
  }

  /// Assigns this object to m and tries to invert m.
  HOST DEVICE
  bool inverse(mat2& m, real eps = math::Limits<real>::eps()) const
  {
    return (m = *this).invert(eps);
  }

  /// Returns v transformed by this object.
  HOST DEVICE
  auto transform(const vec2& v) const
  {
    return v0 * v.x + v1 * v.y;
  }

  /// Returns v transformed by the transposed of this object.
  HOST DEVICE
  auto transposeTransform(const vec2& v) const
  {
    return vec2{v0.dot(v), v1.dot(v)};
  }

  /// Returns a pointer to the elements of this object.
  HOST DEVICE
  explicit operator const real* () const
  {
    return &v0.x;
  }

  void print(const char* s, FILE* f = stdout) const
  {
    fprintf(f, "%s\n", s);
    fprintf(f, "[%9.4g %9.4g]\n", v0.x, v1.x);
    fprintf(f, "[%9.4g %9.4g]\n", v0.y, v1.y);
  }

private:
  vec2 v0; // column 0
  vec2 v1; // column 1

}; // Matrix2x2

template <typename real> using Matrix2x2 = Matrix<real, 2, 2>;

/// Returns s * m.
template <typename real>
HOST DEVICE inline auto
operator *(real s, const Matrix2x2<real>& m)
{
  return m * s;
}

using mat2f = Matrix2x2<float>;
using mat2d = Matrix2x2<double>;

} // end namespace cg

#endif // __Matrix2x2_h

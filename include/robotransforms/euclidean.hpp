#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <type_traits>

namespace robotransforms::euclidean {

template <typename T>
using Vec3 = std::array<T, 3>;

template <typename T>
using Vec4 = std::array<T, 4>;

template <typename T>
using Euler = std::array<T, 3>;

template <typename T>
using RotMat = std::array<Vec3<T>, 3>;

// Homo supports arbitrary 4x4 homogeneous matrices for apply/compose/invert.
// Conversions between Homo and Euclidean pose parameterizations assume an SE(3)
// rigid transform with bottom row [0, 0, 0, 1].
template <typename T>
using Homo = std::array<Vec4<T>, 4>;

template <typename T>
using Quat = std::array<T, 4>;

template <typename T>
using RedQuat = std::array<T, 3>;

template <typename T>
using RotVec = std::array<T, 3>;

template <typename T>
using Srq = std::array<T, 6>;

template <typename T>
using Sre = std::array<T, 6>;

template <typename T>
using Lre = std::array<T, 6>;

template <typename T>
using LrQ = std::array<T, 7>;

template <typename T>
using Lrq = std::array<T, 6>;

template <typename T>
using Lrrv = std::array<T, 6>;

namespace detail {

template <typename T>
inline T abs_value(const T& value) {
    using std::abs;
    return abs(value);
}

template <typename T>
inline T sqrt_value(const T& value) {
    using std::sqrt;
    return sqrt(value);
}

template <typename T>
inline T sin_value(const T& value) {
    using std::sin;
    return sin(value);
}

template <typename T>
inline T cos_value(const T& value) {
    using std::cos;
    return cos(value);
}

template <typename T>
inline T asin_value(const T& value) {
    using std::asin;
    return asin(value);
}

template <typename T>
inline T atan2_value(const T& y, const T& x) {
    using std::atan2;
    return atan2(y, x);
}

template <typename T>
inline T safe_nonnegative(T value) {
    if constexpr (std::is_arithmetic_v<T>) {
        return value < T(0) ? T(0) : value;
    }
    return value;
}

template <typename T>
inline T clip_value(T value, T lower, T upper) {
    if constexpr (std::is_arithmetic_v<T>) {
        if (value < lower) {
            return lower;
        }
        if (value > upper) {
            return upper;
        }
    }
    return value;
}

template <typename T>
inline Vec3<T> add(const Vec3<T>& a, const Vec3<T>& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

template <typename T>
inline Vec3<T> subtract(const Vec3<T>& a, const Vec3<T>& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

template <typename T>
inline Vec3<T> negate(const Vec3<T>& value) {
    return {-value[0], -value[1], -value[2]};
}

template <typename T, std::size_t N>
inline Vec3<T> head3(const std::array<T, N>& value) {
    return {value[0], value[1], value[2]};
}

template <typename T, std::size_t N>
inline Vec3<T> tail3(const std::array<T, N>& value) {
    return {value[3], value[4], value[5]};
}

template <typename T>
inline Quat<T> tail4(const LrQ<T>& value) {
    return {value[3], value[4], value[5], value[6]};
}

template <typename T>
inline RotMat<T> transpose_rotmat(const RotMat<T>& matrix) {
    return {{
        {matrix[0][0], matrix[1][0], matrix[2][0]},
        {matrix[0][1], matrix[1][1], matrix[2][1]},
        {matrix[0][2], matrix[1][2], matrix[2][2]},
    }};
}

template <typename T>
inline Vec3<T> multiply_rotmat_vec3(const RotMat<T>& matrix, const Vec3<T>& value) {
    return {
        matrix[0][0] * value[0] + matrix[0][1] * value[1] + matrix[0][2] * value[2],
        matrix[1][0] * value[0] + matrix[1][1] * value[1] + matrix[1][2] * value[2],
        matrix[2][0] * value[0] + matrix[2][1] * value[1] + matrix[2][2] * value[2],
    };
}

template <typename T>
inline RotMat<T> multiply_rotmat(const RotMat<T>& left, const RotMat<T>& right) {
    RotMat<T> result{};
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            result[i][j] = T(0);
            for (std::size_t k = 0; k < 3; ++k) {
                result[i][j] += left[i][k] * right[k][j];
            }
        }
    }
    return result;
}

template <typename T>
inline Homo<T> multiply_homo(const Homo<T>& left, const Homo<T>& right) {
    Homo<T> result{};
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            result[i][j] = T(0);
            for (std::size_t k = 0; k < 4; ++k) {
                result[i][j] += left[i][k] * right[k][j];
            }
        }
    }
    return result;
}

template <typename T>
inline RotMat<T> rotmat_from_homo(const Homo<T>& matrix) {
    return {{
        {matrix[0][0], matrix[0][1], matrix[0][2]},
        {matrix[1][0], matrix[1][1], matrix[1][2]},
        {matrix[2][0], matrix[2][1], matrix[2][2]},
    }};
}

template <typename T>
inline Vec3<T> translation_from_homo(const Homo<T>& matrix) {
    return {matrix[0][3], matrix[1][3], matrix[2][3]};
}

template <typename T>
inline Homo<T> make_homo(const RotMat<T>& rotation, const Vec3<T>& translation) {
    return {{
        {rotation[0][0], rotation[0][1], rotation[0][2], translation[0]},
        {rotation[1][0], rotation[1][1], rotation[1][2], translation[1]},
        {rotation[2][0], rotation[2][1], rotation[2][2], translation[2]},
        {T(0), T(0), T(0), T(1)},
    }};
}

template <typename T>
inline Homo<T> identity_homo() {
    return {{
        {T(1), T(0), T(0), T(0)},
        {T(0), T(1), T(0), T(0)},
        {T(0), T(0), T(1), T(0)},
        {T(0), T(0), T(0), T(1)},
    }};
}

template <typename T>
inline void invert_homo_subblock(Homo<T>& matrix, Homo<T>& inverse, std::size_t offset) {
    std::size_t pivot_row = offset;
    while (pivot_row < matrix.size() && matrix[pivot_row][offset] == T(0)) {
        ++pivot_row;
    }

    if (pivot_row == matrix.size()) {
        throw std::runtime_error("Matrix is singular");
    }

    if (pivot_row != offset) {
        const auto matrix_row = matrix[offset];
        matrix[offset] = matrix[pivot_row];
        matrix[pivot_row] = matrix_row;

        const auto inverse_row = inverse[offset];
        inverse[offset] = inverse[pivot_row];
        inverse[pivot_row] = inverse_row;
    }

    const T scale = T(1) / matrix[offset][offset];
    for (std::size_t i = 0; i < offset; ++i) {
        inverse[offset][i] *= scale;
    }
    for (std::size_t i = offset; i < matrix.size(); ++i) {
        matrix[offset][i] *= scale;
        inverse[offset][i] *= scale;
    }

    for (std::size_t i = 0; i < matrix.size(); ++i) {
        if (i == offset) {
            continue;
        }

        const T factor = matrix[i][offset];
        for (std::size_t j = 0; j < offset; ++j) {
            inverse[i][j] -= factor * inverse[offset][j];
        }
        for (std::size_t j = offset; j < matrix.size(); ++j) {
            matrix[i][j] -= factor * matrix[offset][j];
            inverse[i][j] -= factor * inverse[offset][j];
        }
    }
}

template <typename T>
inline Homo<T> invert_homo_matrix(const Homo<T>& matrix) {
    auto working = matrix;
    auto inverse = identity_homo<T>();
    for (std::size_t i = 0; i < matrix.size(); ++i) {
        invert_homo_subblock(working, inverse, i);
    }
    return inverse;
}

}  // namespace detail

template <typename T>
inline RotMat<T> convert_euler_to_rotmat(const Euler<T>& euler) {
    const T sy = detail::sin_value(euler[0]);
    const T cy = detail::cos_value(euler[0]);
    const T sp = detail::sin_value(euler[1]);
    const T cp = detail::cos_value(euler[1]);
    const T sr = detail::sin_value(euler[2]);
    const T cr = detail::cos_value(euler[2]);

    return {{
        {cr * cy + sr * sp * sy, -cr * sy + sr * sp * cy, -sr * cp},
        {cp * sy, cp * cy, sp},
        {sr * cy - cr * sp * sy, -sr * sy - cr * sp * cy, cr * cp},
    }};
}

template <typename T>
inline Euler<T> convert_rotmat_to_euler(const RotMat<T>& matrix) {
    return {
        detail::atan2_value(matrix[1][0], matrix[1][1]),
        detail::asin_value(detail::clip_value(matrix[1][2], T(-1), T(1))),
        detail::atan2_value(-matrix[0][2], matrix[2][2]),
    };
}

template <typename T>
inline Quat<T> convert_euler_to_quat(const Euler<T>& euler) {
    const T sy = detail::sin_value(euler[0] * T(0.5));
    const T cy = detail::cos_value(euler[0] * T(0.5));
    const T sp = detail::sin_value(euler[1] * T(0.5));
    const T cp = detail::cos_value(euler[1] * T(0.5));
    const T sr = detail::sin_value(euler[2] * T(0.5));
    const T cr = detail::cos_value(euler[2] * T(0.5));

    return {
        sy * sp * sr + cy * cp * cr,
        cy * sp * cr + sy * cp * sr,
        -sy * sp * cr + cy * cp * sr,
        cy * sp * sr - sy * cp * cr,
    };
}

template <typename T>
inline RedQuat<T> convert_quat_to_redquat(const Quat<T>& q) {
    const T sign = q[0] < T(0) ? T(-1) : T(1);
    return {sign * q[1], sign * q[2], sign * q[3]};
}

template <typename T>
inline Quat<T> convert_redquat_to_quat(const RedQuat<T>& rq) {
    const T imag_norm_sq = rq[0] * rq[0] + rq[1] * rq[1] + rq[2] * rq[2];
    return {
        detail::sqrt_value(detail::safe_nonnegative(T(1) - imag_norm_sq)),
        rq[0],
        rq[1],
        rq[2],
    };
}

template <typename T>
inline RotVec<T> convert_quat_to_rotvec(const Quat<T>& q) {
    const T sign = q[0] < T(0) ? T(-1) : T(1);
    const T imag_norm_sq = q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    const T imag_norm = detail::sqrt_value(detail::safe_nonnegative(imag_norm_sq));
    if (detail::abs_value(imag_norm) < T(1e-12)) {
        return {T(0), T(0), T(0)};
    }

    const T scale = T(2) * sign * detail::asin_value(imag_norm) / imag_norm;
    return {scale * q[1], scale * q[2], scale * q[3]};
}

template <typename T>
inline Quat<T> convert_rotvec_to_quat(const RotVec<T>& rv) {
    const T norm_sq = rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2];
    const T norm = detail::sqrt_value(detail::safe_nonnegative(norm_sq));
    if (detail::abs_value(norm) < T(1e-12)) {
        return {T(1), T(0), T(0), T(0)};
    }

    const T half_norm = norm / T(2);
    const T sin_over_norm = detail::sin_value(half_norm) / norm;
    return {
        detail::cos_value(half_norm),
        rv[0] * sin_over_norm,
        rv[1] * sin_over_norm,
        rv[2] * sin_over_norm,
    };
}

template <typename T>
inline Euler<T> convert_redquat_to_euler(const RedQuat<T>& rq) {
    return convert_quat_to_euler(convert_redquat_to_quat(rq));
}

template <typename T>
inline Euler<T> convert_quat_to_euler(const Quat<T>& q) {
    const T a = q[0];
    const T b = q[1];
    const T c = q[2];
    const T d = q[3];
    return {
        detail::atan2_value(T(2) * (b * c - a * d), T(2) * (a * a + c * c) - T(1)),
        detail::asin_value(detail::clip_value(T(2) * (c * d + a * b), T(-1), T(1))),
        detail::atan2_value(T(2) * (a * c - b * d), T(2) * (a * a + d * d) - T(1)),
    };
}

template <typename T>
inline RotMat<T> convert_redquat_to_rotmat(const RedQuat<T>& rq) {
    return convert_quat_to_rotmat(convert_redquat_to_quat(rq));
}

template <typename T>
inline RotMat<T> convert_quat_to_rotmat(const Quat<T>& q) {
    const T a = q[0];
    const T b = q[1];
    const T c = q[2];
    const T d = q[3];
    const T asq = a * a;
    const T bsq = b * b;
    const T csq = c * c;
    const T dsq = d * d;
    return {{
        {asq + bsq - csq - dsq, T(2) * (b * c + a * d), T(2) * (b * d - a * c)},
        {T(2) * (b * c - a * d), asq - bsq + csq - dsq, T(2) * (c * d + a * b)},
        {T(2) * (b * d + a * c), T(2) * (c * d - a * b), asq - bsq - csq + dsq},
    }};
}

template <typename T>
inline Quat<T> convert_rotmat_to_quat(const RotMat<T>& matrix) {
    const T asq = (T(1) + matrix[0][0] + matrix[1][1] + matrix[2][2]) * T(0.25);
    T b = detail::sqrt_value(detail::clip_value((matrix[0][0] + T(1)) * T(0.5) - asq, T(0), T(1)));
    T c = detail::sqrt_value(detail::clip_value((matrix[1][1] + T(1)) * T(0.5) - asq, T(0), T(1)));
    T d = detail::sqrt_value(detail::clip_value((matrix[2][2] + T(1)) * T(0.5) - asq, T(0), T(1)));

    if (asq > T(1e-12)) {
        b *= matrix[1][2] - matrix[2][1] < T(0) ? T(-1) : T(1);
        c *= matrix[2][0] - matrix[0][2] < T(0) ? T(-1) : T(1);
        d *= matrix[0][1] - matrix[1][0] < T(0) ? T(-1) : T(1);
    } else if (b > T(1e-12)) {
        c *= matrix[0][1] + matrix[1][0] < T(0) ? T(-1) : T(1);
        d *= matrix[2][0] + matrix[0][2] < T(0) ? T(-1) : T(1);
    } else if (c > T(1e-12)) {
        d *= matrix[1][2] + matrix[2][1] < T(0) ? T(-1) : T(1);
    }

    return {detail::sqrt_value(detail::clip_value(asq, T(0), T(1))), b, c, d};
}

template <typename T>
inline LrQ<T> convert_lrq_to_lrQ(const Lrq<T>& lrq) {
    const auto q = convert_redquat_to_quat(detail::tail3(lrq));
    return {lrq[0], lrq[1], lrq[2], q[0], q[1], q[2], q[3]};
}

template <typename T>
inline Lrq<T> convert_lrQ_to_lrq(const LrQ<T>& lrQ) {
    const auto rq = convert_quat_to_redquat(detail::tail4(lrQ));
    return {lrQ[0], lrQ[1], lrQ[2], rq[0], rq[1], rq[2]};
}

template <typename T>
inline LrQ<T> convert_lrrv_to_lrQ(const Lrrv<T>& lrrv) {
    const auto q = convert_rotvec_to_quat(detail::tail3(lrrv));
    return {lrrv[0], lrrv[1], lrrv[2], q[0], q[1], q[2], q[3]};
}

template <typename T>
inline Lrrv<T> convert_lrQ_to_lrrv(const LrQ<T>& lrQ) {
    const auto rv = convert_quat_to_rotvec(detail::tail4(lrQ));
    return {lrQ[0], lrQ[1], lrQ[2], rv[0], rv[1], rv[2]};
}

template <typename T>
inline LrQ<T> convert_lre_to_lrQ(const Lre<T>& lre) {
    const auto q = convert_euler_to_quat(detail::tail3(lre));
    return {lre[0], lre[1], lre[2], q[0], q[1], q[2], q[3]};
}

template <typename T>
inline Lre<T> convert_lrQ_to_lre(const LrQ<T>& lrQ) {
    const auto euler = convert_quat_to_euler(detail::tail4(lrQ));
    return {lrQ[0], lrQ[1], lrQ[2], euler[0], euler[1], euler[2]};
}

template <typename T>
inline Quat<T> invert_quat(const Quat<T>& q) {
    return {q[0], -q[1], -q[2], -q[3]};
}

template <typename T>
inline RedQuat<T> invert_redquat(const RedQuat<T>& rq) {
    return {-rq[0], -rq[1], -rq[2]};
}

template <typename T>
inline RotVec<T> invert_rotvec(const RotVec<T>& rv) {
    return {-rv[0], -rv[1], -rv[2]};
}

template <typename T>
inline Quat<T> compose_quat(const Quat<T>& q1, const Quat<T>& q2) {
    return {
        q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
        q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
        q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3],
        q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1],
    };
}

template <typename T>
inline RedQuat<T> compose_redquat(const RedQuat<T>& rq1, const RedQuat<T>& rq2) {
    return convert_quat_to_redquat(compose_quat(
        convert_redquat_to_quat(rq1),
        convert_redquat_to_quat(rq2)
    ));
}

template <typename T>
inline RotVec<T> compose_rotvec(const RotVec<T>& rv1, const RotVec<T>& rv2) {
    return convert_quat_to_rotvec(compose_quat(
        convert_rotvec_to_quat(rv1),
        convert_rotvec_to_quat(rv2)
    ));
}

template <typename T>
inline RotMat<T> compose_rotmat(const RotMat<T>& matrix1, const RotMat<T>& matrix2) {
    return detail::multiply_rotmat(matrix2, matrix1);
}

template <typename T>
inline Vec3<T> apply_quat(const Quat<T>& q, const Vec3<T>& v) {
    const T a = -v[0] * q[1] - v[1] * q[2] - v[2] * q[3];
    const T b =  v[0] * q[0] + v[1] * q[3] - v[2] * q[2];
    const T c =  v[1] * q[0] + v[2] * q[1] - v[0] * q[3];
    const T d =  v[2] * q[0] + v[0] * q[2] - v[1] * q[1];

    return {
        q[0] * b - q[1] * a - q[2] * d + q[3] * c,
        q[0] * c - q[2] * a - q[3] * b + q[1] * d,
        q[0] * d - q[3] * a - q[1] * c + q[2] * b,
    };
}

template <typename T>
inline Vec3<T> apply_inverse_quat(const Quat<T>& q, const Vec3<T>& v) {
    const T a =  v[0] * q[1] + v[1] * q[2] + v[2] * q[3];
    const T b =  v[0] * q[0] - v[1] * q[3] + v[2] * q[2];
    const T c =  v[1] * q[0] - v[2] * q[1] + v[0] * q[3];
    const T d =  v[2] * q[0] - v[0] * q[2] + v[1] * q[1];

    return {
        q[0] * b + q[1] * a + q[2] * d - q[3] * c,
        q[0] * c + q[2] * a + q[3] * b - q[1] * d,
        q[0] * d + q[3] * a + q[1] * c - q[2] * b,
    };
}

template <typename T>
inline Vec3<T> apply_redquat(const RedQuat<T>& rq, const Vec3<T>& v) {
    return apply_quat(convert_redquat_to_quat(rq), v);
}

template <typename T>
inline Vec3<T> apply_rotvec(const RotVec<T>& rv, const Vec3<T>& v) {
    return apply_quat(convert_rotvec_to_quat(rv), v);
}

template <typename T>
inline Vec3<T> apply_rotmat(const RotMat<T>& matrix, const Vec3<T>& v) {
    return detail::multiply_rotmat_vec3(matrix, v);
}

template <typename T>
inline Vec3<T> apply_euler(const Euler<T>& euler, const Vec3<T>& v) {
    return apply_quat(convert_euler_to_quat(euler), v);
}

template <typename T>
inline Vec3<T> apply_homo(const Homo<T>& matrix, const Vec3<T>& v) {
    const T x = matrix[0][0] * v[0] + matrix[0][1] * v[1] + matrix[0][2] * v[2] + matrix[0][3];
    const T y = matrix[1][0] * v[0] + matrix[1][1] * v[1] + matrix[1][2] * v[2] + matrix[1][3];
    const T z = matrix[2][0] * v[0] + matrix[2][1] * v[1] + matrix[2][2] * v[2] + matrix[2][3];
    const T w = matrix[3][0] * v[0] + matrix[3][1] * v[1] + matrix[3][2] * v[2] + matrix[3][3];
    if constexpr (std::is_arithmetic_v<T>) {
        if (detail::abs_value(w) < T(1e-12)) {
            return {x, y, z};
        }
    }
    return {x / w, y / w, z / w};
}

template <typename T>
inline Vec3<T> apply_srq(const Srq<T>& srq, const Vec3<T>& v) {
    return apply_redquat(detail::tail3(srq), detail::add(v, detail::head3(srq)));
}

template <typename T>
inline Vec3<T> apply_sre(const Sre<T>& sre, const Vec3<T>& v) {
    return apply_euler(detail::tail3(sre), detail::add(v, detail::head3(sre)));
}

// Location-based transforms follow the JS implementation and the documented
// coordinate-transform convention: apply_lr*(T, v) = R(v - location).
template <typename T>
inline Vec3<T> apply_lrQ(const LrQ<T>& lrQ, const Vec3<T>& v) {
    return apply_quat(detail::tail4(lrQ), detail::subtract(v, detail::head3(lrQ)));
}

template <typename T>
inline Vec3<T> apply_lrq(const Lrq<T>& lrq, const Vec3<T>& v) {
    return apply_redquat(detail::tail3(lrq), detail::subtract(v, detail::head3(lrq)));
}

template <typename T>
inline Vec3<T> apply_lrrv(const Lrrv<T>& lrrv, const Vec3<T>& v) {
    return apply_rotvec(detail::tail3(lrrv), detail::subtract(v, detail::head3(lrrv)));
}

template <typename T>
inline Vec3<T> apply_lre(const Lre<T>& lre, const Vec3<T>& v) {
    return apply_euler(detail::tail3(lre), detail::subtract(v, detail::head3(lre)));
}

template <typename T>
inline RotMat<T> invert_rotmat(const RotMat<T>& matrix) {
    return detail::transpose_rotmat(matrix);
}

template <typename T>
inline LrQ<T> invert_lrQ(const LrQ<T>& lrQ) {
    const auto q = detail::tail4(lrQ);
    const auto location = detail::negate(apply_quat(q, detail::head3(lrQ)));
    const auto q_inv = invert_quat(q);
    return {location[0], location[1], location[2], q_inv[0], q_inv[1], q_inv[2], q_inv[3]};
}

template <typename T>
inline Lrq<T> invert_lrq(const Lrq<T>& lrq) {
    const auto rq = detail::tail3(lrq);
    const auto location = detail::negate(apply_redquat(rq, detail::head3(lrq)));
    const auto rq_inv = invert_redquat(rq);
    return {location[0], location[1], location[2], rq_inv[0], rq_inv[1], rq_inv[2]};
}

template <typename T>
inline Lrrv<T> invert_lrrv(const Lrrv<T>& lrrv) {
    const auto rv = detail::tail3(lrrv);
    const auto location = detail::negate(apply_rotvec(rv, detail::head3(lrrv)));
    const auto rv_inv = invert_rotvec(rv);
    return {location[0], location[1], location[2], rv_inv[0], rv_inv[1], rv_inv[2]};
}

template <typename T>
inline Euler<T> invert_euler(const Euler<T>& euler) {
    return convert_quat_to_euler(invert_quat(convert_euler_to_quat(euler)));
}

template <typename T>
inline Srq<T> invert_srq(const Srq<T>& srq) {
    const auto rq = detail::tail3(srq);
    const auto shift = detail::negate(apply_redquat(rq, detail::head3(srq)));
    const auto rq_inv = invert_redquat(rq);
    return {shift[0], shift[1], shift[2], rq_inv[0], rq_inv[1], rq_inv[2]};
}

template <typename T>
inline Homo<T> invert_homo(const Homo<T>& matrix) {
    return detail::invert_homo_matrix(matrix);
}

// This shortcut assumes the upper-left 3x3 block is a rotation matrix and the
// final column is an Euclidean translation.
template <typename T>
inline Homo<T> invert_homo_as_euclidean(const Homo<T>& matrix) {
    const auto rotation = detail::rotmat_from_homo(matrix);
    const auto translation = detail::translation_from_homo(matrix);
    const auto inv_rotation = invert_rotmat(rotation);
    const auto inv_translation = detail::negate(apply_rotmat(inv_rotation, translation));
    return detail::make_homo(inv_rotation, inv_translation);
}

template <typename T>
inline Sre<T> invert_sre(const Sre<T>& sre) {
    return convert_homo_to_sre(invert_homo(convert_sre_to_homo(sre)));
}

template <typename T>
inline Lre<T> invert_lre(const Lre<T>& lre) {
    return convert_homo_to_lre(invert_homo(convert_lre_to_homo(lre)));
}

template <typename T>
inline Homo<T> convert_srq_to_homo(const Srq<T>& srq) {
    const auto rotation = convert_redquat_to_rotmat(detail::tail3(srq));
    const auto translation = apply_rotmat(rotation, detail::head3(srq));
    return detail::make_homo(rotation, translation);
}

template <typename T>
inline Homo<T> convert_lrq_to_homo(const Lrq<T>& lrq) {
    const auto rotation = convert_redquat_to_rotmat(detail::tail3(lrq));
    const auto translation = apply_rotmat(rotation, detail::negate(detail::head3(lrq)));
    return detail::make_homo(rotation, translation);
}

template <typename T>
inline Homo<T> convert_lrQ_to_homo(const LrQ<T>& lrQ) {
    const auto rotation = convert_quat_to_rotmat(detail::tail4(lrQ));
    const auto translation = apply_rotmat(rotation, detail::negate(detail::head3(lrQ)));
    return detail::make_homo(rotation, translation);
}

template <typename T>
inline Homo<T> convert_sre_to_homo(const Sre<T>& sre) {
    const auto rotation = convert_euler_to_rotmat(detail::tail3(sre));
    const auto translation = apply_rotmat(rotation, detail::head3(sre));
    return detail::make_homo(rotation, translation);
}

template <typename T>
inline Homo<T> convert_lre_to_homo(const Lre<T>& lre) {
    const auto rotation = convert_euler_to_rotmat(detail::tail3(lre));
    const auto translation = apply_rotmat(rotation, detail::negate(detail::head3(lre)));
    return detail::make_homo(rotation, translation);
}

// Homo-to-pose conversions assume matrix encodes an SE(3) rigid transform.
template <typename T>
inline Srq<T> convert_homo_to_srq(const Homo<T>& matrix) {
    const auto rotation = detail::rotmat_from_homo(matrix);
    const auto quat = convert_rotmat_to_quat(rotation);
    const auto rq = convert_quat_to_redquat(quat);
    const auto shift = apply_redquat(invert_redquat(rq), detail::translation_from_homo(matrix));
    return {shift[0], shift[1], shift[2], rq[0], rq[1], rq[2]};
}

template <typename T>
inline Lrq<T> convert_homo_to_lrq(const Homo<T>& matrix) {
    const auto rotation = detail::rotmat_from_homo(matrix);
    const auto quat = convert_rotmat_to_quat(rotation);
    const auto rq = convert_quat_to_redquat(quat);
    const auto location = detail::negate(apply_redquat(invert_redquat(rq), detail::translation_from_homo(matrix)));
    return {location[0], location[1], location[2], rq[0], rq[1], rq[2]};
}

template <typename T>
inline LrQ<T> convert_homo_to_lrQ(const Homo<T>& matrix) {
    const auto rotation = detail::rotmat_from_homo(matrix);
    const auto quat = convert_rotmat_to_quat(rotation);
    const auto location = detail::negate(apply_quat(invert_quat(quat), detail::translation_from_homo(matrix)));
    return {location[0], location[1], location[2], quat[0], quat[1], quat[2], quat[3]};
}

template <typename T>
inline Sre<T> convert_homo_to_sre(const Homo<T>& matrix) {
    const auto rotation = detail::rotmat_from_homo(matrix);
    const auto euler = convert_rotmat_to_euler(rotation);
    const auto shift = apply_rotmat(invert_rotmat(rotation), detail::translation_from_homo(matrix));
    return {shift[0], shift[1], shift[2], euler[0], euler[1], euler[2]};
}

template <typename T>
inline Lre<T> convert_homo_to_lre(const Homo<T>& matrix) {
    const auto rotation = detail::rotmat_from_homo(matrix);
    const auto euler = convert_rotmat_to_euler(rotation);
    const auto location = detail::negate(apply_rotmat(invert_rotmat(rotation), detail::translation_from_homo(matrix)));
    return {location[0], location[1], location[2], euler[0], euler[1], euler[2]};
}

template <typename T>
inline Homo<T> compose_homo(const Homo<T>& matrix1, const Homo<T>& matrix2) {
    return detail::multiply_homo(matrix2, matrix1);
}

template <typename T>
inline Srq<T> compose_srq(const Srq<T>& srq1, const Srq<T>& srq2) {
    const auto rq1 = detail::tail3(srq1);
    const auto rq = compose_redquat(rq1, detail::tail3(srq2));
    const auto shift = detail::add(detail::head3(srq1), apply_redquat(invert_redquat(rq1), detail::head3(srq2)));
    return {shift[0], shift[1], shift[2], rq[0], rq[1], rq[2]};
}

template <typename T>
inline LrQ<T> compose_lrQ(const LrQ<T>& lrQ1, const LrQ<T>& lrQ2) {
    const auto q1 = detail::tail4(lrQ1);
    const auto location = detail::add(detail::head3(lrQ1), apply_inverse_quat(q1, detail::head3(lrQ2)));
    const auto q = compose_quat(q1, detail::tail4(lrQ2));
    return {location[0], location[1], location[2], q[0], q[1], q[2], q[3]};
}

template <typename T>
inline Lrq<T> compose_lrq(const Lrq<T>& lrq1, const Lrq<T>& lrq2) {
    const auto rq1 = detail::tail3(lrq1);
    const auto location = detail::add(detail::head3(lrq1), apply_redquat(invert_redquat(rq1), detail::head3(lrq2)));
    const auto rq = compose_redquat(rq1, detail::tail3(lrq2));
    return {location[0], location[1], location[2], rq[0], rq[1], rq[2]};
}

template <typename T>
inline Lrrv<T> compose_lrrv(const Lrrv<T>& lrrv1, const Lrrv<T>& lrrv2) {
    const auto rv1 = detail::tail3(lrrv1);
    const auto location = detail::add(detail::head3(lrrv1), apply_rotvec(invert_rotvec(rv1), detail::head3(lrrv2)));
    const auto rv = compose_rotvec(rv1, detail::tail3(lrrv2));
    return {location[0], location[1], location[2], rv[0], rv[1], rv[2]};
}

template <typename T>
inline Sre<T> compose_sre(const Sre<T>& sre1, const Sre<T>& sre2) {
    return convert_homo_to_sre(compose_homo(convert_sre_to_homo(sre1), convert_sre_to_homo(sre2)));
}

template <typename T>
inline Lre<T> compose_lre(const Lre<T>& lre1, const Lre<T>& lre2) {
    return convert_homo_to_lre(compose_homo(convert_lre_to_homo(lre1), convert_lre_to_homo(lre2)));
}

}  // namespace robotransforms::euclidean
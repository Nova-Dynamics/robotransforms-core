#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>

namespace robotransforms::euclidean {

template <typename T>
using Vec3 = std::array<T, 3>;

template <typename T>
using Quat = std::array<T, 4>;

template <typename T>
using RedQuat = std::array<T, 3>;

template <typename T>
using RotVec = std::array<T, 3>;

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
inline T safe_nonnegative(T value) {
    if constexpr (std::is_arithmetic_v<T>) {
        return value < T(0) ? T(0) : value;
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

template <typename T>
inline Quat<T> tail4(const LrQ<T>& value) {
    return {value[3], value[4], value[5], value[6]};
}

template <typename T>
inline RedQuat<T> tail3_redquat(const Lrq<T>& value) {
    return {value[3], value[4], value[5]};
}

template <typename T>
inline RotVec<T> tail3_rotvec(const Lrrv<T>& value) {
    return {value[3], value[4], value[5]};
}

}  // namespace detail

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
inline LrQ<T> convert_lrq_to_lrQ(const Lrq<T>& lrq) {
    const auto q = convert_redquat_to_quat(detail::tail3_redquat(lrq));
    return {lrq[0], lrq[1], lrq[2], q[0], q[1], q[2], q[3]};
}

template <typename T>
inline Lrq<T> convert_lrQ_to_lrq(const LrQ<T>& lrQ) {
    const auto rq = convert_quat_to_redquat(detail::tail4(lrQ));
    return {lrQ[0], lrQ[1], lrQ[2], rq[0], rq[1], rq[2]};
}

template <typename T>
inline LrQ<T> convert_lrrv_to_lrQ(const Lrrv<T>& lrrv) {
    const auto q = convert_rotvec_to_quat(detail::tail3_rotvec(lrrv));
    return {lrrv[0], lrrv[1], lrrv[2], q[0], q[1], q[2], q[3]};
}

template <typename T>
inline Lrrv<T> convert_lrQ_to_lrrv(const LrQ<T>& lrQ) {
    const auto rv = convert_quat_to_rotvec(detail::tail4(lrQ));
    return {lrQ[0], lrQ[1], lrQ[2], rv[0], rv[1], rv[2]};
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

// Location-based transforms follow the JS implementation and the documented
// coordinate-transform convention: apply_lr*(T, v) = R(v - location).
template <typename T>
inline Vec3<T> apply_lrQ(const LrQ<T>& lrQ, const Vec3<T>& v) {
    return apply_quat(detail::tail4(lrQ), detail::subtract(v, detail::head3(lrQ)));
}

template <typename T>
inline Vec3<T> apply_lrq(const Lrq<T>& lrq, const Vec3<T>& v) {
    return apply_redquat(detail::tail3_redquat(lrq), detail::subtract(v, detail::head3(lrq)));
}

template <typename T>
inline Vec3<T> apply_lrrv(const Lrrv<T>& lrrv, const Vec3<T>& v) {
    return apply_rotvec(detail::tail3_rotvec(lrrv), detail::subtract(v, detail::head3(lrrv)));
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
    const auto rq = detail::tail3_redquat(lrq);
    const auto location = detail::negate(apply_redquat(rq, detail::head3(lrq)));
    const auto rq_inv = invert_redquat(rq);
    return {location[0], location[1], location[2], rq_inv[0], rq_inv[1], rq_inv[2]};
}

template <typename T>
inline Lrrv<T> invert_lrrv(const Lrrv<T>& lrrv) {
    const auto rv = detail::tail3_rotvec(lrrv);
    const auto location = detail::negate(apply_rotvec(rv, detail::head3(lrrv)));
    const auto rv_inv = invert_rotvec(rv);
    return {location[0], location[1], location[2], rv_inv[0], rv_inv[1], rv_inv[2]};
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
    const auto rq1 = detail::tail3_redquat(lrq1);
    const auto location = detail::add(detail::head3(lrq1), apply_redquat(invert_redquat(rq1), detail::head3(lrq2)));
    const auto rq = compose_redquat(rq1, detail::tail3_redquat(lrq2));
    return {location[0], location[1], location[2], rq[0], rq[1], rq[2]};
}

template <typename T>
inline Lrrv<T> compose_lrrv(const Lrrv<T>& lrrv1, const Lrrv<T>& lrrv2) {
    const auto rv1 = detail::tail3_rotvec(lrrv1);
    const auto location = detail::add(detail::head3(lrrv1), apply_rotvec(invert_rotvec(rv1), detail::head3(lrrv2)));
    const auto rv = compose_rotvec(rv1, detail::tail3_rotvec(lrrv2));
    return {location[0], location[1], location[2], rv[0], rv[1], rv[2]};
}

}  // namespace robotransforms::euclidean
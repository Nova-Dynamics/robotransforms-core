#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "robotransforms/euclidean.hpp"
#include "robotransforms/version.hpp"

namespace rt = robotransforms::euclidean;

namespace {

bool approx(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

template <std::size_t N>
bool approx_array(const std::array<double, N>& a, const std::array<double, N>& b, double eps = 1e-9) {
    for (std::size_t i = 0; i < N; ++i) {
        if (!approx(a[i], b[i], eps)) {
            return false;
        }
    }
    return true;
}

template <std::size_t N>
void expect_array(const std::string& name, const std::array<double, N>& actual, const std::array<double, N>& expected, double eps = 1e-9) {
    if (approx_array(actual, expected, eps)) {
        return;
    }

    std::cerr << name << " failed\nexpected: [";
    for (std::size_t i = 0; i < N; ++i) {
        std::cerr << expected[i] << (i + 1 == N ? "" : ", ");
    }
    std::cerr << "]\nactual:   [";
    for (std::size_t i = 0; i < N; ++i) {
        std::cerr << actual[i] << (i + 1 == N ? "" : ", ");
    }
    std::cerr << "]\n";
    std::exit(1);
}

void test_lrQ_application_uses_location_subtraction() {
    const rt::LrQ<double> lrQ{1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};
    const rt::Vec3<double> v{10.0, 20.0, 30.0};
    expect_array("apply_lrQ subtracts location", rt::apply_lrQ(lrQ, v), rt::Vec3<double>{9.0, 18.0, 27.0});
}

void test_lrrv_matches_lrQ_conversion() {
    const rt::Lrrv<double> lrrv{1.0, -2.0, 0.5, 0.2, -0.1, 0.7};
    const rt::Vec3<double> v{3.0, 4.0, -1.0};
    const auto lrQ = rt::convert_lrrv_to_lrQ(lrrv);
    expect_array("apply_lrrv matches converted lrQ", rt::apply_lrrv(lrrv, v), rt::apply_lrQ(lrQ, v), 1e-8);
}

void test_inverse_round_trip() {
    const rt::LrQ<double> lrQ{0.5, -1.0, 2.0, 0.9238795325, 0.0, 0.0, 0.3826834324};
    const rt::Vec3<double> v{4.0, 1.5, -2.0};
    const auto inv = rt::invert_lrQ(lrQ);
    expect_array("invert/apply round trip", rt::apply_lrQ(inv, rt::apply_lrQ(lrQ, v)), v, 1e-8);
}

void test_compose_with_inverse_is_identity() {
    const rt::LrQ<double> lrQ{-0.25, 1.2, 3.4, 0.8660254038, 0.0, 0.5, 0.0};
    const auto inv = rt::invert_lrQ(lrQ);
    const auto identity = rt::compose_lrQ(lrQ, inv);
    expect_array("compose with inverse location", identity, rt::LrQ<double>{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}, 1e-8);
}

void test_quat_rotvec_round_trip() {
    const rt::Quat<double> q{0.9238795325, 0.0, 0.3826834324, 0.0};
    const auto rv = rt::convert_quat_to_rotvec(q);
    const auto q_round_trip = rt::convert_rotvec_to_quat(rv);
    expect_array("quat rotvec round trip", q_round_trip, q, 1e-8);
}

void test_lrq_conversion_matches_application() {
    const rt::Lrq<double> lrq{1.0, 2.0, 3.0, 0.0, 0.0, 0.7071067812};
    const rt::Vec3<double> v{5.0, -1.0, 2.0};
    const auto lrQ = rt::convert_lrq_to_lrQ(lrq);
    expect_array("apply_lrq matches converted lrQ", rt::apply_lrq(lrq, v), rt::apply_lrQ(lrQ, v), 1e-8);
}

void test_version_constants() {
    if (robotransforms::version_major != 0 ||
        robotransforms::version_minor != 1 ||
        robotransforms::version_patch != 0 ||
        robotransforms::version_string != "0.1.0") {
        std::cerr << "version constants failed\n";
        std::exit(1);
    }
}

}  // namespace

int main() {
    test_lrQ_application_uses_location_subtraction();
    test_lrrv_matches_lrQ_conversion();
    test_inverse_round_trip();
    test_compose_with_inverse_is_identity();
    test_quat_rotvec_round_trip();
    test_lrq_conversion_matches_application();
    test_version_constants();
    std::cout << "euclidean tests passed\n";
    return 0;
}
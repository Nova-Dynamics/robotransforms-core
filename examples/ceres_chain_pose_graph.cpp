#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <ceres/product_manifold.h>

#include "robotransforms/euclidean.hpp"

namespace rt = robotransforms::euclidean;

namespace {

using Pose = rt::LrQ<double>;
using Vec3 = rt::Vec3<double>;
using Mat3 = rt::RotMat<double>;

template <typename T>
rt::LrQ<T> make_pose(const T* raw) {
    return {raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6]};
}

template <typename T>
rt::Quat<T> pose_quat(const rt::LrQ<T>& pose) {
    return {pose[3], pose[4], pose[5], pose[6]};
}

template <typename T>
rt::Vec3<T> pose_translation(const rt::LrQ<T>& pose) {
    return {pose[0], pose[1], pose[2]};
}

template <typename T>
rt::LrQ<T> cast_pose(const Pose& pose) {
    return {
        T(pose[0]), T(pose[1]), T(pose[2]),
        T(pose[3]), T(pose[4]), T(pose[5]), T(pose[6]),
    };
}

template <typename T>
rt::Vec3<T> apply_upper_triangular(const Mat3& upper, const rt::Vec3<T>& value) {
    return {
        T(upper[0][0]) * value[0] + T(upper[0][1]) * value[1] + T(upper[0][2]) * value[2],
        T(upper[1][1]) * value[1] + T(upper[1][2]) * value[2],
        T(upper[2][2]) * value[2],
    };
}

Mat3 diagonal_covariance(const Vec3& diagonal) {
    return {{
        {diagonal[0], 0.0, 0.0},
        {0.0, diagonal[1], 0.0},
        {0.0, 0.0, diagonal[2]},
    }};
}

Mat3 inverse_3x3(const Mat3& matrix) {
    const double a = matrix[0][0];
    const double b = matrix[0][1];
    const double c = matrix[0][2];
    const double d = matrix[1][0];
    const double e = matrix[1][1];
    const double f = matrix[1][2];
    const double g = matrix[2][0];
    const double h = matrix[2][1];
    const double i = matrix[2][2];

    const double det =
        a * (e * i - f * h) -
        b * (d * i - f * g) +
        c * (d * h - e * g);

    if (std::abs(det) < 1e-15) {
        throw std::runtime_error("Covariance matrix is singular.");
    }

    const double inv_det = 1.0 / det;
    return {{
        {(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det},
        {(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det},
        {(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det},
    }};
}

Mat3 cholesky_upper_3x3(const Mat3& matrix) {
    Mat3 upper{{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
    }};

    upper[0][0] = std::sqrt(std::max(matrix[0][0], 0.0));
    upper[0][1] = matrix[0][1] / upper[0][0];
    upper[0][2] = matrix[0][2] / upper[0][0];

    const double d11 = matrix[1][1] - upper[0][1] * upper[0][1];
    upper[1][1] = std::sqrt(std::max(d11, 0.0));
    upper[1][2] = (matrix[1][2] - upper[0][1] * upper[0][2]) / upper[1][1];

    const double d22 = matrix[2][2] - upper[0][2] * upper[0][2] - upper[1][2] * upper[1][2];
    upper[2][2] = std::sqrt(std::max(d22, 0.0));

    return upper;
}

Mat3 sqrt_information_from_covariance(const Mat3& covariance) {
    return cholesky_upper_3x3(inverse_3x3(covariance));
}

struct GpsMeasurementInput {
    std::size_t node_idx;
    Pose lrQ_node_to_antenna;
    Vec3 antenna_xyz;
    Mat3 antenna_cov;
};

struct RelativeMeasurement {
    std::size_t from_idx;
    std::size_t to_idx;
    Pose measured_lrQ;
    std::array<double, 6> inv_std;
};

struct GpsMeasurement {
    std::size_t node_idx;
    Pose lrQ_node_to_antenna;
    Vec3 antenna_xyz;
    Mat3 sqrt_information;
};

struct SolverOptions {
    bool anchor_first_node = true;
    ceres::Solver::Options ceres_options;

    SolverOptions() {
        ceres_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        ceres_options.max_num_iterations = 50;
        ceres_options.minimizer_progress_to_stdout = true;
    }
};

struct SolveResult {
    std::vector<Pose> node_poses;
    ceres::Solver::Summary summary;
};

RelativeMeasurement make_chain_measurement(
    std::size_t from_idx,
    const rt::Lrq<double>& measured_lrq,
    const std::array<double, 6>& variance_diag) {
    RelativeMeasurement measurement{};
    measurement.from_idx = from_idx;
    measurement.to_idx = from_idx + 1;
    measurement.measured_lrQ = rt::convert_lrq_to_lrQ(measured_lrq);
    for (std::size_t i = 0; i < variance_diag.size(); ++i) {
        measurement.inv_std[i] = 1.0 / std::sqrt(std::max(variance_diag[i], 1e-15));
    }
    return measurement;
}

GpsMeasurement make_gps_measurement(const GpsMeasurementInput& input) {
    return {
        input.node_idx,
        input.lrQ_node_to_antenna,
        input.antenna_xyz,
        sqrt_information_from_covariance(input.antenna_cov),
    };
}

std::vector<Pose> initialize_chain(const std::vector<rt::Lrq<double>>& adjacent_lrq) {
    std::vector<Pose> node_poses(adjacent_lrq.size() + 1, Pose{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
    for (std::size_t i = 0; i < adjacent_lrq.size(); ++i) {
        node_poses[i + 1] = rt::compose_lrQ(node_poses[i], rt::convert_lrq_to_lrQ(adjacent_lrq[i]));
    }
    return node_poses;
}

struct RelativePoseResidual {
    explicit RelativePoseResidual(RelativeMeasurement measurement)
        : measurement_(std::move(measurement)) {}

    template <typename T>
    bool operator()(const T* const from_raw, const T* const to_raw, T* residuals) const {
        const auto from_pose = make_pose(from_raw);
        const auto to_pose = make_pose(to_raw);
        const auto predicted_delta = rt::compose_lrQ(rt::invert_lrQ(from_pose), to_pose);
        const auto measured_delta = cast_pose<T>(measurement_.measured_lrQ);

        residuals[0] = T(measurement_.inv_std[0]) * (predicted_delta[0] - measured_delta[0]);
        residuals[1] = T(measurement_.inv_std[1]) * (predicted_delta[1] - measured_delta[1]);
        residuals[2] = T(measurement_.inv_std[2]) * (predicted_delta[2] - measured_delta[2]);

        const auto q_error = rt::compose_quat(
            rt::invert_quat(pose_quat(measured_delta)),
            pose_quat(predicted_delta)
        );

        residuals[3] = T(measurement_.inv_std[3]) * (T(2) * q_error[1]);
        residuals[4] = T(measurement_.inv_std[4]) * (T(2) * q_error[2]);
        residuals[5] = T(measurement_.inv_std[5]) * (T(2) * q_error[3]);
        return true;
    }

private:
    RelativeMeasurement measurement_;
};

struct GpsResidual {
    explicit GpsResidual(GpsMeasurement measurement)
        : measurement_(std::move(measurement)) {}

    template <typename T>
    bool operator()(const T* const node_pose_raw, T* residuals) const {
        const auto node_pose = make_pose(node_pose_raw);
        const auto node_to_antenna = cast_pose<T>(measurement_.lrQ_node_to_antenna);
        const auto world_to_antenna = rt::compose_lrQ(node_pose, node_to_antenna);
        const auto error = rt::Vec3<T>{
            world_to_antenna[0] - T(measurement_.antenna_xyz[0]),
            world_to_antenna[1] - T(measurement_.antenna_xyz[1]),
            world_to_antenna[2] - T(measurement_.antenna_xyz[2]),
        };
        const auto weighted_error = apply_upper_triangular(measurement_.sqrt_information, error);
        residuals[0] = weighted_error[0];
        residuals[1] = weighted_error[1];
        residuals[2] = weighted_error[2];
        return true;
    }

private:
    GpsMeasurement measurement_;
};

SolveResult solve_chain_pose_graph(
    const std::vector<rt::Lrq<double>>& adjacent_lrq,
    const std::array<double, 6>& adjacent_variance_diag,
    const std::vector<GpsMeasurementInput>& gps_inputs,
    SolverOptions options = {}) {
    auto node_poses = initialize_chain(adjacent_lrq);

    ceres::Problem problem;
    using PoseManifold = ceres::ProductManifold<ceres::EuclideanManifold<3>, ceres::QuaternionManifold>;

    for (auto& pose : node_poses) {
        problem.AddParameterBlock(pose.data(), 7);
        problem.SetManifold(pose.data(), new PoseManifold());
    }

    if (options.anchor_first_node && !node_poses.empty()) {
        problem.SetParameterBlockConstant(node_poses.front().data());
    }

    for (std::size_t i = 0; i < adjacent_lrq.size(); ++i) {
        auto* cost = new ceres::AutoDiffCostFunction<RelativePoseResidual, 6, 7, 7>(
            new RelativePoseResidual(make_chain_measurement(i, adjacent_lrq[i], adjacent_variance_diag))
        );
        problem.AddResidualBlock(cost, nullptr, node_poses[i].data(), node_poses[i + 1].data());
    }

    for (const auto& gps_input : gps_inputs) {
        if (gps_input.node_idx >= node_poses.size()) {
            throw std::out_of_range("GPS measurement node_idx is outside the node pose array.");
        }

        auto* cost = new ceres::AutoDiffCostFunction<GpsResidual, 3, 7>(
            new GpsResidual(make_gps_measurement(gps_input))
        );
        problem.AddResidualBlock(cost, nullptr, node_poses[gps_input.node_idx].data());
    }

    SolveResult result{};
    result.node_poses = node_poses;
    ceres::Solve(options.ceres_options, &problem, &result.summary);
    return result;
}

void print_pose(const std::string& label, const Pose& pose) {
    std::cout << label << ": ["
              << pose[0] << ", " << pose[1] << ", " << pose[2] << ", "
              << pose[3] << ", " << pose[4] << ", " << pose[5] << ", " << pose[6]
              << "]\n";
}

}  // namespace

int main() {
    const std::vector<rt::Lrq<double>> adjacent_lrq = {
        {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    };

    const std::array<double, 6> adjacent_variance_diag = {
        0.01, 0.1, 0.01, 0.01, 0.01, 0.01,
    };

    const std::vector<GpsMeasurementInput> gps_inputs = {
        {
            0,
            Pose{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
            Vec3{0.0, 0.0, 0.0},
            diagonal_covariance(Vec3{0.25, 0.25, 0.25}),
        },
        {
            3,
            Pose{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
            Vec3{3.05, 0.02, 0.0},
            diagonal_covariance(Vec3{0.25, 0.25, 0.25}),
        },
    };

    SolverOptions options;
    const auto result = solve_chain_pose_graph(adjacent_lrq, adjacent_variance_diag, gps_inputs, options);

    std::cout << result.summary.BriefReport() << "\n";
    for (std::size_t i = 0; i < result.node_poses.size(); ++i) {
        print_pose("node_" + std::to_string(i), result.node_poses[i]);
    }

    return result.summary.IsSolutionUsable() ? 0 : 1;
}
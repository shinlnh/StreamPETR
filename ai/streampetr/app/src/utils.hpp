#ifndef STREAMPETR_UTILS_HPP
#define STREAMPETR_UTILS_HPP

#include <Eigen/Dense>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <vector>

namespace Eigen
{
    using MatrixXdRM = Matrix<double, Dynamic, Dynamic, RowMajor>;
    using Matrix3dRM = Matrix<double, 3, 3, RowMajor>;
    using Matrix4dRM = Matrix<double, 4, 4, RowMajor>;
    using MatrixXfRM = Matrix<float, Dynamic, Dynamic, RowMajor>;
    using Matrix3fRM = Matrix<float, 3, 3, RowMajor>;
    using Matrix4fRM = Matrix<float, 4, 4, RowMajor>;
};  // namespace Eigen


template <std::size_t size>
inline std::array<float, size> to_float_array(const std::vector<double>& double_vector)
{
    std::array<float, size> float_array;
    if (double_vector.size() != size) {
        throw std::runtime_error("Vector and array differ in size!!!")
    }
    std::transform(
        double_vector.begin(), double_vector.begin() + Size, float_array.begin(),
        [](double val) { return static_cast<float>(val); }
    );
    return float_array;
}

Eigen::Matrix3fRM getEigenCameraIntrinsics(
    sensor_msgs::msg::CameraInfo::ConstSharedPtr msg
);

Eigen::Matrix4fRM getEigenTransformMatrix(
    const geometry_msgs::msg::TransformStamped &transform
);

Eigen::MatrixXfRM inverse_sigmoid(Eigen::MatrixXfRM x);

Eigen::MatrixXfRM load_npy_matrix(const std::string& file_path, int rows, int cols);

Eigen::Matrix4dRM invert_transform_matrix(const Eigen::Matrix4dRM& mat);

#endif // STREAMPETR_UTILS_HPP
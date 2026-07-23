#ifndef STREAMPETR_UTILS_HPP
#define STREAMPETR_UTILS_HPP

#include <Eigen/Dense>

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

Eigen::MatrixXfRM inverse_sigmoid(Eigen::MatrixXfRM x);

Eigen::MatrixXfRM load_npy_matrix(const std::string& file_path, int rows, int cols);

Eigen::Matrix4dRM invert_transform_matrix(const Eigen::Matrix4dRM& mat);

#endif // STREAMPETR_UTILS_HPP
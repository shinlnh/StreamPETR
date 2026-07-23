#include "utils.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

#include <fstream>

Eigen::Matrix3fRM getEigenCameraIntrinsics(
    sensor_msgs::msg::CameraInfo::ConstSharedPtr msg
)
{
    Eigen::Matrix3fRM intrinsics;
    intrinsics << 
        msg->k[0], msg->k[1], msg->k[2],
        msg->k[3], msg->k[4], msg->k[5],
        msg->k[6], msg->k[7], msg->k[8];

    return intrinsics;
}


Eigen::Matrix4fRM getEigenTransformMatrix(
    const geometry_msgs::msg::TransformStamped &transform
)
{
    Eigen::Isometry3d iso_mat = tf2::transformToEigen(transform);
    Eigen::Matrix4fRM transform_mat = iso_mat.matrix().cast<float>();

    return transform_mat;
}

// Calculate log(x/(1-x))
Eigen::MatrixXfRM inverse_sigmoid(Eigen::MatrixXfRM x)
{
    // Clamp to avoid NAN and infinity
    static float eps = 1e-7f;
    auto clamped = x.array().max(eps).min(1.0f - eps);
    return (clamped / (1.0f - clamped)).log().matrix();
}

// Function to load numpy float32 matrix data into an Eigen Matrix
Eigen::MatrixXfRM load_npy_matrix(const std::string& file_path, int rows, int cols)
{
    // 1. Open file in binary mode at the end to get size
    std::ifstream file(file_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open: " + file_path);
    }

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    // 2. Validate size (rows * cols * 4 bytes for float32)
    if (size != (std::streamsize)(rows * cols * sizeof(float))) {
        throw std::runtime_error("File size mismatch! Check your dimensions.");
    }

    // 3. Read data into a buffer
    Eigen::MatrixXfRM buffer(rows, cols);
    if (!file.read(reinterpret_cast<char*>(buffer.data()), size)) {
        throw std::runtime_error("Error reading file data.");
    }

    return buffer; 
}

Eigen::Matrix4dRM invert_transform_matrix(const Eigen::Matrix4dRM& mat)
{
    Eigen::Matrix4dRM inv_mat;
    
    // 1. Extract and Transpose the rotation part
    Eigen::Matrix3dRM R_T = mat.block<3, 3>(0, 0).transpose();
    
    // 2. Compute the new translation: -R_T * t
    Eigen::Vector3d T_inv = -R_T * mat.block<3, 1>(0, 3);
    
    // 3. Assemble
    inv_mat.block<3, 3>(0, 0) = R_T;
    inv_mat.block<3, 1>(0, 3) = T_inv;
    inv_mat.row(3) << 0, 0, 0, 1;
    
    return inv_mat;
}
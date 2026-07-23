#ifndef GENERAL_MATHS_H
#define GENERAL_MATHS_H

#include <array>
#include <vector>
#include <math.h>


template <typename T, std::size_t N>
static inline std::array<T, N> to_array(const std::vector<T>& vec) {
    if (vec.size() != N) {
        throw std::runtime_error("Vector size does not match array size");
    }
 
    std::array<T, N> result;
    std::copy(vec.begin(), vec.end(), result.begin()); // Copy elements using std::copy
 
    return result;
}

template <typename T, std::size_t N>
static inline std::vector<std::array<T, N>> to_array_vector(const std::vector<std::vector<T>>& vec_of_vecs) {
    std::vector<std::array<T, N>> result;
    result.reserve(vec_of_vecs.size()); // Reserve space for potential efficiency

    for (const std::vector<T>& inner_vec : vec_of_vecs) {
        if (inner_vec.size() != N) {
            throw std::runtime_error("Inner vector size does not match array size");
        }
        result.push_back(to_array<T, N>(inner_vec)); // Use to_array for each inner vector
    }

    return result;
}

template <typename T>
static inline std::vector<std::array<T, 2>> to_arrayof2_vector(const std::vector<std::vector<T>>& vec_of_vecs) {
    std::vector<std::array<T, 2>> result;
    result.reserve(vec_of_vecs.size()); // Reserve space for potential efficiency

    for (const std::vector<T>& inner_vec : vec_of_vecs) {
        if (inner_vec.size() < 2) {
            throw std::runtime_error("Inner vector size is less than 2");
        }
        std::array<T, 2> array = {inner_vec[0], inner_vec[1]};
        result.push_back(array); // Push the first two elements into the array
    }

    return result;
}

template <typename T>
static inline std::vector<std::array<T, 3>> to_arrayof3_vector(const std::vector<std::vector<T>>& vec_of_vecs) {
    std::vector<std::array<T, 3>> result;
    result.reserve(vec_of_vecs.size()); // Reserve space for potential efficiency

    for (const std::vector<T>& inner_vec : vec_of_vecs) {
        if (inner_vec.size() < 3) {
            throw std::runtime_error("Inner vector size is less than 3");
        }
        std::array<T, 3> array = {inner_vec[0], inner_vec[1], inner_vec[2]};
        result.push_back(array); // Push the first three elements into the array
    }

    return result;
}

template <typename T, std::size_t N>
static inline std::vector<T> to_vector(const std::array<T, N>& arr) {
    std::vector<T> result;
    result.reserve(N);  // Reserve space to avoid unnecessary reallocations

    // Copy elements from the array to the vector
    std::copy(arr.begin(), arr.end(), std::back_inserter(result));

    return result;
}

template <typename T, std::size_t N>
static inline std::vector<std::vector<T>> to_vector_vector(const std::vector<std::array<T, N>>& vec_of_arrays) {
    std::vector<std::vector<T>> result;
    result.reserve(vec_of_arrays.size()); // Reserve space for potential efficiency

    for (const std::array<T, N>& inner_array : vec_of_arrays) {
        result.push_back(to_vector(inner_array)); // Use to_vector for each inner array
    }

    return result;
}

/**
 * @brief Find overlap between 2 ranges. Suppose you have 2 ranges [-0.4 0.4] and [0 1]
 * This function should return 0.4.
 * 
 * @param range_start   The reference range start
 * @param range_end     The reference range end
 * @param check_start   The start of the range you want to check
 * @param check_end     The end of the range you want to check
 * @return double 
 */
static inline double getOverlap(float range_start, float range_end, 
                                float check_start, float check_end) {
    double overlap = std::min(range_end, check_end) - std::max(range_start, check_start);
    return std::max(0.0, overlap);
}


template <typename T>
static inline T square(T x) { return x * x; }

#endif /* GENERAL_MATHS_H */

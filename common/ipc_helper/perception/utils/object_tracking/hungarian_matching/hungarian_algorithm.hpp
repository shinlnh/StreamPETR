#ifndef HUNGARIAN_ALGORITHM_HPP
#define HUNGARIAN_ALGORITHM_HPP

#include <iostream>
#include <vector>
#include <type_traits>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <string>

namespace Munkres {

// helper to clear temporary vectors
inline void clear_covers(std::vector<int>& cover) {
    for (auto& n: cover) n = 0;
}

template<template <typename, typename...> class Container,
         typename T,
         typename... Args>
typename std::enable_if<!std::is_convertible<Container<T, Args...>, std::string>::value &&
                        !std::is_constructible<Container<T, Args...>, std::string>::value,
                        std::ostream&>::type
operator<<(std::ostream& os, const Container<T, Args...>& con)
{
    os << "\t";
    for (auto& elem: con)
        os << elem << "\t";
    os << "\n";
    return os;
}

template<typename T>
void handle_negatives(std::vector<std::vector<T>>& matrix, bool allowed = true)
{
    T minval = std::numeric_limits<T>::max();

    for (auto& elem: matrix)
        for (auto& num: elem)
            minval = std::min(minval, num);

    if (minval < 0) {
        if (!allowed) {
            throw std::runtime_error("Only non-negative values allowed");
        }
        else {
            minval = abs(minval);
            for (auto& elem: matrix)
                for (auto& num: elem)
                    num += minval;
        }
    }
}

template<typename T>
void pad_matrix(std::vector<std::vector<T>>& matrix)
{
    std::size_t i_size = matrix.size();
    std::size_t j_size = matrix[0].size();

    if (i_size > j_size) {
        for (auto& vec: matrix)
            vec.resize(i_size, std::numeric_limits<T>::max());
    }
    else if (i_size < j_size) {
        while (matrix.size() < j_size)
            matrix.push_back(std::vector<T>(j_size, std::numeric_limits<T>::max()));
    }
}

template<typename T>
void step1(std::vector<std::vector<T>>& matrix, int& step)
{
    for (auto& row: matrix) {
        auto smallest = *std::min_element(begin(row), end(row));
        if (smallest > 0)
            for (auto& n: row)
                n -= smallest;
    }

    int sz = matrix.size();
    for (int j=0; j<sz; ++j) {
        T minval = std::numeric_limits<T>::max();
        for (int i=0; i<sz; ++i) {
            minval = std::min(minval, matrix[i][j]);
        }
        if (minval > 0) {
            for (int i=0; i<sz; ++i) {
                matrix[i][j] -= minval;
            }
        }
    }

    step = 2;
}

template<typename T>
void step2(const std::vector<std::vector<T>>& matrix,
           std::vector<std::vector<int>>& M,
           std::vector<int>& RowCover,
           std::vector<int>& ColCover,
           int& step)
{
    int sz = matrix.size();

    for (int r=0; r<sz; ++r)
        for (int c=0; c<sz; ++c)
            if (matrix[r][c] == 0)
                if (RowCover[r] == 0 && ColCover[c] == 0) {
                    M[r][c] = 1;
                    RowCover[r] = 1;
                    ColCover[c] = 1;
                }

    clear_covers(RowCover);
    clear_covers(ColCover);

    step = 3;
}

inline
void step3(const std::vector<std::vector<int>>& M,
           std::vector<int>& ColCover,
           int& step)
{
    int sz = M.size();
    int colcount = 0;

    for (int r=0; r<sz; ++r)
        for (int c=0; c<sz; ++c)
            if (M[r][c] == 1)
                ColCover[c] = 1;

    for (auto& n: ColCover)
        if (n == 1)
            colcount++;

    if (colcount >= sz) {
        step = 7; // solution found
    }
    else {
        step = 4;
    }
}

template<typename T>
void find_a_zero(int& row,
                 int& col,
                 const std::vector<std::vector<T>>& matrix,
                 const std::vector<int>& RowCover,
                 const std::vector<int>& ColCover)
{
    int r = 0;
    int c = 0;
    int sz = matrix.size();
    bool done = false;
    row = -1;
    col = -1;

    while (!done) {
        c = 0;
        while (true) {
            if (matrix[r][c] == 0 && RowCover[r] == 0 && ColCover[c] == 0) {
                row = r;
                col = c;
                done = true;
            }
            c += 1;
            if (c >= sz || done)
                break;
        }
        r += 1;
        if (r >= sz)
            done = true;
    }
}

inline
bool star_in_row(int row,
                 const std::vector<std::vector<int>>& M)
{
    for (unsigned c = 0; c < M.size(); c++)
        if (M[row][c] == 1)
            return true;
    return false;
}

inline
void find_star_in_row(int row,
                      int& col,
                      const std::vector<std::vector<int>>& M)
{
    col = -1;
    for (unsigned c = 0; c < M.size(); c++)
        if (M[row][c] == 1)
            col = c;
}

inline
void find_star_in_col(int c,
                      int& r,
                      const std::vector<std::vector<int>>& M)
{
    r = -1;
    for (unsigned i = 0; i < M.size(); i++)
        if (M[i][c] == 1)
            r = i;
}

inline
void find_prime_in_row(int r,
                       int& c,
                       const std::vector<std::vector<int>>& M)
{
    for (unsigned j = 0; j < M.size(); j++)
        if (M[r][j] == 2)
            c = j;
}

inline
void augment_path(std::vector<std::vector<int>>& path,
                  int path_count,
                  std::vector<std::vector<int>>& M)
{
    for (int p = 0; p < path_count; p++)
        if (M[path[p][0]][path[p][1]] == 1)
            M[path[p][0]][path[p][1]] = 0;
        else
            M[path[p][0]][path[p][1]] = 1;
}

inline
void erase_primes(std::vector<std::vector<int>>& M)
{
    for (auto& row: M)
        for (auto& val: row)
            if (val == 2)
                val = 0;
}

template<typename T>
void step4(const std::vector<std::vector<T>>& matrix,
           std::vector<std::vector<int>>& M,
           std::vector<int>& RowCover,
           std::vector<int>& ColCover,
           int& path_row_0,
           int& path_col_0,
           int& step)
{
    int row = -1;
    int col = -1;
    bool done = false;

    while (!done) {
        find_a_zero(row, col, matrix, RowCover, ColCover);

        if (row == -1) {
            done = true;
            step = 6;
        }
        else {
            M[row][col] = 2;
            if (star_in_row(row, M)) {
                find_star_in_row(row, col, M);
                RowCover[row] = 1;
                ColCover[col] = 0;
            }
            else {
                done = true;
                step = 5;
                path_row_0 = row;
                path_col_0 = col;
            }
        }
    }
}

inline
void step5(std::vector<std::vector<int>>& path,
           int path_row_0,
           int path_col_0,
           std::vector<std::vector<int>>& M,
           std::vector<int>& RowCover,
           std::vector<int>& ColCover,
           int& step)
{
    int r = -1;
    int c = -1;
    int path_count = 1;
    path[path_count - 1][0] = path_row_0;
    path[path_count - 1][1] = path_col_0;
    bool done = false;
    while (!done) {
        find_star_in_col(path[path_count - 1][1], r, M);
        if (r > -1) {
            path_count += 1;
            path[path_count - 1][0] = r;
            path[path_count - 1][1] = path[path_count - 2][1];
        }
        else {
            done = true;
        }
        if (!done) {
            find_prime_in_row(path[path_count - 1][0], c, M);
            path_count += 1;
            path[path_count - 1][0] = path[path_count - 2][0];
            path[path_count - 1][1] = c;
        }
    }

    augment_path(path, path_count, M);
    clear_covers(RowCover);
    clear_covers(ColCover);
    erase_primes(M);
    step = 3;
}

template<typename T>
void find_smallest(T& minval,
                   const std::vector<std::vector<T>>& matrix,
                   const std::vector<int>& RowCover,
                   const std::vector<int>& ColCover)
{
    for (unsigned r = 0; r < matrix.size(); r++)
        for (unsigned c = 0; c < matrix.size(); c++)
            if (RowCover[r] == 0 && ColCover[c] == 0)
                if (minval > matrix[r][c])
                    minval = matrix[r][c];
}

template<typename T>
void step6(std::vector<std::vector<T>>& matrix,
           const std::vector<int>& RowCover,
           const std::vector<int>& ColCover,
           int& step)
{
    T minval = std::numeric_limits<T>::max();
    find_smallest(minval, matrix, RowCover, ColCover);

    int sz = matrix.size();
    for (int r = 0; r < sz; r++)
        for (int c = 0; c < sz; c++) {
            if (RowCover[r] == 1)
                matrix[r][c] += minval;
            if (ColCover[c] == 0)
                matrix[r][c] -= minval;
        }

    step = 4;
}

template<template <typename, typename...> class Container,
         typename T,
         typename... Args>
T output_solution(const Container<Container<T, Args...>>& original,
                  const std::vector<std::vector<int>>& M)
{
    T res = 0;

    for (unsigned j = 0; j < original.begin()->size(); ++j)
        for (unsigned i = 0; i < original.size(); ++i)
            if (M[i][j]) {
                auto it1 = original.begin();
                std::advance(it1, i);
                auto it2 = it1->begin();
                std::advance(it2, j);
                res += *it2;
                continue;
            }

    return res;
}

template<template <typename, typename...> class Container,
         typename T,
         typename... Args>
typename std::enable_if<std::is_integral<T>::value, T>::type
hungarian(Container<Container<T, Args...>>& original,
          bool allow_negatives = true)
{
    std::vector<std::vector<T>> matrix(original.size(),
                                      std::vector<T>(original.begin()->size()));

    auto it = original.begin();
    for (auto& vec : matrix) {
        std::copy(it->begin(), it->end(), vec.begin());
        it = std::next(it);
    }

    if (!std::is_unsigned<T>::value) {
        handle_negatives(matrix, allow_negatives);
    }

    pad_matrix(matrix);
    std::size_t sz = matrix.size();

    std::vector<std::vector<int>> M(sz, std::vector<int>(sz, 0));
    std::vector<int> RowCover(sz, 0);
    std::vector<int> ColCover(sz, 0);

    int path_row_0, path_col_0;
    std::vector<std::vector<int>> path(sz * 2, std::vector<int>(2, 0));

    bool done = false;
    int step = 1;
    while (!done) {
        switch (step) {
        case 1:
            step1(matrix, step);
            break;
        case 2:
            step2(matrix, M, RowCover, ColCover, step);
            break;
        case 3:
            step3(M, ColCover, step);
            break;
        case 4:
            step4(matrix, M, RowCover, ColCover, path_row_0, path_col_0, step);
            break;
        case 5:
            step5(path, path_row_0, path_col_0, M, RowCover, ColCover, step);
            break;
        case 6:
            step6(matrix, RowCover, ColCover, step);
            break;
        case 7:
            for (auto& vec : M)
                vec.resize(original.begin()->size());
            M.resize(original.size());
            done = true;
            break;
        default:
            done = true;
            break;
        }
    }

    if (original.size() != M.size() || original.begin()->size() != M.begin()->size()) {
        throw std::range_error("Hungatian bugs - Different result M matrix size and original matrix size!");
    }

    for (size_t row = 0; row < M.size(); row++) {
        for (size_t col = 0; col < M.begin()->size(); col++) {
            original.at(row).at(col) = M[row][col];
        }
    }

    return output_solution(original, M);
}

} // end namespace Munkres

#endif // HUNGARIAN_ALGORITHM_HPP

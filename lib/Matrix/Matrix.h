#ifndef MATRIX_H
#define MATRIX_H

#include <initializer_list>

template <typename T>
class Matrix {
private:
    int nRows = 0;
    int nCols = 0;

    void swap(Matrix<T> &m);

public:
    T** matrix = nullptr; //public until overload subscript op

    Matrix(int _nRows, int _nCols, std::initializer_list<T> list);
    Matrix(int _nRows, int _nCols, T init_value);
    Matrix(int _nRows, int _nCols);
    Matrix(const Matrix<T> &m);
    ~Matrix();

    //int& operator[] (int index);

    void print(void);
    void print_detailed(void);

    void set(std::initializer_list<T> list);
    void set_row(int row, T values[]);
    void set_col(int col, T values[]);
    void set_diagonal(T values[]);

    void multiply(Matrix<T> m2);
    void transpose(void);

};
#endif

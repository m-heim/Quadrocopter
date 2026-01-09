#ifndef VEC_HPP
#define VEC_HPP
template<int i, int j>
class Matrix {
    public:
    double arr[j][i];
};
template<int i>
class Vec: private Matrix<i, 1>{
    public:
    Vec() {}
    double getElement(int index) {
        return arr[0][index];
    }
    void setElement(int index, double val) {
        this->arr[0][index] = val;
    }
};
#endif
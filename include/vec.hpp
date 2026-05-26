#ifndef VEC_HPP
#define VEC_HPP
template <typename T, int i, int j>
class Matrix
{
public:
    T arr[j][i];
};
template <typename T, int i>
class Vec : private Matrix<T, i, 1>
{
public:
    Vec() {}
    T getElement(int index)
    {
        return arr[0][index];
    }
    void setElement(int index, T val)
    {
        this->arr[0][index] = val;
    }
};
#endif
#ifndef MAT_H
#define MAT_H

#include <iostream>
#include <vector>

class Ma {
public:
    std::vector<std::vector<double>> vec;

    Ma() {}
    Ma(std::vector<std::vector<double>> v) : vec(v) {}

    double pow(double x, int n);
    int size();
    std::vector<int> shape();

    Ma operator-(Ma x);
    Ma operator+(Ma x);
    Ma operator*(Ma x);
    Ma operator/(double x);
    Ma operator-();

    std::vector<double> get_column(std::vector<std::vector<double>> x, int n);
    double dot(std::vector<double> x, std::vector<double> y);
    void show();
    void show(Ma x);
    Ma T();
    std::vector<double> div(std::vector<double> x, double y);
    Ma inv();
    double det();
};

Ma operator+(Ma X, double y);
Ma operator+(double y, Ma X);
Ma operator*(Ma X, double y);
Ma operator*(double y, Ma X);
Ma operator-(Ma X, double y);
Ma operator-(double y, Ma X);

#endif // MAT_H

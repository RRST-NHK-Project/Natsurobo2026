#pragma once
#include <iostream>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point.hpp"
using namespace std;

class matrix{
  private:
    vector<double> get_column(vector<vector<double>> x, int n);
    double dot(vector<double> x, vector<double> y);
    vector<vector<double>> vec;
    vector<double> div(vector<double> &x, double y);
    double pow(double x, int n);
  public:
    matrix() = default;
    matrix(vector<vector<double>> x){vec=x;};
    void show(rclcpp::Logger logger);
    static void show(matrix x, rclcpp::Logger logger);
    matrix operator+(matrix x);
    friend matrix operator+(matrix X, double y);
    friend matrix operator+(double y, matrix X);
    matrix operator*(matrix x);
    friend matrix operator*(matrix X, double y);
    friend matrix operator*(double y, matrix X);
    matrix operator-(matrix x);
    friend matrix operator-(matrix X, double y);
    friend matrix operator-(double y, matrix X);
    matrix operator/(double x);
    matrix operator-();
    matrix T();
    matrix rot(double radian);
    matrix rotR(double radian);
    operator vector<vector<double>>(){return vec;};
    double operator()(int x, int y){return vec[x][y];};
    int size();
    vector<int> shape();
    matrix inv();
    double det();
    static matrix from_point(const geometry_msgs::msg::Point & p);
    static matrix from_vector3(const geometry_msgs::msg::Vector3 & v);
    geometry_msgs::msg::Vector3 to_vector3() const;
    geometry_msgs::msg::Point to_point() const;
};
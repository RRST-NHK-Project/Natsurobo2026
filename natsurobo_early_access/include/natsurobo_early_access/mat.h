#pragma once
#include <iostream>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point.hpp"
using namespace std;

class Ma{
  private:
    vector<double> get_column(vector<vector<double>> x, int n);
    double dot(vector<double> x, vector<double> y);
    vector<vector<double>> vec;
    vector<double> div(vector<double> x, double y);
    double pow(double x, int n);
  public:
    Ma(vector<vector<double>> x){vec=x;};
    void show(rclcpp::Logger logger);
    static void show(Ma x, rclcpp::Logger logger);
    Ma operator+(Ma x);
    friend Ma operator+(Ma X, double y);
    friend Ma operator+(double y, Ma X);
    Ma operator*(Ma x);
    friend Ma operator*(Ma X, double y);
    friend Ma operator*(double y, Ma X);
    Ma operator-(Ma x);
    friend Ma operator-(Ma X, double y);
    friend Ma operator-(double y, Ma X);
    Ma operator/(double x);
    Ma operator-();
    Ma T();
    operator vector<vector<double>>(){return vec;};
    double operator()(int x, int y){return vec[x][y];};
    int size();
    vector<int> shape();
    Ma inv();
    double det();
    static Ma from_point(const geometry_msgs::msg::Point & p);
    static Ma from_vector3(const geometry_msgs::msg::Vector3 & v);
    geometry_msgs::msg::Vector3 to_vector3() const;
    geometry_msgs::msg::Point to_point() const;
};
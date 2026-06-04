#include <iostream>
#include <vector>
using namespace std;

class Ma{
  private:
    vector<double> get_column(vector<vector<double>> x, int n);
    double dot(vector<double> x, vector<double> y);
    vector<vector<double>> vec;
    vector<double> div(vector<double> x, double y);
    double pow(double x, int n);
  public:
    Ma(vector<vector<double>> x){vec=x;};//変換コンストラクタ
    void show();
    static void show(Ma x);
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
    operator vector<vector<double>>(){return vec;};//変換関数
    double operator()(int x, int y){return vec[x][y];};
    int size();
    vector<int> shape();
    Ma inv(); //逆行列
    double det(); //行列式
};

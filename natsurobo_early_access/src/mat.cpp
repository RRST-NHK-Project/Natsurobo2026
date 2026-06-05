#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include "natsurobo_early_access/mat.h"
using namespace std;

double matrix::pow(double x, int n){
  int r = 1;
  for(int i=0;i<n;i++) r *= x;
  return r;
}
int matrix::size(){
  return vec.size() * vec[0].size();
}
vector<int> matrix::shape(){
  return {(int)vec.size(),(int)vec[0].size()};
}
matrix matrix::operator-(matrix x){
  vector<vector<double>> A=vec, B=x.vec, C=vec;
  try{
    if(A.size()!=B.size() || A[0].size()!=B[0].size())
      throw "Invalid matrix size! in matrix matrix::operator-(matrix x)";
  }catch(const char* err){ cout<<"Error:"<<err<<"\n"; exit(1); }
  for(int i=0;i<(int)A.size();i++)
    for(int j=0;j<(int)A[i].size();j++)
      C[i][j]=A[i][j]-B[i][j];
  return matrix(C);
}
matrix matrix::operator+(matrix x){
  vector<vector<double>> A=vec, B=x.vec, C=vec;
  try{
    if(A.size()!=B.size() || A[0].size()!=B[0].size())
      throw "Invalid matrix size! in matrix matrix::operator+(matrix x)";
  }catch(const char* err){ cout<<"Error:"<<err<<"\n"; exit(1); }
  for(int i=0;i<(int)A.size();i++)
    for(int j=0;j<(int)A[i].size();j++)
      C[i][j]=A[i][j]+B[i][j];
  return matrix(C);
}
matrix matrix::operator*(matrix x){
  vector<vector<double>> A=vec, B=x.vec;
  vector<vector<double>> C(A.size(), vector<double>(B[0].size(),0));
  try{
    if(A[0].size()!=B.size()){
      cout<<"cols of A:"<<A[0].size()<<" rows of B:"<<B.size()<<"\n";
      throw "Invalid matrix size! in matrix matrix::operator*(matrix x)";
    }
  }catch(const char* err){ cout<<"Error:"<<err<<"\n"; exit(1); }
  for(int i=0;i<(int)A.size();i++)
    for(int s=0;s<(int)B[0].size();s++)
      C[i][s]=dot(A[i],get_column(B,s));
  return matrix(C);
}
matrix matrix::operator/(double x){
  vector<vector<double>> A=vec;
  for(int i=0;i<(int)A.size();i++)
    for(int j=0;j<(int)A[0].size();j++)
      A[i][j]=vec[i][j]/x;
  return matrix(A);
}
vector<double> matrix::get_column(vector<vector<double>> x, int n){
  vector<double> y(x.size());
  for(int i=0;i<(int)x.size();i++) y[i]=x[i][n];
  return y;
}
double matrix::dot(vector<double> x, vector<double> y){
  double z=0;
  for(int i=0;i<(int)x.size();i++) z+=x[i]*y[i];
  return z;
}

void matrix::show(rclcpp::Logger logger){
  for(int i=0;i<(int)vec.size();i++){
    ostringstream oss;
    for(int j=0;j<(int)vec[i].size();j++){
      oss << vec[i][j];
      if(j+1<(int)vec[i].size()) oss << " ";
    }
    RCLCPP_INFO(logger, "[ %s ]", oss.str().c_str());
  }
}
void matrix::show(matrix x, rclcpp::Logger logger){
  x.show(logger);
}

matrix matrix::T(){
  vector<vector<double>> A=vec;
  vector<vector<double>> C(A[0].size(), vector<double>(A.size()));
  for(int i=0;i<(int)A.size();i++)
    for(int j=0;j<(int)A[0].size();j++)
      C[j][i]=A[i][j];
  return matrix(C);
}
vector<double> matrix::div(vector<double> &x, double y){
  if(y == 0){
    throw "Division by zero error in matrix::div(vector<double> &x, double y)";
  }
  for(int i=0;i<(int)x.size();i++) x[i]/=y;
  return x;
}
matrix matrix::inv(){
  try{
    if(vec.size()!=vec[0].size())
      return *this;
  }catch(const char* err){ cout<<"Error:"<<err<<"\n"; exit(1); }
  int n=vec.size();
  vector<vector<double>> A(n, vector<double>(n*2,0));
  for(int i=0;i<n;i++){
    for(int j=0;j<n;j++){
      A[i][j]=vec[i][j];
      A[i][j+n]=(i==j)?1.0:0.0;
    }
  }
  for(int i=0;i<n;i++){
    int pivot=i;
    for(int j=i+1;j<n;j++){
      if(fabs(A[j][i])>fabs(A[pivot][i])){
        pivot=j;
      }
    }

    if(pivot!=i){ 
      swap(A[i],A[pivot]);
    }
    if(fabs(A[i][i])<1e-9){
      vector<vector<double>> E(n, vector<double>(n));
      for(int i=0;i<n;i++){
        E[i][i] = 1.0;
      }
        return matrix(E);
    }
    double pivot_value=A[i][i];
    A[i]=div(A[i],pivot_value);

    for(int j=0;j<n;j++){
      if(j==i) continue;
      double t=A[j][i];
      for(int k=0;k<n*2;k++){ 
        A[j][k]-=A[i][k]*t;
      }
    }
  }
 
  vector<vector<double>> B(n, vector<double>(n));
  for(int i=0;i<n;i++)
    for(int j=0;j<n;j++)
      B[i][j]=A[i][j+n];
  return matrix(B);
}
double matrix::det(){
  vector<vector<double>> A=vec;
  int n=A.size();
  if(n==1) return A[0][0];
  if(n==2) return A[0][0]*A[1][1]-A[0][1]*A[1][0];
  try{
    if(vec.size()!=vec[0].size())
      throw "Invalid matrix size. det() can only be called by square matrix!";
  }catch(const char* err){ cout<<"Error:"<<err<<"\n"; exit(1); }
  double sum=0;
  for(int i=0;i<n;i++){
    vector<vector<double>> B=A;
    for(int j=0;j<n;j++) B[j].erase(B[j].begin()+i);
    B.erase(B.begin());
    sum+=A[0][i]*pow(-1,i+2)*matrix(B).det();
  }
  return sum;
}
matrix matrix::operator-(){
  vector<vector<double>> C=vec;
  for(int i=0;i<(int)vec.size();i++)
    for(int j=0;j<(int)vec[0].size();j++)
      C[i][j]=-vec[i][j];
  this->vec=C;
  return *this;
}
matrix operator+(matrix X, double y){
  vector<vector<double>> x=X.vec;
  vector<vector<double>> A(x.size(), vector<double>(x[0].size(),y));
  return X+matrix(A);
}
matrix operator+(double y, matrix X){ return X+y; }
matrix operator*(matrix X, double y){
  vector<vector<double>> x=X.vec;
  vector<vector<double>> A=x;
  for(int i=0;i<(int)x.size();i++)
    for(int j=0;j<(int)x[0].size();j++)
      A[i][j]=x[i][j]*y;
  return matrix(A);
}
matrix operator*(double y, matrix X){ return X*y; }
matrix operator-(matrix X, double y){
  vector<vector<double>> x=X.vec;
  vector<vector<double>> A(x.size(), vector<double>(x[0].size(),y));
  return X-matrix(A);
}
matrix operator-(double y, matrix X){ return -X+y; }

matrix matrix::rotR(double radian){
  return matrix({{cos(radian),-sin(radian)},{sin(radian),cos(radian)}});
}
matrix matrix::rot(double radian){
  return matrix({{cos(radian),sin(radian)},{-sin(radian),cos(radian)}});
}

matrix matrix::from_vector3(const geometry_msgs::msg::Vector3 & v){
  return matrix({{v.x},{v.y},{v.z}});
}
matrix matrix::from_point(const geometry_msgs::msg::Point & p){
  return matrix({{p.x},{p.y},{p.z}});
}

geometry_msgs::msg::Vector3 matrix::to_vector3() const {
  geometry_msgs::msg::Vector3 v;
  if(vec.size()==3 && vec[0].size()==1){
    v.x=vec[0][0]; v.y=vec[1][0]; v.z=vec[2][0];
  } else if(vec.size()==1 && vec[0].size()==3){
    v.x=vec[0][0]; v.y=vec[0][1]; v.z=vec[0][2];
  }
  return v;
}
geometry_msgs::msg::Point matrix::to_point() const {
  geometry_msgs::msg::Point p;
  if(vec.size()==3 && vec[0].size()==1){
    p.x=vec[0][0]; p.y=vec[1][0]; p.z=vec[2][0];
  } else if(vec.size()==1 && vec[0].size()==3){
    p.x=vec[0][0]; p.y=vec[0][1]; p.z=vec[0][2];
  }
  return p;
}
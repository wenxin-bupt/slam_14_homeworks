#include <iostream>
#include <ctime>
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算 （逆, Eigen Value..)
#include <Eigen/Dense>

#define MATRIX_SIZE 100

int main(int argc, char** argv) {
    // Decalre a random 100x100 matrix as a linear equation
    // A * x = b
    // Eigen::MatrixXd A;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A;
    A = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> b;
    b = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    std::clock_t time_stt = std::clock();
    // Direct inverse
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x = A.inverse() * b;
    std::cout <<"time use in normal inverse is " << 1000 * (std::clock() - time_stt)/static_cast<double>(CLOCKS_PER_SEC)<< "ms"<< std::endl;
    std::cout <<"x shape rows " << x.rows() <<" cols " << x.cols() << std::endl;


    // Solve its linear equation using QR decomposition

    // Solve its linear equation using Cholesky decomposition



    return 0;
}

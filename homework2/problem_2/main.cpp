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
    // to make a half positive definite matrix
    A = A * A.transpose();
    std::cout << "what is  A " << A << std::endl;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> b;
    b = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    std::clock_t time_stt = std::clock();
    // Direct inverse
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x = A.inverse() * b;
    std::cout <<"time use in normal inverse is " << 1000 * (std::clock() - time_stt)/static_cast<double>(CLOCKS_PER_SEC)<< "ms"<< std::endl;

    // Solve its linear equation using QR decomposition
    time_stt = std::clock();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x_qr = A.colPivHouseholderQr().solve(b);
    std::cout << "time use in qr decomposition is " << 1000 * (std::clock() - time_stt) / static_cast<double>(CLOCKS_PER_SEC) << "ms" << std::endl;
    std::cout << "x is " << x << std::endl;
    std::cout << "x qr is " << x_qr << std::endl;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> b_res = A * x_qr;
    std::cout <<"b original \n"<<b <<"\n b qr is\n " << b_res << std::endl;
    assert(b.isApprox(b_res));

    // Solve its linear equation using Cholesky decomposition
    // NOTE: only works for positive definite matrix
    time_stt = std::clock();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> x_ldlt = A.ldlt().solve(b);
    std::cout << "time use in LDLT decomposition is " << 1000 * (std::clock() - time_stt) / static_cast<double>(CLOCKS_PER_SEC) << "ms" << std::endl;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  b_res_ldlt = A * x_ldlt;

    std::cout <<"b original \n"<<b <<"\n b LDLT is\n " << b_res_ldlt << std::endl;

    return 0;
}

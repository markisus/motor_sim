#include <Eigen/Dense>
#include <iostream>

int main() {
    Eigen::MatrixXd m(4, 4);

    // Output a matrix that counts from 0 to 15
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            m(i, j) = i * 4 + j;
        }
    }

    std::cout << m;

    return 0;
}
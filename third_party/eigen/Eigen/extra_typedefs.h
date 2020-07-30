#include <Eigen/Dense>

// Additional typedefs - biro specific
namespace Eigen {
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 7, 7> Matrix7f;
typedef Eigen::Matrix<float, 8, 8> Matrix8f;
typedef Matrix<float, 6, 1> Vector6f;
typedef Matrix<float, 7, 1> Vector7f;
typedef Matrix<float, 8, 1> Vector8f;
typedef Matrix<float, 16, 1> Vector16f;

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;
typedef Eigen::Matrix<double, 8, 8> Matrix8d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 8, 1> Vector8d;
typedef Matrix<double, 16, 1> Vector16d;

}  // namespace Eigen

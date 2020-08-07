#include <cmath>

template <typename Scalar>
void generate_odd_sine_series(int num_terms, Scalar angle, Scalar* begin) {
    const Scalar sin_angle = std::sin(angle);
    const Scalar cos_angle = std::cos(angle);

    Scalar sa = 0;
    Scalar sb = sin_angle;

    for (int i = 0; i < num_terms - 1; ++i) {
        begin[i] = sb;

        // sine recursion
        // https://trans4mind.com/personal_development/mathematics/trigonometry/multipleAnglesRecursiveFormula.htm#Recursive_Formula

        // advance sb
        sa = 2 * cos_angle * sb - sa;
        std::swap(sb, sa);

        // advance sb again
        sa = 2 * cos_angle * sb - sa;
        std::swap(sb, sa);
    }

    begin[num_terms - 1] = sb;
}

template <typename Scalar>
void generate_odd_sine_series_reference(int num_terms, Scalar angle,
                                        Scalar* begin) {
    for (int i = 0; i < num_terms; ++i) {
        begin[i] = std::sin((2 * i + 1) * angle);
    }
}

#include <iostream>
#include <Eigen/Dense>

int main() {
    std::cout << "RBD Engine: Phase 0 Scaffolding Complete." << std::endl;
    
    // Quick Eigen Sanity Check
    Eigen::Vector3d test_vec(1.0, 2.0, 3.0);
    std::cout << "Eigen linking successful. Test Vector:\n" << test_vec << std::endl;

    return 0;
}
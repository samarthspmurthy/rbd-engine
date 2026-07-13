#pragma once
#include "TwoLinkArm.hpp"

namespace rbd {

class DoublePendulum : public TwoLinkArm {
public:
    DoublePendulum() {
        // Textbook uniform rod parameters
        m1 = 1.0; 
        m2 = 1.0;
        l1 = 1.0; 
        l2 = 1.0;
        lc1 = l1 / 2.0; 
        lc2 = l2 / 2.0;
        
        // I = 1/12 * m * L^2 for slender rods about COM
        I1 = (1.0 / 12.0) * m1 * (l1 * l1);
        I2 = (1.0 / 12.0) * m2 * (l2 * l2);
        
        g = 9.81;
    }
};

} // namespace rbd
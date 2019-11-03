//
//  CosseratRod.cpp
//  SnareNeedleModel
//
//  Created by Art Mahoney on 7/6/16.
//  Copyright © 2016 Art Mahoney. All rights reserved.
//

#include "CosseratRod.h"
#include "Tools.h"





/**
 * The constructor computes the inverse stiffness from the items provided.
 *
 */
CosseratRod::CosseratRod(double inner_diameter /* m */,
                         double outer_diameter /* m */,
                         double length /* m */,
                         double poissons_ratio,
                         double youngs_modulus)  {
    
    
    double second_moment_area = (M_PI / 64.0) * (pow(outer_diameter, 4.0) - pow(inner_diameter, 4.0) );
    double shear_modulus = youngs_modulus / (2.0 * (1.0 + poissons_ratio));
    double polar_moment_area = 2.0 * second_moment_area;
    
    double EI = second_moment_area*youngs_modulus;
    double JG = polar_moment_area*shear_modulus;
    
    this->length = length;
    this->Kinv = Eigen::DiagonalMatrix<double, 3>(1.0 / EI, 1.0 / EI, 1.0 / JG);
    
    return;
}


double CosseratRod::getLength() const {
    return length;
}




/*
const typename CosseratRod::CosseratRodDerivative &CosseratRod::getDerivative() const {
    return derivative;
}
 */

/*
const typename CosseratRod::CosseratRodCallback &CosseratRod::getCallback() const {
    return callback;
}
*/







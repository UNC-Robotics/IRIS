//
//  CosseratRod.hpp
//  SnareNeedleModel
//
//  Created by Art Mahoney on 7/6/16.
//  Copyright © 2016 Art Mahoney. All rights reserved.
//

#ifndef CosseratRod_hpp
#define CosseratRod_hpp

#include <typeinfo>
#include <cstddef>
#include <iostream>
#include <Eigen/Dense>
#include "RK8Integrator.h"
#include "Quaternion.h"



class CosseratRod  {
public: // public classes and typedefs for CosseratRod


private: // private member variables for CosseratRod
    double length;                                              /* length of the rod, m */
    Eigen::Matrix3d Kinv;                                       /* inverse of the stiffness matrix */

    //CosseratRodDerivative derivative;                           /* implements the arc-length derivative */
    //CosseratRodStateMatrixArrayDerivative ref_sub_matrix_state_derivative;
    //CosseratRodSubMatrixStateDerivative sub_matrix_state_derivative;

    //CosseratRodCallback callback;                               /* implements a quaternion-normalizing callback */


public:
    // Constructor
    CosseratRod(double inner_diameter,                  /* m */
                double outer_diameter,                  /* m */
                double length,                          /* m */
                double poissons_ratio = 0.33,           /* default for nitinol */
                double youngs_modulus = 50e9            /* default for nitinol */);

    //CosseratRod(const CosseratRod &rod) : length(rod.length), Kinv(rod.Kinv), derivative(*this), ref_sub_matrix_state_derivative(*this), sub_matrix_state_derivative(*this) {};


    double getLength() const; // returns the length of the rod

    const inline Eigen::Matrix3d &getKinv() const {
        return Kinv;
    }



    //const CosseratRodDerivative &getDerivative() const; // returns the Cosserat arc-length derivative
    //const CosseratRodStateMatrixArrayDerivative &getRefSubMatrixStateDerivative() const { return ref_sub_matrix_state_derivative; };
    //const CosseratRodSubMatrixStateDerivative &getSubMatrixStateDerivative() const { return sub_matrix_state_derivative; };
    //const CosseratRodCallback &getCallback() const; // returns the callback function that normalizes the quaternion part of the state


public:


};



#endif /* CosseratRod_hpp */

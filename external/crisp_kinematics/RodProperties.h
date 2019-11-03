//
//  RodProperties.h
//  SnareNeedleModel
//
//  Created by Art Mahoney on 2/3/17.
//  Copyright © 2017 Art Mahoney. All rights reserved.
//

#ifndef RodProperties_h
#define RodProperties_h


#include <typeinfo>
#include <cstddef>
#include <iostream>
#include "Eigen/Dense"
#include "Quaternion.h"
#include "Tools.h"


/**
 * This class stores the properties of a cosserat rod.
 *
 */
class RodProperties  {
private:
    double my_length;                                              /* length of the rod (set to -1 if none provided in the constructor), m */
    Eigen::Matrix3d my_Kinv;                                       /* inverse of the stiffness matrix (computed in the constructor) */
    
public:
    /**
     * Constructor that takes the rod properties as input.
     *
     */
    RodProperties(double inner_diameter,                  /* m */
                double outer_diameter,                  /* m */
                double length = -1.0,                   /* m */
                double poissons_ratio = 0.33,           /* default for nitinol */
                double youngs_modulus = 50e9            /* default for nitinol */) {
        
        double second_moment_area = (M_PI / 64.0) * (pow(outer_diameter, 4.0) - pow(inner_diameter, 4.0) );
        double shear_modulus = youngs_modulus / (2.0 * (1.0 + poissons_ratio));
        double polar_moment_area = 2.0 * second_moment_area;
        
        double EI = second_moment_area*youngs_modulus;
        double JG = polar_moment_area*shear_modulus;
        
        this->my_length = length;
        this->my_Kinv = Eigen::DiagonalMatrix<double, 3>(1.0 / EI, 1.0 / EI, 1.0 / JG);
    }
    
    
    
    /**
     * Returns the length of the rod.
     *
     */
    double getLength() const { return my_length; };
    
    
    /**
     * Returns the precomputed inverse of the stiffness matrix.
     *
     */
    const inline Eigen::Matrix3d &getKinv() const {
        return my_Kinv;
    }
    
    
    /**
     * Computes the cosserat rod state derivative with respect to arclength as well as the state partial derivative.
     *
     */
    template <class StateType>
    void getDerivative(const StateType &state, StateType &derivative) const {
        
        
        Quaternion quaternion(state.vector().template block<4,1>(3,0));
        Eigen::Vector3d moment = state.vector().template block<3,1>(7,0);
        Eigen::Vector3d force = state.vector().template block<3,1>(10,0);
        
        
        Eigen::Vector3d moment_in_body_frame = quaternion.conjugate() * moment;
        Eigen::Vector3d angular_velocity_body_frame = this->getKinv() * moment_in_body_frame;
        
        
        // Computes the state derivative first.
        derivative.vector().template block<3,1>(0,0).noalias() = quaternion * Eigen::Vector3d::UnitZ(); // arc-length derivative of the position state
        derivative.vector().template block<4,1>(3,0).noalias() = 0.5 * quaternion * Quaternion(0.0, angular_velocity_body_frame); // arc-length derivative of the quaternion
        derivative.vector().template block<3,1>(7,0).noalias() = force.cross(derivative.vector().template block<3,1>(0,0)); // arc-length derivative of the moment
        derivative.vector().template block<3,1>(10,0).noalias() = Eigen::Vector3d::Zero();
        
        
        double w = quaternion.w();
        double r1 = quaternion.vec()(0);
        double r2 = quaternion.vec()(1);
        double r3 = quaternion.vec()(2);
        
        
        //Eigen::Matrix4d dq_dq = Eigen::Matrix4d::Identity() - static_cast<Eigen::Vector4d>(quaternion)*static_cast<Eigen::Vector4d>(quaternion).transpose();
        //Eigen::Matrix4d dq_dq = Eigen::Matrix4d::Identity();
        
        // The derivative of p' with respect to p is 0.
        //Eigen::Matrix3d dpprime_dp = Eigen::Matrix3d::Zero();
        
        // Compute the derivative of p' with respect to q.
        //Eigen::Matrix<double, 3, 4> dpprime_dq = 2.0*(Eigen::Matrix<double, 3, 4>() << r2, r3, w, r1, -r1, -w, r3, r2, 0.0, -2.0*r1, -2.0*r2, 0.0).finished()*dq_dq;
        Eigen::Matrix<double, 3, 4> dpprime_dq; dpprime_dq << 2.0*r2, 2.0*r3, 2.0*w, 2.0*r1, -2.0*r1, -2.0*w, 2.0*r3, 2.0*r2, 0.0, -4.0*r1, -4.0*r2, 0.0;
        
        // The derivative of dpprime_dm is 0.
        Eigen::Matrix3d dpprime_dm = Eigen::Matrix3d::Zero();
        
        // The derivative of dpprime_dn is 0.
        Eigen::Matrix3d dpprime_dn = Eigen::Matrix3d::Zero();
        
        // The derivative of q' with repsect to p is 0.
        //Eigen::Matrix<double, 4, 3> dqprime_dp = Eigen::Matrix<double, 4, 3>::Zero();
        
        
        double m1 = moment(0);
        double m2 = moment(1);
        double m3 = moment(2);
        
        
        // Compute the derivative of q' with respect to q.
        
        //Eigen::Vector3d Qs_m = (Eigen::Matrix3d() << 0.0, 2.0*r3, -2.0*r2, -2.0*r3, 0.0, 2.0*r1, 2.0*r2, -2.0*r1, 0.0).finished()*moment;
        
        //Eigen::Vector3d Qx_m = (Eigen::Matrix3d() << 0.0, 2.0*r2,  2.0*r3,  2.0*r2, -4.0*r1, 2.0*w,  2.0*r3, -2.0*w,  -4.0*r1).finished()*moment;
        
        //Eigen::Vector3d Qy_m = (Eigen::Matrix3d() << -4.0*r2, 2.0*r1, -2.0*w, 2.0*r1, 0.0, 2.0*r3, 2.0*w, 2.0*r3, -4.0*r2).finished()*moment;
        
        //Eigen::Vector3d Qz_m = (Eigen::Matrix3d() << -4.0*r3, 2.0*w, 2.0*r1, -2.0*w, -4.0*r3, 2.0*r2, 2.0*r1, 2.0*r2, 0.0).finished()*moment;
        
        
        
        Eigen::Vector3d Qs_m;
        Qs_m(0) = 2.0*(r3*m2 - r2*m3);
        Qs_m(1) = 2.0*(-r3*m1 + r1*m3);
        Qs_m(2) = 2.0*(r2*m1 - r1*m2);
        
        Eigen::Vector3d Qx_m;
        Qx_m(0) = 2.0*(r2*m2 + r3*m3);
        Qx_m(1) = 2.0*(r2*m1 - 2.0*r1*m2 + w*m3);
        Qx_m(2) = 2.0*(r3*m1 - w*m2 -2.0*r1*m3);
        
        
        Eigen::Vector3d Qy_m;
        Qy_m(0) = 2.0*(-2.0*r2*m1 + r1*m2 -w*m3);
        Qy_m(1) = 2.0*(r1*m1 + r3*m3);
        Qy_m(2) = 2.0*(w*m1 + r3*m2 -2.0*r2*m3);
        
        Eigen::Vector3d Qz_m;
        Qz_m(0) = 2.0*(-2.0*r3*m1 + w*m2 + r1*m3);
        Qz_m(1) = 2.0*(-w*m1 -2.0*r3*m2 +r2*m3);
        Qz_m(2) = 2.0*(r1*m1 + r2*m2);
        
        
        
        
        //Eigen::Matrix<double, 3, 4> dRTM_dq = (Eigen::Matrix<double, 3, 4>() << Qs_m, Qx_m, Qy_m, Qz_m).finished()*dq_dq;
        Eigen::Matrix<double, 3, 4> dRTM_dq; dRTM_dq << Qs_m, Qx_m, Qy_m, Qz_m;
        
        Eigen::Matrix<double, 3, 4> du_dq = this->getKinv() * dRTM_dq;
        Eigen::Matrix<double, 4, 3> tmp; tmp << -quaternion.vec().transpose(), Tools::skew(quaternion.vec())+quaternion.w()*Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 4, 4> tmp2; tmp2 << 0.0, -angular_velocity_body_frame.transpose(), angular_velocity_body_frame, -Tools::skew(angular_velocity_body_frame);
        
        
        //Eigen::Matrix<double, 4, 4> dqprime_dq = 0.5 * (tmp*du_dq + tmp2*dq_dq);
        Eigen::Matrix<double, 4, 4> dqprime_dq = 0.5 * (tmp*du_dq + tmp2);
        //Eigen::Matrix<double, 4, 4> dqprime_dq = 0.5*tmp2;
        //dqprime_dq.noalias() += 0.5*tmp*du_dq;
        
        
        // Compute the derivative of q' with respect to m
        Eigen::Matrix3d du_dm = this->getKinv() * quaternion;
        Eigen::Matrix<double, 4, 3> dqprime_dm = 0.5 * tmp * du_dm;
        
        // The derivative of q' with respect to n is 0.
        Eigen::Matrix<double, 4, 3> dqprime_dn = Eigen::Matrix<double, 4, 3>::Zero();
        
        // The derivative of m' with respect to p is 0.
        //Eigen::Matrix3d dmprime_dp = Eigen::Matrix3d::Zero();
        
        // Compute the derivative of m' with respect to  q.
        Eigen::Matrix<double, 3, 4> dmprime_dq = Tools::skew(force)*dpprime_dq;
        
        // The derivative of m' with respect to m is 0.
        Eigen::Matrix3d dmprime_dm = Eigen::Matrix3d::Zero();
        
        // Compute the derivative of m' with respect to n
        Eigen::Matrix3d dmprime_dn = -Tools::skew(derivative.vector().template block<3,1>(0,0));
        
        // The derivative of n' with respect to everything else is 0.
        //Eigen::Matrix<double, 3, 13> dnprime_deverything = Eigen::Matrix<double, 3, 13>::Zero();
        
        
        
        Eigen::Matrix<double, 10, 10> F;
        F << dpprime_dq, dpprime_dm, dpprime_dn,
        dqprime_dq, dqprime_dm, dqprime_dn,
        dmprime_dq, dmprime_dm, dmprime_dn;
        
        
        
        derivative.derivative().template topRows<10>().noalias() = F*state.derivative().block(3, 0, 10, state.derivative().cols());
        
        
        derivative.derivative().template bottomRows<4>().setConstant(0);
  
        
        
        
        return;
    }

    
    /**
     * Computes the cosserat rod state derivative with respect to arclength.
     *
     */
    template <int rod_state_dimension>
    void getDerivative(const Eigen::Matrix<double, rod_state_dimension, 1> &state_vector,
                       Eigen::Matrix<double, rod_state_dimension, 1> &f) const {
        
        
        Quaternion quaternion(state_vector.template block<4,1>(3,0));
        Eigen::Vector3d moment = state_vector.template block<3,1>(7,0);
        Eigen::Vector3d force = state_vector.template block<3,1>(10,0);
        
        
        Eigen::Vector3d moment_in_body_frame = quaternion.conjugate() * moment;
        Eigen::Vector3d angular_velocity_body_frame = this->getKinv() * moment_in_body_frame;
        
        
        // Computes the state derivative first.
        f.setConstant(0.0);
        f.template block<3,1>(0,0).noalias() = quaternion * Eigen::Vector3d::UnitZ(); // arc-length derivative of the position state
        f.template block<4,1>(3,0).noalias() = 0.5 * quaternion * Quaternion(0.0, angular_velocity_body_frame); // arc-length derivative of the quaternion
        f.template block<3,1>(7,0).noalias() = force.cross(f.template block<3,1>(0,0)); // arc-length derivative of the moment
        //f.template block<3,1>(10,0).noalias() = Eigen::Vector3d::Zero(); // already set to zero
        
        return;
    }

    
    
    
    
};


#endif /* RodProperties_h */

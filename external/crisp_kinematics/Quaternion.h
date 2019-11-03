
//
//  Quaternion.hpp
//  SnareNeedleModel
//
//  Created by Art Mahoney on 7/13/16.
//  Copyright © 2016 Art Mahoney. All rights reserved.
//

#ifndef Quaternion_hpp
#define Quaternion_hpp

#include <iostream>
#include <ostream>
#include "Eigen/Dense"


/**
 * This class is derived from a 4D vector and implements a quaternion.
 *
 */
class Quaternion : public Eigen::Vector4d {
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Quaternion() : Eigen::Vector4d() {};
    Quaternion(double w, double x, double y, double z) : Eigen::Vector4d(w, x, y, z) {};
    Quaternion(double w, const Eigen::Vector3d &vec) : Eigen::Vector4d(w, vec.x(), vec.y(), vec.z()) {};
    Quaternion(const Eigen::Vector4d &vec) : Eigen::Vector4d(vec(0), vec(1), vec(2), vec(3)) {};
    
    
    
    
    double w() const { return this->operator()(0); };
    double x() const { return this->operator()(1); };
    double y() const { return this->operator()(2); };
    double z() const { return this->operator()(3); };
    
    /**
     * Returns the conjugate of the quaterntion.
     *
     */
    Quaternion conjugate() {
        return Quaternion(this->w(), -1.0*this->x(), -1.0*this->y(), -1.0*this->z());
    }

    
    /**
     * Returns the vector part of the quaternion.
     *
     */
    Eigen::Vector3d vec() const {
        return Eigen::Vector3d(this->x(), this->y(), this->z());
    }
    
    
    Eigen::Matrix3d asRot() const {
        Eigen::Matrix3d rot;
        
        double sqw = this->w()*this->w();
        double sqx = this->x()*this->x();
        double sqy = this->y()*this->y();
        double sqz = this->z()*this->z();
        
        rot(0,0) = ( sqx - sqy - sqz + sqw);
        rot(1,1) = (-sqx + sqy - sqz + sqw);
        rot(2,2) = (-sqx - sqy + sqz + sqw);
        
        double tmp1 = this->x()*this->y();
        double tmp2 = this->z()*this->w();
        rot(1,0) = 2.0 * (tmp1 + tmp2);
        rot(0,1) = 2.0 * (tmp1 - tmp2);
        
        tmp1 = this->x()*this->z();
        tmp2 = this->y()*this->w();
        rot(2,0) = 2.0 * (tmp1 - tmp2);
        rot(0,2) = 2.0 * (tmp1 + tmp2);
        tmp1 = this->y()*this->z();
        tmp2 = this->x()*this->w();
        rot(2,1) = 2.0 * (tmp1 + tmp2);
        rot(1,2) = 2.0 * (tmp1 - tmp2);
        
        return rot;
    }
    
    /**
     * Multiplies two quaternions together.
     *
     */
    const Quaternion operator*(const Quaternion &other) const {
        double w_part = this->w()*other.w() - this->vec().dot(other.vec());
        Eigen::Vector3d vec_part = this->w()*other.vec() + other.w()*this->vec() + this->vec().cross(other.vec());
        
        return Quaternion(w_part, vec_part);
    }
    
    

    
    /**
     * Rotates the rows of an Nx3 matrix using the quaternion multiplied on the right.  The result is stored in a matrix of the same size.
     *
     */
    template <typename Derived>
    friend Eigen::Matrix<typename Derived::Scalar, Derived::RowsAtCompileTime, 3> operator*(const Eigen::MatrixBase<Derived> &v, const Quaternion &quat) {
        
        
        Eigen::Matrix<typename Derived::Scalar, 3, Derived::RowsAtCompileTime> tmp = v.transpose();
        Eigen::Matrix<typename Derived::Scalar, 3, 1> tmp_col;
        
        
        
        for (int aa=0; aa<Derived::RowsAtCompileTime; ++aa) {
            tmp_col = quat.vec().cross(tmp.col(aa)) + quat.w()*tmp.col(aa);
            tmp.col(aa).noalias() += 2.0 * quat.vec().cross(tmp_col);
        }
        
        
        return tmp.transpose();
    }
    
    
    
    
    /**
     * Rotates the columns of a 3xN matrix using the quaternion.  The result is stored in a matrix of the same size.
     *
     */
    template <typename Derived>
    friend Eigen::Matrix<typename Derived::Scalar, 3, Derived::ColsAtCompileTime> operator*(const Quaternion &quat, const Eigen::MatrixBase<Derived> &v) {
        
        Eigen::Matrix<typename Derived::Scalar, 3, Derived::ColsAtCompileTime> tmp = v;
        Eigen::Matrix<typename Derived::Scalar, 3, 1> tmp_col;
        
        
        for (int aa=0; aa<Derived::ColsAtCompileTime; ++aa) {
            tmp_col = quat.vec().cross(v.col(aa)) + quat.w()*v.col(aa);
            tmp.col(aa).noalias() += 2.0 * quat.vec().cross(tmp_col);
        }
        
        return tmp;
    }
    
    
    /**
     * Multiples a quaternion by a double on the left side.
     *
     */
    friend Quaternion operator*(double other, const Quaternion &quaternion) {
        return Quaternion(other*quaternion.w(), other*quaternion.x(), other*quaternion.y(), other*quaternion.z());
    }
    
    
    
};

#endif /* Quaternion2_hpp */

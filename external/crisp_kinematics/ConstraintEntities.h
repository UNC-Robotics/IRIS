//
//  ConstraintEntities.h
//  SnareNeedleModel
//
//  Created by Art Mahoney on 8/3/16.
//  Copyright © 2016 Art Mahoney. All rights reserved.
//

#ifndef ConstraintEntities_h
#define ConstraintEntities_h


#include "Entities.h"
#include "State.h"
#include "Quaternion.h"




/**
 * This implements the grasp constraint where the snare tip position and tool body position are coincedent, the snare tip heading and the body heading are perpendicular,
 * the internal load of the force is added to the internal load of the tool, and the component of the moment that is not in the tip heading direction nor the body
 * heading direction is added to the internal moment of the tool.
 *
 */
template<class GrasperType>
class GraspConstraint : public ConstraintEntity<GraspConstraint<GrasperType>, GrasperType> {
public:
    static constexpr int dimension = 6;

private:

    /**
     * This function computes parts needed for the derivative transition matrix that transitions the state derivative from the left to right
     * sides of the grasp discontinuity.
     *
     */
    void computeGraspStateTransitionDerivative(const Eigen::Matrix<double, 13, 1> &snarer_state,
                                               const Eigen::Matrix<double, 13, 1> &snared_state,
                                               Eigen::Matrix<double, 13, 13> &dgrasp_dsnared,
                                               Eigen::Matrix<double, 13, 13> &dgrasp_dsnarer) {

        Quaternion snared_q(snared_state.template segment<4>(3)); // snared quaternion
        Quaternion snarer_q(snarer_state.template segment<4>(3)); // snarer quaternion
        Eigen::Vector3d snarer_m = snarer_state.template segment<3>(7); // snarer moment

        // Compute the state transition matrix, NOTE THAT THIS GETS RECOMPUTED... IT'S INEFFICIENT
        Eigen::Vector3d a1 = snared_q*Eigen::Vector3d::UnitZ();
        Eigen::Vector3d a2 = snarer_q*Eigen::Vector3d::UnitZ();
        double a1_dot_a2 = a1.dot(a2);
        double c = 1.0/(1 - a1_dot_a2*a1_dot_a2);
        Eigen::Matrix3d a1a1T = a1*a1.transpose();
        Eigen::Matrix3d a2a2T = a2*a2.transpose();
        Eigen::Matrix3d a1a2T = a1*a2.transpose();
        Eigen::Matrix3d a2a1T = a2*a1.transpose();
        Eigen::Matrix3d K = a1a1T - a1_dot_a2*a1a2T + a2a2T - a1_dot_a2*a2a1T;
        Eigen::Matrix3d L = Eigen::Matrix3d::Identity() - c*K; // this is I - A*pinv(A), where A = [a1 a2]


        // Compute the derivative of (eye(3)-ApinvA)*snare_m w.r.t. a1
        Eigen::Matrix3d my_L1 = ((a1.dot(snarer_m)*Eigen::Matrix3d::Identity() + a1*snarer_m.transpose())*(Eigen::Matrix3d::Identity() - a2a2T) - a1_dot_a2*(a2.dot(snarer_m)*Eigen::Matrix3d::Identity() + a2*snarer_m.transpose()))*(Eigen::Matrix3d::Identity() - a1a1T);
        //Eigen::Vector3d my_c1 = 2.0*(a1_dot_a2*(Eigen::Matrix3d::Identity() - a1a1T)*a2)*c*c;
        Eigen::Vector3d my_c1 = 2.0*a1_dot_a2*(a2 - a1_dot_a2*a1)*c*c;
        Eigen::Matrix3d dLm_da1 = -K*snarer_m*my_c1.transpose() - c*my_L1; // derivative of L*snarer_m w.r.t. a1

        // Compute da1/dsnared_q
        double snared_w = snared_q.w();
        double snared_r1 = snared_q.vec()(0);
        double snared_r2 = snared_q.vec()(1);
        double snared_r3 = snared_q.vec()(2);
        //Eigen::Matrix4d dsnared_q_dsnared_q = Eigen::Matrix4d::Identity() - static_cast<Eigen::Vector4d>(snared_q)*static_cast<Eigen::Vector4d>(snared_q).transpose();
        Eigen::Matrix4d dsnared_q_dsnared_q = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 3, 4> da1_dsnared_q = 2.0*(Eigen::Matrix<double, 3, 4>() << snared_r2, snared_r3, snared_w, snared_r1, -snared_r1, -snared_w, snared_r3, snared_r2, 0.0, -2.0*snared_r1, -2.0*snared_r2, 0.0).finished()*dsnared_q_dsnared_q;

        dgrasp_dsnared = Eigen::Matrix<double,13,13>::Identity();
        dgrasp_dsnared.template block<3,4>(7,3) = dLm_da1*da1_dsnared_q; // derivative of L*snarer_m w.r.t. snared_q


        // Compute the derivative of (eye(3)-ApinvA)*snare_m w.r.t. a2
        Eigen::Matrix3d my_L2 = (a2.dot(snarer_m)*Eigen::Matrix3d::Identity() + a2*snarer_m.transpose())*(Eigen::Matrix3d::Identity() - a1a1T) - a1_dot_a2*(a1.dot(snarer_m)*Eigen::Matrix3d::Identity() + a1*snarer_m.transpose())*(Eigen::Matrix3d::Identity() - a2a2T);
        Eigen::Vector3d my_c2 = 2.0*a1_dot_a2*(a1 - a1_dot_a2*a2)*c*c;
        Eigen::Matrix3d dLm_da2 = -K*snarer_m*my_c2.transpose() - c*my_L2;

        // Compute da2/dsnarer_q
        double snarer_w = snarer_q.w();
        double snarer_r1 = snarer_q.vec()(0);
        double snarer_r2 = snarer_q.vec()(1);
        double snarer_r3 = snarer_q.vec()(2);
        //Eigen::Matrix4d dsnarer_q_dsnarer_q = Eigen::Matrix4d::Identity() - static_cast<Eigen::Vector4d>(snarer_q)*static_cast<Eigen::Vector4d>(snarer_q).transpose();
        Eigen::Matrix4d dsnarer_q_dsnarer_q = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 3, 4> da2_dsnarer_q = 2.0*(Eigen::Matrix<double, 3, 4>() << snarer_r2, snarer_r3, snarer_w, snarer_r1, -snarer_r1, -snarer_w, snarer_r3, snarer_r2, 0.0, -2.0*snarer_r1, -2.0*snarer_r2, 0.0).finished()*dsnarer_q_dsnarer_q;

        dgrasp_dsnarer = Eigen::Matrix<double,13,13>::Zero();
        dgrasp_dsnarer.template block<3,4>(7,3) = dLm_da2*da2_dsnarer_q; // derivative of L*snarer_m w.r.t. snarer_q
        dgrasp_dsnarer.template block<3,3>(7,7) = L;
        dgrasp_dsnarer.template block<3,3>(10,10) = Eigen::Matrix3d::Identity();

    }


    /**
     * This function computes the derivative of the grasp constraint with respect to the current snared tool and snarer states.
     *
     */
    void computeGraspConstraintDerivative(const Eigen::Matrix<double, 13, 1> &snarer_state,
                                          const Eigen::Matrix<double, 13, 1> &snared_state,
                                          Eigen::Matrix<double, 6, 13> &dc_dsnared,
                                          Eigen::Matrix<double, 6, 13> &dc_dsnarer) {




        Quaternion snared_q(snared_state.template segment<4>(3)); // snared quaternion
        Quaternion snarer_q(snarer_state.template segment<4>(3)); // snarer quaternion

        Eigen::Vector3d a1 = snared_q*Eigen::Vector3d::UnitZ(); // heading of the tool
        Eigen::Vector3d a2 = snarer_q*Eigen::Vector3d::UnitZ(); // heading of the snare

        Eigen::Vector3d snarer_m = snarer_state.template segment<3>(7); // snarer moment

        // Compute da1/dsnared_q
        double snared_w = snared_q.w();
        double snared_r1 = snared_q.vec()(0);
        double snared_r2 = snared_q.vec()(1);
        double snared_r3 = snared_q.vec()(2);
        //Eigen::Matrix4d dsnared_q_dsnared_q = Eigen::Matrix4d::Identity() - static_cast<Eigen::Vector4d>(snared_q)*static_cast<Eigen::Vector4d>(snared_q).transpose();
        Eigen::Matrix4d dsnared_q_dsnared_q = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 3, 4> da1_dsnared_q = 2.0*(Eigen::Matrix<double, 3, 4>() << snared_r2, snared_r3, snared_w, snared_r1, -snared_r1, -snared_w, snared_r3, snared_r2, 0.0, -2.0*snared_r1, -2.0*snared_r2, 0.0).finished()*dsnared_q_dsnared_q;

        // Compute da2/dsnarer_q
        double snarer_w = snarer_q.w();
        double snarer_r1 = snarer_q.vec()(0);
        double snarer_r2 = snarer_q.vec()(1);
        double snarer_r3 = snarer_q.vec()(2);
        //Eigen::Matrix4d dsnarer_q_dsnarer_q = Eigen::Matrix4d::Identity() - static_cast<Eigen::Vector4d>(snarer_q)*static_cast<Eigen::Vector4d>(snarer_q).transpose();
        Eigen::Matrix4d dsnarer_q_dsnarer_q = Eigen::Matrix4d::Identity();
        Eigen::Matrix<double, 3, 4> da2_dsnarer_q = 2.0*(Eigen::Matrix<double, 3, 4>() << snarer_r2, snarer_r3, snarer_w, snarer_r1, -snarer_r1, -snarer_w, snarer_r3, snarer_r2, 0.0, -2.0*snarer_r1, -2.0*snarer_r2, 0.0).finished()*dsnarer_q_dsnarer_q;

        // Compute the derivative of the constraint w.r.t. the snared tool states.
        dc_dsnared = Eigen::Matrix<double, 6, 13>::Zero();
        dc_dsnared.block<3,3>(0,0) = Eigen::Matrix3d::Identity(); // position error derivative
        dc_dsnared.block<1,4>(3,3) = a2.transpose()*da1_dsnared_q; // heading dot product derivative
        dc_dsnared.block<1,4>(4,3) = snarer_m.transpose()*da1_dsnared_q;
        //dc_dsnared.block<1,13>(5,0) = the rest is zero

        // Compute the derivative of the constraint w.r.t. the snarer  states.
        dc_dsnarer = Eigen::Matrix<double, 6, 13>::Zero();
        dc_dsnarer.block<3,3>(0,0) = -Eigen::Matrix3d::Identity(); // position error derivative
        dc_dsnarer.block<1,4>(3,3) = a1.transpose()*da2_dsnarer_q; // heading dot product derivative
        dc_dsnarer.block<1,3>(4,7) = a1.transpose();
        dc_dsnarer.block<1,4>(5,3) = snarer_m.transpose()*da2_dsnarer_q;
        dc_dsnarer.block<1,3>(5,7) = a2.transpose();

    }

private:
    GrasperType &my_grasper_rod;

public:
    GraspConstraint(GrasperType &grasper_rod, double offset_from_tip) : my_grasper_rod(grasper_rod), ConstraintEntity<GraspConstraint<GrasperType>, GrasperType>(offset_from_tip, grasper_rod) {};



    /**
     * This function applies the grasp state transition and computes the grasp constraint, along with its derivative.
     *
     */
    template<int state_dimension, int constraint_dimension, class ParentType>
    void apply(Constraint<constraint_dimension, state_dimension> &constraint,
               State<state_dimension> &left_state,
               const ParentType &snared_rod) {


        State<state_dimension> right_state;

        // First apply the state transition.
        Quaternion q_left_snared(left_state.vector().template segment<4>(snared_rod.getIndex()+3)); // snared quaternion
        Quaternion q_left_snarer(left_state.vector().template segment<4>(my_grasper_rod.getIndex()+3)); // snarer quaternion

        Eigen::Vector3d snarer_m = left_state.vector().template segment<3>(my_grasper_rod.getIndex()+7); // snarer moment


        // Compute the state transition matrix
        Eigen::Vector3d a1 = q_left_snared*Eigen::Vector3d::UnitZ();
        Eigen::Vector3d a2 = q_left_snarer*Eigen::Vector3d::UnitZ();
        double a1_dot_a2 = a1.dot(a2);
        double c = 1.0/(1 - a1_dot_a2*a1_dot_a2);
        Eigen::Matrix3d a1a1T = a1*a1.transpose();
        Eigen::Matrix3d a2a2T = a2*a2.transpose();
        Eigen::Matrix3d a1a2T = a1*a2.transpose();
        Eigen::Matrix3d a2a1T = a2*a1.transpose();
        Eigen::Matrix3d K = a1a1T - a1_dot_a2*a1a2T + a2a2T - a1_dot_a2*a2a1T;
        Eigen::Matrix3d L = Eigen::Matrix3d::Identity() - c*K; // this is I - A*pinv(A), where A = [a1 a2]

        Eigen::Matrix<double, state_dimension, state_dimension> transition_matrix = Eigen::Matrix<double, state_dimension, state_dimension>::Identity();
        //transition_matrix.template block<7,7>(snared_rod.getIndex()(0), my_grasper_rod.getIndex()(0));
        transition_matrix.template block<3,3>(snared_rod.getIndex()+7, my_grasper_rod.getIndex()+7) = L;
        transition_matrix.template block<3,3>(snared_rod.getIndex()+10, my_grasper_rod.getIndex()+10) = Eigen::Matrix3d::Identity();


        // Compute the parts needed for the derivative transition matrix.
        Eigen::Matrix<double,13,13> dgrasp_transition_dsnared;
        Eigen::Matrix<double,13,13> dgrasp_transition_dsnarer;
        computeGraspStateTransitionDerivative(left_state.vector().template segment<13>(my_grasper_rod.getIndex()),
                                              left_state.vector().template segment<13>(snared_rod.getIndex()),
                                              dgrasp_transition_dsnared,
                                              dgrasp_transition_dsnarer); // this function does some extra work that could be optimized...

        // Build the derivative transition matrix using the matrices computed from computeGraspStateTransitionDerivative().
        Eigen::Matrix<double, state_dimension, state_dimension> derivative_transition_matrix = Eigen::Matrix<double, state_dimension, state_dimension>::Identity();
        derivative_transition_matrix.template block<13,13>(snared_rod.getIndex(), snared_rod.getIndex()) = dgrasp_transition_dsnared;
        derivative_transition_matrix.template block<13,13>(snared_rod.getIndex(), my_grasper_rod.getIndex()) = dgrasp_transition_dsnarer;


        // Propagate the state and derivative across the discontinuity using the transition matrix for the state and derivative.
        right_state.vector() = transition_matrix*left_state.vector();
        right_state.derivative() = derivative_transition_matrix*left_state.derivative();


        left_state = right_state; // set the transition


        // Apply the constraint.
        constraint.vector().template segment<3>(this->getIndex()) = left_state.vector().template segment<3>(snared_rod.getIndex()) - left_state.vector().template segment<3>(my_grasper_rod.getIndex()); // position coincidence
        constraint.vector()(this->getIndex()+3) = a1_dot_a2;
        constraint.vector()(this->getIndex()+4) = a1.dot(snarer_m);
        constraint.vector()(this->getIndex()+5) = a2.dot(snarer_m);


        // Compute the parts needed for the constraint derivative matrix.
        Eigen::Matrix<double,6,13> dcgrasp_dsnared;
        Eigen::Matrix<double,6,13> dcgrasp_dsnarer;
        computeGraspConstraintDerivative(left_state.vector().template segment<13>(my_grasper_rod.getIndex()),
                                              left_state.vector().template segment<13>(snared_rod.getIndex()),
                                              dcgrasp_dsnared,
                                              dcgrasp_dsnarer); // this function does some extra work that could be optimized...


        // Compute the derivative of the constraints w.r.t the current state using the parts computed by computeGraspConstraintDerivative().
        Eigen::Matrix<double, 6, state_dimension> constraint_derivative = Eigen::Matrix<double, 6, state_dimension>::Zero();
        constraint_derivative.template block<6,13>(0, snared_rod.getIndex()) = dcgrasp_dsnared;
        constraint_derivative.template block<6,13>(0, my_grasper_rod.getIndex()) = dcgrasp_dsnarer;

        // Compute the derivative of the constraints w.r.t. the initial state.
        constraint.derivative().template block<6, state_dimension>(this->getIndex(), 0) = constraint_derivative*right_state.derivative();
    }


    /**
     * Returns the subvector of the constraint vector that corresponds to this constraint.
     *
     */
    template<int n>
    Eigen::Matrix<double, dimension, 1> interpret(const Eigen::Matrix<double, n, 1> &constraint_vector) const {
        return constraint_vector.template segment<dimension>(this->getIndex());
    }




};



/**
 * This implements the grasp constraint where the snare tip position and tool body position are coincident,
 * the snare tip heading and the body heading are perpendicular, the internal load and moment is added to
 * the internal load and moment of the tool.
 *
 */
template<class GrasperType>
class FullGraspConstraint : public ConstraintEntity<FullGraspConstraint<GrasperType>, GrasperType> {
public:
    static constexpr int dimension = 6;


private:
    GrasperType &my_grasper_rod;

public:
    FullGraspConstraint(GrasperType &grasper_rod, double offset_from_tip) : my_grasper_rod(grasper_rod), ConstraintEntity<FullGraspConstraint<GrasperType>, GrasperType>(offset_from_tip, grasper_rod) {}




    /**
     * This function applies the grasp state transition and computes the grasp constraint, along with its derivative.
     *
     */
    template<int state_dimension, int constraint_dimension, class ParentType>
    void apply(Constraint<constraint_dimension, state_dimension> &constraint,
               State<state_dimension> &left_state,
               const ParentType &snared_rod) {


        State<state_dimension> right_state;

        Eigen::Vector3d p_left_snared = left_state.vector().template segment<3>(snared_rod.getIndex());
        Eigen::Vector3d p_left_snarer = left_state.vector().template segment<3>(my_grasper_rod.getIndex());
        Quaternion q_left_snared(left_state.vector().template segment<4>(snared_rod.getIndex()+3)); // snared quaternion vector component
        Quaternion q_left_snarer(left_state.vector().template segment<4>(my_grasper_rod.getIndex()+3)); // snarer quaternion vector component

        Quaternion rotator(0.7071, 0, 0.7071, 0);

        Quaternion q_left_snarer_rotated = q_left_snarer*rotator;


        Eigen::Matrix<double, state_dimension, state_dimension> transition_matrix = Eigen::Matrix<double, state_dimension, state_dimension>::Identity();
        transition_matrix.template block<6,6>(snared_rod.getIndex()+7, my_grasper_rod.getIndex()+7) = Eigen::Matrix<double, 6, 6>::Identity();


        // Propagate the state and derivative across the discontinuity using the transition matrix for the state and derivative.
        right_state.vector() = transition_matrix*left_state.vector();
        right_state.derivative() = transition_matrix*left_state.derivative();


        left_state = right_state; // set the transition


        // Apply the constraint.
        constraint.vector().template segment<3>(this->getIndex()) = p_left_snared - p_left_snarer; // position coincidence
        constraint.vector().template segment<3>(this->getIndex()+3) = q_left_snared.vec() - q_left_snarer_rotated.vec(); // quaternion vector coincidence



        double w = rotator.w();
        double r1 = rotator.vec()(0);
        double r2 = rotator.vec()(1);
        double r3 = rotator.vec()(2);

        Eigen::Matrix<double, 3, 4> rot_derivative_matrix;
        rot_derivative_matrix << r1, w, r3, -r2, r2, -r3, w, r1, r3, r2, -r1, w;

        // Compute the derivative of the constraints w.r.t the current state using the parts computed by computeGraspConstraintDerivative().
        Eigen::Matrix<double, 6, state_dimension> constraint_derivative = Eigen::Matrix<double, 6, state_dimension>::Zero();
        constraint_derivative.template block<3,3>(0, snared_rod.getIndex()) = Eigen::Matrix3d::Identity();
        constraint_derivative.template block<3,3>(3, snared_rod.getIndex()+4) = Eigen::Matrix3d::Identity();
        constraint_derivative.template block<3,3>(0, my_grasper_rod.getIndex()) = -Eigen::Matrix3d::Identity();
        constraint_derivative.template block<3,4>(3, my_grasper_rod.getIndex()+3) = -(rot_derivative_matrix);


        // Compute the derivative of the constraints w.r.t. the initial state.
        constraint.derivative().template block<6, state_dimension>(this->getIndex(),0) = constraint_derivative*right_state.derivative();

    }


    /**
     * Returns the subvector of the constraint vector that corresponds to this constraint.
     *
     */
    template<int n>
    Eigen::Matrix<double, dimension, 1> interpret(const Eigen::Matrix<double, n, 1> &constraint_vector) const {
        return constraint_vector.template segment<dimension>(this->getIndex());
    }




};






/**
 * This implements the constraint where the internal moment and force a rod that isn't grasping another have to be zero at the tip.
 *
 */
class LoadFreeConstraint : public TipConstraintEntity<LoadFreeConstraint> {
public:
    static constexpr int dimension = 6;

public:
    LoadFreeConstraint() : TipConstraintEntity() {};

    /**
     * This function applies the grasp state transition and computes the grasp constraint, along with its derivative.
     *
     */
    template<int state_dimension, int constraint_dimension>
    void apply(Constraint<constraint_dimension, state_dimension> &constraint,
               State<state_dimension> &state,
               const DerivativeBase &parent) {


        // Apply the constraint to the state vector.
        constraint.vector().template segment<3>(this->getIndex()) = state.vector().template segment<3>(parent.getIndex()+7); // pick off the moment
        constraint.vector().template segment<3>(this->getIndex()+3) = state.vector().template segment<3>(parent.getIndex()+10); // pick off the load

        constraint.derivative().template block<6, state_dimension>(this->getIndex(), 0) = state.derivative().template block<6, state_dimension>(parent.getIndex()+7, 0); // pick off the part of the state derivative that corresponds to the load at the tip

    }


    /**
     * Returns the subvector of the constraint vector that corresponds to this constraint.
     *
     */
    template<int n>
    Eigen::Matrix<double, dimension, 1> interpret(const Eigen::Matrix<double, n, 1> &constraint_vector) const {
        return constraint_vector.template segment<dimension>(this->getIndex());
    }


};


/**
 * Constrains the length of a rod.
 *
 */
class LengthConstraint : public BaseConstraintEntity<LengthConstraint> {
public:
    static const int dimension = 1;

private:
    double my_length;

public:
    LengthConstraint(double length) : my_length(length), BaseConstraintEntity() {};

    double &length() { return my_length; };

    /**
     * Applies the base pose constraint.
     *
     */
    template<class ParentEntityType, int state_dimension, int constraint_dimension>
    void apply(Constraint<constraint_dimension, state_dimension> &constraint,
               State<state_dimension> &state,
               const ParentEntityType &parent) {

        constraint.vector()(this->getIndex()) = parent.interpret(state.vector()).length() - my_length;

        constraint.derivative()(this->getIndex(), parent.getIndex()+13) = 1.0;
    };

    /**
     * Returns the subvector of the constraint vector that corresponds to this constraint.
     *
     */
    template<int constraint_dimension>
    double interpret(const Eigen::Matrix<double, constraint_dimension, 1> &constraint_vector) const {
        return constraint_vector(this->getIndex());
    }
};




/**
 * This implements a constraint on a rod's base pose.  The constraint vector is packed as [px-pdx, py-pdy, pz-pdz, q'*q-1, qx-qdx, qy-qdy, qz-qdz], where
 * px and pdx are the x component of the rod actual and desired position, respectively.  q is the rod's actual quaternion, and qx and qdx are the x component
 * of the actual and desired quaternion, respectively.  The constraint q'*q-1 keeps the rod's quaternion at unit-length.
 *
 * It has dimension 7 because it also enforces the base quaternion to be unit-length.
 *
 */
class BasePoseConstraint : public BaseConstraintEntity<BasePoseConstraint> {
public:
    static const int dimension = 7;

private:
    Eigen::Vector3d my_position;
    Quaternion my_quaternion;

public:
    BasePoseConstraint(const Eigen::Vector3d &position, const Quaternion &quaternion) : my_position(position), my_quaternion(quaternion), BaseConstraintEntity() {};



    /**
     * This overloads the isActive() function of the BaseEntity so that any arc-length less than the location of this constraint activates this constraint.  This basically extends all pose constraints to be located at the rear-most arc-length.
     *
     */
    bool isActive(double arclength) { return (arclength <= my_interval(1)); };


    /**
     * Use these functions to access and set the base pose and quaternion.
     *
     */
    const Eigen::Vector3d &position() const { return my_position; };
    const Quaternion &quaternion() const { return my_quaternion; };


    /**
     * Applies the base pose constraint.
     *
     */
    template<int state_dimension, int constraint_dimension>
    void apply(Constraint<constraint_dimension, state_dimension> &constraint,
               State<state_dimension> &state,
               const DerivativeBase &parent) {

        Eigen::Vector4d quaternion = state.vector().template segment<4>(parent.getIndex()+3);

        // Apply the constraint to the state vector.
        constraint.vector().template segment<3>(this->getIndex()) = state.vector().template segment<3>(parent.getIndex()) - my_position;
        constraint.vector()(this->getIndex()+3) = (quaternion.squaredNorm() - 1.0);
        constraint.vector().template segment<3>(this->getIndex()+4) = state.vector().template segment<3>(parent.getIndex()+4) - my_quaternion.vec();

        Eigen::Matrix<double, 7, 7> derivative_matrix = Eigen::Matrix<double, 7, 7>::Identity();
        derivative_matrix.template block<1,4>(3,3) = 2.0*quaternion.transpose();

        constraint.derivative().template block<7, 7>(this->getIndex(), parent.getIndex()) = derivative_matrix*state.derivative().template block<7, 7>(parent.getIndex(), parent.getIndex());

    };

    /**
     * Returns the subvector of the constraint vector that corresponds to this constraint.
     *
     */
    template<int n>
    Eigen::Matrix<double, dimension, 1> interpret(const Eigen::Matrix<double, n, 1> &constraint_vector) const {
        return constraint_vector.template segment<dimension>(this->getIndex());
    }
};







/**
 * This implements an RCM constraint on a rod's body.
 *
 */
template<class RCMType>
class RCMConstraint : public ConstraintEntity<RCMConstraint<RCMType>, RCMType> {
public:
    static constexpr int dimension = 3;

private:
    RCMType &my_rcm_state;
    Eigen::Vector3d my_rcm_point;

public:


    /**
     * Class constructor.
     *
     */
    RCMConstraint(RCMType &rcm_state, const Eigen::Vector3d &rcm_point) : my_rcm_state(rcm_state), my_rcm_point(rcm_point), ConstraintEntity<RCMConstraint<RCMType>, RCMType>(0.0, rcm_state) {};

    /**
     * Use this to access and set the rcm position.
     *
     */
    Eigen::Vector3d &position() { return my_rcm_point; };


    /**
     * This function applies the grasp state transition and computes the grasp constraint, along with its derivative.
     *
     */
    template<int state_dimension, int constraint_dimension, class ParentType>
    void apply(Constraint<constraint_dimension, state_dimension> &constraint,
               State<state_dimension> &left_state,
               const ParentType &rod) {

        Eigen::Vector3d parent_rod_position = left_state.vector().template segment<3>(rod.getIndex()); // position of the rod at the current RCM point

        Quaternion q_rod(left_state.vector().template segment<4>(rod.getIndex()+3)); // rod quaternion
        Eigen::Vector3d heading = q_rod*Eigen::Vector3d::UnitZ();

        constraint.vector().template segment<3>(this->getIndex()) = parent_rod_position - my_rcm_point;
        constraint.derivative().template block<3, state_dimension>(this->getIndex(),0) = left_state.derivative().template block<3, state_dimension>(rod.getIndex(),0);
        constraint.derivative().template block<3, 1>(this->getIndex(), my_rcm_state.getIndex()) = heading;

    }

    // Overloaded from BaseEntity
    template<int n>
    BaseEntity::Interval getLocalInterval(const State<n> &state) const {

        return BaseEntity::Interval(my_rcm_state.template interpret<n>(state.vector()), my_rcm_state.template interpret<n>(state.vector()));
    };





    /**
     * Returns the subvector of the constraint vector that corresponds to this constraint.
     *
     */
    template<int n>
    Eigen::Matrix<double, dimension, 1> interpret(const Eigen::Ref<Eigen::Matrix<double, n, 1>> constraint_vector) const {
        return constraint_vector.template segment<dimension>(this->getIndex());
    }
};






/**
 * This constrains the base position and enforces a rod quaternion to be unit length.
 *
 */
class BasePositionConstraint : public BaseConstraintEntity<BasePositionConstraint> {
public:
    static constexpr int dimension = 4;

private:
    Eigen::Vector3d my_entry_point;

public:
    BasePositionConstraint(const Eigen::Vector3d &entry_point) : BaseConstraintEntity<BasePositionConstraint>(), my_entry_point(entry_point) {}


    /**
     * Use this to set and retrieve the desired constraint position.
     *
     */
    Eigen::Vector3d &position() { return my_entry_point; }



    /**
     * This function applies the grasp state transition and computes the grasp constraint, along with its derivative.
     *
     */
    template<int state_dimension, int constraint_dimension, class ParentType>
    void apply(Constraint<constraint_dimension, state_dimension> &constraint,
               State<state_dimension> &state,
               const ParentType &parent_rod) {


        Eigen::Vector3d parent_rod_position = state.vector().template segment<3>(parent_rod.getIndex());
        Quaternion parent_rod_quaternion(state.vector().template segment<4>(parent_rod.getIndex()+3));

        // Apply the constraint to the state vector.
        constraint.vector().template segment<3>(this->getIndex()) = parent_rod_position - my_entry_point;
        constraint.vector()(this->getIndex()+3) = parent_rod_quaternion.squaredNorm() - 1.0; // enforce the quaternion to be unit length

        Eigen::Matrix<double, dimension, 7> derivative_matrix = Eigen::Matrix<double, dimension, 7>::Zero();
        derivative_matrix.template block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        derivative_matrix.template block<1,4>(3,3) = 2.0*parent_rod_quaternion.transpose();

        constraint.derivative().template block<dimension, 7>(this->getIndex(), parent_rod.getIndex()) = derivative_matrix*state.derivative().template block<7, 7>(parent_rod.getIndex(), parent_rod.getIndex());
    }




    /**
     * Returns the subvector of the constraint vector that corresponds to this constraint.
     *
     */
    template<int n>
    Eigen::Matrix<double, dimension, 1> interpret(const Eigen::Ref<Eigen::Matrix<double, n, 1>> constraint_vector) const {
        return constraint_vector.template segment<dimension>(this->getIndex());
    }

};



/**
 * This implements an applied load on the body of a rod.  It's not really a constraint because it's always satisfied.
 *
 */
template<class ForceType>
class AppliedLoad : public ConstraintEntity<ForceType> {
public:
    static constexpr int dimension = 3;



private:
    ForceType &my_force_state;

public:
    /**
     * Class constructor that applies a force of the given type to rod at the given location.  The location is specified
     * in arc-length relative to the base of the rod.
     *
     */
    AppliedLoad(ForceType &force_state, double offset_from_tip) : my_force_state(force_state), ConstraintEntity<ForceType>(offset_from_tip) {};


    /**
     * This function applies the load state transition and computes the load constraint (which is always zero I think), along with its derivative.
     *
     */
    template<int state_dimension, int constraint_dimension, class ParentType>
    void apply(Constraint<constraint_dimension, state_dimension> &constraint,
               State<state_dimension> &left_state,
               const ParentType &rod) {

        State<state_dimension> right_state;

        Eigen::Matrix<double, state_dimension, state_dimension> state_transition_matrix = Eigen::Matrix<double, state_dimension, state_dimension>::Identity();

        state_transition_matrix.template block<3,3>(rod.getIndex()+10, my_force_state.getIndex()) = Eigen::Matrix3d::Identity();

        right_state.vector() = state_transition_matrix*left_state.vector();
        right_state.derivative() = state_transition_matrix*left_state.derivative();


        left_state = right_state;

        constraint.vector().template segment<3>(this->getIndex()) = my_force_state.interpret(left_state.vector()) -  my_force_state.force();

        constraint.derivative().template block<3, 3>(this->getIndex(), my_force_state.getIndex()) = Eigen::Matrix3d::Identity();
    }





    /**
     * Returns the subvector of the constraint vector that corresponds to this constraint.
     *
     */
    template<int n>
    Eigen::Matrix<double, dimension, 1> interpret(const Eigen::Matrix<double, n, 1> &constraint_vector) const {
        return constraint_vector.template segment<dimension>(this->getIndex());
    }



};









/**
 * This implements a constraint on a rod's tip pose.  The constraint vector is packed as [px-pdx, py-pdy, pz-pdz, q'*q-1, qx-qdx, qy-qdy, qz-qdz], where
 * px and pdx are the x component of the rod actual and desired position, respectively.  q is the rod's actual quaternion, and qx and qdx are the x component
 * of the actual and desired quaternion, respectively.  The constraint q'*q-1 keeps the rod's quaternion at unit-length.
 *
 * It has dimension 7 because it also enforces the base quaternion to be unit-length.
 *
 * This is useful for inverse kinematics.
 *
 */
class TipPoseConstraint : public TipConstraintEntity<TipPoseConstraint> {
public:
    static const int dimension = 6;

    Eigen::Vector3d my_position;
    Quaternion my_quaternion;

public:
    TipPoseConstraint(const Eigen::Vector3d &position, const Quaternion &quaternion) : my_position(position), my_quaternion(quaternion), TipConstraintEntity() {};




    Eigen::Vector3d &position() { return my_position; };
    Eigen::Vector3d position() const { return my_position; };
    Quaternion &quaternion() { return my_quaternion; };
    Quaternion quaternion() const { return my_quaternion; };





    template<int state_dimension, int constraint_dimension>
    void apply(Constraint<constraint_dimension, state_dimension> &constraint,
               State<state_dimension> &state,
               const DerivativeBase &parent) {


        Eigen::Vector4d quaternion = state.vector().template segment<4>(parent.getIndex()+3);

        //std::cout << "C: arc-length pose " << this->getIndex()(0) << std::endl;

       // std::cout << "inverse kinematics" << std::endl;
        //std::cout << "actual " << quaternion.transpose() << std::endl;
        //std::cout << "desired " << my_quaternion.transpose() << std::endl;


        // Apply the constraint to the state vector.
        constraint.vector().template segment<3>(this->getIndex()) = state.vector().template segment<3>(parent.getIndex()) - my_position; // position constraint
        //constraint.vector()(this->getIndex()+3) = quaternion.squaredNorm() - 1.0;
        constraint.vector().template segment<3>(this->getIndex()+3) = quaternion.template segment<3>(1) - my_quaternion.vec();



        Eigen::Matrix<double, 6, 7> derivative_matrix = Eigen::Matrix<double, 6, 7>::Zero();
        derivative_matrix.template block<3,3>(0,0) = Eigen::Matrix<double, 3, 3>::Identity();
        derivative_matrix.template block<3,3>(3,4) = Eigen::Matrix<double, 3, 3>::Identity();


        //derivative_matrix.template block<1,4>(3,3) = 2.0*quaternion.transpose();

        constraint.derivative().template block<dimension, state_dimension>(this->getIndex(), parent.getIndex()) = derivative_matrix*state.derivative().template block<7, state_dimension>(parent.getIndex(), 0);


    };

    /**
     * Returns the subvector of the constraint vector that corresponds to this constraint.
     *
     */
    template<int n>
    Eigen::Matrix<double, dimension, 1> interpret(const Eigen::Matrix<double, n, 1> &constraint_vector) const {
        return constraint_vector.template segment<dimension>(this->getIndex());
    }



};


/**
 * This implements a constraint at the base that only constrians the quaternion to be unit-length.  This is used if you are using the tip pose constraint for inverse kinematics.
 *
 */
class BaseFreeConstraint : public BaseConstraintEntity<BaseFreeConstraint> {
public:
    static const int dimension = 1;


public:
    BaseFreeConstraint() : BaseConstraintEntity() {};



    // This overloads the isActive() function of the BaseEntity so that any arc-length less than the location of this constraint activates this constraint.  This basically extends the position constraint to be located at the rear-most arc-length.
    bool isActive(double arclength) { return (arclength <= my_interval(1)); };


    template<int state_dimension, int constraint_dimension>
    void apply(Constraint<constraint_dimension, state_dimension> &constraint,
               State<state_dimension> &state,
               const DerivativeBase &parent) {

        //std::cout << "C: position & roll " << this->getIndex()(0) << std::endl;

        Quaternion rod_quaternion(state.vector().template segment<4>(parent.getIndex()+3));


        // Apply the constraint to the state vector.
        constraint.vector()(this->getIndex()) = rod_quaternion.squaredNorm() - 1.0;



        Eigen::Matrix<double, dimension, 4> derivative_matrix = 2.0*rod_quaternion.transpose();




        constraint.derivative().template block<dimension, 4>(this->getIndex(), parent.getIndex()+3) = derivative_matrix*state.derivative().template block<4, 4>(parent.getIndex()+3, parent.getIndex()+3);

    };

    /**
     * Returns the subvector of the constraint vector that corresponds to this constraint.
     *
     */
    template<int n>
    Eigen::Matrix<double, dimension, 1> interpret(const Eigen::Matrix<double, n, 1> &constraint_vector) const {
        return constraint_vector.template segment<dimension>(this->getIndex());
    }



};


/**
 * This implements a constraint at the base that only constrians the quaternion to be unit-length.  This is used if you are using the tip pose constraint for inverse kinematics.
 *
 */
class BaseFreeRollConstraint : public BaseConstraintEntity<BaseFreeRollConstraint> {
public:
    static const int dimension = 2;


public:
    BaseFreeRollConstraint() : BaseConstraintEntity() {};



    // This overloads the isActive() function of the BaseEntity so that any arc-length less than the location of this constraint activates this constraint.  This basically extends the position constraint to be located at the rear-most arc-length.
    bool isActive(double arclength) { return (arclength <= my_interval(1)); };


    template<int state_dimension, int constraint_dimension>
    void apply(Constraint<constraint_dimension, state_dimension> &constraint,
               State<state_dimension> &state,
               const DerivativeBase &parent) {


        Quaternion rod_quaternion(state.vector().template segment<4>(parent.getIndex()+3));
        Eigen::Vector3d rod_heading = rod_quaternion*Eigen::Vector3d::UnitZ();


        // Apply the constraint to the state vector.
        constraint.vector()(this->getIndex()) = rod_quaternion.squaredNorm() - 1.0;
        constraint.vector()(this->getIndex()+1) = rod_heading.dot(rod_quaternion.vec());



        Eigen::Matrix<double, dimension, 4> derivative_matrix = Eigen::Matrix<double, dimension, 4>::Zero();

        derivative_matrix.template block<1,4>(0,0) = 2.0*rod_quaternion.transpose();
        derivative_matrix.template block<1,4>(1,0) = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0).transpose();



        constraint.derivative().template block<dimension, 4>(this->getIndex(), parent.getIndex()+3) = derivative_matrix*state.derivative().template block<4, 4>(parent.getIndex()+3, parent.getIndex()+3);



    };

    /**
     * Returns the subvector of the constraint vector that corresponds to this constraint.
     *
     */
    template<int n>
    Eigen::Matrix<double, dimension, 1> interpret(const Eigen::Matrix<double, n, 1> &constraint_vector) const {
        return constraint_vector.template segment<dimension>(this->getIndex());
    }



};






class BodyMomentConstraint : public BaseConstraintEntity<BodyMomentConstraint> {
public:
    static const int dimension = 1;

private:
    double m_moment;
    Eigen::Vector3d m_direction; // unit-length direction in body frame

public:
    BodyMomentConstraint(const Eigen::Vector3d &direction, double moment) : BaseConstraintEntity(),  m_moment(moment), m_direction(direction) {};

    double &moment() { return m_moment; };
    Eigen::Vector3d &direction() { return m_direction; };




    // This overloads the isActive() function of the BaseEntity so that any arc-length less than the location of this constraint activates this constraint.  This basically extends the position constraint to be located at the rear-most arc-length.
    bool isActive(double arclength) { return (arclength <= my_interval(1)); };


    template<int state_dimension, int constraint_dimension>
    void apply(Constraint<constraint_dimension, state_dimension> &constraint,
               State<state_dimension> &state,
               const DerivativeBase &parent) {

        Eigen::Vector3d moment(state.vector().template segment<3>(parent.getIndex()+7)); // world frame

        Quaternion quaternion(state.vector().template segment<4>(parent.getIndex()+3)); // maps body to world

        Quaternion inverse_quaternion = quaternion.conjugate();


        Eigen::Vector3d moment_in_body_frame = inverse_quaternion.asRot()*moment;


        Eigen::Matrix3d skew_moment; skew_moment << 0.0, -moment.z(), moment.y(), moment.z(), 0.0, -moment.x(), -moment.y(), moment.x(), 0.0;


        Eigen::Matrix<double, 3, 4> quaternion_partial_derivative;
        quaternion_partial_derivative.leftCols<1>() = 2.0*(inverse_quaternion.w()*moment + inverse_quaternion.vec().cross(moment));
        quaternion_partial_derivative.rightCols<3>() = -2.0*(-moment*inverse_quaternion.vec().transpose() + moment.dot(inverse_quaternion.vec())*Eigen::Matrix3d::Identity() + inverse_quaternion.vec()*moment.transpose() - inverse_quaternion.w()*skew_moment);



        // Apply the constraint to the state vector.
        constraint.vector()(this->getIndex()) = moment_in_body_frame.dot(m_direction) - m_moment;


        constraint.derivative().template block<1,4>(this->getIndex(), parent.getIndex()+3) = m_direction.transpose()*quaternion_partial_derivative;

        constraint.derivative().template block<1,3>(this->getIndex(), parent.getIndex()+7) = m_direction.transpose()*inverse_quaternion.asRot();


    };

    /**
     * Returns the subvector of the constraint vector that corresponds to this constraint.
     *
     */
    template<int n>
    double interpret(const Eigen::Matrix<double, n, 1> &constraint_vector) const {
        return constraint_vector(this->getIndex());
    }



};








class WorldForceConstraint : public BaseConstraintEntity<WorldForceConstraint> {
public:
    static const int dimension = 1;

private:
    double m_force;
    Eigen::Vector3d m_direction; // unit-length direction in body frame

public:
    WorldForceConstraint(const Eigen::Vector3d &direction, double force) : BaseConstraintEntity(), m_force(force), m_direction(direction) {};

    double &force() { return m_force; };
    Eigen::Vector3d &direction() { return m_direction; };


    // This overloads the isActive() function of the BaseEntity so that any arc-length less than the location of this constraint activates this constraint.  This basically extends the position constraint to be located at the rear-most arc-length.
    bool isActive(double arclength) { return (arclength <= my_interval(1)); };


    template<int state_dimension, int constraint_dimension>
    void apply(Constraint<constraint_dimension, state_dimension> &constraint,
               State<state_dimension> &state,
               const DerivativeBase &parent) {

        Eigen::Vector3d force(state.vector().template segment<3>(parent.getIndex()+10)); // world frame


        // Apply the constraint to the state vector.
        constraint.vector()(this->getIndex()) = m_direction.dot(force) - m_force;

        constraint.derivative().template block<1,3>(this->getIndex(), parent.getIndex()+10) = m_direction.transpose();

    };

    /**
     * Returns the subvector of the constraint vector that corresponds to this constraint.
     *
     */
    template<int n>
    double interpret(const Eigen::Matrix<double, n, 1> &constraint_vector) const {
        return constraint_vector(this->getIndex());
    }



};



#endif /* ConstraintEntities_h */

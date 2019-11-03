//
//  DerivativeEntities.h
//  SnareNeedleModel
//
//  Created by Art Mahoney on 8/3/16.
//  Copyright © 2016 Art Mahoney. All rights reserved.
//

#ifndef DerivativeEntities_h
#define DerivativeEntities_h

#include "Entities.h"
#include "RodProperties.h"
#include "Integrator.h"
#include "Tools.h"
#include "Quaternion.h"


class RodBase {}; // this is a parent class that is inherited by all Rods regardless of their constraints


/**
 * This implements a cosserat rod as a DerivativeEntity.  A rod consists of a 13 dimensional state.
 *
 */
template<class ...Ts>
class Rod : public DerivativeEntity<Rod<Ts...>, Ts...>, public RodBase {
public:
    static constexpr int dimension = 13;
    

    /**
     * This provides an interface to access elements of a rod's state with position(), quaternion(), etc. functions.  This
     * is returned from this class's interpret() function.
     *
     */
    class StateRef : public Eigen::Ref<Eigen::Matrix<double, dimension, 1> > {
    public:
        StateRef(Eigen::Ref<Eigen::Matrix<double, dimension, 1> > state_ref) : Eigen::Ref<Eigen::Matrix<double, dimension, 1> >(state_ref) {};
        
        Eigen::Ref<Eigen::Vector3d> position() { return (*this).template segment<3>(0); };
        Eigen::Ref<Eigen::Vector4d> quaternion() { return (*this).template segment<4>(3); };
        Eigen::Ref<Eigen::Vector3d> moment() { return (*this).template segment<3>(7); };
        Eigen::Ref<Eigen::Vector3d> force() { return (*this).template segment<3>(10); };
    };

    
    /**
     * This computes the arc length derivative of a cosserat rod.  It's derived from the DerivativeFunction defined in the
     * Integrator class.  It's used by the Integrator class to integrate the state in arc-length.  It returns a SubState
     * class which contains only the state vector and derivative that correspond to the rod.  This helps the integrator
     * take advantage of some of the snare-system's structure to reduce computation.
     *
     */
    class DerivativeFunctor {
    private:
        const RodProperties &my_rod_properties;
        
    public:
        DerivativeFunctor(const RodProperties &rod_properties) : my_rod_properties(rod_properties) {};
        
        template<class StateWindow>
        void operator()(const double &time, const StateWindow &state, StateWindow &derivative) const {
            my_rod_properties.template getDerivative(state, derivative);
            return;
        }
    };
    
    
private:
    RodProperties my_rod_properties;
    State<dimension, dimension> initial_state;
    
public:
    Rod(const RodProperties &rod_properties, Ts&... ts) : my_rod_properties(rod_properties), initial_state(), DerivativeEntity<Rod<Ts...>, Ts...>(BaseEntity::Interval(0.0,0.0), ts...) {
    

        BaseEntity::setLocalInterval(BaseEntity::Interval(-rod_properties.getLength(),0.0));
        
    };


    /**
     * Returns the identity element for the partial derivative.
     *
     */
    Eigen::Matrix<double, dimension, dimension> derivativeIdentity(const Eigen::Matrix<double, dimension, 1> &state_vector) {
        //Eigen::Matrix<double, 4, 1> my_quaternion = state_vector.template segment<4>(3).normalized();
        Eigen::Matrix<double, dimension, dimension> derivative_identity = Eigen::Matrix<double, dimension, dimension>::Identity();
        //derivative_identity.template block<4,4>(3,3).noalias() = Eigen::Matrix<double, 4, 4>::Identity() - my_quaternion*my_quaternion.transpose();
        return derivative_identity;
    }
    
    


    DerivativeFunctor derivative() {
        return DerivativeFunctor(this->my_rod_properties);
    }
    

    /**
     * Returns a reference state that corresponds to the location where this rod's state lies in the full state vector.
     *
     */
    template<class T>
    StateRef interpret(T &state_vector) const {
        return StateRef(state_vector.template segment<dimension>(this->getIndex()));
    }
    
    template<class T>
    StateRef interpret(Eigen::Ref<T> state_vector) const {
        return StateRef(state_vector.template segment<dimension>(this->getIndex()));
    }
    

    
};









template<class ...Ts>
class VariableLengthRod : public DerivativeEntity<VariableLengthRod<Ts...>, Ts...> {
public:
    static constexpr int dimension = 14;

    /**
     * This provides an interface to access elements of a rod's state with position(), quaternion(), etc. functions.  This
     * is returned from this class's interpret() function.
     *
     */
    class StateRef : public Eigen::Ref<Eigen::Matrix<double, dimension, 1> > {
    public:
        StateRef(Eigen::Ref<Eigen::Matrix<double, dimension, 1> > state_ref) : Eigen::Ref<Eigen::Matrix<double, dimension, 1> >(state_ref) {};
        
        Eigen::Ref<Eigen::Vector3d> position() { return (*this).template segment<3>(0); };
        Eigen::Ref<Eigen::Vector4d> quaternion() { return (*this).template segment<4>(3); };
        Eigen::Ref<Eigen::Vector3d> moment() { return (*this).template segment<3>(7); };
        Eigen::Ref<Eigen::Vector3d> force() { return (*this).template segment<3>(10); };
        double &length() { return (*this)(13); };
    };

    /**
     * This computes the arc length derivative of a cosserat rod.  It's derived from the DerivativeFunction defined in the
     * Integrator class.  It's used by the Integrator class to integrate the state in arc-length.  It returns a SubState
     * class which contains only the state vector and derivative that correspond to the rod.  This helps the integrator
     * take advantage of some of the snare-system's structure to reduce computation.
     *
     */
    class DerivativeFunctor {
    private:
        const RodProperties &my_rod_properties;
        
    public:
        DerivativeFunctor(const RodProperties &rod_properties) : my_rod_properties(rod_properties) {};
        
        template<class StateWindow>
        void operator()(const double &time, const StateWindow &state, StateWindow &derivative) const {
            my_rod_properties.template getDerivative(state, derivative);
            return;
        }
    };

private:
    RodProperties my_rod_properties;

    
public:
    VariableLengthRod(const RodProperties &rod_properties, Ts&...ts) : my_rod_properties(rod_properties), DerivativeEntity<VariableLengthRod<Ts...>, Ts...>(BaseEntity::Interval(0.0,0.0), ts...) {
        
        
    }
    
    
    /**
     * Returns the identity element for the partial derivative.
     *
     */
    Eigen::Matrix<double, dimension, dimension> derivativeIdentity(const Eigen::Matrix<double, dimension, 1> &state_vector) {
        //Eigen::Matrix<double, 4, 1> my_quaternion = state_vector.template segment<4>(3).normalized();
        Eigen::Matrix<double, dimension, dimension> derivative_identity = Eigen::Matrix<double, dimension, dimension>::Identity();
        
        Eigen::Matrix<double, dimension, 1> arc_length_derivative;
        
        my_rod_properties.getDerivative<dimension>(state_vector, arc_length_derivative);
        
        derivative_identity.template block<dimension,1>(0,13) = arc_length_derivative;
        derivative_identity(13,13) = 1.0;
        
        return derivative_identity;
    }
    
    
    // Overloaded from BaseEntity
    template<int n>
    BaseEntity::Interval getLocalInterval(const BaseEntity::Interval &parent_interval, const State<n> &state) const {
        double my_length = state.vector()(this->getIndex()+13);
        
        return BaseEntity::Interval(-my_length, 0.0);
    }
    
    
    DerivativeFunctor derivative() {
        return DerivativeFunctor(this->my_rod_properties);
    }
    
    /**
     * Returns a reference state that corresponds to the location where this rod's state lies in the full state vector.
     *
     */
    template<class T>
    StateRef interpret(T &state_vector) const {
        return StateRef(state_vector.template segment<dimension>(this->getIndex()));
    }
    
    template<class T>
    StateRef interpret(Eigen::Ref<T> state_vector) const {
        return StateRef(state_vector.template segment<dimension>(this->getIndex()));
    }
    
    
    
};













/**
 * This class helps to implement an RCM constraint.  It consists of a 1-dimensional state that measures where the nearest point on a rod
 * is to the RCM point in space.  The closest point is measured in arc-length from the rod's base.  This arc-length state does not vary
 * with arc-length (i.e., it's a parameter), but it's included in the list of states so that it appears in the Jacobian and can be used
 * by the BVP solver in order to satisfy an RCM constraint.
 *
 */
class RCMPoint : public ParameterEntity<RCMPoint> {
public:
    static constexpr int dimension = 1;
    
private:
    

    
public:
    
    /**
     * The location is the arc-length location of the nearest point on a rod's body to the RCM point.  Before solving the BVP, this is just a guess.
     *
     */
    RCMPoint() :  ParameterEntity() {};
    
    
};











/**
 * This represents the applied force as a state.  An AppliedLoad constraint is made between this applied force and the rod that this force is
 * applied to at the location where this force is applied.  Including this force as part of the state (regardless of its magnitude) can be
 * used to compute the system's compliance.
 *
 */
class Force : public DerivativeEntity<Force> {
public:
    static constexpr int dimension = 3;
    
    
private:
    

    /**
     * This implements the arc-length derivative for the integrator... it's always zero.
     *
     */
    class DerivativeFunctor {
    public:
        DerivativeFunctor()  {};
        
        template<class State, class Window>
        void operator()(const double &time, const StateWindow<State, Window> &state, StateWindow<State, Window> &derivative) const {
            return;
        }
    };

    
    Eigen::Vector3d m_force; // stores the applied force
    
public:
    

    Force(const Eigen::Vector3d &force) : m_force(force),  DerivativeEntity<Force>(BaseEntity::Interval(0.0,0.0)) {};
    

    
    
    /**
     * Returns if this state is changing at the specified arc-length.
     *
     */
    bool isActive(double arclength) { return false; }; // this state is never active during integration because it doesn't change with arc length
    
    
    /**
     * Returns the identity element for the partial derivative.
     *
     */
    Eigen::Matrix<double, dimension, dimension> derivativeIdentity(const Eigen::Matrix<double, dimension, 1> &state_vector) {
        
        Eigen::Matrix<double, dimension, dimension> derivative_identity = Eigen::Matrix<double, dimension, dimension>::Identity();
        
        return derivative_identity;
    }

    /**
     * Returns the internal applied force.
     *
     */
    Eigen::Vector3d force() const { return m_force; }
    Eigen::Vector3d &force() { return m_force; }

    /**
     * Returns the derivative for integration.
     *
     */
    DerivativeFunctor derivative() { return DerivativeFunctor(); }
    
    /**
     * Returns the subvector of the complete system state that corresponds to this force.
     *
     */
    template<int n>
    Eigen::Ref<Eigen::Matrix<double, dimension, 1>> interpret(Eigen::Matrix<double, n, 1> &state_vector) {
        return state_vector.template segment<dimension>(my_index);
    }
    
    template<class T>
    Eigen::Ref<Eigen::Matrix<double, dimension, 1>> interpret(Eigen::Ref<T> state_vector) const {
        return state_vector.template segment<dimension>(my_index);
    }
    


    

    
};













#endif /* DerivativeEntities_h */

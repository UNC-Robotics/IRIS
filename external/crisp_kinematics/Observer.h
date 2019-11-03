//
//  Observer.h
//  SnareNeedleModel
//
//  Created by Art Mahoney on 8/15/16.
//  Copyright © 2016 Art Mahoney. All rights reserved.
//

#ifndef Observer_h
#define Observer_h

#include <vector>
#include "State.h"



/**
 * This class is used to watch a specific derivative entity at a desired arc-length.  It just makes it so you can keep track of individual entities
 * without dealing with the entire state vector, which can be large for complicated systems.
 *
 */
template<class SystemType, class DerivativeType>
class Observer {
private:

    // These structs help add up the total dimensions of the entities in a parameter pack Ts2.
    template<class Entity, class... Ts2>
    struct AddupEntityDimensions: std::integral_constant< int, 0 > {};

    template<class Entity, class T2, class... Ts2>
    struct AddupEntityDimensions<Entity, T2, Ts2...> :
    std::integral_constant< int, T2::template getDimension<Entity>()*(std::is_base_of<Entity,T2>::value) + AddupEntityDimensions<Entity, Ts2...>::value > {};


    /**
     * This functor helps construct a Jacobian matrix with columns that correspond to a desired set of derivative entities.
     *
     */
    template<int partial_jacobian_cols>
    class BuildJacobianFunctor {
    private:
        const Eigen::Matrix<double, DerivativeType::dimension, SystemType::constraint_dimension> &my_full_jacobian;
        Eigen::Matrix<double, DerivativeType::dimension, partial_jacobian_cols> &my_partial_jacobian;
        int my_index;

    public:
        BuildJacobianFunctor(const Eigen::Matrix<double, DerivativeType::dimension, SystemType::constraint_dimension> &full_jacobian,
                             Eigen::Matrix<double, DerivativeType::dimension, partial_jacobian_cols> &partial_jacobian) : my_partial_jacobian(partial_jacobian), my_full_jacobian(full_jacobian), my_index(0) {}

        // Only do something if the ConstraintType is derived from the ConstraintBase class.
        template <class ConstraintType, typename std::enable_if<std::is_base_of<ConstraintBase,ConstraintType>::value, int>::type = 0>
        void operator()(ConstraintType &t) {
            my_partial_jacobian.template block<DerivativeType::dimension, ConstraintType::getDimension()>(0, my_index) = my_full_jacobian.template block<DerivativeType::dimension, ConstraintType::getDimension()>(0, t.getIndex());

            my_index += t.getDimension();
        }

        // Don't do anything if the ConstraintType is not derived from ConstraintBase.
        template <class ConstraintType, typename std::enable_if<!std::is_base_of<ConstraintBase,ConstraintType>::value, int>::type = 0>
        void operator()(ConstraintType &t) {
        }

    };


private:
    const DerivativeType &my_derivative_entity;
    double my_location_relative_to_entity_base;

    State<SystemType::state_dimension> *my_state_ptr; // pointer to the state that this observer keeps track of

public:
    Observer(const DerivativeType &derivative_entity, double location_relative_to_entity_base = 0.0) : my_derivative_entity(derivative_entity), my_location_relative_to_entity_base(location_relative_to_entity_base), my_state_ptr(NULL) {}


    /*
    void setLocation(double location_relative_to_entity_base) {

        my_location_relative_to_entity_base = location_relative_to_entity_base;

    }*/

    double &location() { return my_location_relative_to_entity_base; };

    const DerivativeType &entity() const { return my_derivative_entity; };


    /**
     * Returns the arc-length location where the observed state comes from.
     *
     */
    double arcLength() const {
        return my_derivative_entity.getInterval()(0) + my_location_relative_to_entity_base;
    }




    /**
     * Attaches the observer to the given state reference.
     *
     */
    void attach(State<SystemType::state_dimension> &state) {
        my_state_ptr = &state;
    }


    /**
     * Uses the interpret() function associated with the underlying derivative entity that is being observed.
     *
     */
    auto state() -> decltype(my_derivative_entity.interpret(my_state_ptr->vector())) {
        return my_derivative_entity.interpret(my_state_ptr->vector());
    }


    /**
     * Returns the section of the state derivative that corresponds to the derivative entity.
     *
     */
    Eigen::Matrix<double, DerivativeType::dimension, SystemType::state_dimension> derivative() {
        return my_state_ptr->derivative().template block<DerivativeType::dimension, SystemType::state_dimension>(my_derivative_entity.getIndex(), 0);
    }


    /**
     * This computes the jacobian matrix.  The columns are associated with the derivative entities that are included in the template arguments of this function.
     *
     */
    template<class ...Ts>
    Eigen::Matrix<double, DerivativeType::dimension, AddupEntityDimensions<ConstraintBase, Ts...>::value> jacobian(const Constraint<SystemType::constraint_dimension, SystemType::state_dimension> &constraints, Ts&... args) {

        Eigen::Matrix<double, DerivativeType::dimension, AddupEntityDimensions<ConstraintBase, Ts...>::value> partial_jacobian;

        Eigen::Matrix<double, DerivativeType::dimension, SystemType::constraint_dimension> partial_derivative = my_state_ptr->derivative().template block<DerivativeType::dimension, SystemType::constraint_dimension>(my_derivative_entity.getIndex(), 0);



        Eigen::Matrix<double, DerivativeType::dimension, SystemType::constraint_dimension> full_jacobian = (constraints.derivative().transpose().householderQr().solve(partial_derivative.transpose())).transpose();



        BuildJacobianFunctor<AddupEntityDimensions<ConstraintBase, Ts...>::value>  build_partial_jacobian(full_jacobian, partial_jacobian);
        ApplyToListInOrder<BuildJacobianFunctor<AddupEntityDimensions<ConstraintBase, Ts...>::value>, Ts&...> apply(build_partial_jacobian, args...);

        return partial_jacobian;
    }

};




/**
 * This maintains a list of observers that are attached to a derivitive entity at a set of arclengths.
 *
 */
template<class SystemType, class DerivativeType>
class ObserverList  {
public:
    const DerivativeType &my_derivative_entity; // the derivative entity that this list observes
    std::vector< Observer<SystemType, DerivativeType> > observer_vector; // vector of obserbers


public:

    /**
     * Creates a list of observers that are uniformly spaced between the base_arclength and the tip_arclength of a derivative entity.
     * The argument 'count' is the number of observers to space in the interval.
     *
     */
    ObserverList(const DerivativeType &derivative_entity, int count) : my_derivative_entity(derivative_entity) {

        observer_vector.reserve(count);
        /*
        double step_length = (tip_arclength-base_arclength)/(count-1);
        for (int aa=0; aa<count; ++aa) {
            observer_vector.push_back(Observer<SystemType, DerivativeType>(derivative_entity, base_arclength + step_length*aa));
        }
         */
        for (int aa=0; aa<count; ++aa) {
            observer_vector.push_back(Observer<SystemType, DerivativeType>(derivative_entity));
        }
    }


    const DerivativeType &entity() const { return my_derivative_entity; };


    /**
     * Overloads the () operator to return the operator at the desired index.
     *
     */
    Observer<SystemType, DerivativeType> &operator()(int index) {
        return observer_vector[index];
    }

    /**
     * Returns the observer at the left boundary of the interval.
     *
     */
    Observer<SystemType, DerivativeType> &base() {
        return observer_vector[0];
    }


    /**
     * Returns the observer at the right boundary of the interval.
     *
     */
    Observer<SystemType, DerivativeType> &tip() {
        return *(observer_vector.end()-1);
    }


    /**
     * Returns the total number of observers.  Should be the same as 'count'.
     *
     */
    int size() {
        return observer_vector.size();
    }





};





#endif /* Observer_h */

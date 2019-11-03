//
//  Entities.h
//  SnareNeedleModel
//
//  Created by Art Mahoney on 8/2/16.
//  Copyright © 2016 Art Mahoney. All rights reserved.
//

#ifndef Entities_h
#define Entities_h





#include "ApplyToEntities.h"

class BunchBase {};
class NodeBase {};




/**
 * This is the base entity for all components of a system.  It includes the arc-length interval over which the entity should be integrated as well as the index into the state or constraint vector where the entity's state is stored.
 *
 */
class BaseEntity  {
public:
    template<class ...Ts2> friend class System;
    
    typedef Eigen::Vector2d Interval;
    typedef int Index;
    
public:
    int my_dimension; // state dimension
    
    Index my_index;
    
    Interval my_interval; // global
    Interval my_local_interval;
    


    
public:
    BaseEntity(int dimension = 0, const Interval &local_interval = Interval(0.0, 0.0)) : my_dimension(dimension), my_index(0), my_interval(local_interval), my_local_interval(local_interval) {};
    
    inline Index getIndex() const { return my_index; };
    inline Interval getInterval() const { return my_interval; };
    inline int getDimension() const { return my_dimension; };
    virtual bool isActive(double arclength) { return ((my_interval(0) <= arclength) && (arclength <= my_interval(1))); };
    
    inline void setIndex(int index) { my_index = index; };
    void setInterval(const Interval &interval) { my_interval = interval; };
    
    template <int n>
    Interval getLocalInterval(const Interval &parent_interval, const State<n> &state) const { return my_local_interval; };
    
    void setLocalInterval(const Interval &local_interval) { my_local_interval = local_interval; };

    
    
};







/**
 * This class is the base class for a node.  The node structure sets up how entities in the system relate to another.
 * Nodes tend to be states, while the connection between nodes are constraints.  Leafs are given to the node through
 * the template arguments.
 *
 */
template<class ...Ts>
class NodeEntity : public NodeBase  {
public:
    
    template<class FunctorClass, class ParentType, class ...Ts2>  friend struct Apply;
    
    typedef std::tuple<Ts&...> LeafTuple;
    
private:
    
    // These structures help to recursively add up the dimension of all the leaf classes of the given Entity type.
    template<class Entity, class... Ts2>
    struct AddupEntityDimensions : std::integral_constant< int, 0 > {};
    
    template<class Entity, class T2, class... Ts2>
    struct AddupEntityDimensions<Entity, T2, Ts2...> :
    std::integral_constant< int, T2::template getDimension<Entity>()*(std::is_base_of<Entity,T2>::value | std::is_base_of<NodeBase, T2>::value) + AddupEntityDimensions<Entity, Ts2...>::value > {};
    

    // These structures help to recursively add up the count of all leaf classes of the given Entity type.
    template<class Entity, class... Ts2>
    struct AddupEntityCount : std::integral_constant< int, 0 > {};
    
    template<class Entity, class T2, class... Ts2>
    struct AddupEntityCount<Entity, T2, Ts2...> :
    std::integral_constant< int, T2::template getTypeCount<Entity>() + AddupEntityCount<Entity, Ts2...>::value > {};

    
public:
    std::tuple<Ts&...> my_args; // stores the leafs
    
    
protected:
    // These functions recursively apply functors to the leafs.
    template <class FunctorClass, class ParentType, int... Is>
    void applyFunctor(FunctorClass &functor, ParentType parent, Helper::index<Is...>) {
        Apply<FunctorClass, ParentType, Ts&...> apply(functor, parent, std::get<Is>(my_args)...);
    }
    
    template <class FunctorClass, class ParentType>
    void applyFunctor(FunctorClass &functor, ParentType parent) {
        applyFunctor<FunctorClass, ParentType>(functor, parent, Helper::gen_seq<sizeof...(Ts)>{});
    }

    
public:
    
    
    NodeEntity(Ts&... args) : my_args(std::forward<Ts&>(args)...) {};
    //NodeEntity(Ts&... args) : my_args(args...) {};

    
    // These functions return the total dimension of the entities attached to this node.
    template<class T>
    static constexpr int getDimension() {
        return AddupEntityDimensions<T, Ts...>::value; // returns the total dimension of the entities of class type T attached to this node
    }
    static constexpr int getDimension() { return 0; }; // returns 0 by default when no entity type is given
    
    
    
    // This recursively returns the count of the given entity type that is in the tree below this node.
    template<class T>
    static constexpr int getTypeCount() {
        return AddupEntityCount<T, Ts...>::value;
    }
    
    // This function recursively applies a functor to the node's leafs.
    template <class FunctorClass>
    void applyFunctor(FunctorClass &functor) {
        applyFunctor<FunctorClass, BaseEntity >(functor, BaseEntity(), Helper::gen_seq<sizeof...(Ts)>{});
    }
};




template<class ...Ts>
class BunchEntity : public BunchBase, public NodeEntity<Ts...>  {
    
    template<class FunctorClass, class ParentType, class ...Ts2>  friend struct Apply;

    
private:
    
    // These functions recursively apply functors to the leafs.
    template <class FunctorClass, class ParentType, int... Is>
    void applyFunctor(FunctorClass &functor, ParentType parent, Helper::index<Is...>) {
        Apply<FunctorClass, ParentType, Ts&...> apply(functor, parent, std::get<Is>(this->my_args)...);
    }
    
    // These functions recursively apply functors to the leafs.
    template <class FunctorClass, int... Is>
    void applyToList(FunctorClass &functor, Helper::index<Is...>) {
        ApplyToList<FunctorClass, Ts&...> apply_to_list(functor, std::get<Is>(this->my_args)...);
    }
    
public:
    
    BunchEntity(Ts&... ts) : NodeEntity<Ts...>(ts...) {}
    
    template <class FunctorClass>
    void applyToList(FunctorClass &functor) {
        applyToList(functor, Helper::gen_seq<sizeof...(Ts)>{});
    }
    
    template <class FunctorClass, class ParentType>
    void applyFunctor(FunctorClass &functor, ParentType parent) {
        applyFunctor<FunctorClass, ParentType>(functor, parent, Helper::gen_seq<sizeof...(Ts)>{});
    }
    
    
    
};













class DerivativeBase : public BaseEntity {
public:
    DerivativeBase(int dimension, const Interval &interval) : BaseEntity(dimension, interval) {};
};



/**
 * This class is the base class for all entities that include a derivative of integration.  This encompasses all entities that can be
 * viewed as states.
 *
 */
template<class DerivativeType, class ...Ts>
class DerivativeEntity : public DerivativeBase, public NodeEntity<Ts...> {
private:
    

    
    
public:
    DerivativeEntity(const Interval &interval, Ts&... ts) : NodeEntity<Ts...>(ts...), DerivativeBase(DerivativeType::dimension, interval) {};
    

    // Returns the dimension of the children nodes, if the template type argument is not derived from DerivativeEntity.
    template<class T, typename std::enable_if<!std::is_base_of<T,DerivativeBase>::value,int>::type = 0 >
    static constexpr int getDimension() { return NodeEntity<Ts...>::template getDimension<T>(); };
    
    // Returns this rod's dimension plus the dimension of the children nodes if the template argument is derived from DerivativeEntity.
    template<class T, typename std::enable_if<std::is_base_of<T,DerivativeBase>::value,int>::type = 0 >
    static constexpr int getDimension() { return DerivativeType::dimension + NodeEntity<Ts...>::template getDimension<T>(); };
    
    // Returns this rod's dimension without any template arguments.
    static constexpr int getDimension() { return DerivativeType::dimension; }

    
    // Returns the dimension of the children nodes, if the template type argument is not derived from DerivativeEntity.
    template<class T, typename std::enable_if<!std::is_base_of<T,DerivativeType>::value,int>::type = 0 >
    static constexpr int getTypeCount() { return NodeEntity<Ts...>::template getTypeCount<T>(); };
    
    // Returns this rod's dimension plus the dimension of the children nodes if the template argument is derived from DerivativeEntity.
    template<class T, typename std::enable_if<std::is_base_of<T,DerivativeType>::value,int>::type = 0 >
    static constexpr int getTypeCount() { return 1 + NodeEntity<Ts...>::template getTypeCount<T>(); };


    
    
};



template<class ParameterType>
class ParameterEntity : public DerivativeEntity<ParameterEntity<ParameterType>> {
public:
    static constexpr int dimension = 1;
    
protected:
    /**
     * This implements the arc-length derivative for the integrator... it's always zero.
     *
     */
    class DerivativeFunctor {
    public:
        DerivativeFunctor()  {};
        
        template<class StateWindow>
        void operator()(const double &time, const StateWindow &state, StateWindow &derivative) const {
            return;
        }
    };
    
    
public:
    ParameterEntity() : DerivativeEntity<ParameterEntity<ParameterType>>(BaseEntity::Interval(0.0,0.0)) {};
    
    
    DerivativeFunctor derivative() {
        return DerivativeFunctor();
    }
    
    
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
     * Returns a reference to the arc-length where the RCM should be, stored in the full state vector.
     *
     */
    template<int n>
    double &interpret(Eigen::Matrix<double, n, 1> &state_vector) { return state_vector(this->getIndex()); }
    
    template<int n>
    double interpret(const Eigen::Matrix<double, n, 1> &state_vector) const { return state_vector(this->getIndex()); }
    
    template<class T>
    double &interpret(Eigen::Ref<T> state_vector) const {
        return state_vector(this->getIndex());
    }
};









/**
 * A constraint entity is very similar to a derivative entity except for the fact that they don't exist on an interval.  In code, however,
 * they are implemented to exist on a tiny interval of width 2*INTERVAL_WIDTH, centered at the desired location.  This is just for numerical
 * robustness so that the isActive function will work even if the input arc-length isn't exactly where this constraint exists.
 *
 * This class is the base entity that isn't specified as a template.  All constraints are ultimately derived from ConstraintBase regardless
 * of whether or not they are templated.
 *
 */
class ConstraintBase : public BaseEntity {
private:
    static constexpr double INTERVAL_WIDTH = 0.00000001;
    
public:
    ConstraintBase(int dimension, double location = 0.0) : BaseEntity(dimension, Interval(location,location)) {};
    
    bool isActive(double arclength) { return ((my_interval(0) <= arclength+INTERVAL_WIDTH) && (arclength-INTERVAL_WIDTH <= my_interval(1))); };
};



/**
 * Templated constraint entity that sets up the leafs of a constraint.  Most constraints will be derived from this class so that they
 * can fit within the tree structure of a snare system.
 *
 */
template<class ConstraintType, class ...Ts>
class ConstraintEntity : public ConstraintBase, public NodeEntity<Ts...> {
private:
    
    
public:
    /**
     * Simple constructor that initializes the constraint as a node and a constraint base.
     *
     */
    ConstraintEntity(double location, Ts&... ts) : NodeEntity<Ts...>(ts...), ConstraintBase(ConstraintType::dimension, location) {};
    
    
    // Returns the dimension of the children nodes, if the template type argument is not derived from DerivativeEntity.
    template<class T, typename std::enable_if<!std::is_base_of<T,ConstraintBase>::value,int>::type = 0 >
    static constexpr int getDimension() { return NodeEntity<Ts...>::template getDimension<T>(); };
    
    // Returns this rod's dimension plus the dimension of the children nodes if the template argument is derived from DerivativeEntity.
    template<class T, typename std::enable_if<std::is_base_of<T,ConstraintBase>::value,int>::type = 0 >
    static constexpr int getDimension() { return ConstraintType::dimension + NodeEntity<Ts...>::template getDimension<T>(); };
    
    // Returns this rod's dimension without any template arguments.
    static constexpr int getDimension() { return ConstraintType::dimension; }
    
    
    // Returns the dimension of the children nodes, if the template type argument is not derived from DerivativeEntity.
    template<class T, typename std::enable_if<!std::is_base_of<T,ConstraintType>::value,int>::type = 0 >
    static constexpr int getTypeCount() { return NodeEntity<Ts...>::template getTypeCount<T>(); };
    
    // Returns this rod's dimension plus the dimension of the children nodes if the template argument is derived from DerivativeEntity.
    template<class T, typename std::enable_if<std::is_base_of<T,ConstraintType>::value,int>::type = 0 >
    static constexpr int getTypeCount() { return 1 + NodeEntity<Ts...>::template getTypeCount<T>(); };
    
};


template<class ConstraintType, class ...Ts>
class BaseConstraintEntity : public ConstraintEntity<ConstraintType, Ts...> {
public:
    BaseConstraintEntity(Ts&... ts) : ConstraintEntity<ConstraintType, Ts...>(0.0, ts...) {}
    
    
    template <int n>
    BaseEntity::Interval getLocalInterval(const BaseEntity::Interval &parent_interval, const State<n> &state) const {
        return BaseEntity::Interval(parent_interval(0)-parent_interval(1), parent_interval(0)-parent_interval(1));
    }
    
};



template<class ConstraintType, class ...Ts>
class TipConstraintEntity : public ConstraintEntity<ConstraintType, Ts...> {
public:
    TipConstraintEntity(Ts&... ts) : ConstraintEntity<ConstraintType, Ts...>(0.0, ts...) {}
  
};








#endif /* Entities_h */

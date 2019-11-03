//
//  State.h
//  SnareNeedleModel
//
//  Created by Art Mahoney on 8/3/16.
//  Copyright © 2016 Art Mahoney. All rights reserved.
//

#ifndef State_h
#define State_h

#include "Eigen/Dense"

template<int dimension, int derivative_dimension>
class SubState;


template<int rows_t, int cols_t>
class VandyWindow {
private:
    static constexpr int my_rows = rows_t;
    static constexpr int my_cols = cols_t;

    int my_i;
    int my_j;

public:
    VandyWindow(int i, int j=0) : my_i(i), my_j(j) {};

    inline int i() const { return my_i; };
    inline int j() const { return my_j; };

    static constexpr int rows() { return my_rows; };
    static constexpr int cols() { return my_cols; };

};


class DynamicWindow {
private:
    int my_rows;
    int my_cols;
    int my_i;
    int my_j;

public:
    DynamicWindow(int rows, int cols, int i, int j) : my_rows(rows), my_cols(cols), my_i(i), my_j(j) {};

    int rows() const { return my_rows; };
    int cols() const { return my_cols; };
    int i() const { return my_i; };
    int j() const { return my_j; };
};




/**
 * This stores the state of a system.  It includes the state vector and the state derivative matrix.
 * All the data is stored in one matrix where the left column is the state vector and the remaining
 * right columns are the state derivative.
 *
 */
template<int my_state_dimension, int my_derivative_dimension = my_state_dimension>
class State {
public:
    static constexpr int state_dimension = my_state_dimension;
    static constexpr int derivative_dimension = my_derivative_dimension;

    typedef Eigen::Matrix<double, state_dimension, derivative_dimension+1> DataType;
    typedef Eigen::Matrix<double, state_dimension, 1> VectorType;
    typedef Eigen::Matrix<double, state_dimension, derivative_dimension> DerivativeType;


private:
    DataType my_data; // stores the state vector and derivative in one matrix

    Eigen::Ref<VectorType> vector_ref;
    Eigen::Ref<DerivativeType> derivative_ref;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Default class constructor, set's the state and its derivative to 0.
     *
     */
    State() : my_data(DataType::Zero()), vector_ref(my_data.col(0)), derivative_ref(my_data.template block<state_dimension, derivative_dimension>(0,1)) {};


    /**
     * Copy constructor.
     *
     */
    State(const State<state_dimension, derivative_dimension> &other) : vector_ref(my_data.col(0)), derivative_ref(my_data.template block<state_dimension, derivative_dimension>(0,1)) {
        my_data.noalias() = other.my_data;
    }


    /**
     * Assignment operator for states.
     *
     */
    const State &operator=(const State<state_dimension, derivative_dimension> &other) {
        my_data.noalias() = other.my_data;
        return *this;
    }


    /**
     * Overloads the assignment operator for SubStates.  It works by copying the SubState into the its position inside this class.  Mostly used by the Integrator class.
     * (See the SubState class definitiion below.)
     *
     */
    template <int substate_dimension>
    const State &operator=(const SubState<substate_dimension, derivative_dimension> &substate) {
        my_data.template block<substate_dimension, derivative_dimension+1>(substate.index(), 0) = substate.data();
        return *this;
    }


    /**
     * Overloads the += operator, mostly used by the Integrator class.
     *
     */
    const State &operator+=(const State<state_dimension, derivative_dimension> &other) {
        my_data.noalias() += other.data();
        return *this;
    }


    /**
     * These functions return references to the full data, the state vector, or the state derivative.
     *
     */
    inline DataType &data() { return my_data; };
    inline const DataType &data() const { return my_data; };
    inline Eigen::Ref<VectorType> vector() { return vector_ref; };
    inline const Eigen::Ref<VectorType> vector() const { return vector_ref; };
    inline Eigen::Ref<DerivativeType> derivative() { return derivative_ref; };
    inline const Eigen::Ref<DerivativeType> derivative() const { return derivative_ref; };

};



/**
 * This template implements the pre-multiply operator for any class T.  It's mostly used in the Integrator class to pre-multiply Eigen vectors by scalars.
 *
 */
template<int state_dimension, int derivative_dimension, class T>
State<state_dimension, derivative_dimension> operator*(const T &t, const State<state_dimension, derivative_dimension> &other) {
    State<state_dimension, derivative_dimension> tmp;
    tmp.data().noalias() = t*other.data();
    return tmp;
}







template<class State, class VandyWindow>
class ConstStateWindow {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Block<const typename State::DataType, VandyWindow::rows(), State::derivative_dimension+1> DataType;
    typedef Eigen::Block<const typename State::DataType, VandyWindow::rows(), 1> VectorType;
    typedef Eigen::Block<const typename State::DataType, VandyWindow::rows(), State::derivative_dimension> DerivativeType;
    typedef Eigen::Matrix<double, VandyWindow::rows(), State::derivative_dimension+1> MatrixType;


private:
    DataType my_data;

    VectorType my_vector;
    DerivativeType my_derivative;

public:
    ConstStateWindow(const State &state, const VandyWindow &window) : my_data(state.data().template block<VandyWindow::rows(), State::derivative_dimension+1>(window.i(), 0)), my_vector(state.data().template block<VandyWindow::rows(), 1>(window.i(), 0)), my_derivative(state.data().template block<VandyWindow::rows(), State::derivative_dimension>(window.i(), 1)) {
    }

    VectorType vector() const { return my_vector; };
    DerivativeType derivative() const { return my_derivative; };
    DataType data() const { return my_data; };
};




template<class State, class VandyWindow>
class StateWindow {
public:
    typedef Eigen::Block<typename State::DataType, VandyWindow::rows(), State::derivative_dimension+1> DataType;
    typedef Eigen::Block<typename State::DataType, VandyWindow::rows(), 1> VectorType;
    typedef Eigen::Block<typename State::DataType, VandyWindow::rows(), State::derivative_dimension> DerivativeType;
    typedef Eigen::Matrix<double, VandyWindow::rows(), State::derivative_dimension+1> MatrixType;

private:
    DataType my_data;

    VectorType my_vector;
    DerivativeType my_derivative;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StateWindow(State &state, const VandyWindow &window) : my_data(state.data().template block<VandyWindow::rows(), State::derivative_dimension+1>(window.i(), 0)), my_vector(state.data().template block<VandyWindow::rows(), 1>(window.i(), 0)), my_derivative(state.data().template block<VandyWindow::rows(), State::derivative_dimension>(window.i(), 1)) {

    }

    inline VectorType vector() { return my_vector; };
    inline const VectorType vector() const { return my_vector; };
    inline DerivativeType derivative() { return my_derivative; };
    inline const DerivativeType derivative() const { return my_derivative; };
    inline DataType data() { return my_data; };
    inline const DataType data() const { return my_data; };

    const StateWindow &operator=(const StateWindow<State, VandyWindow> &other_state_window) {
        my_data.noalias() = other_state_window.data();
        return *this;
    }

    const StateWindow &operator=(const ConstStateWindow<State, VandyWindow> &other_state_window) {
        my_data.noalias() = other_state_window.data();
        return *this;
    }

    const StateWindow &operator+=(const typename StateWindow<State, VandyWindow>::MatrixType &matrix) {
        my_data.noalias() += matrix;
        return *this;
    }

};

template<class State, class VandyWindow, class T>
typename StateWindow<State, VandyWindow>::DataType operator*(const T &val, const StateWindow<State, VandyWindow> &state) {
    typename StateWindow<State, VandyWindow>::DataType tmp;

    tmp.noalias() = val * state.data();

    return tmp;
}





template<class State, class VandyWindow>
class ConstDynamicStateWindow {
public:
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DataType;
    typedef Eigen::Block<const typename State::DataType> VectorType;
    typedef Eigen::Block<const typename State::DataType> DerivativeType;



private:

    VectorType my_vector;
    DerivativeType my_derivative;
    const VandyWindow &my_window;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConstDynamicStateWindow(const State &state, const VandyWindow &window) : my_vector(state.data().block(window.i(), 0, window.rows(), 1)),
    my_derivative(state.data().block(window.i(), 1+window.j(), window.rows(), window.cols())),
    my_window(window) {
    }

    inline const VectorType vector() const { return my_vector; };
    inline const DerivativeType derivative() const { return my_derivative; };


};



template<class State, class VandyWindow>
class DynamicStateWindow {
public:
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DataType;
    typedef Eigen::Block<typename State::DataType> VectorType;
    typedef Eigen::Block<typename State::DataType> DerivativeType;

private:
    const VandyWindow &my_window;
    State &my_state;

    int my_i;
    int my_j;
    int my_rows;
    int my_cols;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    DynamicStateWindow(State &state, const VandyWindow &window) : my_state(state), my_window(window) {}


    inline VectorType vector() { return my_state.data().block(my_window.i(), 0, my_window.rows(), 1); };
    inline const VectorType vector() const { return my_state.data().block(my_window.i(), 0, my_window.rows(), 1); };
    inline DerivativeType derivative() { return my_state.data().block(my_window.i(), 1+my_window.j(), my_window.rows(), my_window.cols()); };
    inline const DerivativeType derivative() const { return my_state.data().block(my_window.i(), 1+my_window.j(), my_window.rows(), my_window.cols()); };
    inline int cols() const { return my_window.cols(); };

    const DynamicStateWindow<State, VandyWindow> &operator+=(const DataType &matrix) {
        this->vector().noalias() += matrix.template leftCols<1>();
        this->derivative().noalias() += matrix.rightCols(my_window.cols());
        return *this;
    }

    const DynamicStateWindow<State, VandyWindow> &operator=(const DynamicStateWindow<State, VandyWindow> &other) {
        this->vector().noalias() = other.vector();
        this->derivative().noalias() = other.derivative();
        return *this;
    }

    const DynamicStateWindow<State, VandyWindow> &operator=(const ConstDynamicStateWindow<State, VandyWindow> &other) {
        this->vector().noalias() = other.vector();
        this->derivative().noalias() = other.derivative();
        return *this;
    }
};



template<class State, class VandyWindow, class T>
typename DynamicStateWindow<State, VandyWindow>::DataType operator*(const T &val, const DynamicStateWindow<State, VandyWindow> &state_window) {

    typename DynamicStateWindow<State, VandyWindow>::DataType tmp(state_window.derivative().rows(), state_window.derivative().cols()+1 /* plus one for vector+derivative matrix storage */);

    tmp.template leftCols<1>() = state_window.vector();
    tmp.rightCols(state_window.cols()) = state_window.derivative();

    tmp *= val;

    return tmp;
}




/**
 * This class stores the vector and derivative of a system's constraints.
 *
 */
template<int constraint_dimension, int state_dimension>
class Constraint {
public:
    typedef Eigen::Matrix<double, constraint_dimension, 1> VectorType;
    typedef Eigen::Matrix<double, constraint_dimension, state_dimension> DerivativeType;
    typedef Eigen::Matrix<double, constraint_dimension, state_dimension+1> DataType;

private:
    DataType my_data;

    Eigen::Ref<VectorType> vector_ref;
    Eigen::Ref<DerivativeType> derivative_ref;



public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Constraint() : my_data(DataType::Zero()), vector_ref(my_data.col(0)), derivative_ref(my_data.template block<constraint_dimension, state_dimension>(0,1)) {};

    inline DataType &data() { return my_data; };
    inline const DataType &data() const { return my_data; };
    inline Eigen::Ref<VectorType> vector() { return vector_ref; };
    inline const Eigen::Ref<VectorType> vector() const { return vector_ref; };
    inline Eigen::Ref<DerivativeType> derivative() { return derivative_ref; };
    inline const Eigen::Ref<DerivativeType> derivative() const { return derivative_ref; };
};




#endif /* State_h */

//
//  ParallelCosseratRod.hpp
//  SnareNeedleModel
//
//  Created by Art Mahoney on 7/7/16.
//  Copyright © 2016 Art Mahoney. All rights reserved.
//

#ifndef ParallelCosseratRod_hpp
#define ParallelCosseratRod_hpp

#include <iostream>
#include <vector>
#include <utility>
#include <algorithm>
#include <limits>
#include "Eigen/Dense"
#include "Entities.h"
#include "State.h"
#include "Integrator.h"
#include "RK8Integrator.h"
#include "Tools.h"
#include "DerivativeEntities.h"
#include "SystemSolver.h"
#include "Observer.h"



template<class ...Ts>
class System : public NodeEntity<Ts...> {
public:
    static constexpr int constraint_dimension = NodeEntity<Ts...>::template getDimension<ConstraintBase>(); // dimension of the constraints
    static constexpr int state_dimension = NodeEntity<Ts...>::template getDimension<DerivativeBase>(); // dimension of the states (should be equal to the constraints)

    static constexpr int interval_integration_steps = 3; // this is the number of integration steps that the integrator will use between stopping points.

    // Note that interval_integration_steps is the number of steps between stopping points.  If you attach observers (see Observer.h) then the number of stopping points will
    // increase and the space between stopping points will decrease.  This has the result of increasing the accuracy and increasing the computation time.


    typedef Eigen::Matrix<double, state_dimension, 1> StateVector;
    typedef Eigen::Matrix<double, constraint_dimension, 1> ConstraintVector;


private:

    RK8Integrator<double, State<state_dimension> > rk8_integrator;


    /**
     * This class sets up the arc-length intervals where entities occur on the arc-length number line.  This is used to
     * determine where the integrator needs to pause to apply discontinuity transitions and constraints.  The interval
     * is also used to decide where a derivative entity is active and needs to be integrated.
     *
     */
    class SetArcLengthFunctor {
    private:
        State<state_dimension> my_state;
    public:
        SetArcLengthFunctor(const State<state_dimension> &state) : my_state(state) {};

        // If the class is a DerivativeBase
        template<class T, class ParentType, typename std::enable_if<std::is_base_of<DerivativeBase,T>::value, int>::type = 0>
        void operator()(T &t, const ParentType &parent) {
            t.setInterval(t.getLocalInterval(parent.getInterval(), my_state) + parent.getInterval());
            //std::cout << "d (" << t.getInterval().transpose() << ") [" << t.getIndex() << "]" << std::endl;
            //std::cout << "   (" << t.getLocalInterval(parent.getInterval(), my_state).transpose() << ") " << "(" << parent.getInterval().transpose() << ")" <<std::endl;


        }

        // If the class is a ConstraintBase
        template<class T, class ParentType, typename std::enable_if<std::is_base_of<ConstraintBase,T>::value, int>::type = 0>
        void operator()(T &t, const ParentType &parent) {
            BaseEntity::Interval parent_interval = parent.getInterval();
            t.setInterval(t.getLocalInterval(parent_interval , my_state) + BaseEntity::Interval(parent_interval(1), parent_interval(1)));
            //std::cout << "c (" << t.getInterval().transpose() << ") [" << t.getIndex() << "]" << std::endl;
        }

        // If the class isn't a DerivativeBase or a ConstraintBase.
        template<class T, class ParentType, typename std::enable_if<!std::is_base_of<ConstraintBase,T>::value && !std::is_base_of<DerivativeBase,T>::value, int>::type = 0>
        void operator()(T &t, const ParentType &parent) {}


    };


    /**
     * This stores where all the entities occur on the arc-length number line.  Arc-length locations of critical things are
     * stored in the given vector.  This also makes sure that there aren't duplicate arc-lengths stored in the vector and that
     * the vector is in ascending order.
     *
     */
    class BuildArcLengthVectorFunctor {
    private:
        std::vector<double> &my_arc_length_locations; // stores all the start and end arc-length interval locations

    public:
        BuildArcLengthVectorFunctor(std::vector<double> &arc_length_locations) : my_arc_length_locations(arc_length_locations) {};

        template<class ParentType>
        void operator()(BaseEntity &t, const ParentType &parent) {
            BaseEntity::Interval entity_interval = t.getInterval();

            // add the start and end intervals
            my_arc_length_locations.push_back(entity_interval(0));
            my_arc_length_locations.push_back(entity_interval(1));

            // sort the vector with the new arc lengths added
            std::sort(my_arc_length_locations.begin(), my_arc_length_locations.end());

            // remove duplicate arc lengths
            std::vector<double>::iterator it;
            it = std::unique(my_arc_length_locations.begin(), my_arc_length_locations.end(), [](double l, double r) { return (fabs(l-r) <= 0.0000001); } );
            my_arc_length_locations.erase(it, my_arc_length_locations.end());
        }
    };


    /**
     * This class is used to set the index locations of all the entities in the constraint vector and the derivative vector.
     *
     */
    class BuildIndexFunctor {
    private:
        int current_constraint_index;
        int current_derivative_index;

    public:
        BuildIndexFunctor() : current_constraint_index(0), current_derivative_index(0) {};

        // Handles derivative entities.
        template<class ParentType>
        void operator()(DerivativeBase &t, const ParentType &parent) {
            t.setIndex(current_derivative_index);
            current_derivative_index += t.getDimension();
        }

        // Handles constraint entities.
        template<class ParentType>
        void operator()(ConstraintBase &t, const ParentType &parent) {
            t.setIndex(current_constraint_index);
            current_constraint_index += t.getDimension();
        }
    };


    /**
     * This class is used to apply constraints and transitions across arc-length discontinuities.
     *
     */
    class ApplyConstraintFunctor {
    private:
        double my_arclength;
        Constraint<constraint_dimension, state_dimension> &my_constraint;
        State<state_dimension> &my_state;

    public:
        ApplyConstraintFunctor(double arclength,
                               Constraint<constraint_dimension, state_dimension> &constraint,
                               State<state_dimension> &state) : my_arclength(arclength), my_constraint(constraint), my_state(state) {};


        // Apply the constraint and transition.
        template<class T, class ParentType, typename std::enable_if<std::is_base_of<ConstraintBase,T>::value, int>::type = 0>
        void operator()(T &t, const ParentType &parent) {

            if (t.isActive(my_arclength)) { // check to see if the constraint is active and should be applied to the state vector
                t.apply(my_constraint, my_state, parent);
            }

        }


        // Basically do nothing if the input isn't a constraint entity, just pass the index down if the node type isn't a constraint.
        template<class T, class ParentType, typename std::enable_if<!std::is_base_of<ConstraintBase,T>::value, int>::type = 0>
        void operator()(T &t, const ParentType &parent) {}
    };








    /**
     * This functor is used to set the identity element of the initial partial derivative matrix.
     *
     */
    class DerivativeIdentityFunctor {
    private:
        Eigen::Matrix<double, state_dimension, 1> my_current_state;
        Eigen::Ref<Eigen::Matrix<double, state_dimension, state_dimension>> my_derivative_identity;

    public:
        DerivativeIdentityFunctor(const Eigen::Matrix<double, state_dimension, 1> &current_state, Eigen::Ref<Eigen::Matrix<double, state_dimension, state_dimension>> derivative_identity) : my_current_state(current_state), my_derivative_identity(derivative_identity) {}

        // Only set the derivative identity element for derivative entities.
        template<class T, class ParentType, typename std::enable_if<std::is_base_of<DerivativeBase,T>::value, int>::type = 0>
        void operator()(T &t, const ParentType &parent) {

           // std::cout << T::getDimension() << " " << t.getIndex() << std::endl;

            my_derivative_identity.template block<T::getDimension(),T::getDimension()>(t.getIndex(), t.getIndex()) = t.derivativeIdentity(my_current_state.template segment<T::getDimension()>(t.getIndex()));
        }

        // Do nothing if the input isn't a derivative entity.
        template<class T, class ParentType, typename std::enable_if<!std::is_base_of<DerivativeBase,T>::value, int>::type = 0>
        void operator()(T &t, ParentType &parent) {}
    };


    /**
     * This class functor is used to normalize a state vector so that the quaternion elements are unit length.
     *
     */
    /*
    class NormalizationFunctor {
    private:
        Eigen::Matrix<double, state_dimension, 1> &my_current_state;

    public:
        NormalizationFunctor(Eigen::Matrix<double, state_dimension, 1> &current_state) : my_current_state(current_state) {}

        // Integrate only the derivative entities.
        template<class T, class ParentType, typename std::enable_if<std::is_base_of<DerivativeBase,T>::value, int>::type = 0>
        void operator()(T &t, const ParentType &parent) {
            my_current_state.template segment<T::getDimension()>(t.getIndex()(0)) = t.normalize(my_current_state.template segment<T::getDimension()>(t.getIndex()(0)));
        }

        // Do nothing if the input isn't a derivative entity.
        template<class T, class ParentType, typename std::enable_if<!std::is_base_of<DerivativeBase,T>::value, int>::type = 0>
        void operator()(T &t, ParentType &parent) {}
    };
    */



    /**
     * This class is used to integrate derivative entities.
     *
     */
    template <class Integrator>
    class IntegrateStateFunctor {
    private:
        std::vector<double>::iterator my_arclength_iterator_start;
        std::vector<double>::iterator my_arclength_iterator_final;

        typename std::vector<State<state_dimension>>::iterator my_state_iterator;


        Integrator &my_integrator;

    public:
        IntegrateStateFunctor(Integrator &integrator,
                              std::vector<double>::iterator arclength_iterator_start,
                              std::vector<double>::iterator arclength_iterator_final,
                              typename std::vector<State<state_dimension>>::iterator state_iterator) : my_arclength_iterator_start(arclength_iterator_start), my_arclength_iterator_final(arclength_iterator_final), my_state_iterator(state_iterator), my_integrator(integrator) {}

        // Integrate only the derivative entities.
        template<class T, class ParentType, typename std::enable_if<std::is_base_of<DerivativeBase,T>::value, int>::type = 0>
        void operator()(T &t, const ParentType &parent) {


            double start_arclength = *my_arclength_iterator_start;
            double final_arclength = *my_arclength_iterator_final;


            VandyWindow<T::getDimension(), state_dimension> window(t.getIndex());
            //DynamicWindow window(T::getDimension(), state_dimension-t.getIndex(), t.getIndex(), t.getIndex());

            //std::cout << window.i() << " " << window.j() << " " << window.rows() << " " << window.cols() << std::endl;

          //  std::cout << "[" << start_arclength << ", " << final_arclength << "] - " << t.getIndex() << std::endl;


            // Perform integration if the entity is active.
            if (t.isActive( (final_arclength+start_arclength)/2.0) ) { // integrate the substate to the next location




               my_integrator.integrate(t.derivative(), my_arclength_iterator_start, my_arclength_iterator_final, my_state_iterator, window);


            }
            else { // copy the start sub state to the next location
                long int distance = std::distance(my_arclength_iterator_start, my_arclength_iterator_final);

                StateWindow<State<state_dimension>, VandyWindow<T::getDimension(), state_dimension>> initial_state_window(*my_state_iterator, window);

                //DynamicStateWindow<State<state_dimension>, DynamicWindow> initial_state_window(*my_state_iterator, window);

                //DynamicStateWindow<State<state_dimension>, VandyWindow<T::getDimension(), state_dimension> > initial_state_window(*my_state_iterator, window);


                for (typename std::vector<State<state_dimension>>::iterator it = my_state_iterator; it <= my_state_iterator+distance; ++it) {

		  StateWindow<State<state_dimension>, VandyWindow<T::getDimension(), state_dimension> >(*it, window) = initial_state_window;

                    //DynamicStateWindow<State<state_dimension>, DynamicWindow>(*it, window) = initial_state_window;

                    //DynamicStateWindow<State<state_dimension>, VandyWindow<T::getDimension(), state_dimension> >(*it, window) = initial_state_window;



                    //tmp = *my_state_iterator; // this intermediate copy step isn't efficient but it's needed to ensure that the sub state is copied properly, hopefully fix this in the future.
                    //*it = tmp;
                }


            }


        }

        // Do nothing if the input isn't a derivative entity.
        template<class T, class ParentType, typename std::enable_if<!std::is_base_of<DerivativeBase,T>::value, int>::type = 0>
        void operator()(T &t, ParentType &parent) {}

    };


    /**
     * This class adds the observer arc-lengths locations to a vector of arc lengths where the integrator has to stop.
     *
     */
    class AddObserverLocationsFunctor {
    private:
        std::vector<double> &my_arc_length_locations;

    public:
        AddObserverLocationsFunctor(std::vector<double> &arc_length_locations) : my_arc_length_locations(arc_length_locations) {};

        /**
         * Add a list of observers.
         *
         */
        template<class SystemType, class DerivativeType>
        void operator()(ObserverList<SystemType, DerivativeType> &t) {

            BaseEntity::Interval interval = t.entity().getInterval();

            double step_length = (interval(1)-interval(0))/(t.size()-1);

            for (int aa=0; aa<t.size(); ++aa) {
                t(aa).location() = step_length*aa;
                my_arc_length_locations.push_back(t(aa).arcLength());
            }

            // sort the vector with the new arc lengths added
            std::sort(my_arc_length_locations.begin(), my_arc_length_locations.end());

            // remove duplicate arc lengths
            std::vector<double>::iterator it;
            it = std::unique(my_arc_length_locations.begin(), my_arc_length_locations.end(), [](double l, double r) { return (fabs(l-r) <= 0.0000001); } );
            my_arc_length_locations.erase(it, my_arc_length_locations.end());
        }

        /**
         * Add a single observer.
         *
         */
        template<class SystemType, class DerivativeType>
        void operator()(Observer<SystemType, DerivativeType> &t) {
            my_arc_length_locations.push_back(t.arcLength());

            // sort the vector with the new arc lengths added
            std::sort(my_arc_length_locations.begin(), my_arc_length_locations.end());

            // remove duplicate arc lengths
            std::vector<double>::iterator it;
            it = std::unique(my_arc_length_locations.begin(), my_arc_length_locations.end(), [](double l, double r) { return (fabs(l-r) <= 0.0000001); } );
            my_arc_length_locations.erase(it, my_arc_length_locations.end());
        }

    };


    /**
     * This class attaches observers to states stored at the desired arc-lengths.  This is a brute force method and probably could be sped up by pre-sorting.
     *
     */
    class AttachObserversFunctor {
    private:
        std::vector<double> &my_arc_length_array;
        std::vector<State<state_dimension>> &my_state_array;

    public:
        AttachObserversFunctor(std::vector<double> &arc_length_array, std::vector<State<state_dimension>> &state_array) : my_arc_length_array(arc_length_array), my_state_array(state_array) {};

        /**
         * Attach a list of observers.
         *
         */
        template<class SystemType, class DerivativeType>
        void operator()(ObserverList<SystemType, DerivativeType> &t) {
            for (int aa=0; aa<t.size(); ++aa) {
     		t(aa).attach(my_state_array[my_arc_length_array.size()-1]);
                for (unsigned bb=0; bb<my_arc_length_array.size(); ++bb) {
                    if ( fabs(t(aa).arcLength() - my_arc_length_array[bb]) <= 0.0000001) {
                        t(aa).attach(my_state_array[bb]);
                        break;
                    }
                }
            }
        }

        /**
         * Attach a single observer.
         *
         */
        template<class SystemType, class DerivativeType>
        void operator()(Observer<SystemType, DerivativeType> &t) {
            t.attach(my_state_array[my_arc_length_array.size()-1]);
            for (unsigned bb=0; bb<my_arc_length_array.size(); ++bb) {
                if ( fabs(t.arcLength() - my_arc_length_array[bb]) <= 0.0000001) {
                    t.attach(my_state_array[bb]);
                    break;
                }
            }
        }
    };


    /**
     * Gathers the indices into the state vector where the moment and forces are stored.
     *
     */
    class GatherMomentForceIndices {
    private:
        std::vector<int> &my_moment_force_indices;

    public:
        GatherMomentForceIndices(std::vector<int> &moment_force_indices) : my_moment_force_indices(moment_force_indices) {
            my_moment_force_indices.reserve(NodeEntity<Ts...>::template getTypeCount<NodeBase>());
        }


        // Record the the starting index in the state vector of the moment and force states if the entity is a RodBase.
        template<class T, class ParentType, typename std::enable_if<std::is_base_of<RodBase,T>::value, int>::type = 0>
        void operator()(T &t, const ParentType &parent) {
            my_moment_force_indices.push_back(t.getIndex()+7);
        }

        template<class T, class ParentType, typename std::enable_if<!std::is_base_of<RodBase,T>::value, int>::type = 0>
        void operator()(T &t, const ParentType &parent) {}
    };




public:


    /**
     * This stores the solution of a solved system.  It stores all the states and all the constraints, plus derivatives.
     *
     */
    class Solution {
    public: // public members
        std::vector<double> important_arc_lengths; // arc-lengths where the integrator has to stop
        std::vector<double> arc_length_array; // arc-lengths where there is an integrated state stored
        std::vector<State<state_dimension>> state_array; // array of integrated states

        Constraint<constraint_dimension, state_dimension> constraints; // stores the constraints after integration, has the vector and its state derivative

    public: // public functions
        Solution() : important_arc_lengths(), arc_length_array(), state_array(), constraints() {};

        double error() {
            return this->constraints.vector().norm();
        }

        State<state_dimension> end() { return *(state_array.end()-1); }; // returns the state at the end arc-length
        State<state_dimension> base() { return state_array[0]; }; // returns the state at the base arc-length
    };


public:

    System(Ts&... ts) :  NodeEntity<Ts...>(ts...) {
        // Build up the indices where the constraint and derivative entities fall in the state and constraint vectors.
        BuildIndexFunctor index_initializer;
        this->template applyFunctor<BuildIndexFunctor>(index_initializer); // initialize the index into the state and constraint vectors for each member
    };


    /**
     * Initializes the internal states of the system in preparation for integration.
     *
     */
    template<class ...Ts2>
    void initialize(Solution &solution, const Eigen::Matrix<double, state_dimension, 1> &initial_state_vector, Ts2&... observers) {

        solution.important_arc_lengths.clear();
        solution.state_array.clear();
        solution.arc_length_array.clear();

        State<state_dimension> initial_integrator_state;

        // Build the initial state out of the provided initial state vector.
        initial_integrator_state.vector() = initial_state_vector;
        initial_integrator_state.derivative() = Eigen::Matrix<double, state_dimension, state_dimension>::Identity();
        DerivativeIdentityFunctor identity_matrix_builder(initial_integrator_state.vector(), initial_integrator_state.derivative());
        this->template applyFunctor<DerivativeIdentityFunctor>(identity_matrix_builder);


        /*
        NormalizationFunctor normalizer(initial_integrator_state.vector());
        this->template applyFunctor<NormalizationFunctor>(normalizer);
         */

        // Figures out where the arc-length locations of all the elements are.
        SetArcLengthFunctor arc_length_initializer(initial_integrator_state);
        this->template applyFunctor<SetArcLengthFunctor>(arc_length_initializer); // inialize all the arc-length intervals of the system members


        // Arranges all the arc-lengths where the integrator must stop into a vector.
        BuildArcLengthVectorFunctor arclength_vector_initializer(solution.important_arc_lengths);
        this->template applyFunctor<BuildArcLengthVectorFunctor>(arclength_vector_initializer);


        // Generate the arc lengths where integration steps occur.
        for (unsigned aa=0; aa<solution.important_arc_lengths.size()-1; ++aa) {
            std::vector<double> tmp;
            Tools::linspace(solution.important_arc_lengths[aa], solution.important_arc_lengths[aa+1], interval_integration_steps, tmp);
            solution.arc_length_array.insert(solution.arc_length_array.end(), tmp.begin(), tmp.end()-1);
        }
        solution.arc_length_array.push_back(*(solution.important_arc_lengths.end()-1));


        // Add the observer locations to the arc length array where the integrator has to stop.
        AddObserverLocationsFunctor  add_observer_locations(solution.arc_length_array);
        ApplyToList<AddObserverLocationsFunctor, Ts2&...> apply(add_observer_locations, observers...);

        // Allocate space for the state array, and set the base array to the initial state.
        solution.state_array.resize(solution.arc_length_array.size());
        solution.state_array[0] = initial_integrator_state; // set the initial integrator state

        // Attach the observers to the appropriate states in the state array.
        AttachObserversFunctor attach_observers(solution.arc_length_array, solution.state_array);
        ApplyToList<AttachObserversFunctor, Ts2&...> apply2(attach_observers, observers...);


    }


    /**
     * Integrates the system.  The constraint data structure stores the vector value of the constraint
     * along with the constraint derivative.  This is used to solve the BVP problem.  NOTE that the initialize()
     * function must be called before this is executed!
     *
     */
    void integrate(Solution &solution) {

        std::vector<double>::iterator arc_length_iterator = solution.arc_length_array.begin();
        typename std::vector<State<state_dimension>>::iterator state_iterator = solution.state_array.begin();


        for (unsigned aa=0; aa<solution.important_arc_lengths.size()-1; ++aa) {

            // Searches for the interator location of the next important arc length where the integration should be stopped.  TODO: do a binary search here.
            std::vector<double>::iterator find_result;
            for (find_result = arc_length_iterator; find_result < solution.arc_length_array.end(); find_result++) {
                if (fabs(*find_result-solution.important_arc_lengths[aa+1]) <= 0.0000001)
                    break;
            }
	    if (find_result == solution.arc_length_array.end()) {////////////////
	      --find_result;
	    }

            long int distance = std::distance(arc_length_iterator, find_result);

            // First apply the constraints.
            ApplyConstraintFunctor apply_constraints(*arc_length_iterator, solution.constraints,  *state_iterator );
            this->template applyFunctor<ApplyConstraintFunctor>(apply_constraints); // apply constraints to proximal end of interval before integrating

            // Integrate the states forward to the next stopping point.
            IntegrateStateFunctor< RK8Integrator<double, State<state_dimension> > > state_integrator(rk8_integrator, arc_length_iterator, find_result, state_iterator);
            this->template applyFunctor<IntegrateStateFunctor < RK8Integrator<double, State<state_dimension> > > >(state_integrator);

            arc_length_iterator += distance;
            state_iterator += distance;

        }

        // Apply the constraints at the end.
        ApplyConstraintFunctor apply_constraints(*arc_length_iterator, solution.constraints,  *state_iterator );
        this->template applyFunctor<ApplyConstraintFunctor>(apply_constraints); // apply constraints to the end of the interval
    }




    /**
     * Test for stability.  The test works by looking to see if the derivative that maps small changes in the forces and moments at all arc-lengths
     * to small changes in the constraint is full rank.  If there is an arc-length where the derivative drops rank, then that means there is probably
     * an instability.  This is just a heuristic at this point... more work needs to be done to formalize and prove this process.
     *
     */
    double stability(Solution &solution) {

        Eigen::Matrix<double, constraint_dimension, 6*NodeEntity<Ts...>::template getTypeCount<RodBase>()> lagrange_derivative;
        lagrange_derivative.setConstant(0.0);

        Eigen::Matrix<double, constraint_dimension, state_dimension> tmp;

        // Get the indices that correspond to the moment and force in the state vector for each rod.
        std::vector<int> indices;
        GatherMomentForceIndices moment_force_indices(indices);
        this->template applyFunctor<GatherMomentForceIndices>(moment_force_indices);


        std::cout << "indices:" << std::endl;
        for (unsigned bb=0; bb<indices.size(); ++bb)
            std::cout << indices[bb] << std::endl;
        std::cout << std::endl;
        std::cout << NodeEntity<Ts...>::template getTypeCount<RodBase>() << std::endl;



        double lagrange_derivative_determinant = 0.0;
        double stability_measure = std::numeric_limits<double>::max();

        // Loop over all arc-lengths where a state has been integrated.
        for (unsigned aa=0; aa<solution.state_array.size(); ++aa) {

            tmp = solution.state_array[aa].derivative().transpose().householderQr().solve(solution.constraints.derivative().transpose()).transpose(); // solve for the derivative that relates small changes in the state at the current arc-length to small changes in the constraint


            //std::cout << solution.state_array[aa].derivative() << std::endl;



            // Pic off the columns of the derivative that are associated with force and moment
            for (int col_i=0; col_i<indices.size(); ++col_i) {
                lagrange_derivative.template block<constraint_dimension,6>(0, col_i*6).noalias() = tmp.template block<constraint_dimension,6>(0, indices[col_i]);
            }





            lagrange_derivative_determinant = (lagrange_derivative.transpose()*lagrange_derivative).determinant();

            std::cout << lagrange_derivative_determinant << std::endl;

            if (stability_measure > lagrange_derivative_determinant)
                stability_measure = lagrange_derivative_determinant;

        }

        return stability_measure;
    }


    /**
     * This solves the system by finding the initial state vector that satisfies all the constraints.
     *
     */
    int solve(const Eigen::Matrix<double, state_dimension, 1> &initial_state_vector, Eigen::Matrix<double, state_dimension, 1> &solution_state_vector) {

        solution_state_vector = initial_state_vector;

        SystemSolver<System<Ts...>> solver(*this);
        return solver.solve(solution_state_vector);

    }


};


#endif /* ParallelCosseratRod_hpp */

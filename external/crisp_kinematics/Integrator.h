//
//  Integrator.hpp
//  SnareNeedleModel
//
//  Created by Art Mahoney on 7/1/16.
//  Copyright © 2016 Art Mahoney. All rights reserved.
//

#ifndef Integrator_hpp
#define Integrator_hpp

#include <vector>





/**
 * This class defines a standard integrator.
 *
 */
template <class TimePoint, class State, class StorageState = State>
class Integrator {
    
public: // public class definitions
    

    
    // This class is meant to be defined by the user for their particular system.
    class DerivativeFunction {
    public: // public operators
        virtual State operator()(const TimePoint &time, const State &state) const = 0;
        
        
        
    };
    
    // This class should be used to implement a callback function that is executed at every timestep of the integration.  Note that this function is allowed to change the input argument.  If the return value is anything other than 0, then the integration will stop immediately.
    class CallbackFunction {
    public: // public operators
        virtual int operator()(const TimePoint &time, State &state) const;
    };
    
    /*
    // This class should be used to implement a state transition.
    class TransitionFunction {
    private: // private member variables
        TimePoint my_time;
    
    public: // public member functions
        TransitionFunction(const TimePoint &time);
        const TimePoint &getTime() const;
        
    public: // public operators
        virtual int operator()(const TimePoint &time, State &state) const;
    };
     */
    
public: // public type definitions
    
    typedef std::vector< TimePoint > TimeVector; // stores time points when integration is supposed to occur
    typedef std::vector< StorageState > ResultVector; // stores the results of integration
    //typedef std::vector< TransitionFunction > TransitionVector;
    
    
public: // private member functions
    
    // Implementation of the integration step that must be implemented by derived classes.
    virtual int integrationStep(const DerivativeFunction &function,
                                const TimePoint &current_time,
                                const TimePoint &next_time,
                                const StorageState &current_state,
                                StorageState &next_state) = 0;
    
    // Implementation of the integration step that incorporates a callback function.  Uses the integrationStep() function that doesn't incorporate the callback.
    /*int integrationStep(const DerivativeFunction &function,
                                const CallbackFunction &callback,
                                const TimePoint &current_time,
                                const TimePoint &next_time,
                                const State &current_state,
                                State &next_state) const;
    */
     
public: // public members
    // Default constructor... empty.
    Integrator() {};
    
    
    int integrate(typename TimeVector::iterator time_iterator_start,
                  typename TimeVector::iterator time_iterator_end,
                  typename ResultVector::iterator result_iterator); // integrates without a derivative function, assumes it's zero
    
    int integrate(const DerivativeFunction &function,
                  typename TimeVector::iterator time_iterator_start,
                  typename TimeVector::iterator time_iterator_end,
                  typename ResultVector::iterator result_iterator);
    
    int integrate(const DerivativeFunction &function,
                  const CallbackFunction &callback,
                  typename TimeVector::iterator time_iterator_start,
                  typename TimeVector::iterator time_iterator_end,
                  typename ResultVector::iterator result_iterator);
    
    


    
};





/**
  * This performs the integration using the virtual integrationStep function that has to be implemented.
 *
 */

template <class TimePoint, class State, class StorageState>
int Integrator<TimePoint, State, StorageState>::integrate(const DerivativeFunction &function,
                                                          typename TimeVector::iterator time_iterator_start,
                                                          typename TimeVector::iterator time_iterator_end,
                                                          typename ResultVector::iterator result_iterator) {

    return this->integrate(function, CallbackFunction(), time_iterator_start, time_iterator_end, result_iterator);
}


/**
 * This performs the integration using the virtual integrationStep function that has to be implemented.
 *
 */
template <class TimePoint, class State, class StorageState>
int Integrator<TimePoint, State, StorageState>::integrate(const DerivativeFunction &function,
                                                          const CallbackFunction &callback,
                                                          typename TimeVector::iterator time_iterator_start,
                                                          typename TimeVector::iterator time_iterator_end,
                                                          typename ResultVector::iterator result_iterator) {
    
   // if (time_vector.size() != result_vector.size())
   //     return 1;
    
    
    
    //typename TimeVector::const_iterator time_it = time_vector.begin();
    //typename ResultVector::iterator result_it = result_vector.begin();
    
    
    // Set the initial condition
    //*result_it = initial_state;
    
    // Increment the time and result iterators
    ++time_iterator_start;
    ++result_iterator;
    
    // Iterate through all the time points in the time vector
    while (time_iterator_start <= time_iterator_end) {
        
        
        
        // Perform the integration, save the result in the result iterator.
        if (int err = integrationStep(function, *(time_iterator_start-1), *(time_iterator_start), *(result_iterator-1), *(result_iterator))) {
            return err;
        }
        
        
        
        
        //std::cout << result_it->state << std::endl;
        
        ++result_iterator;
        ++time_iterator_start;
    }
    
    return 0;
}




/**
 * Implements an empty default callback function.
 *
 */
template<class TimePoint, class State, class StorageState>
int Integrator<TimePoint, State, StorageState>::CallbackFunction::operator()(const TimePoint &time, State &state) const {
    
    return 0;
}


/**
 * Implements an empty state transition function.
 *
 */
/*
template<class TimePoint, class State, class StorageState>
int Integrator<TimePoint, State, StorageState>::TransitionFunction::operator()(const TimePoint &time, State &state) const {
    
    return 0;
}
*/


#endif /* Integrator_hpp */

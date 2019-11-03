//
//  RK8Integrator.hpp
//  SnareNeedleModel
//
//  Created by Art Mahoney on 7/6/16.
//  Copyright © 2016 Art Mahoney. All rights reserved.
//

#ifndef RK8Integrator_hpp
#define RK8Integrator_hpp

#include "Integrator.h"
#include "State.h"


template<class TimePoint, class State>
class RK8Integrator  {
public:
    
    State next_state_tmp;
    State derivative_tmp;
    State k[13]; // stores intermediate results during the integration
    State multiplication_tmp;
    
    double c[13] = {
        0.0,
        1.0 / 18.0,
        1.0 / 12.0,
        1.0 / 8.0,
        5.0 / 16.0,
        3.0 / 8.0,
        59.0 / 400.0,
        93.0 / 200.0,
        5490023248.0 / 9719169821.0,
        13.0 / 20.0,
        1201146811.0 / 1299019798.0,
        1.0,
        1.0
    };
    
    double a[13][12] = {
        {
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        },
        {
            1.0 / 18.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        },
        {
            1.0 / 48.0,
            1.0 / 16.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        },
        {
            1.0 / 32.0,
            0.0,
            3.0 / 32.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        },
        {
            5.0 / 16.0,
            0.0,
            -75.0 / 64.0,
            75.0 / 64.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        },
        {
            3.0 / 80.0,
            0.0,
            0.0,
            3.0 / 16.0,
            3.0 / 20.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        },
        {
            29443841.0 / 614563906.0,
            0.0,
            0.0,
            77736538.0 / 692538347.0,
            -28693883.0 / 1125000000.0,
            23124283.0 / 1800000000.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        },
        {
            16016141.0 / 946692911.0,
            0.0,
            0.0,
            61564180.0 / 158732637.0,
            22789713.0 / 633445777.0,
            545815736.0 / 2771057229.0,
            -180193667.0 / 1043307555.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        },
        {
            39632708.0 / 573591083.0,
            0.0,
            0.0,
            -433636366.0 / 683701615.0,
            -421739975.0 / 2616292301.0,
            100302831.0 / 723423059.0,
            790204164.0 / 839813087.0,
            800635310.0 / 3783071287.0,
            0.0,
            0.0,
            0.0,
            0.0
        },
        {
            246121993.0 / 1340847787.0,
            0.0,
            0.0,
            -37695042795.0 / 15268766246.0,
            -309121744.0 / 1061227803.0,
            -12992083.0 / 490766935.0,
            6005943493.0 / 2108947869.0,
            393006217.0 / 1396673457.0,
            123872331.0 / 1001029789.0,
            0.0,
            0.0,
            0.0
        },
        {
            -1028468189.0 / 846180014.0,
            0.0,
            0.0,
            8478235783.0 / 508512852.0,
            1311729495.0 / 1432422823.0,
            -10304129995.0 / 1701304382.0,
            -48777925059.0 / 3047939560.0,
            15336726248.0 / 1032824649.0,
            -45442868181.0 / 3398467696.0,
            3065993473.0 / 597172653.0,
            0.0,
            0.0
        },
        {
            185892177.0 / 718116043.0,
            0.0,
            0.0,
            -3185094517.0 / 667107341.0,
            -477755414.0 / 1098053517.0,
            -703635378.0 / 230739211.0,
            5731566787.0 / 1027545527.0,
            5232866602.0 / 850066563.0,
            -4093664535.0 / 808688257.0,
            3962137247.0 / 1805957418.0,
            65686358.0 / 487910083.0,
            0.0
        },
        {
            403863854.0/491063109.0,
            0.0,
            0.0,
            -5068492393.0/434740067.0,
            -411421997.0/543043805.0,
            652783627.0/914296604.0,
            11173962825.0/925320556.0,
            -13158990841.0/6184727034.0,
            3936647629.0/1978049680.0,
            -160528059.0/685178525.0,
            248638103.0/1413531060.0,
            0.0
        }
    };
    
    double b[12] = {
        13451932.0 / 455176623.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -808719846.0 / 976000145.0,
        1757004468.0 / 5645159321.0,
        656045339.0 / 265891186.0,
        -3867574721.0 / 1518517206.0,
        465885868.0 / 322736535.0,
        53011238.0 / 667516719.0,
        2.0 / 45.0
    };
    
    double bhat[13] = {
        14005451.0 / 335480064.0,
        0.0,
        0.0,
        0.0,
        0.0,
        -59238493.0 / 1068277825.0,
        181606767.0 / 758867731.0,
        561292985.0 / 797845732.0,
        -1041891430.0 / 1371343529.0,
        760417239.0 / 1151165299.0,
        118820643.0 / 751138087.0,
        -528747749.0 / 2220607170.0,
        1.0 / 4.0
    };
    
    


    
public:
    
    RK8Integrator() {}

    
    
    template<class DerivativeFunction, class Window>
    int integrationStep(const DerivativeFunction &function, const TimePoint &current_time, const TimePoint &next_time, const State &current_state, State &next_state, const Window &window) {
        
        //typedef Eigen::Block<typename State::DataType, Window::dimension, State::derivative_dimension+1> StateWindow;
        //typedef Eigen::Block<const typename State::DataType, Window::dimension, State::derivative_dimension+1> ConstStateWindow;

        
        double current_time_d = static_cast<double>(current_time);
        double next_time_d = static_cast<double>(next_time);
        double h_d = next_time_d - current_time_d;
        

        
        /*
        ComputationState next_state_window
        
        */
        
        
        StateWindow<State, Window> next_state_window(next_state, window);
        ConstStateWindow<State, Window> current_state_window(current_state, window);
        StateWindow<State, Window> derivative_window(derivative_tmp, window);

        
        StateWindow<State, Window> k_window[] = {StateWindow<State, Window>(k[0], window),
                                                 StateWindow<State, Window>(k[1], window),
                                                 StateWindow<State, Window>(k[2], window),
                                                 StateWindow<State, Window>(k[3], window),
                                                 StateWindow<State, Window>(k[4], window),
                                                 StateWindow<State, Window>(k[5], window),
                                                 StateWindow<State, Window>(k[6], window),
                                                 StateWindow<State, Window>(k[7], window),
                                                 StateWindow<State, Window>(k[8], window),
                                                 StateWindow<State, Window>(k[9], window),
                                                 StateWindow<State, Window>(k[10], window),
                                                 StateWindow<State, Window>(k[11], window),
                                                 StateWindow<State, Window>(k[12], window)};
        
        
        
        
        /*
        
        DynamicStateWindow<State, Window> next_state_window(next_state, window);
        ConstDynamicStateWindow<State, Window> current_state_window(current_state, window);
        DynamicStateWindow<State, Window> derivative_window(derivative_tmp, window);
        
        
        DynamicStateWindow<State, Window> k_window[] = {DynamicStateWindow<State, Window>(k[0], window),
            DynamicStateWindow<State, Window>(k[1], window),
            DynamicStateWindow<State, Window>(k[2], window),
            DynamicStateWindow<State, Window>(k[3], window),
            DynamicStateWindow<State, Window>(k[4], window),
            DynamicStateWindow<State, Window>(k[5], window),
            DynamicStateWindow<State, Window>(k[6], window),
            DynamicStateWindow<State, Window>(k[7], window),
            DynamicStateWindow<State, Window>(k[8], window),
            DynamicStateWindow<State, Window>(k[9], window),
            DynamicStateWindow<State, Window>(k[10], window),
            DynamicStateWindow<State, Window>(k[11], window),
            DynamicStateWindow<State, Window>(k[12], window)};
        
        */
        
        
        
        
        next_state_window = current_state_window;
        
        for (int aa=0; aa<13; ++aa)
            k_window[aa] = current_state_window;
        
        
        for (int aa=0; aa<13; ++aa) {

            function(current_time + h_d*this->c[aa], k_window[aa], derivative_window);
        
            k_window[aa] = derivative_window;
            
            
            for (int bb=1+aa; bb<13; ++bb) {
                
                
                typename StateWindow<State, Window>::MatrixType tmp = k_window[aa].data();
                tmp *= h_d*this->a[bb][aa];
                k_window[bb].data() += tmp;
                
                
                //k_window[bb] += (h_d*this->a[bb][aa])*k_window[aa];
                
                
            }
            
            
            typename StateWindow<State, Window>::MatrixType tmp = k_window[aa].data();
            tmp *= (h_d*this->bhat[aa]);
            next_state_window.data() += tmp;
            
             
            //next_state_window += (h_d*this->bhat[aa])*k_window[aa];
            
        }
        
        
        /*
        next_state = current_state;
        
        for (int aa=0; aa<13; ++aa)
            k[aa] = current_state;
        
        for (int aa=0; aa<13; ++aa) {
            
            
            k[aa] = function(current_time + h_d*this->c[aa], k[aa], window);
            
            for (int bb=1+aa; bb<13; ++bb) {
                
                k[bb] += (h_d*this->a[bb][aa])*k[aa];
            }
            
            next_state += (h_d*this->bhat[aa])*k[aa];
        }
         */
        
        return 0;
    }

    
    
    
    /**
     * This performs the integration using the virtual integrationStep function that has to be implemented.
     *
     */
    template<class DerivativeFunction, class Window>
    int integrate(const DerivativeFunction &function, typename std::vector<TimePoint>::iterator time_iterator_start,
                  typename std::vector<TimePoint>::iterator time_iterator_end, typename std::vector<State>::iterator result_iterator, const Window &window) {

        // Increment the time and result iterators
        ++time_iterator_start;
        ++result_iterator;
        
        // Iterate through all the time points in the time vector
        while (time_iterator_start <= time_iterator_end) {
            
            // Perform the integration, save the result in the result iterator.
            if (int err = integrationStep<DerivativeFunction, Window>(function, *(time_iterator_start-1), *(time_iterator_start), *(result_iterator-1), *(result_iterator), window)) {
                return err;
            }
            
            ++result_iterator;
            ++time_iterator_start;
        }
        
        return 0;
    }

    

    
};












#endif /* RK8Integrator_hpp */

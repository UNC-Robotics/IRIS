//
//  Tools.hpp
//  SnareNeedleModel
//
//  Created by Art Mahoney on 7/1/16.
//  Copyright © 2016 Art Mahoney. All rights reserved.
//

#ifndef Tools_hpp
#define Tools_hpp

#include <iostream>
#include <vector>
#include "Eigen/Dense"

namespace Tools {
    
    
    template<class TimePoint>
    void linspace(const TimePoint &left, const TimePoint &right, unsigned int n, std::vector<TimePoint> &time_array) {
        
        double left_d = static_cast<double>(left);
        double right_d = static_cast<double>(right);
        
        double h = (right_d-left_d)/n;
        
        time_array.resize(n+1);
        
        unsigned int count = 0;
        typename std::vector<TimePoint >::iterator it;
        for (it = time_array.begin(); it!=time_array.end(); ++it) {
            
            *it = left + count*h;

            ++count;
        }
        
        return;
    }
     
    
    
    template <typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived> &v) {
        
        //BOOST_STATIC_ASSERT(Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1);
        Eigen::Matrix<typename Derived::Scalar, 3, 3> out;
        out <<     0, -v[2],  v[1],
        v[2],     0, -v[0],
        -v[1],  v[0],     0;
        
        return out;
    }
    
    
    
    
    

    

    
    
    
};

#endif /* Tools_hpp */

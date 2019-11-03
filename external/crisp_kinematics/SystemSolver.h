//
//  SystemSolver.h
//  SnareNeedleModel
//
//  Created by Art Mahoney on 8/10/16.
//  Copyright © 2016 Art Mahoney. All rights reserved.
//

#ifndef SystemSolver_h
#define SystemSolver_h

#define SHOW_SOLVER_ERROR 0

#include <chrono>
#include "State.h"



/**
 * This class solves the system for the initial state that satisfies the system's constraints.
 *
 */
template<class System>
class SystemSolver {
private:
    
    
    static int MAX_ITERATIONS() { return 1000; }; // the max number of iteration steps that the hill climber should take
    static double ZERO_SINGULAR_VALUE() { return 1e-8; }; // a singular value below this number is considered 0
    static double TRUST_REGION() { return 0.01; }; // the radius of the trust region, limits the step in initial state
    static double CONVERGED() { return 1e-6; }; // the solver is considered to be convered to the solution of the residual changes less than this amount
    
    int my_iterations;
    
    System &m_system;
    
public:
    /**
     * Constructor that takes in a reference to the system.
     *
     */
    SystemSolver(System &system) : m_system(system), my_iterations(0) {}
    
    int iterations() { return my_iterations; };
    
    
    /**
     * Solves for the initial states that satsify the system's constraints.  It uses a simple hill-climbing method based on the singular value decomposition.
     *
     */
    int solve(Eigen::Matrix<double, System::state_dimension, 1> &my_initial_state) {
        
        //std::cout << "SOLVER:" << std::endl;
        
        Eigen::Matrix<double, System::state_dimension, 1> solution_current = my_initial_state; // initialize the current solution
        
        double residual = std::numeric_limits<double>::max(); // initialize the residual to the maximum a double can hold
        
        
        // Iterate for the max number of iterations,
        for (my_iterations = 0; my_iterations < MAX_ITERATIONS(); ++my_iterations) {
            
            
            // Compute the current residual and constraint jacobian.
            typename System::Solution integrator_solution;
            m_system.initialize(integrator_solution, solution_current);
            
            


            m_system.integrate(integrator_solution);


            
            
            Eigen::Matrix<double, System::constraint_dimension, 1> residual_vector = integrator_solution.constraints.vector();
            Eigen::Matrix<double, System::constraint_dimension, System::state_dimension> jacobian = integrator_solution.constraints.derivative();
            
            
            // Compute the singular value decomposition.
            Eigen::JacobiSVD<Eigen::Matrix<double, System::constraint_dimension, System::state_dimension>> svd_decomposition(jacobian,Eigen::ComputeFullU | Eigen::ComputeFullV);
            svd_decomposition.setThreshold(ZERO_SINGULAR_VALUE()); // set the singular value threshold, below which is considered zero
            //double cond = svd_decomposition.singularValues()(0) / svd_decomposition.singularValues()(svd_decomposition.singularValues().size()-1);
            //std::cout << "condition number " << cond << std::endl;
            // Solve for a change in the initial state.
            Eigen::Matrix<double, System::state_dimension, 1> delta = svd_decomposition.solve(residual_vector);
            
            
            
            //Eigen::Matrix<double, System::state_dimension, 1> delta =  jacobian.householderQr().solve(residual_vector);
            

            /*
            Eigen::Matrix<double, System::constraint_dimension, System::constraint_dimension> JJt = jacobian*jacobian.transpose();
            Eigen::LLT<Eigen::Matrix<double, System::constraint_dimension, System::constraint_dimension>> llt(JJt);
            //Eigen::Matrix<double, System::constraint_dimension, 1> tmp = residual_vector;
            llt.solveInPlace(residual_vector);
            //Eigen::Matrix<double, System::state_dimension, 1> delta = jacobian.transpose()*llt.solve(residual_vector);
            Eigen::Matrix<double, System::state_dimension, 1> delta = jacobian.transpose()*residual_vector;
            */

            
            
            // Implement the trust region, but scaling the step in the initial state to be less than the desired radius.
            Eigen::Matrix<double, System::state_dimension, 1> scaled_delta = delta;
            if (delta.norm() >= TRUST_REGION()) {
                scaled_delta.normalize();
                scaled_delta *= TRUST_REGION();
            }

            double new_residual = integrator_solution.constraints.vector().norm();
            
           //std::cout << my_iterations << " " << new_residual << std::endl;
            
            // Check if converged or if the residual has increased
            /*if (fabs(new_residual-residual) <= CONVERGED()) { // residual convereged, stop
                
                my_initial_state = solution_current;
                
                return my_iterations;
            }*/
            if (new_residual <= CONVERGED()) {
                my_initial_state = solution_current;
                //std::cout << new_residual << std::endl;
                return my_iterations;
            }
            else if (new_residual >= residual) { // residual increased, stop
#if SHOW_SOLVER_ERROR
                std::cout << "SOLVER: error - the residual increased in the process of solving!" << std::endl;
#endif
                //break;
                return -1;
            }
            else { // keep iterating
                solution_current -= scaled_delta;
                residual = new_residual;
            }
            
            
        }
        
        
        
        
        return 1;
    }
    
    

};


#endif /* SystemSolver_h */

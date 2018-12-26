#ifndef MPC_H
#define MPC_H

#include <vector>
#include <memory>
#include <cppad/cppad.hpp>
#include "helper.h"

struct Solution 
{
    const double steering;     // steering control angle in radians
    const double accelerator;  // accelerator control value [-1...1]
    // Predicted trajectory (green line)
    const std::vector<double> x;
    const std::vector<double> y;
    
    // Reference line (yellow line)
    const std::vector<double> next_x;
    const std::vector<double> next_y;

    Solution(
      double steering, 
      double accelerator, 
      std::vector<double> x, 
      std::vector<double> y,
      std::vector<double> next_x,
      std::vector<double> next_y) 
        : steering(steering), accelerator(accelerator), x(x), y(y), next_x(next_x), next_y(next_y) {} 
};

class MPC {
public:

    typedef CppAD::AD<double> ADdouble;
    typedef CPPAD_TESTVECTOR(ADdouble) ADvector;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    /** Vehicle's state. */
    struct State 
    {
        const ADdouble x;
        const ADdouble y;
        const ADdouble psi;
        const ADdouble v;
        const ADdouble cte;
        const ADdouble epsi;

        State(ADdouble x, ADdouble y, ADdouble psi, ADdouble v, ADdouble cte, ADdouble epsi) : 
          x(x), y(y), psi(psi), v(v), cte(cte), epsi(epsi)
        {}

        inline std::vector<ADdouble> values() const
        {
          return { x, y, psi, v, cte, epsi };
        }
    };

    struct StateIndeces
    {
      const size_t x_start;
      const size_t y_start;
      const size_t psi_start; 
      const size_t v_start; 
      const size_t cte_start; 
      const size_t epsi_start;
      const size_t delta_start; 
      const size_t a_start;

      std::vector<size_t> list;

      StateIndeces(size_t n) : 
        x_start(0), 
        y_start(n), 
        psi_start(n * 2), 
        v_start(n * 3), 
        cte_start(n * 4), 
        epsi_start(n * 5),
        delta_start(n * 6), 
        a_start(n * 7 - 1) 
        { 
          list = { x_start, y_start, psi_start, v_start, cte_start, epsi_start };
        } 
    };

    const int  POLYNOMIAL_ORDER {3};   // order of the polynomial

    const double Lf = 2.67;            // Distance: vehicle's front to it's center of gravity
    
    MPC();
    virtual ~MPC();
    
    /** Solve the model given an initial state and polynomial coefficients.
     * @param state: the current state of the vehicle
     * @param coeffs: the polynomial coefficients
     */
     std::shared_ptr<Solution> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
    
    /** Need this for CppAD::ipopt::solve to work
     * @param fg: vector of constraints
     * @param vars: vector of variables
     */
    void operator()(ADvector & fg, const ADvector & vars);
    
private:
    
    /** Updates vector of constraints according to the vector of vars and states at timestep t and t+1
     * @param fg: vector of constraints
     * @param vars: vector of variables
     * @param t: timestep
     */
    void update(ADvector& fg, const ADvector& vars, int t);

    /** Apply cost weights 
     * @param fg: vector of constraints
     * @param vars: vector of variables
     * @param t: timestep
    */
    void apply_cost(ADvector & fg, const ADvector & vars, int t);

    inline ADdouble dx(State & state0, State & state1) const 
    { 
      return state1.x - (state0.x + state0.v * CppAD::cos(state0.psi) * this->dt);
    };

    inline ADdouble dy(State & state0, State & state1) const
    {
        return state1.y - (state0.y + state0.v * CppAD::sin(state0.psi) * this->dt);
    };

    inline ADdouble dpsi(State & state0, State & state1, ADdouble & delta0) const
    {
        return state1.psi - (state0.psi - state0.v / this->Lf * delta0 * this->dt);
    };

    inline ADdouble dv(State & state0, State & state1, ADdouble & a) const 
    {
        return state1.v - (state0.v + a * this->dt);
    };

    inline ADdouble dcte(State & state0, State & state1, ADdouble & f0) const 
    {
        return  state1.cte - (f0 - state0.y + state0.v * CppAD::sin(state0.epsi) * this->dt);
    };

    inline ADdouble depsi(State & state0, State & state1, ADdouble & psides0, ADdouble & delta0) const 
    {
        return state1.epsi - ((state0.psi - psides0) + state0.v / this->Lf * delta0 * this->dt);
    };
    
    // timesteps count
    const size_t n { 10 };  
    // timestep duration, s                
    const double dt { 0.1 };  
    // state dimensionality             
    const size_t state_dimension{ 6 };   
    // reference speed
    const double ref_v { 85 };           
    //start indices of the state
    const StateIndeces si{ n }; 
    // cost weights
    const std::vector<double> cost_weights = {2, 600, 0.2, 100, 2, 0.002, 0.05, 600, 1};

    // number of independent variables
    const size_t number_vars { (n * state_dimension + (this->n - 1) * 2) };
    // Number of constraints 
    const size_t number_constraints { (n * state_dimension) };
    // polynomial coefficients
    Eigen::VectorXd poly_coeffs;           
};

#endif /* MPC_H */

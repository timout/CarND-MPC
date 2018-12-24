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
    // points of minimum cost trajectory returned from the solver (green line)
    const std::vector<double> x;
    const std::vector<double> y;
    
    // Reference points in the vehicle's coordinate system (yellow line)
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
        ADdouble x;
        ADdouble y;
        ADdouble psi;
        ADdouble v;
        ADdouble cte;
        ADdouble epsi;

        State(ADdouble x, ADdouble y, ADdouble psi, ADdouble v, ADdouble cte, ADdouble epsi) : 
          x(x), y(y), psi(psi), v(v), cte(cte), epsi(epsi)
        {}

        std::vector<ADdouble> values() 
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
    
    /** Updates vector of constraints according to the vector of vars and states at
     * timestep t and t+1
     * @param fg: vector of constraints
     * @param vars: vector of variables
     * @param t: timestep
     * @param state_1: state at time t+1
     * @param state_0: state at time t
     */
    void update(
      ADvector& fg, 
      const ADvector& vars, 
      int t,
      State & state_1,
      State & state_0);

    inline ADdouble dx(ADdouble & x_1, ADdouble & x_0, ADdouble & v0, ADdouble & psi0) const 
    { 
      return x_1 - (x_0 + v0 * CppAD::cos(psi0) * this->dt);
    };

    inline ADdouble dy(ADdouble & y_1, ADdouble & y_0, ADdouble & v0, ADdouble & psi0) const
    {
        return y_1 - (y_0 + v0 * CppAD::sin(psi0) * this->dt);
    };

    inline ADdouble dpsi(ADdouble & psi_1, ADdouble & psi_0, ADdouble & v0, ADdouble & delta0) const
    {
        return psi_1 - (psi_0 - v0 / this->Lf * delta0 * this->dt);
    };

    inline ADdouble dv(ADdouble & v_1, ADdouble & v_0, ADdouble & a) const 
    {
        return v_1 - (v_0 + a * this->dt);
    };

    inline ADdouble dcte(ADdouble & cte_1, ADdouble & f0, ADdouble & y0, ADdouble & v0, ADdouble & epsi0) const 
    {
        return  cte_1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * this->dt);
    };

    inline ADdouble depsi(ADdouble & epsi1, ADdouble & psi0, ADdouble & psides0, ADdouble & v0, ADdouble & delta0) const 
    {
        return epsi1 - ((psi0 - psides0) + v0 / this->Lf * delta0 * this->dt);
    };
     
    /** Creates state for a given timestep index
     * @param t: ndex
     * @param vars: variables
     */
    State create_state(int t, const ADvector & vars);
    
    const size_t n = 10;                  // timesteps count
    const double dt {0.1};               // timestep duration, s
    const size_t state_dimension = 6;   // state dimensionality

    const double ref_v {60};           // reference speed

    const StateIndeces indeces{n};  //start indices of the state
    
    // cost weights
    const std::vector<double> cost_weights = {.5, 500, .4, 1, 1, .003, .03, 500, 1};

    // number of independent variables
    const size_t number_vars { (n * state_dimension + (this->n - 1) * 2) };
    /// Number of constraints 
    const size_t number_constraints = (n * state_dimension);

    Eigen::VectorXd poly_coeffs;  // polynomial coefficients          
};

#endif /* MPC_H */

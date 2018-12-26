#include "MPC.h"
#include <cppad/ipopt/solve.hpp>

MPC::MPC() {}

MPC::~MPC() {}

void MPC::operator()(ADvector & fg, const ADvector & vars) 
{
    fg[0] = 0;
    for (int t = 0; t < this->n; t++) apply_cost(fg, vars, t);
    // Since cost is located at 0: add 1 to each of the starting indices
    for (size_t i: si.list) fg[1 + i] = vars[i];
    for (int t = 1; t < this->n; ++t) update(fg, vars, t);
}

void MPC::apply_cost(ADvector & fg, const ADvector & vars, int t)
{       
    // Reference State Cost
    fg[0] += cost_weights[0] * CppAD::pow(vars[si.cte_start + t], 2);
    fg[0] += cost_weights[1] * CppAD::pow(vars[si.epsi_start + t], 2);
    fg[0] += cost_weights[2] * CppAD::pow(vars[si.v_start + t] - ref_v, 2);
    // Actuators use cost.
    if (t < this->n - 1) { 
        // Minimizes the use of actuators.
        fg[0] += cost_weights[3] * CppAD::pow(vars[si.delta_start + t], 2);
        fg[0] += cost_weights[4] * CppAD::pow(vars[si.a_start + t], 2);
        // Extra cost to reduce acceleration when cte goes up for smoother driving
        fg[0] += cost_weights[5] * CppAD::pow(vars[si.a_start + t] * vars[si.v_start + t] * vars[si.cte_start + t], 2);
        // Extra cost to reduce acceleration when epsi goes up for smoother driving
        fg[0] += cost_weights[6] * CppAD::pow(vars[si.a_start + t] * vars[si.v_start + t] * vars[si.epsi_start + t], 2);

        // Actuator change rate cost
        if (t < this->n - 2) {          
            // Minimize the change in actuator 
            fg[0] += cost_weights[7] * CppAD::pow(vars[si.delta_start + t + 1] - vars[si.delta_start + t], 2);
            fg[0] += cost_weights[8] * CppAD::pow(vars[si.a_start + t + 1] - vars[si.a_start + t], 2);
        }
    }
}

void MPC::update(ADvector & fg, const ADvector & vars, int t1)
{
    int t0 = t1 - 1;
    // State at time t.
    State state0(
      vars[si.x_start + t0], 
      vars[si.y_start + t0],
      vars[si.psi_start + t0], 
      vars[si.v_start + t0],
      vars[si.cte_start + t0], 
      vars[si.epsi_start + t0]
    );
    // State at time t+1 .
    State state1(
      vars[si.x_start + t1], 
      vars[si.y_start + t1],
      vars[si.psi_start + t1], 
      vars[si.v_start + t1],
      vars[si.cte_start + t1], 
      vars[si.epsi_start + t1]
    );

    ADdouble delta0 = vars[si.delta_start + t0];
    ADdouble a0 = vars[si.a_start + t0];   
    ADdouble f0 = poly_coeffs[0]; 
    ADdouble df0 = 0;
    // Computes f(x) at state_0 and it's derivative
    for (int i = 1; i < poly_coeffs.size(); ++i) {
        f0 += poly_coeffs[i] * pow(state0.x, i);
        df0 += i * poly_coeffs[i] * pow(state0.x, i-1);
    }
    // Computes desired psi as the arctangent of the derivative.
    ADdouble psides0 = CppAD::atan(df0);
    // Updates constraints
    fg[si.x_start + t1 + 1] = dx(state0, state1);
    fg[si.y_start + t1 + 1] = dy(state0, state1);
    fg[si.psi_start + t1 + 1] = dpsi(state0, state1, delta0);
    fg[si.v_start + t1 + 1] = dv(state0, state1, a0); 
    fg[si.cte_start + t1 + 1] = dcte(state0, state1, f0);
    fg[si.epsi_start + t1 + 1] = depsi(state0, state1, psides0, delta0);
}

std::shared_ptr<Solution> MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) {

    this->poly_coeffs = coeffs;
  
    std::vector<double> state_vals = { x0[0], x0[1], x0[2], x0[3], x0[4], x0[5] };
    
    // Initial value of the independent variables. Should be 0 except for the initial values.
    Dvector vars(number_vars);
    for (int i = 0; i < number_vars; ++i) vars[i] = 0.0;
    // Set the initial variable values
    for (int i = 0; i < si.list.size(); ++i) {
        vars[si.list[i]] = state_vals[i];
    }

    // Lower and upper limits for x
    Dvector vars_lowerbound(number_vars);
    Dvector vars_upperbound(number_vars);
    
    // Set all non-actuators upper and lowerlimits to the max negative and positive values.
    for (int i = 0; i < si.delta_start; ++i) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
    for (int i = si.delta_start; i < si.a_start; ++i) {
        vars_lowerbound[i] = deg2rad(-25);
        vars_upperbound[i] = deg2rad(25);
    }
    // Acceleration/decceleration upper and lower limits.
    for (int i = si.a_start; i < number_vars; ++i) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }
    // Lower and upper limits for constraints : 0
    Dvector constraints_lowerbound(number_constraints);
    Dvector constraints_upperbound(number_constraints);
    for (int i = 0; i < number_constraints; ++i) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    // the initial state indices.
    for (int i = 0; i < si.list.size(); ++i) {
        constraints_lowerbound[si.list[i]] = state_vals[i];
        constraints_upperbound[si.list[i]] = state_vals[i];
    }
    std::string options;
    options += "Integer print_level  0\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    
    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;  
    // solve the problem
    CppAD::ipopt::solve<Dvector, MPC>(
      options, 
      vars,
      vars_lowerbound,
      vars_upperbound, 
      constraints_lowerbound,
      constraints_upperbound,
      *this,
      solution);
    
    double steering = solution.x[si.delta_start]/deg2rad(25);
    double accelerator = solution.x[si.a_start];
    
    // fill minimum cost trajectory points
    std::vector<double> x = std::vector<double>(this->n - 1);
    std::vector<double> y = std::vector<double>(this->n - 1);
    for (int i = 0; i < this->n - 1; ++i) {
        x.push_back(solution.x[si.x_start + i + 1]);
        y.push_back(solution.x[si.y_start + i + 1]);
    }
    // fill the reference points in vehicles coordinate space
    int num_points = 25;
    std::vector<double> next_x = std::vector<double>(num_points-1);
    std::vector<double> next_y = std::vector<double>(num_points-1);
    for(int i = 1; i < num_points; ++i) {
        next_x.push_back(2 * i);
        next_y.push_back(polyeval(poly_coeffs, 2 * i));
    }
    return std::make_shared<Solution>(steering, accelerator, x, y, next_x, next_y);
}
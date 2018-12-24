#include "MPC.h"
#include <cppad/ipopt/solve.hpp>

MPC::MPC() {}

MPC::~MPC() {}

void MPC::operator()(ADvector & fg, const ADvector & vars) 
{
    fg[0] = 0;
    for (int t = 0; t < this->n; t++) {
        ADdouble cte = vars[indeces.cte_start + t];
        ADdouble epsi = vars[indeces.epsi_start + t];
        ADdouble v = vars[indeces.v_start + t];
        // The part of the cost based on the reference state.
        // Expresses general concern about cte, epsi and maintaining speed
        fg[0] += cost_weights[0] * CppAD::pow(cte, 2);
        fg[0] += cost_weights[1] * CppAD::pow(epsi, 2);
        fg[0] += cost_weights[2] * CppAD::pow(v - ref_v, 2);
        // Usage of actuators.
        if (t < this->n - 1) { 
            ADdouble delta = vars[indeces.delta_start + t];
            ADdouble accel = vars[indeces.a_start + t];
            // Minimizes the use of actuators.
            fg[0] += cost_weights[3] * CppAD::pow(delta, 2);
            fg[0] += cost_weights[4] * CppAD::pow(accel, 2);
            // Adds additional cost for co-dependence of curent speed, acceleration and cte/epsi
            fg[0] += cost_weights[5] * CppAD::pow(accel * v * cte, 2);
            fg[0] += cost_weights[6] * CppAD::pow(accel * v * epsi, 2);

            if (t < this->n - 2) {          
              ADdouble next_delta = vars[indeces.delta_start + t + 1];
              ADdouble next_accel = vars[indeces.a_start + t + 1];
              // Minimizes the value gap between sequential actuations.
              fg[0] += cost_weights[7] * CppAD::pow(next_delta - delta, 2);
              fg[0] += cost_weights[8] * CppAD::pow(next_accel - accel, 2);
            }
        }
    }

    for (size_t i: indeces.list) fg[1 + i] = vars[i];
    for (int t = 1; t < this->n; ++t) update(fg, vars, t);
}

void MPC::update(ADvector & fg, const ADvector & vars, int t)
{
    // State at time t.
    State state_0(
      vars[indeces.x_start + t - 1], 
      vars[indeces.y_start + t - 1],
      vars[indeces.psi_start + t - 1], 
      vars[indeces.v_start + t - 1],
      vars[indeces.cte_start + t - 1], 
      vars[indeces.epsi_start + t - 1]
    );
    // State at time t+1 .
    State state_1(
      vars[indeces.x_start + t], 
      vars[indeces.y_start + t],
      vars[indeces.psi_start + t], 
      vars[indeces.v_start + t],
      vars[indeces.cte_start + t], 
      vars[indeces.epsi_start + t]
    );

    ADdouble delta0 = vars[indeces.delta_start + t - 1];
    ADdouble a0 = vars[indeces.a_start + t - 1];   
    ADdouble f0 = poly_coeffs[0]; 
    ADdouble df0 = 0;
    // Computes f(x) at state_0 and it's derivative
    for (int i = 1; i < poly_coeffs.size(); ++i) {
        f0 += poly_coeffs[i] * pow(state_0.x, i);
        df0 += i * poly_coeffs[i] * pow(state_0.x, i-1);
    }
    // Computes desired psi as the arctangent of the derivative.
    ADdouble psides0 = CppAD::atan(df0);
    // Updates constraints
    fg[indeces.x_start + t + 1] = dx(state_0, state_1);
    fg[indeces.y_start + t + 1] = dy(state_0, state_1);
    fg[indeces.psi_start + t + 1] = dpsi(state_0, state_1, delta0);
    fg[indeces.v_start + t + 1] = dv(state_0, state_1, a0); 
    fg[indeces.cte_start + t + 1] = dcte(state_0, state_1, f0);
    fg[indeces.epsi_start + t + 1] = depsi(state_0, state_1, psides0, delta0);
}

std::shared_ptr<Solution> MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) {

    this->poly_coeffs = coeffs;
  
    std::vector<double> state_vals = { x0[0], x0[1], x0[2], x0[3], x0[4], x0[5] };
    
    // Initial value of the independent variables. Should be 0 except for the initial values.
    Dvector vars(number_vars);
    for (int i = 0; i < number_vars; ++i) vars[i] = 0.0;
    // Set the initial variable values
    for (int i = 0; i < indeces.list.size(); ++i) {
        vars[indeces.list[i]] = state_vals[i];
    }

    // Lower and upper limits for x
    Dvector vars_lowerbound(number_vars);
    Dvector vars_upperbound(number_vars);
    
    // Set all non-actuators upper and lowerlimits to the max negative and positive values.
    for (int i = 0; i < indeces.delta_start; ++i) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
    for (int i = indeces.delta_start; i < indeces.a_start; ++i) {
        vars_lowerbound[i] = deg2rad(-25);
        vars_upperbound[i] = deg2rad(25);
    }
    // Acceleration/decceleration upper and lower limits.
    for (int i = indeces.a_start; i < number_vars; ++i) {
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
    for (int i = 0; i < indeces.list.size(); ++i) {
        constraints_lowerbound[indeces.list[i]] = state_vals[i];
        constraints_upperbound[indeces.list[i]] = state_vals[i];
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
    
    double steering = solution.x[indeces.delta_start]/deg2rad(25);
    double accelerator = solution.x[indeces.a_start];
    
    // fill minimum cost trajectory points
    std::vector<double> x = std::vector<double>(this->n - 1);
    std::vector<double> y = std::vector<double>(this->n - 1);
    for (int i = 0; i < this->n - 1; ++i) {
        x.push_back(solution.x[indeces.x_start + i + 1]);
        y.push_back(solution.x[indeces.y_start + i + 1]);
    }
    // fill the reference points in vehicles coordinate space
    int num_points = 20;
    std::vector<double> next_x = std::vector<double>(num_points-1);
    std::vector<double> next_y = std::vector<double>(num_points-1);
    for(int i = 1; i < 20; ++i) {
        next_x.push_back(2 * i);
        next_y.push_back(polyeval(poly_coeffs, 2 * i));
    }
    return std::make_shared<Solution>(steering, accelerator, x, y, next_x, next_y);
}
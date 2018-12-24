#include "MPC.h"
#include <cppad/ipopt/solve.hpp>

MPC::MPC() {}

MPC::~MPC() {}

void MPC::operator()(ADvector& fg, const ADvector& vars) 
{
    fg[0] = 0;
    ADdouble curr_cte = 0;
    ADdouble curr_epsi = 0;
    ADdouble curr_v = 0;
    ADdouble curr_delta = 0;
    ADdouble curr_accel = 0;
    ADdouble next_delta = 0;
    ADdouble next_accel = 0;
    
    for (int t = 0; t < this->n; t++) {
        curr_cte = vars[indeces.cte_start + t];
        curr_epsi = vars[indeces.epsi_start + t];
        curr_v = vars[indeces.v_start + t];
        // The part of the cost based on the reference state.
        // Expresses general concern about cte, epsi and maintaining speed
        fg[0] += cost_weights[0] * CppAD::pow(curr_cte, 2);
        fg[0] += cost_weights[1] * CppAD::pow(curr_epsi, 2);
        fg[0] += cost_weights[2] * CppAD::pow(curr_v - ref_v, 2);
        // Usage of actuators.
        if (t < this->n - 1) { 
            curr_delta = vars[indeces.delta_start + t];
            curr_accel = vars[indeces.a_start + t];
            // Minimizes the use of actuators.
            fg[0] += cost_weights[3] * CppAD::pow(curr_delta, 2);
            fg[0] += cost_weights[4] * CppAD::pow(curr_accel, 2);
            // Adds additional cost for co-dependence of curent speed, acceleration and cte/epsi
            fg[0] += cost_weights[5] * CppAD::pow(curr_accel * curr_v * curr_cte, 2);
            fg[0] += cost_weights[6] * CppAD::pow(curr_accel * curr_v * curr_epsi, 2);
        }
        if (t < this->n - 2) {          
            next_delta = vars[indeces.delta_start + t + 1];
            next_accel = vars[indeces.a_start + t + 1];
            // Minimizes the value gap between sequential actuations.
            fg[0] += cost_weights[7] * CppAD::pow(next_delta - curr_delta, 2);
            fg[0] += cost_weights[8] * CppAD::pow(next_accel - curr_accel, 2);
        }
    }

    for (size_t i: indeces.list) fg[1 + i] = vars[i];
    
    for (int t = 1; t < this->n; ++t) {
        // The state at time t+1 .
        State state_1 = create_state(t, vars);
        // The state at time t.
        State state_0 = create_state(t-1, vars);
        update(fg, vars, t, state_1, state_0);
    }
}

void MPC::update(
  ADvector & fg, 
  const ADvector & vars, 
  int t, 
  State & state_1, 
  State & state_0) 
{
    ADdouble delta0 = vars[indeces.delta_start + t - 1];
    ADdouble a0 = vars[indeces.a_start + t - 1];   
    ADdouble f0 = 0;
    ADdouble df0 = 0;
    // Computes f(x) at state_0 and it's derivative
    for (int i = 0; i < poly_coeffs.size(); ++i) {
        f0 += poly_coeffs[i] * pow(state_0.x, i);
        if (i > 0) {
            df0 += i * poly_coeffs[i] * pow(state_0.x, i-1);
        }
    }
    // Computes desired psi as the arctangent of the derivative.
    ADdouble psides0 = CppAD::atan(df0);
    // Updates constraints
    fg[indeces.x_start + t + 1] = dx(state_1.x, state_0.x, state_0.v, state_0.psi);
    fg[indeces.y_start + t + 1] = dy(state_1.y, state_0.y, state_0.v, state_0.psi);
    fg[indeces.psi_start + t + 1] = dpsi(state_1.psi, state_0.psi, state_0.v, delta0);
    fg[indeces.v_start + t + 1] = dv(state_1.v, state_0.v, a0); //state_1.v - (state_0.v + a0 * this->dt)


    fg[indeces.cte_start + t + 1] = dcte(state_1.cte, f0, state_0.y, state_0.v, state_0.epsi);
    fg[indeces.epsi_start + t + 1] = depsi(state_1.epsi, state_0.psi, psides0, state_0.v, delta0);
}

MPC::State MPC::create_state(int t, const ADvector & vars) {
    return State(
      vars[indeces.x_start + t], 
      vars[indeces.y_start + t],
      vars[indeces.psi_start + t], 
      vars[indeces.v_start + t],
      vars[indeces.cte_start + t], 
      vars[indeces.epsi_start + t]
    );
}

std::shared_ptr<Solution> MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) {

    this->poly_coeffs = coeffs;
  
    std::vector<double> state_vals = { x0[0], x0[1], x0[2], x0[3], x0[4], x0[5] };
    
    // Initial value of the independent variables.
    // Should be 0 except for the initial values.
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
    std::vector<double> tx = std::vector<double>(this->n - 1);
    std::vector<double> ty = std::vector<double>(this->n - 1);
    for (int i = 0; i < this->n - 1; ++i) {
        tx.push_back(solution.x[indeces.x_start + i + 1]);
        ty.push_back(solution.x[indeces.y_start + i + 1]);
    }
    // fill the reference points in vehicles coordinate space
    int num_points = 20;
    std::vector<double> next_tx = std::vector<double>(num_points-1);
    std::vector<double> next_ty = std::vector<double>(num_points-1);
    for(int i = 1; i < 20; ++i) {
        next_tx.push_back(2 * i);
        next_ty.push_back(polyeval(poly_coeffs, 2 * i));
    }
    return std::make_shared<Solution>(steering, accelerator, tx, ty, next_tx, next_ty);
}
#pragma once

#include <orochi_hx/model/orochi_hx_robot_model.h>
#include <orochi_hx/control/orochi_hx_controller.h>
#include <vector>
#include <Eigen/Core>

double nonlinearWrenchAllocationMinObjective(const std::vector<double>& x, std::vector<double>& grad, void* ptr);
void nonlinearWrenchAllocationEqConstraints(unsigned m, double* result, unsigned n, const double* x, double* grad,
                                            void* ptr);

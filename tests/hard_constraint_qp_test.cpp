#include <cmath>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam_unstable/linear/QPSolver.h>
#include <iostream>

int main() {
    const gtsam::Key state_key = gtsam::Symbol('x', 0);
    const gtsam::Key dual_key = gtsam::Symbol('q', 0);

    // Unconstrained objective: minimize 0.5 * x^2.
    gtsam::GaussianFactorGraph cost;
    cost.emplace_shared<gtsam::JacobianFactor>(state_key, gtsam::Matrix11::Identity(),
                                               gtsam::Vector1::Zero(),
                                               gtsam::noiseModel::Unit::Create(1));

    // Hard constraint: x >= 1, represented as -x <= -1.
    gtsam::InequalityFactorGraph inequalities;
    inequalities.add(state_key, gtsam::RowVector::Constant(1, -1.0), -1.0, dual_key);

    const gtsam::QP problem(cost, gtsam::EqualityFactorGraph{}, inequalities);
    gtsam::VectorValues feasible_initialization;
    feasible_initialization.insert(state_key, gtsam::Vector1(1.0));
    const gtsam::VectorValues solution =
        gtsam::QPSolver(problem).optimize(feasible_initialization).first;
    const double value = solution.at(state_key)(0);

    if (value < 1.0 - 1.0e-7 || std::abs(value - 1.0) > 1.0e-6) {
        std::cerr << "Hard-constraint QP returned x=" << value << ", expected x=1\n";
        return 1;
    }

    std::cout << "Hard-constraint QP test passed\n";
    return 0;
}

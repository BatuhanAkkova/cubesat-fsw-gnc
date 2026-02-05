#pragma once
#include <functional>

namespace sim {
namespace engine {

/**
 * @brief Runge-Kutta 4th Order Integrator
 * @tparam StateType The type of the state vector (must support arithmetic operations).
 */
template <typename StateType>
class Integrator {
public:
    using ODE = std::function<StateType(double t, const StateType& y)>;

    /**
     * @brief Perform a single RK4 step.
     * 
     * @param t Current time.
     * @param y Current state.
     * @param dt Time step.
     * @param f Function that computes the derivative dy/dt = f(t, y).
     * @return StateType The new state at t + dt.
     */
    static StateType rk4(double t, const StateType& y, double dt, const ODE& f) {
        StateType k1 = f(t, y);
        StateType k2 = f(t + dt * 0.5, y + k1 * dt * 0.5);
        StateType k3 = f(t + dt * 0.5, y + k2 * dt * 0.5);
        StateType k4 = f(t + dt, y + k3 * dt);

        return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
};

} // namespace engine
} // namespace sim

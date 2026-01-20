"""
Numerical Integrator. Currently implements Runge-Kutta 4 (RK4).

数值积分器。目前实现四阶龙格-库塔法 (RK4)。
"""
class Integrator:
    @staticmethod
    def rk4_step(deriv_func, y, dt):
        """
        Standard RK4 solver.
        标准 RK4 求解器。
        """
        k1 = deriv_func(y)
        k2 = deriv_func(y + (dt * 0.5) * k1)
        k3 = deriv_func(y + (dt * 0.5) * k2)
        k4 = deriv_func(y + dt * k3)
        return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
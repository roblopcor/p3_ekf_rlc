import numpy as np

def velocity_motion_model():
    def state_transition_matrix_A():
        A = np.eye(3)
        return A

    def control_input_matrix_B(mu, delta_t):
        theta = mu[2]
        B = np.array([
            [np.cos(theta) * delta_t, 0],
            [np.sin(theta) * delta_t, 0],
            [0, delta_t]
        ])
        return B

    return state_transition_matrix_A, control_input_matrix_B


def velocity_motion_model_linearized():
    print("Using velocity_motion_model_linearized (3D EKF)")

    # 1. Función de transición de estado no lineal g(mu, u, delta_t)
    def state_transition_function_g(mu=None, u=None, delta_t=None):
        """
        mu = [x, y, theta]
        u = [v, omega]
        """
        x, y, theta = mu
        v, omega = u

        x_new = x + v * np.cos(theta) * delta_t
        y_new = y + v * np.sin(theta) * delta_t
        theta_new = theta + omega * delta_t

        return np.array([x_new, y_new, theta_new])

    # 2. Jacobiano de g respecto al estado mu (G)
    def jacobian_of_g_wrt_state_G(mu=None, u=None, delta_t=None):
        x, y, theta = mu
        v, omega = u

        G = np.array([
            [1, 0, -v * np.sin(theta) * delta_t],
            [0, 1,  v * np.cos(theta) * delta_t],
            [0, 0, 1]
        ])

        return G

    # 3. Jacobiano de g respecto al control u (V)
    def jacobian_of_g_wrt_control_V(mu=None, u=None, delta_t=None):
        theta = mu[2]

        V = np.array([
            [np.cos(theta) * delta_t, 0],
            [np.sin(theta) * delta_t, 0],
            [0, delta_t]
        ])

        return V

    return state_transition_function_g, jacobian_of_g_wrt_state_G, jacobian_of_g_wrt_control_V


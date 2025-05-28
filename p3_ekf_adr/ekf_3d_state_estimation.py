import rclpy
import numpy as np

from .motion_models.velocity_motion_models import velocity_motion_model_linearized
from .observation_models.odometry_observation_models import odometry_observation_model_linearized

from .filters.ekf import ExtendedKalmanFilter
from .kf_node import KalmanFilterNode as ExtendedKalmanFilterNode

def main(args=None):
    # Estado inicial: [x, y, θ]
    mu0 = np.zeros(3)
    Sigma0 = np.eye(3)

    # ================================
    # Selección de configuración de ruido
    # Opciones: "base", "alta_observacion", "alto_modelo"
    noise_case = "alto_modelo"
    # ================================

    if noise_case == "base":
        print("Usando configuración: caso base")
        proc_noise_std = [0.002, 0.002, 0.001]  # Ruido bajo en el modelo
        obs_noise_std = [1.02, 1.02, 100.01]    # Ruido moderado en observación

    elif noise_case == "alta_observacion":
        print("Usando configuración: alta incertidumbre en la observación")
        proc_noise_std = [0.002, 0.002, 0.001]  # Ruido bajo en el modelo
        obs_noise_std = [10.0, 10.0, 1000.0]    # Mucho ruido en las observaciones

    elif noise_case == "alto_modelo":
        print("Usando configuración: alta incertidumbre en el modelo de movimiento")
        proc_noise_std = [0.2, 0.2, 0.1]        # Mucho ruido en el modelo
        obs_noise_std = [1.02, 1.02, 100.01]    # Ruido moderado en observación

    else:
        raise ValueError(f"Configuración de ruido no reconocida: '{noise_case}'")

    # Inicializar el filtro de Kalman extendido con los modelos adecuados
    ekf = ExtendedKalmanFilter(
        mu0,
        Sigma0,
        velocity_motion_model_linearized,
        odometry_observation_model_linearized,
        proc_noise_std=proc_noise_std,
        obs_noise_std=obs_noise_std
    )

    # Inicializar ROS2 y ejecutar el nodo del filtro
    rclpy.init(args=args)
    kalman_filter_node = ExtendedKalmanFilterNode(ekf)
    rclpy.spin(kalman_filter_node)

    # Finalizar ejecución del nodo
    kalman_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

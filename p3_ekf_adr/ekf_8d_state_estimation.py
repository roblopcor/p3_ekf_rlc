import rclpy
import numpy as np

# Importación de los modelos 8D
from .motion_models.acceleration_motion_models import acceleration_motion_model_linearized_2
from .observation_models.odometry_imu_observation_models import odometry_imu_observation_model_with_acceleration_motion_model_linearized_2

from .filters.ekf import ExtendedKalmanFilter
from .kf_node import KalmanFilterFusionNode as ExtendedKalmanFilterFusionNode

def main(args=None):
    # Estado inicial: [x, y, θ, v_x, v_y, ω, a_x, a_y]
    mu0 = np.zeros(8)
    Sigma0 = np.eye(8)

    # ================================
    # Selección de configuración de ruido
    # Cambia este valor para probar cada caso:
    noise_case = "alto_modelo"  # opciones: "base", "alta_observacion", "alto_modelo"
    # ================================

    if noise_case == "base":
        print("Usando configuración: caso base")
        proc_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1]  # Ruido moderado en el modelo
        obs_noise_std  = [100.0, 100.0, 1000.0,
                          6.85e-06,
                          1.10e-06,
                          0.00153,
                          0.00153]  # Ruido moderado en las observaciones

    elif noise_case == "alta_observacion":
        print("Usando configuración: alta incertidumbre en la observación")
        proc_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1]  # Ruido moderado en el modelo
        obs_noise_std  = [1000.0, 1000.0, 10000.0,
                          6.85e-05,
                          1.10e-05,
                          0.15,
                          0.15]  # Ruido alto en las observaciones

    elif noise_case == "alto_modelo":
        print("Usando configuración: alta incertidumbre en el modelo de movimiento")
        proc_noise_std = [1.0, 1.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0]  # Ruido alto en el modelo
        obs_noise_std  = [100.0, 100.0, 1000.0,
                          6.85e-06,
                          1.10e-06,
                          0.00153,
                          0.00153]  # Ruido moderado en las observaciones

    else:
        raise ValueError(f"Configuración de ruido no reconocida: {noise_case}")

    # Inicialización del EKF con modelos 8D
    ekf = ExtendedKalmanFilter(
        mu0,
        Sigma0,
        acceleration_motion_model_linearized_2,
        odometry_imu_observation_model_with_acceleration_motion_model_linearized_2,
        proc_noise_std=proc_noise_std,
        obs_noise_std=obs_noise_std
    )

    # Inicializar ROS2 y ejecutar el nodo
    rclpy.init(args=args)
    kalman_filter_node = ExtendedKalmanFilterFusionNode(ekf)
    rclpy.spin(kalman_filter_node)

    # Finalizar ejecución del nodo
    kalman_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

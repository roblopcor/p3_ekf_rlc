import numpy as np
import time
from p3_ekf_adr.motion_models.velocity_motion_models import velocity_motion_model_linearized
from p3_ekf_adr.observation_models.odometry_observation_models import odometry_observation_model_linearized

class ExtendedKalmanFilter:

    def __init__(self, initial_state, initial_covariance, motion_model, observation_model, **kwargs):
        # Procesar argumentos opcionales de ruido, con valores por defecto según tamaño del estado
        proc_noise_std = kwargs.get('proc_noise_std', len(initial_state) * [0.01])
        obs_noise_std = kwargs.get('obs_noise_std', len(initial_state) * [0.01])

        # Estado inicial y matriz de covarianza inicial (incertidumbre)
        self.mu = initial_state
        self.Sigma = initial_covariance

        # Modelos: función de transición de estado, Jacobianos G y V
        self.g, self.G, self.V = motion_model()

        # Desviaciones estándar del ruido de proceso y matriz de covarianza R
        self.proc_noise_std = np.array(proc_noise_std)
        self.R = np.diag(self.proc_noise_std ** 2)

        # Modelo de observación y su Jacobiano H
        self.h, self.H = observation_model()

        # Desviaciones estándar del ruido de observación y matriz de covarianza Q
        self.obs_noise_std = np.array(obs_noise_std)
        self.Q = np.diag(self.obs_noise_std ** 2)

        # Listas para guardar tiempos de ejecución (predicción y actualización)
        self.exec_times_pred = []
        self.exec_times_upd = []

    def predict(self, u, dt):
        start_time = time.time()

        # 1. Calcular nueva estimación de estado usando función de transición g
        self.mu = self.g(self.mu, u, dt)

        # 2. Calcular Jacobiano de la función g respecto al estado
        G_t = self.G(self.mu, u, dt)

        # 3. Calcular Jacobiano de la función g respecto a la entrada de control (no usado aquí directamente)
        V = self.V(self.mu, u, dt)

        # 4. Actualizar la matriz de covarianza con la propagación del error y ruido de proceso
        self.Sigma = G_t @ self.Sigma @ G_t.T + self.R

        end_time = time.time()
        execution_time = end_time - start_time
        self.exec_times_pred.append(execution_time)

        return self.mu, self.Sigma

    def update(self, z, dt):
        start_time = time.time()

        # 1. Calcular Jacobiano de la función de observación H_t en el estado actual
        H_t = self.H(self.mu)

        # 2. Calcular la covarianza de la innovación
        S = H_t @ self.Sigma @ H_t.T + self.Q

        # 3. Calcular la ganancia de Kalman K
        K = self.Sigma @ H_t.T @ np.linalg.inv(S)

        # 4. Calcular la innovación (diferencia entre observación real y estimada)
        z_vec = z.reshape(-1)
        h_vec = self.h(self.mu).reshape(-1)
        y = z_vec - h_vec

        # 5. Actualizar la estimación del estado usando la ganancia de Kalman y la innovación
        self.mu = self.mu + K @ y

        # 6. Actualizar la matriz de covarianza de la estimación
        I = np.eye(len(self.mu))
        self.Sigma = (I - K @ H_t) @ self.Sigma

        end_time = time.time()
        self.exec_times_upd.append(end_time - start_time)

        return self.mu, self.Sigma

import numpy as np
from dataclasses import dataclass

@dataclass
class NPAParameters:
    mass: float = 100.0
    added_mass: np.ndarray = np.zeros(6)
    max_thrust: float = 300.0
    inertia: np.ndarray = np.diag([50, 60, 70])
    damping: np.ndarray = np.diag([30, 30, 30, 10, 10, 5])

class NPADynamics:
    def __init__(self, params: NPAParameters):
        self.params = params
        self.debug = False
        
    def compute_6dof(self, state: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """Вычисление производных состояния для 6 степеней свободы
    
        Args:
            state: Вектор состояния [u, v, w, p, q, r, x, y, z, phi, theta, psi]
            tau: Вектор управляющих сил/моментов [X, Y, Z, K, M, N]
    
        Returns:
            Вектор производных состояния [du, dv, dw, dp, dq, dr, dx, dy, dz, dphi, dtheta, dpsi]
        """
        #Проверка входных данных
        if len(state) != 12:
            raise ValueError(
                f"Некорректная размерность state. Ожидается 12 элементов, получено {len(state)}. "
                f"Порядок: [u, v, w, p, q, r, x, y, z, phi, theta, psi]"
            )
        if len(tau) != 6:
            raise ValueError(
                f"Некорректная размерность tau. Ожидается 6 элементов, получено {len(tau)}. "
                f"Порядок: [X, Y, Z, K, M, N]"
            )

        nu = state[:6]  # Линейные и угловые скорости
        eta = state[6:]  # Позиция и углы Эйлера
    
        # Вычисление матриц
        M = self._compute_mass_matrix()
        C = self._compute_coriolis_matrix(nu)
        D = self._compute_damping_matrix(nu)
    
        # Диагностический вывод (только при включенном debug)
        if getattr(self, 'debug', False):
            print("\n=== Диагностика compute_6dof() ===")
            print(f"Позиция: [x={state[6]:.2f}, y={state[7]:.2f}, z={state[8]:.2f}] м")
            print(f"Скорости: [u={nu[0]:.2f}, v={nu[1]:.2f}, w={nu[2]:.2f}] м/с")
            print(f"Угл. скорости: [p={nu[3]:.2f}, q={nu[4]:.2f}, r={nu[5]:.2f}] рад/с")
            print("\nУправляющие воздействия:")
            print(f"Силы: X={tau[0]:.2f}, Y={tau[1]:.2f}, Z={tau[2]:.2f} Н")
            print(f"Моменты: K={tau[3]:.2f}, M={tau[4]:.2f}, N={tau[5]:.2f} Н·м")
            print("\nМатрица массы M:\n", np.round(M, 3))
            print("\nКориолисовы силы C(v)v:\n", np.round(C @ nu, 3))
            print("\nДемпфирующие силы D(v)v:\n", np.round(D @ nu, 3))
    
        # Основное уравнение динамики
        try:
            dnu_dt = np.linalg.solve(M, tau - C @ nu - D @ nu)
        except np.linalg.LinAlgError as e:
            raise RuntimeError("Ошибка решения системы уравнений. Проверьте параметры массы.") from e
    
        # Кинематические уравнения
        deta_dt = self._compute_kinematics(eta, nu)
    
        # Проверка результатов
        if np.any(np.isnan(dnu_dt)) or np.any(np.isnan(deta_dt)):
            raise RuntimeError("Обнаружены NaN в производных состояния. Проверьте входные параметры.")
    
        return np.concatenate([dnu_dt, deta_dt])
    
    def _compute_mass_matrix(self) -> np.ndarray:
        #Матрица инерции с учётом добавочной массы
        M = np.zeros((6,6))
        M[:3,:3] = np.diag([
            self.params.mass + self.params.added_mass[0],
            self.params.mass + self.params.added_mass[1],
            self.params.mass + self.params.added_mass[2]
        ])
        M[3:,3:] = self.params.inertia
        return M
        
    def _compute_coriolis_matrix(self, nu: np.ndarray) -> np.ndarray:
        """Вычисление матрицы кориолисовых сил с учетом добавочных масс
    
        Args:
            nu: Вектор скоростей [u, v, w, p, q, r]
        
        Returns:
            Матрица C(nu) размерности 6x6
        """
        m = self.params.mass
        I = self.params.inertia
    
        # Учет добавочных масс (если они заданы в параметрах)
        if hasattr(self.params, 'added_mass'):
            m += self.params.added_mass[0]  # Добавочная масса по оси X
            # Для полной модели нужно учитывать добавочные массы по всем осям
    
        # Линейные скорости
        u, v, w = nu[0], nu[1], nu[2]
        # Угловые скорости
        p, q, r = nu[3], nu[4], nu[5]
    
        # Матрица кориолисовых сил (упрощенная форма Фоссена)
        return np.array([
            [0,     0,     0,     0,     m*w,  -m*v],
            [0,     0,     0,     -m*w,  0,     m*u],
            [0,     0,     0,     m*v,   -m*u,  0],
            [0,     m*w,   -m*v,  0,     I[2,2]*r, -I[1,1]*q],
            [-m*w,  0,     m*u,   -I[2,2]*r, 0,     I[0,0]*p],
            [m*v,   -m*u,  0,     I[1,1]*q,  -I[0,0]*p, 0]
        ])
        
    def _compute_damping_matrix(self, nu: np.ndarray) -> np.ndarray:
        """матрица демпфирования с квадратичной компонентой"""

        # Линейное демпфирование (базовое)
        linear_damp = np.diag([41, 22, 22, 9, 9 , 6])
        
        # Квадратичное демпфирование (зависит от скорости)
        # Коэффициенты скорости для нелинейной части
        velocity_factor = 1 + 0.15*np.tanh(np.abs(nu)/2)

        """np.array([
            1.0 + 0.25*abs(nu[0]),  # Основное движение по X
            1.0 + 0.18*abs(nu[1]),   # По Y
            1.0 + 0.18*abs(nu[2]),   # По Z
            1.0,                    # Вращение вокруг X
            1.0,                    # Вращение вокруг Y
            1.0                     # Вращение вокруг Z
        ])
            """

            # Учет добавочных масс (если заданы)
        if hasattr(self.params, 'added_mass'):
            velocity_factor[:3] *= 1 + self.params.added_mass[:3]/self.params.mass
    
        # Суммарное демпфирование
        return linear_damp * velocity_factor

        
    def _compute_kinematics(self, eta: np.ndarray, nu: np.ndarray) -> np.ndarray:
        """Проверенная версия кинематических уравнений"""
        phi, theta, psi = eta[3:]
    
        # Матрица преобразования для линейных скоростей
        R = np.array([
            [np.cos(psi)*np.cos(theta), -np.sin(psi)*np.cos(phi)+np.cos(psi)*np.sin(theta)*np.sin(phi), np.sin(psi)*np.sin(phi)+np.cos(psi)*np.cos(phi)*np.sin(theta)],
            [np.sin(psi)*np.cos(theta), np.cos(psi)*np.cos(phi)+np.sin(phi)*np.sin(theta)*np.sin(psi), -np.cos(psi)*np.sin(phi)+np.sin(theta)*np.sin(psi)*np.cos(phi)],
            [-np.sin(theta), np.cos(theta)*np.sin(phi), np.cos(theta)*np.cos(phi)]
        ])
    
        # Матрица преобразования для угловых скоростей
        T = np.array([
            [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
        ])
    
        linear_vel = R @ nu[:3]
        angular_vel = T @ nu[3:]
    
        return np.concatenate([linear_vel, angular_vel])


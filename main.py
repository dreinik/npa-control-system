from cProfile import label
import numpy as np
from core.dynamics import NPAParameters, NPADynamics
from core.Controller.pid import PIDController
import matplotlib.pyplot as plt
import csv
import os

class NPASimulation:
    def __init__(self):
        self.params = NPAParameters(
            mass=100.0,
            max_thrust=300.0,
            inertia=np.diag([50, 60, 70]),
            damping=np.diag([30, 30, 30, 10, 10, 5])
        )
        self.model = NPADynamics(self.params)
        self.model.debug = True
        self.pid = self._init_pid_controller()
        self.debug = True  # Добавили флаг отладки
    
    def _init_pid_controller(self):
        """Инициализация PID-регулятора с новыми параметрами"""
        return PIDController(
            Kp=22.0,  # Пропорциональный
            Ki=0.18,   # Интегральный (уменьшен)
            Kd=24.0,  # Дифференциальный (увеличен)
            setpoint=0,
            output_limits=(-self.params.max_thrust, self.params.max_thrust)
        )
    
    def _initialize_state(self):
        """Начальные условия системы"""
        state = np.zeros(12)
        state[6] = 5.0  # Начальная позиция X
        return state
    
    def _print_simulation_info(self):
        """Вывод информации о параметрах симуляции"""
        print("=== Параметры симуляции ===")
        print(f"Масса НПА: {self.params.mass} кг")
        print(f"Макс. тяга: {self.params.max_thrust} Н")
        print(f"PID коэффициенты: Kp={self.pid.Kp}, Ki={self.pid.Ki}, Kd={self.pid.Kd}\n")
    
    def run_simulation(self, target_x=30.0, duration=20.0):
        """Основной цикл симуляции с адаптивным управлением"""
        # Инициализация
        state = self._initialize_state()
        time_points = np.linspace(0, duration, int(duration*100))
        results = []

        # Начальные коэффициенты PID
        self.pid.Kp = 18.0  # Стартовые значения для фазы разгона
        self.pid.Ki = 0.5
        self.pid.Kd = 10.0

        for i, t in enumerate(time_points):
            current_pos = state[6]
            error = current_pos - target_x
            distance_to_target = abs(error)
        
            # Адаптивное управление (5 фаз)
            if distance_to_target > 15.0:  # Фаза агрессивного разгона
                self.pid.Kp = 25.0
                self.pid.Kd = 10.0
                self.pid.Ki = 0.5
                max_speed = min(6.8, distance_to_target * 0.35)

            elif distance_to_target > 5.0:  # Фаза стабилизации
                self.pid.Kp = 18.0
                self.pid.Kd = 18.0
                self.pid.Ki = 0.3
                max_speed = 5.5

            elif distance_to_target > 1.0:  # Предварительное торможение
                self.pid.Kp = 9.0
                self.pid.Kd = 28.0
                self.pid.Ki = 0.2
                max_speed = 2.2

            elif distance_to_target > 0.3:  # Точный подход
                self.pid.Kp = 5.0
                self.pid.Kd = 40.0
                self.pid.Ki = 2.0
                max_speed = 0.7

            else:  # Финальная коррекция
                self.pid.Kp = 0.8
                self.pid.Kd = 60.0
                self.pid.Ki = 0.05
                max_speed = 0.15

            # Ограничение скорости
            current_speed = state[0]
            if abs(current_speed) > max_speed:
                effective_error = np.sign(error) * max_speed - current_speed
            else:
                effective_error = error

            # Расчет управления
            control_x = self.pid.update(effective_error, dt=time_points[1]-time_points[0])
            control_x = np.clip(control_x, -self.params.max_thrust, self.params.max_thrust)

            # Диагностика
            if self.sim_debug and i % 100 == 0:
                print(f"t={t:.2f}s | Pos={current_pos:.2f}m | Speed={state[0]:.2f}m/s | "
                    f"Control={control_x:.2f}N | PID=[{self.pid.Kp:.1f},{self.pid.Ki:.2f},{self.pid.Kd:.1f}]")

            # Применение сил
            tau = np.array([control_x, 0, 0, 0, 0, 0])
            state_derivative = self.model.compute_6dof(state, tau)
            state += state_derivative * (time_points[1]-time_points[0])

            # Сохранение результатов
            results.append({
                'time': t,
                'position': state[6],
                'velocity': state[0],
                'control': control_x,
                'Kp': self.pid.Kp,
                'Kd': self.pid.Kd
            })

        return results
    
    def analyze_results(self, results):
        """Анализ и проверка результатов"""
        final_pos = results[-1]['position']
        max_speed = max(abs(r['velocity']) for r in results)
        max_thrust = max(abs(r['control']) for r in results)
        
        print("\n=== Результаты симуляции ===")
        print(f"Финалная позиция: {final_pos:.2f} м (цель: 30 м)")
        print(f"Макс. скорость: {max_speed:.2f} м/с (лимит: 7 м/с)")
        print(f"Макс. тяга: {max_thrust:.2f} Н (лимит: 300 Н)")
        
        if 29.5 <= final_pos <= 30.5:
            print("Успех: Цель достигнута с высокой точностью!")
        elif 29.0 <= final_pos <= 31.0:
            print("Предупреждение: Цель достигнута с допустимым отклонением")
        else:
            print("Ошибка: Значительное отклонение от цели!")

        # Анализ последних 10% траектории
        last_part = results[int(len(results)*0.9):]
        settling_pos = np.mean([r['position'] for r in last_part])
        settling_error = settling_pos - 30.0
    
        print("\n=== Детальный анализ ===")
        print(f"Финальная позиция: {final_pos:.2f} м")
        print(f"Установившаяся позиция: {settling_pos:.2f} м")
        print(f"Установившаяся ошибка: {settling_error:.2f} м")
    
        if abs(settling_error) <= 0.3:
            print("Статус: Высокая точность (ошибка ≤ 0.3 м)")
        elif abs(settling_error) <= 0.5:
            print("Статус: Приемлемая точность (ошибка ≤ 0.5 м)")
        else:
            print("Статус: Требуется настройка")

         # Анализ последних 5% траектории
        final_part = results[int(len(results)*0.95):]
        settling_pos = np.mean([r['position'] for r in final_part])
    
        print("\n=== Точный анализ ===")
        print(f"Средняя позиция в финальной фазе: {settling_pos:.3f} м")
        print(f"Отклонение: {abs(settling_pos-30):.3f} м")
        print(f"Макс. перерегулирование: {max(0, max(r['position'] for r in results)-30):.3f} м")
    
        if abs(settling_pos-30) <= 0.3:
            print("РЕЗУЛЬТАТ: Высшая точность достигнута!")
        elif abs(settling_pos-30) <= 0.5:
            print("РЕЗУЛЬТАТ: Точность в допустимых пределах")
        else:
            print("РЕЗУЛЬТАТ: Требуется дополнительная настройка")
    
        # Дополнительная диагностика
        overshoot = max(0, final_pos - 30)
        print(f"Перерегулирование: {overshoot:.2f} м")
        assert max_thrust <= 300, "Превышена максимальная тяга!"

    def export_results(self, results):
        """Экспорт результатов симуляции в CSV файл"""
        try:
            # 1. Подготовка пути
            unity_project_path = r"D:\диплом\NPA_Similation\unity_project"
            csv_path = os.path.join(unity_project_path,"NPA_Similation", "Assets", "StreamingAssets", "trajectory.csv")
        
            # Создаем папки, если их нет
            os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        
            # 2. Запись данных
            with open(csv_path, 'w', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)
            
                # Заголовок
                writer.writerow(['time', 'pos_x', 'pos_y', 'pos_z'])
            
                # 3. Обработка данных с проверкой
                for r in results:
                    # Вариант A: Если position - это массив/список
                    if isinstance(r['position'], (list, np.ndarray, tuple)):
                        writer.writerow([
                            r['time'],
                            r['position'][0],  # x
                            r['position'][1],  # y
                            r['position'][2]   # z
                        ])
                
                    # Вариант B: Если position - это словарь
                    elif isinstance(r['position'], dict):
                        writer.writerow([
                            r['time'],
                            r['position']['x'],
                            r['position']['y'],
                            r['position']['z']
                        ])
                
                    # Вариант C: Если position - это число (скаляр)
                    else:
                        # Преобразуем число в координаты (пример)
                        pos = float(r['position'])
                        writer.writerow([
                            r['time'],
                            pos,       # x = position
                            0,          # y = 0
                            0           # z = 0
                        ])
                    
            print(f"Данные успешно экспортированы в {csv_path}")
            return True
        
        except Exception as e:
            print(f"Ошибка экспорта: {str(e)}")
            return False

        
    def visualize(self, results):
        """Визуализация результатов"""
        plt.figure(figsize=(12, 10))
        
        # График позиции
        plt.subplot(3, 1, 1)
        plt.plot([r['time'] for r in results], [r['position'] for r in results], 'b-', linewidth=2, label='цель')
        plt.axhline(y=30, color='g', linestyle='--', linewidth=1)
        plt.axhspan(29.9, 30.1, facecolor='#90EE90', alpha=0.2, label='Высокая точность ±0.1м')
        plt.axhspan(29.7, 30.3, facecolor='#FFCCCB', alpha=0.15, label='Допуск ±0.3м')
        plt.title('Точное позиционирование НПА', pad=15)
        plt.ylabel('Позиция (м)')
        plt.legend(loc='upper right')
        plt.grid(True, linestyle=':', alpha=0.7)

        
        # График скорости
        plt.subplot(3, 1, 2)
        plt.plot([r['time'] for r in results], [r['velocity'] for r in results], 'g-', label='Скорость')
        plt.axhline(y=7, color='m', linestyle=':', label='Макс. скорость')
        plt.axhline(y=-7, color='m', linestyle='--')
        plt.ylabel('Скорость (м/с)', fontsize=12)
        plt.grid(True, linestyle=':', alpha=0.7)
        plt.title(f'Макс. скорость: {max(abs(r["velocity"]) for r in results):.2f} м/с')
        plt.ylim(-8, 8)
        plt.legend()
        
        # График управления
        plt.subplot(3, 1, 3)
        plt.plot([r['time'] for r in results], [r['control'] for r in results], 'k-', label='Управление')
        plt.axhline(y=300, color='b', linestyle='-.', label='Макс. тяга')
        plt.xlabel('Время (с)', fontsize=12)
        plt.ylabel('Тяга (Н)', fontsize=12)
        plt.title(f'Пиковая тяга: {max(abs(r["control"]) for r in results):.0f} Н')
        plt.grid(True, linestyle=':', alpha=0.7)
        plt.ylim(-350, 350)
        plt.legend()

        # График коэффициентов PID
        plt.figure(figsize=(12, 4))
        plt.plot([r['time'] for r in results], [r['Kp'] for r in results], label='Kp')
        plt.plot([r['time'] for r in results], [r['Kd'] for r in results], label='Kd')
        plt.xlabel('Время (с)')
        plt.ylabel('Коэффициенты PID')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    simulation = NPASimulation()
    simulation.model.debug = False  # Отключаем отладку динамики
    simulation.sim_debug = False   # Отключаем отладку симуляции
    results = simulation.run_simulation()
    simulation.analyze_results(results)
    simulation.visualize(results)
    simulation.export_results(results)


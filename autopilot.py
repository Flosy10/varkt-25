import krpc
import time
import math
import matplotlib.pyplot as plt
import csv
import os
from datetime import datetime


TARGET_ALTITUDE = 150000
TURN_START_ALT = 1000
TURN_END_ALT = 45000

conn = None
vessel = None
ap = None


def setup_staging():
    global conn, vessel, ap
    print("Инициализация автопилота...")

    try:
        conn = krpc.connect()
        vessel = conn.space_center.active_vessel
        ap = vessel.auto_pilot
    except Exception as e:
        print(f"Ошибка подключения к KSP: {e}")
        return False

    vessel.control.throttle = 1.0
    vessel.control.sas = False
    vessel.control.rcs = False
    ap.reference_frame = vessel.surface_reference_frame
    ap.target_pitch_and_heading(90, 90)
    ap.engage()
    time.sleep(1)
    return True


def manage_max_q():
    if vessel is None:
        return
    alt = vessel.flight().mean_altitude
    if 5000 < alt < 15000:
        dynamic_pressure = vessel.flight().dynamic_pressure
        if dynamic_pressure > 20000:
            vessel.control.throttle = 0.85
            return
    vessel.control.throttle = 1.0


class RocketStager:
    def __init__(self, vessel):
        self.vessel = vessel
        self.current_stage = 1
        self.stage_separated = [False, False, False]
        self.stage_names = ["Твердотопливные ускорители", "Вторая ступень", "Третья ступень"]

    def get_current_stage_resources(self):
        try:
            all_resources = self.vessel.resources
            current_stage = self.vessel.control.current_stage

            liquid_fuel = all_resources.amount('LiquidFuel')
            oxidizer = all_resources.amount('Oxidizer')
            solid_fuel = all_resources.amount('SolidFuel')

            print(
                f"Стадия {current_stage}: Жидкое топливо: {liquid_fuel:.1f}, Окислитель: {oxidizer:.1f}, Твердое: {solid_fuel:.1f}")

            return solid_fuel, liquid_fuel, oxidizer

        except Exception as e:
            print(f"Ошибка получения ресурсов: {e}")
            return 0, 0, 0

    def check_stage_1_separation(self):
        if self.stage_separated[0]:
            return False

        solid_fuel, liquid_fuel, oxidizer = self.get_current_stage_resources()

        if solid_fuel <= 5.0:
            print(f"СТУПЕНЬ 1: Твердое топливо израсходовано ({solid_fuel:.1f})! Отделение ускорителей.")
            self.separate_current_stage()
            self.stage_separated[0] = True
            self.current_stage = 2
            print("Переход к СТУПЕНИ 2")
            return True

        return False

    def check_stage_2_separation(self):
        if self.stage_separated[1] or self.current_stage != 2:
            return False

        solid_fuel, liquid_fuel, oxidizer = self.get_current_stage_resources()
        available_thrust = self.vessel.available_thrust
        print(f"СТУПЕНЬ 2: Тяга: {available_thrust:.1f}, Топливо: {liquid_fuel:.1f}")

        solid_fuel, liquid_fuel, oxidizer = self.get_current_stage_resources()
        available_thrust = self.vessel.available_thrust
        if (liquid_fuel <= 0.1 and oxidizer <= 0.1) or available_thrust < 1:
            print(f"СТУПЕНЬ 3: Топливо полностью израсходовано!")
            self.vessel.control.throttle = 0.0
            self.stage_separated[2] = True
            return True

        return False

    def separate_current_stage(self):
        print(f"Активация разделения ступени {self.vessel.control.current_stage}")
        self.vessel.control.activate_next_stage()
        time.sleep(3)

        new_stage = self.vessel.control.current_stage
        print(f"Новая текущая ступень: {new_stage}")

    def manage_all_stages(self):
        if self.current_stage == 1:
            return self.check_stage_1_separation()
        elif self.current_stage == 2:
            return self.check_stage_2_separation()
        elif self.current_stage == 3:
            return self.check_stage_3_separation()
        return False


def check_circularization():
    if vessel is None:
        return False
    apoapsis = vessel.orbit.apoapsis_altitude
    periapsis = vessel.orbit.periapsis_altitude

    if apoapsis >= TARGET_ALTITUDE and periapsis < TARGET_ALTITUDE - 10000:
        return True
    return False


def circularize_orbit():
    if vessel is None or ap is None:
        return


    ap.target_pitch_and_heading(0, 90)
    time.sleep(2)

    while vessel.orbit.time_to_apoapsis > 60:
        time.sleep(1)

    while vessel.orbit.time_to_apoapsis > 10:
        time.sleep(0.1)

    vessel.control.throttle = 1.0

    start_time = time.time()
    while vessel.orbit.periapsis_altitude < TARGET_ALTITUDE - 1000:
        if time.time() - start_time > 180:
            break

        apoapsis = vessel.orbit.apoapsis_altitude
        periapsis = vessel.orbit.periapsis_altitude

        if periapsis > TARGET_ALTITUDE - 5000:
            vessel.control.throttle = 0.5
        if periapsis > TARGET_ALTITUDE - 2000:
            vessel.control.throttle = 0.2

        print(f"Апо: {apoapsis / 1000:.1f}км, Пери: {periapsis / 1000:.1f}км")
        time.sleep(0.5)

    vessel.control.throttle = 0.0

    apoapsis = vessel.orbit.apoapsis_altitude
    periapsis = vessel.orbit.periapsis_altitude
    print(f"Итоговая орбита: Апоцентр {apoapsis / 1000:.1f}км, Перицентр {periapsis / 1000:.1f}км")


class DataLogger:
    def __init__(self):
        self.start_time = time.time()
        self.data_dir = "flight_data"
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.data_file = os.path.join(self.data_dir, f"flight_data_{timestamp}.csv")

        with open(self.data_file, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'velocity', 'mass', 'altitude', 'thrust', 'stage'])

        print(f"Файл для записи данных создан: {self.data_file}")

    def save_data(self, vessel):
        if vessel is None:
            return

        current_time = time.time() - self.start_time
        current_velocity = vessel.flight(vessel.orbit.body.reference_frame).speed
        current_mass = vessel.mass
        current_altitude = vessel.flight().mean_altitude
        current_thrust = vessel.available_thrust
        current_stage = vessel.control.current_stage

        with open(self.data_file, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                f"{current_time:.2f}",
                f"{current_velocity:.2f}",
                f"{current_mass:.2f}",
                f"{current_altitude:.2f}",
                f"{current_thrust:.2f}",
                f"{current_stage}"
            ])


def launch_with_data_logging():
    if vessel is None:
        print("Ошибка: корабль не определен")
        return

    vessel.control.activate_next_stage()


    data_logger = DataLogger()


    while vessel.flight().surface_altitude < 10:
        data_logger.save_data(vessel)
        time.sleep(0.1)

    stager = RocketStager(vessel)

    while vessel.flight().mean_altitude < TURN_START_ALT:
        manage_max_q()
        data_logger.save_data(vessel)
        time.sleep(0.1)



    while vessel.flight().mean_altitude < TURN_END_ALT:
        alt = vessel.flight().mean_altitude
        frac = (alt - TURN_START_ALT) / (TURN_END_ALT - TURN_START_ALT)
        pitch = max(0, 90 * (1 - frac))
        ap.target_pitch_and_heading(pitch, 90)

        manage_max_q()
        stager.manage_all_stages()
        data_logger.save_data(vessel)
        time.sleep(0.1)

    ap.target_pitch_and_heading(0, 90)


    while vessel.orbit.apoapsis_altitude < TARGET_ALTITUDE:
        stage_changed = stager.manage_all_stages()

        data_logger.save_data(vessel)


        if stager.current_stage == 3 and stager.stage_separated[2]:
            print("Топливо полностью израсходовано до достижения орбиты")
            break

        time.sleep(0.1)


    vessel.control.throttle = 0.0
    print(f'Достигнута высота апоцентра: {vessel.orbit.apoapsis_altitude / 1000:.1f} км')

    if check_circularization():
        circularize_orbit()
    else:
        apoapsis = vessel.orbit.apoapsis_altitude
        periapsis = vessel.orbit.periapsis_altitude
        print(f"Орбита: Апоцентр {apoapsis / 1000:.1f}км, Перицентр {periapsis / 1000:.1f}км")


    data_logger.save_data(vessel)
    print(f"Данные полета сохранены в файл: {data_logger.data_file}")


    return data_logger.data_file



def plot_from_data_file(data_file_path):
    print(f"Строим графики из файла: {os.path.basename(data_file_path)}")

    times = []
    velocities = []
    masses = []
    altitudes = []
    thrusts = []

    with open(data_file_path, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        next(reader)  

        for row in reader:
            if len(row) == 6:
                times.append(float(row[0]))
                velocities.append(float(row[1]))
                masses.append(float(row[2]))
                altitudes.append(float(row[3]))
                thrusts.append(float(row[4]))

    if not times:
        print("Нет данных для построения графиков!")
        return

    plots_dir = "flight_plots"
    if not os.path.exists(plots_dir):
        os.makedirs(plots_dir)

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')


    plt.figure(figsize=(12, 8))

    plt.subplot(2, 1, 1)
    plt.plot(times, velocities, 'b-', linewidth=2, label='Общая скорость')
    plt.xlabel('Время (с)')
    plt.ylabel('Скорость (м/с)', color='b')
    plt.tick_params(axis='y', labelcolor='b')
    plt.grid(True, alpha=0.3)
    plt.legend(loc='upper left')

    ax2 = plt.gca().twinx()
    ax2.plot(times, [h / 1000 for h in altitudes], 'r-', linewidth=2, label='Высота')
    ax2.set_ylabel('Высота (км)', color='r')
    ax2.tick_params(axis='y', labelcolor='r')
    ax2.legend(loc='upper right')
    plt.title('Общая скорость и Высота')


    plt.subplot(2, 1, 2)
    plt.plot(times, masses, 'g-', linewidth=2, label='Масса')
    plt.xlabel('Время (с)')
    plt.ylabel('Масса (т)', color='g')
    plt.tick_params(axis='y', labelcolor='g')
    plt.grid(True, alpha=0.3)
    plt.legend(loc='upper left')

    ax4 = plt.gca().twinx()
    ax4.plot(times, [t / 1000 for t in thrusts], 'orange', linewidth=2, label='Тяга')
    ax4.set_ylabel('Тяга (кН)', color='orange')
    ax4.tick_params(axis='y', labelcolor='orange')
    ax4.legend(loc='upper right')
    plt.title('Масса и Тяга')

    plt.tight_layout()

    combined_filename = os.path.join(plots_dir, f"combined_plot_{timestamp}.png")
    plt.savefig(combined_filename, dpi=300, bbox_inches='tight')
    print(f"Комбинированный график сохранен: {combined_filename}")


    plot_individual_graphs(times, velocities, masses, altitudes, thrusts, plots_dir, timestamp)

    plt.show()


def plot_individual_graphs(times, velocities, masses, altitudes, thrusts, plots_dir, timestamp):
    plt.figure(figsize=(10, 6))
    plt.plot(times, velocities, 'b-', linewidth=2)
    plt.xlabel('Время (с)')
    plt.ylabel('Общая скорость (м/с)')
    plt.title('Общая скорость ракеты')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, f"velocity_{timestamp}.png"), dpi=300, bbox_inches='tight')
    plt.close()

    plt.figure(figsize=(10, 6))
    plt.plot(times, [h / 1000 for h in altitudes], 'r-', linewidth=2)
    plt.xlabel('Время (с)')
    plt.ylabel('Высота (км)')
    plt.title('Высота полета')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, f"altitude_{timestamp}.png"), dpi=300, bbox_inches='tight')
    plt.close()

    plt.figure(figsize=(10, 6))
    plt.plot(times, masses, 'g-', linewidth=2)
    plt.xlabel('Время (с)')
    plt.ylabel('Масса (кг)')
    plt.title('Масса ракеты')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, f"mass_{timestamp}.png"), dpi=300, bbox_inches='tight')
    plt.close()

    plt.figure(figsize=(10, 6))
    plt.plot(times, [t / 1000 for t in thrusts], 'orange', linewidth=2)
    plt.xlabel('Время (с)')
    plt.ylabel('Тяга (кН)')
    plt.title('Тяга двигателей')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(plots_dir, f"thrust_{timestamp}.png"), dpi=300, bbox_inches='tight')
    plt.close()

    print("Отдельные графики сохранены")


if __name__ == '__main__':
    try:


        if not setup_staging():
            print("Не удалось инициализировать автопилот")
            exit(1)

        data_file = launch_with_data_logging()

        if ap and hasattr(ap, 'engaged'):
            if ap.engaged:
                ap.disengage()
        elif ap:
            ap.disengage()

        if vessel:
            vessel.control.sas = True



        plot_from_data_file(data_file)

    except Exception as e:
        print(f"Ошибка во время выполнения: {e}")
        try:
            if vessel:
                vessel.control.throttle = 0.0
            if ap:
                ap.disengage()
        except:

            pass

import math
import matplotlib.pyplot as plt

class WaterRocket:
    def __init__(self, vessel_capacity, water_fill, dry_mass, air_pressure, drag_coefficient, rocket_diameter, nozzle_diameter, time_step=0.01):
        self.vessel_capacity = vessel_capacity
        self.water_fill = water_fill
        self.dry_mass = dry_mass
        self.air_pressure = air_pressure
        self.drag_coefficient = drag_coefficient
        self.rocket_diameter = rocket_diameter
        self.nozzle_diameter = nozzle_diameter
        self.time_step = time_step
        self.air_fill = self.calculate_air_fill()
        self.initial_pressure_volume_constant = self.calculate_initial_pressure_volume_constant()
        self.calculate_nozzle_area()
        self.mass = self.calculate_mass()
        self.weight = self.calculate_weight()
        self.thrust = self.calculate_thrust()
        self.velocity = 0
        self.drag_force = self.calculate_drag_force()
        self.net_force = self.calculate_net_force()
        self.acceleration = self.calculate_acceleration()

    def __str__(self):
        return (f"Vessel capacity: {self.vessel_capacity} m^3\n"
                f"Water fill: {self.water_fill} m^3\n"
                f"Rocket's dry mass: {self.dry_mass} kg\n"
                f"Air pressure: {self.air_pressure} Pa\n"
                f"Drag coefficient: {self.drag_coefficient}\n"
                f"Rocket's diameter: {self.rocket_diameter} m\n"
                f"Nozzle diameter: {self.nozzle_diameter} m\n"
                f"Nozzle area: {self.nozzle_area:.6f} m^2\n"
                f"Time step: {self.time_step} s\n"
                f"Mass: {self.mass} kg\n"
                f"Weight: {self.weight} N\n"
                f"Thrust: {self.thrust} N\n"
                f"Drag Force: {self.drag_force} N\n"
                f"Net Force: {self.net_force} N\n"
                f"Acceleration: {self.acceleration} m/s^2")

    def calculate_nozzle_area(self):
        radius = self.nozzle_diameter / 2
        self.nozzle_area = math.pi * (radius ** 2)

    def calculate_air_fill(self):
        return self.vessel_capacity - self.water_fill

    def calculate_initial_pressure_volume_constant(self):
        return self.air_pressure * self.air_fill

    def calculate_pressure(self):
        return self.initial_pressure_volume_constant / self.air_fill

    def calculate_water_loss(self, external_pressure=101325, water_density=1000):
        velocity_exit = math.sqrt((2 * (self.air_pressure - external_pressure)) / water_density)
        volume_loss = velocity_exit * self.nozzle_area * self.time_step
        return volume_loss

    def decrease_water_fill(self, amount):
        if amount < 0:
            raise ValueError("Amount to decrease must be non-negative.")
        if amount > self.water_fill:
            self.water_fill = 0
        self.water_fill -= amount

    def calculate_thrust(self, external_pressure=101325):
        return 2 * (self.air_pressure - external_pressure) * self.nozzle_area

    def calculate_drag_force(self, air_density=1.225):
        cross_sectional_area = math.pi * (self.rocket_diameter / 2) ** 2
        return 0.5 * air_density * self.velocity ** 2 * self.drag_coefficient * cross_sectional_area

    def calculate_net_force(self):
        return self.thrust - self.weight - self.drag_force

    def calculate_acceleration(self):
        return self.net_force / self.mass

    def recalculate(self):
        self.air_fill = self.calculate_air_fill()
        self.air_pressure = self.calculate_pressure()
        self.mass = self.calculate_mass()
        self.weight = self.calculate_weight()
        self.thrust = self.calculate_thrust()
        self.drag_force = self.calculate_drag_force()
        self.net_force = self.calculate_net_force()
        self.acceleration = self.calculate_acceleration()
    
    def updateStep(self):
        water_loss = self.calculate_water_loss()
        self.decrease_water_fill(water_loss)
        self.recalculate()
        self.velocity += self.acceleration * self.time_step

    def getWaterFill(self):
        return self.water_fill

    def getMass(self):
        return self.mass

    def getThrust(self):
        return self.thrust

    def getAcceleration(self):
        return self.acceleration

    def calculate_mass(self, water_density=1000):
        return self.dry_mass + self.water_fill * water_density
    
    def calculate_weight(self, gravity=9.81):
        return self.mass * gravity

def get_input(prompt, default):
    try:
        user_input = input(f"{prompt} (default {default}): ")
        if user_input == "":
            return default
        return float(user_input)
    except ValueError:
        print("Invalid input. Using default value.")
        return default
        
def main():
    vessel_capacity = get_input("Vessel capacity [m^3]", 1e-3)
    water_fill = get_input("Water fill [m^3]", 0.3e-3)
    dry_mass = get_input("Rocket's dry mass [kg]", 0.17)
    air_pressure = get_input("Air pressure [Pa]", 6e5)
    drag_coefficient = get_input("Drag coefficient [-]", 0.45)
    rocket_diameter = get_input("Rocket's diameter [m]", 81e-3)
    nozzle_diameter = get_input("Nozzle diameter [m]", 20e-3)
    time_step = get_input("Time step [s]", 0.01)

    rocket = WaterRocket(vessel_capacity, water_fill, dry_mass, air_pressure, drag_coefficient, rocket_diameter, nozzle_diameter, time_step)
    print(rocket)

    water_levels = []
    time_points = []
    air_fills = []
    pressures = []
    masses = []
    accelerations = []
    thrusts = []
    current_time = 0

    air_fills.append(rocket.calculate_air_fill())
    pressures.append(rocket.calculate_pressure())
    water_levels.append(rocket.getWaterFill())
    masses.append(rocket.getMass())
    accelerations.append(rocket.getAcceleration())
    thrusts.append(rocket.getThrust())
    time_points.append(current_time)
    while rocket.getWaterFill() > 0:
        rocket.updateStep()
        water_levels.append(rocket.getWaterFill())
        time_points.append(current_time)
        air_fills.append(rocket.calculate_air_fill())
        pressures.append(rocket.calculate_pressure())
        masses.append(rocket.getMass())
        accelerations.append(rocket.getAcceleration())
        thrusts.append(rocket.getThrust())
        current_time += time_step

    plt.figure(figsize=(12, 12))

    plt.subplot(6, 1, 1)
    plt.plot(time_points, water_levels, label='Water Fill')
    plt.xlabel('Time (s)')
    plt.ylabel('Water Fill (m^3)')
    plt.title('Water Fill Over Time')
    plt.legend()

    plt.subplot(6, 1, 2)
    plt.plot(time_points, air_fills, label='Air Fill', color='orange')
    plt.xlabel('Time (s)')
    plt.ylabel('Air Fill (m^3)')
    plt.title('Air Fill Over Time')
    plt.legend()

    plt.subplot(6, 1, 3)
    plt.plot(time_points, pressures, label='Pressure', color='green')
    plt.xlabel('Time (s)')
    plt.ylabel('Pressure (Pa)')
    plt.title('Pressure Over Time')
    plt.legend()

    plt.subplot(6, 1, 4)
    plt.plot(time_points, masses, label='Mass', color='red')
    plt.xlabel('Time (s)')
    plt.ylabel('Mass (kg)')
    plt.title('Mass Over Time')
    plt.legend()

    plt.subplot(6, 1, 5)
    plt.plot(time_points, accelerations, label='Acceleration', color='purple')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s^2)')
    plt.title('Acceleration Over Time')
    plt.legend()

    plt.subplot(6, 1, 6)
    plt.plot(time_points, thrusts, label='Thrust', color='brown')
    plt.xlabel('Time (s)')
    plt.ylabel('Thrust (N)')
    plt.title('Thrust Over Time')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

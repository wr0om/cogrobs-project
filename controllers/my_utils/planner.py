from unified_planning.shortcuts import *
from unified_planning.model.metrics import *
from unified_planning.engines import PlanGenerationResultStatus
from unified_planning.io import PDDLWriter
import time
import math
import random
import warnings

warnings.simplefilter("ignore", UserWarning)

class DronePathFinderWithUncertainty:
    def __init__(self, locations, rad_list):
        self.locations = locations
        self.location_type = UserType("Location")
        self.problem = Problem("drone_path_finder_project")
        self.rad_list = rad_list
        self.object2name = {}
        self.costs = {}
        self.mac = {}
        self.plan = None
        self.init_problem()

    def init_problem(self):
        """Initializes the problem with locations and fluents."""
        # Creating predicates
        At = Fluent("At", BoolType(), location=self.location_type)
        Visited = Fluent("Visited", BoolType(), location=self.location_type)

        # Creating objects from locations and adding them to the problem
        location_objects = [Object(loc, self.location_type) for loc in self.locations]
        for loc, loc_obj in zip(self.locations, location_objects):
            self.object2name[loc_obj] = loc

        self.problem.add_objects(location_objects)
        self.problem.add_fluent(At, default_initial_value=False)
        self.problem.add_fluent(Visited, default_initial_value=False)

        # Initial state (starting at the first location)
        self.problem.set_initial_value(At(location_objects[0]), True)
        self.problem.set_initial_value(Visited(location_objects[0]), True)

        # Goal state (all locations visited)
        for loc in location_objects:
            self.problem.add_goal(Visited(loc))

        # Calculate distances and action costs
        for i in range(len(location_objects)):
            for j in range(i+1, len(location_objects)):
                if i != j:
                    cost = self.calc_distance(self.locations[i], self.locations[j])
                    rad = 2*(self.rad_list[j - 1] if j > 0 else 0)
                    self.costs[(location_objects[i], location_objects[j])] = cost + rad
                    self.costs[(location_objects[j], location_objects[i])] = cost + rad
                    print(f"rad: {rad}, cost: {cost}")

        # Create actions for moving between locations
        for (loc1, loc2), cost in self.costs.items():
            move = InstantaneousAction(f"move ({self.object2name[loc1]}) -> ({self.object2name[loc2]})")
            move.add_precondition(At(loc1))
            move.add_effect(At(loc2), True)
            move.add_effect(At(loc1), False)
            move.add_effect(Visited(loc2), True)

            self.problem.add_action(move)
            self.mac[move] = Int(int(cost))

        # Add quality metric to minimize the total cost
        self.problem.add_quality_metric(up.model.metrics.MinimizeActionCosts(self.mac))

    def calc_distance(self, loc1, loc2):
        """Calculates Euclidean distance between two 3D coordinates."""
        x1, y1, z1 = map(float, loc1.split(','))
        x2, y2, z2 = map(float, loc2.split(','))
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

    def solve(self):
        """Solves the problem and returns the optimal plan."""
        with OneshotPlanner(problem_kind=self.problem.kind,
                            optimality_guarantee=PlanGenerationResultStatus.SOLVED_OPTIMALLY) as optimal_planner:
            result = optimal_planner.solve(self.problem)
            self.plan = result.plan
        return self.plan

    def get_total_cost(self):
        """Calculates and returns the total cost of the optimal plan."""
        total_cost = 0
        if self.plan:
            for a in self.plan.actions:
                total_cost += self.mac[a.action]._content.payload
        return total_cost

    def get_plan_coords(self):
            if self.plan:
                actions = self.plan.actions
                plan_coords = []
                for action in actions:
                    input_str = str(action)
                    goal_x, goal_y, goal_z = [float(coord) for coord in input_str.split('->')[1].strip()[1:-1].split(',')]
                    plan_coords.append((goal_x, goal_y, goal_z))
                return plan_coords
            return None


def random_location_generator(num_locations, x_range=(0, 50), y_range=(0, 50), z_range=(0, 50)):
    """Generates a list of random locations as 'x,y,z' strings."""
    return [f"{random.randint(*x_range)},{random.randint(*y_range)},{random.randint(*z_range)}"
            for _ in range(num_locations)]


def calculate_means(all_execution_times):
    # Calculate mean execution time for each number of drones across all runs
    num_runs = len(all_execution_times)
    num_tests = len(all_execution_times[0])

    mean_execution_times = []
    for i in range(num_tests):
        total = sum(all_execution_times[run][i] for run in range(num_runs))
        mean_execution_times.append(total / num_runs)

    return mean_execution_times

def main():
    # Test the drone path finder with random locations
    locations = random_location_generator(10)
    # Measure execution time
    start_time = time.time()

    drone_planner = DronePathFinder(locations)
    plan = drone_planner.solve()

    exec_time = time.time() - start_time

    
    # Print the optimal plan and total cost
    if plan:
        print(f"Execution time: {exec_time:.4f} seconds, Optimal plan:")
        for action in drone_planner.plan.actions:
            cost = drone_planner.mac[action.action]._content.payload
            print(f"{action}, cost: {cost}")
        print(f"Total cost: {drone_planner.get_total_cost()}")
    else:
        print("No plan found!")


def test(n_runs=10):
    max_drones = 20  # The maximum number of drones
    num_drones_list = list(range(2, max_drones + 1, 2))  # List of drone numbers to test
    all_execution_times = []  # To store execution times for each run

    # Loop for n_runs different random tests
    for run in range(n_runs):
        print(f"Run {run + 1}/{n_runs}")
        execution_times = []  # Store the execution times for each run

        # Test different numbers of drones
        for num_drones in num_drones_list:
            locations = random_location_generator(num_drones)  # Randomly generate drone locations
            drone_planner = DronePathFinder(locations)  # Initialize the planner

            # Measure execution time
            start_time = time.time()
            drone_planner.solve()
            exec_time = time.time() - start_time

            print(f"Number of drones: {num_drones}, Execution time: {exec_time:.4f} seconds")
            execution_times.append(exec_time)

        all_execution_times.append(execution_times)

    # Calculate the mean execution times
    mean_execution_times = calculate_means(all_execution_times)


    print("num_drones_list =", num_drones_list)
    print("all_execution_times =", all_execution_times)
    print("mean_execution_times =", mean_execution_times)
    
    # Return the number of drones list, all individual execution times, and mean execution times
    return num_drones_list, all_execution_times, mean_execution_times



if __name__ == '__main__':
    main()

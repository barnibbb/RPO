import os
import struct
import time
import numpy as np
import gurobipy as gp
from gurobipy import GRB


def read_map(file_path):
    irradiance_map = {}

    with open(file_path, 'rb') as f:
        while True:
            data = f.read(3 * 2 + 4)
            if not data or len(data) < 10:
                break
            
            key_x, key_y, key_z, value = struct.unpack('<HHHf', data)
            key = (key_x, key_y, key_z)

            irradiance_map[key] = value
    
    return irradiance_map


def load_all_maps(folder_path):
    position_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.bin')])
    position_count = len(position_files)

    all_keys = set()
    irradiance_data = []

    for file in position_files:
        irr_map = read_map(os.path.join(folder_path, file))
        # print(f"{file}\t{len(irr_map)}")
        irradiance_data.append(irr_map)
        all_keys.update(irr_map.keys())

    all_keys = sorted(all_keys)
    key_to_index = {key: idx for idx, key in enumerate(all_keys)}

    dose_matrix = np.zeros((len(all_keys), position_count), dtype=np.float32)

    for j, irr_map in enumerate(irradiance_data):
        for key, value in irr_map.items():
            i = key_to_index[key]
            dose_matrix[i, j] = value

    return dose_matrix, key_to_index, position_files

def solve_irradiance_lp(dose_matrix, dose_threshold, time_budget):
    n_voxels, n_positions = dose_matrix.shape

    # LP problem initialization
    model = gp.Model("UV_Dose_Optimization")

    # Variables
    t = model.addVars(n_positions, lb=0, name="t")
    z = model.addVars(n_voxels, vtype=GRB.BINARY, name="z")
    # u = model.addVars(n_positions, vtype=GRB.BINARY, name="u")  # Num pos minimization

    # Objective function: Minimize the number of positions used
    # model.setObjective(gp.quicksum(u[j] for j in range(n_positions)), GRB.MINIMIZE)

    # Constraints
    for i in range(n_voxels):
        model.addConstr(gp.quicksum(dose_matrix[i, j] * t[j] for j in range(n_positions)) >= dose_threshold * z[i], name=f"dose_constraint_{i}")

    model.addConstr(gp.quicksum(t[j] for j in range(n_positions)) <= time_budget, name="time_budget")

    # for j in range(n_positions):
    #     model.addConstr(t[j] <= 1000 * u[j], name=f"position_usage_{j}")

    # Objective: max coverage of voxels
    model.setObjective(gp.quicksum(z[i] for i in range(n_voxels)), GRB.MAXIMIZE)

    # Optimize the model
    print("Starting optimization...")
    model.setParam('TimeLimit', 3600)  # Set a time limit of 10 minutes
    
    model.optimize()



    print("Optimal solution found:")
    print(f"Objective value: {model.objVal}")
    # return {v.varName: v.x for v in model.getVars()}
    return {f"t[{j}]": t[j].x for j in range(n_positions)}

    # if model.status == GRB.OPTIMAL:
    #     print("Optimal solution found:")
    #     print(f"Objective value: {model.objVal}")
    #     return {v.varName: v.x for v in model.getVars()}
    # else:
    #     print("No optimal solution found.")
    #     return None


def verify(dose_matrix, radiation_times, dose_threshold, time_budget):
    doses = dose_matrix @ radiation_times

    covered = doses > dose_threshold
    num_covered = np.sum(covered)

    total_time = np.sum(radiation_times)

    under_threshold = np.where(covered & (doses < dose_threshold))[0]

    print(f"Total elements: {len(doses)}")
    print(f"Covered elements: {num_covered}")
    print(f"Percentage: {float(num_covered) / float(len(doses))}")
    print(f"Total radiation time used: {total_time:.2f} seconds (budget: {time_budget})")



if __name__ == "__main__":

    

    folder_path = '/home/barni/work/RPO/experiments/irradiance_cafe'

    dose_matrix, key_index, files = load_all_maps(folder_path)

    print(dose_matrix.shape)

    dose_threshold = 280.0
    time_budget = 10800.0

    start = time.time()

    radiation_times_dict = solve_irradiance_lp(dose_matrix, dose_threshold, time_budget)

    stop = time.time()

    print(f"Time taken: {stop - start:.2f} seconds")

    # for j, time in radiation_times_dict.items():
    #     print(f"{j}: {time:.2f} seconds")

    radiation_times = np.array([radiation_times_dict[f"t[{j}]"] for j in range(dose_matrix.shape[1])])

    np.set_printoptions(suppress=True, precision=2)
    print("Optimal radiation times:", np.array2string(np.round(radiation_times, 2), separator=' ', max_line_width=np.inf))

    # Verify the solution
    verify(dose_matrix, radiation_times, dose_threshold, time_budget)

    

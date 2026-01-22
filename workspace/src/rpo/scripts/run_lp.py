import os
import struct
import numpy as np
import pulp
import time

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


def load_initial_map(initial_map_path, key_to_index, t_initial, dose_threshold):
    irr_map = read_map(initial_map_path)

    n_voxels = len(key_to_index)
    initial_dose = np.zeros(n_voxels, dtype=np.float32)

    additional_elements = 0
    completed_elements = 0

    for key, value in irr_map.items():
        if key in key_to_index:
            i = key_to_index[key]
            initial_dose[i] = value * t_initial
            if value * t_initial >= dose_threshold:
                completed_elements += 1
        else:
            if value * t_initial >= dose_threshold:
                additional_elements += 1

    print(f"Additional elements: {additional_elements}\n")
    print(f"Completed elements: {completed_elements}\n")

    return initial_dose

def filter_for_lp(dose_matrix, initial_dose, dose_threshold):
    remaining_mask = initial_dose < dose_threshold

    filtered_matrix = dose_matrix[remaining_mask, :]

    remaining_indices = np.where(remaining_mask)[0]
    satisfied_indices = np.where(~remaining_mask)[0]

    return filtered_matrix, remaining_indices, satisfied_indices




def solve_irradiance_lp(dose_matrix, effective_threshold, time_budget):
    n_voxels, n_positions = dose_matrix.shape

    # LP problem initialization
    prob = pulp.LpProblem("UV_Dose_Optimization", pulp.LpMaximize)

    # Variables
    t = [pulp.LpVariable(f"t_{j}", lowBound=0) for j in range(n_positions)]
    z = [pulp.LpVariable(f"z_{i}", cat='Binary') for i in range(n_voxels)]

    # Constraints
    print("Adding dose constraints ...")
    for i in range(n_voxels):
        dose_expr = pulp.lpSum(dose_matrix[i][j] * t[j] for j in range(n_positions))
        prob += (dose_expr >= effective_threshold * z[i])

    # Total radiation time limit
    print("Adding radiation time limit ...")
    prob += pulp.lpSum(t) <= time_budget

    # Objective
    prob += pulp.lpSum(z)

    # Solve
    print("Run solver")

    solver = pulp.COIN_CMD(timeLimit=300, msg=False)
    # solver = pulp.COIN_CMD()

    prob.solve(solver)

    times = [pulp.value(var) for var in t]
    covered = [i for i in range(n_voxels) if pulp.value(z[i]) > 0.5]

    return times, covered


def solve_irradiance_lp_iterative(dose_matrix, dose_threshold, time_budget, n_final, drop_k=10, zero_thersh=0.1):

    active_positions = list(range(dose_matrix.shape[1]))
    iteration = 0

    while len(active_positions) > n_final:
        start1 = time.time()
    
        iteration += 1
        print(f"\n--- Iteration {iteration}, {len(active_positions)} positions active ---")

        # Subset dose matrix
        sub_matrix = dose_matrix[:, active_positions]

        # Solve LP for current set
        times, covered = solve_single_lp(sub_matrix, dose_threshold, time_budget)

        times = np.array(times)

        verify(sub_matrix, times, dose_threshold, time_budget)

        zero_positions = [i for i, t in enumerate(times) if t < zero_thersh]

        if zero_positions:
            to_drop = zero_positions
        else:
            to_drop = np.argsort(times)[:drop_k].tolist()

        active_positions = [p for i, p in enumerate(active_positions) if i not in to_drop]

        stop1 = time.time()

        print(f"Time taken {iteration}: {stop1 - start1:.2f} seconds.")


    print(f"\nFinal optimization with {len(active_positions)} positions...")

    sub_matrix = dose_matrix[:, active_positions]
    final_times, final_covered = solve_single_lp(sub_matrix, dose_threshold, time_budget)

    full_times = np.zeros(dose_matrix.shape[1])
    for idx, pos in enumerate(active_positions):
        full_times[pos] = final_times[idx]

    return full_times, final_covered


def solve_single_lp(sub_matrix, dose_threshold, time_budget):
    n_voxels, n_positions = sub_matrix.shape

    # LP problem initialization
    prob = pulp.LpProblem("UV_Dose_Optimization", pulp.LpMaximize)

    # Variables
    t = [pulp.LpVariable(f"t_{j}", lowBound=0) for j in range(n_positions)]
    z = [pulp.LpVariable(f"z_{i}", cat='Binary') for i in range(n_voxels)]

    # Constraints
    print("Adding dose constraints ...")
    for i in range(n_voxels):
        # print(f"{i}/{n_voxels} voxel")
        dose_expr = pulp.lpSum(sub_matrix[i][j] * t[j] for j in range(n_positions))
        prob += (dose_expr >= dose_threshold * z[i])

    # Total radiation time limit
    print("Adding radiation time limit ...")
    prob += pulp.lpSum(t) <= time_budget

    # Objective
    print("Objective ...")
    prob += pulp.lpSum(z)

    # solver = pulp.PULP_CBC_CMD(threads=12, timeLimit=300, msg=False)
    solver = pulp.COIN_CMD(timeLimit=300, msg=False)
    # solver = pulp.HiGHS_CMD(threads=12, timeLimit=300, msg=False)
    print("Run solver ...")
    prob.solve(solver)

    times = [pulp.value(var) for var in t]
    covered = [i for i in range(n_voxels) if pulp.value(z[i]) > 0.5]

    return times, covered



def two_stage_lp(dose_matrix, dose_threshold, time_budget, k):
    n_voxels, n_positions = dose_matrix.shape

    times_full, covered_full = solve_single_lp(dose_matrix, dose_threshold, time_budget)

    times_full = np.array(times_full)

    verify(dose_matrix, times_full, dose_threshold, time_budget)

    top_local_indices = np.argsort(-times_full)[:k]
    top_local_indices = list(map(int, top_local_indices))

    sub_matrix = dose_matrix[:, top_local_indices]

    times_k, covered_k = solve_single_lp(sub_matrix, dose_threshold, time_budget)

    verify(sub_matrix, times_k, dose_threshold, time_budget)




def verify(dose_matrix, radiation_times, dose_threshold, time_budget):
    print(f"Dose mx: {len(dose_matrix[0])}\nRadiation times: {len(radiation_times)}")

    doses = dose_matrix @ radiation_times

    covered = doses > dose_threshold
    num_covered = np.sum(covered)

    total_time = np.sum(radiation_times)

    under_threshold = np.where(covered & (doses < dose_threshold))[0]

    print(f"Total elements: {len(doses)}")
    print(f"Covered elements: {num_covered}")
    print(f"Percentage: {float(num_covered) / float(len(doses))}")
    print(f"Total radiation time used: {total_time:.2f} seconds (budget: {time_budget})")



def verify2(filtered_matrix, radiation_times, initial_dose, remaining_indices, satisfied_indices, dose_threshold, time_budget):
    lp_dose = filtered_matrix @ radiation_times

    final_dose_remaining = initial_dose[remaining_indices] + lp_dose

    covered_remaining = final_dose_remaining > dose_threshold

    num_covered_remaining = np.sum(covered_remaining)

    num_satisfied_initial = len(satisfied_indices)

    total_voxels = len(initial_dose)
    total_covered = num_satisfied_initial + num_covered_remaining

    total_time = np.sum(radiation_times)

    print("----- VERIFICATION -----")
    print(f"Total voxels: {total_voxels}")
    print(f"Initially satisfied: {num_satisfied_initial}")
    print(f"Satisfied after LP: {num_covered_remaining}")
    print(f"TOTAL satisfied: {total_covered}")
    print(f"Coverage percentage: {total_covered / total_voxels:.3f}")
    print(f"Total radiation time used: {total_time:.2f} (budget: {time_budget})")



# Sol: 46498 60402


if __name__ == '__main__':

    folder_path = '/home/appuser/data/irradiance_infirmary_2'

    output_file = '/home/appuser/data/infirmary2.sol'

    initial_map_path = '/home/appuser/data/32768_32768_32816.bin'

    dose_threshold = 280.0
    time_budget = 1800.0

    dose_matrix, key_index, files = load_all_maps(folder_path)

    initial_dose = load_initial_map(initial_map_path, key_index, time_budget, dose_threshold)

    filtered_matrix, remaining_indices, satisfied_indices = filter_for_lp(dose_matrix, initial_dose, dose_threshold)

    print(f"Total voxels: {len(initial_dose)}")
    print(f"Already satisfied by initial radiation: {len(satisfied_indices)}")
    print(f"Remaining voxels requiring LP optimization: {len(remaining_indices)}")

    remaining_required_dose = dose_threshold - initial_dose[remaining_indices]
    remaining_required_dose = np.maximum(0, remaining_required_dose)

    # effective_threshold = np.maximum(0, dose_threshold - initial_dose)

    print(dose_matrix.shape)

    start = time.time()

    two_stage_lp(dose_matrix, dose_threshold, time_budget, 16)

    # radiation_times, covered_voxels = solve_irradiance_lp(dose_matrix, dose_threshold, time_budget)

    # radiation_times, covered_voxels = solve_irradiance_lp(filtered_matrix, remaining_required_dose, time_budget)

    # radiation_times, covered_voxels = solve_irradiance_lp_iterative(dose_matrix, dose_threshold, time_budget, 90)

    stop = time.time()

    print(f"Time taken: {stop - start:.2f} seconds.")

    print(f"Optimal radiation times: {radiation_times}")
    print(f"Covered voxels: {len(covered_voxels)}")
    print(f"All elements: {dose_matrix.shape[0]}")
    print(f"Ratio of covered voxels: {len(covered_voxels) / dose_matrix.shape[0]}")
    

    # verify2(filtered_matrix, radiation_times, initial_dose, remaining_indices, satisfied_indices, dose_threshold, time_budget)

    verify(dose_matrix, radiation_times, dose_threshold, time_budget)

    with open(output_file, 'w') as f:
        for time in radiation_times:
            f.write(f"{time:.6f} ")

        f.write("\n")
    

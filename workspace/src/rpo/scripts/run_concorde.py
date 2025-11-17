import os
import subprocess
import sys
import time

from pathlib import Path

import numpy as np

## Converting tsp format to cost matrix
def tsplib2costmatrix(tsp_file):
    with open(tsp_file, "r") as f:
        lines = f.readlines()

    dimension = None
    edge_weight_type = None
    edge_weight_format = None
    coords = []
    matrix_values = []
    reading_coords = False
    reading_matrix = False
    
    for line in lines:
        line = line.strip()
        if line.startswith("DIMENSION"):
            dimension = int(line.split(":")[1].strip())
        elif line.startswith("EDGE_WEIGHT_TYPE"):
            edge_weight_type = line.split(":")[1].strip()
        elif line.startswith("EDGE_WEIGHT_FORMAT"):
            edge_weight_format = line.split(":")[1].strip()
        elif line.startswith("NODE_COORD_SECTION"):
            reading_coords = True
            continue
        elif line.startswith("EDGE_WEIGHT_SECTION"):
            reading_matrix = True
            continue
        elif "EOF" in line:
            break

        if reading_coords:
            _, x, y = line.strip().split()
            coords.append((float(x), float(y)))
        elif reading_matrix:
            matrix_values.extend([float(x) for x in line.split()])

    if edge_weight_type == "EUC_2D":
        coords = np.array(coords)
        diff = coords[:, None, :] - coords[None, :, :]
        cost_matrix = np.sqrt(np.sum(diff**2, axis=-1))
    else:
        cost_matrix = np.zeros((dimension, dimension))
        if edge_weight_format == "FULL_MATRIX":
            mat = np.array(matrix_values).reshape(dimension, dimension)
            cost_matrix = mat
        elif edge_weight_format in ["LOWER_DIAG_ROW", "LOWER_DIAG_COL"]:
            mat = np.zeros((dimension, dimension))
            idx = 0
            for i in range(dimension):
                for j in range(i+1):
                    mat[i,j] = matrix_values[idx]
                    mat[j,i] = matrix_values[idx]
                    idx += 1
            cost_matrix = mat

    return cost_matrix


## Tour cost computation with numpy
def compute_cost(tour, cost_matrix, open_loop=False):
    tour = np.array(tour)
    if not open_loop:
        return cost_matrix[tour, np.roll(tour, -1)].sum()
    else:
        if tour[0] != 0:
            tour = np.concatenate(([0], tour))
        return cost_matrix[tour[:-1], tour[1:]].sum()


## Write solution to file
def write_solution(sol_file, cost, exec_time, tour):
    with open(sol_file, 'w') as f:
        f.write(f"{cost:.6f} {exec_time:.6f} ")
        f.write(" ".join(str(x) for x in tour) + "\n")


## Run Concorde exact solver for TSP
def run_concorde(tsp_file, sol_file):

    # TSPLIB to cost matrix conversion
    cost_matrix = tsplib2costmatrix(tsp_file)
    
    # Files before execution
    work_dir = Path.cwd()
    before = set(work_dir.iterdir())

    # Start timer
    start_time = time.time()

    # Concorde execution
    subprocess.run(["concorde", tsp_file], check=True)

    # End timer
    exec_time = time.time() - start_time

    # Reading tour from output file
    tsp_path = os.path.abspath(tsp_file)
    tsp_name = os.path.splitext(os.path.basename(tsp_path))[0]
    tour_path = os.path.join(os.getcwd(), tsp_name + ".sol")
    tour = []
    with open(tour_path, "r") as f:
        lines = f.readlines()

    for line in lines[1:]:
        numbers = list(map(int, line.split()))
        tour.extend(numbers)

    cost = compute_cost(tour, cost_matrix)       

    # Delete new files
    sol_path = Path(sol_file).resolve()
    after = set(work_dir.iterdir())
    new_files = after - before
    for file in new_files:
        if file.resolve() != sol_path:
            file.unlink()


    # Writing solution to file
    write_solution(sol_file, cost, exec_time, tour)

    # Print solution details
    print(f"Concorde solution cost: {cost:.0f}, time: {exec_time:.6f} seconds.")



if __name__ == "__main__":

    if len(sys.argv) < 3:
        print("Usage: python3 exact.py <tsp_file> <sol_file>")
        sys.exit(1)

    tsp_file = sys.argv[1]
    sol_file = sys.argv[2]

    run_concorde(tsp_file, sol_file)
    
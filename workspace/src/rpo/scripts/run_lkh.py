import os
import subprocess
import sys
import time


# Generating special file format for LKH solver
def generate_par_file(tsp_file):

    # Setting path
    tsp_path = os.path.abspath(tsp_file)
    tsp_dir = os.path.dirname(tsp_path)
    tsp_name = os.path.splitext(os.path.basename(tsp_path))[0]
    tour_path = os.path.join(tsp_dir, tsp_name + ".tour")
    par_path = os.path.join(tsp_dir, tsp_name + ".par")

    # Writing run parameters to file
    with open(par_path, 'w') as f:
        f.write(f"PROBLEM_FILE = {tsp_file}\n")
        f.write(f"OUTPUT_TOUR_FILE = {tour_path}\n")
        f.write(f"RUNS = 1\n")

    return par_path, tour_path


## Write solution to file
def write_solution(sol_file, cost, exec_time, tour):
    with open(sol_file, 'w') as f:
        f.write(f"{cost:.6f} {exec_time:.6f} ")
        f.write(" ".join(str(x) for x in tour) + "\n")


# Get tour from LKH output file
def get_lkh_tour(tour_file, sol_file, exec_time):
    cost = None
    tour = []

    with open(tour_file, "r") as file:
        for line in file:
            line = line.strip()
            if line.startswith("COMMENT") and "Length" in line:
                # Extracting tour cost
                cost = int(line.split("=")[-1].strip())
            elif line == "TOUR_SECTION":
                continue
            elif line == "-1" or line == "EOF":
                break
            elif line.isdigit():
                # Start numbering from zero
                tour.append(int(line) - 1)

    if cost is None:
        raise ValueError("Cost not found in .tour file")  


    # Writing solution to file
    write_solution(sol_file, cost, exec_time, tour)

    return tour, cost



# Run LKH executable
def run_lkh(tsp_file, sol_file):

    # Start timer
    start_time = time.time()

    # Generating special file format for LKH solver
    par_file, tour_file = generate_par_file(tsp_file)

    # Run LKH executable
    subprocess.run(["LKH", par_file])

    # End timer
    exec_time = time.time() - start_time

    # Converting LKH output to .sol format
    tour, cost = get_lkh_tour(tour_file, sol_file, exec_time)

    # Print solution details
    print(f"LKH solution cost: {cost}, time: {exec_time:.6f} seconds.")



if __name__ == "__main__":

    if len(sys.argv) < 3:
        print("Usage: python3 lkh.py <tsp_file> <sol_file>")
        sys.exit(1)

    tsp_file = sys.argv[1]
    sol_file = sys.argv[2]

    run_lkh(tsp_file, sol_file)

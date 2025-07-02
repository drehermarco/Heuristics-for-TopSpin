import csv
import re

input_file = "test.txt"
output_file = "ida_star_results.csv"

results = []

with open(input_file, "r") as f:
    content = f.read()

# Split on the delimiter
runs = content.strip().split("-----------------------------------------")

for run in runs:
    lines = run.strip().splitlines()
    if len(lines) < 5:
        continue  # Ensure there are enough lines to extract data

    initial_state_match = re.search(r"Initial State:\s+(.+?)\s+\|\s+h\s+=\s+(\d+)", lines[0])
    time_match = re.search(r"([0-9.]+)\s+seconds", lines[1])
    nodes_match = re.search(r"Nodes expanded:\s+(\d+)", lines[2])
    length_match = re.search(r"Solution length:\s+(\d+)", lines[3])
    cost_match = re.search(r"Total cost:\s+(\d+)", lines[4])

    if not all([initial_state_match, time_match, nodes_match, length_match, cost_match]):
        continue 

    initial_state = initial_state_match.group(1)
    h_val = int(initial_state_match.group(2))
    time_s = float(time_match.group(1))
    nodes = int(nodes_match.group(1))
    solution_length = int(length_match.group(1))
    cost = int(cost_match.group(1))

    results.append({
        "initial_state": initial_state,
        "h": h_val,
        "time_s": time_s,
        "nodes_expanded": nodes,
        "solution_length": solution_length,
        "total_cost": cost
    })

with open(output_file, "w", newline="") as csvfile:
    fieldnames = ["initial_state", "h", "time_s", "nodes_expanded", "solution_length", "total_cost"]
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    writer.writerows(results)

print(f"Extracted {len(results)} runs to {output_file}")

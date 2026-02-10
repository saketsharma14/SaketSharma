"""
Utilities Module

Input/Output and validation functions:
- Load JSON files
- Save solution.json
- Validate solution structure
"""

import json
import os
from typing import Dict, List, Tuple


def load_inputs() -> Tuple[Dict, Dict, Dict]:
    """
    Load all input files.
    
    Returns:
        (map_data, sensor_data, objectives_data)
    """
    # Load public map
    with open('map2/public_map_2.json', 'r') as f:
        map_data = json.load(f)
    
    # Load sensor data
    with open('map2/sensor_data_2.json', 'r') as f:
        sensor_data = json.load(f)
    
    # Load objectives
    with open('map2/objectives_2.json', 'r') as f:
        objectives_data = json.load(f)
    
    return map_data, sensor_data, objectives_data


def save_solution(solution: Dict[str, List[int]]):
    """
    Save solution to solution.json.
    
    Args:
        solution: Dictionary mapping vehicle_id -> path (list of nodes)
    
    Output format:
        {
            "truck1": [0, 0, 1, 2, ...],
            "truck2": [0, 0, 0, 3, ...],
            "truck3": [...],
            "drone1": [...],
            "drone2": [...]
        }
    """
    with open('solution.json', 'w') as f:
        json.dump(solution, f, indent=2)


def validate_solution(solution: Dict[str, List[int]], 
                      graph, T: int) -> Tuple[bool, List[str]]:
    """
    Validate solution structure and constraints.
    
    Args:
        solution: Vehicle paths
        graph: Graph instance
        T: Total time steps
    
    Returns:
        (is_valid, list_of_errors)
    """
    errors = []
    
    # Expected vehicles
    expected_vehicles = ['truck1', 'truck2', 'truck3', 'drone1', 'drone2']
    
    for vehicle_id in expected_vehicles:
        if vehicle_id not in solution:
            errors.append(f"Missing vehicle: {vehicle_id}")
            continue
        
        path = solution[vehicle_id]
        vehicle_type = 'truck' if 'truck' in vehicle_id else 'drone'
        
        # Check path length
        if len(path) != T:
            errors.append(f"{vehicle_id}: Path length {len(path)} != {T}")
        
        # Check path validity
        for t in range(len(path) - 1):
            current = path[t]
            next_node = path[t + 1]
            
            # Waiting is always valid
            if current == next_node:
                continue
            
            # Check if edge exists
            if not graph.is_valid_edge(current, next_node, vehicle_type):
                road_type = graph.get_road_type(current, next_node)
                if road_type == -1:
                    errors.append(
                        f"{vehicle_id} at t={t}: No road from {current} to {next_node}"
                    )
                elif road_type == 0 and vehicle_type == 'truck':
                    errors.append(
                        f"{vehicle_id} at t={t}: Truck cannot use airspace (road type 0)"
                    )
    
    is_valid = len(errors) == 0
    return is_valid, errors


def print_solution_stats(solution: Dict[str, List[int]], 
                        vehicles: List[Dict]):
    """
    Print detailed statistics about the solution.
    
    Args:
        solution: Vehicle paths
        vehicles: Vehicle state data from planner
    """
    print("\n" + "=" * 60)
    print("SOLUTION STATISTICS")
    print("=" * 60)
    
    for vehicle in vehicles:
        vid = vehicle['id']
        print(f"\n{vid}:")
        print(f"  Path length: {len(solution[vid])}")
        print(f"  Travel cost: {vehicle['total_cost']:.2f}")
        print(f"  Points earned: {vehicle['total_points']:.2f}")
        print(f"  Objectives: {len(vehicle['objectives'])}")
        
        if vehicle['objectives']:
            print(f"  Completed:")
            for obj in vehicle['objectives']:
                print(f"    - Node {obj['node']}: {obj['points']} points")
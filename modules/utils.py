"""
Utility Module

Handles JSON I/O and validation. No business logic.
"""

import json
from pathlib import Path
from typing import Dict, List, Tuple


def load_inputs() -> Tuple[Dict, Dict, Dict]:
    """
    Load all input JSON files.
    
    Returns:
        Tuple of (map_data, sensor_data, objectives_data)
    """
    map_data = _load_json("public_map.json")
    sensor_data = _load_json("sensor_data.json")
    objectives_data = _load_json("objectives.json")
    
    return map_data, sensor_data, objectives_data


def save_solution(solution: Dict[str, List[int]]):
    """
    Save solution to JSON file.
    
    Args:
        solution: Dictionary mapping vehicle_id -> path
    """
    with open("solution.json", 'w') as f:
        json.dump(solution, f, indent=2)


def validate_solution(solution: Dict[str, List[int]], graph, T: int) -> Tuple[bool, List[str]]:
    """
    Validate solution against constraints.
    
    Args:
        solution: Solution dictionary
        graph: Graph instance
        T: Total time steps
        
    Returns:
        Tuple of (is_valid, list_of_errors)
    """
    errors = []
    
    for vehicle_id, path in solution.items():
        vehicle_type = "truck" if "truck" in vehicle_id else "drone"
        
        # Check path length
        if len(path) != T:
            errors.append(f"{vehicle_id}: Path length {len(path)} != {T}")
            continue
        
        # Check valid transitions
        for t in range(len(path) - 1):
            current = path[t]
            next_node = path[t + 1]
            
            # Waiting is valid
            if current == next_node:
                continue
            
            # Check edge exists
            if not graph.is_valid_edge(current, next_node, vehicle_type):
                road_type = graph.get_road_type(current, next_node)
                if road_type == -1:
                    errors.append(f"{vehicle_id}@t={t}: No road {current}→{next_node}")
                elif vehicle_type == "truck" and road_type == 0:
                    errors.append(f"{vehicle_id}@t={t}: Truck can't fly {current}→{next_node}")
        
        # Check valid nodes
        for t, node in enumerate(path):
            if node < 0 or node >= graph.num_nodes:
                errors.append(f"{vehicle_id}@t={t}: Invalid node {node}")
    
    return len(errors) == 0, errors


def _load_json(filename: str) -> Dict:
    """Load JSON file with error handling."""
    filepath = Path(filename)
    if not filepath.exists():
        raise FileNotFoundError(f"File not found: {filename}")
    
    with open(filepath, 'r') as f:
        return json.load(f)
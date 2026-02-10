#!/usr/bin/env python3
"""
Vehicle Routing Optimization - Main Entry Point
Orchestrates the workflow without containing business logic.
"""

import sys
from modules.utils import load_inputs, save_solution, validate_solution
from modules.graph import Graph
from modules.cost import CostCalculator
from modules.planner import Planner


def main():
    """
    Workflow orchestration:
    1. Load inputs
    2. Initialize components
    3. Run planner
    4. Validate and save
    """
    print("=" * 60)
    print("Vehicle Routing Optimization System")
    print("=" * 60)
    
    # Step 1: Load all input data
    print("\n[1/4] Loading inputs...")
    try:
        map_data, sensor_data, objectives_data = load_inputs()
        print(f"  ✓ Loaded {map_data['N']} nodes, T={map_data['T']}")
        print(f"  ✓ Loaded {len(objectives_data['objectives'])} objectives")
    except Exception as e:
        print(f"  ✗ Error loading inputs: {e}")
        sys.exit(1)
    
    # Step 2: Initialize components
    print("\n[2/4] Initializing system...")
    try:
        graph = Graph(map_data['map'])
        cost_calc = CostCalculator(map_data['road_weights'], sensor_data)
        planner = Planner(
            graph=graph,
            cost_calc=cost_calc,
            start_node=objectives_data['start_node'],
            objectives=objectives_data['objectives'],
            late_penalty=objectives_data['late_penalty_per_step'],
            T=map_data['T']
        )
        print(f"  ✓ Graph: {graph.num_nodes} nodes")
        print(f"  ✓ Vehicles: {planner.num_trucks} trucks, {planner.num_drones} drones")
    except Exception as e:
        print(f"  ✗ Error initializing: {e}")
        sys.exit(1)
    
    # Step 3: Run planner
    print("\n[3/4] Computing optimal routes...")
    try:
        solution = planner.solve()
        score = planner.get_total_score()
        completed = planner.get_completed_count()
        print(f"  ✓ Total score: {score:.2f}")
        print(f"  ✓ Completed: {completed}/{len(objectives_data['objectives'])}")
    except Exception as e:
        print(f"  ✗ Error during planning: {e}")
        sys.exit(1)
    
    # Step 4: Validate and save
    print("\n[4/4] Saving solution...")
    try:
        is_valid, errors = validate_solution(solution, graph, map_data['T'])
        
        if not is_valid:
            print("  ⚠ Warning: Solution has validation errors:")
            for error in errors[:3]:
                print(f"    - {error}")
            if len(errors) > 3:
                print(f"    ... and {len(errors) - 3} more errors")
        
        # Save solution.json
        save_solution(solution)
        print(f"  ✓ Solution saved to solution.json")
        
    except Exception as e:
        print(f"  ✗ Error saving solution: {e}")
        sys.exit(1)
    
    # Summary
    print("\n" + "=" * 60)
    print("COMPLETE")
    print("=" * 60)
    print(f"Score: {score:.2f} | Objectives: {completed} | Output: solution.json")
    print("=" * 60)


if __name__ == "__main__":
    main()
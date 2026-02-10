"""
Planner Module

The brain of the system:
- Prioritizes objectives
- Assigns to vehicles
- Runs A* pathfinding
- Computes scores
- Builds full paths
"""

import heapq
from typing import List, Dict, Tuple, Optional


class Planner:
    """
    Main planning algorithm.
    
    Strategy: Greedy assignment with A* pathfinding
    """
    
    def __init__(self, graph, cost_calc, start_node: int, 
                 objectives: List[Dict], late_penalty: float, T: int):
        """
        Initialize planner.
        
        Args:
            graph: Graph instance
            cost_calc: CostCalculator instance
            start_node: Starting node for all vehicles
            objectives: List of objective dicts
            late_penalty: Points lost per time step late
            T: Total time steps
        """
        self.graph = graph
        self.cost_calc = cost_calc
        self.start_node = start_node
        self.objectives = objectives
        self.late_penalty = late_penalty
        self.T = T
        
        # Fleet configuration
        self.num_trucks = 3
        self.num_drones = 2
        
        # Initialize vehicles
        self.vehicles = self._init_vehicles()
    
    def solve(self) -> Dict[str, List[int]]:
        """
        Main solving algorithm.
        
        Returns:
            Solution dict: {vehicle_id: [path]}
        """
        # Sort objectives by priority
        sorted_objs = self._prioritize_objectives()
        
        # Greedy assignment
        for obj in sorted_objs:
            self._assign_objective(obj)
        
        # Build full-length paths
        solution = {}
        for vehicle in self.vehicles:
            # Extend to T steps
            while len(vehicle['path']) < self.T:
                vehicle['path'].append(vehicle['current_node'])
            
            solution[vehicle['id']] = vehicle['path']
        
        return solution
    
    def get_total_score(self) -> float:
        """Calculate total score across all vehicles."""
        total_points = sum(v['total_points'] for v in self.vehicles)
        total_cost = sum(v['total_cost'] for v in self.vehicles)
        return total_points - total_cost
    
    def get_completed_count(self) -> int:
        """Count completed objectives."""
        return sum(len(v['objectives']) for v in self.vehicles)
    
    def _init_vehicles(self) -> List[Dict]:
        """Initialize vehicle fleet."""
        vehicles = []
        
        for i in range(1, self.num_trucks + 1):
            vehicles.append({
                'id': f'truck{i}',
                'type': 'truck',
                'current_node': self.start_node,
                'current_time': 0,
                'path': [self.start_node],
                'total_cost': 0.0,
                'total_points': 0.0,
                'objectives': []
            })
        
        for i in range(1, self.num_drones + 1):
            vehicles.append({
                'id': f'drone{i}',
                'type': 'drone',
                'current_node': self.start_node,
                'current_time': 0,
                'path': [self.start_node],
                'total_cost': 0.0,
                'total_points': 0.0,
                'objectives': []
            })
        
        return vehicles
    
    def _prioritize_objectives(self) -> List[Dict]:
        """
        Sort objectives by priority.
        
        Heuristic: High points / tight deadline = high priority
        """
        def priority_key(obj):
            time_window = obj['deadline'] - obj['release'] + 1
            return -obj['points'] / max(1, time_window)
        
        return sorted(self.objectives, key=priority_key)
    
    def _assign_objective(self, obj: Dict):
        """
        Assign objective to best vehicle (greedy).
        
        Args:
            obj: Objective dictionary
        """
        best_vehicle = None
        best_benefit = -float('inf')
        best_data = None
        
        # Evaluate each vehicle
        for vehicle in self.vehicles:
            if vehicle['current_time'] > obj['deadline']:
                continue
            
            # Find path to objective
            path, cost, arrival = self._find_path(
                vehicle['current_node'],
                vehicle['current_time'],
                obj['node'],
                vehicle['type'],
                obj['deadline']
            )
            
            if path is None:
                continue
            
            # Wait if arriving before release
            if arrival < obj['release']:
                wait = obj['release'] - arrival
                path.extend([obj['node']] * wait)
                arrival = obj['release']
            
            # Calculate points
            points = self._calculate_score(arrival, obj)
            
            # Net benefit
            benefit = points - cost
            
            if benefit > best_benefit and benefit > 0:
                best_benefit = benefit
                best_vehicle = vehicle
                best_data = (path, cost, arrival, points)
        
        # Assign to best vehicle
        if best_vehicle and best_data:
            path, cost, arrival, points = best_data
            
            # Update vehicle path (skip first node, already there)
            for node in path[1:]:
                if node == best_vehicle['current_node']:
                    # Waiting
                    best_vehicle['path'].append(node)
                    best_vehicle['current_time'] += 1
                else:
                    # Moving
                    edge_cost = self.cost_calc.get_cost(
                        self.graph.get_road_type(best_vehicle['current_node'], node),
                        best_vehicle['current_time'],
                        best_vehicle['type']
                    )
                    best_vehicle['path'].append(node)
                    best_vehicle['current_node'] = node
                    best_vehicle['current_time'] += 1
                    best_vehicle['total_cost'] += edge_cost if edge_cost else 0
            
            best_vehicle['total_points'] += points
            best_vehicle['objectives'].append(obj)
    
    def _find_path(self, start_node: int, start_time: int, 
                   target_node: int, vehicle_type: str,
                   deadline: int) -> Tuple[Optional[List[int]], float, int]:
        """
        Find optimal path using A* with time-dependent costs.
        
        Args:
            start_node: Starting node
            start_time: Starting time
            target_node: Target node
            vehicle_type: "truck" or "drone"
            deadline: Latest acceptable arrival
            
        Returns:
            (path, total_cost, arrival_time) or (None, inf, T) if no path
        """
        # Priority queue: (f_score, g_score, node, time, path)
        pq = [(0, 0, start_node, start_time, [start_node])]
        visited = {}
        
        while pq:
            f, g, node, time, path = heapq.heappop(pq)
            
            # Found target
            if node == target_node and time <= deadline:
                return path, g, time
            
            # Exceeded time
            if time >= self.T or time > deadline:
                continue
            
            # Already visited with better cost
            state = (node, time)
            if state in visited and visited[state] <= g:
                continue
            visited[state] = g
            
            # Option 1: Wait (no cost)
            heapq.heappush(pq, (g, g, node, time + 1, path + [node]))
            
            # Option 2: Move
            for next_node, road_type in self.graph.get_neighbors(node, vehicle_type):
                edge_cost = self.cost_calc.get_cost(road_type, time, vehicle_type)
                
                if edge_cost is None:
                    continue
                
                new_g = g + edge_cost
                new_f = new_g  # Simple heuristic (Dijkstra)
                
                heapq.heappush(pq, (
                    new_f,
                    new_g,
                    next_node,
                    time + 1,
                    path + [next_node]
                ))
        
        return None, float('inf'), self.T
    
    def _calculate_score(self, arrival: int, obj: Dict) -> float:
        """
        Calculate points for objective based on arrival time.
        
        Args:
            arrival: Arrival time
            obj: Objective dict
            
        Returns:
            Points earned (0 if missed)
        """
        if arrival < obj['release'] or arrival > obj['deadline']:
            return 0.0
        
        if arrival == obj['release']:
            return obj['points']
        
        # Late penalty
        lateness = arrival - obj['release']
        points = obj['points'] - (self.late_penalty * lateness)
        
        return max(0.0, points)
"""
Graph Module

Stores the road network and enforces structural constraints.
Does NOT handle costs - that's in cost.py.
"""

from typing import List, Tuple


class Graph:
    """
    Road network representation.
    
    Enforces:
    - Valid edges exist
    - Trucks cannot use Road Type 0 (airspace)
    - Drones can use all road types
    """
    
    def __init__(self, adjacency_matrix: List[List[int]]):
        """
        Initialize graph.
        
        Args:
            adjacency_matrix: NxN matrix where [i][j] = road type from i to j
                             -1 means no road
                             0 means airspace (drones only)
                             1-5 are ground roads
        """
        self.adjacency_matrix = adjacency_matrix
        self.num_nodes = len(adjacency_matrix)
    
    def get_neighbors(self, node: int, vehicle_type: str) -> List[Tuple[int, int]]:
        """
        Get valid neighbors for a vehicle at a node.
        
        Args:
            node: Current node
            vehicle_type: "truck" or "drone"
            
        Returns:
            List of (neighbor_node, road_type) tuples
        """
        neighbors = []
        
        for next_node in range(self.num_nodes):
            road_type = self.adjacency_matrix[node][next_node]
            
            # No road exists
            if road_type == -1:
                continue
            
            # CRITICAL: Trucks cannot use Road Type 0 (airspace)
            if vehicle_type == "truck" and road_type == 0:
                continue
            
            # Valid neighbor
            neighbors.append((next_node, road_type))
        
        return neighbors
    
    def is_valid_edge(self, from_node: int, to_node: int, vehicle_type: str) -> bool:
        """
        Check if edge is valid for vehicle type.
        
        Args:
            from_node: Source node
            to_node: Destination node
            vehicle_type: "truck" or "drone"
            
        Returns:
            True if vehicle can use this edge
        """
        road_type = self.adjacency_matrix[from_node][to_node]
        
        # No road
        if road_type == -1:
            return False
        
        # Trucks cannot fly
        if vehicle_type == "truck" and road_type == 0:
            return False
        
        return True
    
    def get_road_type(self, from_node: int, to_node: int) -> int:
        """
        Get road type between two nodes.
        
        Returns:
            Road type: -1 (no road), 0 (airspace), 1-5 (ground roads)
        """
        return self.adjacency_matrix[from_node][to_node]
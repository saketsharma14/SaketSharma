"""
Cost Calculator Module

Encodes all problem physics:
- Base weights lookup
- Weather-based blocking rules
- Final traversal costs

This is where the domain rules live.
"""

from typing import Dict, List, Optional


class CostCalculator:
    """
    Handles all cost computation and hazard detection.
    
    Rules:
    - Road Type 0 (airspace): Always free for drones, never blocked
    - Road Types 1-5: Cost = base_weight × road_type (or ×5 if blocked)
    
    Blocking Rules (BOTH conditions must be true):
    - Trucks: (W × earth_shock > 10) AND (W × rainfall > 30)
    - Drones: (W × wind > 60) AND (W × visibility < 6)
    """
    
    def __init__(self, road_weights: Dict[str, List[float]], 
                 sensor_data: Dict[str, List[float]]):
        """
        Initialize cost calculator.
        
        Args:
            road_weights: {"1": [weights...], "2": [...], ...}
            sensor_data: {"rainfall": [...], "wind": [...], ...}
        """
        self.road_weights = road_weights
        self.sensors = sensor_data
        
        # Cache: (road_type, time, vehicle_type) -> cost
        self._cache = {}
    
    def get_cost(self, road_type: int, time: int, vehicle_type: str) -> Optional[float]:
        """
        Calculate traversal cost for an edge.
        
        This is the core physics function.
        
        Args:
            road_type: Road type (0-5 or -1)
            time: Time step
            vehicle_type: "truck" or "drone"
            
        Returns:
            Cost to traverse, or None if invalid
        """
        # Check cache
        cache_key = (road_type, time, vehicle_type)
        if cache_key in self._cache:
            return self._cache[cache_key]
        
        # No road
        if road_type == -1:
            self._cache[cache_key] = None
            return None
        
        # Trucks cannot use airspace
        if vehicle_type == "truck" and road_type == 0:
            self._cache[cache_key] = None
            return None
        
        # Road Type 0: Free for drones, never blocked
        if road_type == 0:
            self._cache[cache_key] = 0.0
            return 0.0
        
        # Road Types 1-5: Compute cost with weather
        base_weight = self.road_weights[str(road_type)][time]
        is_blocked = self._is_blocked(base_weight, time, vehicle_type)
        
        # Calculate final cost
        base_cost = base_weight * road_type
        final_cost = base_cost * 5 if is_blocked else base_cost
        
        self._cache[cache_key] = final_cost
        return final_cost
    
    def _is_blocked(self, base_weight: float, time: int, vehicle_type: str) -> bool:
        """
        Check if road is blocked (hazardous) based on weather.
        
        CRITICAL: Uses AND rule - BOTH conditions must be true.
        
        Args:
            base_weight: Base weight of the road
            time: Time step
            vehicle_type: "truck" or "drone"
            
        Returns:
            True if road is blocked
        """
        if vehicle_type == "truck":
            # Truck blocking rule
            earth_shock = self.sensors['earth_shock'][time]
            rainfall = self.sensors['rainfall'][time]
            
            condition1 = (base_weight * earth_shock) > 10
            condition2 = (base_weight * rainfall) > 30
            
            return condition1 and condition2  # BOTH must be true
            
        else:  # drone
            # Drone blocking rule
            wind = self.sensors['wind'][time]
            visibility = self.sensors['visibility'][time]
            
            condition1 = (base_weight * wind) > 60
            condition2 = (base_weight * visibility) < 6
            
            return condition1 and condition2  # BOTH must be true
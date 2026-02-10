# Vehicle Routing Optimization System

Multi-agent, time-dependent routing optimization for trucks and drones.

## Project Structure

```
.
├── main.py                 # Entry point
├── requirements.txt        # Dependencies (none - stdlib only)
├── modules/
│   ├── __init__.py        # Package init
│   ├── utils.py           # I/O and validation
│   ├── graph.py           # Road network
│   ├── cost.py            # Cost calculation & blocking rules
│   └── planner.py         # A* pathfinding & assignment
├── public_map.json        # Input: Graph structure
├── sensor_data.json       # Input: Weather data
├── objectives.json        # Input: Delivery objectives
└── solution.json          # Output: Generated solution
```

## Usage

```bash
python main.py
```

### Input Files Required

1. **public_map.json**
   ```json
   {
     "N": <number_of_nodes>,
     "T": <time_steps>,
     "map": [[adjacency matrix]],
     "road_weights": {
       "0": [weights...],
       "1": [weights...],
       ...
     }
   }
   ```

2. **sensor_data.json**
   ```json
   {
     "rainfall": [...],
     "wind": [...],
     "visibility": [...],
     "earth_shock": [...]
   }
   ```

3. **objectives.json**
   ```json
   {
     "start_node": <int>,
     "late_penalty_per_step": <float>,
     "objectives": [
       {
         "node": <int>,
         "release": <int>,
         "deadline": <int>,
         "points": <float>
       },
       ...
     ]
   }
   ```

### Output

**solution.json**
```json
{
  "truck1": [0, 0, 1, 2, ...],
  "truck2": [0, 0, 0, 3, ...],
  "truck3": [...],
  "drone1": [...],
  "drone2": [...]
}
```

Each vehicle has a path of exactly T nodes.

## Algorithm

1. **Prioritization**: Sort objectives by points/time_window ratio
2. **Assignment**: Greedy assignment to vehicles with best benefit
3. **Pathfinding**: Time-aware A* with weather-dependent costs
4. **Extension**: Pad paths to length T by waiting

## Key Rules

### Road Types
- **Type 0**: Airspace (drones only, zero cost, never blocked)
- **Types 1-5**: Ground roads (both vehicles)

### Costs
- **Base**: `base_weight × road_type`
- **Blocked**: `base_weight × road_type × 5`

### Blocking (Perfect Storm - BOTH conditions must be true)
- **Trucks**: `(W × earth_shock > 10) AND (W × rainfall > 30)`
- **Drones**: `(W × wind > 60) AND (W × visibility < 6)`

### Scoring
- Arrive before release → Invalid (0 points)
- Arrive in [release, deadline] → `points - penalty × (arrival - release)`
- Arrive after deadline → 0 points

## Score Calculation

```
Total Score = Σ(objective points) - Σ(travel costs)
```
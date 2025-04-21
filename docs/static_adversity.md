# Static Adversity Documentation

## Overview

Static adversities are predefined adversarial events that remain fixed in the simulation environment. They are used to create specific challenging scenarios such as collisions, construction zones, and stalled vehicles.

## Types of Static Adversities

### 1. Collision Adversity
- **Purpose**: Simulates vehicle collisions in the environment
- **Implementation**: `CollisionAdversity` class
- **Key Features**:
  - Supports different collision types (rear-end, side-swipe)
  - Allows configuration of vehicle types and positions
  - Can be set with a duration for temporary collisions

### 2. Construction Adversity
- **Purpose**: Simulates construction zones that block lanes
- **Implementation**: `ConstructionAdversity` class
- **Key Features**:
  - Blocks specific lanes from vehicle access
  - Can be configured for different lane types

### 3. Stalled Object Adversity
- **Purpose**: Simulates stalled or broken-down vehicles
- **Implementation**: `StalledObjectAdversity` class
- **Key Features**:
  - Places stationary vehicles in specified locations
  - Can be configured with different vehicle types
  - Supports duration settings

## Configuration

Static adversities are configured in the simulation configuration file (e.g., `config.yaml`):

```yaml
adversity_cfg:
  static:
    collision:
      _target_: terasim_nde_nade.adversity.static.CollisionAdversity
      lane_id: "edge_0_0"  # Lane where collision occurs
      lane_position: 50    # Position on the lane
      object_type: ["car", "truck"]  # Types of vehicles involved
      other_settings:
        collision_type: "rear_end"  # Type of collision
        duration: 100  # Duration in simulation steps
```

## Initialization Process

1. **Environment Start**:
   - Static adversities are initialized in the `on_start` method of the NADE environment
   - The `StaticAdversityManager` is created and initialized with the configuration

2. **Warmup Phase**:
   - During the warmup phase (`sumo_warmup` method), static adversities are updated each simulation step
   - This ensures proper placement and stability of adversarial elements
   - The system checks for excessive vehicle counts (>2500) and reloads if necessary

3. **Update Mechanism**:
   - Each static adversity has an `update` method that:
     - Maintains the position and state of adversarial elements
     - Handles duration-based removal of elements
     - Ensures stability of the simulation

## Implementation Details

### Collision Adversity Implementation

```python
class CollisionAdversity(AbstractStaticAdversity):
    def initialize(self):
        # Create vehicles
        object1_id = f"BV_1_{self._object_type[0]}_collision"
        object2_id = f"BV_2_{self._object_type[1]}_collision"
        
        # Add vehicles to simulation
        traci.vehicle.add(object1_id, ...)
        traci.vehicle.add(object2_id, ...)
        
        # Position vehicles based on collision type
        if collision_type == "rear_end":
            # Position for rear-end collision
            ...
        elif collision_type == "side_swipe":
            # Position for side-swipe collision
            ...
```

### Construction Adversity Implementation

```python
class ConstructionAdversity(AbstractStaticAdversity):
    def initialize(self):
        # Block the lane
        traci.lane.setDisallowed(self._lane_id, ["all"])
```

### Stalled Object Adversity Implementation

```python
class StalledObjectAdversity(AbstractStaticAdversity):
    def initialize(self):
        # Create stalled vehicle
        stalled_object_id = f"BV_{self._object_type}_stalled_object"
        traci.vehicle.add(stalled_object_id, ...)
        traci.vehicle.setSpeed(stalled_object_id, 0)
```

## Error Handling

1. **Validation**:
   - Each static adversity implements an `is_effective` method
   - Validates lane IDs, positions, and vehicle types
   - Provides default values for missing parameters

2. **Runtime Errors**:
   - Logs warnings for invalid configurations
   - Handles vehicle removal errors gracefully
   - Monitors simulation stability during warmup

## Best Practices

1. **Configuration**:
   - Always specify lane IDs and positions
   - Set appropriate durations for temporary adversities
   - Use valid vehicle types from the simulation

2. **Placement**:
   - Consider traffic flow when placing adversities
   - Avoid placing multiple adversities too close together
   - Ensure sufficient space for vehicle reactions

3. **Monitoring**:
   - Monitor vehicle counts during warmup
   - Check for simulation stability
   - Verify proper removal of temporary adversities

## Example Usage

```python
# Configuration example
adversity_config = {
    "static": {
        "collision": {
            "_target_": "terasim_nde_nade.adversity.static.CollisionAdversity",
            "lane_id": "edge_0_0",
            "lane_position": 50,
            "object_type": ["car", "truck"],
            "other_settings": {
                "collision_type": "rear_end",
                "duration": 100
            }
        }
    }
}

# The adversity will be automatically initialized and managed by the NADE environment
``` 
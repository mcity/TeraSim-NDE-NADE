#!/usr/bin/env python3
"""
Example usage of StalledPedestrianAdversity class.

This example demonstrates how to use the StalledPedestrianAdversity class
to add stationary pedestrians to a SUMO simulation.
"""

import os
import sys
from pathlib import Path

# Add the parent directory to sys.path to import terasim_nde_nade
parent_dir = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(parent_dir))

from terasim_nde_nade.adversity.static import StalledPedestrianAdversity


def create_stalled_pedestrian_examples():
    """Create different types of stalled pedestrian adversities."""
    
    # Example 1: Default pedestrian at bus stop
    bus_stop_pedestrian = StalledPedestrianAdversity(
        lane_id="edge_main_0",  # Edge or lane ID where pedestrian will be placed
        lane_position=50.0,     # Position along the edge/lane in meters
        start_time=0,          # Start time (simulation steps)
        end_time=300,          # End time (simulation steps), -1 for permanent
        object_type="DEFAULT", # Pedestrian type
        other_settings={
            "description": "Person waiting at bus stop",
            "scenario": "bus_stop_waiting"
        }
    )
    
    # Example 2: Elderly person having difficulty crossing
    elderly_pedestrian = StalledPedestrianAdversity(
        lane_id="edge_crossing_1",
        lane_position=25.0,
        start_time=60,
        end_time=180,
        object_type="ELDERLY",
        other_settings={
            "description": "Elderly person having difficulty crossing",
            "scenario": "crossing_assistance_needed"
        }
    )
    
    # Example 3: Child playing on sidewalk
    child_pedestrian = StalledPedestrianAdversity(
        lane_id="edge_residential_0",
        lane_position=75.0,
        start_time=120,
        end_time=240,
        object_type="CHILD",
        other_settings={
            "description": "Child playing on sidewalk",
            "scenario": "residential_area_activity"
        }
    )
    
    # Example 4: Disabled person in wheelchair
    disabled_pedestrian = StalledPedestrianAdversity(
        lane_id="edge_downtown_1",
        lane_position=100.0,
        start_time=0,
        end_time=-1,  # Permanent
        object_type="DISABLED",
        other_settings={
            "description": "Person in wheelchair at accessible location",
            "scenario": "accessibility_scenario"
        }
    )
    
    # Example 5: Adult pedestrian looking at phone
    distracted_pedestrian = StalledPedestrianAdversity(
        lane_id="edge_commercial_0",
        lane_position=30.0,
        start_time=90,
        end_time=150,
        object_type="ADULT",
        other_settings={
            "description": "Adult pedestrian distracted by phone",
            "scenario": "distracted_walking"
        }
    )
    
    return [
        bus_stop_pedestrian,
        elderly_pedestrian,
        child_pedestrian,
        disabled_pedestrian,
        distracted_pedestrian
    ]


def example_configuration_yaml():
    """Return example YAML configuration for stalled pedestrian adversities."""
    
    yaml_config = """
# Example configuration for stalled pedestrian adversities
adversity_cfg:
  static:
    # Bus stop scenario
    bus_stop_pedestrian:
      _target_: terasim_nde_nade.adversity.static.StalledPedestrianAdversity
      lane_id: "edge_main_0"
      lane_position: 50.0
      start_time: 0
      end_time: 300
      object_type: "DEFAULT"
      other_settings:
        description: "Person waiting at bus stop"
        scenario: "bus_stop_waiting"
    
    # Elderly crossing scenario
    elderly_crossing:
      _target_: terasim_nde_nade.adversity.static.StalledPedestrianAdversity
      lane_id: "edge_crossing_1"
      lane_position: 25.0
      start_time: 60
      end_time: 180
      object_type: "ELDERLY"
      other_settings:
        description: "Elderly person having difficulty crossing"
        scenario: "crossing_assistance_needed"
    
    # Child playing scenario
    child_playing:
      _target_: terasim_nde_nade.adversity.static.StalledPedestrianAdversity
      lane_id: "edge_residential_0"
      lane_position: 75.0
      start_time: 120
      end_time: 240
      object_type: "CHILD"
      other_settings:
        description: "Child playing on sidewalk"
        scenario: "residential_area_activity"
    
    # Wheelchair accessibility scenario
    wheelchair_access:
      _target_: terasim_nde_nade.adversity.static.StalledPedestrianAdversity
      lane_id: "edge_downtown_1"
      lane_position: 100.0
      start_time: 0
      end_time: -1  # Permanent
      object_type: "DISABLED"
      other_settings:
        description: "Person in wheelchair at accessible location"
        scenario: "accessibility_scenario"
    
    # Distracted pedestrian scenario
    distracted_pedestrian:
      _target_: terasim_nde_nade.adversity.static.StalledPedestrianAdversity
      lane_id: "edge_commercial_0"
      lane_position: 30.0
      start_time: 90
      end_time: 150
      object_type: "ADULT"
      other_settings:
        description: "Adult pedestrian distracted by phone"
        scenario: "distracted_walking"
"""
    
    return yaml_config


def main():
    """Main function to demonstrate the usage."""
    
    print("=== Stalled Pedestrian Adversity Examples ===")
    print()
    
    # Create examples
    pedestrian_adversities = create_stalled_pedestrian_examples()
    
    print(f"Created {len(pedestrian_adversities)} stalled pedestrian adversity examples:")
    print()
    
    for i, adversity in enumerate(pedestrian_adversities, 1):
        print(f"{i}. Pedestrian Type: {adversity._object_type}")
        print(f"   Lane ID: {adversity._lane_id}")
        print(f"   Position: {adversity._lane_position}")
        print(f"   Duration: {adversity._start_time} - {adversity._end_time}")
        if adversity._other_settings:
            print(f"   Description: {adversity._other_settings.get('description', 'N/A')}")
            print(f"   Scenario: {adversity._other_settings.get('scenario', 'N/A')}")
        print()
    
    print("=== YAML Configuration Example ===")
    print()
    print(example_configuration_yaml())


if __name__ == "__main__":
    main()
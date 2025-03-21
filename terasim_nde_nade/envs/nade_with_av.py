from addict import Dict
import copy
from loguru import logger
import numpy as np
import random

from terasim.overlay import traci, profile
import terasim.utils as utils
from terasim.params import AgentType

from .nade import NADE

from ..utils import (
    apply_collision_avoidance,
    calclulate_distance_from_centered_agent,
    CommandType,
    NDECommand,
    get_collision_type_and_prob,
    is_car_following,
    update_control_cmds_from_predicted_trajectory,
)

CAV_ID = "CAV"
CAV_ROUTE_ID = "cav_route"

class NADEWithAV(NADE):
    def __init__(self, cav_cfg, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cav_cfg = cav_cfg
        self.cache_radius = 100 if "cache_radius" not in cav_cfg else cav_cfg.cache_radius
        self.control_radius = 50 if "control_radius" not in cav_cfg else cav_cfg.control_radius
        print(self.cache_radius,self.control_radius)
        self.excluded_agent_set = set([CAV_ID])
        self.insert_bv = False

    def on_start(self, ctx):
        """Initialize the surrogate model and add AV to env.
        
        Args:
            ctx (dict): Context dictionary.
        """
        # initialize the surrogate model and add AV to env
        super().on_start(ctx)
        self.add_cav_safe()

    def add_cav_unsafe(self, edge_id="EG_35_1_14", lane_id=None, position=0, speed=0):
        """Add a CAV to the simulation.

        Args:
            edge_id (str): Edge ID where the CAV is added.
            lane_id (str): Lane ID where the CAV is added.
            position (float): Position of the CAV.
            speed (float): Speed of the CAV.
        """
        if lane_id is None:
            lane_id = edge_id + "_0"

        if hasattr(self.cav_cfg, "type"):
            cav_type = self.cav_cfg.type
        else:
            cav_type = "DEFAULT_VEHTYPE"

        self.add_vehicle(
            veh_id=CAV_ID,
            route_id=CAV_ROUTE_ID,
            lane="best",
            lane_id=lane_id,
            position=position,
            speed=speed,
            type_id=cav_type,
        )
        # set the CAV with white color
        traci.vehicle.setColor(CAV_ID, (255, 255, 255, 255))

        traci.vehicle.subscribeContext(
            CAV_ID,
            traci.constants.CMD_GET_VEHICLE_VARIABLE,
            self.cache_radius,
            [traci.constants.VAR_DISTANCE],
        )

    def add_cav_safe(self):
        """Add a CAV to the simulation safely.
        """
        # handle the route of cav: first check if there are any existing routes with the same name
        if CAV_ROUTE_ID in traci.route.getIDList():
            cav_route = traci.route.getEdges(CAV_ROUTE_ID)
        else:
            # add the cav_route to the SUMO simulation
            assert hasattr(self.cav_cfg, "route"), "CAV route is not defined in the config file"
            cav_route = self.cav_cfg.route
            traci.route.add(CAV_ROUTE_ID, cav_route)
        edge_id = cav_route[0]
        lanes = traci.edge.getLaneNumber(edge_id)
        max_attempts = 10
        if hasattr(self.cav_cfg, "type"):
            cav_type = self.cav_cfg.type
        else:
            cav_type = "DEFAULT_VEHTYPE"
        min_safe_distance = 10 + traci.vehicletype.getLength(cav_type)  # Minimum safe distance from other vehicles

        for attempt in range(max_attempts):
            lane = random.randint(0, lanes - 1)
            lane_id = f"{edge_id}_{lane}"
            lane_length = traci.lane.getLength(lane_id)
            position = random.uniform(0, lane_length)

            if self.is_position_safe(lane_id, position, min_safe_distance):
                self.add_cav_unsafe(edge_id, lane_id, position)
                logger.info(f"CAV added safely at lane {lane_id}, position {position}")
                if self.simulator.gui_flag:
                    traci.gui.trackVehicle("View #0", CAV_ID)
                return

        logger.warning("Unable to find a safe position for CAV, using fallback method")
        self.add_cav_fallback(edge_id)

    def is_position_safe(self, lane_id, position, min_safe_distance):
        """Check if the position is safe to add a CAV.

        Args:
            lane_id (str): Lane ID.
            position (float): Position to check.
            min_safe_distance (float): Minimum safe distance from other vehicles.

        Returns:
            bool: True if the position is safe, False otherwise.
        """
        # check if this lane allows vehicles
        allowed = traci.lane.getAllowed(lane_id)
        if "passenger" not in allowed:
            return False

        # Check vehicles on the same lane
        vehicles = traci.lane.getLastStepVehicleIDs(lane_id)
        for veh in vehicles:
            veh_pos = traci.vehicle.getLanePosition(veh)
            if abs(veh_pos - position) < min_safe_distance:
                return False
        return True

    def add_cav_fallback(self, edge_id):
        """Add a CAV to the simulation using a fallback method.

        Args:
            edge_id (str): Edge ID where the CAV is added.
        """
        lane = random.randint(0, traci.edge.getLaneNumber(edge_id) - 1)
        lane_id = f"{edge_id}_{lane}"
        position = traci.lane.getLength(lane_id) / 2

        # Clear area around the chosen position
        self.clear_area_around_position(
            lane_id, position, 10
        )  # Clear 10m around the position

        self.add_cav_unsafe(edge_id, lane_id, position)
        logger.warning(
            f"CAV added using fallback method at lane {lane_id}, position {position}"
        )

    def clear_area_around_position(self, lane_id, position, clear_distance):
        """Clear the area around a position on a lane.

        Args:
            lane_id (str): Lane ID.
            position (float): Position to clear around.
            clear_distance (float): Distance to clear around the position.
        """
        vehicles = traci.lane.getLastStepVehicleIDs(lane_id)
        for veh in vehicles:
            veh_pos = traci.vehicle.getLanePosition(veh)
            if abs(veh_pos - position) < clear_distance:
                traci.vehicle.remove(veh)
        logger.info(f"Cleared area around position {position} on lane {lane_id}")

    def preparation(self):
        """Prepare for the NADE step."""
        super().preparation()
        # if not self.insert_bv and traci.vehicle.getRoadID("CAV") == "379459612#1-AddedOnRampEdge":
        #     traci.vehicle.add(
        #         "BV",
        #         "test_merge",
        #         typeID="veh_passenger",
        #         departSpeed="max",
        #         departLane="2",
        #     )
        #     self.insert_bv = True

        # if not self.insert_bv and traci.vehicle.getRoadID("CAV") == "424040132#2" and traci.vehicle.getLanePosition("CAV") > 200:
        #     traci.vehicle.add(
        #         "BV",
        #         "test_merge2",
        #         typeID="veh_passenger",
        #         departSpeed=traci.vehicle.getSpeed("CAV"),
        #         departLane="0",
        #         arrivalLane="1",
        #         departPos="1150",
        #     )
        #     self.insert_bv = True

    @profile
    def NDE_decision(self, ctx):
        if CAV_ID in traci.vehicle.getIDList():
            cav_context_subscription_results = traci.vehicle.getContextSubscriptionResults(CAV_ID)
            tmp_terasim_controlled_vehicle_ids = list(cav_context_subscription_results.keys())
            # also exclude the static adversarial vehicles
            static_adversarial_object_id_list = []
            if self.static_adversity is not None and self.static_adversity.adversity is not None:
                for object_id in self.static_adversity.adversity._static_adversarial_object_id_list:
                    static_adversarial_object_id_list.append(object_id)
                    if object_id in tmp_terasim_controlled_vehicle_ids:
                        tmp_terasim_controlled_vehicle_ids.remove(object_id)
            self.simulator.ctx = {
                "terasim_controlled_vehicle_ids": tmp_terasim_controlled_vehicle_ids,
                "static_adversarial_object_id_list": static_adversarial_object_id_list,
            }
        return super().NDE_decision(self.simulator.ctx)

    @profile
    def NADE_decision(self, env_command_information, env_observation):
        """Make decisions using the NADE model.

        Args:
            env_command_information (dict): Command information from the environment.
            env_observation (dict): Observation from the environment.

        Returns:
            tuple: Tuple containing the control commands, updated command information, weight, future trajectory, maneuver challenge, and criticality.
        """
        predicted_CAV_control_command = self.predict_cav_control_command(env_observation)
        if env_command_information[AgentType.VEHICLE][CAV_ID] is None:
            env_command_information[AgentType.VEHICLE][CAV_ID] = Dict()
        if predicted_CAV_control_command is not None:
            env_command_information[AgentType.VEHICLE][CAV_ID]["ndd_command_distribution"] = Dict(
                {
                    # "adversarial": predicted_CAV_control_command,
                    # "normal": NDECommand(command_type=CommandType.DEFAULT, prob=0),
                    "normal": predicted_CAV_control_command,
                }
            )
        else:
            env_command_information[AgentType.VEHICLE][CAV_ID]["ndd_command_distribution"] = Dict(
                {
                    "normal": NDECommand(command_type=CommandType.DEFAULT, prob=1),
                }
            )
        CAV_command_cache = copy.deepcopy(env_command_information[AgentType.VEHICLE][CAV_ID]["command_cache"])

        # filter the env_command_information and env_observation by the control radius from CAV
        distance_from_CAV = calclulate_distance_from_centered_agent(
            env_observation, CAV_ID, AgentType.VEHICLE
        )
        neighbor_agent_set = set(
            [agent_id for agent_id in distance_from_CAV if distance_from_CAV[agent_id] <= self.control_radius]
        )
        filtered_env_command_information = {
            agent_type: {
                agent_id: env_command_information[agent_type][agent_id]
                for agent_id in env_command_information[agent_type]
                if agent_id in neighbor_agent_set
            }
            for agent_type in env_command_information
        }
        filtered_env_observation = {
            agent_type: {
                agent_id: env_observation[agent_type][agent_id]
                for agent_id in env_observation[agent_type]
                if agent_id in neighbor_agent_set
            }
            for agent_type in env_observation
        }

        (
            nade_control_commands,
            filtered_env_command_information,
            weight,
            filtered_env_future_trajectory,
            filtered_env_maneuver_challenge,
            filtered_env_criticality,
        ) = super().NADE_decision(filtered_env_command_information, filtered_env_observation)
        nade_control_commands[AgentType.VEHICLE][CAV_ID] = CAV_command_cache
        return (
            nade_control_commands,
            env_command_information,
            weight,
            filtered_env_future_trajectory,
            filtered_env_maneuver_challenge,
            filtered_env_criticality,
        )

    @profile
    def NADE_decision_and_control(self, env_command_information, env_observation):
        """Make decisions and control the agents around the CAV using the NADE model.

        Args:
            env_command_information (dict): Command information from the environment.
            env_observation (dict): Observation from the environment.
        """
        if CAV_ID in traci.vehicle.getIDList():
            CAV_control_command_cache = env_command_information[AgentType.VEHICLE][CAV_ID]["command_cache"]
            (
                nade_control_commands,
                env_command_information,
                weight,
                env_future_trajectory,
                _,
                _,
            ) = self.NADE_decision(
                env_command_information, env_observation
            )
            self.importance_sampling_weight *= weight  # update weight by negligence
            nade_control_commands, env_command_information, weight, self.record = apply_collision_avoidance(
                env_future_trajectory, env_command_information, nade_control_commands, self.record, excluded_agent_set=self.excluded_agent_set
            )
            self.importance_sampling_weight *= (
                weight  
            ) # update weight by collision avoidance
            nade_control_commands = update_control_cmds_from_predicted_trajectory(
                nade_control_commands, env_future_trajectory, excluded_agent_set=self.excluded_agent_set
            ) # update the control commands according to the predicted trajectory
            if hasattr(self, "nnde_make_decisions"):
                nnde_control_commands, _ = self.nnde_make_decisions()
                nade_control_commands = self.merge_NADE_NeuralNDE_control_commands(
                    nade_control_commands, nnde_control_commands
                )
            self.refresh_control_commands_state()
            if CAV_ID in nade_control_commands[AgentType.VEHICLE] and CAV_control_command_cache is not None:
                nade_control_commands[AgentType.VEHICLE][CAV_ID] = CAV_control_command_cache
            self.execute_control_commands(nade_control_commands)
            self.record_step_data(env_command_information)

    def calculate_total_distance(self):
        """Calculate the total distance traveled by the CAV.

        Returns:
            float: Total distance traveled by the CAV.
        """
        try:
            CAV_distance = traci.vehicle.getDistance(CAV_ID)
        except:
            CAV_distance = 0
            if CAV_ID not in self.distance_info.before:
                CAV_distance += self.distance_info.after[CAV_ID]
            else:
                CAV_distance += (
                    self.distance_info.after[CAV_ID] - self.distance_info.before[CAV_ID]
                )
        return CAV_distance

    def predict_cav_control_command(
        self, env_observation
    ):
        """Predict the control command for the CAV.

        Args:
            env_observation (dict): Observation from the environment.

        Returns:
            NDECommand: Predicted control command for the CAV.
        """
        original_cav_speed = env_observation[AgentType.VEHICLE][CAV_ID]["ego"]["velocity"]
        original_cav_acceleration = env_observation[AgentType.VEHICLE][CAV_ID]["ego"]["acceleration"]
        new_cav_speed = traci.vehicle.getSpeedWithoutTraCI(CAV_ID)
        new_cav_acceleration = (
            new_cav_speed - original_cav_speed
        ) / utils.get_step_size()

        original_cav_angle = env_observation[AgentType.VEHICLE][CAV_ID]["ego"]["heading"]
        cav_lane_id = traci.vehicle.getLaneID(CAV_ID)
        cav_lane_position = traci.vehicle.getLanePosition(CAV_ID)
        cav_lane_angle = traci.lane.getAngle(
            laneID=cav_lane_id,
            relativePosition=max(
                cav_lane_position - 0.5 * traci.vehicle.getLength(CAV_ID), 0
            ),
        )
        CAV_command = None

        # step 1. use CAV signal to predict the control command
        cav_signal = traci.vehicle.getSignals(CAV_ID)
        if cav_signal == 1: # right turn signal, please consider the drive rule: lefthand or righthand
            if self.configuration.drive_rule == "righthand":
                CAV_command = NDECommand(
                    command_type=CommandType.RIGHT,
                    prob=1,
                    duration=1.0,
                    info={"adversarial_mode": "RightFoll"},
                )
            else:
                CAV_command = NDECommand(
                    command_type=CommandType.LEFT,
                    prob=1,
                    duration=1.0,
                    info={"adversarial_mode": "LeftFoll"},
                )
        elif cav_signal == 2: # left turn signal, please consider the drive rule: lefthand or righthand
            if self.configuration.drive_rule == "righthand":
                CAV_command = NDECommand(
                    command_type=CommandType.LEFT,
                    prob=1,
                    duration=1.0,
                    info={"adversarial_mode": "LeftFoll"},
                )
            else:
                CAV_command = NDECommand(
                    command_type=CommandType.RIGHT,
                    prob=1,
                    duration=1.0,
                    info={"adversarial_mode": "RightFoll"},
                )

        elif cav_signal == 0: # no signal
            # step 2. use the difference between the lane change angle adn the original cav angle to predict the control command (LEFT turn or RIGHT turn)
            # the angle is defined as SUmo's angle, the north is 0, the east is 90, the south is 180, the west is 270
            # the angle is in degree
            angle_diff = (cav_lane_angle - original_cav_angle + 180) % 360 - 180

            if angle_diff > 10:
                CAV_command = NDECommand(
                    command_type=CommandType.LEFT,
                    prob=1,
                    duration=1.0,
                    info={"adversarial_mode": "LeftFoll"},
                )
            elif angle_diff < -10:
                CAV_command = NDECommand(
                    command_type=CommandType.RIGHT,
                    prob=1,
                    duration=1.0,
                    info={"adversarial_mode": "RightFoll"},
                )

            if original_cav_acceleration - new_cav_acceleration > 1.5:
                # predict the cav control command as negligence
                leader_info = traci.vehicle.getLeader(CAV_ID)
                is_car_following_flag = False
                if leader_info is not None:
                    is_car_following_flag = is_car_following(CAV_ID, leader_info[0])
                CAV_command = NDECommand(
                    command_type=CommandType.ACCELERATION,
                    acceleration=original_cav_acceleration,
                    prob=1,
                    duration=1.0,
                    info={
                        "adversarial_mode": "Lead",
                        "is_car_following_flag": is_car_following_flag,
                    },
                )

        if CAV_command:
            _, predicted_collision_type = get_collision_type_and_prob(
                obs_dict=env_observation[AgentType.VEHICLE][CAV_ID],
                adversarial_command=CAV_command,
            )
            CAV_command.info.update(
                {"predicted_collision_type": predicted_collision_type}
            )
        return CAV_command

    def should_continue_simulation(self):
        """Check if the simulation should continue. There are four conditions to stop the simulation:
        1. Collision happens between two vehicles.
        2. Collision happens between a vehicle and a VRU.
        3. CAV leaves the simulation.
        4. Simulation timeout.

        Returns:
            bool: True if the simulation should continue, False otherwise.
        """
        num_colliding_vehicles = self.simulator.get_colliding_vehicle_number()
        self._vehicle_in_env_distance("after")

        if num_colliding_vehicles >= 2:
            colliding_vehicles = self.simulator.get_colliding_vehicles()
            veh_1_id = colliding_vehicles[0]
            veh_2_id = colliding_vehicles[1]
            self.record.update(
                {
                    "veh_1_id": veh_1_id,
                    "veh_1_obs": self.vehicle_list[veh_1_id].observation,
                    "veh_2_id": veh_2_id,
                    "veh_2_obs": self.vehicle_list[veh_2_id].observation,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "collision",
                }
            )
            return False
        elif num_colliding_vehicles == 1: # collision happens between a vehicle and a vru.
            colliding_vehicles = self.simulator.get_colliding_vehicles()
            veh_1_id = colliding_vehicles[0]
            collision_objects = traci.simulation.getCollisions()
            if veh_1_id == collision_objects[0].collider:
                vru_1_id = collision_objects[0].victim
            elif veh_1_id == collision_objects[0].victim:
                vru_1_id = collision_objects[0].collider
            else:
                vru_1_id = None
            self.record.update(
                {
                    "veh_1_id": veh_1_id,
                    "veh_1_obs": self.vehicle_list[veh_1_id].observation,
                    "vru_1_id": vru_1_id,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "collision",
                }
            )
            return False
        elif CAV_ID not in traci.vehicle.getIDList():
            logger.info(f"{CAV_ID} left the simulation, stop the simulation.")
            self.record.update(
                {
                    "veh_1_id": None,
                    "veh_2_id": None,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "CAV_left",
                }
            )
            return False
        elif utils.get_time() >= self.warmup_time + self.run_time:
            logger.info("Simulation timeout, stop the simulation.")
            self.record.update(
                {
                    "veh_1_id": None,
                    "veh_2_id": None,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "timeout",
                }
            )
            return False
        return True

    def record_step_data(self, env_command_information):
        """Record the step data.

        Args:
            env_command_information (dict): Command information from the environment.
        """
        step_log = Dict()
        bv_criticality_list = []
        for agent_type in [AgentType.VEHICLE, AgentType.VULNERABLE_ROAD_USER]:
            for agent_id, agent_command_info in env_command_information[agent_type].items():
                maneuver_challenge = agent_command_info.get("maneuver_challenge", None)
                if maneuver_challenge and maneuver_challenge.get("adversarial", None):
                    step_log[agent_id]["maneuver_challenge"] = maneuver_challenge

                keys = ["avoidable", "conflict_vehicle_list", "mode"]
                step_log[agent_id].update(
                    {key: agent_command_info[key] for key in keys if agent_command_info.get(key)}
                )
                if step_log[agent_id].get("avoidable"):
                    step_log[agent_id].pop("avoidable") # remove the avoidable key if it is True
                if agent_id != CAV_ID:
                    criticality = 0.0
                    if (
                        "criticality" in agent_command_info
                        and "adversarial" in agent_command_info["criticality"]
                    ):
                        criticality = agent_command_info["criticality"]["adversarial"]
                    bv_criticality_list.append(criticality)
        # pop the empty dict
        step_log = {k: v for k, v in step_log.items() if v}
        step_log = {
            "weight": self.importance_sampling_weight,
            "vehicle_log": step_log,
        }
        time_step = utils.get_time()
        self.record.step_info[time_step] = step_log
        self.record.weight_step_info[time_step] = self.step_weight
        self.record.epsilon_step_info[time_step] = self.step_epsilon
        self.record.criticality_step_info[time_step] = sum(bv_criticality_list)
        self.record.drl_obs[time_step] = self.collect_drl_obs(env_command_information).tolist()
        overall_avoidable = True
        for agent_type in env_command_information:
            for agent_id in env_command_information[agent_type]:
                if not env_command_information[agent_type][agent_id].get("avoidable", True):
                    overall_avoidable = False
        self.record.avoidable[time_step] = overall_avoidable

        return step_log

    def collect_drl_obs(self, env_command_information):
        """Collect the observation for D2RL-based adversity model.

        Args:
            env_command_information (dict): Command information from the environment.

        Returns:
            np.ndarray: Observation for D2RL-based adversity model.
        """
        CAV_global_position = list(traci.vehicle.getPosition(CAV_ID))
        CAV_speed = traci.vehicle.getSpeed(CAV_ID)
        CAV_heading = traci.vehicle.getAngle(CAV_ID)
        CAV_driving_distance = traci.vehicle.getDistance(CAV_ID)
        # position x, position y, CAV driving distance, velocity, heading
        vehicle_info_list = []
        controlled_bv_num = 1
        for veh_id, veh_command_information in env_command_information[AgentType.VEHICLE].items():
            if veh_id == CAV_ID:
                continue
            if (
                "criticality" in veh_command_information
                and "negligence" in veh_command_information["criticality"]
            ):
                criticality = veh_command_information["criticality"]["adversarial"]
                if criticality > 0:
                    vehicle_local_position = list(traci.vehicle.getPosition(veh_id))
                    vehicle_relative_position = [
                        vehicle_local_position[0] - CAV_global_position[0],
                        vehicle_local_position[1] - CAV_global_position[1],
                    ]
                    vehicle_speed = traci.vehicle.getSpeed(veh_id)
                    vehicle_heading = traci.vehicle.getAngle(veh_id)
                    vehicle_info_list.extend(
                        vehicle_relative_position + [vehicle_speed] + [vehicle_heading]
                    )
                    break

        if not vehicle_info_list:
            vehicle_info_list.extend([-100, -100, 0, 0])

        velocity_lb, velocity_ub = 0, 10
        CAV_position_lb, CAV_position_ub = [0, 0], [240, 400]
        driving_distance_lb, driving_distance_ub = 0, 1000
        heading_lb, heading_ub = 0, 360
        vehicle_info_lb, vehicle_info_ub = [-20, -20, 0, 0], [20, 20, 10, 360]

        lb_array = np.array(
            CAV_position_lb
            + [velocity_lb]
            + [driving_distance_lb]
            + [heading_lb]
            + vehicle_info_lb
        )
        ub_array = np.array(
            CAV_position_ub
            + [velocity_ub]
            + [driving_distance_ub]
            + [heading_ub]
            + vehicle_info_ub
        )
        total_obs_for_DRL_ori = np.array(
            CAV_global_position
            + [CAV_speed]
            + [CAV_driving_distance]
            + [CAV_heading]
            + vehicle_info_list
        )

        total_obs_for_DRL = (
            2 * (total_obs_for_DRL_ori - lb_array) / (ub_array - lb_array) - 1
        )
        total_obs_for_DRL = np.clip(total_obs_for_DRL, -5, 5)
        return np.array(total_obs_for_DRL).astype(float)

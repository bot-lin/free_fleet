#!/usr/bin/env python3

# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import importlib
import json
import threading
import time
import traceback
from typing import Annotated
from urllib.error import URLError, HTTPError
from urllib.request import Request, urlopen

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from action_msgs.msg import GoalStatus as RosGoalStatus
from geometry_msgs.msg import Pose as RosPose
from geometry_msgs.msg import PoseStamped as RosPoseStamped
from nav2_msgs.action import NavigateThroughPoses
from sensor_msgs.msg import BatteryState as RosBatteryState
from tf_transformations import euler_from_quaternion, quaternion_from_euler

import rmf_adapter.easy_full_control as rmf_easy
from rmf_adapter.robot_update_handle import (
    ActionExecution,
    ActivityIdentifier,
    Tier,
)

from free_fleet.utils import namespacify
from free_fleet_adapter.action import RobotActionContext, RobotActionState
from free_fleet_adapter.robot_adapter import (
    ExecutionFeedback,
    ExecutionHandle,
    RobotAdapter,
)


def _wait_future(fut, timeout_sec: float) -> bool:
    """等待 future 完成，不做 spin（避免嵌套 spin/抢占 executor）。"""
    done_evt = threading.Event()

    def _mark_done(_):
        done_evt.set()

    fut.add_done_callback(_mark_done)
    return done_evt.wait(timeout=timeout_sec)


class Nav2RobotAdapter(RobotAdapter):

    def __init__(
        self,
        name: str,
        configuration,
        robot_config_yaml,
        plugin_config: dict | None,
        node,
        fleet_handle,
        fleet_config: rmf_easy.FleetConfiguration | None,
        tf_buffer,
    ):
        super().__init__(name, node, fleet_handle)

        self.configuration = configuration
        self.robot_config_yaml = robot_config_yaml
        self.fleet_config = fleet_config
        self.tf_buffer = tf_buffer

        self.exec_handle: ExecutionHandle | None = None
        self.map_name = self.robot_config_yaml["initial_map"]
        self.map_frame = self.robot_config_yaml.get("map_frame", "map")
        self.robot_frame = self.robot_config_yaml.get("robot_frame", "base_footprint")
        self.robot_ip = self.robot_config_yaml.get("network_ip")

        self.service_call_timeout_sec = float(
            self.robot_config_yaml.get("service_call_timeout_sec", 5.0)
        )

        # Fetch current map in the main thread before doing anything else.
        # If this fails, treat the robot as not connected and do not add it to the fleet.
        self._map_http_timeout_sec = float(
            self.robot_config_yaml.get("current_map_http_timeout_sec", 1.0)
        )
        if self._map_http_timeout_sec <= 0:
            self._map_http_timeout_sec = 1.0

        if not self.robot_ip:
            raise RuntimeError(
                f'Robot [{self.name}] has no "network_ip" in config; '
                'cannot fetch current map.'
            )

        url = f'http://{self.robot_ip}:5000/deploy/getCurrentMap'
        self.node.get_logger().info(
            f'Robot [{self.name}] fetching current map (startup) from [{url}] '
            f'(timeout={self._map_http_timeout_sec:.2f}s)...'
        )
        current_map = self._fetch_current_map_name()
        if not current_map:
            raise RuntimeError(
                f'Robot [{self.name}] cannot fetch current map from {url}'
            )
        self.map_name = current_map
        self.node.get_logger().info(
            f'Robot [{self.name}] current map (startup): [{self.map_name}]'
        )

        # Robot state
        self.battery_soc = 1.0
        self.robot_pose = None
        self._state_lock = threading.Lock()

        self.replan_counts = 0
        self.nav_issue_ticket = None

        # Plugin actions
        self.action_to_plugin_name: dict[str, str] = {}
        self.action_factories = {}

        # ROS2 subscriptions (absolute topics)
        self._ros_robot_pose_topic = f'/{namespacify("robot_pose", self.name)}'
        self._ros_battery_topic = f'/{namespacify("battery/state", self.name)}'

        self.robot_pose_sub = self.node.create_subscription(
            RosPose,
            self._ros_robot_pose_topic,
            self._robot_pose_ros_callback,
            qos_profile_sensor_data,
        )
        self.battery_state_sub = self.node.create_subscription(
            RosBatteryState,
            self._ros_battery_topic,
            self._battery_state_ros_callback,
            qos_profile_sensor_data,
        )

        # ROS2 Nav2 Action + Services (namespaced by robot name)
        self._nav_action_name = f'/{namespacify("navigate_through_poses", self.name)}'
        self._nav_action_client = ActionClient(
            self.node, NavigateThroughPoses, self._nav_action_name
        )

        # Map switching is done via robot HTTP API (see change_map)

        # Initialize robot pose
        init_timeout_sec = float(self.robot_config_yaml.get("init_timeout_sec", 10))
        self.node.get_logger().info(f"Initializing robot [{self.name}]...")
        init_robot_pose = rclpy.Future()

        def _get_init_pose():
            pose = self.get_pose()
            if pose is not None and not init_robot_pose.done():
                init_robot_pose.set_result(pose)

        init_pose_timer = self.node.create_timer(1.0, _get_init_pose)
        rclpy.spin_until_future_complete(self.node, init_robot_pose, timeout_sec=init_timeout_sec)
        self.node.destroy_timer(init_pose_timer)

        if init_robot_pose.result() is None:
            raise RuntimeError(f"Timeout trying to initialize robot [{self.name}]")

        state = rmf_easy.RobotState(
            self.get_map_name(),
            init_robot_pose.result(),
            self.get_battery_soc(),
        )

        if self.fleet_handle is None:
            self.node.get_logger().warn(
                f"Fleet unavailable, skipping adding robot [{self.name}] to fleet."
            )
            return

        self.update_handle = self.fleet_handle.add_robot(
            self.name,
            state,
            self.configuration,
            rmf_easy.RobotCallbacks(
                lambda destination, execution: self.navigate(destination, execution),
                lambda activity: self.stop(activity),
                lambda category, description, execution: self.execute_action(
                    category, description, execution
                ),
            ),
        )
        if not self.update_handle:
            raise RuntimeError(
                f"Failed to add robot [{self.name}] to fleet "
                f"[{self.fleet_handle.more().fleet_name}]"
            )

        # Load plugin actions
        if self.fleet_config is None:
            self.node.get_logger().info(
                f"No fleet configuration provided for RobotAdapter [{self.name}]. "
                "No plugin actions will be loaded."
            )
            return

        if plugin_config is not None:
            for plugin_name, action_config in plugin_config.items():
                try:
                    module = action_config["module"]
                    plugin = importlib.import_module(module)
                    action_context = RobotActionContext(
                        self.node,
                        self.name,
                        self.update_handle,
                        self.fleet_config,
                        action_config,
                        self.get_battery_soc,
                        self.get_map_name,
                        self.get_pose,
                    )
                    action_factory = plugin.ActionFactory(action_context)
                    for action in action_factory.actions:
                        target_plugin = self.action_to_plugin_name.get(action)
                        if target_plugin is not None and target_plugin != plugin_name:
                            raise Exception(
                                f"Action [{action}] is duplicated across plugins: "
                                f"{target_plugin} and {plugin_name}"
                            )
                        if not action_factory.supports_action(action):
                            raise ValueError(
                                f"Plugin [{plugin_name}] does not support configured action [{action}]"
                            )
                        self.action_to_plugin_name[action] = plugin_name
                    self.action_factories[plugin_name] = action_factory
                except ImportError as e:
                    self.node.get_logger().error(
                        f"Unable to import plugin module [{action_config.get('module')}] "
                        f"for plugin [{plugin_name}] on robot [{self.name}]: {type(e)}: {e}"
                    )
                    self.node.get_logger().debug(traceback.format_exc())
                except Exception as e:
                    self.node.get_logger().error(
                        f"Unable to create ActionFactory for plugin [{plugin_name}] "
                        f"on robot [{self.name}]: {type(e)}: {e}"
                    )
                    self.node.get_logger().debug(traceback.format_exc())

    # ------------------------
    # ROS subscriptions
    # ------------------------
    def _battery_state_ros_callback(self, msg: RosBatteryState):
        soc = float(msg.percentage) if msg.percentage == msg.percentage else self.battery_soc
        if soc < 0.0:
            soc = 0.0
        if soc > 1.0:
            # Some stacks publish [0, 100]
            soc = soc / 100.0 if soc <= 100.0 else 1.0
        with self._state_lock:
            self.battery_soc = soc

    def _robot_pose_ros_callback(self, msg: RosPose):
        with self._state_lock:
            self.robot_pose = msg

    # ------------------------
    # RobotAdapter API
    # ------------------------
    def get_battery_soc(self) -> float:
        with self._state_lock:
            return float(self.battery_soc)

    def get_map_name(self) -> str:
        with self._state_lock:
            return str(self.map_name)

    def get_pose(self) -> Annotated[list[float], 3] | None:
        with self._state_lock:
            pose = self.robot_pose
        if pose is None:
            return None

        yaw = euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        )[2]
        return [pose.position.x, pose.position.y, yaw]

    def update(self, state: rmf_easy.RobotState):
        if self.update_handle is None:
            self.node.get_logger().error(
                f"Failed to update robot {self.name}, update_handle is None"
            )
            return

        activity_identifier = None
        exec_handle = self.exec_handle
        if exec_handle:
            if exec_handle.execution and getattr(exec_handle, "_ros_result_future", None):
                if self._is_navigation_done(exec_handle):
                    exec_handle.execution.finished()
                    exec_handle.execution = None
                    self.replan_counts = 0
            elif exec_handle.execution and exec_handle.action:
                current_action_state = exec_handle.action.update_action()
                match current_action_state:
                    case RobotActionState.CANCELED | RobotActionState.COMPLETED | RobotActionState.FAILED:
                        exec_handle.execution.finished()
                        exec_handle.action = None
            activity_identifier = exec_handle.activity

        self.update_handle.update(state, activity_identifier)

    def navigate(self, destination: rmf_easy.Destination, execution: rmf_easy.CommandExecution):
        self._request_stop(self.exec_handle)
        self.node.get_logger().info(
            f"Commanding [{self.name}] to navigate to {destination.position} on map [{destination.map}]"
        )
        self.exec_handle = ExecutionHandle(execution)
        self._handle_navigate_through_poses(
            destination.map,
            destination.position[0],
            destination.position[1],
            0.0,
            destination.position[2],
            self.exec_handle,
        )

    def stop(self, activity: ActivityIdentifier):
        exec_handle = self.exec_handle
        if exec_handle is None:
            return
        if exec_handle.execution is not None and activity.is_same(exec_handle.activity):
            self._request_stop(exec_handle)
            self.exec_handle = None

    def execute_action(self, category: str, description: dict, execution: ActionExecution):
        current_exec_handle = self.exec_handle
        if current_exec_handle is not None and current_exec_handle.action is not None:
            if current_exec_handle.execution is not None:
                current_exec_handle.execution.finished()
            current_exec_handle.action = None

        action_factory = None
        plugin_name = self.action_to_plugin_name.get(category)
        if plugin_name:
            action_factory = self.action_factories.get(plugin_name)
        else:
            for _, factory in self.action_factories.items():
                if factory.supports_action(category):
                    factory.actions.append(category)
                    action_factory = factory
                    break

        if action_factory:
            robot_action = action_factory.perform_action(category, description, execution)
            self.exec_handle = ExecutionHandle(execution)
            self.exec_handle.set_action(robot_action)
            return

        raise NotImplementedError(f"RobotAction [{category}] was not configured for this fleet.")

    # ------------------------
    # Nav2 control (ROS2)
    # ------------------------
    def _request_stop(self, exec_handle: ExecutionHandle | None):
        if exec_handle is None:
            return
        # Wait until a goal_id is set (ExecutionHandle mutex held until then)
        with exec_handle.mutex:
            if exec_handle.goal_id is not None:
                self._handle_stop_navigation(exec_handle)

    def _handle_stop_navigation(self, exec_handle: ExecutionHandle):
        gh = getattr(exec_handle, "_ros_goal_handle", None)
        if gh is None:
            return
        try:
            gh.cancel_goal_async()
        except Exception as e:
            self.node.get_logger().warn(f"Failed to cancel goal: {type(e)}: {e}")

    def _is_navigation_done(self, nav_handle: ExecutionHandle) -> bool:
        result_future = getattr(nav_handle, "_ros_result_future", None)
        if result_future is None:
            return True
        if not result_future.done():
            return False

        try:
            result = result_future.result()
        except Exception as e:
            self.replan_counts += 1
            self.node.get_logger().error(
                f"Navigation result future failed: {type(e)}: {e}, replan count [{self.replan_counts}]"
            )
            if self.update_handle is not None:
                self.update_handle.more().replan()
            return False

        status = getattr(result, "status", None)
        if status == RosGoalStatus.STATUS_SUCCEEDED:
            return True
        if status in (
            RosGoalStatus.STATUS_ACCEPTED,
            RosGoalStatus.STATUS_EXECUTING,
            RosGoalStatus.STATUS_CANCELING,
        ):
            return False

        # aborted/rejected/canceled -> trigger replan
        self.replan_counts += 1
        self.node.get_logger().error(
            f"Navigation ended with status [{status}], replan count [{self.replan_counts}]"
        )
        if self.update_handle is not None:
            self.update_handle.more().replan()
        return False

    def _handle_navigate_through_poses(
        self,
        map_name: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
        nav_handle: ExecutionHandle,
    ):
        if map_name != self.map_name:
            self.node.get_logger().info(
                f"Destination is on map [{map_name}], while robot [{self.name}] is on map [{self.map_name}]. "
                "Attempting to change map..."
            )
            if not self.change_map(map_name):
                self.replan_counts += 1
                self.node.get_logger().error(
                    f"Failed to change map to [{map_name}], replan count [{self.replan_counts}]"
                )
                if self.update_handle is not None:
                    self.update_handle.more().replan()
                nav_handle.set_goal_id(None)
                return
            self.map_name = map_name

        if not self._nav_action_client.wait_for_server(timeout_sec=self.service_call_timeout_sec):
            self.replan_counts += 1
            self.node.get_logger().error(
                f"Nav2 action server [{self._nav_action_name}] not available, replan count [{self.replan_counts}]"
            )
            if self.update_handle is not None:
                self.update_handle.more().replan()
            nav_handle.set_goal_id(None)
            return

        pose_stamped = RosPoseStamped()
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.map_frame
        pose_stamped.pose.position.x = float(x)
        pose_stamped.pose.position.y = float(y)
        pose_stamped.pose.position.z = float(z)
        quat = quaternion_from_euler(0, 0, yaw)
        pose_stamped.pose.orientation.x = float(quat[0])
        pose_stamped.pose.orientation.y = float(quat[1])
        pose_stamped.pose.orientation.z = float(quat[2])
        pose_stamped.pose.orientation.w = float(quat[3])

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [pose_stamped]
        goal_msg.behavior_tree = self.robot_config_yaml.get("behavior_tree", "/data/behavior_trees/zc_nav.xml")
        # NOTE: Different Nav2 distributions / builds may expose different Goal
        # fields for NavigateThroughPoses. Set optional fields only when present
        # to avoid runtime AttributeError.
        if hasattr(goal_msg, "planner_id"):
            goal_msg.planner_id = self.robot_config_yaml.get("planner_id", "GridBased")
        if hasattr(goal_msg, "controller_id"):
            goal_msg.controller_id = self.robot_config_yaml.get("controller_id", "FollowPath")
        if hasattr(goal_msg, "goal_checker_id"):
            goal_msg.goal_checker_id = self.robot_config_yaml.get("goal_checker_id", "")
        if hasattr(goal_msg, "progress_checker_id"):
            goal_msg.progress_checker_id = self.robot_config_yaml.get("progress_checker_id", "")

        def _feedback_cb(feedback_msg):
            eh = self.exec_handle
            if eh is None:
                return
            eh.last_received_feedback = ExecutionFeedback(
                feedback_msg.feedback, self.node.get_clock().now().seconds_nanoseconds()[0]
            )

        send_future = self._nav_action_client.send_goal_async(
            goal_msg, feedback_callback=_feedback_cb
        )

        def _on_goal_response(fut):
            try:
                gh = fut.result()
            except Exception as e:
                self.replan_counts += 1
                self.node.get_logger().error(
                    f"Failed to send nav goal: {type(e)}: {e}, replan count [{self.replan_counts}]"
                )
                if self.update_handle is not None:
                    self.update_handle.more().replan()
                nav_handle.set_goal_id(None)
                return

            if not gh.accepted:
                self.replan_counts += 1
                self.node.get_logger().error(
                    f"Navigation goal rejected, replan count [{self.replan_counts}]"
                )
                if self.update_handle is not None:
                    self.update_handle.more().replan()
                nav_handle.set_goal_id(None)
                return

            nav_handle._ros_goal_handle = gh
            nav_handle._ros_result_future = gh.get_result_async()
            nav_handle.set_goal_id(gh.goal_id.uuid)

        send_future.add_done_callback(_on_goal_response)

    def change_map(self, map_name: str) -> bool:
        # Change map via robot HTTP API
        url = f"http://{self.robot_ip}:5000/deploy/changeMapByName/{map_name}"
        req = Request(
            url,
            method="GET",
            headers={
                "Accept": "application/json",
                "x-api-key": "1234567890",
            },
        )

        self.node.get_logger().info(
            f'Robot [{self.name}] changing map via HTTP: [{url}] '
            f'(timeout={self.service_call_timeout_sec:.2f}s)...'
        )

        try:
            with urlopen(req, timeout=self.service_call_timeout_sec) as resp:
                status = getattr(resp, "status", None)
                body = resp.read()
        except HTTPError as e:
            self.node.get_logger().error(
                f'HTTP {e.code} from {url}: {e.reason}'
            )
            return False
        except URLError as e:
            self.node.get_logger().error(
                f'HTTP request to {url} failed: {type(e.reason)}: {e.reason}'
            )
            return False
        except Exception as e:
            self.node.get_logger().error(
                f'HTTP request to {url} failed: {type(e)}: {e}'
            )
            return False

        try:
            payload = json.loads(body.decode("utf-8"))
        except Exception:
            preview = body[:200]
            self.node.get_logger().error(
                f'Invalid JSON from {url} (http_status={status}): {preview!r}'
            )
            return False

        if payload.get("code") != 0:
            self.node.get_logger().error(
                f'changeMapByName failed: code={payload.get("code")} payload={payload}'
            )
            return False

        msg = payload.get("message")
        if msg:
            self.node.get_logger().info(
                f'Robot [{self.name}] changeMapByName success: {msg}'
            )

        # Update local map name immediately
        with self._state_lock:
            self.map_name = str(map_name)
        return True

    # ------------------------
    # Current map via HTTP
    # ------------------------
    def _fetch_current_map_name(self) -> str | None:
        url = f"http://{self.robot_ip}:5000/deploy/getCurrentMap"
        req = Request(
            url,
            method="GET",
            headers={
                "Accept": "application/json",
                "x-api-key": "1234567890",
            },
        )
        try:
            with urlopen(req, timeout=self._map_http_timeout_sec) as resp:
                status = getattr(resp, "status", None)
                body = resp.read()
        except HTTPError as e:
            self.node.get_logger().error(
                f'HTTP {e.code} from {url}: {e.reason}'
            )
            return None
        except URLError as e:
            self.node.get_logger().error(
                f'HTTP request to {url} failed: {type(e.reason)}: {e.reason}'
            )
            return None
        except Exception as e:
            self.node.get_logger().error(
                f'HTTP request to {url} failed: {type(e)}: {e}'
            )
            return None

        try:
            payload = json.loads(body.decode("utf-8"))
        except Exception:
            preview = body[:200]
            self.node.get_logger().error(
                f'Invalid JSON from {url} (http_status={status}): {preview!r}'
            )
            return None

        if payload.get("code") != 0:
            self.node.get_logger().error(
                f'HTTP {url} returned code={payload.get("code")} payload={payload}'
            )
            return None
        data = payload.get("data") or {}
        map_name = data.get("map_name")
        if not map_name:
            self.node.get_logger().error(
                f'HTTP {url} missing data.map_name payload={payload}'
            )
            return None
        return str(map_name)

    # ------------------------
    # Issue tickets
    # ------------------------
    def create_nav_issue_ticket(self, category, msg, nav_goal_id=None):
        if self.update_handle is None:
            self.node.get_logger().error(
                f"Failed to create navigation issue ticket for robot {self.name}: update_handle is None"
            )
            return None

        tier = Tier.Error
        detail = {"nav_goal_id": f"{nav_goal_id}", "message": msg}
        nav_issue_ticket = self.update_handle.more().create_issue(tier, category, detail)
        self.node.get_logger().info(
            f"Created [{category}] issue ticket for robot [{self.name}] with nav goal ID [{nav_goal_id}]"
        )
        return nav_issue_ticket



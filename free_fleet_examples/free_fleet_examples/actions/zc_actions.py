#!/usr/bin/env python3

# Copyright 2025 Open Source Robotics Foundation, Inc.
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

import time

from free_fleet_adapter.action import (
    RobotAction,
    RobotActionContext,
    RobotActionFactory,
    RobotActionState,
)
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from std_msgs.msg import Empty


class ActionFactory(RobotActionFactory):

    def __init__(self, context: RobotActionContext):
        RobotActionFactory.__init__(self, context)
        self.supported_actions = [
            'go_to_charger'
        ]

    def supports_action(self, category: str) -> bool:
        return category in self.supported_actions

    def perform_action(
        self,
        category: str,
        description: dict,
        execution
    ) -> RobotAction:
        # TODO(ac): Re-instate this for ROS 2 Kilted onwards, as this feature
        # is only added after Jazzy release.
        # See https://github.com/open-rmf/rmf_ros2/pull/392 for more info.
        # execution.set_automatic_cancel(False)
        match category:
            case 'go_to_charger':
                return GoToCharger(description, execution, self.context)



# The delayed_hello_world custom action parses the description for a user
# name and duration for waiting, performs logging only after the specified
# duration has elapsed, and allows users to cancel the action via a custom
# Empty topic with topic name `cancel_delayed_hello_world`.
# Cancel the action with the following command
# ros2 topic pub --once  /cancel_delayed_hello_world std_msgs/msg/Empty "{}"
class GoToCharger(RobotAction):

    def __init__(
        self,
        description: dict,
        execution,
        context: RobotActionContext
    ):
        RobotAction.__init__(self, context, execution)

        self.context.node.get_logger().info(
            'New GoToCharger requested for robot '
            f'[{self.context.robot_name}]'
        )
        self.description = description
        self.start_millis = None

        # Enum used for tracking whether this action has been completed
        self.state = RobotActionState.IN_PROGRESS

        # Custom cancellation interaction
        self.cancel_topic = 'cancel_go_to_charger'
        self.cancel_sub = None

        # Nav2 NavigateThroughPoses action client (namespaced)
        self._action_name = f'/{self.context.robot_name}/navigate_through_poses'
        self._client = ActionClient(
            self.context.node, NavigateThroughPoses, self._action_name
        )
        self._send_goal_future = None
        self._goal_handle = None
        self._result_future = None

    def _cancel_action(self, msg: Empty):
        self.state = RobotActionState.CANCELING
        self.context.node.get_logger().info(
            'Received custom cancel command go_to_charger action for '
            f'robot [{self.context.robot_name}]'
        )

        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception as e:
                self.context.node.get_logger().warn(
                    f'Failed to cancel docking goal: {type(e)}: {e}'
                )

        def _cancel_success():
            self.state = RobotActionState.CANCELED

        def _cancel_fail():
            self.state = RobotActionState.FAILED

        # Canceling the entire task due to canceling this action
        self.cancel_task_of_action(
            _cancel_success,
            _cancel_fail,
            'Custom cancel behavior of go_to_charger action'
        )

    def update_action(self) -> RobotActionState:
        if self.state == RobotActionState.COMPLETED or \
                self.state == RobotActionState.CANCELING or \
                self.state == RobotActionState.CANCELED:
            return self.state

        # Set up the custom cancellation topic and start the docking action once
        if self.start_millis is None:
            # Start the subscription to listen for cancellation of this action
            self.cancel_sub = self.context.node.create_subscription(
                Empty,
                self.cancel_topic,
                self._cancel_action,
                10
            )

            self.start_millis = round(time.time() * 1000)

        # Send the docking goal if we haven't yet
        if self._send_goal_future is None and self._goal_handle is None:
            if not self._client.wait_for_server(timeout_sec=2.0):
                self.context.node.get_logger().error(
                    f'[{self.context.robot_name}] Nav2 action server not available: '
                    f'[{self._action_name}]'
                )
                self.state = RobotActionState.FAILED
                return self.state

            goal = NavigateThroughPoses.Goal()
            # As requested: docking uses the same action, but no poses
            goal.poses = []
            goal.behavior_tree = '/data/actions/reflector_docking.xml'

            # Optional fields exist on some Nav2 versions
            if hasattr(goal, 'planner_id'):
                goal.planner_id = ''
            if hasattr(goal, 'controller_id'):
                goal.controller_id = ''
            if hasattr(goal, 'goal_checker_id'):
                goal.goal_checker_id = ''

            self.context.node.get_logger().info(
                f'[{self.context.robot_name}] Sending GoToCharger docking goal '
                f'via [{self._action_name}]'
            )

            self._send_goal_future = self._client.send_goal_async(goal)
            return RobotActionState.IN_PROGRESS

        # If goal response is ready, latch goal handle
        if self._goal_handle is None and self._send_goal_future is not None:
            if self._send_goal_future.done():
                try:
                    self._goal_handle = self._send_goal_future.result()
                except Exception as e:
                    self.context.node.get_logger().error(
                        f'[{self.context.robot_name}] Failed to send docking goal: '
                        f'{type(e)}: {e}'
                    )
                    self.state = RobotActionState.FAILED
                    return self.state

                if not self._goal_handle.accepted:
                    self.context.node.get_logger().error(
                        f'[{self.context.robot_name}] Docking goal rejected'
                    )
                    self.state = RobotActionState.FAILED
                    return self.state

                self._result_future = self._goal_handle.get_result_async()
                self.context.node.get_logger().info(
                    f'[{self.context.robot_name}] Docking goal accepted'
                )

        # Wait for result
        if self._result_future is not None:
            if not self._result_future.done():
                self.state = RobotActionState.IN_PROGRESS
                return self.state

            try:
                res = self._result_future.result()
                status = getattr(res, 'status', None)
            except Exception as e:
                self.context.node.get_logger().error(
                    f'[{self.context.robot_name}] Docking result future failed: '
                    f'{type(e)}: {e}'
                )
                self.state = RobotActionState.FAILED
                return self.state

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.context.node.get_logger().info(
                    f'[{self.context.robot_name}] Docking succeeded'
                )
                self.state = RobotActionState.COMPLETED
                return self.state

            if status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
                self.context.node.get_logger().warn(
                    f'[{self.context.robot_name}] Docking ended with status [{status}]'
                )
                self.state = RobotActionState.FAILED
                return self.state

        # Update that the action is still in progress
        self.state = RobotActionState.IN_PROGRESS
        return self.state

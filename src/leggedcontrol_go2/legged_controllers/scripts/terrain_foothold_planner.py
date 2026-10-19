#!/usr/bin/env python3

"""Select conservative foothold heights from an Elevation Mapping GridMap.

The default is deliberately one-shot. In continuous mode, replanning is gated
on transitions into all-feet stance so a reference is never replaced mid-swing.
"""

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point32
from grid_map_msgs.msg import GridMap
from ocs2_msgs.msg import MpcObservation
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener
from std_msgs.msg import String, UInt8

from legged_terrain_msgs.msg import FootholdPlan


FOOT_NAMES = ("LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT")


def arguments():
    parser = argparse.ArgumentParser(description="Publish one map-derived Go2 foothold plan")
    parser.add_argument("--map-topic", default="/perceptive_terrain_map")
    parser.add_argument("--forward", type=float, default=0.18, help="nominal landing offset in base x [m]")
    parser.add_argument("--max-step-length", type=float, default=0.16, help="hard horizontal reachability limit [m]")
    parser.add_argument("--nominal-leg-extension", type=float, default=0.40,
                        help="maximum hip-to-foothold distance used by plane scoring [m]")
    parser.add_argument("--search-radius", type=float, default=0.12, help="candidate search radius [m]")
    parser.add_argument("--foot-radius", type=float, default=0.02, help="Go2 spherical contact-foot radius [m]")
    parser.add_argument("--foothold-margin", type=float, default=0.015,
                        help="additional inward margin for convex contact regions [m]")
    parser.add_argument("--max-region-half-size", type=float, default=0.16,
                        help="maximum convex-region growth from its nominal foothold [m]")
    # geo_trav is a cost: flat observed terrain is near 0.0 and hazards
    # approach 0.5.  Lower is safer.
    parser.add_argument("--max-traversability-cost", type=float, default=0.05,
                        help="maximum cost for an ordinary non-edge landing")
    parser.add_argument("--max-edge-traversability-cost", type=float, default=0.10,
                        help="maximum cost accepted only for a verified rising stair edge")
    parser.add_argument("--plateau-lookahead", type=float, default=0.12,
                        help="distance ahead used to verify a stair tread [m]")
    parser.add_argument("--min-stair-rise", type=float, default=0.025,
                        help="minimum rise that activates tread-height snapping [m]")
    parser.add_argument("--plateau-tolerance", type=float, default=0.012,
                        help="maximum height disagreement between tread samples [m]")
    parser.add_argument("--max-slope-deg", type=float, default=25.0)
    parser.add_argument("--max-roughness", type=float, default=0.025, help="height standard deviation [m]")
    parser.add_argument("--max-foot-height-range", type=float, default=0.015, help="maximum height variation under the foot [m]")
    parser.add_argument("--lateral-search", type=float, default=0.035, help="maximum lateral map search from foot lane [m]")
    parser.add_argument("--max-step", type=float, default=0.18, help="maximum up/down height change [m]")
    parser.add_argument("--clearance", type=float, default=0.10, help="clearance above highest endpoint [m]")
    parser.add_argument("--max-map-age", type=float, default=0.75)
    parser.add_argument("--max-yaw-drift-deg", type=float, default=15.0)
    parser.add_argument("--max-roll-pitch-deg", type=float, default=18.0,
                        help="fail-safe torso roll/pitch limit [deg]")
    parser.add_argument("--min-base-height", type=float, default=0.20,
                        help="fail-safe minimum MPC base height [m]")
    parser.add_argument("--travel-yaw-deg", type=float,
                        help="fixed odom-frame travel heading; defaults to startup yaw")
    parser.add_argument("--yaw-correction-gain", type=float, default=1.5,
                        help="autonomous heading correction gain [rad/s per rad]")
    parser.add_argument("--max-yaw-rate", type=float, default=0.20,
                        help="autonomous heading correction limit [rad/s]")
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--dry-run", action="store_true", help="print a complete plan without publishing it")
    parser.add_argument("--target-foot", choices=FOOT_NAMES,
                        help="validate/publish only this next swing foot in one-shot mode")
    parser.add_argument(
        "--continuous", action="store_true",
        help="replan once on every transition into all-feet stance (Ctrl+C to stop)",
    )
    parser.add_argument("--autonomous", action="store_true",
                        help="atomically plan, start stair_crawl, command velocity, and stop safely")
    parser.add_argument("--autonomous-speed", type=float, default=0.03,
                        help="forward cmd_vel used in autonomous mode [m/s]")
    parser.add_argument("--rear-catchup-speed-scale", type=float, default=0.25,
                        help="fraction of autonomous speed retained while rear feet catch up")
    parser.add_argument("--touchdown-tolerance", type=float, default=0.09)
    parser.add_argument("--touchdown-settle-time", type=float, default=0.8)
    parser.add_argument("--swing-transition-timeout", type=float, default=8.0,
                        help="fail-safe timeout while waiting for the expected swing mode [s]")
    parser.add_argument("--goal-height-rise", type=float, default=0.22)
    parser.add_argument("--observation-topic", default="/legged_robot_mpc_observation")
    parser.add_argument("--min-replan-period", type=float, default=0.30)
    # ROS 2 launch appends remapping arguments (for example
    # ``--ros-args -r __node:=...``).  Leave those for rclpy instead of
    # rejecting them in the planner-specific argument parser.
    args, _ros_args = parser.parse_known_args()
    return args


class Grid:
    def __init__(self, msg, received_monotonic=None):
        self.frame = msg.header.frame_id
        self.stamp = msg.header.stamp
        # The mapper may stamp with /clock while this one-shot process uses a
        # wall clock.  A monotonic receipt timestamp is therefore the only
        # reliable freshness measure across both clock domains.
        self.received_monotonic = time.monotonic() if received_monotonic is None else received_monotonic
        self.resolution = msg.info.resolution
        self.length_x = msg.info.length_x
        self.length_y = msg.info.length_y
        self.center_x = msg.info.pose.position.x
        self.center_y = msg.info.pose.position.y
        self.outer_start = int(msg.outer_start_index)
        self.inner_start = int(msg.inner_start_index)
        self.layers = {}
        for name, array in zip(msg.layers, msg.data):
            dims = array.layout.dim
            if len(dims) != 2:
                continue
            column_major = dims[0].label == "column_index"
            if column_major:
                rows, cols = int(dims[1].size), int(dims[0].size)
                matrix = np.asarray(array.data, dtype=np.float32).reshape((rows, cols), order="F")
            else:
                rows, cols = int(dims[0].size), int(dims[1].size)
                matrix = np.asarray(array.data, dtype=np.float32).reshape((rows, cols), order="C")
            # Undo GridMap's circular buffer so logical index (0, 0) is the
            # positive-x, positive-y corner of the published map geometry.
            matrix = np.roll(matrix, (-self.outer_start, -self.inner_start), axis=(0, 1))
            self.layers[name] = matrix
        if "elevation" not in self.layers:
            raise ValueError("map has no elevation layer")
        required = {
            "elevation_median", "planar", "plane_id",
            "paper_normal_x", "paper_normal_y", "paper_normal_z",
        }
        missing = sorted(required.difference(self.layers))
        if missing:
            raise ValueError(
                "terrain planner requires /perceptive_terrain_map; missing "
                f"paper layers: {', '.join(missing)}")
        # There is deliberately no raw-map fallback.  Footholds must use the
        # same classified and segmented terrain representation as the MPC.
        self.layers["elevation"] = self.layers["elevation_median"]
        self.layers["is_valid"] = self.layers["planar"]
        self.layers["geo_trav"] = 1.0 - np.clip(self.layers["planar"], 0.0, 1.0)
        self.layers["normal_x"] = self.layers["paper_normal_x"]
        self.layers["normal_y"] = self.layers["paper_normal_y"]
        self.layers["normal_z"] = self.layers["paper_normal_z"]
        self.rows, self.cols = self.layers["elevation"].shape

    def index(self, x, y):
        row = int(math.floor((self.center_x + self.length_x / 2.0 - x) / self.resolution))
        col = int(math.floor((self.center_y + self.length_y / 2.0 - y) / self.resolution))
        return row, col

    def position(self, row, col):
        x = self.center_x + self.length_x / 2.0 - (row + 0.5) * self.resolution
        y = self.center_y + self.length_y / 2.0 - (col + 0.5) * self.resolution
        return x, y

    def inside(self, row, col):
        return 0 <= row < self.rows and 0 <= col < self.cols


class TerrainFootholdPlanner(Node):
    def __init__(self, args):
        super().__init__("terrain_foothold_planner")
        self.args = args
        self.forward_offset = max(-args.max_step_length, min(args.forward, args.max_step_length))
        if self.forward_offset != args.forward:
            self.get_logger().warn(
                f"clamped --forward from {args.forward:.3f} to safe reach limit {self.forward_offset:.3f} m"
            )
        self.grid = None
        self.mode = None
        self.mpc_base_position = None
        self.mpc_base_velocity = None
        self.mpc_base_yaw = None
        self.mpc_base_roll = None
        self.mpc_base_pitch = None
        self.previous_mode = None
        self.planned_mode = None
        self.previous_planned_mode = None
        self.plan_pending = False
        self.terrain_gait_active = False
        self.next_swing_leg = 0
        self.commanded_speed = 0.0
        self.commanded_velocity = np.zeros(2)
        self.reference_yaw = (None if args.travel_yaw_deg is None
                              else math.radians(args.travel_yaw_deg))
        self.latest_yaw_error = 0.0
        self.autonomous_started = False
        self.last_autonomous_velocity_publish = -math.inf
        self.pause_autonomous_velocity = False
        # Ignore the initial stance stream until stair_crawl explicitly arms
        # the LF swing sequence.
        self.awaiting_swing = True
        self.nominal_lateral_offsets = [None] * 4
        self.last_map_targets = [None] * 4
        self.validation_pending_leg = None
        self.validation_ready_time = 0.0
        self.expected_swing_since = time.monotonic()
        self.initial_floor_height = None
        self.callback_group = ReentrantCallbackGroup()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            GridMap, args.map_topic, self.map_callback, qos,
            callback_group=self.callback_group)
        self.create_subscription(
            MpcObservation, args.observation_topic, self.observation_callback, 1,
            callback_group=self.callback_group)
        # The controller publishes this at its real-time update rate.  GridMap
        # deserialization can occupy this Python executor long enough for a
        # depth-1 subscription to lose a complete 0.55 s swing phase.
        mode_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            UInt8, "/legged_robot/planned_mode", self.planned_mode_callback, mode_qos,
            callback_group=self.callback_group)
        self.create_subscription(
            String, "/legged_robot_gait_command", self.gait_callback, 1,
            callback_group=self.callback_group)
        self.create_subscription(
            Twist, "/cmd_vel", self.velocity_callback, 1,
            callback_group=self.callback_group)
        self.publisher = self.create_publisher(FootholdPlan, "/legged_robot/foothold_plan", 1)
        self.velocity_publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.gait_publisher = self.create_publisher(String, "/legged_robot_gait_command", 1)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def map_callback(self, msg):
        try:
            self.grid = Grid(msg, time.monotonic())
        except ValueError as error:
            self.get_logger().error(str(error))

    def observation_callback(self, msg):
        self.mode = int(msg.mode)
        # Centroidal state layout: normalized momentum [0:6], followed by
        # base position xyz and ZYX Euler angles.  OCS2's horizontal world
        # origin differs from the simulator's absolute odom origin.
        if len(msg.state.value) >= 12:
            self.mpc_base_position = np.asarray(msg.state.value[6:9], dtype=float)
            self.mpc_base_yaw = float(msg.state.value[9])
            self.mpc_base_pitch = float(msg.state.value[10])
            self.mpc_base_roll = float(msg.state.value[11])
            self.mpc_base_velocity = np.asarray(msg.state.value[0:3], dtype=float)
        self.previous_mode = self.mode

    def planned_mode_callback(self, msg):
        self.planned_mode = int(msg.data)
        expected_swing_modes = (7, 11, 13, 14)  # LF, RF, LH, RH swing.
        if (self.terrain_gait_active and self.validation_pending_leg is None and
                self.planned_mode == expected_swing_modes[self.next_swing_leg]):
            self.awaiting_swing = False
        # The controller publishes at its real-time update rate while this
        # Python node spins much more slowly.  DDS can therefore coalesce the
        # samples between a swing and the following stance.  Once the expected
        # swing has been observed, accept the next observed stance even if the
        # immediately preceding sample was another mode.
        if (self.terrain_gait_active and self.validation_pending_leg is None and
                self.planned_mode == 15 and not self.awaiting_swing):
            completed_leg = self.next_swing_leg
            self.validation_pending_leg = completed_leg
            self.validation_ready_time = time.monotonic() + self.args.touchdown_settle_time
            self.next_swing_leg = (completed_leg + 1) % 4
            self.awaiting_swing = True
            self.expected_swing_since = time.monotonic()
            self.plan_pending = self.terrain_gait_active
            self.get_logger().info(
                f"{FOOT_NAMES[completed_leg]} swing completed; touchdown validation scheduled")
        self.previous_planned_mode = self.planned_mode

    def gait_callback(self, msg):
        self.terrain_gait_active = msg.data.strip().lower() == "stair_crawl"
        if self.terrain_gait_active:
            # stair_crawl starts with LF and always inserts a stance pause.
            self.next_swing_leg = 0
            self.awaiting_swing = True
            self.expected_swing_since = time.monotonic()
            self.validation_pending_leg = None
            self.plan_pending = self.planned_mode == 15
            self.get_logger().info("stair_crawl armed; next horizontal target is LF_FOOT")
        else:
            self.plan_pending = False

    def velocity_callback(self, msg):
        previous_speed = self.commanded_speed
        self.commanded_speed = math.hypot(msg.linear.x, msg.linear.y)
        self.commanded_velocity[:] = (msg.linear.x, msg.linear.y)
        if (self.terrain_gait_active and self.planned_mode == 15 and
                previous_speed <= 1e-3 < self.commanded_speed):
            if self.args.autonomous:
                return
            # The gait phase keeps cycling even while cmd_vel is zero.  Reset
            # the gait when motion actually begins so the planner and gait
            # scheduler both restart at LF instead of targeting different
            # legs after a long stationary wait.
            self.next_swing_leg = 0
            self.plan_pending = True
            self.gait_publisher.publish(String(data="stair_crawl"))
            self.get_logger().info("motion armed; restarted stair_crawl at LF_FOOT")

    def transform(self, target, source):
        return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time(), timeout=Duration(seconds=0.1))

    @staticmethod
    def yaw_from_transform(transform):
        q = transform.transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def layer_value(self, name, row, col, default):
        layer = self.grid.layers.get(name)
        if layer is None:
            return default
        return float(layer[row, col])

    def forward_plateau(self, x, y, direction_x, direction_y, lift_height, edge_height):
        """Return a verified higher tread height ahead of an interpolated edge."""
        sample_distances = (0.5 * self.args.plateau_lookahead, self.args.plateau_lookahead)
        heights = []
        for distance in sample_distances:
            row, col = self.grid.index(x + direction_x * distance, y + direction_y * distance)
            if not self.grid.inside(row, col):
                return None
            height = self.layer_value("elevation", row, col, math.nan)
            valid = self.layer_value("is_valid", row, col, 1.0)
            if not math.isfinite(height) or valid < 0.5:
                return None
            heights.append(height)
        if abs(heights[0] - heights[1]) > self.args.plateau_tolerance:
            return None
        plateau = float(np.median(heights))
        rise = plateau - lift_height
        if rise < self.args.min_stair_rise or rise > self.args.max_step:
            return None
        # The target centre must already be on the upper half of the mapped
        # transition.  This prevents snapping a point that is still before
        # the physical stair edge up to the tread height.
        if edge_height < lift_height + 0.5 * rise:
            return None
        return plateau

    def select(self, nominal_x, nominal_y, lift_height, origin_x, origin_y,
               direction_x, direction_y, hip_position=None):
        center_row, center_col = self.grid.index(nominal_x, nominal_y)
        search_cells = int(math.ceil(self.args.search_radius / self.grid.resolution))
        patch_cells = max(1, int(math.ceil(self.args.foot_radius / self.grid.resolution)))
        min_normal_z = math.cos(math.radians(self.args.max_slope_deg))
        best = None

        for row in range(center_row - search_cells, center_row + search_cells + 1):
            for col in range(center_col - search_cells, center_col + search_cells + 1):
                if not self.grid.inside(row, col):
                    continue
                x, y = self.grid.position(row, col)
                distance = math.hypot(x - nominal_x, y - nominal_y)
                if distance > self.args.search_radius:
                    continue
                if math.hypot(x - origin_x, nominal_y - origin_y) > self.args.max_step_length:
                    continue
                if abs(y - nominal_y) > max(self.args.lateral_search, 0.51 * self.grid.resolution):
                    continue
                elevation = self.layer_value("elevation", row, col, math.nan)
                valid = self.layer_value("is_valid", row, col, 1.0)
                traversability_cost = self.layer_value("geo_trav", row, col, math.inf)
                normal_z = self.layer_value("normal_z", row, col, 1.0)
                if (not math.isfinite(elevation) or valid < 0.5 or
                        traversability_cost > self.args.max_edge_traversability_cost or normal_z < min_normal_z or
                        abs(elevation - lift_height) > self.args.max_step):
                    continue

                landing_height = elevation
                landing_x, landing_y = x, y
                edge_landing = traversability_cost > self.args.max_traversability_cost
                if edge_landing:
                    landing_height = self.forward_plateau(
                        x, y, direction_x, direction_y, lift_height, elevation)
                    if landing_height is None:
                        continue
                    # The edge cell is useful for detecting the rise but is
                    # not a valid foot-centre target.  Move half the verified
                    # lookahead onto the upper tread; keeping x/y at the edge
                    # while snapping only z placed rear feet above the riser.
                    landing_x = x + 0.5 * self.args.plateau_lookahead * direction_x
                    landing_y = y + 0.5 * self.args.plateau_lookahead * direction_y
                    if math.hypot(
                            landing_x - origin_x,
                            landing_y - origin_y) > self.args.max_step_length:
                        continue
                    row, col = self.grid.index(landing_x, landing_y)
                    if not self.grid.inside(row, col):
                        continue
                    tread_height = self.layer_value("elevation", row, col, math.nan)
                    tread_valid = self.layer_value("is_valid", row, col, 1.0)
                    if (not math.isfinite(tread_height) or tread_valid < 0.5 or
                            abs(tread_height - landing_height) > self.args.plateau_tolerance):
                        continue

                r0, r1 = max(0, row - patch_cells), min(self.grid.rows, row + patch_cells + 1)
                c0, c1 = max(0, col - patch_cells), min(self.grid.cols, col + patch_cells + 1)
                patch = self.grid.layers["elevation"][r0:r1, c0:c1]
                # Evaluate a circular physical footprint, not the complete
                # square of neighbouring GridMap cells.  At the current 6 cm
                # map resolution the Go2's 2 cm spherical foot occupies one
                # cell; the previous 3x3 square represented an artificial
                # 18 cm foot and rejected every valid stair landing.
                row_offsets = (np.arange(r0, r1) - row) * self.grid.resolution
                col_offsets = (np.arange(c0, c1) - col) * self.grid.resolution
                footprint = (
                    row_offsets[:, np.newaxis] ** 2 + col_offsets[np.newaxis, :] ** 2
                    <= self.args.foot_radius ** 2 + 1e-12
                )
                finite = patch[footprint & np.isfinite(patch)]
                if finite.size == 0:
                    continue
                roughness = float(np.std(finite))
                height_range = float(np.max(finite) - np.min(finite))
                if roughness > self.args.max_roughness or height_range > self.args.max_foot_height_range:
                    continue
                kinematic_penalty = 0.0
                if hip_position is not None:
                    extension = math.sqrt(
                        (landing_x - hip_position[0]) ** 2 +
                        (landing_y - hip_position[1]) ** 2 +
                        (landing_height - hip_position[2]) ** 2)
                    if extension > self.args.nominal_leg_extension:
                        continue
                    # Smoothly discourage solutions close to full extension.
                    ratio = extension / self.args.nominal_leg_extension
                    kinematic_penalty = 0.02 * ratio ** 6
                distance = math.hypot(landing_x - nominal_x, landing_y - nominal_y)
                # A low-cost centre cell is not sufficient: the MPC needs a
                # non-empty convex support region around the commanded foot
                # centre.  Reject polygon-less candidates here so the search
                # can consider the next safe map cell instead of selecting one
                # candidate and rejecting the complete plan afterwards.
                #
                # Keep the foot in its nominal lateral lane, matching the
                # actual touchdown returned below.
                if self.convex_region(
                        landing_x, nominal_y, landing_height,
                        direction_x, direction_y) is None:
                    continue
                score = distance + 0.25 * roughness + 0.03 * traversability_cost + kinematic_penalty
                if edge_landing:
                    score += 0.01
                if best is None or score < best[0]:
                    # Keep the commanded foot exactly in its nominal lateral
                    # lane; y is used only to sample nearby map cells.
                    best = (
                        score, landing_x, nominal_y, landing_height,
                        traversability_cost, roughness)
        return best

    def select_tread_interior(self, lift, nominal_y, target_height,
                              direction_x, direction_y):
        """Find a directly observed upper-tread cell for split-level rear catch-up."""
        lateral_x, lateral_y = -direction_y, direction_x
        step = 0.5 * self.grid.resolution
        best = None
        forward = max(0.04, step)
        while forward <= self.args.max_step_length + 1e-9:
            lateral = -self.args.lateral_search
            while lateral <= self.args.lateral_search + 1e-9:
                x = lift.x + forward * direction_x + lateral * lateral_x
                y = lift.y + forward * direction_y + lateral * lateral_y
                row, col = self.grid.index(x, y)
                if self.grid.inside(row, col):
                    elevation = self.layer_value("elevation", row, col, math.nan)
                    valid = self.layer_value("is_valid", row, col, 1.0)
                    cost = self.layer_value("geo_trav", row, col, math.inf)
                    normal_z = self.layer_value("normal_z", row, col, 1.0)
                    if (math.isfinite(elevation) and valid >= 0.5 and
                            abs(elevation - target_height) <= max(
                                self.args.plateau_tolerance, 0.02) and
                            cost <= self.args.max_traversability_cost and
                            normal_z >= math.cos(math.radians(self.args.max_slope_deg))):
                        # Confirm that another cell farther into the tread has
                        # the same height.  This rejects the riser/edge cell
                        # even when interpolation marks it valid.
                        ahead_row, ahead_col = self.grid.index(
                            x + self.grid.resolution * direction_x,
                            y + self.grid.resolution * direction_y)
                        if self.grid.inside(ahead_row, ahead_col):
                            ahead = self.layer_value(
                                "elevation", ahead_row, ahead_col, math.nan)
                            ahead_valid = self.layer_value(
                                "is_valid", ahead_row, ahead_col, 1.0)
                            if (math.isfinite(ahead) and ahead_valid >= 0.5 and
                                    abs(ahead - elevation) <= self.args.plateau_tolerance):
                                score = abs(forward - max(0.12, self.forward_offset))
                                score += 0.25 * abs(lateral)
                                if best is None or score < best[0]:
                                    best = (
                                        score, x, nominal_y, elevation,
                                        cost, 0.0)
                lateral += step
            forward += step
        return best

    def region_cell_safe(self, x, y, plane_height, plane_id):
        """Paper steppability test, including the physical foot support patch."""
        row, col = self.grid.index(x, y)
        if not self.grid.inside(row, col):
            return False
        elevation = self.layer_value("elevation", row, col, math.nan)
        valid = self.layer_value("is_valid", row, col, 1.0)
        normal_z = self.layer_value("normal_z", row, col, 1.0)
        cost = self.layer_value("geo_trav", row, col, math.inf)
        cell_plane_id = self.layer_value("plane_id", row, col, -1.0)
        if (not math.isfinite(elevation) or valid < 0.5 or
                int(round(cell_plane_id)) != plane_id or
                abs(elevation - plane_height) > self.args.plateau_tolerance or
                normal_z < math.cos(math.radians(30.0)) or
                cost > self.args.max_traversability_cost):
            return False
        radius = self.args.foot_radius + self.args.foothold_margin
        cells = max(1, int(math.ceil(radius / self.grid.resolution)))
        values = []
        for rr in range(row - cells, row + cells + 1):
            for cc in range(col - cells, col + cells + 1):
                if not self.grid.inside(rr, cc):
                    return False
                px, py = self.grid.position(rr, cc)
                if math.hypot(px - x, py - y) > radius + 0.51 * self.grid.resolution:
                    continue
                h = self.layer_value("elevation", rr, cc, math.nan)
                v = self.layer_value("is_valid", rr, cc, 1.0)
                support_plane_id = self.layer_value("plane_id", rr, cc, -1.0)
                if (not math.isfinite(h) or v < 0.5 or
                        int(round(support_plane_id)) != plane_id):
                    return False
                values.append(h)
        return bool(values) and max(values) - min(values) <= self.args.max_foot_height_range

    def convex_region(self, x, y, z, direction_x, direction_y):
        """Grow a convex inner rectangle inside one observed planar component.

        This is the four-vertex form of the paper's round-robin polygon growth:
        every proposed expansion is accepted only when all newly covered map
        cells pass the steppability and same-plane tests.
        """
        lateral_x, lateral_y = -direction_y, direction_x
        seed_row, seed_col = self.grid.index(x, y)
        if not self.grid.inside(seed_row, seed_col):
            return None
        plane_value = self.layer_value("plane_id", seed_row, seed_col, -1.0)
        if not math.isfinite(plane_value) or plane_value < 0.0:
            return None
        plane_id = int(round(plane_value))
        # Seed with a half-cell-wide support region.  A full-cell seed plus
        # region_cell_safe()'s physical foot margin double-counted clearance
        # at a stair edge and made the last safe lower-floor approach cell
        # impossible.  Subsequent round-robin growth still uses this same
        # steppability test and expands as far as observed terrain permits.
        step = max(0.25 * self.grid.resolution, 0.015)
        extents = [step, step, step, step]  # forward, back, left, right
        limit = self.args.max_region_half_size

        def world(u, v):
            return (x + u * direction_x + v * lateral_x,
                    y + u * direction_y + v * lateral_y)

        def rectangle_safe(candidate):
            front, back, left, right = candidate
            du = max(0.5 * self.grid.resolution, 0.02)
            dv = du
            u = -back
            while u <= front + 1e-9:
                v = -right
                while v <= left + 1e-9:
                    wx, wy = world(u, v)
                    if not self.region_cell_safe(wx, wy, z, plane_id):
                        return False
                    v += dv
                u += du
            return True

        if not rectangle_safe(extents):
            return None
        changed = True
        while changed:
            changed = False
            for side in range(4):
                proposal = list(extents)
                proposal[side] += step
                if proposal[side] <= limit and rectangle_safe(proposal):
                    extents = proposal
                    changed = True

        front, back, left, right = extents
        local_corners = ((-back, -right), (front, -right),
                         (front, left), (-back, left))
        polygon = []
        for u, v in local_corners:
            wx, wy = world(u, v)
            row, col = self.grid.index(wx, wy)
            wz = self.layer_value("elevation", row, col, z)
            point = Point32()
            point.x, point.y, point.z = float(wx), float(wy), float(wz)
            polygon.append(point)

        row, col = self.grid.index(x, y)
        nx = self.layer_value("normal_x", row, col, 0.0)
        ny = self.layer_value("normal_y", row, col, 0.0)
        nz = self.layer_value("normal_z", row, col, 1.0)
        norm = math.sqrt(nx * nx + ny * ny + nz * nz)
        if norm < 1e-6:
            nx, ny, nz, norm = 0.0, 0.0, 1.0, 1.0
        return polygon, (nx / norm, ny / norm, nz / norm)

    def swing_apex_height(self, start, end):
        """Clear the highest observed terrain along the complete swing chord."""
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        samples = max(2, int(math.ceil(distance / (0.5 * self.grid.resolution))))
        highest = max(start[2], end[2])
        for i in range(samples + 1):
            alpha = i / samples
            x = (1.0 - alpha) * start[0] + alpha * end[0]
            y = (1.0 - alpha) * start[1] + alpha * end[1]
            row, col = self.grid.index(x, y)
            if not self.grid.inside(row, col):
                continue
            height = self.layer_value("elevation", row, col, math.nan)
            if math.isfinite(height):
                highest = max(highest, height)
        # Protect the optimizer from isolated map spikes, as in the reference
        # implementation's bounded swing-height adaptation.
        highest = min(highest, max(start[2], end[2]) + 2.0 * self.args.clearance)
        return highest + self.args.clearance

    def check_base_safety(self):
        if self.mpc_base_position is None or self.mpc_base_yaw is None:
            raise RuntimeError("MPC observation has no base pose yet")
        if self.mpc_base_position[2] < self.args.min_base_height:
            raise RuntimeError(
                f"base height {self.mpc_base_position[2]:.3f} m is below "
                f"safety limit {self.args.min_base_height:.3f} m")
        attitude_limit = math.radians(self.args.max_roll_pitch_deg)
        if (self.mpc_base_roll is None or self.mpc_base_pitch is None or
                abs(self.mpc_base_roll) > attitude_limit or
                abs(self.mpc_base_pitch) > attitude_limit):
            roll_deg = math.degrees(self.mpc_base_roll or 0.0)
            pitch_deg = math.degrees(self.mpc_base_pitch or 0.0)
            raise RuntimeError(
                f"base attitude roll={roll_deg:.1f} pitch={pitch_deg:.1f} deg "
                f"exceeds safety limit")

    def make_plan(self):
        if self.grid is None:
            return None
        age = time.monotonic() - self.grid.received_monotonic
        if age > self.args.max_map_age:
            raise RuntimeError(f"elevation map is stale ({age:.2f} s)")

        base = self.transform(self.grid.frame, "base")
        self.check_base_safety()
        yaw = self.yaw_from_transform(base)
        if self.reference_yaw is None:
            self.reference_yaw = yaw
        yaw_error = math.atan2(math.sin(yaw - self.reference_yaw), math.cos(yaw - self.reference_yaw))
        self.latest_yaw_error = yaw_error
        if abs(yaw_error) > math.radians(self.args.max_yaw_drift_deg):
            raise RuntimeError(f"base yaw drift {math.degrees(yaw_error):.1f} deg exceeds safety limit")
        dx = self.forward_offset * math.cos(self.reference_yaw)
        dy = self.forward_offset * math.sin(self.reference_yaw)
        direction = np.array([
            math.cos(self.reference_yaw), math.sin(self.reference_yaw)])
        # Eq. (12): reactive inverted-pendulum correction on the first
        # upcoming foothold.  State [0:3] is normalized linear momentum
        # (CoM velocity) in this centroidal interface.
        if self.mpc_base_velocity is not None:
            pendulum_time = math.sqrt(max(0.05, 0.30) / 9.81)
            commanded_world = np.array([
                math.cos(yaw) * self.commanded_velocity[0] - math.sin(yaw) * self.commanded_velocity[1],
                math.sin(yaw) * self.commanded_velocity[0] + math.cos(yaw) * self.commanded_velocity[1],
            ])
            velocity_error = self.mpc_base_velocity[:2] - commanded_world
            # The centroidal state exposes normalized momentum, not a clean
            # Cartesian base-velocity measurement.  Its lateral component
            # contains gait-phase momentum and previously drove a nominally
            # forward crawl sideways.  Retain only the correction projected
            # onto the locked travel heading.
            forward_correction = float(np.clip(
                pendulum_time * np.dot(velocity_error, direction),
                -0.04, 0.04))
            dx += forward_correction * direction[0]
            dy += forward_correction * direction[1]

        self.pause_autonomous_velocity = False
        plan = FootholdPlan()
        plan.header.stamp = self.get_clock().now().to_msg()
        plan.header.frame_id = self.grid.frame
        plan.foot_names = list(FOOT_NAMES)
        for index, foot_name in enumerate(FOOT_NAMES):
            foot = self.transform(self.grid.frame, foot_name).transform.translation
            hip = self.transform(self.grid.frame, foot_name.replace("_FOOT", "_hip")).transform.translation
            direction_x = math.cos(self.reference_yaw)
            direction_y = math.sin(self.reference_yaw)
            lateral_x = -direction_y
            lateral_y = direction_x
            # Lock each foot to its original lane in the global travel frame.
            # A base-relative offset lets a small lateral base error move all
            # four lanes on every cycle, producing cumulative diagonal drift.
            current_lateral = foot.x * lateral_x + foot.y * lateral_y
            if self.nominal_lateral_offsets[index] is None:
                self.nominal_lateral_offsets[index] = current_lateral
            lateral_correction = float(np.clip(
                self.nominal_lateral_offsets[index] - current_lateral,
                -0.03, 0.03))
            nominal_x = foot.x + dx + lateral_correction * lateral_x
            nominal_y = foot.y + dy + lateral_correction * lateral_y
            lift_row, lift_col = self.grid.index(foot.x, foot.y)
            # The TF foot origin is at the centre of the spherical contact.
            # Its bottom is a direct measurement of the current support
            # surface and is especially useful when the map cell beneath a
            # foot lies on an interpolated stair edge.
            measured_support_height = foot.z - self.args.foot_radius
            lift_height = measured_support_height
            if self.grid.inside(lift_row, lift_col):
                mapped_lift = self.layer_value("elevation", lift_row, lift_col, math.nan)
                if math.isfinite(mapped_lift):
                    lift_height = max(mapped_lift, measured_support_height)
            if self.initial_floor_height is None:
                self.initial_floor_height = lift_height
            # Record the measured support state even if no new forward
            # landing candidate exists. It allows an elevated front foot to
            # be held safely in place while the rear pair catches up.
            plan.lift_off_positions[index].x = foot.x
            plan.lift_off_positions[index].y = foot.y
            plan.lift_off_positions[index].z = lift_height
            candidate = self.select(
                nominal_x, nominal_y, lift_height, foot.x, foot.y,
                direction_x, direction_y,
                (hip.x + 0.5 * dx, hip.y + 0.5 * dy, hip.z))
            if candidate is None:
                continue
            _, x, y, z, traversability_cost, roughness = candidate
            plan.touch_down_positions[index].x = x
            plan.touch_down_positions[index].y = y
            plan.touch_down_positions[index].z = z
            plan.minimum_apex_heights[index] = self.swing_apex_height(
                (foot.x, foot.y, lift_height), (x, y, z))
            plan.valid[index] = True
            region = self.convex_region(x, y, z, direction_x, direction_y)
            if region is not None:
                polygon, normal = region
                plan.plane_origins[index].x = x
                plan.plane_origins[index].y = y
                plan.plane_origins[index].z = z
                plan.plane_normals[index].x = normal[0]
                plan.plane_normals[index].y = normal[1]
                plan.plane_normals[index].z = normal[2]
                plan.convex_regions[index].points = polygon
                plan.region_valid[index] = True
                plan.horizontal_valid[index] = True
            self.get_logger().info(
                f"{foot_name}: ({x:.3f}, {y:.3f}, {z:.3f}), "
                f"trav_cost={traversability_cost:.3f}, roughness={roughness:.3f}, "
                f"region_vertices={len(plan.convex_regions[index].points)}"
            )

        if self.args.autonomous:
            current_front_height = min(
                plan.lift_off_positions[0].z, plan.lift_off_positions[1].z)
            current_rear_height = min(
                plan.lift_off_positions[2].z, plan.lift_off_positions[3].z)
            if current_front_height - current_rear_height > 0.025:
                for index in (0, 1):
                    if plan.valid[index]:
                        continue
                    lift = plan.lift_off_positions[index]
                    touch = plan.touch_down_positions[index]
                    touch.x = lift.x
                    touch.y = lift.y
                    touch.z = lift.z
                    plan.minimum_apex_heights[index] = lift.z + self.args.clearance
                    plan.valid[index] = True
                    plan.horizontal_valid[index] = True
                    self.get_logger().info(
                        f"{FOOT_NAMES[index]}: using elevated in-place hold target")

        if self.args.continuous:
            if not (plan.valid[self.next_swing_leg] and
                    plan.region_valid[self.next_swing_leg]):
                rear_approach_support = plan.valid[2] and plan.valid[3]
                if (not self.args.autonomous or
                        (sum(plan.valid) < 3 and not rear_approach_support)):
                    raise RuntimeError(
                        f"safe plan rejected because next foot {FOOT_NAMES[self.next_swing_leg]} "
                        "has no convex safe region")
                # At a riser, the first interior upper-tread cell can be a few
                # centimetres beyond the swing foot's reach even though the
                # other three feet have safe same-level approach targets.
                # Hold this foot for one crawl cycle while the guarded body
                # velocity closes that distance; never invent an elevated
                # target at the edge.
                index = self.next_swing_leg
                lift = plan.lift_off_positions[index]
                touch = plan.touch_down_positions[index]
                touch.x, touch.y, touch.z = lift.x, lift.y, lift.z
                plan.minimum_apex_heights[index] = lift.z + self.args.clearance
                plan.valid[index] = True
                self.get_logger().warn(
                    f"{FOOT_NAMES[index]} upper tread is not reachable yet; "
                    "using one guarded in-place approach cycle")
            if self.args.autonomous:
                # A plan is consumed across the whole LF/RF/LH/RH crawl
                # cycle.  Never leave a later leg invalid: it would otherwise
                # retain an old target while the touchdown state machine still
                # validates it.  Holding that foot at its measured support is
                # safe, and it can receive a new forward candidate next cycle.
                for index in range(4):
                    if plan.valid[index]:
                        continue
                    lift = plan.lift_off_positions[index]
                    touch = plan.touch_down_positions[index]
                    touch.x = lift.x
                    touch.y = lift.y
                    touch.z = lift.z
                    plan.minimum_apex_heights[index] = lift.z + self.args.clearance
                    plan.valid[index] = True
                    self.get_logger().warn(
                        f"{FOOT_NAMES[index]} has no current map candidate; "
                        "using an in-place hold for this crawl cycle")
                # MPC needs the complete crawl cycle early enough for its
                # asynchronous policy update.  Per-stance publications leave
                # only ~0.1 s before later swings and arrive too late.
                plan.horizontal_valid = list(plan.valid)
            else:
                # A non-autonomous continuous update belongs to exactly one
                # upcoming swing.  Mask the complete plan, not only its XY
                # target: valid feet also modify swing height, support
                # regions, and terrain torso references in the reference
                # manager.  Republishing all four as valid at every stance
                # reset unrelated contact schedules and made the body sink
                # immediately after the second (RF) update.
                for index in range(4):
                    active = index == self.next_swing_leg
                    plan.valid[index] = active
                    plan.horizontal_valid[index] = active
                    plan.region_valid[index] = active
                    if not active:
                        plan.convex_regions[index].points = []
        elif self.args.target_foot:
            target_index = FOOT_NAMES.index(self.args.target_foot)
            if not (plan.valid[target_index] and plan.region_valid[target_index]):
                raise RuntimeError(
                    f"safe plan rejected because target foot {self.args.target_foot} "
                    "has no convex safe region")
            # A preload is deliberately a single-foot message.  Leaving the
            # other feet valid/region-valid lets the reference manager install
            # their height and region constraints even when horizontal_valid
            # is false.  During the stance-to-crawl transition those latent
            # constraints can produce a four-leg correction before the gait
            # scheduler reaches the requested foot.
            for index in range(4):
                active = index == target_index
                plan.valid[index] = active
                plan.horizontal_valid[index] = active
                plan.region_valid[index] = active
                if not active:
                    plan.convex_regions[index].points = []
        elif not (all(plan.valid) and all(plan.region_valid)):
            raise RuntimeError(
                "safe plan rejected because at least one foot has no convex safe region")

        if self.args.autonomous and all(plan.valid):
            front_pair_catchup = False
            front_lift_heights = [
                plan.lift_off_positions[0].z,
                plan.lift_off_positions[1].z,
            ]
            if abs(front_lift_heights[0] - front_lift_heights[1]) > 0.025:
                front_pair_catchup = True
                self.pause_autonomous_velocity = True
                higher = 0 if front_lift_heights[0] > front_lift_heights[1] else 1
                lower = 1 - higher
                target_height = front_lift_heights[higher]
                # Freeze the elevated front foot and both rear supports.  Only
                # the lower front foot may move until the front pair shares
                # the same tread; advancing the elevated front diagonally
                # twists the support polygon and rolls the robot.
                for index in (higher, 2, 3):
                    lift = plan.lift_off_positions[index]
                    touch = plan.touch_down_positions[index]
                    touch.x, touch.y, touch.z = lift.x, lift.y, lift.z
                    plan.minimum_apex_heights[index] = (
                        lift.z + self.args.clearance)
                lift = plan.lift_off_positions[lower]
                touch = plan.touch_down_positions[lower]
                candidate = self.select_tread_interior(
                    lift, touch.y, target_height, direction_x, direction_y)
                if candidate is not None:
                    _, x, y, z, _, _ = candidate
                    touch.x, touch.y, touch.z = x, y, z
                    plan.minimum_apex_heights[lower] = (
                        max(lift.z, z) + self.args.clearance)
                    self.get_logger().info(
                        f"{FOOT_NAMES[lower]}: selected front-pair "
                        "upper-tread catch-up target")
                else:
                    touch.x, touch.y, touch.z = lift.x, lift.y, lift.z
                    plan.minimum_apex_heights[lower] = (
                        lift.z + self.args.clearance)
                    self.get_logger().warn(
                        f"{FOOT_NAMES[lower]} upper tread is not reachable yet; "
                        "holding all feet during guarded approach")
                self.get_logger().info(
                    "asymmetric front support detected; synchronizing front pair")

            # Hold the front pair only after both front feet are actually
            # supported on a higher tread.  Comparing planned touchdown
            # heights here triggered the hold before the first front swing,
            # leaving every foot on the floor while asking the rear pair to
            # "catch up".
            front_height = min(plan.lift_off_positions[0].z, plan.lift_off_positions[1].z)
            # Both rear feet must be on the tread before forward body motion
            # resumes. Using max() released the hold as soon as only one rear
            # foot climbed and pulled the other foot away from its target.
            rear_height = min(plan.lift_off_positions[2].z, plan.lift_off_positions[3].z)
            if not front_pair_catchup and front_height - rear_height > 0.025:
                self.pause_autonomous_velocity = True
                # Do not keep extending the front pair along a tread while the
                # rear pair is still on the lower support surface. Swing the
                # fronts in place for this crawl cycle and let the rears close
                # the fore-aft gap first.
                for index in (0, 1):
                    plan.touch_down_positions[index].x = plan.lift_off_positions[index].x
                    plan.touch_down_positions[index].y = plan.lift_off_positions[index].y
                    plan.touch_down_positions[index].z = plan.lift_off_positions[index].z
                    plan.minimum_apex_heights[index] = (
                        plan.lift_off_positions[index].z + self.args.clearance)
                # Split a large rear correction into short, repeatable crawl
                # steps. Rear tracking becomes unreliable when catch-up asks
                # for the complete nominal stride in a single swing.
                max_rear_catchup = 0.07
                for index in (2, 3):
                    lift = plan.lift_off_positions[index]
                    touch = plan.touch_down_positions[index]
                    progress = ((touch.x - lift.x) * direction_x +
                                (touch.y - lift.y) * direction_y)
                    # Near a riser the closest-cell score can repeatedly pick
                    # the current lower floor cell.  Search toward the full
                    # reachable stride so the rear foot can select the
                    # verified upper tread instead of crawling in place.
                    if progress < 0.02 and touch.z < front_height - 0.025:
                        catchup_distance = min(
                            self.args.max_step_length - 0.01,
                            max(0.12, self.forward_offset))
                        candidate = self.select(
                            lift.x + catchup_distance * direction_x,
                            lift.y + catchup_distance * direction_y,
                            lift.z, lift.x, lift.y, direction_x, direction_y)
                        if candidate is not None:
                            _, x, y, z, _, _ = candidate
                            touch.x, touch.y, touch.z = x, y, z
                            plan.minimum_apex_heights[index] = (
                                max(lift.z, z) + self.args.clearance)
                    if touch.z < front_height - 0.025:
                        candidate = self.select_tread_interior(
                            lift, touch.y, front_height,
                            direction_x, direction_y)
                        if candidate is not None:
                            _, x, y, z, _, _ = candidate
                            touch.x, touch.y, touch.z = x, y, z
                            plan.minimum_apex_heights[index] = (
                                max(lift.z, z) + self.args.clearance)
                            self.get_logger().info(
                                f"{FOOT_NAMES[index]}: selected verified "
                                "upper-tread catch-up target")

                    dx = touch.x - lift.x
                    dy = touch.y - lift.y
                    distance = math.hypot(dx, dy)
                    height_change = touch.z - lift.z
                    # Never shorten x/y while retaining an elevated z target:
                    # that places touchdown vertically above the riser face
                    # and is the source of rear-foot sliding.  Elevated,
                    # reach-checked tread candidates must be taken whole.
                    if abs(height_change) < self.args.min_stair_rise and distance > max_rear_catchup:
                        scale = max_rear_catchup / distance
                        touch.x = lift.x + scale * dx
                        touch.y = lift.y + scale * dy
                self.get_logger().info(
                    "front pair held in place while rear feet catch up to the current tread")

        map_targets = [None] * 4
        map_lift_offs = [None] * 4
        for index, active in enumerate(plan.horizontal_valid):
            if active and plan.valid[index]:
                lift = plan.lift_off_positions[index]
                point = plan.touch_down_positions[index]
                map_lift_offs[index] = (float(lift.x), float(lift.y), float(lift.z))
                map_targets[index] = (float(point.x), float(point.y), float(point.z))

        # Footholds are selected in the elevation map's odom frame, whereas
        # Pinocchio/OCS2 uses a local world origin established at startup.
        # Express every valid point in that local MPC world before publishing.
        odom_base = base.transform.translation
        rotation = self.mpc_base_yaw - yaw
        cos_rotation, sin_rotation = math.cos(rotation), math.sin(rotation)
        for index in range(4):
            if not plan.valid[index]:
                continue
            for point in (plan.lift_off_positions[index], plan.touch_down_positions[index]):
                relative_x = point.x - odom_base.x
                relative_y = point.y - odom_base.y
                point.x = self.mpc_base_position[0] + cos_rotation * relative_x - sin_rotation * relative_y
                point.y = self.mpc_base_position[1] + sin_rotation * relative_x + cos_rotation * relative_y
                point.z = self.mpc_base_position[2] + point.z - odom_base.z
            if plan.region_valid[index]:
                origin = plan.plane_origins[index]
                relative_x = origin.x - odom_base.x
                relative_y = origin.y - odom_base.y
                origin.x = self.mpc_base_position[0] + cos_rotation * relative_x - sin_rotation * relative_y
                origin.y = self.mpc_base_position[1] + sin_rotation * relative_x + cos_rotation * relative_y
                origin.z = self.mpc_base_position[2] + origin.z - odom_base.z
                normal = plan.plane_normals[index]
                normal_x = cos_rotation * normal.x - sin_rotation * normal.y
                normal_y = sin_rotation * normal.x + cos_rotation * normal.y
                normal.x, normal.y = normal_x, normal_y
                for vertex in plan.convex_regions[index].points:
                    relative_x = vertex.x - odom_base.x
                    relative_y = vertex.y - odom_base.y
                    vertex.x = self.mpc_base_position[0] + cos_rotation * relative_x - sin_rotation * relative_y
                    vertex.y = self.mpc_base_position[1] + sin_rotation * relative_x + cos_rotation * relative_y
                    vertex.z = self.mpc_base_position[2] + vertex.z - odom_base.z
            plan.minimum_apex_heights[index] = (
                self.mpc_base_position[2] + plan.minimum_apex_heights[index] - odom_base.z)
        plan.header.frame_id = "mpc_world"
        for index, active in enumerate(plan.horizontal_valid):
            if active:
                self.last_map_targets[index] = map_targets[index]
                lift = plan.lift_off_positions[index]
                target = plan.touch_down_positions[index]
                odom_lift = map_lift_offs[index]
                odom_target = map_targets[index]
                odom_dx = odom_target[0] - odom_lift[0]
                odom_dy = odom_target[1] - odom_lift[1]
                mpc_dx = target.x - lift.x
                mpc_dy = target.y - lift.y
                self.get_logger().info(
                    f"{FOOT_NAMES[index]} frame conversion: "
                    f"odom lift=({odom_lift[0]:.3f},{odom_lift[1]:.3f}) "
                    f"target=({odom_target[0]:.3f},{odom_target[1]:.3f}) "
                    f"delta=({odom_dx:+.3f},{odom_dy:+.3f}); "
                    f"MPC lift=({lift.x:.3f},{lift.y:.3f}) "
                    f"target=({target.x:.3f},{target.y:.3f}) "
                    f"delta=({mpc_dx:+.3f},{mpc_dy:+.3f})")
        return plan

    def verify_touchdown(self, leg):
        target = self.last_map_targets[leg]
        if target is None:
            # A continuous planner can be started after a guarded one-shot
            # preload from another process.  Its first observed swing is then
            # valid controller synchronization, but the target is not present
            # in this process-local cache.  Skip validation for that one
            # synchronization swing; the next plan is generated and cached by
            # this node, so every later touchdown remains strictly validated.
            self.get_logger().warn(
                f"{FOOT_NAMES[leg]} was externally preloaded; "
                "accepting first swing as continuous-planner synchronization")
            return
        foot = self.transform(self.grid.frame, FOOT_NAMES[leg]).transform.translation
        error = math.hypot(foot.x - target[0], foot.y - target[1])
        if error > self.args.touchdown_tolerance:
            surface_height = foot.z - self.args.foot_radius
            height_error = abs(surface_height - target[2])
            # A crawl foot can settle a few centimetres behind the horizontal
            # spline target because the body continues moving during the
            # stance pause.  Accept that only when it is demonstrably on the
            # requested support surface; the height check below still rejects
            # a foot left on the lower stair.
            support_fallback_limit = self.args.touchdown_tolerance + 0.05
            if (self.initial_floor_height is not None and
                    target[2] <= self.initial_floor_height + 0.025):
                # Allow estimator/map discretization on the flat approach,
                # but do not accept a foot that remained near its old support.
                # The former 0.18 m relaxation let a 0.10 m command repeatedly
                # "succeed" without forward progress.
                support_fallback_limit = min(
                    support_fallback_limit,
                    self.args.touchdown_tolerance + 0.03)
            if error > support_fallback_limit or height_error > 0.025:
                raise RuntimeError(
                    f"{FOOT_NAMES[leg]} missed touchdown: horizontal={error:.3f} m, "
                    f"surface-height={height_error:.3f} m")
            self.get_logger().warn(
                f"{FOOT_NAMES[leg]} touchdown accepted by support height: "
                f"horizontal={error:.3f} m, surface-height={height_error:.3f} m")
            return
        self.get_logger().info(
            f"{FOOT_NAMES[leg]} touchdown verified: horizontal error={error:.3f} m")

    def all_feet_on_goal_plateau(self):
        if self.initial_floor_height is None:
            return False
        threshold = self.initial_floor_height + self.args.goal_height_rise
        for foot_name in FOOT_NAMES:
            foot = self.transform(self.grid.frame, foot_name).transform.translation
            row, col = self.grid.index(foot.x, foot.y)
            if not self.grid.inside(row, col):
                return False
            height = self.layer_value("elevation", row, col, math.nan)
            if not math.isfinite(height) or height < threshold:
                return False
        return True

    def publish_plan(self, plan, copies=5):
        for _ in range(copies):
            plan.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(plan)
            time.sleep(0.1)

    def fail_safe_stop(self):
        stop = Twist()
        stop.linear.z = 0.30
        stance = String(data="stance")
        clear = FootholdPlan()
        clear.header.frame_id = self.grid.frame if self.grid is not None else "odom"
        clear.foot_names = list(FOOT_NAMES)
        for _ in range(3):
            clear.header.stamp = self.get_clock().now().to_msg()
            self.velocity_publisher.publish(stop)
            self.gait_publisher.publish(stance)
            self.publisher.publish(clear)
            time.sleep(0.05)
        self.get_logger().error("FAIL-SAFE: commanded zero velocity, stance, and cleared foothold overrides")

    def complete_stop(self):
        stop = Twist()
        stop.linear.z = 0.30
        stance = String(data="stance")
        clear = FootholdPlan()
        clear.header.frame_id = "mpc_world"
        clear.foot_names = list(FOOT_NAMES)
        for _ in range(3):
            clear.header.stamp = self.get_clock().now().to_msg()
            self.velocity_publisher.publish(stop)
            self.gait_publisher.publish(stance)
            self.publisher.publish(clear)
            time.sleep(0.05)
        self.get_logger().info("AUTONOMOUS COMPLETE: top plateau reached; commanded stance")

    def start_autonomous_motion(self):
        """Start the gait only after its first foothold plan is installed."""
        gait = String(data="stair_crawl")
        command = Twist()
        speed_scale = (
            max(0.0, min(1.0, self.args.rear_catchup_speed_scale))
            if self.pause_autonomous_velocity else 1.0)
        command.linear.x = max(0.0, self.args.autonomous_speed) * speed_scale
        command.linear.z = 0.30
        command.angular.z = max(
            -self.args.max_yaw_rate,
            min(self.args.max_yaw_rate, -self.args.yaw_correction_gain * self.latest_yaw_error))
        self.gait_publisher.publish(gait)
        time.sleep(0.10)
        self.velocity_publisher.publish(command)
        self.last_autonomous_velocity_publish = time.monotonic()
        self.autonomous_started = True
        self.awaiting_swing = True
        self.expected_swing_since = time.monotonic()
        self.plan_pending = False
        self.get_logger().warn(
            f"AUTONOMOUS START: first plan installed before stair_crawl; "
            f"forward speed={command.linear.x:.3f} m/s")

    def refresh_autonomous_velocity(self):
        """Advance the finite MPC pose target while autonomous travel is active."""
        command = Twist()
        speed_scale = (
            max(0.0, min(1.0, self.args.rear_catchup_speed_scale))
            if self.pause_autonomous_velocity else 1.0)
        command.linear.x = max(0.0, self.args.autonomous_speed) * speed_scale
        command.linear.z = 0.30
        command.angular.z = max(
            -self.args.max_yaw_rate,
            min(self.args.max_yaw_rate, -self.args.yaw_correction_gain * self.latest_yaw_error))
        self.velocity_publisher.publish(command)
        self.last_autonomous_velocity_publish = time.monotonic()


def main():
    args = arguments()
    if args.autonomous:
        args.continuous = True
    rclpy.init(args=[])
    node = TerrainFootholdPlanner(args)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    deadline = time.monotonic() + args.timeout
    next_attempt = 0.0
    last_publish = -math.inf
    exit_code = 1
    try:
        while rclpy.ok() and (args.continuous or time.monotonic() < deadline):
            executor.spin_once(timeout_sec=0.1)
            now = time.monotonic()
            if args.autonomous and node.autonomous_started:
                try:
                    node.check_base_safety()
                except RuntimeError as error:
                    node.get_logger().error(str(error))
                    node.fail_safe_stop()
                    node.autonomous_started = False
                    return 1
            if (args.autonomous and node.autonomous_started and
                    now - node.last_autonomous_velocity_publish >= 0.5):
                node.refresh_autonomous_velocity()
            if node.grid is None or now < next_attempt:
                continue
            if node.reference_yaw is None:
                try:
                    node.reference_yaw = node.yaw_from_transform(node.transform(node.grid.frame, "base"))
                    node.get_logger().info(f"locked autonomous travel yaw at {math.degrees(node.reference_yaw):.1f} deg")
                except TransformException:
                    continue
            if args.continuous:
                if node.planned_mode is None:
                    continue
                if args.autonomous and not node.autonomous_started:
                    if node.planned_mode != 15:
                        continue
                    node.next_swing_leg = 0
                    node.plan_pending = True
                else:
                    if not node.terrain_gait_active:
                        continue
                    if not args.autonomous and node.commanded_speed <= 1e-3:
                        continue
                if (node.autonomous_started and node.awaiting_swing and
                        node.validation_pending_leg is None):
                    if now - node.expected_swing_since > args.swing_transition_timeout:
                        node.get_logger().error(
                            f"expected {FOOT_NAMES[node.next_swing_leg]} swing was not observed "
                            f"within {args.swing_transition_timeout:.1f} s")
                        node.fail_safe_stop()
                        node.autonomous_started = False
                        return 1
                    continue
                if not node.plan_pending or node.planned_mode != 15:
                    continue
                if now - last_publish < args.min_replan_period:
                    continue
                if node.validation_pending_leg is not None:
                    if now < node.validation_ready_time:
                        continue
                    try:
                        node.verify_touchdown(node.validation_pending_leg)
                        node.validation_pending_leg = None
                        if args.autonomous and node.all_feet_on_goal_plateau():
                            node.complete_stop()
                            node.autonomous_started = False
                            return 0
                        if args.autonomous and node.next_swing_leg != 0:
                            # Remaining targets for this LF/RF/LH/RH cycle
                            # were installed together before LF started.
                            node.plan_pending = False
                            continue
                    except (TransformException, RuntimeError) as error:
                        node.get_logger().warn(str(error))
                        if not args.dry_run:
                            node.fail_safe_stop()
                        node.plan_pending = False
                        if args.autonomous:
                            # A guarded autonomous run is terminal after a
                            # failed touchdown.  Remaining alive here leaves a
                            # stale command publisher that can interfere with
                            # the next simulation or /clock epoch.
                            node.autonomous_started = False
                            return 1
                        continue
            next_attempt = now + 0.5
            try:
                plan = node.make_plan()
                if plan is not None:
                    if args.dry_run:
                        node.get_logger().info("Dry run succeeded; no foothold plan was published.")
                    else:
                        # Continuous mode already has established DDS discovery
                        # and must finish inside the short stance pause.
                        first_autonomous_plan = args.autonomous and not node.autonomous_started
                        node.publish_plan(plan, copies=3 if first_autonomous_plan else (1 if args.continuous else 5))
                        tracked = ("complete LF/RF/LH/RH cycle" if args.autonomous else
                                   (FOOT_NAMES[node.next_swing_leg] if args.continuous else
                                    (args.target_foot if args.target_foot else
                                     "all planned swing feet")))
                        node.get_logger().warn(f"Published terrain plan; horizontal MPC target: {tracked}.")
                    last_publish = time.monotonic()
                    node.plan_pending = False
                    exit_code = 0
                    if args.autonomous and not node.autonomous_started and not args.dry_run:
                        node.start_autonomous_motion()
                    if not args.continuous:
                        break
            except TransformException as error:
                node.get_logger().warn(str(error), throttle_duration_sec=1.0)
            except RuntimeError as error:
                node.get_logger().warn(str(error), throttle_duration_sec=1.0)
                if args.continuous and not args.dry_run:
                    if args.autonomous and not node.autonomous_started:
                        # No gait or velocity command has been issued yet.
                        # Sparse edge cells can make one map update incomplete;
                        # keep waiting for a complete safe initial plan instead
                        # of aborting an otherwise stationary startup.
                        continue
                    node.fail_safe_stop()
                    node.plan_pending = False
        if exit_code and not args.continuous:
            node.get_logger().error("timed out without publishing a complete safe foothold plan")
    except (KeyboardInterrupt, ExternalShutdownException):
        exit_code = 130
    finally:
        if args.autonomous and node.autonomous_started and rclpy.ok():
            try:
                node.fail_safe_stop()
            except Exception:
                # SIGINT may invalidate the rcl context before Python enters
                # this finally block.  The explicit command-line stop remains
                # available, and shutdown must not produce a traceback.
                pass
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Paper-compatible elevation-map classification and plane segmentation.

Implements Sections III-A/B/D of Grandia et al., RA-L 2023:
  * minimum-border inpainting followed by a median filter,
  * 3x3 PCA steppability (sigma_n <= 2 cm, n_z >= 0.82),
  * connected components, global plane validation and deterministic RANSAC,
  * contour extraction with a one-cell safety margin,
  * a smooth torso-reference height layer.
"""

import copy
import math
import time

import cv2
import numpy as np
from scipy import ndimage

import rclpy
from geometry_msgs.msg import Point32
from grid_map_msgs.msg import GridMap
from rclpy.node import Node

from legged_terrain_msgs.msg import PlanarRegion, PlanarTerrain


class Grid:
    def __init__(self, msg):
        self.msg = msg
        self.resolution = float(msg.info.resolution)
        self.length_x = float(msg.info.length_x)
        self.length_y = float(msg.info.length_y)
        self.center_x = float(msg.info.pose.position.x)
        self.center_y = float(msg.info.pose.position.y)
        self.outer_start = int(msg.outer_start_index)
        self.inner_start = int(msg.inner_start_index)
        self.layers = {}
        self.templates = {}
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
            self.layers[name] = np.roll(
                matrix, (-self.outer_start, -self.inner_start), axis=(0, 1))
            self.templates[name] = array
        if "elevation" not in self.layers:
            raise ValueError("input GridMap has no elevation layer")
        self.rows, self.cols = self.layers["elevation"].shape

    def position(self, row, col):
        return (
            self.center_x + self.length_x / 2.0 - (row + 0.5) * self.resolution,
            self.center_y + self.length_y / 2.0 - (col + 0.5) * self.resolution,
        )

    def encode(self, matrix, template):
        circular = np.roll(
            np.asarray(matrix, np.float32),
            (self.outer_start, self.inner_start), axis=(0, 1))
        result = copy.deepcopy(template)
        column_major = result.layout.dim[0].label == "column_index"
        result.data = circular.flatten(order="F" if column_major else "C").tolist()
        return result


def fit_plane(points):
    mean = np.mean(points, axis=0)
    covariance = (points - mean).T @ (points - mean) / max(1, len(points))
    values, vectors = np.linalg.eigh(covariance)
    normal = vectors[:, int(np.argmin(values))]
    if normal[2] < 0.0:
        normal = -normal
    return mean, normal / max(np.linalg.norm(normal), 1e-12), math.sqrt(max(0.0, float(values[0])))


class PerceptiveTerrainProcessor(Node):
    def __init__(self):
        super().__init__("perceptive_terrain_processor")
        self.declare_parameter("input_topic", "/elevation_map_raw")
        self.declare_parameter("output_topic", "/perceptive_terrain_map")
        self.declare_parameter("terrain_topic", "/planar_terrain")
        self.declare_parameter("processing_rate", 20.0)
        self.declare_parameter("local_roughness_limit", 0.020)
        self.declare_parameter("local_normal_z_limit", 0.82)
        self.declare_parameter("plane_roughness_limit", 0.025)
        self.declare_parameter("plane_normal_z_limit", 0.87)
        self.declare_parameter("minimum_plane_cells", 4)
        self.declare_parameter("ransac_distance", 0.025)
        self.declare_parameter("safety_margin_cells", 1)
        self.declare_parameter("torso_dilation", 0.20)
        self.declare_parameter("torso_box", 0.10)
        self.declare_parameter("torso_gaussian", 0.05)

        self.local_roughness = float(self.get_parameter("local_roughness_limit").value)
        self.local_normal_z = float(self.get_parameter("local_normal_z_limit").value)
        self.plane_roughness = float(self.get_parameter("plane_roughness_limit").value)
        self.plane_normal_z = float(self.get_parameter("plane_normal_z_limit").value)
        self.min_cells = int(self.get_parameter("minimum_plane_cells").value)
        self.ransac_distance = float(self.get_parameter("ransac_distance").value)
        self.margin_cells = int(self.get_parameter("safety_margin_cells").value)

        self.latest = None
        self.last_stamp = None
        self.processing = False
        self.map_pub = self.create_publisher(
            GridMap, str(self.get_parameter("output_topic").value), 1)
        self.terrain_pub = self.create_publisher(
            PlanarTerrain, str(self.get_parameter("terrain_topic").value), 1)
        self.create_subscription(
            GridMap, str(self.get_parameter("input_topic").value), self.map_callback, 1)
        rate = max(1.0, float(self.get_parameter("processing_rate").value))
        self.create_timer(1.0 / rate, self.process_latest)
        self.get_logger().info(
            "paper terrain processor ready: PCA 3x3, sigma<=0.020, nz>=0.82, "
            "plane sigma<=0.025, nz>=0.87")

    def map_callback(self, msg):
        self.latest = msg

    @staticmethod
    def inpaint_min_border(elevation):
        valid = np.isfinite(elevation)
        if not np.any(valid):
            return elevation.copy()
        filled = elevation.copy()
        # grid_map_filters_rsl::inpainting::minValues, used by the paper:
        # every connected unknown region receives the minimum finite value
        # along its complete one-cell contour.  This deliberately creates a
        # conservative horizontal surface behind occlusions.
        unknown_labels, count = ndimage.label(
            ~valid, structure=ndimage.generate_binary_structure(2, 1))
        structure = np.ones((3, 3), dtype=bool)
        for label in range(1, count + 1):
            region = unknown_labels == label
            boundary = ndimage.binary_dilation(region, structure=structure) & valid
            if np.any(boundary):
                filled[region] = float(np.min(elevation[boundary]))
            else:
                # This is only possible when the unknown component surrounds
                # the entire finite map.  Preserve a finite conservative map.
                filled[region] = float(np.min(elevation[valid]))
        return filled

    def classify(self, height):
        resolution = self.grid.resolution
        mean_z = ndimage.uniform_filter(height, size=3, mode="nearest")
        mean_z2 = ndimage.uniform_filter(height * height, size=3, mode="nearest")
        offsets = np.arange(-1, 2, dtype=float)
        x = np.repeat((-offsets * resolution)[:, None], 3, axis=1)
        y = np.repeat((-offsets * resolution)[None, :], 3, axis=0)
        cov_xz = ndimage.convolve(height, x / 9.0, mode="nearest")
        cov_yz = ndimage.convolve(height, y / 9.0, mode="nearest")
        cov_zz = np.maximum(0.0, mean_z2 - mean_z * mean_z)
        covariance = np.zeros(height.shape + (3, 3), dtype=float)
        covariance[..., 0, 0] = 2.0 * resolution * resolution / 3.0
        covariance[..., 1, 1] = 2.0 * resolution * resolution / 3.0
        covariance[..., 2, 2] = cov_zz
        covariance[..., 0, 2] = covariance[..., 2, 0] = cov_xz
        covariance[..., 1, 2] = covariance[..., 2, 1] = cov_yz
        values, vectors = np.linalg.eigh(covariance)
        normal = vectors[..., :, 0]
        normal[normal[..., 2] < 0.0] *= -1.0
        roughness = np.sqrt(np.maximum(0.0, values[..., 0])).astype(np.float32)
        normals = np.moveaxis(normal.astype(np.float32), -1, 0)
        steppable = np.logical_and(
            roughness <= self.local_roughness,
            normals[2] >= self.local_normal_z).astype(np.uint8)
        steppable[[0, -1], :] = 0
        steppable[:, [0, -1]] = 0
        return steppable, roughness, normals

    def points_for_mask(self, mask, height):
        rc = np.argwhere(mask)
        points = np.empty((len(rc), 3), dtype=float)
        points[:, 0] = (
            self.grid.center_x + self.grid.length_x / 2.0
            - (rc[:, 0] + 0.5) * self.grid.resolution
        )
        points[:, 1] = (
            self.grid.center_y + self.grid.length_y / 2.0
            - (rc[:, 1] + 0.5) * self.grid.resolution
        )
        points[:, 2] = height[rc[:, 0], rc[:, 1]]
        return rc, points

    def ransac_regions(self, component, height):
        remaining = component.copy()
        regions = []
        rng = np.random.default_rng(0)
        while int(np.count_nonzero(remaining)) >= self.min_cells:
            rc, points = self.points_for_mask(remaining, height)
            best = None
            iterations = min(250, max(40, len(points) * 2))
            for _ in range(iterations):
                sample = rng.choice(len(points), 3, replace=False)
                p0, p1, p2 = points[sample]
                normal = np.cross(p1 - p0, p2 - p0)
                norm = np.linalg.norm(normal)
                if norm < 1e-9:
                    continue
                normal /= norm
                if normal[2] < 0:
                    normal = -normal
                if normal[2] < self.plane_normal_z:
                    continue
                inliers = np.abs((points - p0) @ normal) <= self.ransac_distance
                if best is None or np.count_nonzero(inliers) > np.count_nonzero(best):
                    best = inliers
            if best is None or np.count_nonzero(best) < self.min_cells:
                break
            candidate = np.zeros_like(component)
            candidate[rc[best, 0], rc[best, 1]] = 1
            labels, count = ndimage.label(candidate, structure=ndimage.generate_binary_structure(2, 1))
            accepted = False
            for label in range(1, count + 1):
                island = labels == label
                if np.count_nonzero(island) < self.min_cells:
                    continue
                _, island_points = self.points_for_mask(island, height)
                origin, normal, sigma = fit_plane(island_points)
                if sigma <= self.plane_roughness and normal[2] >= self.plane_normal_z:
                    regions.append((island, origin, normal))
                    remaining[island] = 0
                    accepted = True
            if not accepted:
                remaining[rc[best, 0], rc[best, 1]] = 0
        return regions

    def segment(self, steppable, height):
        labels, count = ndimage.label(
            steppable, structure=ndimage.generate_binary_structure(2, 1))
        regions = []
        for label in range(1, count + 1):
            component = labels == label
            if np.count_nonzero(component) < self.min_cells:
                continue
            _, points = self.points_for_mask(component, height)
            origin, normal, sigma = fit_plane(points)
            if sigma <= self.plane_roughness and normal[2] >= self.plane_normal_z:
                regions.append((component, origin, normal))
            else:
                regions.extend(self.ransac_regions(component, height))
        return regions

    def polygon_from_contour(self, contour, origin, normal):
        polygon = []
        for item in contour:
            col, row = item[0]
            x, y = self.grid.position(int(row), int(col))
            z = origin[2]
            if abs(normal[2]) > 1e-8:
                z = origin[2] - (
                    normal[0] * (x - origin[0]) + normal[1] * (y - origin[1])) / normal[2]
            polygon.append(Point32(x=float(x), y=float(y), z=float(z)))
        return polygon

    def region_message(self, index, mask, origin, normal):
        binary = (mask.astype(np.uint8) * 255)
        if self.margin_cells > 0:
            kernel = np.ones((3, 3), np.uint8)
            eroded = cv2.erode(binary, kernel, iterations=self.margin_cells)
            if np.count_nonzero(eroded) >= self.min_cells:
                binary = eroded
        contours, hierarchy = cv2.findContours(
            binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        hierarchy = hierarchy[0]
        outer_indices = [i for i, item in enumerate(hierarchy) if item[3] < 0]
        if not outer_indices:
            return None
        outer = max(outer_indices, key=lambda i: cv2.contourArea(contours[i]))
        region = PlanarRegion()
        region.id = index
        region.origin.x, region.origin.y, region.origin.z = map(float, origin)
        region.normal.x, region.normal.y, region.normal.z = map(float, normal)
        region.boundary.points = self.polygon_from_contour(contours[outer], origin, normal)
        child = hierarchy[outer][2]
        while child >= 0:
            hole = copy.deepcopy(region.boundary)
            hole.points = self.polygon_from_contour(contours[child], origin, normal)
            region.holes.append(hole)
            child = hierarchy[child][0]
        return region if len(region.boundary.points) >= 3 else None

    def torso_layer(self, height):
        resolution = self.grid.resolution
        dilate_n = max(1, int(round(
            float(self.get_parameter("torso_dilation").value) / resolution)))
        box_n = max(1, int(round(
            float(self.get_parameter("torso_box").value) / resolution)))
        gaussian_n = max(1, int(round(
            float(self.get_parameter("torso_gaussian").value) / resolution)))
        dilated = ndimage.maximum_filter(height, size=2 * dilate_n + 1, mode="nearest")
        boxed = ndimage.uniform_filter(dilated, size=2 * box_n + 1, mode="nearest")
        return ndimage.gaussian_filter(boxed, sigma=gaussian_n, mode="nearest")

    def publish_map(self, source, layers):
        output = copy.deepcopy(source)
        template = source.data[source.layers.index("elevation")]
        for name, matrix in layers.items():
            if name in output.layers:
                output.data[output.layers.index(name)] = self.grid.encode(matrix, template)
            else:
                output.layers.append(name)
                output.data.append(self.grid.encode(matrix, template))
        output.basic_layers = ["elevation", "steppability"]
        self.map_pub.publish(output)

    def process_latest(self):
        if self.latest is None or self.processing:
            return
        stamp = (self.latest.header.stamp.sec, self.latest.header.stamp.nanosec)
        if stamp == self.last_stamp:
            return
        self.processing = True
        started = time.monotonic()
        try:
            source = self.latest
            self.grid = Grid(source)
            raw = self.grid.layers["elevation"]
            inpainted = self.inpaint_min_border(raw)
            median = cv2.medianBlur(inpainted.astype(np.float32), 3)
            steppable, roughness, normals = self.classify(median)
            regions = self.segment(steppable, median)
            plane_ids = np.full(raw.shape, -1.0, np.float32)
            planar = np.zeros(raw.shape, np.float32)
            terrain = PlanarTerrain()
            terrain.header = source.header
            for index, (mask, origin, normal) in enumerate(regions):
                message = self.region_message(index, mask, origin, normal)
                if message is None:
                    continue
                message.id = len(terrain.regions)
                terrain.regions.append(message)
                plane_ids[mask] = float(message.id)
                planar[mask] = 1.0
            self.publish_map(source, {
                "elevation_inpainted": inpainted,
                "elevation_median": median,
                "roughness": roughness,
                "steppability": steppable.astype(np.float32),
                "planar": planar,
                "plane_id": plane_ids,
                "paper_normal_x": normals[0],
                "paper_normal_y": normals[1],
                "paper_normal_z": normals[2],
                "torso_reference": self.torso_layer(median),
            })
            self.terrain_pub.publish(terrain)
            self.last_stamp = stamp
            elapsed = 1000.0 * (time.monotonic() - started)
            self.get_logger().info(
                f"processed {raw.shape[0]}x{raw.shape[1]} map: "
                f"{len(terrain.regions)} planes in {elapsed:.1f} ms",
                throttle_duration_sec=2.0)
        except Exception as error:
            self.get_logger().error(f"terrain processing failed: {error}")
        finally:
            self.processing = False


def main():
    rclpy.init()
    node = PerceptiveTerrainProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

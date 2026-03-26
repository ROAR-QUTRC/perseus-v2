import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import open3d as o3d
import numpy as np
import os
import glob
from pathlib import Path


def find_output_folder(start: Path) -> Path:
    current = start
    for _ in range(5):
        candidate = current / "web_ui/static/maps"
        if candidate.exists():
            return candidate
        current = current.parent
    raise RuntimeError("Could not find web_ui/static/maps folder")


class PcdToGlbNode(Node):
    """
    A ROS2 node that watches a folder for new PCD files and converts them to .glb
    with a ground-normalised height heatmap colouring.

    Ground detection strategy (in order of preference):
      1. RANSAC plane fitting — finds the dominant horizontal plane regardless
         of where the rover started (ditch, ramp, etc.).  Returns the full plane
         equation (a, b, c, d) so per-vertex signed distance can be computed,
         which correctly handles sloped ground without a colour gradient tilt.
      2. Histogram peak fallback — used when RANSAC returns a near-vertical plane
         (e.g. a wall dominated the scene).  A flat horizontal plane is synthesised
         at the most common Z value.

    Heatmap:  blue = below ground · green = ground level · red = above ground
    """

    def __init__(self):
        super().__init__("pcd_to_glb")
        self._declare_parameters()
        self._load_parameters()

        try:
            self.output_folder = find_output_folder(Path(__file__).parent)
        except RuntimeError as e:
            self.get_logger().error(str(e))
            raise

        os.makedirs(self.input_folder, exist_ok=True)
        os.makedirs(self.output_folder, exist_ok=True)

        self.processed: set = set()

        self.timer = self.create_timer(self.check_interval, self.check_for_new_files)
        self.get_logger().info(f"Monitoring {self.input_folder} for new PCD files")
        self.get_logger().info(f"Output maps folder: {self.output_folder}")
        self.get_logger().info(f"Checking time interval: {self.check_interval}s")

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "input_folder",
            str(Path.home() / "maps"),
            ParameterDescriptor(description="Folder to monitor for new PCD files"),
        )
        self.declare_parameter(
            "check_interval",
            10.0,
            ParameterDescriptor(description="How often to check for new files in seconds"),
        )
        self.declare_parameter(
            "poisson_depth",
            9,
            ParameterDescriptor(description="Depth for Poisson surface reconstruction"),
        )
        self.declare_parameter(
            "density_quantile",
            0.003,
            ParameterDescriptor(description="Density filter quantile for mesh cleanup"),
        )
        # self.declare_parameter(
        #     "crop_margin",
        #     0.1,
        #     ParameterDescriptor(description="Margin around point cloud bounds when cropping in metres"),
        # )

        # Ground detection
        self.declare_parameter(
            "ransac_distance_threshold",
            0.05,
            ParameterDescriptor(
                description="RANSAC inlier distance threshold in metres "
                            "(tighten for flat ground, loosen for noisy LiDAR)"
            ),
        )
        self.declare_parameter(
            "ransac_iterations",
            2000,
            ParameterDescriptor(description="Number of RANSAC iterations for plane fitting"),
        )
        self.declare_parameter(
            "ground_z_alignment_min",
            0.7,
            ParameterDescriptor(
                description="Minimum |cos(angle)| between plane normal and Z axis to accept as "
                            "ground (0–1). Below this triggers histogram fallback."
            ),
        )
        # Heatmap colour range (metres relative to detected ground plane)
        self.declare_parameter(
            "heatmap_z_low",
            -1, 
            ParameterDescriptor(description="Signed distance (m) below ground mapped to full blue"),
        )
        self.declare_parameter(
            "heatmap_z_high",
            1.5,
            ParameterDescriptor(description="Signed distance (m) above ground mapped to full red"),
        )

    def _load_parameters(self) -> None:
        self.input_folder = Path(
            self.get_parameter("input_folder").get_parameter_value().string_value
        )
        self.check_interval = (
            self.get_parameter("check_interval").get_parameter_value().double_value
        )
        self.poisson_depth = (
            self.get_parameter("poisson_depth").get_parameter_value().integer_value
        )
        self.density_quantile = (
            self.get_parameter("density_quantile").get_parameter_value().double_value
        )
        # self.crop_margin = (
        #     self.get_parameter("crop_margin").get_parameter_value().double_value
        # )
        self.ransac_distance_threshold = (
            self.get_parameter("ransac_distance_threshold").get_parameter_value().double_value
        )
        self.ransac_iterations = (
            self.get_parameter("ransac_iterations").get_parameter_value().integer_value
        )
        self.ground_z_alignment_min = (
            self.get_parameter("ground_z_alignment_min").get_parameter_value().double_value
        )
        self.heatmap_z_low = (
            self.get_parameter("heatmap_z_low").get_parameter_value().double_value
        )
        self.heatmap_z_high = (
            self.get_parameter("heatmap_z_high").get_parameter_value().double_value
        )
        

    # ------------------------------------------------------------------
    # File watching
    # ------------------------------------------------------------------

    def check_for_new_files(self) -> None:
        pcd_files = sorted(glob.glob(os.path.join(self.input_folder, "*.pcd")))
        new_files = [f for f in pcd_files if f not in self.processed]
        for pcd_path in new_files:
            self.process_file(pcd_path)
            self.processed.add(pcd_path)

    # ------------------------------------------------------------------
    # Ground plane estimation
    # ------------------------------------------------------------------

    def _estimate_ground_plane(
        self, point_cloud: o3d.geometry.PointCloud
    ) -> tuple[float, float, float, float]:
        """
        Returns the RANSAC plane model (a, b, c, d) where ax + by + cz + d = 0
        and the normal (a, b, c) points upward (c > 0) in the ORIGINAL
        (pre-flip) coordinate space.

        Falls back to a synthetic horizontal plane at the histogram-peak Z
        when RANSAC finds a near-vertical surface (e.g. a wall).
        """
        plane_model, inliers = point_cloud.segment_plane(
            distance_threshold=self.ransac_distance_threshold,
            ransac_n=3,
            num_iterations=self.ransac_iterations,
        )
        a, b, c, d = plane_model

        normal = np.array([a, b, c], dtype=float)
        normal /= np.linalg.norm(normal)
        z_alignment = abs(normal[2])

        if z_alignment >= self.ground_z_alignment_min:
            # Ensure normal points upward (positive Z) in the original space
            if c < 0:
                a, b, c, d = -a, -b, -c, -d
            self.get_logger().info(
                f"Ground plane via RANSAC — "
                f"z_alignment={z_alignment:.3f}, inliers={len(inliers)}, "
                f"plane=({a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0)"
            )
            return (a, b, c, d)

        # ---- Fallback: flat horizontal plane at histogram peak Z ----
        self.get_logger().warn(
            f"RANSAC plane near-vertical (z_alignment={z_alignment:.3f} < "
            f"{self.ground_z_alignment_min}), falling back to histogram peak"
        )
        pts = np.asarray(point_cloud.points)
        hist, bin_edges = np.histogram(pts[:, 2], bins=200)
        peak_bin = int(np.argmax(hist))
        ground_z = float((bin_edges[peak_bin] + bin_edges[peak_bin + 1]) / 2.0)
        self.get_logger().info(f"Ground z from histogram peak: {ground_z:.3f} m")
        # Horizontal upward-facing plane: 0x + 0y + 1z - ground_z = 0
        return (0.0, 0.0, 1.0, -ground_z)

    # ------------------------------------------------------------------
    # Main processing
    # ------------------------------------------------------------------

    def process_file(self, pcd_path: str) -> None:
        filename = os.path.splitext(os.path.basename(pcd_path))[0]
        output_path = os.path.join(self.output_folder, f"{filename}.glb")
        self.get_logger().info(f"Processing: {pcd_path}")

        try:
            point_cloud = o3d.io.read_point_cloud(pcd_path)

            point_cloud.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=3.0, max_nn=30)
            )
            point_cloud.orient_normals_consistent_tangent_plane(k=20)

            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
                point_cloud, depth=self.poisson_depth
            )
            densities = np.asarray(densities)
            mesh.remove_vertices_by_mask(
                densities < np.quantile(densities, self.density_quantile)
            )

            # ----------------------------------------------------------
            # Compute per-vertex signed distance BEFORE flipping Z.
            #
            # We do this in the original coordinate space where the plane
            # normal is well-defined as upward-facing (c > 0 guaranteed by
            # _estimate_ground_plane).  This avoids any sign ambiguity
            # introduced by the Z-flip.
            #
            #   signed_dist = (a·x + b·y + c·z + d) / ||(a,b,c)||
            #
            # Positive  → above ground (walls, obstacles)
            # Zero      → on the ground plane
            # Negative  → below ground (ditches, pits)
            # ----------------------------------------------------------
            a, b, c, d = self._estimate_ground_plane(point_cloud)
            plane_normal_len = np.sqrt(a**2 + b**2 + c**2)

            pre_flip_vertices = np.asarray(mesh.vertices)
            z_relative = (
                a * pre_flip_vertices[:, 0]
                + b * pre_flip_vertices[:, 1]
                + c * pre_flip_vertices[:, 2]
                + d
            ) / plane_normal_len
            

            # Flip Z axis — mesh renders inverted in Three.js otherwise.
            # Heights were already captured above so this only affects geometry.
            pre_flip_vertices[:, 2] *= -1
            mesh.vertices = o3d.utility.Vector3dVector(pre_flip_vertices)

            mesh.triangle_normals = o3d.utility.Vector3dVector([])
            mesh.compute_vertex_normals()

            # ----------------------------------------------------------
            # Ground-normalised height heatmap
            #   t=0 (z_low)  → blue   (below ground / ditch)
            #   t=0.5        → green  (ground level)
            #   t=1 (z_high) → red    (above ground / obstacles / walls)
            # ----------------------------------------------------------
            t = np.clip(
                0.5 + z_relative / (2.0 * np.where(z_relative >= 0, self.heatmap_z_high, abs(self.heatmap_z_low))),
                0.0, 1.0
            )

            colors = np.zeros((len(t), 3))
            colors[:, 1] = np.clip(2.0 * t, 0, 1) * np.clip(2.0 * (1.0 - t), 0, 1)   # G: peaks at mid
            colors[:, 0] = np.clip(1.0 - 2.0 * t, 0, 1)                               # B: lower half
            colors[:, 2] = np.clip(2.0 * t - 1.0, 0, 1)                               # R: upper half

            mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
            pts = np.asarray(point_cloud.points)

        

            o3d.io.write_triangle_mesh(output_path, mesh)
            self.get_logger().info(f"Saved: {output_path}")
            self.get_logger().info(
                f"z_relative stats: min={z_relative.min():.3f}, max={z_relative.max():.3f}, "
                f"mean={z_relative.mean():.3f}, median={np.median(z_relative):.3f}"
            )

        except Exception as e:
            self.get_logger().error(f"Error processing {pcd_path}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PcdToGlbNode()
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
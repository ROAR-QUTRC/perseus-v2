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
    A ROS2 node that watches a folder for new PCD files and converts them to GLB.
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

        self.processed = set()

        self.timer = self.create_timer(self.check_interval, self.check_for_new_files)

        self.get_logger().info(f"Monitoring {self.input_folder} for new PCD files")
        self.get_logger().info(f"Output maps folder: {self.output_folder}")
        self.get_logger().info(f"Checking time interval: {self.check_interval}s")

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "input_folder",
            str(Path.home() / "maps"),
            ParameterDescriptor(description="Folder to monitor for new PCD files"),
        )
        self.declare_parameter(
            "check_interval",
            10.0,
            ParameterDescriptor(
                description="How often to check for new files in seconds"
            ),
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
        self.declare_parameter(
            "crop_margin",
            0.1,
            ParameterDescriptor(
                description="Margin around point cloud bounds when cropping in metres"
            ),
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
        self.crop_margin = (
            self.get_parameter("crop_margin").get_parameter_value().double_value
        )

    def check_for_new_files(self) -> None:
        pcd_files = sorted(glob.glob(os.path.join(self.input_folder, "*.pcd")))
        new_files = [f for f in pcd_files if f not in self.processed]

        for pcd_path in new_files:
            self.process_file(pcd_path)
            self.processed.add(pcd_path)

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

            # # Crop to point cloud edges to try and fix warping issues??
            # pts = np.asarray(point_cloud.points)
            # x_min = pts[:, 0].min() - self.crop_margin
            # y_min = pts[:, 1].min() - self.crop_margin
            # x_max = pts[:, 0].max() + self.crop_margin
            # y_max = pts[:, 1].max() + self.crop_margin
            # vertices = np.asarray(mesh.vertices)
            # outside = (
            #     (vertices[:, 0] < x_min) | (vertices[:, 0] > x_max) |
            #     (vertices[:, 1] < y_min) | (vertices[:, 1] > y_max)
            # )
            # mesh.remove_vertices_by_mask(outside)

            # Flip Z axis - Shows inverted otherwise
            vertices = np.asarray(mesh.vertices)
            vertices[:, 2] *= -1
            mesh.vertices = o3d.utility.Vector3dVector(vertices)
            mesh.triangle_normals = o3d.utility.Vector3dVector([])
            mesh.compute_vertex_normals()

            # # Height based colouring / heatmap
            # vertices = np.asarray(mesh.vertices)
            # z = vertices[:, 2]
            # z_min, z_max = z.min(), z.max()
            # t = 1.0 - (z - z_min) / (z_max - z_min + 1e-8)
            # colors = np.zeros((len(t), 3))
            # colors[:, 0] = np.clip(2.0 * t - 1.0, 0, 1)
            # colors[:, 1] = np.clip(2.0 * t, 0, 1) * np.clip(2.0 * (1 - t), 0, 1)
            # colors[:, 2] = np.clip(1.0 - 2.0 * t, 0, 1)
            # mesh.vertex_colors = o3d.utility.Vector3dVector(colors)

            o3d.io.write_triangle_mesh(output_path, mesh)
            self.get_logger().info(f"Saved: {output_path}")

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

#!/usr/bin/env python3
"""Open3D demo — visual proof that the Nix-managed Open3D install works."""

import sys
import numpy as np
import open3d as o3d
import plotly.graph_objects as go


def make_helix_point_cloud(n_points=5000, turns=5, radius=1.0, height=4.0):
    """Create a colourful double-helix point cloud."""
    t = np.linspace(0, turns * 2 * np.pi, n_points)
    # First helix strand
    x1 = radius * np.cos(t)
    y1 = radius * np.sin(t)
    z1 = np.linspace(0, height, n_points)
    # Second strand, offset by pi
    x2 = radius * np.cos(t + np.pi)
    y2 = radius * np.sin(t + np.pi)
    z2 = z1.copy()

    points = np.vstack(
        [
            np.column_stack([x1, y1, z1]),
            np.column_stack([x2, y2, z2]),
        ]
    )

    # Colour by height: blue at bottom → red at top
    heights = points[:, 2] / height
    colors = np.zeros((len(points), 3))
    colors[:, 0] = heights  # red increases with height
    colors[:, 2] = 1.0 - heights  # blue decreases with height
    colors[:, 1] = 0.3 * np.sin(heights * np.pi)  # a touch of green in the middle

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def make_sample_meshes():
    """Create a few primitive meshes arranged in a row."""
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5)
    sphere.translate([-2, 0, 0])
    sphere.paint_uniform_color([0.1, 0.7, 0.3])
    sphere.compute_vertex_normals()

    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=0.3, height=1.5)
    cylinder.translate([0, 0, 0])
    cylinder.paint_uniform_color([0.8, 0.2, 0.2])
    cylinder.compute_vertex_normals()

    torus = o3d.geometry.TriangleMesh.create_torus(torus_radius=0.6, tube_radius=0.2)
    torus.translate([2, 0, 0])
    torus.paint_uniform_color([0.2, 0.4, 0.9])
    torus.compute_vertex_normals()

    return [sphere, cylinder, torus]


def mesh_to_plotly(mesh, name, color):
    """Convert an Open3D TriangleMesh to a plotly Mesh3d trace."""
    verts = np.asarray(mesh.vertices)
    tris = np.asarray(mesh.triangles)
    return go.Mesh3d(
        x=verts[:, 0],
        y=verts[:, 1],
        z=verts[:, 2],
        i=tris[:, 0],
        j=tris[:, 1],
        k=tris[:, 2],
        color=f"rgb({int(color[0] * 255)},{int(color[1] * 255)},{int(color[2] * 255)})",
        opacity=0.9,
        name=name,
        flatshading=True,
    )


def main():
    print(f"Open3D version : {o3d.__version__}")
    print(f"NumPy version  : {np.__version__}")
    print()

    # --- Build geometry ---
    pcd = make_helix_point_cloud()
    pts = np.asarray(pcd.points)
    cols = np.asarray(pcd.colors)
    meshes = make_sample_meshes()
    total_triangles = sum(len(m.triangles) for m in meshes)
    total_vertices = sum(len(m.vertices) for m in meshes)

    bbox_min = pts.min(axis=0)
    bbox_max = pts.max(axis=0)

    print(f"Double-helix point cloud: {len(pcd.points)} points")
    print(
        f"  Bounding box min: [{bbox_min[0]:.2f}, {bbox_min[1]:.2f}, {bbox_min[2]:.2f}]"
    )
    print(
        f"  Bounding box max: [{bbox_max[0]:.2f}, {bbox_max[1]:.2f}, {bbox_max[2]:.2f}]"
    )
    print(
        f"Primitive meshes : {len(meshes)} objects, {total_triangles} triangles, {total_vertices} vertices"
    )

    # --- Build plotly figure ---
    fig = go.Figure()

    # Point cloud as scatter3d
    rgb_strings = [
        f"rgb({int(r * 255)},{int(g * 255)},{int(b * 255)})" for r, g, b in cols
    ]
    fig.add_trace(
        go.Scatter3d(
            x=pts[:, 0],
            y=pts[:, 1],
            z=pts[:, 2],
            mode="markers",
            marker=dict(size=1.5, color=rgb_strings),
            name=f"Helix ({len(pts)} pts)",
        )
    )

    # Meshes
    mesh_info = [
        ("Sphere", [0.1, 0.7, 0.3]),
        ("Cylinder", [0.8, 0.2, 0.2]),
        ("Torus", [0.2, 0.4, 0.9]),
    ]
    for mesh, (name, color) in zip(meshes, mesh_info):
        fig.add_trace(mesh_to_plotly(mesh, name, color))

    # Coordinate axes as lines
    axis_len = 0.8
    for axis, color, label in [(0, "red", "X"), (1, "green", "Y"), (2, "blue", "Z")]:
        end = [0, 0, 0]
        end[axis] = axis_len
        fig.add_trace(
            go.Scatter3d(
                x=[0, end[0]],
                y=[0, end[1]],
                z=[0, end[2]],
                mode="lines+text",
                line=dict(color=color, width=5),
                text=["", label],
                textposition="top center",
                textfont=dict(size=14, color=color),
                showlegend=False,
            )
        )

    # --- Stats overlay annotation ---
    stats_text = (
        f"<b>Open3D {o3d.__version__}</b>  |  "
        f"Python {sys.version.split()[0]}  |  "
        f"NumPy {np.__version__}<br>"
        f"Points: {len(pts):,}  |  "
        f"Meshes: {len(meshes)}  |  "
        f"Triangles: {total_triangles:,}  |  "
        f"Vertices: {total_vertices:,}<br>"
        f"BBox: [{bbox_min[0]:.1f}, {bbox_min[1]:.1f}, {bbox_min[2]:.1f}] "
        f"to [{bbox_max[0]:.1f}, {bbox_max[1]:.1f}, {bbox_max[2]:.1f}]"
    )

    fig.update_layout(
        title=dict(
            text="<b>Open3D Demo</b> — Nix-managed build",
            font=dict(size=22),
        ),
        annotations=[
            dict(
                text=stats_text,
                xref="paper",
                yref="paper",
                x=0.01,
                y=0.99,
                xanchor="left",
                yanchor="top",
                showarrow=False,
                font=dict(family="monospace", size=13, color="white"),
                bgcolor="rgba(30,30,30,0.85)",
                bordercolor="rgba(100,100,100,0.6)",
                borderwidth=1,
                borderpad=8,
            ),
        ],
        scene=dict(
            xaxis_title="X",
            yaxis_title="Y",
            zaxis_title="Z",
            aspectmode="data",
            bgcolor="rgb(20,20,30)",
        ),
        paper_bgcolor="rgb(30,30,40)",
        font=dict(color="white"),
        legend=dict(
            bgcolor="rgba(30,30,30,0.85)",
            bordercolor="rgba(100,100,100,0.6)",
            borderwidth=1,
        ),
        margin=dict(l=0, r=0, t=60, b=0),
    )

    # --- Show in browser ---
    outpath = "/tmp/open3d_demo.html"
    fig.write_html(outpath)
    print(f"\nSaved interactive visualisation to {outpath}")
    print("Opening in browser...")
    fig.show()
    print("Done.")


if __name__ == "__main__":
    main()

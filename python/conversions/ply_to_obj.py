import open3d as o3d
import trimesh
import numpy as np

def clean_point_cloud(point_cloud, voxel_size=0.01, nb_neighbors=20, std_ratio=2.0, radius=0.05, min_neighbors=5):
    """
    Cleans a point cloud using downsampling and outlier removal.
    - Voxel downsampling to reduce points.
    - Statistical outlier removal to remove noise.
    - Radius outlier removal for further cleaning.
    """
    print("Cleaning point cloud...")

    # Voxel downsampling
    point_cloud = point_cloud.voxel_down_sample(voxel_size)
    print(f"Downsampled point cloud to {len(point_cloud.points)} points using voxel size {voxel_size}.")

    # Statistical outlier removal
    cl, ind = point_cloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    point_cloud = point_cloud.select_by_index(ind)
    print(f"Removed outliers using statistical criteria, remaining points: {len(point_cloud.points)}.")

    # Radius outlier removal
    cl, ind = point_cloud.remove_radius_outlier(nb_points=min_neighbors, radius=radius)
    point_cloud = point_cloud.select_by_index(ind)
    print(f"Removed radius outliers, remaining points: {len(point_cloud.points)}.")

    return point_cloud

def create_triangle_soup_from_point_cloud(point_cloud, triangle_size=0.01):
    """
    Creates a triangle soup from a point cloud.
    Each point is represented as a small triangle.
    """
    vertices = []
    faces = []
    uvs = []
    normals = []
    face_index = 0

    # Normalize the normal vector (1, 1, 1)
    normal = np.array([1, 1, 1], dtype=np.float32)
    normalized_normal = normal / np.linalg.norm(normal)

    for point in np.asarray(point_cloud.points):
        x, y, z = point

        # Create three vertices for a small triangle
        v1 = (x, y, z)
        v2 = (x + triangle_size, y, z)
        v3 = (x, y + triangle_size, z)

        # Append vertices
        vertices.extend([v1, v2, v3])

        # Append UVs (all set to (0, 0))
        uvs.extend([(0, 0), (0, 0), (0, 0)])

        # Append normals (all set to normalized (1, 1, 1))
        normals.extend([normalized_normal, normalized_normal, normalized_normal])

        # Append a face using the indices of the vertices
        faces.append([face_index, face_index + 1, face_index + 2])
        face_index += 3

    return np.array(vertices), np.array(faces), np.array(uvs), np.array(normals)

def convert_ply_to_obj(input_ply, output_obj, triangle_size=0.01, voxel_size=0.01, nb_neighbors=20, std_ratio=2.0, radius=0.05, min_neighbors=5):
    """
    Converts a .ply file to an .obj file with triangle soup using Open3D and Trimesh.
    Includes point cloud cleaning steps.
    """
    # Read the point cloud using Open3D
    point_cloud = o3d.io.read_point_cloud(input_ply)
    if not point_cloud.has_points():
        raise ValueError("The point cloud contains no points!")

    print(f"Loaded point cloud with {len(point_cloud.points)} points.")

    # Clean the point cloud
    point_cloud = clean_point_cloud(
        point_cloud,
        voxel_size=voxel_size,
        nb_neighbors=nb_neighbors,
        std_ratio=std_ratio,
        radius=radius,
        min_neighbors=min_neighbors
    )

    # Create triangle soup with UVs and normals
    vertices, faces, uvs, normals = create_triangle_soup_from_point_cloud(point_cloud, triangle_size)

    # Create a Trimesh object
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, vertex_normals=normals)

    # Add UV coordinates to the mesh
    mesh.visual.uv = uvs

    # Export to .obj
    mesh.export(output_obj)
    print(f"Conversion complete! Output saved to '{output_obj}'")

# Example usage
input_ply_file = "resources/fdml/scans/labs/250122-182138-3.ply"  # Replace with your .ply file path
output_obj_file = "resources/fdml/scans/labs/lab446a_v2.obj"  # Replace with desired .obj file path
triangle_size = 0.03  # Adjust the size of the triangle as needed

# Point cloud cleaning parameters
voxel_size = 0.03
nb_neighbors = 20
std_ratio = 2.0
radius = 0.05
min_neighbors = 5

convert_ply_to_obj(
    input_ply=input_ply_file,
    output_obj=output_obj_file,
    triangle_size=triangle_size,
    voxel_size=voxel_size,
    nb_neighbors=nb_neighbors,
    std_ratio=std_ratio,
    radius=radius,
    min_neighbors=min_neighbors
)
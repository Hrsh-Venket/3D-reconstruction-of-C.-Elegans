import numpy as np
import skimage as ski
import skimage.io as skio
import PIL as pil
import PIL.Image as pilimg
import matplotlib.pyplot as plt
import cv2
import os
import open3d as o3d
from scipy.ndimage import gaussian_filter
from sklearn.neighbors import KNeighborsClassifier

input_image = np.load('input_image.npy')
input_trimap = np.load('input_trimap.npy')

print(f"Data loaded")

from skimage.filters import threshold_multiotsu

def image_to_trimap(image):
    sato = ski.filters.sato(input_image, sigmas=range(3, 5, 5), black_ridges=False, mode='constant', cval=0)
    
    thresholds = threshold_multiotsu(sato, classes=3)

    regions = np.digitize(sato, bins=thresholds)
    
    scaled_regions = regions * 128
    scaled_regions[scaled_regions > 255] = 255
    
    return scaled_regions

output_trimap = image_to_trimap(input_image)

print(f"trimap generated")

def gaussian(distance):
    sigma = 1.0 
    weight = np.exp(-(distance**2) / (2 * sigma**2))
    return weight

def label_propogation_pcl(trimap, window_size = 3, sigma =(1, 1, 1)):
    temp_trimap = trimap.copy()
    
    sigma = (1, 1, 1)

    distance_x = 0.325
    distance_y = 0.325
    distance_z = 1
    
    scale_factor_x = distance_x / sigma[0]
    scale_factor_y = distance_y / sigma[1]
    scale_factor_z = distance_z / sigma[2]

    no = np.argwhere(temp_trimap == 0)
    yes = np.argwhere(temp_trimap == 255)
    unknown = np.argwhere(temp_trimap == 128)

    # convert to point cloud for 0, 128, 255
    point_cloud_no = no * [scale_factor_x, scale_factor_y, scale_factor_z]
    point_cloud_yes = yes * [scale_factor_x, scale_factor_y, scale_factor_z]
    point_cloud_unknown = unknown * [scale_factor_x, scale_factor_y, scale_factor_z]

    # concatenate 'no' and 'yes' point clouds
    point_cloud_known = np.concatenate((point_cloud_no, point_cloud_yes), axis=0)
    labels_known = np.concatenate((np.zeros(len(point_cloud_no)), np.ones(len(point_cloud_yes))), axis=0)

    # apply weighted knn on the unknowns finding nearest in yes + no
    knn = KNeighborsClassifier(n_neighbors=100, weights=gaussian)
    knn.fit(point_cloud_known, labels_known)

    labels_unknown = knn.predict(point_cloud_unknown)

    # assign the value to the unknowns in the trimap
    for idx, point in enumerate(unknown):
        temp_trimap[tuple(point)] = labels_unknown[idx] * 255
    
    return temp_trimap


bayes = label_propogation_pcl(output_trimap)

print(f"label propogation complete")

indices = np.argwhere(ski.morphology.remove_small_objects(bayes==255, min_size=10000, connectivity=4))

print(f"small objects removed")

scale_factor_x = 0.325
scale_factor_y = 0.325
scale_factor_z = 1

indices[:, 0] = indices[:, 0] * scale_factor_x
indices[:, 1] = indices[:, 1] * scale_factor_y
indices[:, 2] = indices[:, 2] * scale_factor_z

# Create a PointCloud object
pcd = o3d.geometry.PointCloud()

# Convert the numpy array to a Vector3dVector
pcd.points = o3d.utility.Vector3dVector(indices)

# Perform statistical outlier removal
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=1000, std_ratio=1)

# Now 'pcd' is the cleaned point cloud

o3d.visualization.draw_geometries([pcd])

print(f"point cloud visualised")

alpha = 15
print(f"alpha={alpha:.3f}")
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
mesh.compute_vertex_normals()


def pick_points(mesh):
    print("1) Please pick points using [shift + left click]")
    print("   Press [shift + right click] to undo point picking")
    print("2) After picking points, press 'Q' to close the window")

    vis = o3d.visualization.VisualizerWithVertexSelection()
    vis.create_window()
    vis.add_geometry(mesh)
    vis.run()  # user picks points
    vis.destroy_window()

    return vis.get_picked_points()


picked_points = pick_points(mesh)

points = []

for picked_point in picked_points:
    # print(f"Index: {picked_point.index}, Coordinates: {picked_point.coord}")
    points.append(picked_point.coord)
    
points = np.array(points)
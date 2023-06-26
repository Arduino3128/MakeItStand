import stltovoxel
import numpy as np
from stl import mesh
import matplotlib.pyplot as plt
import open3d as o3d

# stltovoxel.convert_file("input.stl", "output.npy", resolution=100)
# voxels = np.load("output.npy")

# print(voxels.shape)
# np.save("data.npy", voxels)
# ground_point = voxels.min()

raw_mesh = mesh.Mesh.from_file("MIS.stl")
vol, cog, inertia = raw_mesh.get_mass_properties()
print(cog, sep=" ")

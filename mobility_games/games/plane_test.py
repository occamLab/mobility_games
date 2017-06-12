import pcl
import numpy as np

cloud = pcl.load("cloud.pcd")

print "data size: " + str(cloud.size)

#Filtering
fil = cloud.make_passthrough_filter()
fil.set_filter_field_name("z")
fil.set_filter_limits(0, 1.5)
cloud_filtered = fil.filter()

print "filtered data size: " + str(cloud_filtered.size)

#Plane
seg = cloud_filtered.make_segmenter_normals(ksearch=50)
seg.set_optimize_coefficients(True)
seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
seg.set_normal_distance_weight(0.1)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_max_iterations(100)
seg.set_distance_threshold(0.03)
indices, plane_model = seg.segment()

print "plane model: " + str(plane_model)

cloud_plane = cloud_filtered.extract(indices, negative=False)
cloud_plane.to_file("cloud_plane.pcd")

print "plane_size: " + str(cloud_plane.size)

#Cylinder
cloud_cyl = cloud_filtered.extract(indices, negative=True)

seg = cloud_cyl.make_segmenter_normals(ksearch=50)
seg.set_optimize_coefficients(True)
seg.set_model_type(pcl.SACMODEL_CYLINDER)
seg.set_normal_distance_weight(0.1)
seg.set_method_type(pcl.SAC_RANSAC)
seg.set_max_iterations(10000)
seg.set_distance_threshold(0.05)
seg.set_radius_limits(0, 0.1)
indices, cyl_model = seg.segment()

print "cylinder model: " + str(cyl_model)

cloud_cylinder = cloud_cyl.extract(indices, negative=False)
cloud_cylinder.to_file("cloud_cylinder.pcd")

print "cylinder_size: " + str(cloud_cylinder.size)

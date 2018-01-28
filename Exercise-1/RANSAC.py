# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('PR2_POINTCLOUD.pcd')


# Voxel Grid filter

# Create a VoxelGrid filter object for our input point cloud
vox = cloud.make_voxel_grid_filter()

LEAF_SIZE = 0.003
# Set the voxel (or leaf) size  
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
# Call the filter function to obtain the resultant downsampled point cloud
cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)


#Outlier Removal filter

outlier_filter = cloud_filtered.make_statistical_outlier_filter()
# Set the number of neighboring points to analyze for any given point
outlier_filter.set_mean_k(60)
# Set threshold scale factor
x = 0.5
# Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
outlier_filter.set_std_dev_mul_thresh(x)

# Finally call the filter function for magic
cloud_filtered = outlier_filter.filter()
filename = 'clean_cloud.pcd'
pcl.save(cloud_filtered, filename)

# PassThrough filter

# Create a PassThrough filter object through z_axis.
passthrough_z = cloud_filtered.make_passthrough_filter()
# Assign axis and range to the passthrough filter object.
filter_axis = 'z'
passthrough_z.set_filter_field_name(filter_axis)
axis_min = 0.3
axis_max = 1.5
passthrough_z.set_filter_limits(axis_min, axis_max)
# Finally use the filter function to obtain the resultant point cloud. 
cloud_filtered = passthrough_z.filter()

# Create a PassThrough filter object through y_axis.
passthrough_y = cloud_filtered.make_passthrough_filter()
# Assign axis and range to the passthrough filter object.
filter_axis = 'y'
passthrough_y.set_filter_field_name(filter_axis)
axis_min = -0.4
axis_max = 0.4
passthrough_y.set_filter_limits(axis_min, axis_max)
# Finally use the filter function to obtain the resultant point cloud. 
cloud_filtered = passthrough_y.filter()

filename = 'pass_through_filtered.pcd'
pcl.save(cloud_filtered, filename)


# RANSAC plane segmentation
# Create the segmentation object
seg = cloud_filtered.make_segmenter()

# Set the model you wish to fit 
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)

# Max distance for a point to be considered fitting the model
# Experiment with different values for max_distance 
# for segmenting the table
max_distance = 0.025
seg.set_distance_threshold(max_distance)

# Call the segment function to obtain set of inlier indices and model coefficients
inliers, coefficients = seg.segment()

# Extract inliers

cloud_table = cloud_filtered.extract(inliers, negative=False)
filename = 'cloud_table.pcd'
pcl.save(cloud_table, filename)

cloud_object = cloud_filtered.extract(inliers, negative=True)
filename = 'cloud_objects.pcd'
pcl.save(cloud_object, filename)

outlier_filter = cloud_object.make_statistical_outlier_filter()
# Set the number of neighboring points to analyze for any given point
outlier_filter.set_mean_k(50)
# Set threshold scale factor
x = 0.5
# Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
outlier_filter.set_std_dev_mul_thresh(x)

# Finally call the filter function for magic
cloud_objects = outlier_filter.filter()
filename = 'clean_cloud.pcd'
pcl.save(cloud_objects, filename)


# Save pcd for tabletop objects



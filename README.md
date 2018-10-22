## Project: Perception Pick & Place
### Third project implementation for the Udacity Robotics Software Engineer Nanodegree - 3D Perception for Pick and Place operation
---
[//]: # (Image References)

[image1]: ./images/outlier_filter_1.PNG
[image2]: ./images/voxel_filter_1.PNG
[image3]: ./images/passthrough_filter.PNG
[image4]: ./images/ransac.PNG
[image5]: ./images/ransac_outlier_filter.PNG
[image6]: ./images/word1_training.PNG
[image7]: ./images/world1_accuracy.PNG
[image8]: ./images/world2_training.PNG
[image9]: ./images/world2_accuracy.PNG
[image10]: ./images/word3_training.PNG
[image11]: ./images/world3_accuracy.PNG
[image12]: ./images/detected_1.PNG
[image13]: ./images/detected_2.PNG
[image14]: ./images/detected_3.PNG
[image15]: ./images/point_cloud_world_1.PNG
[image16]: ./images/point_cloud_world_2.PNG
[image17]: ./images/point_cloud_world_3.PNG
[image18]: ./images/picked_object.PNG
[image19]: ./images/raised_object.PNG
[image20]: ./images/reached_place_box.PNG

# The steps taken for completing the perception project:
1. Clone the [project repo](https://github.com/udacity/RoboND-Perception-Project/) into the *catkin_ws* workspace, install missing dependencies (`rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y`) and build the workspace (`catkin_make`)
2. Train an SVM model on three set of objects, that will be present on three different setups (worlds) within the simulated environment.
3. Subscribe to the ROS topic `/pr2/world/points` for receiving the point cloud from the PR2 robot depth camera, affected by noise.
4. Apply filtering techniques, alongside RANSAC plane fitting to eliminate noise and separate the target objects from the rest of the envirnoment.
5. Apply DBSCAN Algorithm (Euclidian clustering) for grouping the point cloud into the objects and assign a corresponding label to each of them.
6. Compute the  spatial average of an object's points (centroid) to allow the pick and place operation to complete.
7. Change the **world scenario** and **pick list** in the `pick_place_project.launch` to test the obtained SVM models and obtain a high accuracy in each setup (usually 100% in world 1 and 2, and 87% in world 3).
8. Publish point cloud to the `/pr2/3d_map/points` instead of `/pr2/voxel_grid/points` in `sensors.yaml` to allow Moveit! to compute the robot's trajectory. Generate collision map by rotating PR2 in place so that it capture the table sides.
9. Request messages to the `pick_place_routine` service to plan the trajectory and make the robot move the objects in the corresponding box.
10. Save information about the detected objects, such as name, pick and place positions and picking arm.

---
## Detailed steps taken

#### 1. Pipeline for filtering and RANSAC plane.

a. To the initial noisy point cloud, an outlier removal filter was applied, with the parameters of `4` *neighbouring points* and a *scale factor* of `0.0001`, the result being as in the below image:

```
# Make statistical outlier filter
outlier_filter = cloud.make_statistical_outlier_filter()
# Set the number of neighboring points to analyze for any given point
outlier_filter.set_mean_k(4)
# Set threshold scale factor
x = 0.0001
# Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
 outlier_filter.set_std_dev_mul_thresh(x)
# Call  filter function
cloud_filtered = outlier_filter.filter()
```

![image1]

b. Given that the point cloud is too dense, Voxel Downsampling technique was applied next, with a *voxel size* of `0.01` and the resulting point cloud can be observed in the image.

![image2]

c. Since there are multiple surfaces that are not of interest in the scene, a series of passthrough filters was then used, on the **z**, **x** as well as **y** axis, each keeping only the point from tthe given interval, delimited by `axis_min` and `axis_max`:

```
###     PassThrough Filter along z
# Make passthrough filter
passthrough = cloud_filtered.make_passthrough_filter()
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
# Set minimun and maximum values for filtering the axis
axis_min = 0.6
axis_max = 1.1
passthrough.set_filter_limits(axis_min, axis_max)
# Finally use the filter function to obtain the resultant point cloud. 
cloud_filtered = passthrough.filter()

###     Apply PassThrough Filter again along x
passthrough = cloud_filtered.make_passthrough_filter()
filter_axis = 'x'
passthrough.set_filter_field_name(filter_axis)
axis_min = 0.3
axis_max = 1.0
passthrough.set_filter_limits(axis_min, axis_max)
cloud_filtered = passthrough.filter()

###     Apply PassThrough Filter along y as well, to remove table corners
passthrough = cloud_filtered.make_passthrough_filter()
filter_axis = 'y'
passthrough.set_filter_field_name(filter_axis)
axis_min = -0.455
axis_max = 0.455
passthrough.set_filter_limits(axis_min, axis_max)
cloud_filtered = passthrough.filter()
```

A point cloud similar to the one below was obtained.
![image3]

d. RANSAC plane segnentation was implemented, which fitted the model for the table. By extracting the negative of it, the objects were obtained, as in the image below:

![image4]

Since there were still some remaining points part of the table applying an additional outlier removal filter, the remaied points part of the table were removed, leaving the working point cloud as it follows:

![image5]

#### 2. Pipeline for clustering.  
The DBSCAN Algorithm, or Euclidian Clustering was used to perform the segmentention in distinct objects from the above filtered point cloud, which is based only on the spatial distance between two points in order to decide their belonging. As described in the section below, the cluster tolerance was set to 0.02, while the size of the cluster was between 100 and 1500 points.

```
###     Euclidean Clustering
# Apply function to convert XYZRGB to XYZ
white_cloud = XYZRGB_to_XYZ(cloud_objects) 
# Make a k-d tree for facilitating the neighbour's search
tree = white_cloud.make_kdtree()
# Create a cluster extraction object
ec = white_cloud.make_EuclideanClusterExtraction()
# Set tolerances for distance threshold as well as minimum and maximum cluster size (in points)
ec.set_ClusterTolerance(0.02)
ec.set_MinClusterSize(100)
ec.set_MaxClusterSize(1500)
# Search the k-d tree for clusters
ec.set_SearchMethod(tree)
# Extract indices for each of the discovered clusters
cluster_indices = ec.Extract()
```

Each object was represented using a different color, in each of the test worlds the following being obtained:

![image15]                 | ![alt text][image16]      | ![image17]
:-------------------------:|:-------------------------:|:-------------------------:
World 1                    |	World 2		                 |	World 3

#### 2. Features extraction and SVM training.  Object recognition pipeline.
The training and recognition was performed using histograms in the HSV color space, using around 50 positions of each object. In the three images below the training of the models was perfomed, the accuracy being around 95% in each case.

![image5]	![image7]

![image8]	![image9]

![image10]	![image11]

As for the recognition step, for each segmented object, HSV histogram was computed and it was compared with the model corresponding to each world. In the first and second scenarios all the objects were correctly identified, as in the following two pictures, whereas in the third world, only seven of the 8 were identified.

![image12]
![image13]
![image14]


### Pick and Place Setup

For all three tabletop setups (`test*.world`), object recognition was perfomed by reading the corresponding `pick_list_*.yaml` file. Consequently, a request to the `PickPlace` service was perfomed, so that the object is placed in the associated box.

![image18]
**Object being grasped by PR2 robot on the table**

![image19]
**Object being picked by PR2 robot from the table**

![image20]
**PR2 reached target box with the object**

In the end, information about the objects, such as its name, computed centroid (spatial average of its points along the axis), arm to be raised with, etc. is output them to the `output_*.yaml` file.


### Final Remarks

The SVM models have an accuracy over 95%, being trained with histograms within the HSV color space rather than RGB and using 50 different positions in the feature capturing phase for each object. I have observed that by using a considerably larger amount of positions did not reflect in the overall accuracy of the recognition.
The main problems that arise in the detection and classification step is rather how the filters and clustering are applied. For this reason, in the third world scenario, where the glue if right in the back of the notebook, even though the object might be properly recognised, it is completely removed from the point cloud, being considered noise.
In cases as such, of significant help might be a sequence of recognition trials, each being separated by a rotation of the robot, so that it can have a wider view of the world in front.
At the same time, a recognition process based only on color histograms might easily fail diftinct objects that have the same colors, which is why other more relevant features, such as shape or texture should be used for a reliable detection.




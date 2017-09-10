## Launch files README

This folder contains low-level launch files for the 'ucl_drone' package.

These files are called by higher level launch files.


#### Files description

 * `driver.xml` Launches ardrone autonomy node
 * `controller.xml` Launches the controller, pathplanning and strategy nodes
 * `slam.xml` Launches image_proc, computer vision, mapping and bundle adjustment nodes.
 * `gui.xml` Launches vision gui node
 * `global_params.xml` Does not launch nay nodes, but contains parameters used by multiple other files

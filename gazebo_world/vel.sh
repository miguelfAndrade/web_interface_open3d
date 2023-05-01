cd build
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/velodyne_plugin/build
gazebo --verbose ../velodyne_2.world
# gazebo --verbose ../velodyne_animation.world
echo 'nv' | sudo -S chmod 777 /dev/tty* & sleep 1;
roslaunch mavros px4.launch & sleep 6;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
roslaunch realsense2_camera rs_camera.launch & sleep 10;
roslaunch vins vins_d435.launch & sleep 5;
roslaunch diff_planner exp_rviz_test.launch & sleep 1;
wait;

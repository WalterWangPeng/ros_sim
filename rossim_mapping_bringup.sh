tmux source ~/.tmux.conf
tmux new -d -s rossim_mapping -n empty
#空白窗口：用户自由使用
tmux split-window -h -t rossim_mapping:empty
tmux split-window -v -t rossim_mapping:empty.1
tmux split-window -v -t rossim_mapping:empty.0
#机器人显示和控制
tmux new-window -n sim -t rossim_mapping
tmux split-window -h -t rossim_mapping:sim
tmux split-window -v -t rossim_mapping:sim.1
tmux split-window -v -t rossim_mapping:sim.0
#gmapping
tmux new-window -n gmapping -t rossim_mapping
tmux split-window -h -t rossim_mapping:gmapping
tmux split-window -v -t rossim_mapping:gmapping.1
tmux split-window -v -t rossim_mapping:gmapping.0
#地图构建启动
echo "now start roscore"
tmux send-keys -t rossim_mapping:sim.0 'roscore' C-m
echo "please wait 1 seconds for next step.."
sleep 1
echo "now start rviz"
tmux send-keys -t rossim_mapping:sim.1 'source ~/ros_sim/devel/setup.bash && roslaunch ayuan_robot ayuan_robot_slam_sim.launch' C-m
echo "please wait 1 seconds for next step.."
sleep 1
# echo "now start gazebo"
# tmux send-keys -t rossim_mapping:sim.2 'source hy_route/learn/ros_sim/devel/setup.bash && roslaunch ayuan_gazebo ayuan.launch' C-m
# echo "please wait 1 seconds for next step.."
# sleep 1
echo "now start teleop_robot"
tmux send-keys -t rossim_mapping:sim.3 'source ~/ros_sim/devel/setup.bash && rosrun teleop_robot teleop_robot_key.py' C-m
echo "please wait 1 seconds for next step.."
sleep 1
tmux send-keys -t rossim_mapping:gmapping.0 'source ~/ros_sim/devel/setup.bash && roslaunch gmapping ayuan_slam_gmapping.launch' C-m
tmux send-keys -t rossim_mapping:gmapping.1 'rosrun map_server map_saver -f ~/ros_sim/src/ayuan_gazebo/map/<you_map_name>'
tmux a -t rossim_mapping:sim
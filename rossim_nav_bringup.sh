tmux source ~/.tmux.conf
tmux new -d -s rossim_nav -n empty
#空白窗口：用户自由使用
tmux split-window -h -t rossim_nav:empty
tmux split-window -v -t rossim_nav:empty.1
tmux split-window -v -t rossim_nav:empty.0
#机器人nav
tmux new-window -n sim -t rossim_nav
tmux split-window -h -t rossim_nav:sim
tmux split-window -v -t rossim_nav:sim.1
tmux split-window -v -t rossim_nav:sim.0
#机器人nav
tmux new-window -n nav -t rossim_nav
tmux split-window -h -t rossim_nav:nav
tmux split-window -v -t rossim_nav:nav.1
tmux split-window -v -t rossim_nav:nav.0
#sim start
echo "now start roscore"
tmux send-keys -t rossim_nav:sim.0 'roscore' C-m
echo "please wait 1 seconds for next step.."
sleep 1
# echo "now start rviz"
# tmux send-keys -t rossim_nav:sim.1 'source hy_route/learn/ros_sim/devel/setup.bash && roslaunch ayuan_description ayuan_nav.launch' C-m
# echo "please wait 1 seconds for next step.."
# sleep 1
# echo "now start gazebo"
# tmux send-keys -t rossim_nav:sim.2 'source hy_route/learn/ros_sim/devel/setup.bash && roslaunch ayuan_gazebo ayuan.launch' C-m
# echo "please wait 1 seconds for next step.."
# sleep 1
echo "now start sim"
tmux send-keys -t rossim_nav:sim.1 'source ~/ros_sim/devel/setup.bash && roslaunch ayuan_robot ayuan_robot_nav_sim.launch' C-m
echo "please wait 1 seconds for next step.."
sleep 1
echo "now start teleop_robot"
tmux send-keys -t rossim_nav:sim.3 'source ~/ros_sim/devel/setup.bash && rosrun teleop_robot teleop_robot_key.py'
echo "please wait 1 seconds for next step.."
sleep 1
# nav start
tmux send-keys -t rossim_nav:nav.0 'source ~/ros_sim/devel/setup.bash && roslaunch ayuan_robot ayuan_robot_nav.launch' C-m
# echo "please wait 1 seconds for next step.."
# sleep 1
# tmux send-keys -t rossim_nav:nav.1 'source hy_route/learn/ros_sim/devel/setup.bash && roslaunch ayuan_robot ayuan_robot_nav_movebase.launch' C-m
tmux a -t rossim_nav:nav
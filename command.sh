gnome-terminal -e "bash -c 'echo move_to_start && cd ~/franka && source devel/setup.bash && roslaunch franka_example_controllers move_to_start.launch robot_ip:=\"10.0.0.2\" load_gripper:=false rviz:=true && sleep 1000'" &
gnome-terminal -e "bash -c 'echo stockfish && cd /home/alp/bitirme/ChessMate && source devel/setup.bash && cd /home/alp/bitirme/ChessMate/src/chessmate/scripts && python3 stockfish_driver.py && sleep 1000'" &
gnome-terminal -e "bash -c 'echo vision && cd /home/alp/bitirme/ChessMate && source devel/setup.bash && cd /home/alp/bitirme/ChessMate/src/chessmate/scripts/vision && python3 vision_bridge.py && sleep 1000'" &
gnome-terminal -e "bash -c 'echo arduino && cd /home/alp/bitirme/ChessMate && source devel/setup.bash && cd /home/alp/bitirme/ChessMate/src/chessmate/scripts/ && python3 arduino-driver.py && sleep 1000'" &
gnome-terminal -e "bash -c 'echo chessmate && cd /home/alp/bitirme/ChessMate && source devel/setup.bash && rosrun chessmate chessmate && sleep 1000'" &
gnome-terminal -e "bash -c 'echo hri component && cd ~/franka && source devel/setup.bash && cd ~/franka/src/Franka-Panda/franka_example_controllers/scripts/ && python3 hri_component.py && sleep 1000'" &

gnome-terminal -e "bash -c 'cd ~/franka && source devel/setup.bash && rosrun franka_gripper franka_gripper_service 10.0.0.2 0 0 && sleep 1000'" &
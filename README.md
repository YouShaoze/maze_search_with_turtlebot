# maze_search_with_turtlebot
Solving a maze in gazebo using ros with BFS, UCS, Astar and GBFS

Instructions:
1) Make sure you have ros kinetic is preinstalled. Run setup.sh.
2) Change path as instructed while runnning setup.sh Go to catkin workspace.

Every time you open a new terminal, make sure to do : source devel/setup.bash from catkin workspace
1) Run roscore
2) Run 'rosrun search server.py' ( use --help for more details)
3) Run "export TURTLEBOT3_MODEL=burger" in new terminal and  run 'roslaunch search maze.launch'
4) 'roslaunch search maze.launch ' will be run to start the gazebo model
5) Run 'rosrun search move_tbot3.py' to control robot
13) 'rosrun search search_algorithm.py -a <algo name(bfs,ucs,astar,gbfs)>

#unzip search.zip
mkdir search_problem
cd search_problem
mkdir src
cd src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
mv ../../search .
cd search/scripts
chmod a+x *
cd ../worlds
pwd
echo "Copy this path and replace this with existing path in gen_maze.py!!!"
cd ../../..
catkin_make

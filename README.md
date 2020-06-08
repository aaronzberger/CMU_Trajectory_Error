# CMU_trajectory_error
For ROS robots: get an overview of the error in the robot's position and orientation, reading desired location from a csv file and actual location from a bag file.

To make this program accessible on your computer, clone this repository to your catkin workspace and compile.

To run this program, use the following command:
  rosrun trajectory_error error [csv file path] [bag file path]

Running the program with the above command will output all calculated data and analysis to the terminal.
It will also generate two files in your working directory:
  1) A csv file with all calculated data and analysis. The file name ends with _error.csv
  2) A csv file with just the calculated errors for each time stamp; this is used for graphing. The file name ends with _error_graph_data.csv

After running the program with the above command, use gnuplot to graph the data for visualization.

To install gnuplot on Ubuntu, first update your system:
  sudo apt-get update
  
Next, install gnuplot:
  sudo apt-get install gnuplot
  
A gnuplot script is attached in this repository. To run it, use the following command:
  gnuplot -c make_error_graphs.gnu [error_graph_data.csv file path]

Gnuplot will open two windows containing one graph each.

If you run into any problems when compiling or executing, please contact me:
  aaronzberger@gmail.com

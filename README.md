# Project Title

This is a ros2 package to drive the AWS deepracer simualtion gazebo model autonomously for the medium track using OpenCV and deeplearing


### Prerequisites

  * aws-robomaker-sample-application-deepracer (https://github.com/aws-robotics/aws-robomaker-sample-application-deepracer/tree/ros2)
  * Knowledge of ROS2
  * Install OpenCV
  * To use deep learning keras and tensorflow must be install along with their depencies e.g numpy, 
sklearn etc

## Getting Started

 * Clone the package to your ros2 workspace
 * Build the package (```colcon build --symlink-install --packages-select deepracer_runner```)
 * Source the worksapce (```sourec install/local_setup.bash```)


### Running

  * Start the Gazebo deepracer simualtion using the racecar_medium_track world. On my box I use ```ros2 launch deepracer_simulation racetrack_with_racecar.launch.py gui:=true world:=src/deepracer_simulation/worlds/racecar_medium_track.world```
  * Open a terminal and source your ros2 worksapce
  * Type ```ros2 run deepracer_runner cv_runner``` to start the node
  * Type ```ros2 run deepracer_runner cv_runner --help``` to check the optional arguments to the node
  
  [![Demo Of the simualtion ruuning on youtube](https://drive.google.com/file/d/1qPBWKu4LKlq_G-8hIpMf_aB_I4yi1O5Z/view?usp=sharing)](https://www.youtube.com/watch?v=7tyUCWxltQM)

## Built With

* [ROS](https://github.com/ros2/ros2) - Robot Operating System

## Contributing

## Authors

* **Kayode Emmnauel O.** - (https://github.com/tobi007)


## License

## Acknowledgments

* Thanks to **David Tian** for the post on DeepPiCar — Part 1: How to Build a Deep Learning, Self Driving Robotic Car on a Shoestring Budget (https://towardsdatascience.com/deeppicar-part-1-102e03c83f2c). Most of the code were form the post with and some few modeifications to suit ROS2.
* **Billie Thompson** - README template used - [PurpleBooth](https://github.com/PurpleBooth)



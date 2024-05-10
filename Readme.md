ROS2 + environmentin stetup %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
DEV + BOT         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


* UPDATE

    sudo apt update && sudo apt upgrade -y


* INSTALL NECESSARY APPS - GIT vsCODE PYTHON PIP

	sudo apt install git idle3 python3-pip -y

	sudo snap install code -y 


* CHECK FOR UTF-8

    locale


* FIRST ENSURE THAT UBUNTU UNIVERSE REPOSITORY IS ENABLED

    sudo apt install software-properties-common -y

    sudo add-apt-repository universe


* ADD ROS 2 GPG KEY

    sudo apt update && sudo apt install curl -y

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


* ADD REPOSITORY TO YOUR SOURCE LIST

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


* INSTALL ROS 2 PACKAGES

    sudo apt update && sudo apt upgrade -y

    sudo apt install ros-humble-desktop -y

	sudo apt install python3-colcon-common-extensions -y



Environment setup test %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
DEV                       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


* UPDATE

    sudo apt update && sudo apt upgrade -y


* ADD SOURCE TO YOUR BASHRC

	 echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

   	 source /opt/ros/humble/setup.bash

   	 ros2


* GIT CLONE PROJECT GUI

   	 cd ~

   	 mkdir -p piper_dev

   	 cd piper_dev

   	 git clone https://github.com/Dezinter8/Piper_Gui


* CREATE PYTHONIRTUAL ENV
	
	sudo apt update && sudo apt install  python3-virtualenv -y

   	 cd ~/piper_dev/Piper_Gui

	virtualenv venv
 
	source venv/bin/activate

   	pip install PyQt5 PyQt5-tools vtk opencv-python-headless

* INSTALL CAMERA PACKAGES

	sudo apt install ros-${ROS_DISTRO}-rqt-image-view -Y

	sudo apt install ros-${ROS_DISTRO}-image-transport-plugins


	 
Project commands %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


* Run simulation (Gazebo) - BOT 

   	cd ~/piper_ws

   	source install/setup.bash

   	ros2 launch piper_bot launch_sim.launch.py world:=./src/piper_bot/worlds/pipe.world


* Run gui application - DEV 

   	 cd ~/piper_dev/Piper_Gui

	 source venv/bin/activate

	python3 main.py


* Controll robot movement - BOT / DEV 

   	 ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped


* Running camera node - BOT 

	ros2 run v4l2_camera v4l2_camera_node


* Reciving and displaying photos - DEV

	ros2 run rqt_image_view rqt_image_view


* Display ros2 nodes - BOT / DEV
	
	ros2 node list


* Display ros2 topic list  - BOT / DEV

	ros2 topic list


* Display ros2 topic message - BOT / DEV

	ros2 topic echo <topic name>


* Display ros2 detailed topic info - BOT / DEV

	$ ros2 topic info <topic name>

	$ ros2 interface show <Type:>


QT Designer
    
* Converting ui file to python
	$ pyuic5 mainwindow.ui -o MainWindow.py

* Converting resources to python
	$ pyrcc5 resources.qrc -o resources_rc.py








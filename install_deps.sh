# dependency for pyaudio
sudo apt-get install libasound-dev portaudio19-dev
sudo pip install pyaudio
sudo pip install audiolazy
sudo pip install pydub
sudo pip install pyttsx
sudo apt-get install ffmpeg

# install pcl
sudo apt-get install libpcl1.7*
mkdir ~/Apps
cd ~/Apps
git clone https://github.com/strawlab/python-pcl.git
sudo pip install Cython=0.21.2
cd python-pcl
make
sudo python setup.py install

# install ros dependencies in catkin_ws 
# if already installed, will print an error but won't do anything bad
cd ~/catkin_ws/src
git clone https://github.com/RIVeR-Lab/apriltags_ros.git
git clone https://github.com/lrse/ros-keyboard.git
sudo apt-get install libsdl-image1.2-dev # SDL dependency for ros-keyboard
cd ~/catkin_ws
catkin_make

# return to parent directory
roscd mobility_games/..

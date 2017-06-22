# dependency for pyaudio
sudo apt-get install libasound-dev portaudio19-dev
sudo pip install pyaudio
sudo pip install audiolazy
sudo pip install pydub
sudo pip install pyttsx
sudo apt-get install ffmpeg

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

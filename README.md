Установка Ardupilot

cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout Copter-4.4
git submodule update --init --recursive
sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect
sudo apt install pip3
python3 -m pip install -U pip
python3 -m pip install -U matplotlib
python3 -m pip install scipy
sudo pip install future pymavlink MAVProxy
gedit ~/.bashrc
----------------
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
----------------
. ~/.bashrc
sudo ln -s /usr/bin/python3 /usr/bin/python
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
------------------------------------------------------------------------------------------

Установка QGC

sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage
------------------------------------------------------------------------------------------

Установка Gazebo и ArduPilot Plugin

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt-get install gazebo11 libgazebo11-dev
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world

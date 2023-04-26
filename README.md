# Gazebo World & Web Interface
World for gazebo robot simulator, which contains a lidar sensor that rotates at a constant speed. The world has also a 3D model of a cafeteria.

### Steps (Only works on linux)
- Install Gazebo - https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install
- Install the development package `sudo apt install libgazebo11-dev`
- Install python3 and pip3
- Instal open3D with pip3
- Create a folder named `models` in the folder `.gazebo` and copy the folder `velodyne_hdl-32` to that location
- Create a new directory in the gazebo_world folder named **build** by running the command `mkdir build`
- Inside the **build** directory run `cmake ..` `make`
- Run the command `./vel.sh` in the root of the project
- Open a new terminal and run the command `./vel_data` inside the **build** directory
- Run `python3 app.py` in the root of the project
- Go to `localhost::8080` in the browser

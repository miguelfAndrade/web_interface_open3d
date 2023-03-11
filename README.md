# Gazebo World & Web Interface
World for gazebo robot simulator, which contains a lidar sensor that rotates at a constant speed. The world has also a 3D model of a cafeteria.

### Steps (Only works on linux)
- Install Gazebo - https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install
- Install the development package `sudo apt install libgazebo11-dev`
- Create a new directory in the root of the project named **build** by running the command `mkdir build`
- Inside the **build** directory run `cmake ..` `make`
- Run the command `./vel.sh` in the root of the project
- Open a new terminal and run the command `./vel_data` inside the **build** directory

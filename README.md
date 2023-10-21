# Web interface to visualize, interact and modify 3D geometry created from a simulated LiDAR sensor point cloud
My dissertation project, **Development of a 3D visualisation system for a robotic simulator**, which consist in a simulated LiDAR sensor created in Gazebo and a web interface to view in 3D the information retrieved from the sensor.


### Steps
**DISCLAIMER**: I've only tested this in Linux, more precisely in [Linux Mint 21.2](https://linuxmint.com/edition.php?id=306). However you can try to adapt the steps for Windows or Mac.

To run the project in your local machine go through the following steps.

#### Dependencies
First we need to have some dependencies installed.

- Install **CMake**
  - `sudo apt install cmake`

- Install **Python 3.10** and **pip3** (In the [Linux Mint 21.2](https://linuxmint.com/edition.php?id=306) Python 3.10 is already installed)
  - `sudo apt install python3.10`
  
  - `sudo apt install python3-pip`

- Install [Gazebo](http://classic.gazebosim.org/) and the **Gazebo development package** (This project uses the ***Gazebo Classic***)
  - `sudo apt install gazebo`
  
  - `sudo apt install libgazebo-dev`

- Install [Open3D](http://www.open3d.org/) and [Flask](https://flask.palletsprojects.com/en/3.0.x/) via pip
  - `pip3 install open3d`
  
  - `pip3 install Flask`

#### Run the project

- Go to the ***home directory*** and locate the ***.gazebo*** folder (This folder is hidden and you need to press **CTRL+H** to be able to see it).
- Create a folder named ***models*** inside the folder ***.gazebo*** and copy the folder ***vehicle*** to that location (This folder ***vehicle*** is inside the ***gazebo_world*** directory).

- Open a new terminal inside the ***gazebo_world*** directory.
- Create a new folder called **build** `mkdir build`.
- Inside the **build** directory run `cmake ..` and after run `make`
- Run the command `./vel.sh` in the root of the project
- Open a new terminal and run `python3 app.py` in the root of the project
- Go to `localhost::8080` in the browser

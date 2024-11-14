# RI 24/25 - Assignment 1

## Group members

- Christopher Schneider - <up202401397@fe.up.pt>
- Toni Grgurevic - <up202408625@up.pt>
- Juan Camilo Pacheco - <up202402591@fe.up.pt>

## Directory structure

```txt
 ├── tilde 
      ├── experiments           
          ├── experiment1
          ├── experiment2
          ├── experiment3
          ├── experiment4
          ├── experiment5
          └── code
              └── plot.py
      ├── launch               
          └── tilde.launch.py
      ├── param
          └── params.yaml
      ├── resource
          └── tilde
      ├── rviz
          └── robot_navigation.rviz
      ├── tilde
          └── __init__.py
      ├── world
          ├── layer.yaml
          ├── second.model.yaml
          ├── serp.model.yaml
          ├── tilde.png
          └── world.yaml          
      ├── Readme.md            
      ├── package.xml
      ├── setup.cfg
      └── setup.py
```

## Requirements

- Ros2 Humble 
- Flatland

## Setup and run

1. `git clone https://github.com/ToniGrgurevic/ri.git` into your
   workspace's `src` directory
2. Install the required dependencies using
   `rosdep install -i --from-path src --rosdistro humble -y`
3. `colcon build`
4. `source install/setup.bash`
5. `ros2 launch tilde tilde.launch.py`



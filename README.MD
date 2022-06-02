# Panda clik demo

[![N|Solid](https://www.unicampania.it/doc/img/logo_vanvitelli.jpg)](https://www.ingegneria.unicampania.it/roboticslab)

Demo rete ros con generatori di traiettoria e inversione cinematica (CLIK) per il corso di Programmazione dei Robot.

## Install

- Prerequisiti: Installare Moveit!

Nella cartella src del ros workspace
```bash
gitclone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```

## Avviare la demo

Avviare il rosmaster:
```bash
roscore
```

In un nuovo terminale avviare il simulatore e i server del clik e generatori di traiettoria:
```bash
roslaunch panda_clik_demo demo.launch
```

In un nuovo terminale avviare il nodo task:
```bash
rosrun panda_clik_demo task_node
```

## License

GNU General Public License v3.0

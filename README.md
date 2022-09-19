## Startreihenfolge
auf Roboter:
1. mur_sb mur.launch

auf Master (/dev/sdb3 fuer altes SKript):
2. mur_sb ur_start_pose.py wird mit launch aus bauschaum-package gestartet (jetzt auch "roslaunch mur_sb trajectory_generation.launch")
3. mur_sb ur_execute_trajectory

## Infos
Testvorgabe Objekt fuer ur_start..: scripts/publisher wall

Noch math domain errors bei schelchter Ausrichtung UR (acos?)


## Simulation
```
roslaunch mur_sb mur_sim.launch
roslaunch mur_sb trajectory_generation_circle.launch
rosrun mur_sb ur_execute_trajectory.py
rosrun mur_sb pub_circle_pose.py
```

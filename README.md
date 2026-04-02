**DO THE FOLLOWING TO GET ALL REQUIRED REPOS INCUDING SUBMODULSAND RECURSIVE MODULES(see setup guide for OMY):**

```
git clone https://github.com/kiashik/throw_and_catch.git
cd throw_and_catch
git submodule update --init --recursive
```

How to update a specific submodule (replace submodule name):
```
cd throw_and_catch_ws/src/image_pipeline
git pull origin jazzy    # specific branch name is jazzy
cd ../../..
git add throw_and_catch_ws/src/image_pipeline
git commit -m "Update image_pipeline submodule"
```

Remember to manually modify the the apprriate files:
```	
open_manipulator_bringup/config/omy_f3m/hardware_controller_manager.yaml
    allow_nonzero_velocity_at_trajectory_end: true    # ashik added this. Allows the controller to end a trajectory with nonzero velocity, which maybe important for our use case of 
    # tracking a moving target. If false, the controller will try to decelerate to zero velocity at the end of each trajectory, which can cause issues when trying to track a moving target.

# May need to make small modification to the followign two files:
open_manipulator_bringup/launch/omy_f3m_gazebo.launch.py
modified:   open_manipulator_moveit_config/launch/omy_f3m_moveit.launch.py
```

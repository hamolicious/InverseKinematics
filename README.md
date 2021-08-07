## InverseKinematics
Inverse Kinematics implementation in Python and PyGame

![screenshot](https://github.com/hamolicious/InverseKinematics/blob/master/screenshots/screen_recording.gif?raw=true)

# Usage
```python
# import object from ik.py
from ik import InverseKinematicsActor2D, Constraints
# create an inverse kinematics object
arm = InverseKinematicsActor2D(pos=(300, 300), limb_lengths=[100, 100, 100])
# add constraints to limbs
arm[0].bones[0].constraint = lambda angle : Constraints.clamp(250, 350, angle)
# updating the object re-calculates limb orientations
arm.update()
# pass in a pygame `Surface` object to be drawn to
arm.display(screen)
```




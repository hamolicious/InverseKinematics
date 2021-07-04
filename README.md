## InverseKinematics
Inverse Kinematics implementation in Python and PyGame

![screenshot](https://github.com/hamolicious/InverseKinematics/blob/master/screenshots/screen_recording.gif?raw=true)

# Usage
```python
# import object from ik.py
from ik import InverseKinematicsActor
# create an inverse kinematics object
arm = InverseKinematicsActor(pos=(300, 300), limb_length=50, num_of_limbs=3)
# updating the object re-calculates limb orientations
arm.update()
# pass in a pygame `Surface` object to be drawn to
arm.display(screen)
```




# Marker-Assisted Ballistic Acquisition Low-Latency Servoing

![catch.py throw plot](docs/media/maballs2.png)



**M**arker-**A**ssisted: the ball and the robot both wear OptiTrack markers, so the mocap system tracks the ball's flight and the robot's own base/tool frames in the same space, continuously.

![throw](docs/media/throw.gif)

**B**allistic **A**cquisition: ball positions are fit to a projectile trajectory and intersected with a fixed catch plane to acquire a single intercept point and time.

![trajectory acquisition](docs/media/throw_acquisition.gif)

![trajectory fit converging with sample count](docs/media/malballs3.png)

**L**ow-**L**atency: actuator speed and move-time limits are measured on the real arm, so it only commits to a catch once it can provably beat the ball there.

**S**ervoing: once committed, target poses stream to the arm at 125Hz, letting it keep refining its aim as the trajectory prediction sharpens.

![servoj retarget, slow motion](docs/media/servo_retarget_slowmo.gif)
*slow motion — arm direction updating as the prediction converges*

## Live visualization

`catch.py` renders each throw's actual ball path, the arm's TCP path, and every commit/re-aim point, time-colored, in real time. After throw has been detected to end a sound que reveals if the system is confident it managed to catch the projectile.

![catch.py throw plot](docs/media/maballs1.png)

## Status

Working end to end on real hardware. Best measured catch rate over 62 throws: 81%.

## Setup

Running this on a fresh machine: **[docs/demo_setup_new_laptop.md](docs/demo_setup_new_laptop.md)** — a step-by-step install, network, Motive/robot checklist and run guide for the UR10 rig, written so someone other than the author can show the demo.

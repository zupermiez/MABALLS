# Marker-Assisted Ballistic Acquisition with Low-Latency Servoing

A UR12e arm catches thrown objects in flight, tracked and guided by OptiTrack motion capture:

Marker-Assisted: the ball and the robot both wear OptiTrack markers, so the mocap system tracks the ball's flight and the robot's own base
frames in the same space, continuously.

throw (docs/media/throw.gif)
ball mid-flight, markers visible

Ballistic Acquisition: ball positions are fit to a projectile trajectory and intersected with a fixed catch plane to acquire a single intercept
point and time.

Low-Latency: actuator speed and move-time limits are measured on the real arm, so it only commits to a catch once it can provably beat the ball
there.

Servoing: once committed, target poses stream to the arm at 125Hz, letting it keep refining its aim as the trajectory prediction sharpens.

servoj retarget (docs/media/servo_retarget_slowmo.gif)
slow motion — arm direction updating as the prediction converges

Status

Working end to end on real hardware. Best measured catch rate over 62 throws: 81%.

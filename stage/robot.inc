define kinect ranger
(
  sensor
  (
    fov 180.0
    samples 640
    range [0.2 10]
  )
  # generic model properties
  color "black"
  size [ 0.0 0.0 0.0 ]
)

define robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  
  localization "odom"

  odom_error [0.0 0.0  0.00  0.0]

  size [ 0.6 0.4 0.0 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  kinect(pose [ 0.0 0.0 0.0 0.0 ])
)

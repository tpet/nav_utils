# Nodes

## `odom_proc`
Splits odometry into two parts, one of which is resolved via *tf*.<br>
It allows to rename parent frame. 

## `odom_to_tf`
Converts odometry messages to *tf* transforms.

## `tf_to_odom`
Converts *tf* transforms to odometry messages.<br>
Allows two-phase transform lookup with the root using latest transform without waiting.
This is useful e.g. for fast *map*-to-*base_link* odometry from slow *map*-to-*odom* transforms.

## `tf_fast_repeater`
Use this if a transform you have needs to be republished on a higher frequency
always with current timestamp (to allow constant extrapolation of the transform).
Takes `parent_frame`->`child_frame` transform and republishes its inverse as
`child_frame`->`new_parent_frame` on `publish_frequency`, always taking the
latest available transform and stamping it with current time.

# Nodelets

As above, in nodelets.

## `nav_utils/odom_to_tf`

## `nav_utils/odom_proc`

## `nav_utils/tf_to_odom`

## `nav_utils/tf_fast_repeater`
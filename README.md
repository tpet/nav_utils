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

## `tf_copy`
Copy a transform to another place in the tree. Reads `parent_frame`->`child_frame`
TF and publishes it as `new_parent_frame`->`new_child_frame`. If `invert_tf` is true,
then the transforms gets inverted (just the geometry, not the frame names).
If `static_tf` is true, the transform is only published when it actually changes,
and is published by a static transform publisher. `publish_frequency` specifies the
frequency on which the TF is updated, and if it is nonstatic, the also published.
`error_if_tf_missing` specifies, whether ROS should issue an error in console if the source
TF is missing.

# Nodelets

As above, in nodelets.

## `nav_utils/odom_to_tf`

## `nav_utils/odom_proc`

## `nav_utils/tf_to_odom`

## `nav_utils/tf_fast_repeater`

## `nav_utils/tf_copy`
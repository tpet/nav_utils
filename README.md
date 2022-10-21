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

## `tf_to_path`
Converts *tf* transforms to path messages.<br>
Allows out-of-order transform messages to come and adds older measurements to the correct
place in the path. The individual path points are looked up at times when the `stamp_trigger_frame`
frame is updated on the `tf_msg` topic (the idea is that points on the path have the very same timestamps as
your robot's odometry). Do not forget to remap `tf_msg` to the actual TF topic. `stamp_trigger_frame`
is by default the same as `child_frame`, but you have to set it different e.g. if your `child_frame`
is a static transform to the actual top-level body link.

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

## `tf_filter` (Python)

The node filters TF messages by
 - including / excluding parent frames,
 - including / excluding child frames,
 - limiting publishing frequency.

It subscribes topic `tf` and publishes to `tf_filtered`.

It allows collecting and republishing transforms in regular intervals.

## `tf_connect` (Python)

The `tf_connect` node allows to connect multiple disconnected TF trees by defining coincident frames.

## `odom_twist_to_child_frame`

Some robot drivers incorrectly publish twist in odom frame instead of body frame. This node fixes it.

## `odom_recompute_twist`

Some robot drivers do not publish twist or publish a wrong one. This node computes it from differences of the poses.

# Nodelets

As above, in nodelets.

## `nav_utils/odom_to_tf`

## `nav_utils/odom_proc`

## `nav_utils/tf_to_odom`

## `nav_utils/tf_fast_repeater`

## `nav_utils/tf_copy`
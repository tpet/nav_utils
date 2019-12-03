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

# Nodelets

As above, in nodelets.

## `nav_utils/odom_to_tf`

## `nav_utils/odom_proc`

## `nav_utils/tf_to_odom`
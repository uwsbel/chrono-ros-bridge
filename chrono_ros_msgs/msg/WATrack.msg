# ROS msg representation of a WATrack
# May be an entire track or just at one instance

# -----------------------
# Instantaneous detection
# -----------------------

# List of 3D points that define the left and right boundaries of the track
geometry_msgs/Point[] left_visible_points
geometry_msgs/Point[] right_visible_points

# -------
# Mapping
# -------

# List of 3D points that define the centerline of the track
# Also has the left and right widths to calculate the road boundaries
# Points are defined relative to the start line of the track and in global GPS coordinates
geometry_msgs/Point[] mapped_coords # x=latitude,y=longitude,z=altitude
geometry_msgs/Point[] mapped_points
geometry_msgs/Point[] mapped_widths # x=left width,y=right width,z=0

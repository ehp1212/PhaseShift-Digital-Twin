# ----------------------------------------
# VoxelChange.msg
#
# - Provide interpretable geometry features
# - Enable downstream semantic + state estimation
# - Visualize dynamic vs static behavior in 3D space
# ----------------------------------------

# Center position of voxel
geometry_msgs/Point position

# Confidence from static voxel map
# Range: [0.0 ~ 1.0]
float32 static_confidence

# Number of consecutive frames this voxel has been observed
uint16 persistence

# Number of neighboring voxels
uint16 neighbor_count

# Combined dynamic likelihood score
# dynamic_score = normalized_persistence * (1 - static_confidence)
# Range: [0.0 ~ 1.0]
float32 dynamic_score

# Temporal stability of the voxel
float32 temporal_stability
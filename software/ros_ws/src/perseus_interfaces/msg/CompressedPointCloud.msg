# A voxel-downsampled Livox point cloud with positions Draco-compressed.
#
# `draco_data` holds only x/y/z, encoded with DracoPy.encode(..., preserve_order=True) so
# decoded point N corresponds to timestamps[N]/tags[N]/lines[N] below. Intensity is dropped.
# quantization_bits records the setting used to encode draco_data, so a decoder doesn't need
# to know it out of band.

std_msgs/Header header
uint8[] draco_data
uint8 quantization_bits
float64[] timestamps
uint8[] tags
uint8[] lines

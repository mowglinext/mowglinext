std_msgs/Header header
# Interleaved map-frame x,y coordinates, stored as float32: x0,y0,x1,y1,...
float32[] xy
# Point-index offsets delimiting independent paths; starts with 0 and ends at xy.size()/2.
uint32[] subpath_offsets

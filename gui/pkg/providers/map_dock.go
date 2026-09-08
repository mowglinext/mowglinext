package providers

import "github.com/mowglinext/mowglinext/pkg/msgs/mowgli"

// addDockPose preserves absence until the map server supplies a pose. Copy the
// values under the cache lock; pointers must not alias the live mutable cache.
func (r *RosProvider) addDockPose(data *mowgli.Map) {
	r.mtx.Lock()
	defer r.mtx.Unlock()
	if !r.dockPoseSet {
		return
	}
	x, y, heading := r.dockX, r.dockY, r.dockHeading
	data.DockX, data.DockY, data.DockHeading = &x, &y, &heading
}

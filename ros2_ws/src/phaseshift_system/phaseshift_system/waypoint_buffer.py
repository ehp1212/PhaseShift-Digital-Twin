from geometry_msgs.msg import PoseStamped

class WaypointBuffer:

    def __init__(self, max_size=50, min_distance=0.5):
        self._buffer = []
        self._max_size = max_size
        self._min_distance = min_distance

    def add(self, pose: PoseStamped):
        if not self._buffer:
            self._buffer.append(pose)
            return

        last = self._buffer[-1]

        if self._distance(pose, last) > self._min_distance:
            self._buffer.append(pose)

            if len(self._buffer) > self._max_size:
                self._buffer.pop(0)

    def get_previous(self):
        if len(self._buffer) < 2:
            return None
        return self._buffer[-2]

    def clear(self):
        self._buffer.clear()

    def _distance(self, p1, p2):
        dx = p1.pose.position.x - p2.pose.position.x
        dy = p1.pose.position.y - p2.pose.position.y
        return (dx**2 + dy**2) ** 0.5
import time


class PipelineStep:
    def __init__(self, node):
        self.node = node
        self._entered = False

    def on_enter(self):
        pass

    def run(self) -> bool:
        raise NotImplementedError

    def on_exit(self):
        pass

    def tick(self) -> bool:
        if not self._entered:
            self.on_enter()
            self._entered = True

        done = self.run()

        if done:
            self.on_exit()

        return done

    def reset(self):
        self._entered = False


# ======================================================
# TIMEOUT STEP (FIXED)
# ======================================================

class TimeoutStep(PipelineStep):

    def __init__(self, node, step: PipelineStep, timeout_sec: float, on_timeout=None):
        super().__init__(node)
        self.step = step
        self.timeout_sec = timeout_sec
        self.on_timeout = on_timeout

        self._start_time = None
        self._timed_out = False

    def on_enter(self):
        self._start_time = time.time()
        self._timed_out = False
        self.step.reset()

    def run(self):

        if self._timed_out:
            return True

        if self.step.tick():
            return True

        if (time.time() - self._start_time) > self.timeout_sec:
            self.node.get_logger().error(
                f"[PIPELINE] Timeout in {self.step.__class__.__name__}"
            )

            self._timed_out = True

            if self.on_timeout:
                self.on_timeout()

            return True

        return False

    def reset(self):
        super().reset()
        self._timed_out = False
        self.step.reset()


# ======================================================
# PARALLEL STEP
# ======================================================

class ParallelStep(PipelineStep):

    def __init__(self, node, steps):
        super().__init__(node)
        self.steps = steps

    def on_enter(self):
        self.node.get_logger().info(
            f"[PIPELINE] Enter ParallelStep ({len(self.steps)} steps)"
        )

        for step in self.steps:
            step.reset()

    def run(self):

        all_done = True

        for step in self.steps:
            if not step.tick():
                all_done = False

        return all_done

    def reset(self):
        super().reset()
        for step in self.steps:
            step.reset()


# ======================================================
# PIPELINE
# ======================================================

class Pipeline:

    def __init__(self, steps):
        self.steps = steps
        self.index = 0

    def reset(self):
        self.index = 0
        for step in self.steps:
            step.reset()

    def run(self):

        if self.index >= len(self.steps):
            return True

        step = self.steps[self.index]

        if step.tick():
            self.index += 1

        return self.index >= len(self.steps)
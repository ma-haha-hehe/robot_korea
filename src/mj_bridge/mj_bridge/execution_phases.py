"""Stage timing independent of motion control and success scoring."""
from contextlib import contextmanager
import time


class PhaseRecorder:
    def __init__(self, simulation_time, on_update=None):
        self.simulation_time = simulation_time
        self.on_update = on_update
        self.records = []
        self.failed_phase = None

    @contextmanager
    def stage(self, name, block=None):
        start_wall = time.monotonic()
        start_simulation = self.simulation_time()
        record = {'phase': name, 'block': block, 'status': 'running'}
        self.records.append(record)
        if self.on_update is not None:
            self.on_update(self.records)
        try:
            yield
        except Exception as error:
            record.update(status='failed', error=str(error))
            self.failed_phase = name
            raise
        else:
            record['status'] = 'completed'
        finally:
            record['wall_s'] = time.monotonic() - start_wall
            record['simulation_s'] = self.simulation_time() - start_simulation
            if self.on_update is not None:
                self.on_update(self.records)

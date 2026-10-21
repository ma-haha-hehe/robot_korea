"""Execution journal and guarded assembly-state transitions."""
from contextlib import contextmanager
import math
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
        except BaseException as error:
            record.update(status='failed', error=str(error) or type(error).__name__)
            self.failed_phase = name
            raise
        else:
            record['status'] = 'completed'
        finally:
            record['wall_s'] = time.monotonic() - start_wall
            record['simulation_s'] = self.simulation_time() - start_simulation
            if self.on_update is not None:
                self.on_update(self.records)


class StateTransitionError(RuntimeError):
    """A missing predecessor or measured guard must stop the episode."""


class ExecutionStateMachine(PhaseRecorder):
    BLOCK_PHASES = ('source_observation', 'empty_approach', 'grasp_approach',
                    'grasp_closure', 'lift', 'loaded_transfer',
                    'alignment_and_insertion', 'release', 'retreat',
                    'placement_verification')

    def __init__(self, simulation_time, on_update=None, on_failure=None,
                 require_grasp_contact=True):
        super().__init__(simulation_time, on_update)
        self.on_failure = on_failure
        self.require_grasp_contact = require_grasp_contact
        self.plan = ()
        self.block_index = 0
        self.phase_index = 0
        self.current_state = 'idle'
        self.status = 'idle'
        self.error = None
        self._active = False

    def configure_plan(self, block_ids):
        if isinstance(block_ids, (str, bytes)):
            raise StateTransitionError('plan must be a sequence of block IDs')
        ids = tuple(block_ids)
        if (self.current_state != 'planning' or not self._active or self.plan
                or not ids or any(not isinstance(i, str) or not i for i in ids)
                or len(set(ids)) != len(ids)):
            raise StateTransitionError('configure one nonempty, unique plan during planning')
        self.plan = ids

    def evidence(self, **values):
        if not self._active:
            raise StateTransitionError('evidence requires an active execution state')
        self.records[-1].setdefault('evidence', {}).update(values)

    def expected(self):
        if self.status in ('failed', 'completed'):
            return None, None
        if not self.plan:
            return 'planning', None
        if self.block_index == len(self.plan):
            return 'final_stability', None
        return self.BLOCK_PHASES[self.phase_index], self.plan[self.block_index]

    def summary(self):
        state, block = self.expected()
        return {'status': self.status, 'state': self.current_state,
                'next_state': state, 'next_block': block,
                'planned_blocks': len(self.plan), 'verified_blocks': self.block_index,
                'failed_phase': self.failed_phase, 'error': self.error}

    def _completion_guard(self, name):
        if self.status == 'failed':
            raise StateTransitionError('a failed execution state cannot resume')
        proof = self.records[-1].get('evidence', {})
        if name == 'planning' and not self.plan:
            raise StateTransitionError('planning did not provide a complete block order')
        if name == 'grasp_approach' and proof.get('pose_confirmed') is not True:
            raise StateTransitionError('grasp approach lacks measured pose confirmation')
        if name == 'grasp_closure':
            if (proof.get('closure_complete') is not True or
                    (self.require_grasp_contact and proof.get('grasp_confirmed') is not True)):
                raise StateTransitionError('grasp closure lacks confirmed holding contact')
        if name == 'lift' and self.require_grasp_contact:
            measured = proof.get('measured_lift_m')
            if (not isinstance(measured, (int, float)) or isinstance(measured, bool)
                    or not math.isfinite(measured) or measured < .06):
                raise StateTransitionError('lift lacks measured clearance of at least 60 mm')
        if name == 'release':
            release = proof.get('release_confirmation', {})
            fingers = release.get('finger_positions_m', [])
            valid = (len(fingers) == 2
                     and all(isinstance(v, (int, float)) and math.isfinite(v) and v >= .0395
                             for v in fingers)
                     and release.get('target_contact') is False
                     and release.get('target_contacts_observable') is True)
            if not valid:
                raise StateTransitionError('release lacks measured open, clear and observable fingers')
        if name in ('placement_verification', 'final_stability'):
            if proof.get('placement_verified') is not True:
                raise StateTransitionError('placement must pass measured-state verification')

    def _fail(self, name, error):
        if self.status in ('failed', 'completed'):
            return
        self.current_state = name
        self.status = 'failed'
        self.failed_phase = name
        self.error = str(error) or type(error).__name__
        if self.on_failure is not None:
            self.on_failure(error)

    @contextmanager
    def stage(self, name, block=None):
        expected = self.expected()
        if self.status in ('failed', 'completed') or self._active or (name, block) != expected:
            error = StateTransitionError(f'illegal transition to {name}/{block}; expected {expected}')
            self._fail(name, error)
            raise error
        self._active = True
        self.current_state = name
        self.status = 'running'
        try:
            with super().stage(name, block):
                yield self
                self._completion_guard(name)
        except BaseException as error:
            self._fail(name, error)
            raise
        else:
            if name == 'final_stability':
                self.status = 'completed'
            elif name != 'planning':
                self.phase_index += 1
                if self.phase_index == len(self.BLOCK_PHASES):
                    self.phase_index = 0
                    self.block_index += 1
        finally:
            self._active = False
            if self.on_update is not None:
                self.on_update(self.records)

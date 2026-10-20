import math
from types import SimpleNamespace
import numpy as np
import pytest
from mj_bridge.support_geometry import support_margin_m
from mj_bridge.reference_executor import OracleExecutor, ExecutionFailure


def block(x=0., y=0., z=0., kind='brick_2x2', yaw=0.):
    return {'type': kind, 'position': [x, y, z], 'yaw_rad': yaw}


def test_full_support_has_positive_margin():
    assert support_margin_m(block(z=.0192), [block()]) == pytest.approx(.016)


def test_offset_support_places_mass_at_edge():
    assert support_margin_m(block(z=.0192), [block(y=.016)]) == pytest.approx(0.)


def test_bridge_support_uses_convex_hull_not_individual_centres():
    supports = [block(x=-.024), block(x=.024)]
    assert support_margin_m(block(z=.0192, kind='brick_4x2'), supports) == pytest.approx(.016)


def test_rotation_and_translation_preserve_margin():
    assert support_margin_m(block(x=.3, y=.2, z=.0192, yaw=math.pi/2),
                            [block(x=.316, y=.2, yaw=math.pi/2)]) == pytest.approx(0.)


def test_different_layer_or_disjoint_part_is_not_support():
    assert support_margin_m(block(z=.0192), [block(z=.0384), block(x=.1)]) is None


def capture_fixture(monkeypatch, height=.002):
    monkeypatch.setattr('mj_bridge.support_geometry.support_margin_m', lambda *_: 0.)
    monkeypatch.setattr('mj_bridge.support_geometry.support_overlap_area_m2', lambda *_: 2 * .016**2)
    monkeypatch.setattr('mj_bridge.support_geometry.jaw_crosses_support_edge', lambda *_: False)
    e = OracleExecutor.__new__(OracleExecutor)
    e.events = []
    e.node = SimpleNamespace(episode_manifest={'target_blocks': []})
    e.model = SimpleNamespace(body=lambda _: SimpleNamespace(id=0))
    e.data = SimpleNamespace(xpos=np.array([[0., 0., height]]), xmat=np.eye(3).reshape(1, 9))
    e.hand = 0
    e.tool_offset = np.zeros(3)
    e.speed_scale = 1.5
    e.confirm_held_part = lambda target: None
    return e


def test_capture_rejects_unbounded_descent_before_motion(monkeypatch):
    e = capture_fixture(monkeypatch, .005)
    e.move_vertical = lambda *_: pytest.fail('must not command an unbounded descent')
    with pytest.raises(ExecutionFailure, match='bounded entry'):
        e.capture_support_edge({'body_name': 'part', 'position': [0., 0., 0.]}, 0.)


def test_capture_restores_speed_and_guard_after_motion_failure(monkeypatch):
    e = capture_fixture(monkeypatch)
    def fail(*_):
        assert e.speed_scale == .2
        assert e._capture_guard_target is not None
        raise ExecutionFailure('contact limit')
    e.move_vertical = fail
    with pytest.raises(ExecutionFailure, match='contact limit'):
        e.capture_support_edge({'body_name': 'part', 'position': [0., 0., 0.]}, 0.)
    assert e.speed_scale == 1.5
    assert e._capture_guard_target is None


def test_capture_refuses_release_if_measured_height_does_not_follow_command(monkeypatch):
    e = capture_fixture(monkeypatch)
    e.move_vertical = lambda *_: None
    with pytest.raises(ExecutionFailure, match='did not reach'):
        e.capture_support_edge({'body_name': 'part', 'position': [0., 0., 0.]}, 0.)


def motion_fixture():
    e = OracleExecutor.__new__(OracleExecutor)
    e.qadr = np.array([0])
    e.node = SimpleNamespace(target_qpos=np.array([1.]))
    e.model = SimpleNamespace(body=lambda _: SimpleNamespace(id=0, name='bench__part'),
                              opt=SimpleNamespace(timestep=.0005), geom_bodyid=np.array([0, 1]))
    e.data = SimpleNamespace(xpos=np.array([[0., 0., .002]]),
                             xquat=np.array([[math.cos(.005/2), math.sin(.005/2), 0., 0.]]),
                             qpos=np.array([.2]), contact=[])
    return e


def test_fast_tipping_and_falling_unloads_before_contact_limit():
    from mj_bridge.reference_executor import _CaptureLoaded
    e = motion_fixture()
    e._capture_previous = (.003, .00202)
    with pytest.raises(_CaptureLoaded):
        e.check_capture_motion({'body_name': 'part', 'position': [0., 0., 0.]})
    np.testing.assert_array_equal(e.node.target_qpos, e.data.qpos)


def test_slow_capture_does_not_unload_on_small_orientation_change():
    e = motion_fixture()
    e._capture_previous = (.003, .002003)
    e.check_capture_motion({'body_name': 'part', 'position': [0., 0., 0.]})
    np.testing.assert_array_equal(e.node.target_qpos, [1.])


def test_early_contact_depth_guard_is_independent_of_tipping():
    from mj_bridge.reference_executor import _CaptureLoaded
    e = motion_fixture()
    e.data.contact = [SimpleNamespace(geom1=0, geom2=1, dist=-.00021)]
    with pytest.raises(_CaptureLoaded):
        e.check_capture_motion({'body_name': 'part', 'position': [0., 0., 0.]})


def test_capture_does_not_recover_from_lost_alignment():
    e = motion_fixture()
    e.data.xpos[0, 0] = .002
    with pytest.raises(ExecutionFailure, match='lost alignment'):
        e.check_capture_motion({'body_name': 'part', 'position': [0., 0., 0.]})


def test_jaw_direction_distinguishes_safe_capture_from_cross_edge_loading():
    from mj_bridge.support_geometry import jaw_crosses_support_edge
    target = block(z=.0192)
    lower = [block(y=.016)]
    assert jaw_crosses_support_edge(target, lower, 0.)
    assert not jaw_crosses_support_edge(target, lower, math.pi/2)
    assert not jaw_crosses_support_edge(target, [block()], 0.)


def test_cross_edge_detection_rotates_with_the_whole_assembly():
    from mj_bridge.support_geometry import jaw_crosses_support_edge
    target = block(x=.3, y=.2, z=.0192, yaw=math.pi/4)
    lower = [block(x=.3-.016/math.sqrt(2), y=.2+.016/math.sqrt(2), yaw=math.pi/4)]
    assert jaw_crosses_support_edge(target, lower, math.pi/4)
    assert not jaw_crosses_support_edge(target, lower, 3*math.pi/4)


def test_cross_edge_grasp_releases_without_an_extra_held_descent(monkeypatch):
    e = capture_fixture(monkeypatch)
    monkeypatch.setattr('mj_bridge.support_geometry.jaw_crosses_support_edge', lambda *_: True)
    e.move_vertical = lambda *_: pytest.fail('cross-edge grasp must unload first')
    assert e.capture_support_edge({'body_name': 'part', 'position': [0., 0., 0.]}, 0.)['descent_m'] == 0.


def test_small_support_area_counts_overlap_not_full_part_size():
    from mj_bridge.support_geometry import support_overlap_area_m2
    assert support_overlap_area_m2(block(z=.0192), [block(y=.016)]) == pytest.approx(2*.016**2)
    assert support_overlap_area_m2(block(z=.0192, kind='brick_4x2'),
                                   [block(y=.016, kind='brick_4x2')]) == pytest.approx(4*.016**2)


def test_broad_support_retains_normal_unload_path(monkeypatch):
    e = capture_fixture(monkeypatch)
    monkeypatch.setattr('mj_bridge.support_geometry.support_overlap_area_m2', lambda *_: 4*.016**2)
    e.move_vertical = lambda *_: pytest.fail('broad support needs no extra held capture')
    assert e.capture_support_edge({'body_name': 'part', 'position': [0., 0., 0.]}, 0.) is None

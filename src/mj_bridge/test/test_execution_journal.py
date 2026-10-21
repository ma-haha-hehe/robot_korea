"""Interrupted journal writes must preserve the last complete result."""
import json
import pytest
from mj_bridge.benchmark_core import dump_json
from mj_bridge.execution_phases import PhaseRecorder


def test_invalid_payload_does_not_truncate_previous_result(tmp_path):
    path = tmp_path/'result.json'
    dump_json(path, {'state': 'previous', 'description': '积木'})
    previous = path.read_bytes()
    with pytest.raises(TypeError):
        dump_json(path, {'state': object()})
    assert path.read_bytes() == previous
    assert json.loads(path.read_text())['description'] == '积木'


def test_failed_replace_leaves_readable_previous_journal_and_no_temp_file(tmp_path, monkeypatch):
    import mj_bridge.benchmark_core as module
    path = tmp_path/'result.json'
    dump_json(path, {'state': 'previous'})
    def fail(*_):
        raise OSError('write interrupted')
    monkeypatch.setattr(module.os, 'replace', fail)
    with pytest.raises(OSError, match='interrupted'):
        dump_json(path, {'state': 'new'})
    assert json.loads(path.read_text()) == {'state': 'previous'}
    assert list(tmp_path.iterdir()) == [path]


def test_keyboard_interrupt_marks_journal_failed():
    recorder = PhaseRecorder(lambda: 0.)
    with pytest.raises(KeyboardInterrupt):
        with recorder.stage('lift', 'brick'):
            raise KeyboardInterrupt()
    assert recorder.failed_phase == 'lift'
    assert recorder.records[0]['status'] == 'failed'

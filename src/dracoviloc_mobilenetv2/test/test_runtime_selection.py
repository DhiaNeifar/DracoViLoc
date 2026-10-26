import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'mobilenetv2'))
from runtime_selection import select_runtime  # noqa: E402

METADATA = {
    'runtime': {'gpu': 'Orin', 'engine': 'drone_fp32.engine'},
    'local_runtime': {'gpu': 'Quadro T2000 with Max-Q Design',
                      'engine': 'drone_fp32_t2000.engine', 'python': '/usr/bin/python3'},
}


@pytest.fixture
def model_dir(tmp_path):
    directory = tmp_path / 'models' / 'mobilenetv2'
    directory.mkdir(parents=True)
    (directory / 'metadata.json').write_text(json.dumps(METADATA))
    return directory


@pytest.mark.parametrize('gpu,engine', [
    ('Orin (nvgpu)', 'drone_fp32.engine'),
    ('Quadro T2000 with Max-Q Design', 'drone_fp32_t2000.engine'),
])
def test_engine_follows_gpu(model_dir, gpu, engine):
    selected = select_runtime(model_dir, gpu=gpu, probe=lambda python: True)
    assert selected['engine'] == str(model_dir / engine)


def test_first_working_interpreter_is_chosen(model_dir):
    selected = select_runtime(model_dir, gpu='Orin (nvgpu)',
                              probe=lambda python: python == '/usr/bin/python3')
    assert selected['python'] == '/usr/bin/python3'


@pytest.mark.parametrize('gpu', ['', 'NVIDIA GeForce RTX 5050'])
def test_unsupported_machine_is_explained(model_dir, gpu):
    with pytest.raises(RuntimeError):
        select_runtime(model_dir, gpu=gpu, probe=lambda python: True)

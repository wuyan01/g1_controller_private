"""Generate camera-enabled sources in the build tree; never edit the vendor tree."""
import json
from pathlib import Path
import sys


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise RuntimeError(f'Unsupported unitree_mujoco source: expected one {old!r}')
    return source.replace(old, new, 1)


def main():
    sim, output = map(Path, sys.argv[1:])
    source = (sim / 'src/main.cc').read_text()
    source = replace_once(
        source,
        'std::filesystem::path(getExecutableDir()).parent_path()',
        f'std::filesystem::path({json.dumps(str(sim.resolve()))})',
    )
    (output / 'main.cc').write_text(source)
    source = (sim / 'mujoco/simulate/simulate.cc').read_text()
    source = '#include "camera_publisher.h"\n' + source
    source = replace_once(source, 'this->LoadOnRenderThread();',
                          'g1_camera_reset();\n        this->LoadOnRenderThread();')
    source = replace_once(source, '    }  // MutexLock (unblocks simulation thread)',
                          '      g1_camera_update(m_, d_);\n'
                          '    }  // MutexLock (unblocks simulation thread)')
    source = replace_once(source, '    // render while simulation is running',
                          '    g1_camera_render(&this->platform_ui->mjr_context());\n'
                          '    // render while simulation is running')
    (output / 'simulate.cc').write_text(source)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Create a portable G1 chair scene without modifying the Unitree checkout."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mujoco-dir', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--chair-x', type=float, default=-1.4)
    parser.add_argument('--chair-y', type=float, default=0.0)
    args = parser.parse_args()
    model_dir = args.mujoco_dir.resolve() / 'unitree_robots/g1'
    output = args.output_dir.resolve()
    # Read first so invalid inputs do not leave a partially generated scene.
    robot = ET.parse(model_dir / 'g1_29dof.xml')
    scene = ET.parse(model_dir / 'scene_29dof.xml')
    compiler = robot.getroot().find('compiler')
    compiler.set('meshdir', str(model_dir / compiler.get('meshdir', 'meshes')))
    from ament_index_python.packages import get_package_share_directory
    prop = ET.parse(Path(get_package_share_directory('g1_controller')) / 'config/mujoco/chair.xml')
    chair = prop.getroot().find('body')
    chair.set('pos', f'{args.chair_x} {args.chair_y} 0')
    scene.getroot().find('worldbody').append(chair)
    scene.getroot().set('model', 'g1_29dof chair scene')
    scene.getroot().find('include').set('file', 'g1_29dof_chair_robot.xml')
    output.mkdir(parents=True, exist_ok=True)
    for tree, name in [(robot, 'g1_29dof_chair_robot.xml'), (scene, 'scene_29dof_chair.xml')]:
        ET.indent(tree, space='  ')
        tree.write(output / name, encoding='unicode')
    print(output / 'scene_29dof_chair.xml')


if __name__ == '__main__':
    main()

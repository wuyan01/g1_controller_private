#!/usr/bin/env python3
"""Combine the photo desk environment with Unitree's controller-compatible G1."""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def resolve_assets(root, directory):
    compiler = root.find('compiler')
    for asset in root.findall('./asset/*'):
        if 'file' in asset.attrib:
            subdir = compiler.get('meshdir' if asset.tag == 'mesh' else 'texturedir', '')
            path = (directory / subdir / asset.get('file')).resolve()
            if not path.is_file():
                raise FileNotFoundError(path)
            asset.set('file', str(path))


def make_scene(mujoco_dir, source, desk_x):
    robot_path = mujoco_dir / 'unitree_robots/g1/g1_29dof.xml'
    robot = ET.parse(robot_path).getroot()
    desk = ET.parse(source).getroot()
    if desk.find('actuator') is not None or desk.find('include') is not None:
        raise ValueError('Use the environment-only desk_scene.xml')
    resolve_assets(robot, robot_path.parent)
    resolve_assets(desk, source.parent)
    # Explicit absolute asset paths permit a single compiler for both models.
    compiler = robot.find('compiler')
    compiler.attrib.update(desk.find('compiler').attrib)
    compiler.attrib.pop('meshdir', None)
    compiler.attrib.pop('texturedir', None)
    for tag in ('option', 'size'):
        previous = robot.find(tag)
        if previous is not None:
            robot.remove(previous)
        if desk.find(tag) is not None:
            robot.append(desk.find(tag))
    robot.find('asset').extend(desk.find('asset'))
    world = robot.find('worldbody')
    # Rotate the desk -90 degrees about Z: its open front faces the G1's +X.
    # Keep free props at world level and after the robot to preserve joint indices.
    for element in desk.find('worldbody'):
        x, y, z = map(float, element.get('pos', '0 0 0').split())
        element.set('pos', f'{desk_x + y} {-x} {z}')
        if element.tag in ('body', 'camera'):
            w, x, y, z = map(float, element.get('quat', '1 0 0 0').split())
            s = math.sqrt(0.5)
            element.set('quat', f'{s*(w+z)} {s*(x+y)} {s*(y-x)} {s*(z-w)}')
        elif element.tag == 'light':
            x, y, z = map(float, element.get('dir', '0 0 -1').split())
            element.set('dir', f'{y} {-x} {z}')
        world.append(element)
    robot.set('model', 'g1_29dof photo desk')
    ET.SubElement(robot, 'statistic', center=f'{desk_x / 2} 0 0.7', extent='2.5')
    return ET.ElementTree(robot)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mujoco-dir', required=True, type=Path)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--desk-x', type=float, default=1.2,
                        help='Desk center in front of the robot, in meters (default: 1.2)')
    args = parser.parse_args()
    tree = make_scene(args.mujoco_dir.resolve(), args.source.resolve(), args.desk_x)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir.resolve() / 'scene_29dof_desk.xml'
    ET.indent(tree, space='  ')
    tree.write(output, encoding='unicode')
    print(output)


if __name__ == '__main__':
    main()

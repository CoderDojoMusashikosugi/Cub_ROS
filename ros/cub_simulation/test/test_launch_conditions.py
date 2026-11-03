"""Check actual launch decisions without starting hardware or loading a profile file."""
import importlib.util
from pathlib import Path
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[2]


def load_launch(relative):
    spec = importlib.util.spec_from_file_location('launch_under_test', ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context_for(name='none'):
    context = LaunchContext()
    context.launch_configurations['simulation'] = name
    context.launch_configurations['port'] = '9090'
    context.launch_configurations['address'] = '127.0.0.1'
    return context


@pytest.mark.parametrize('name,expected', [('none', True), ('godot', False)])
def test_cub4_hardware_nodes_are_decided_in_launch(name, expected):
    module = load_launch('cub4_bringup/launch/common.launch.py')
    nodes = [a for a in module.generate_launch_description().entities if isinstance(a, Node)]
    conditional = [a for a in nodes if a.condition is not None]
    assert len(conditional) == 3  # wheel odometry, GNSS monitor, serial start button
    assert all(a.condition.evaluate(context_for(name)) == expected for a in conditional)
    assert len(nodes) - len(conditional) == 6  # shared nodes remain enabled


@pytest.mark.parametrize('name,clear', [('none', True), ('godot', False)])
def test_common_routes_target_respects_cub_target(monkeypatch, name, clear):
    monkeypatch.setenv('CUB_TARGET', 'mcub_direct')
    module = load_launch('cub_bringup/launch/common.launch.py')
    context = context_for(name)
    actions = module.launch_setup(context)
    assert actions[0].condition.evaluate(context) == clear
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert [a.condition.evaluate(context) for a in includes] == [i == 1 for i in range(4)]
    assert not any(key.startswith('simulation_') for key in context.launch_configurations)


@pytest.mark.parametrize('name,ekf,odom_tf', [('none', True, False), ('godot', False, True)])
def test_localization_switches_ekf_and_odom_to_tf_for_godot(monkeypatch, name, ekf, odom_tf):
    monkeypatch.setenv('CUB_TARGET', 'cub4')
    module = load_launch('cub_bringup/launch/localization.launch.py')
    context = context_for(name)
    actions = module.launch_setup(context)
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    nodes = [a for a in actions if isinstance(a, Node)]
    # includes[0] is common.launch.py (unconditional), includes[1] is ekf_locali.launch.py
    assert includes[1].condition.evaluate(context) == ekf
    # nodes[0] is odom_to_tf, nodes[1] is static_transform_publisher (map -> odom)
    assert nodes[0].condition.evaluate(context) == odom_tf
    assert nodes[1].condition.evaluate(context) is True


def test_simulation_bringup_only_starts_godot_nodes_for_godot():
    module = load_launch('cub_simulation/launch/bringup.launch.py')
    assert not any(isinstance(a, Node) for a in module.start(context_for('none')))
    nodes = module.start(context_for('godot'))
    assert len(nodes) == 2
    assert all(a.condition.evaluate(context_for('godot')) for a in nodes)
    assert all(not a.condition.evaluate(context_for('none')) for a in nodes)


@pytest.mark.parametrize('file', [
    'cub_bringup/launch/common.launch.py',
    'cub4_bringup/launch/common.launch.py',
    'cub_bringup/launch/localization.launch.py',
    'cub_bringup/launch/navigation.launch.py',
    'cub_bringup/launch/simulation.launch.py',
    'cub_simulation/launch/bringup.launch.py',
])
def test_default_and_unknown_simulation_arguments(file):
    module = load_launch(file)
    argument = next(a for a in module.generate_launch_description().entities
                    if isinstance(a, DeclareLaunchArgument) and a.name == 'simulation')
    context = LaunchContext()
    argument.execute(context)
    assert context.launch_configurations['simulation'] == 'none'
    with pytest.raises(RuntimeError, match='not valid'):
        argument.execute(context_for('typo'))


def test_godot_diagnostics_and_urdf_lidar_match():
    import json
    import xml.etree.ElementTree as ET
    import yaml

    diagnostics = yaml.safe_load((ROOT / 'cub_simulation/config/godot_diagnostics.yaml').read_text())
    topics = diagnostics['topic_hz_checker']['ros__parameters']['topics']
    assert topics['odom']['topic'] == '/odom'
    assert topics['odom']['type'] == 'nav_msgs/msg/Odometry'
    assert topics['odom']['expected_hz'] == 20.0
    assert topics['scan']['topic'] == '/scan'
    assert topics['scan']['type'] == 'sensor_msgs/msg/LaserScan'
    assert topics['scan']['expected_hz'] == 10.0

    urdf = ET.parse(ROOT / 'cub_description/urdf/cub4.urdf')
    velodyne_joint = next(j for j in urdf.findall('./joint') if j.find('child').get('link') == 'velodyne')
    assert velodyne_joint.find('parent').get('link') == 'base_link'
    xyz = [float(v) for v in velodyne_joint.find('origin').get('xyz').split()]
    assert xyz == pytest.approx([0.0, 0.0, 0.2492])

    cub4_meta_path = ROOT.parent / 'simulator/assets/cub4/cub4.json'
    if cub4_meta_path.exists():
        cub4_meta = json.loads(cub4_meta_path.read_text())
        assert cub4_meta['urdf_lidar']['frame_id'] == 'velodyne'
        assert cub4_meta['urdf_lidar']['xyz'] == pytest.approx(xyz)

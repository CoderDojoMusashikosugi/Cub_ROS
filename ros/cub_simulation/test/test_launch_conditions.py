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


@pytest.mark.parametrize('name,route,clear', [('none', 1, True), ('godot', 0, False)])
def test_common_routes_hardware_target_and_godot_cub4(monkeypatch, name, route, clear):
    monkeypatch.setenv('CUB_TARGET', 'mcub_direct')
    module = load_launch('cub_bringup/launch/common.launch.py')
    context = context_for(name)
    actions = module.launch_setup(context)
    assert actions[0].condition.evaluate(context) == clear
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert [a.condition.evaluate(context) for a in includes] == [i == route for i in range(4)]
    assert not any(key.startswith('simulation_') for key in context.launch_configurations)


@pytest.mark.parametrize('file', ['navigation.launch.py', 'localization.launch.py'])
def test_sensor_dependent_launch_rejects_velocity_only_godot(file):
    module = load_launch('cub_bringup/launch/' + file)
    assert module.require_navigation(context_for('none')) == []
    with pytest.raises(ValueError, match='velocity control only'):
        module.require_navigation(context_for('godot'))


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

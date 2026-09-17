#!/usr/bin/env python3
"""
Behaviour test for the launch files' simulation-clock argument.

Each launch file is imported and its launch description inspected directly: the
declared argument names, their defaults, and the value every node receives for
its internal `use_sim_time` parameter. No node is started.

Contract under test (P2.2 fix plan section 1, plus the shared environment
default):

* `use_sim` is the single external argument.
* `use_sim_time` is not declared as a launch argument; it stays the ROS node's
  internal parameter name.
* Every node that sets `use_sim_time` resolves it from `use_sim`.
* The default is read from `USE_SIM_TIME` in the environment inherited by the
  launching process. When it is unset or empty, each launch keeps its own
  pre-existing default: mapping false, localization true, and
  localization_with_nonground false.
* An explicit `use_sim:=...` launch argument overrides the environment default.
* The value is sampled when the launch starts, so a new export is only picked
  up by a new launch, never by a running node.
"""
import contextlib
import importlib.util
import os
import unittest
from pathlib import Path
from unittest import mock

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import (
    normalize_to_list_of_substitutions,
    perform_substitutions,
)
from launch_ros.actions import Node

_REPO_ROOT = Path(__file__).resolve().parent.parent

# Environment variable carrying the shared default for the `use_sim` argument
# into every launch process.
_ENV_VAR = "USE_SIM_TIME"

# relative launch path -> use_sim default when _ENV_VAR is unset
_LAUNCHES = {
    "mapping": ("launch/mapping.launch.py", "false"),
    "localization": ("launch/localization.launch.py", "true"),
    "localization_with_nonground": (
        "launch/localization_with_nonground.launch.py",
        "false",
    ),
}


@contextlib.contextmanager
def _environment(use_sim_time):
    """
    Run the block with _ENV_VAR set to `use_sim_time` (None removes it).

    The environment is replaced wholesale so a value exported by whoever runs
    the tests cannot leak in and decide the result.
    """
    env = dict(os.environ)
    if use_sim_time is None:
        env.pop(_ENV_VAR, None)
    else:
        env[_ENV_VAR] = use_sim_time
    with mock.patch.dict(os.environ, env, clear=True):
        yield


def _load_launch_module(name, relative_path):
    path = _REPO_ROOT / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _launch_description(name, relative_path):
    module = _load_launch_module(name, relative_path)
    description = module.generate_launch_description()
    if not hasattr(description, "entities"):
        raise AssertionError(
            f"{relative_path} did not return a LaunchDescription")
    return description


def _launch_description_with_env(name, relative_path, use_sim_time):
    """Generate the launch description under a controlled environment."""
    with _environment(use_sim_time):
        return _launch_description(name, relative_path)


def _declared_names(description):
    return [
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]


def _declared_default(description, name):
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name == name:
            return perform_substitutions(LaunchContext(), entity.default_value)
    return None


def _nodes(description):
    return [entity for entity in description.entities
            if isinstance(entity, Node)]


def _node_label(node):
    """Return the executable name for failure messages."""
    return getattr(node, "_Node__node_executable", "node")


def _resolve(context, value):
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(value))


def _default_context(description):
    """
    Build a context carrying every declared argument at its declared default.

    Executing the declarations is what makes this a *default launch* check:
    seeding the context by hand would only test the test.
    """
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(context)
    return context


def _use_sim_time_by_node(description, context):
    """
    Resolve every node's internal use_sim_time.

    Returns (label, value) pairs for the nodes that set it, plus the labels of
    any node that does not. Collecting only the nodes that do set it would let
    one silently drop the parameter and still pass.

    launch_ros stores a node's parameters name-mangled, with substitution
    tuples as keys, so both sides have to be resolved to be compared.
    """
    present = []
    missing = []
    for node in _nodes(description):
        label = _node_label(node)
        value = None
        for group in getattr(node, "_Node__parameters", ()) or ():
            if not isinstance(group, dict):
                continue
            for name, parameter in group.items():
                if _resolve(context, name) == "use_sim_time":
                    value = _resolve(context, parameter)
        if value is None:
            missing.append(label)
        else:
            present.append((label, value))
    return present, missing


class LaunchSimClockTest(unittest.TestCase):
    def test_declares_only_use_sim(self):
        for name, (relative_path, _default) in _LAUNCHES.items():
            with self.subTest(launch=name):
                declared = _declared_names(
                    _launch_description_with_env(name, relative_path, None))
                self.assertEqual(declared.count("use_sim"), 1, declared)
                self.assertNotIn("use_sim_time", declared, declared)

    def test_declared_default(self):
        # No environment variable and no override: each launch's own default.
        for name, (relative_path, default) in _LAUNCHES.items():
            with self.subTest(launch=name):
                description = _launch_description_with_env(
                    name, relative_path, None)
                self.assertEqual(
                    _declared_default(description, "use_sim"), default)

    def test_every_node_sets_use_sim_time(self):
        for name, (relative_path, _default) in _LAUNCHES.items():
            with self.subTest(launch=name):
                description = _launch_description_with_env(
                    name, relative_path, None)
                present, missing = _use_sim_time_by_node(
                    description, _default_context(description))
                self.assertEqual(
                    missing, [],
                    f"{relative_path}: these nodes set no use_sim_time")
                self.assertEqual(len(present), len(_nodes(description)))
                self.assertTrue(present)

    def test_default_launch_resolves_the_declared_default(self):
        # No override at all: every node must get the declared default.
        for name, (relative_path, default) in _LAUNCHES.items():
            with self.subTest(launch=name):
                description = _launch_description_with_env(
                    name, relative_path, None)
                present, _missing = _use_sim_time_by_node(
                    description, _default_context(description))
                self.assertTrue(present)
                for label, value in present:
                    self.assertEqual(
                        value, default, f"{relative_path}: {label}")

    def test_override_reaches_every_node(self):
        for name, (relative_path, _default) in _LAUNCHES.items():
            for override in ("true", "false"):
                with self.subTest(launch=name, use_sim=override):
                    description = _launch_description_with_env(
                        name, relative_path, None)
                    context = _default_context(description)
                    context.launch_configurations["use_sim"] = override
                    present, _missing = _use_sim_time_by_node(
                        description, context)
                    self.assertTrue(present)
                    for label, value in present:
                        self.assertEqual(
                            value, override, f"{relative_path}: {label}")

    def test_environment_sets_the_default(self):
        # `USE_SIM_TIME=...` in the environment decides the default of every
        # launch, both in the declared argument and in every node.
        for name, (relative_path, _default) in _LAUNCHES.items():
            for env_value in ("true", "false"):
                with self.subTest(launch=name, env=env_value):
                    description = _launch_description_with_env(
                        name, relative_path, env_value)
                    self.assertEqual(
                        _declared_default(description, "use_sim"), env_value)
                    present, missing = _use_sim_time_by_node(
                        description, _default_context(description))
                    self.assertEqual(missing, [], relative_path)
                    self.assertTrue(present)
                    for label, value in present:
                        self.assertEqual(
                            value, env_value, f"{relative_path}: {label}")

    def test_empty_environment_falls_back_to_launch_default(self):
        # `USE_SIM_TIME=` (empty) is treated as unset, not as "".
        for name, (relative_path, default) in _LAUNCHES.items():
            with self.subTest(launch=name):
                description = _launch_description_with_env(
                    name, relative_path, "")
                self.assertEqual(
                    _declared_default(description, "use_sim"), default)
                present, _missing = _use_sim_time_by_node(
                    description, _default_context(description))
                self.assertTrue(present)
                for label, value in present:
                    self.assertEqual(
                        value, default, f"{relative_path}: {label}")

    def test_explicit_argument_overrides_environment(self):
        # An explicit use_sim:=... on the command line wins over the export.
        for name, (relative_path, _default) in _LAUNCHES.items():
            for env_value, override in (("true", "false"), ("false", "true")):
                with self.subTest(
                        launch=name, env=env_value, use_sim=override):
                    description = _launch_description_with_env(
                        name, relative_path, env_value)
                    context = _default_context(description)
                    context.launch_configurations["use_sim"] = override
                    present, _missing = _use_sim_time_by_node(
                        description, context)
                    self.assertTrue(present)
                    for label, value in present:
                        self.assertEqual(
                            value, override, f"{relative_path}: {label}")

    def test_restart_picks_up_a_new_export(self):
        # Importing the module must not freeze the value. The description is
        # rebuilt on every launch start, so a later export is honoured by the
        # next launch while an already running launch keeps its old value.
        for name, (relative_path, _default) in _LAUNCHES.items():
            with self.subTest(launch=name):
                with _environment(None):
                    module = _load_launch_module(name, relative_path)
                for env_value in ("true", "false"):
                    with _environment(env_value):
                        description = module.generate_launch_description()
                    self.assertEqual(
                        _declared_default(description, "use_sim"), env_value)
                    present, _missing = _use_sim_time_by_node(
                        description, _default_context(description))
                    self.assertTrue(present)
                    for label, value in present:
                        self.assertEqual(
                            value, env_value, f"{relative_path}: {label}")

    def test_keeps_the_internal_parameter_name(self):
        # The external argument was renamed; the ROS parameter was not.
        for name, (relative_path, _default) in _LAUNCHES.items():
            with self.subTest(launch=name):
                text = (_REPO_ROOT / relative_path).read_text(encoding="utf-8")
                self.assertIn('"use_sim_time": use_sim', text)
                self.assertNotIn("LEGACY_USE_SIM_UNSET", text)
                self.assertNotIn("_effective_use_sim_time", text)


if __name__ == "__main__":
    unittest.main()

"""
config_loader.py — reads params.yaml directly at runtime.

Every node calls load_params() once in __init__ and uses the returned
dict instead of declare_parameters() defaults. This means editing
params.yaml takes effect on the next node launch with no code changes.

Uses importlib.resources so the yaml is found correctly both in a
colcon source build and when the package is installed into site-packages.
"""
import yaml
import importlib.resources as pkg_resources

import slalom_simulator


def load_params() -> dict:
    """
    Load and return the flat ros__parameters dict from params.yaml.

    The yaml top-level key is '/**' with a 'ros__parameters' sub-key.
    Nested keys (e.g. process_noise.sigma_ax_squared) are kept nested
    in the returned dict — access them as cfg['process_noise']['sigma_ax_squared'].
    """
    with pkg_resources.open_text(slalom_simulator, 'params.yaml') as f:
        raw = yaml.safe_load(f)
    # params.yaml uses  /**:  ros__parameters:  ...
    return raw['/**']['ros__parameters']

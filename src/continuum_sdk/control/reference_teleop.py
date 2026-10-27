"""Reference Omega mapping and IK for the independent simulation branch.

Imports the user's reference implementation without starting its application,
viewer, physics or device threads. The display model is deliberately unrelated
to the reference application's default XML.
"""
import dataclasses
import importlib
import importlib.util
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation


def load_reference(root):
    package = Path(root).resolve() / 'teleop_core'
    name = '_continuum_reference_teleop'
    if name in sys.modules:
        if Path(sys.modules[name].__file__).resolve() != package / '__init__.py':
            raise ValueError('A different reference teleop package is already loaded.')
    else:
        spec = importlib.util.spec_from_file_location(name, package / '__init__.py',
                                                     submodule_search_locations=[str(package)])
        if spec is None or spec.loader is None:
            raise ValueError(f'Missing reference teleop package: {package}')
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        try:
            spec.loader.exec_module(module)
        except Exception:
            del sys.modules[name]
            raise
    return SimpleNamespace(**{part: importlib.import_module(name+'.'+part)
                              for part in ('config','state','teleop','kinematics','ik')})


def real_branch_translation_map(omega_config, frame):
    """Composite raw-device to reference-base translation used by the robot branch.

    Mirrors device -> WORLD (omega_teleop.yaml) -> internal IK frame
    (robot_interface frame), then relabels the internal frame into the
    reference project's base frame. Reads configuration only.
    """
    from continuum_sdk.control.cartesian_frame import apply_axis_transform
    gain = np.asarray(omega_config.translation.gain, dtype=float)
    columns = []
    for axis in np.eye(3):
        world = apply_axis_transform(axis, omega_config.translation.map,
                                     omega_config.translation.signs) * gain
        internal = apply_axis_transform(world, frame.translation_map, frame.translation_signs)
        # Internal IK frame [x, y, z] -> reference base frame [x, z, -y].
        columns.append(np.array([internal[0], internal[2], -internal[1]]))
    return np.column_stack(columns)


def parse_omega_message(message):
    if not isinstance(message, dict):
        raise ValueError('Omega message must be an object.')
    if message.get('protocol_version') != 1 or message.get('kind') != 'omega_pose':
        raise ValueError('omega-reference input requires omega_pose messages.')
    p = np.asarray(message['position'], dtype=float)
    r = np.asarray(message['rotation'], dtype=float)
    if p.shape != (3,) or r.shape != (3,3) or not np.all(np.isfinite(p)) or not np.all(np.isfinite(r)):
        raise ValueError('Omega pose must contain finite position[3] and rotation[3,3].')
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(r),1.,atol=1e-5):
        raise ValueError('Omega rotation must be a proper orthonormal matrix.')
    for key in ('connected','clutch'):
        if not isinstance(message[key], bool):
            raise ValueError(f'Omega {key} must be boolean.')
    if not isinstance(message.get('session'),str) or not message['session']:
        raise ValueError('Omega input requires a nonempty session identifier.')
    if type(message.get('sequence')) is not int or message['sequence'] < 0:
        raise ValueError('Omega sequence must be a nonnegative integer.')
    return p, r, message['connected'], message['clutch']



class ReferenceTeleop:
    def __init__(self, reference, initial_joint, *, swap_rotation_xz=False,
                 translation_map=None):
        self.swap_rotation_xz = swap_rotation_xz
        self._raw_rotation = None
        self._mapped_rotation = np.eye(3)
        self.reference = reference
        self.kinematics = reference.kinematics.ContinuumKinematics(reference.config.KinematicsConfig())
        self.controller_config = reference.config.ControllerConfig()
        self.ik = reference.ik.DampedLeastSquaresIK(self.kinematics, reference.config.IKConfig(),
                                                  task_mode=self.controller_config.task_mode)
        self.ik.u[:] = np.asarray(initial_joint, dtype=float)
        self.position, self.rotation = self.kinematics.fk_tip_from_u(self.ik.u)
        self.omega = reference.state.OmegaState()
        self.targets = reference.state.SharedTargetState()
        # The upstream mapper only needs target_pose_in_base for clutch anchoring.
        # Seed from the shared FK, not a different XML's settled physics pose.
        self.scene = SimpleNamespace(target_pose_in_base=lambda:
                                     (self.position.copy(), self.rotation[:,2].copy(), self.rotation.copy()))
        teleop_config = reference.config.TeleopConfig()
        if translation_map is not None:
            # The upstream mapper applies position_scale * position_map. Fold both
            # into the supplied matrix so gain and direction come from one source.
            teleop_config = dataclasses.replace(teleop_config, position_scale=1.)
        self.mapper = reference.teleop.TeleopMapper(self.scene, threading.Lock(), self.omega,
                                                    self.targets, None, teleop_config)
        if translation_map is not None:
            candidate = np.asarray(translation_map, dtype=float)
            if candidate.shape != (3,3) or not np.all(np.isfinite(candidate)):
                raise ValueError('translation_map must be a finite 3x3 matrix.')
            self.mapper.position_map = candidate
        self.needs_anchor = True
        self.clutch = False
        self.connected = False
        self.was_clutch = False
        self.holding = True
        self.error = np.zeros(6)
        self.session = None
        self.sequence = -1

    def receive(self, message):
        p,r,connected,clutch = parse_omega_message(message)
        # Operator-calibrated input basis: [x, y, z] -> [x, z, -y].
        p = p[[0, 2, 1]] * np.array([1, 1, -1])
        if message['session'] == self.session and message['sequence'] <= self.sequence:
            raise ValueError('Duplicate or out-of-order Omega sample.')
        if message['session'] != self.session:
            self.pause()
            self._raw_rotation = None
        self.session, self.sequence = message['session'], message['sequence']
        self.connected, self.clutch = connected, clutch
        if connected:
            if self.swap_rotation_xz:
                # Operator-calibrated local increments: reverse handle twist X,
                # then apply the existing X/Z basis change. Net: [rz, ry, rx].
                if self._raw_rotation is None:
                    self._mapped_rotation = np.eye(3)
                else:
                    delta = self._raw_rotation.T @ r
                    rotvec = Rotation.from_matrix(delta).as_rotvec()
                    rotvec[0] *= -1.0  # Local handle-axis twist; not WORLD Rz.
                    delta = Rotation.from_rotvec(rotvec).as_matrix()
                    permutation = np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
                    self._mapped_rotation = (self._mapped_rotation
                                             @ permutation @ delta @ permutation.T)
                    u, _, vt = np.linalg.svd(self._mapped_rotation)
                    self._mapped_rotation = u @ vt
                self._raw_rotation = r.copy()
                r = self._mapped_rotation.copy()
            self.omega.update(p,r,0.)
        else:
            self._raw_rotation = None
            self.omega.set_connected(False)
            self.pause()

    def pause(self):
        self.needs_anchor = True
        self.holding = True

    def step(self):
        if not self.connected:
            self.pause()
            return self.ik.u.copy()
        if self.needs_anchor or (self.was_clutch and not self.clutch):
            self.mapper.initialize()
            self.needs_anchor = False
            self.was_clutch = self.clutch
            self.holding = True
            return self.ik.u.copy()
        self.was_clutch = self.clutch
        if self.clutch:
            self.mapper._refresh_previous_haptic_pose()
            self.holding = True
            return self.ik.u.copy()
        self.mapper.update_once()
        target = self.targets.snapshot()
        self.position = target.position_base.copy()
        self.rotation = target.rotation_base.copy()
        self.error = self.ik.task_error(self.ik.u,self.position,z_goal=self.rotation[:,2])
        for _ in range(self.controller_config.ik_inner_steps):
            if self.ik.has_converged(self.error):
                break
            self.error,_ = self.ik.step(self.position,z_goal=self.rotation[:,2])
        self.holding = False
        return self.ik.u.copy()

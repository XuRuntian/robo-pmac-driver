"""Geometry-only mirror for the existing single-arm MJCF. No physics stepping."""
import math
import numpy as np


class KinematicMirror:
    def __init__(self, xml_path, *, initial_pose='source'):
        if initial_pose not in ('zero', 'source'):
            raise ValueError('initial_pose must be zero or source.')
        self.initial_pose = initial_pose
        import mujoco as mj
        self.mj = mj
        self.model = mj.MjModel.from_xml_path(str(xml_path))
        self.data = mj.MjData(self.model)
        self.groups = {}
        used = set()
        for name, expected in [('ax',12), ('ay',12), ('cy',6), ('cx',6)]:
            tid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_TENDON, name)
            if tid < 0 or self.model.tendon_num[tid] != expected:
                raise ValueError(f'Unsupported MJCF tendon group: {name}')
            ids = range(self.model.tendon_adr[tid], self.model.tendon_adr[tid]+expected)
            addresses = []
            for i in ids:
                if self.model.wrap_type[i] != mj.mjtWrap.mjWRAP_JOINT or self.model.wrap_prm[i] != 1:
                    raise ValueError('Mirror requires unit-coefficient fixed joint tendons.')
                jid = self.model.wrap_objid[i]
                if self.model.jnt_type[jid] != mj.mjtJoint.mjJNT_HINGE:
                    raise ValueError('Expected hinge joint.')
                adr = int(self.model.jnt_qposadr[jid])
                if adr in used:
                    raise ValueError('Overlapping visual tendon groups.')
                used.add(adr); addresses.append(adr)
            self.groups[name] = addresses
        jid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, 'linear_joint')
        if jid < 0 or self.model.jnt_type[jid] != mj.mjtJoint.mjJNT_SLIDE:
            raise ValueError('Missing linear_joint slide.')
        self.linear_address = int(self.model.jnt_qposadr[jid])
        self._display_reference = None
        self.update([0,0,0,0,0])
        # The model's initial drawing is not an incoming source pose.
        self._display_reference = None

    def update(self, joint, *, display_gain=1.0):
        values = np.asarray(joint, dtype=float)
        if values.shape != (5,) or not np.all(np.isfinite(values)):
            raise ValueError('Expected finite [d,theta_a,phi_a,theta_c,phi_c].')
        if not math.isfinite(display_gain) or display_gain <= 0:
            raise ValueError('Display gain must be finite and positive.')
        d,ta,pa,tc,pc = values
        # Preserve the existing mirror's calibrated tendon-component convention.
        raw = np.array([d, -ta*math.cos(pa), -ta*math.sin(pa),
                        -tc*math.cos(pc), -tc*math.sin(pc)])
        reference = raw if self._display_reference is None else self._display_reference
        # Amplify changes from the first valid source pose, not its static offset.
        # Bend components avoid angle-wrap discontinuities in phi.
        origin = np.zeros(5) if self.initial_pose == 'zero' else reference
        scaled = origin + display_gain * (raw-reference)
        if not np.all(np.isfinite(scaled)):
            raise ValueError('Scaled display pose must be finite.')
        if self._display_reference is None:
            self._display_reference = raw.copy()
        d = scaled[0]
        components = dict(zip(('ax','ay','cy','cx'), scaled[1:]))
        self.data.qpos[:] = self.model.qpos0
        self.data.qpos[self.linear_address] = d
        for name, addresses in self.groups.items():
            self.data.qpos[addresses] = components[name]/len(addresses)
        self.data.qvel[:] = 0
        # Only position transforms, no mj_step, collision, constraint or dynamics solve.
        self.mj.mj_kinematics(self.model, self.data)
        self.mj.mj_camlight(self.model, self.data)

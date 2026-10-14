"""Geometry-only mirror for the existing single-arm MJCF. No physics stepping."""
import math
import numpy as np


class KinematicMirror:
    def __init__(self, xml_path):
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
        self.update([0,0,0,0,0])

    def update(self, joint):
        values = np.asarray(joint, dtype=float)
        if values.shape != (5,) or not np.all(np.isfinite(values)):
            raise ValueError('Expected finite [d,theta_a,phi_a,theta_c,phi_c].')
        d,ta,pa,tc,pc = values
        # Preserve the existing mirror's calibrated tendon-component convention.
        components = dict(ax=-ta*math.cos(pa), ay=-ta*math.sin(pa),
                          cy=-tc*math.cos(pc), cx=-tc*math.sin(pc))
        self.data.qpos[:] = self.model.qpos0
        self.data.qpos[self.linear_address] = d
        for name, addresses in self.groups.items():
            self.data.qpos[addresses] = components[name]/len(addresses)
        self.data.qvel[:] = 0
        # Only position transforms, no mj_step, collision, constraint or dynamics solve.
        self.mj.mj_kinematics(self.model, self.data)
        self.mj.mj_camlight(self.model, self.data)

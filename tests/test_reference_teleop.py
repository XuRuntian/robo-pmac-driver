from pathlib import Path
import numpy as np
import pytest

from continuum_sdk.control.reference_teleop import load_reference, ReferenceTeleop

ROOT = Path(__file__).resolve().parents[2]/'surgical_continuum_robot/remote_control_ex'


@pytest.fixture
def ref():
    if not (ROOT/'teleop_core').exists():
        pytest.skip('Reference project is not installed')
    return load_reference(ROOT)


def message(n, p=(0,0,0), r=None, clutch=False, connected=True, session='test'):
    return dict(protocol_version=1,kind='omega_pose',session=session,sequence=n,
                position=list(p),rotation=(np.eye(3) if r is None else r).tolist(),
                connected=connected,clutch=clutch)


def test_reference_mapping_and_ik_match_direct_equations(ref):
    initial = np.array([.016,.08,.2,.04,-.1])
    shared = ReferenceTeleop(ref,initial)
    independent = ref.ik.DampedLeastSquaresIK(shared.kinematics,ref.config.IKConfig(),task_mode='pos_z')
    independent.u[:] = initial
    p,r = shared.kinematics.fk_tip_from_u(initial)
    shared.receive(message(0));np.testing.assert_array_equal(shared.step(),initial)
    previous_p=np.zeros(3);previous_r=np.eye(3)
    # Use both position and orientation; no external xyz/rotvec mapping allowed.
    for n in range(1,25):
        pos=np.array([n*1e-5,n*2e-5,-n*1e-5])
        angle=n*.001;c,s=np.cos(angle),np.sin(angle)
        rot=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        p += shared.mapper.position_map @ (pos-previous_p)
        r = r @ (shared.mapper.rotation_map @ (previous_r.T @ rot) @ shared.mapper.rotation_map.T)
        u,_,v=np.linalg.svd(r);r=u@v
        error=independent.task_error(independent.u,p,z_goal=r[:,2])
        for _ in range(2):
            if independent.has_converged(error):break
            error,_=independent.step(p,z_goal=r[:,2])
        shared.receive(message(n,pos,rot));actual=shared.step()
        np.testing.assert_allclose(shared.position,p,atol=1e-13)
        np.testing.assert_allclose(shared.rotation,r,atol=1e-13)
        np.testing.assert_allclose(actual,independent.u,atol=1e-11)
        previous_p,previous_r=pos,rot


def test_clutch_disconnect_and_new_session_do_not_inject_master_displacement(ref):
    shared=ReferenceTeleop(ref,np.zeros(5))
    shared.receive(message(0));shared.step();goal=shared.position.copy()
    shared.receive(message(1,[1,1,1],clutch=True));shared.step()
    shared.receive(message(2,[2,2,2],clutch=False));shared.step()
    np.testing.assert_array_equal(shared.position,goal)
    shared.receive(message(3,[3,3,3],connected=False));shared.step()
    shared.receive(message(4,[4,4,4]));shared.step()
    np.testing.assert_array_equal(shared.position,goal)
    shared.receive(message(0,[5,5,5],session='new'));shared.step()
    np.testing.assert_array_equal(shared.position,goal)
    assert shared.holding


def test_invalid_and_replayed_input_leave_reference_unchanged(ref):
    shared=ReferenceTeleop(ref,np.zeros(5));shared.receive(message(0));shared.step()
    before=shared.omega.snapshot().position.copy()
    for bad in [message(0,[1,1,1]),message(1,[float('nan'),0,0]),message(1,r=np.zeros((3,3)))]:
        with pytest.raises(ValueError):shared.receive(bad)
        np.testing.assert_array_equal(shared.omega.snapshot().position,before)


@pytest.mark.parametrize('axis,mapped_axis', [(0,2),(1,1),(2,0)])
@pytest.mark.parametrize('angle', [-.03,.03])
def test_simulation_swaps_rx_rz_reverses_result_z_and_preserves_ry(ref, axis, mapped_axis, angle):
    from continuum_sdk.kinematics.dls_ik import rotvec_to_matrix
    initial = np.array([.016,.08,.2,.04,-.1])
    swapped = ReferenceTeleop(ref, initial, swap_rotation_xz=True)
    original = ReferenceTeleop(ref, initial)
    # Nonidentity device reference: test relative rotations, not Euler slots.
    anchor = rotvec_to_matrix(np.array([.3,-.2,.4]))
    swapped.receive(message(0, r=anchor)); swapped.step()
    original.receive(message(0)); original.step()
    for n in range(1,5):
        delta = np.eye(3)[axis]*angle*n
        expected_delta = np.eye(3)[mapped_axis]*angle*n*(-1 if mapped_axis == 2 else 1)
        swapped.receive(message(n, r=anchor @ rotvec_to_matrix(delta)))
        original.receive(message(n, r=rotvec_to_matrix(expected_delta)))
        actual = swapped.step(); expected = original.step()
        np.testing.assert_allclose(swapped.rotation, original.rotation, atol=1e-12)
        np.testing.assert_allclose(actual, expected, atol=1e-9)

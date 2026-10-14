"""Read-only visualization snapshot; joint values use the driver calibration."""
from continuum_sdk.kinematics.joint_motor_model import MotorAngles


def joint_vector(logical, motor_model):
    joint = motor_model.motor_angles_to_joint(MotorAngles(*logical[:4]))
    return [float(logical[4]), joint.theta_a, joint.phi_a, joint.theta_c, joint.phi_c]


def visual_snapshot(sequence, feedback, target, motor_model, *, execute, feedback_age_s):
    return dict(kind='continuum_visual', version=1, sequence=sequence,
                source='encoder_model' if execute else 'dry_run_model',
                feedback_joint=joint_vector(feedback, motor_model),
                target_joint=joint_vector(target, motor_model),
                feedback_age_s=max(0., float(feedback_age_s)))

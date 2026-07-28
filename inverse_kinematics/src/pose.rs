use nalgebra::{Rotation3, SVector, Vector3};

use crate::{Pose, Residuals, jacobian::TASK_DIMENSION};

pub(crate) type TaskVector = SVector<f64, TASK_DIMENSION>;

pub(crate) fn is_finite(pose: &Pose) -> bool {
    pose.position_m
        .iter()
        .chain(&pose.orientation_rpy_rad)
        .all(|value| value.is_finite())
}

pub(crate) fn rotation(pose: &Pose) -> Rotation3<f64> {
    Rotation3::from_euler_angles(
        pose.orientation_rpy_rad[0],
        pose.orientation_rpy_rad[1],
        pose.orientation_rpy_rad[2],
    )
}

#[cfg(test)]
pub(crate) fn compose(parent: &Pose, child: &Pose) -> Pose {
    let parent_rotation = rotation(parent);
    let child_rotation = rotation(child);
    let position =
        Vector3::from(parent.position_m) + parent_rotation * Vector3::from(child.position_m);
    let orientation = (parent_rotation * child_rotation).euler_angles();
    Pose {
        position_m: position.into(),
        orientation_rpy_rad: [orientation.0, orientation.1, orientation.2],
    }
}

pub(crate) fn error(target: &Pose, current: &Pose) -> TaskVector {
    let position = Vector3::from(target.position_m) - Vector3::from(current.position_m);
    let orientation = rotation(current)
        .rotation_to(&rotation(target))
        .scaled_axis();
    TaskVector::from_row_slice(&[
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
    ])
}

pub(crate) fn residuals(
    pose_error: &TaskVector,
    position_weight: f64,
    orientation_weight: f64,
) -> Residuals {
    let position_m = pose_error.fixed_rows::<3>(0).norm();
    let orientation_rad = pose_error.fixed_rows::<3>(3).norm();
    let weighted = (position_weight * position_m.powi(2)
        + orientation_weight * orientation_rad.powi(2))
    .sqrt();
    Residuals {
        position_m,
        orientation_rad,
        weighted,
    }
}

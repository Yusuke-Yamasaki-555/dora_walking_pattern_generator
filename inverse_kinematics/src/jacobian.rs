use nalgebra::SMatrix;

use crate::IK_JOINT_COUNT;

pub(crate) const TASK_DIMENSION: usize = 6;
pub type ExternalJacobian = [[f64; IK_JOINT_COUNT]; TASK_DIMENSION];
pub(crate) type Jacobian = SMatrix<f64, TASK_DIMENSION, IK_JOINT_COUNT>;

pub(crate) fn to_matrix(jacobian: &ExternalJacobian) -> Option<Jacobian> {
    if jacobian.iter().flatten().any(|value| !value.is_finite()) {
        return None;
    }
    Some(Jacobian::from_fn(|row, column| jacobian[row][column]))
}

use crate::{IK_JOINT_COUNT, IkError, JointAngles, JointVelocities};

pub(crate) fn calculate(
    angles: JointAngles,
    previous_angles: Option<JointAngles>,
    control_period_s: f64,
) -> Result<JointVelocities, IkError> {
    if !control_period_s.is_finite() || control_period_s <= 0.0 {
        return Err(IkError::InvalidControlPeriod);
    }
    let Some(previous) = previous_angles else {
        return Ok(JointVelocities([0.0; IK_JOINT_COUNT]));
    };
    if previous.0.iter().any(|angle| !angle.is_finite()) {
        return Err(IkError::InvalidPreviousJointAngles);
    }

    Ok(JointVelocities(std::array::from_fn(|index| {
        (angles.0[index] - previous.0[index]) / control_period_s
    })))
}

use std::collections::HashSet;

use nalgebra::{Cholesky, SMatrix, SVector};

use crate::{
    IK_JOINT_COUNT, IkError, IkRequest, IkSettings, IkSolution, IkStatus, JointAngles, JointLimit,
    KinematicEvaluation, LegKinematics, Residuals,
    jacobian::{Jacobian, TASK_DIMENSION, to_matrix},
    pose::{TaskVector, error as pose_error, is_finite, residuals},
    settings::invalid_field,
    velocity,
};

type JointVector = SVector<f64, IK_JOINT_COUNT>;
type JointMatrix = SMatrix<f64, IK_JOINT_COUNT, IK_JOINT_COUNT>;
type TaskMatrix = SMatrix<f64, TASK_DIMENSION, TASK_DIMENSION>;

/// 杉原のLevenberg-Marquardt法でCassie片脚5自由度の近似IK解を求める。
///
/// 5自由度では一般の6次元Poseを完全には満たせないため、未収束も`IkSolution`として返す。
/// `Err`は入力不正、モデル評価失敗、数値計算破綻に限定する。
pub fn solve_ik(
    model: &impl LegKinematics,
    request: &IkRequest,
    settings: &IkSettings,
) -> Result<IkSolution, IkError> {
    validate_inputs(model, request, settings)?;

    let limits = model.joint_limits();
    let mut angles = request.initial_angles;
    for iteration in 0..settings.max_iterations {
        let evaluation = evaluate(model, request, angles)?;
        let error = pose_error(
            &request.target_foot_pose_in_world,
            &evaluation.foot_pose_in_world,
        );
        let current_residuals = residuals(
            &error,
            settings.position_weight,
            settings.orientation_weight,
        );
        if current_residuals.weighted <= settings.pose_tolerance {
            return solution(
                request,
                angles,
                evaluation,
                current_residuals,
                iteration,
                IkStatus::Converged,
            );
        }

        let jacobian =
            to_matrix(&evaluation.jacobian).ok_or(IkError::InvalidKinematicEvaluation)?;
        let delta = lm_step(&jacobian, &error, settings)?;
        let next_angles = apply_step(angles, delta, limits);
        let applied_step = JointVector::from(next_angles.0) - JointVector::from(angles.0);
        angles = next_angles;

        if applied_step.norm() <= settings.step_tolerance {
            let final_evaluation = evaluate(model, request, angles)?;
            let final_residuals = evaluation_residuals(request, &final_evaluation, settings);
            return solution(
                request,
                angles,
                final_evaluation,
                final_residuals,
                iteration + 1,
                IkStatus::StepTooSmall,
            );
        }
    }

    let final_evaluation = evaluate(model, request, angles)?;
    let final_residuals = evaluation_residuals(request, &final_evaluation, settings);
    solution(
        request,
        angles,
        final_evaluation,
        final_residuals,
        settings.max_iterations,
        IkStatus::MaxIterations,
    )
}

fn evaluation_residuals(
    request: &IkRequest,
    evaluation: &KinematicEvaluation,
    settings: &IkSettings,
) -> Residuals {
    residuals(
        &pose_error(
            &request.target_foot_pose_in_world,
            &evaluation.foot_pose_in_world,
        ),
        settings.position_weight,
        settings.orientation_weight,
    )
}

fn lm_step(
    jacobian: &Jacobian,
    error: &TaskVector,
    settings: &IkSettings,
) -> Result<JointVector, IkError> {
    let weights = TaskMatrix::from_diagonal(&TaskVector::from_row_slice(&[
        settings.position_weight,
        settings.position_weight,
        settings.position_weight,
        settings.orientation_weight,
        settings.orientation_weight,
        settings.orientation_weight,
    ]));
    let weighted_squared_error = error.dot(&(weights * error));
    // 論文の1/2を整数除算にせず、杉原法の定義どおり0.5倍する。
    let damping = 0.5 * weighted_squared_error + settings.minimum_bias;
    let normal_matrix =
        jacobian.transpose() * weights * jacobian + JointMatrix::identity() * damping;
    let gradient = jacobian.transpose() * weights * error;
    if !normal_matrix
        .iter()
        .chain(gradient.iter())
        .all(|value| value.is_finite())
    {
        return Err(IkError::NonFiniteComputation);
    }

    let delta = Cholesky::new(normal_matrix)
        .ok_or(IkError::LinearSolveFailed)?
        .solve(&gradient);
    delta
        .iter()
        .all(|value| value.is_finite())
        .then_some(delta)
        .ok_or(IkError::NonFiniteComputation)
}

fn apply_step(
    angles: JointAngles,
    delta: JointVector,
    limits: &[JointLimit; IK_JOINT_COUNT],
) -> JointAngles {
    JointAngles(std::array::from_fn(|index| {
        (angles.0[index] + delta[index]).clamp(limits[index].minimum_rad, limits[index].maximum_rad)
    }))
}

fn evaluate(
    model: &impl LegKinematics,
    request: &IkRequest,
    angles: JointAngles,
) -> Result<KinematicEvaluation, IkError> {
    let evaluation = model
        .evaluate(&request.waist_pose_in_world, &angles)
        .map_err(IkError::Kinematics)?;
    if !is_finite(&evaluation.foot_pose_in_world) || to_matrix(&evaluation.jacobian).is_none() {
        return Err(IkError::InvalidKinematicEvaluation);
    }
    Ok(evaluation)
}

fn solution(
    request: &IkRequest,
    angles: JointAngles,
    evaluation: KinematicEvaluation,
    residuals: Residuals,
    iterations: usize,
    status: IkStatus,
) -> Result<IkSolution, IkError> {
    if !residuals.position_m.is_finite()
        || !residuals.orientation_rad.is_finite()
        || !residuals.weighted.is_finite()
    {
        return Err(IkError::NonFiniteComputation);
    }
    Ok(IkSolution {
        angles,
        velocities: velocity::calculate(angles, request.previous_angles, request.control_period_s)?,
        achieved_foot_pose_in_world: evaluation.foot_pose_in_world,
        residuals,
        iterations,
        status,
    })
}

fn validate_inputs(
    model: &impl LegKinematics,
    request: &IkRequest,
    settings: &IkSettings,
) -> Result<(), IkError> {
    if let Some(field) = invalid_field(settings) {
        return Err(IkError::InvalidSettings(field));
    }
    if !is_finite(&request.waist_pose_in_world) {
        return Err(IkError::InvalidPose("waist_pose_in_world"));
    }
    if !is_finite(&request.target_foot_pose_in_world) {
        return Err(IkError::InvalidPose("target_foot_pose_in_world"));
    }
    if request
        .initial_angles
        .0
        .iter()
        .any(|value| !value.is_finite())
    {
        return Err(IkError::InvalidJointAngles);
    }
    if !request.control_period_s.is_finite() || request.control_period_s <= 0.0 {
        return Err(IkError::InvalidControlPeriod);
    }
    if request
        .previous_angles
        .is_some_and(|angles| angles.0.iter().any(|value| !value.is_finite()))
    {
        return Err(IkError::InvalidPreviousJointAngles);
    }

    let mut unique_names = HashSet::with_capacity(IK_JOINT_COUNT);
    for (index, name) in model.joint_names().iter().enumerate() {
        if name.trim().is_empty() {
            return Err(IkError::InvalidJointName { index });
        }
        if !unique_names.insert(name) {
            return Err(IkError::DuplicateJointName(name.clone()));
        }
    }
    for (index, (limit, angle)) in model
        .joint_limits()
        .iter()
        .zip(request.initial_angles.0)
        .enumerate()
    {
        if !limit.minimum_rad.is_finite()
            || !limit.maximum_rad.is_finite()
            || limit.minimum_rad >= limit.maximum_rad
        {
            return Err(IkError::InvalidJointLimit { index });
        }
        if !(limit.minimum_rad..=limit.maximum_rad).contains(&angle) {
            return Err(IkError::InitialAngleOutOfRange { index });
        }
        if request.previous_angles.is_some_and(|previous| {
            !(limit.minimum_rad..=limit.maximum_rad).contains(&previous.0[index])
        }) {
            return Err(IkError::PreviousAngleOutOfRange { index });
        }
    }
    Ok(())
}

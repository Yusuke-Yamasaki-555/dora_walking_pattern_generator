//! 目標関節状態をMuJoCoのmotor controlへ変換する純粋な制御計算。

use std::collections::HashMap;

use crate::{JointState, JointTarget, SimulationError};

/// 全能動関節に適用する暫定PDゲイン。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PdGains {
    pub position: f64,
    pub velocity: f64,
}

impl Default for PdGains {
    fn default() -> Self {
        // Cassieを立位安定化する最終ゲインではなく、入出力確認用の控えめな暫定値。
        Self {
            position: 5.0,
            velocity: 0.2,
        }
    }
}

/// アクチュエータ名とMJCFで定義された入力範囲。
#[derive(Clone, Debug, PartialEq)]
pub struct ActuatorLimit {
    pub name: String,
    pub minimum: f64,
    pub maximum: f64,
}

/// MuJoCoへ書き込む、範囲制限済みのmotor control。
#[derive(Clone, Debug, PartialEq)]
pub struct MotorCommand {
    pub name: String,
    pub control: f64,
}

/// 目標値と現在値からPD制御入力を計算する。
///
/// 外部状態を読み書きしない純粋関数であり、MuJoCoやDoraから独立してテストできる。
pub fn compute_motor_commands(
    targets: &[JointTarget],
    current: &[JointState],
    limits: &[ActuatorLimit],
    gains: PdGains,
) -> Result<Vec<MotorCommand>, SimulationError> {
    if !gains.position.is_finite()
        || gains.position < 0.0
        || !gains.velocity.is_finite()
        || gains.velocity < 0.0
    {
        return Err(SimulationError::InvalidGains);
    }

    let targets_by_name: HashMap<&str, &JointTarget> = targets
        .iter()
        .map(|target| (target.name.as_str(), target))
        .collect();
    let current_by_name: HashMap<&str, &JointState> = current
        .iter()
        .map(|state| (state.name.as_str(), state))
        .collect();

    if targets_by_name.len() != targets.len() {
        return Err(SimulationError::DuplicateJointTarget);
    }
    if let Some(target) = targets
        .iter()
        .find(|target| !limits.iter().any(|limit| limit.name == target.name))
    {
        return Err(SimulationError::UnexpectedJointTarget(target.name.clone()));
    }

    limits
        .iter()
        .map(|limit| {
            let target = targets_by_name
                .get(limit.name.as_str())
                .ok_or_else(|| SimulationError::MissingJointTarget(limit.name.clone()))?;
            let state = current_by_name
                .get(limit.name.as_str())
                .ok_or_else(|| SimulationError::MissingJointState(limit.name.clone()))?;
            if !target.position.is_finite() || !target.velocity.is_finite() {
                return Err(SimulationError::NonFiniteJointTarget(limit.name.clone()));
            }

            let control = gains.position * (target.position - state.position)
                + gains.velocity * (target.velocity - state.velocity);
            Ok(MotorCommand {
                name: limit.name.clone(),
                control: control.clamp(limit.minimum, limit.maximum),
            })
        })
        .collect()
}

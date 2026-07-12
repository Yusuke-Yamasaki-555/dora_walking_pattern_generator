//! MuJoCo上のCassieへ制御入力を与え、状態を読み出す暫定コントローラ。
//!
//! 制御則は純粋関数へ分離し、MuJoCoを更新する副作用は`CassieSimulation`に閉じ込める。

mod controller;
mod simulation;
mod state;

use std::{fmt, path::PathBuf};

pub use controller::{ActuatorLimit, MotorCommand, PdGains, compute_motor_commands};
pub use simulation::{
    CONTROL_PERIOD, CassieSimulation, EXPECTED_SIMULATION_PERIOD, STEPS_PER_CONTROL_PERIOD,
    SimulationConfig,
};
pub use state::{
    ActuatorState, JointState, JointTarget, NamedValue, NamedValues, Pose, RobotState,
};

/// モデル構成または制御入力が要件を満たさない場合のエラー。
#[derive(Clone, Debug, PartialEq)]
pub enum SimulationError {
    ModelLoad(String),
    DataCreation(String),
    UnexpectedSimulationPeriod(f64),
    UnexpectedActuatorCount(usize),
    MissingModelObject(String),
    MissingJointTarget(String),
    UnexpectedJointTarget(String),
    MissingJointState(String),
    NonFiniteJointTarget(String),
    DuplicateJointTarget,
    InvalidGains,
}

impl fmt::Display for SimulationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ModelLoad(message) => {
                write!(formatter, "Cassieモデルを読み込めません: {message}")
            }
            Self::DataCreation(message) => {
                write!(
                    formatter,
                    "MuJoCoシミュレーションを初期化できません: {message}"
                )
            }
            Self::UnexpectedSimulationPeriod(period) => write!(
                formatter,
                "MJCFのシミュレーション周期は0.5 msでなければなりません: {period} s"
            ),
            Self::UnexpectedActuatorCount(count) => {
                write!(
                    formatter,
                    "Cassieのアクチュエータ数は10個を期待します: {count}"
                )
            }
            Self::MissingModelObject(name) => {
                write!(formatter, "Cassieモデルに必要な要素がありません: {name}")
            }
            Self::MissingJointTarget(name) => write!(formatter, "関節目標がありません: {name}"),
            Self::UnexpectedJointTarget(name) => {
                write!(formatter, "能動関節ではない目標が含まれています: {name}")
            }
            Self::MissingJointState(name) => write!(formatter, "関節状態がありません: {name}"),
            Self::NonFiniteJointTarget(name) => {
                write!(formatter, "関節目標は有限値でなければなりません: {name}")
            }
            Self::DuplicateJointTarget => formatter.write_str("関節目標の名前が重複しています"),
            Self::InvalidGains => formatter.write_str("PDゲインは有限な非負値でなければなりません"),
        }
    }
}

impl std::error::Error for SimulationError {}

/// Git submodule内にあるCassieの描画用MJCFへの絶対パスを返す。
pub fn default_cassie_scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("mujoco_menagerie")
        .join("agility_cassie")
        .join("scene.xml")
}

// 将来のDoraノード境界。ポートのArrowスキーマ確定後に有効化する。
//
// #[dora::main]
// fn dora_main() -> Result<(), Box<dyn std::error::Error>> {
//     let (mut node, mut events) = dora::Node::init_from_env()?;
//     let mut simulation = CassieSimulation::load(
//         default_cassie_scene_path(),
//         SimulationConfig::default(),
//     )?;
//     while let Some(event) = events.recv() {
//         let targets: Vec<JointTarget> = decode_joint_targets(event)?;
//         let state = simulation.step_control(&targets)?;
//         node.send_output("robot_state", encode_robot_state(&state)?)?;
//     }
//     Ok(())
// }

#[cfg(test)]
mod test;

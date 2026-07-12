//! `controller_sim` の入出力を表す、MuJoCoに依存しない値型。

/// 名前と1つの数値を対応付けた値。
#[derive(Clone, Debug, PartialEq)]
pub struct NamedValue {
    pub name: String,
    pub value: f64,
}

/// 名前と次元数が可変の数値列を対応付けた値。
#[derive(Clone, Debug, PartialEq)]
pub struct NamedValues {
    pub name: String,
    pub values: Vec<f64>,
}

/// 能動関節へ与える目標角度・目標角速度。
#[derive(Clone, Debug, PartialEq)]
pub struct JointTarget {
    pub name: String,
    /// 目標関節角度 [rad]。
    pub position: f64,
    /// 目標関節角速度 [rad/s]。
    pub velocity: f64,
}

/// 制御計算で使用する能動関節の現在状態。
#[derive(Clone, Debug, PartialEq)]
pub struct JointState {
    pub name: String,
    /// 現在関節角度 [rad]。
    pub position: f64,
    /// 現在関節角速度 [rad/s]。
    pub velocity: f64,
}

/// MuJoCoの1つのアクチュエータに関する状態。
#[derive(Clone, Debug, PartialEq)]
pub struct ActuatorState {
    pub name: String,
    pub control: f64,
    pub length: f64,
    pub velocity: f64,
    pub force: f64,
}

/// 骨盤のワールド座標系における位置と姿勢。
///
/// `orientation` はMuJoCoと同じ `[w, x, y, z]` のクォータニオンである。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

/// 制御1周期の完了後に読み出すロボットの状態。
#[derive(Clone, Debug, PartialEq)]
pub struct RobotState {
    pub simulation_time: f64,
    pub joint_positions: Vec<NamedValue>,
    pub joint_velocities: Vec<NamedValue>,
    pub sensors: Vec<NamedValues>,
    pub actuators: Vec<ActuatorState>,
    pub pelvis_pose: Pose,
}

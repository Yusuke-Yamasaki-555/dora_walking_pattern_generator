/// Cassie片脚の能動関節数。
pub const IK_JOINT_COUNT: usize = 5;

/// ワールド座標系の位置[m]とZYXオイラー角[rad]で表すPose。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub position_m: [f64; 3],
    /// `[roll, pitch, yaw]` [rad]。
    pub orientation_rpy_rad: [f64; 3],
}

/// Cassie片脚の関節角[rad]。並びは運動学モデルが返す関節名と一致する。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointAngles(pub [f64; IK_JOINT_COUNT]);

/// Cassie片脚の関節角速度[rad/s]。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointVelocities(pub [f64; IK_JOINT_COUNT]);

/// 一つの能動関節の可動範囲[rad]。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointLimit {
    pub minimum_rad: f64,
    pub maximum_rad: f64,
}

/// 1制御周期分のIK入力。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IkRequest {
    pub waist_pose_in_world: Pose,
    pub target_foot_pose_in_world: Pose,
    pub initial_angles: JointAngles,
    /// 前制御周期の解[rad]。指定する場合は現在のモデルの関節可動範囲内でなければならない。
    pub previous_angles: Option<JointAngles>,
    pub control_period_s: f64,
}

/// 反復を正常に終了した理由。未収束も数値計算自体が成立した結果として表す。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IkStatus {
    Converged,
    StepTooSmall,
    MaxIterations,
}

/// 最終Poseと目標Poseの残差。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Residuals {
    pub position_m: f64,
    pub orientation_rad: f64,
    pub weighted: f64,
}

/// 5自由度IKが返す関節目標と診断情報。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IkSolution {
    pub angles: JointAngles,
    pub velocities: JointVelocities,
    pub achieved_foot_pose_in_world: Pose,
    pub residuals: Residuals,
    /// 実際に関節角へ適用した更新回数。
    pub iterations: usize,
    pub status: IkStatus,
}

#![forbid(unsafe_code)]
//! Cassieの片脚5自由度を対象とする数値逆運動学コア。
//!
//! 公開境界では距離を[m]、角度を[rad]で扱う。Poseの姿勢は
//! `[roll, pitch, yaw]` とし、右手系の `Rz(yaw) * Ry(pitch) * Rx(roll)` で解釈する。
//! モデル取得、設定ファイル読込、Dora通信はこの計算コアの外側で行う。

mod error;
mod settings;
mod types;

pub use error::{IkError, SettingsError};
pub use settings::{IkSettings, parse_settings};
pub use types::{
    IK_JOINT_COUNT, IkRequest, IkSolution, IkStatus, JointAngles, JointLimit, JointVelocities,
    Pose, Residuals,
};

#[cfg(test)]
mod tests;

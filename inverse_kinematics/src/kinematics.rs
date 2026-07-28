use crate::{
    IK_JOINT_COUNT, JointAngles, JointLimit, KinematicsError, Pose, jacobian::ExternalJacobian,
};

/// ある関節角における足先Poseと能動関節座標のヤコビアン。
///
/// ヤコビアンの行は順に並進X/Y/Zと回転X/Y/Z、列は`joint_names`と同じ順序である。
/// Cassieでは受動関節と閉リンク拘束を解いた結果をIssue #8側で評価して返す。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KinematicEvaluation {
    pub foot_pose_in_world: Pose,
    pub jacobian: ExternalJacobian,
}

/// IKがロボットモデルへ要求する純粋な運動学境界。
///
/// 実装は同じ入力に対して同じ出力を返し、ファイル、共有メモリ、MuJoCo状態を直接変更しない。
/// Issue #8の境界でモデル情報のスナップショットを構築してから、このtraitを実装する。
pub trait LegKinematics {
    fn joint_names(&self) -> &[String; IK_JOINT_COUNT];

    fn joint_limits(&self) -> &[JointLimit; IK_JOINT_COUNT];

    fn evaluate(
        &self,
        waist_pose_in_world: &Pose,
        joint_angles: &JointAngles,
    ) -> Result<KinematicEvaluation, KinematicsError>;
}

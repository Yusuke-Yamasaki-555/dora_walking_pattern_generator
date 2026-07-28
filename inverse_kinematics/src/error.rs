use std::fmt;

/// IK入力、モデル評価、数値計算を継続できない場合のエラー。
#[derive(Clone, Debug, PartialEq)]
pub enum IkError {
    InvalidSettings(&'static str),
    InvalidPose(&'static str),
    InvalidJointName { index: usize },
    InvalidJointAngles,
    InvalidPreviousJointAngles,
    InvalidControlPeriod,
    InvalidJointLimit { index: usize },
    InitialAngleOutOfRange { index: usize },
    DuplicateJointName(String),
    Kinematics(KinematicsError),
    InvalidKinematicEvaluation,
    LinearSolveFailed,
    NonFiniteComputation,
}

impl fmt::Display for IkError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidSettings(name) => {
                write!(formatter, "IK設定値が不正です: {name}")
            }
            Self::InvalidPose(name) => {
                write!(formatter, "{name}の位置と姿勢は有限値でなければなりません")
            }
            Self::InvalidJointName { index } => {
                write!(formatter, "関節{index}の名前が空です")
            }
            Self::InvalidJointAngles => {
                formatter.write_str("初期関節角はすべて有限値でなければなりません")
            }
            Self::InvalidPreviousJointAngles => {
                formatter.write_str("前周期の関節角はすべて有限値でなければなりません")
            }
            Self::InvalidControlPeriod => {
                formatter.write_str("制御周期は有限な正の値でなければなりません")
            }
            Self::InvalidJointLimit { index } => {
                write!(formatter, "関節{index}の可動範囲が不正です")
            }
            Self::InitialAngleOutOfRange { index } => {
                write!(formatter, "関節{index}の初期角度が可動範囲外です")
            }
            Self::DuplicateJointName(name) => {
                write!(formatter, "関節名が重複しています: {name}")
            }
            Self::Kinematics(error) => {
                write!(formatter, "運動学モデルの評価に失敗しました: {error}")
            }
            Self::InvalidKinematicEvaluation => {
                formatter.write_str("運動学モデルが有限なPoseとヤコビアンを返しませんでした")
            }
            Self::LinearSolveFailed => {
                formatter.write_str("LM正規方程式をCholesky分解できませんでした")
            }
            Self::NonFiniteComputation => formatter.write_str("IK反復中に非有限値が発生しました"),
        }
    }
}

impl std::error::Error for IkError {}

/// Issue #8側の運動学評価が失敗した理由。
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct KinematicsError {
    message: String,
}

impl KinematicsError {
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl fmt::Display for KinematicsError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&self.message)
    }
}

impl std::error::Error for KinematicsError {}

/// `key=value`形式のIK設定が契約を満たさない場合のエラー。
#[derive(Clone, Debug, PartialEq)]
pub enum SettingsError {
    MalformedLine { line: usize },
    UnknownKey { line: usize, key: String },
    DuplicateKey { line: usize, key: String },
    MissingKey(&'static str),
    InvalidValue { line: usize, key: String },
    OutOfRange(&'static str),
}

impl fmt::Display for SettingsError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MalformedLine { line } => {
                write!(
                    formatter,
                    "設定ファイルの{line}行目がkey=value形式ではありません"
                )
            }
            Self::UnknownKey { line, key } => {
                write!(
                    formatter,
                    "設定ファイルの{line}行目に未知のキーがあります: {key}"
                )
            }
            Self::DuplicateKey { line, key } => {
                write!(
                    formatter,
                    "設定ファイルの{line}行目でキーが重複しています: {key}"
                )
            }
            Self::MissingKey(key) => write!(formatter, "設定キーがありません: {key}"),
            Self::InvalidValue { line, key } => {
                write!(formatter, "設定ファイルの{line}行目の値が不正です: {key}")
            }
            Self::OutOfRange(key) => {
                write!(formatter, "設定値は有限な正の値でなければなりません: {key}")
            }
        }
    }
}

impl std::error::Error for SettingsError {}

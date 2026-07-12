//! 制御周期ごとの関節角度を蓄積し、解析しやすいdat形式へ保存する。

use std::{
    fmt,
    fs::{self, File},
    io::{BufWriter, Write},
    path::Path,
};

use crate::RobotState;

/// 1制御周期における全能動関節の角度スナップショット。
#[derive(Clone, Debug, PartialEq)]
pub struct JointAngleSample {
    pub simulation_time: f64,
    pub positions: Vec<f64>,
}

/// 同一の関節順序で記録された角度履歴。
#[derive(Clone, Debug, PartialEq)]
pub struct JointAngleLog {
    joint_names: Vec<String>,
    samples: Vec<JointAngleSample>,
}

impl JointAngleLog {
    /// 最初の状態から列名を確定する。以後、同じ名前・順序の状態だけを受け付ける。
    pub fn new(initial_state: &RobotState) -> Self {
        Self {
            joint_names: initial_state
                .joint_positions
                .iter()
                .map(|joint| joint.name.clone())
                .collect(),
            samples: Vec::new(),
        }
    }

    pub fn joint_names(&self) -> &[String] {
        &self.joint_names
    }

    pub fn samples(&self) -> &[JointAngleSample] {
        &self.samples
    }

    /// 制御周期の完了後に得た関節角度を1行分追加する。
    pub fn record(&mut self, state: &RobotState) -> Result<(), LoggingError> {
        let names_match = state.joint_positions.len() == self.joint_names.len()
            && state
                .joint_positions
                .iter()
                .zip(&self.joint_names)
                .all(|(joint, expected)| joint.name == *expected);
        if !names_match {
            return Err(LoggingError::JointLayoutChanged);
        }
        self.samples.push(JointAngleSample {
            simulation_time: state.simulation_time,
            positions: state
                .joint_positions
                .iter()
                .map(|joint| joint.value)
                .collect(),
        });
        Ok(())
    }

    /// 空白区切りのdatファイルへ全履歴をまとめて書き出す。
    pub fn write_dat(&self, output_path: impl AsRef<Path>) -> Result<(), LoggingError> {
        let output_path = output_path.as_ref();
        if let Some(parent) = output_path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
        {
            fs::create_dir_all(parent).map_err(|error| LoggingError::Io(error.to_string()))?;
        }
        let file =
            File::create(output_path).map_err(|error| LoggingError::Io(error.to_string()))?;
        let mut writer = BufWriter::new(file);

        write!(writer, "# time_s")?;
        for name in &self.joint_names {
            write!(writer, " {name}_rad")?;
        }
        writeln!(writer)?;
        for sample in &self.samples {
            write!(writer, "{:.9}", sample.simulation_time)?;
            for position in &sample.positions {
                write!(writer, " {:.15}", position)?;
            }
            writeln!(writer)?;
        }
        writer.flush()?;
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LoggingError {
    JointLayoutChanged,
    Io(String),
}

impl fmt::Display for LoggingError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::JointLayoutChanged => {
                formatter.write_str("記録中に能動関節の名前または順序が変化しました")
            }
            Self::Io(message) => write!(formatter, "関節角度ログを保存できません: {message}"),
        }
    }
}

impl std::error::Error for LoggingError {}

impl From<std::io::Error> for LoggingError {
    fn from(error: std::io::Error) -> Self {
        Self::Io(error.to_string())
    }
}

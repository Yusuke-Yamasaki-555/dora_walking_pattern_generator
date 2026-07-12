//! Cassieモデルの読み込み、MuJoCo時間発展、状態読み出しを行う副作用境界。

use std::{collections::HashMap, path::Path};

use mujoco_rs::prelude::{MjData, MjModel, MjtObj};

use crate::{
    ActuatorLimit, ActuatorState, JointState, JointTarget, NamedValue, NamedValues, PdGains, Pose,
    RobotState, SimulationError, compute_motor_commands,
};

/// Issue #1で確定したコントローラ周期 [s]。
pub const CONTROL_PERIOD: f64 = 0.010;
/// Cassie公式MJCFの既定シミュレーション周期 [s]。
pub const EXPECTED_SIMULATION_PERIOD: f64 = 0.000_5;
/// 1制御周期に実行するMuJoCoのステップ数。
pub const STEPS_PER_CONTROL_PERIOD: usize = 20;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SimulationConfig {
    pub gains: PdGains,
}

#[derive(Clone, Debug)]
struct ActiveJoint {
    name: String,
    qpos_address: usize,
    qvel_address: usize,
    actuator_id: usize,
}

#[derive(Clone, Debug)]
struct SensorDescriptor {
    name: String,
    address: usize,
    dimension: usize,
}

/// CassieのMuJoCoシミュレーションとモデル内の名前対応表を所有する。
pub struct CassieSimulation {
    data: MjData<Box<MjModel>>,
    active_joints: Vec<ActiveJoint>,
    actuator_limits: Vec<ActuatorLimit>,
    sensors: Vec<SensorDescriptor>,
    pelvis_body_id: usize,
    config: SimulationConfig,
}

impl CassieSimulation {
    /// Cassieの`scene.xml`を読み込み、`home`キーフレームから開始する。
    pub fn load(
        scene_path: impl AsRef<Path>,
        config: SimulationConfig,
    ) -> Result<Self, SimulationError> {
        let model = MjModel::from_xml(scene_path.as_ref())
            .map_err(|error| SimulationError::ModelLoad(error.to_string()))?;
        let simulation_period = model.opt().timestep;
        if (simulation_period - EXPECTED_SIMULATION_PERIOD).abs() > f64::EPSILON {
            return Err(SimulationError::UnexpectedSimulationPeriod(
                simulation_period,
            ));
        }
        if model.nu() != 10 {
            return Err(SimulationError::UnexpectedActuatorCount(model.nu() as usize));
        }

        let active_joints = active_joints(&model)?;
        let actuator_limits = actuator_limits(&model)?;
        let sensors = sensors(&model)?;
        let pelvis_body_id = model
            .name_to_id(MjtObj::mjOBJ_BODY, "cassie-pelvis")
            .ok_or_else(|| SimulationError::MissingModelObject("body cassie-pelvis".into()))?;
        let home_key_id = model
            .name_to_id(MjtObj::mjOBJ_KEY, "home")
            .ok_or_else(|| SimulationError::MissingModelObject("keyframe home".into()))?;

        let mut data = MjData::try_new(Box::new(model))
            .map_err(|error| SimulationError::DataCreation(error.to_string()))?;
        data.reset_keyframe(home_key_id)
            .map_err(|error| SimulationError::DataCreation(error.to_string()))?;
        data.forward();

        Ok(Self {
            data,
            active_joints,
            actuator_limits,
            sensors,
            pelvis_body_id,
            config,
        })
    }

    /// 現在姿勢をそのまま維持する目標値を返す。
    pub fn hold_current_targets(&self) -> Vec<JointTarget> {
        self.joint_states()
            .into_iter()
            .map(|state| JointTarget {
                name: state.name,
                position: state.position,
                velocity: 0.0,
            })
            .collect()
    }

    /// 目標値から制御入力を計算し、0.5 ms刻みで20ステップ進める。
    pub fn step_control(&mut self, targets: &[JointTarget]) -> Result<RobotState, SimulationError> {
        let commands = compute_motor_commands(
            targets,
            &self.joint_states(),
            &self.actuator_limits,
            self.config.gains,
        )?;
        let actuator_ids: HashMap<&str, usize> = self
            .active_joints
            .iter()
            .map(|joint| (joint.name.as_str(), joint.actuator_id))
            .collect();
        for command in commands {
            let id = actuator_ids[command.name.as_str()];
            self.data.ctrl_mut()[id] = command.control;
        }
        for _ in 0..STEPS_PER_CONTROL_PERIOD {
            self.data.step();
        }
        Ok(self.state())
    }

    /// 現在のMuJoCo状態を、外部ライブラリに依存しない値へコピーする。
    pub fn state(&self) -> RobotState {
        let joint_states = self.joint_states();
        RobotState {
            simulation_time: self.data.time(),
            joint_positions: joint_states
                .iter()
                .map(|state| NamedValue {
                    name: state.name.clone(),
                    value: state.position,
                })
                .collect(),
            joint_velocities: joint_states
                .iter()
                .map(|state| NamedValue {
                    name: state.name.clone(),
                    value: state.velocity,
                })
                .collect(),
            sensors: self
                .sensors
                .iter()
                .map(|sensor| NamedValues {
                    name: sensor.name.clone(),
                    values: self.data.sensordata()
                        [sensor.address..sensor.address + sensor.dimension]
                        .to_vec(),
                })
                .collect(),
            actuators: self
                .active_joints
                .iter()
                .map(|joint| {
                    let id = joint.actuator_id;
                    ActuatorState {
                        name: joint.name.clone(),
                        control: self.data.ctrl()[id],
                        length: self.data.actuator_length()[id],
                        velocity: self.data.actuator_velocity()[id],
                        force: self.data.actuator_force()[id],
                    }
                })
                .collect(),
            pelvis_pose: Pose {
                position: self.data.xpos()[self.pelvis_body_id],
                orientation: self.data.xquat()[self.pelvis_body_id],
            },
        }
    }

    /// 描画デモがviewerを構築するためのモデル参照。
    pub fn model(&self) -> &MjModel {
        self.data.model()
    }

    /// 最新状態をpassive viewerへ同期する。
    pub fn sync_viewer(&mut self, viewer: &mut mujoco_rs::viewer::MjViewer) {
        viewer.sync_data(&mut self.data);
    }

    fn joint_states(&self) -> Vec<JointState> {
        self.active_joints
            .iter()
            .map(|joint| JointState {
                name: joint.name.clone(),
                position: self.data.qpos()[joint.qpos_address],
                velocity: self.data.qvel()[joint.qvel_address],
            })
            .collect()
    }
}

fn active_joints(model: &MjModel) -> Result<Vec<ActiveJoint>, SimulationError> {
    (0..model.nu() as usize)
        .map(|actuator_id| {
            let name = model
                .id_to_name(MjtObj::mjOBJ_ACTUATOR, actuator_id)
                .ok_or_else(|| {
                    SimulationError::MissingModelObject(format!("actuator id {actuator_id}"))
                })?
                .to_owned();
            let joint_id = model
                .name_to_id(MjtObj::mjOBJ_JOINT, &name)
                .ok_or_else(|| SimulationError::MissingModelObject(format!("joint {name}")))?;
            Ok(ActiveJoint {
                name,
                qpos_address: model.jnt_qposadr()[joint_id] as usize,
                qvel_address: model.jnt_dofadr()[joint_id] as usize,
                actuator_id,
            })
        })
        .collect()
}

fn actuator_limits(model: &MjModel) -> Result<Vec<ActuatorLimit>, SimulationError> {
    (0..model.nu() as usize)
        .map(|id| {
            let name = model
                .id_to_name(MjtObj::mjOBJ_ACTUATOR, id)
                .ok_or_else(|| SimulationError::MissingModelObject(format!("actuator id {id}")))?
                .to_owned();
            let [minimum, maximum] = model.actuator_ctrlrange()[id];
            Ok(ActuatorLimit {
                name,
                minimum,
                maximum,
            })
        })
        .collect()
}

fn sensors(model: &MjModel) -> Result<Vec<SensorDescriptor>, SimulationError> {
    (0..model.nsensor() as usize)
        .map(|id| {
            Ok(SensorDescriptor {
                name: model
                    .id_to_name(MjtObj::mjOBJ_SENSOR, id)
                    .ok_or_else(|| SimulationError::MissingModelObject(format!("sensor id {id}")))?
                    .to_owned(),
                address: model.sensor_adr()[id] as usize,
                dimension: model.sensor_dim()[id] as usize,
            })
        })
        .collect()
}

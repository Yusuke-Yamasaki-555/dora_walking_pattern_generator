use std::{path::PathBuf, thread, time::Duration};

use controller_sim::{
    CONTROL_PERIOD, CassieSimulation, JointAngleLog, SimulationConfig, default_cassie_scene_path,
    sinusoidal_joint_target,
};
use mujoco_rs::viewer::MjViewer;

const DEMO_DURATION: f64 = 2.0;
const KNEE_TARGET_FREQUENCY_HZ: f64 = 0.5;
const KNEE_RANGE_UTILIZATION: f64 = 0.95;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut simulation =
        CassieSimulation::load(default_cassie_scene_path(), SimulationConfig::default())?;
    let mut targets = simulation.hold_current_targets();
    let knee_limits = ["left-knee", "right-knee"].map(|name| {
        simulation
            .joint_position_limit(name)
            .expect("Cassieモデルには左右のknee関節が必要です")
            .clone()
    });
    let mut joint_log = JointAngleLog::new(&simulation.state());
    let mut viewer = MjViewer::builder()
        .window_name("Cassie controller_sim")
        .warn_non_realtime(true)
        .build_passive(simulation.model())?;

    for _ in 0..(DEMO_DURATION / CONTROL_PERIOD).round() as usize {
        if !viewer.running() {
            break;
        }
        // 左右の膝を同相で大きく動かし、目標入力に対する姿勢変化を見やすくする。
        let time = simulation.state().simulation_time;
        for limit in &knee_limits {
            let sine_target = sinusoidal_joint_target(
                limit,
                time,
                KNEE_TARGET_FREQUENCY_HZ,
                KNEE_RANGE_UTILIZATION,
            )?;
            let target_name = sine_target.name.clone();
            if let Some(target) = targets.iter_mut().find(|target| target.name == target_name) {
                *target = sine_target;
            }
        }
        let state = simulation.step_control(&targets)?;
        joint_log.record(&state)?;
        simulation.sync_viewer(&mut viewer);
        viewer.render()?;
        thread::sleep(Duration::from_secs_f64(CONTROL_PERIOD));
    }

    let output = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("logs")
        .join("joint_angles.dat");
    joint_log.write_dat(&output)?;
    println!("関節角度ログを出力しました: {}", output.display());
    Ok(())
}

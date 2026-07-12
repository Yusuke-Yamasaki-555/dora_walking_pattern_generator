use std::{thread, time::Duration};

use controller_sim::{
    CONTROL_PERIOD, CassieSimulation, SimulationConfig, default_cassie_scene_path,
};
use mujoco_rs::viewer::MjViewer;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut simulation =
        CassieSimulation::load(default_cassie_scene_path(), SimulationConfig::default())?;
    let mut targets = simulation.hold_current_targets();
    let mut viewer = MjViewer::builder()
        .window_name("Cassie controller_sim")
        .warn_non_realtime(true)
        .build_passive(simulation.model())?;

    while viewer.running() {
        // 入出力確認用に左hip-rollへ小さな正弦波目標を与える。
        let time = simulation.state().simulation_time;
        if let Some(target) = targets
            .iter_mut()
            .find(|target| target.name == "left-hip-roll")
        {
            target.position = 0.03 * (2.0 * std::f64::consts::PI * 0.5 * time).sin();
            target.velocity = 0.03
                * 2.0
                * std::f64::consts::PI
                * 0.5
                * (2.0 * std::f64::consts::PI * 0.5 * time).cos();
        }
        simulation.step_control(&targets)?;
        simulation.sync_viewer(&mut viewer);
        viewer.render()?;
        thread::sleep(Duration::from_secs_f64(CONTROL_PERIOD));
    }
    Ok(())
}

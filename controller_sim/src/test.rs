use super::*;
use std::fs;

const ACTIVE_JOINT_NAMES: [&str; 10] = [
    "left-hip-roll",
    "left-hip-yaw",
    "left-hip-pitch",
    "left-knee",
    "left-foot",
    "right-hip-roll",
    "right-hip-yaw",
    "right-hip-pitch",
    "right-knee",
    "right-foot",
];

fn load_simulation() -> CassieSimulation {
    CassieSimulation::load(default_cassie_scene_path(), SimulationConfig::default())
        .expect("sparse-checkoutしたCassieのscene.xmlを読み込める必要があります")
}

fn all_finite(values: impl IntoIterator<Item = f64>) -> bool {
    values.into_iter().all(f64::is_finite)
}

#[test]
fn pd_controller_is_pure_and_clamps_motor_controls() {
    let targets = vec![JointTarget {
        name: "joint".into(),
        position: 10.0,
        velocity: 0.0,
    }];
    let current = vec![JointState {
        name: "joint".into(),
        position: 0.0,
        velocity: 0.0,
    }];
    let limits = vec![ActuatorLimit {
        name: "joint".into(),
        minimum: -1.0,
        maximum: 1.0,
    }];

    let commands = compute_motor_commands(&targets, &current, &limits, PdGains::default()).unwrap();

    assert_eq!(
        commands,
        vec![MotorCommand {
            name: "joint".into(),
            control: 1.0,
        }]
    );
}

#[test]
fn pd_controller_rejects_missing_duplicate_and_non_finite_targets() {
    let current = vec![JointState {
        name: "joint".into(),
        position: 0.0,
        velocity: 0.0,
    }];
    let limits = vec![ActuatorLimit {
        name: "joint".into(),
        minimum: -1.0,
        maximum: 1.0,
    }];

    assert_eq!(
        compute_motor_commands(&[], &current, &limits, PdGains::default()),
        Err(SimulationError::MissingJointTarget("joint".into()))
    );

    let duplicate = vec![
        JointTarget {
            name: "joint".into(),
            position: 0.0,
            velocity: 0.0,
        },
        JointTarget {
            name: "joint".into(),
            position: 0.0,
            velocity: 0.0,
        },
    ];
    assert_eq!(
        compute_motor_commands(&duplicate, &current, &limits, PdGains::default()),
        Err(SimulationError::DuplicateJointTarget)
    );

    let non_finite = vec![JointTarget {
        name: "joint".into(),
        position: f64::NAN,
        velocity: 0.0,
    }];
    assert_eq!(
        compute_motor_commands(&non_finite, &current, &limits, PdGains::default()),
        Err(SimulationError::NonFiniteJointTarget("joint".into()))
    );

    let unexpected = vec![
        JointTarget {
            name: "joint".into(),
            position: 0.0,
            velocity: 0.0,
        },
        JointTarget {
            name: "passive-joint".into(),
            position: 0.0,
            velocity: 0.0,
        },
    ];
    assert_eq!(
        compute_motor_commands(&unexpected, &current, &limits, PdGains::default()),
        Err(SimulationError::UnexpectedJointTarget(
            "passive-joint".into()
        ))
    );
}

#[test]
fn cassie_model_exposes_only_the_ten_actuated_joints() {
    let simulation = load_simulation();
    let state = simulation.state();
    let names: Vec<&str> = state
        .joint_positions
        .iter()
        .map(|joint| joint.name.as_str())
        .collect();

    assert_eq!(names, ACTIVE_JOINT_NAMES);
    assert_eq!(state.joint_velocities.len(), 10);
    assert_eq!(state.actuators.len(), 10);
    assert!(!names.iter().any(|name| name.contains("achilles-rod")));
}

#[test]
fn state_contains_every_mjcf_sensor_and_pelvis_pose() {
    let simulation = load_simulation();
    let state = simulation.state();

    assert_eq!(state.sensors.len(), 20);
    assert_eq!(
        state
            .sensors
            .iter()
            .map(|sensor| sensor.values.len())
            .sum::<usize>(),
        29
    );
    assert!(
        state
            .sensors
            .iter()
            .any(|sensor| sensor.name == "pelvis-orientation")
    );
    assert!(
        state
            .sensors
            .iter()
            .any(|sensor| sensor.name == "pelvis-angular-velocity")
    );
    assert!(all_finite(state.pelvis_pose.position));
    assert!(all_finite(state.pelvis_pose.orientation));
}

#[test]
fn one_control_period_advances_exactly_twenty_simulation_steps() {
    let mut simulation = load_simulation();
    let targets = simulation.hold_current_targets();
    let state = simulation.step_control(&targets).unwrap();

    assert_eq!(STEPS_PER_CONTROL_PERIOD, 20);
    assert!((state.simulation_time - CONTROL_PERIOD).abs() < 1e-12);
}

#[test]
fn simulated_state_remains_finite_for_several_seconds() {
    let mut simulation = load_simulation();
    let targets = simulation.hold_current_targets();
    let mut state = simulation.state();

    for _ in 0..300 {
        state = simulation.step_control(&targets).unwrap();
    }

    assert!((state.simulation_time - 3.0).abs() < 1e-9);
    assert!(all_finite(
        state.joint_positions.iter().map(|joint| joint.value)
    ));
    assert!(all_finite(
        state.joint_velocities.iter().map(|joint| joint.value)
    ));
    assert!(all_finite(
        state
            .sensors
            .iter()
            .flat_map(|sensor| sensor.values.iter().copied())
    ));
    assert!(all_finite(state.actuators.iter().flat_map(|actuator| [
        actuator.control,
        actuator.length,
        actuator.velocity,
        actuator.force,
    ])));
}

#[test]
fn positive_joint_target_produces_a_corresponding_response() {
    let mut simulation = load_simulation();
    let initial = simulation.state();
    let initial_position = initial
        .joint_positions
        .iter()
        .find(|joint| joint.name == "left-hip-roll")
        .unwrap()
        .value;
    let mut targets = simulation.hold_current_targets();
    let target = targets
        .iter_mut()
        .find(|target| target.name == "left-hip-roll")
        .unwrap();
    target.position += 0.03;

    let state = simulation.step_control(&targets).unwrap();
    let final_position = state
        .joint_positions
        .iter()
        .find(|joint| joint.name == "left-hip-roll")
        .unwrap()
        .value;
    let control = state
        .actuators
        .iter()
        .find(|actuator| actuator.name == "left-hip-roll")
        .unwrap()
        .control;

    assert!(control > 0.0);
    assert!(final_position > initial_position);
}

#[test]
fn knee_sine_target_reaches_near_both_mjcf_limits() {
    let simulation = load_simulation();
    let limit = simulation.joint_position_limit("left-knee").unwrap();
    let upper = sinusoidal_joint_target(limit, 0.5, 0.5, 0.95).unwrap();
    let lower = sinusoidal_joint_target(limit, 1.5, 0.5, 0.95).unwrap();
    let expected_margin = (limit.upper - limit.lower) * 0.025;

    assert!((upper.position - (limit.upper - expected_margin)).abs() < 1e-12);
    assert!((lower.position - (limit.lower + expected_margin)).abs() < 1e-12);
    assert!(upper.velocity.abs() < 1e-12);
    assert!(lower.velocity.abs() < 1e-12);
}

#[test]
fn records_every_control_period_and_writes_two_second_dat_file() {
    let mut simulation = load_simulation();
    let mut targets = simulation.hold_current_targets();
    let knee_limits = ["left-knee", "right-knee"]
        .map(|name| simulation.joint_position_limit(name).unwrap().clone());
    let mut log = JointAngleLog::new(&simulation.state());

    for _ in 0..200 {
        let time = simulation.state().simulation_time;
        for limit in &knee_limits {
            let sine_target = sinusoidal_joint_target(limit, time, 0.5, 0.95).unwrap();
            let target_name = sine_target.name.clone();
            *targets
                .iter_mut()
                .find(|target| target.name == target_name)
                .unwrap() = sine_target;
        }
        let state = simulation.step_control(&targets).unwrap();
        log.record(&state).unwrap();
    }

    assert_eq!(log.joint_names(), ACTIVE_JOINT_NAMES);
    assert_eq!(log.samples().len(), 200);
    assert!((log.samples().first().unwrap().simulation_time - 0.01).abs() < 1e-12);
    assert!((log.samples().last().unwrap().simulation_time - 2.0).abs() < 1e-9);

    let output = std::env::temp_dir().join(format!(
        "controller_sim_joint_angles_{}.dat",
        std::process::id()
    ));
    log.write_dat(&output).unwrap();
    let contents = fs::read_to_string(&output).unwrap();
    fs::remove_file(output).unwrap();
    let lines: Vec<&str> = contents.lines().collect();

    assert_eq!(lines.len(), 201);
    assert!(lines[0].starts_with("# time_s left-hip-roll_rad"));
    assert_eq!(lines[1].split_whitespace().count(), 11);
}

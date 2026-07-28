use std::f64::consts::FRAC_PI_2;

use crate::{
    ExternalJacobian, IkError, IkRequest, IkSettings, IkStatus, JointAngles, JointLimit,
    KinematicEvaluation, KinematicsError, LegKinematics, Pose, SettingsError,
    jacobian::to_matrix,
    parse_settings,
    pose::{compose, error},
    solve_ik,
};

const VALID_SETTINGS: &str = "\
max_iterations=1024
pose_tolerance=1e-8
step_tolerance=1e-13
position_weight=0.9999
orientation_weight=0.9999
minimum_bias=1e-3
";

struct SyntheticFiveDofModel {
    names: [String; 5],
    limits: [JointLimit; 5],
    non_finite_jacobian: bool,
}

impl SyntheticFiveDofModel {
    fn new() -> Self {
        Self {
            names: std::array::from_fn(|index| format!("joint-{index}")),
            limits: [JointLimit {
                minimum_rad: -2.0,
                maximum_rad: 2.0,
            }; 5],
            non_finite_jacobian: false,
        }
    }
}

impl LegKinematics for SyntheticFiveDofModel {
    fn joint_names(&self) -> &[String; 5] {
        &self.names
    }

    fn joint_limits(&self) -> &[JointLimit; 5] {
        &self.limits
    }

    fn evaluate(
        &self,
        waist_pose_in_world: &Pose,
        joint_angles: &JointAngles,
    ) -> Result<KinematicEvaluation, KinematicsError> {
        let mut jacobian: ExternalJacobian = [[0.0; 5]; 6];
        for (index, row) in jacobian.iter_mut().take(5).enumerate() {
            row[index] = 1.0;
        }
        if self.non_finite_jacobian {
            jacobian[0][0] = f64::NAN;
        }
        Ok(KinematicEvaluation {
            foot_pose_in_world: Pose {
                position_m: [
                    waist_pose_in_world.position_m[0] + joint_angles.0[0],
                    waist_pose_in_world.position_m[1] + joint_angles.0[1],
                    waist_pose_in_world.position_m[2] + joint_angles.0[2],
                ],
                orientation_rpy_rad: [joint_angles.0[3], joint_angles.0[4], 0.0],
            },
            jacobian,
        })
    }
}

fn default_settings() -> IkSettings {
    parse_settings(VALID_SETTINGS).unwrap()
}

fn default_request() -> IkRequest {
    IkRequest {
        waist_pose_in_world: Pose {
            position_m: [0.0, 0.0, 1.0],
            orientation_rpy_rad: [0.0; 3],
        },
        target_foot_pose_in_world: Pose {
            position_m: [0.2, -0.1, 0.7],
            orientation_rpy_rad: [0.1, -0.2, 0.0],
        },
        initial_angles: JointAngles([0.0; 5]),
        previous_angles: None,
        control_period_s: 0.01,
    }
}

#[test]
fn parses_complete_settings() {
    assert_eq!(
        parse_settings(VALID_SETTINGS).unwrap(),
        IkSettings {
            max_iterations: 1024,
            pose_tolerance: 1e-8,
            step_tolerance: 1e-13,
            position_weight: 0.9999,
            orientation_weight: 0.9999,
            minimum_bias: 1e-3,
        }
    );
}

#[test]
fn permits_blank_lines_and_comments() {
    let source = VALID_SETTINGS.replace("minimum_bias=1e-3", "minimum_bias=1e-3 # invalid");
    assert!(matches!(
        parse_settings(&source),
        Err(SettingsError::InvalidValue { .. })
    ));

    let source = format!("# IK settings\n\n{VALID_SETTINGS}");
    assert!(parse_settings(&source).is_ok());
}

#[test]
fn rejects_missing_unknown_duplicate_and_malformed_settings() {
    let missing = VALID_SETTINGS.replace("minimum_bias=1e-3\n", "");
    assert_eq!(
        parse_settings(&missing),
        Err(SettingsError::MissingKey("minimum_bias"))
    );

    let unknown = format!("{VALID_SETTINGS}gain=1\n");
    assert!(matches!(
        parse_settings(&unknown),
        Err(SettingsError::UnknownKey { .. })
    ));

    let duplicate = format!("{VALID_SETTINGS}max_iterations=2\n");
    assert!(matches!(
        parse_settings(&duplicate),
        Err(SettingsError::DuplicateKey { .. })
    ));

    let malformed = format!("{VALID_SETTINGS}broken\n");
    assert!(matches!(
        parse_settings(&malformed),
        Err(SettingsError::MalformedLine { .. })
    ));
}

#[test]
fn rejects_unparseable_non_finite_and_non_positive_settings() {
    for source in [
        VALID_SETTINGS.replace("max_iterations=1024", "max_iterations=none"),
        VALID_SETTINGS.replace("pose_tolerance=1e-8", "pose_tolerance=NaN"),
        VALID_SETTINGS.replace("step_tolerance=1e-13", "step_tolerance=0"),
        VALID_SETTINGS.replace("position_weight=0.9999", "position_weight=-1"),
    ] {
        assert!(parse_settings(&source).is_err());
    }
}

#[test]
fn composes_child_pose_in_waist_frame_with_zyx_euler_angles() {
    let waist = Pose {
        position_m: [1.0, 2.0, 0.8],
        orientation_rpy_rad: [0.0, 0.0, FRAC_PI_2],
    };
    let child = Pose {
        position_m: [0.5, 0.0, -0.8],
        orientation_rpy_rad: [0.0, 0.0, 0.0],
    };

    let world = compose(&waist, &child);

    assert!((world.position_m[0] - 1.0).abs() < 1e-12);
    assert!((world.position_m[1] - 2.5).abs() < 1e-12);
    assert!(world.position_m[2].abs() < 1e-12);
    assert!((world.orientation_rpy_rad[2] - FRAC_PI_2).abs() < 1e-12);
}

#[test]
fn computes_orientation_error_as_relative_rotation_vector() {
    let current = Pose {
        position_m: [0.0; 3],
        orientation_rpy_rad: [0.0; 3],
    };
    let target = Pose {
        position_m: [0.1, -0.2, 0.3],
        orientation_rpy_rad: [0.0, 0.0, 0.25],
    };

    let pose_error = error(&target, &current);

    assert_eq!(pose_error.fixed_rows::<3>(0).as_slice(), &[0.1, -0.2, 0.3]);
    assert!(pose_error[3].abs() < 1e-12);
    assert!(pose_error[4].abs() < 1e-12);
    assert!((pose_error[5] - 0.25).abs() < 1e-12);
}

#[test]
fn rejects_non_finite_external_jacobian() {
    let mut jacobian = [[0.0; 5]; 6];
    assert!(to_matrix(&jacobian).is_some());
    jacobian[4][2] = f64::NAN;
    assert!(to_matrix(&jacobian).is_none());
}

#[test]
fn solves_reachable_five_dof_pose_and_returns_zero_initial_velocity() {
    let solution = solve_ik(
        &SyntheticFiveDofModel::new(),
        &default_request(),
        &default_settings(),
    )
    .unwrap();

    assert_eq!(solution.status, IkStatus::Converged);
    assert!(solution.iterations > 0);
    assert!(solution.residuals.weighted <= default_settings().pose_tolerance);
    assert_eq!(solution.velocities.0, [0.0; 5]);
    for (actual, expected) in solution.angles.0.iter().zip([0.2, -0.1, -0.3, 0.1, -0.2]) {
        assert!((actual - expected).abs() < 1e-8);
    }
}

#[test]
fn returns_finite_approximation_for_unreachable_sixth_pose_axis() {
    let mut request = default_request();
    request.target_foot_pose_in_world.orientation_rpy_rad[2] = 0.4;

    let solution = solve_ik(&SyntheticFiveDofModel::new(), &request, &default_settings()).unwrap();

    assert_eq!(solution.status, IkStatus::StepTooSmall);
    assert!(solution.residuals.orientation_rad > 0.39);
    assert!(solution.angles.0.iter().all(|value| value.is_finite()));
}

#[test]
fn calculates_joint_velocity_from_previous_cycle() {
    let mut request = default_request();
    request.previous_angles = Some(JointAngles([0.1, -0.2, -0.3, 0.0, -0.1]));

    let solution = solve_ik(&SyntheticFiveDofModel::new(), &request, &default_settings()).unwrap();

    for (actual, expected) in solution
        .velocities
        .0
        .iter()
        .zip([10.0, 10.0, 0.0, 10.0, -10.0])
    {
        assert!((actual - expected).abs() < 1e-6);
    }
}

#[test]
fn reports_max_iterations_with_the_latest_residual() {
    let mut settings = default_settings();
    settings.max_iterations = 1;

    let solution = solve_ik(&SyntheticFiveDofModel::new(), &default_request(), &settings).unwrap();

    assert_eq!(solution.status, IkStatus::MaxIterations);
    assert_eq!(solution.iterations, 1);
    assert!(solution.residuals.weighted.is_finite());
}

#[test]
fn rejects_invalid_runtime_inputs_and_model_outputs() {
    let mut request = default_request();
    request.control_period_s = 0.0;
    assert_eq!(
        solve_ik(&SyntheticFiveDofModel::new(), &request, &default_settings()),
        Err(IkError::InvalidControlPeriod)
    );

    let mut model = SyntheticFiveDofModel::new();
    model.non_finite_jacobian = true;
    assert_eq!(
        solve_ik(&model, &default_request(), &default_settings()),
        Err(IkError::InvalidKinematicEvaluation)
    );
}

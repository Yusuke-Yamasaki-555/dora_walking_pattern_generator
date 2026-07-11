use super::*;

const EPSILON: f64 = 1e-12;

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < EPSILON,
        "expected {expected:.15}, got {actual:.15}"
    );
}

fn assert_error(foot_step: &FootStep, parameters: LipmParameters, expected: GenerationError) {
    assert_eq!(
        generate_com_trajectory(foot_step, parameters).unwrap_err(),
        expected
    );
}

#[test]
fn source_default_input_generates_the_valid_cpp_prefix() {
    let pattern = generate_com_trajectory(&default_foot_step(), DEFAULT_PARAMETERS).unwrap();

    // 最終範囲外アクセスより前の C++ 計算式から得た固定値。C++ の集約初期化に合わせて
    // Z は常に 0 とする。固定値との比較で、将来の変更による数値ずれを検出する。
    assert_eq!(pattern.cc_cog_pos_ref.len(), 554);
    assert_eq!(pattern.cc_cog_vel_ref.len(), 554);
    assert_eq!(pattern.wc_foot_land_pos_ref.len(), 7);
    assert_eq!(pattern.cc_cog_pos_ref[0], [0.0, 0.0, 0.0]);
    assert_close(pattern.cc_cog_pos_ref[1][0], 0.0);
    assert_close(pattern.cc_cog_pos_ref[1][1], 0.000_000_249_720_461_166);
    assert_close(pattern.cc_cog_pos_ref[79][1], 0.017_009_414_478_489);
    assert_close(pattern.cc_cog_pos_ref[80][0], 0.000_000_204_118_178);
    assert_close(pattern.cc_cog_pos_ref[80][1], 0.018_253_034_418_730);
    assert_eq!(pattern.cc_cog_pos_ref[553][2], 0.0);
}

#[test]
fn two_foot_positions_generate_one_complete_segment() {
    let foot_step = FootStep {
        foot_pos: vec![[0.0, 0.0], [0.0, 0.037]],
        waist_height: DEFAULT_PARAMETERS.waist_pos_z,
        walking_step_time: DEFAULT_PARAMETERS.walking_cycle,
    };
    let pattern = generate_com_trajectory(&foot_step, DEFAULT_PARAMETERS).unwrap();

    assert_eq!(pattern.cc_cog_pos_ref.len(), 80);
    assert_eq!(pattern.cc_cog_vel_ref.len(), 80);
    assert_eq!(pattern.wc_foot_land_pos_ref.len(), 1);
}

#[test]
fn globalizes_only_relative_x_positions_after_the_second_foot() {
    assert_eq!(
        globalize_x(&[[1.0, 1.0], [2.0, 2.0], [0.5, 3.0], [-0.25, 4.0]]),
        vec![[1.0, 1.0], [2.0, 2.0], [2.5, 3.0], [2.25, 4.0]]
    );
}

#[test]
fn rejects_too_few_or_non_finite_foot_positions() {
    let mut foot_step = default_foot_step();
    foot_step.foot_pos.truncate(1);
    assert_error(
        &foot_step,
        DEFAULT_PARAMETERS,
        GenerationError::TooFewFootsteps,
    );

    let mut foot_step = default_foot_step();
    foot_step.foot_pos[1][0] = f64::NAN;
    assert_error(
        &foot_step,
        DEFAULT_PARAMETERS,
        GenerationError::NonFiniteFootPosition,
    );
}

#[test]
fn rejects_invalid_or_inconsistent_foot_step_metadata() {
    let mut foot_step = default_foot_step();
    foot_step.waist_height = f64::INFINITY;
    assert_error(
        &foot_step,
        DEFAULT_PARAMETERS,
        GenerationError::NonPositiveFootStepWaistHeight,
    );

    let mut foot_step = default_foot_step();
    foot_step.walking_step_time = 0.0;
    assert_error(
        &foot_step,
        DEFAULT_PARAMETERS,
        GenerationError::NonPositiveFootStepWalkingTime,
    );

    let mut foot_step = default_foot_step();
    foot_step.waist_height += 0.01;
    assert_error(
        &foot_step,
        DEFAULT_PARAMETERS,
        GenerationError::InconsistentWaistHeight,
    );

    let mut foot_step = default_foot_step();
    foot_step.walking_step_time += 0.1;
    assert_error(
        &foot_step,
        DEFAULT_PARAMETERS,
        GenerationError::InconsistentWalkingCycle,
    );
}

#[test]
fn rejects_invalid_calculation_parameters() {
    for (parameters, expected) in [
        (
            LipmParameters {
                control_cycle: f64::NAN,
                ..DEFAULT_PARAMETERS
            },
            GenerationError::NonPositiveControlCycle,
        ),
        (
            LipmParameters {
                walking_cycle: 0.0,
                ..DEFAULT_PARAMETERS
            },
            GenerationError::NonPositiveWalkingCycle,
        ),
        (
            LipmParameters {
                waist_pos_z: f64::INFINITY,
                ..DEFAULT_PARAMETERS
            },
            GenerationError::NonPositiveWaistHeight,
        ),
    ] {
        assert_error(&default_foot_step(), parameters, expected);
    }
}

#[test]
fn supports_other_integral_control_cycle_ratios() {
    let parameters = LipmParameters {
        control_cycle: 0.02,
        ..DEFAULT_PARAMETERS
    };
    let pattern = generate_com_trajectory(&default_foot_step(), parameters).unwrap();

    assert_eq!(pattern.cc_cog_pos_ref.len(), 274);
    assert_eq!(pattern.wc_foot_land_pos_ref.len(), 7);
}

#[test]
fn rejects_a_non_integral_control_cycle_ratio() {
    let parameters = LipmParameters {
        control_cycle: 0.03,
        ..DEFAULT_PARAMETERS
    };
    assert_error(
        &default_foot_step(),
        parameters,
        GenerationError::NonIntegralCycleRatio,
    );
}

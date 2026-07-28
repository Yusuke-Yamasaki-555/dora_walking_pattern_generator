use crate::{IkSettings, SettingsError, parse_settings};

const VALID_SETTINGS: &str = "\
max_iterations=1024
pose_tolerance=1e-8
step_tolerance=1e-13
position_weight=0.9999
orientation_weight=0.9999
minimum_bias=1e-3
";

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

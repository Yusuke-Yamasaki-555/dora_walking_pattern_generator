use std::collections::HashMap;

use crate::SettingsError;

const MAX_ITERATIONS: &str = "max_iterations";
const POSE_TOLERANCE: &str = "pose_tolerance";
const STEP_TOLERANCE: &str = "step_tolerance";
const POSITION_WEIGHT: &str = "position_weight";
const ORIENTATION_WEIGHT: &str = "orientation_weight";
const MINIMUM_BIAS: &str = "minimum_bias";
const KEYS: [&str; 6] = [
    MAX_ITERATIONS,
    POSE_TOLERANCE,
    STEP_TOLERANCE,
    POSITION_WEIGHT,
    ORIENTATION_WEIGHT,
    MINIMUM_BIAS,
];

/// 杉原LM法の反復、収束、重み付けを決める設定。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IkSettings {
    pub max_iterations: usize,
    pub pose_tolerance: f64,
    pub step_tolerance: f64,
    pub position_weight: f64,
    pub orientation_weight: f64,
    pub minimum_bias: f64,
}

/// 厳格な`key=value`文字列をIK設定へ変換する純粋関数。
///
/// 空行と、先頭の空白を除いた後に`#`で始まる行だけを無視する。
pub fn parse_settings(source: &str) -> Result<IkSettings, SettingsError> {
    let mut values = HashMap::new();

    for (index, source_line) in source.lines().enumerate() {
        let line_number = index + 1;
        let line = source_line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        let mut fields = line.split('=');
        let key = fields.next().unwrap_or_default().trim();
        let value = fields.next().map(str::trim);
        if key.is_empty() || value.is_none() || value == Some("") || fields.next().is_some() {
            return Err(SettingsError::MalformedLine { line: line_number });
        }
        if !KEYS.contains(&key) {
            return Err(SettingsError::UnknownKey {
                line: line_number,
                key: key.to_owned(),
            });
        }
        if values.insert(key, (line_number, value.unwrap())).is_some() {
            return Err(SettingsError::DuplicateKey {
                line: line_number,
                key: key.to_owned(),
            });
        }
    }

    let max_iterations = parse_usize(&values, MAX_ITERATIONS)?;
    let settings = IkSettings {
        max_iterations,
        pose_tolerance: parse_f64(&values, POSE_TOLERANCE)?,
        step_tolerance: parse_f64(&values, STEP_TOLERANCE)?,
        position_weight: parse_f64(&values, POSITION_WEIGHT)?,
        orientation_weight: parse_f64(&values, ORIENTATION_WEIGHT)?,
        minimum_bias: parse_f64(&values, MINIMUM_BIAS)?,
    };
    validate(settings)?;
    Ok(settings)
}

fn parse_usize(
    values: &HashMap<&str, (usize, &str)>,
    key: &'static str,
) -> Result<usize, SettingsError> {
    let (line, value) = values
        .get(key)
        .copied()
        .ok_or(SettingsError::MissingKey(key))?;
    value.parse().map_err(|_| SettingsError::InvalidValue {
        line,
        key: key.to_owned(),
    })
}

fn parse_f64(
    values: &HashMap<&str, (usize, &str)>,
    key: &'static str,
) -> Result<f64, SettingsError> {
    let (line, value) = values
        .get(key)
        .copied()
        .ok_or(SettingsError::MissingKey(key))?;
    value.parse().map_err(|_| SettingsError::InvalidValue {
        line,
        key: key.to_owned(),
    })
}

fn validate(settings: IkSettings) -> Result<(), SettingsError> {
    if settings.max_iterations == 0 {
        return Err(SettingsError::OutOfRange(MAX_ITERATIONS));
    }
    for (key, value) in [
        (POSE_TOLERANCE, settings.pose_tolerance),
        (STEP_TOLERANCE, settings.step_tolerance),
        (POSITION_WEIGHT, settings.position_weight),
        (ORIENTATION_WEIGHT, settings.orientation_weight),
        (MINIMUM_BIAS, settings.minimum_bias),
    ] {
        if !value.is_finite() || value <= 0.0 {
            return Err(SettingsError::OutOfRange(key));
        }
    }
    Ok(())
}

pub(crate) fn invalid_field(settings: &IkSettings) -> Option<&'static str> {
    if settings.max_iterations == 0 {
        return Some(MAX_ITERATIONS);
    }
    [
        (POSE_TOLERANCE, settings.pose_tolerance),
        (STEP_TOLERANCE, settings.step_tolerance),
        (POSITION_WEIGHT, settings.position_weight),
        (ORIENTATION_WEIGHT, settings.orientation_weight),
        (MINIMUM_BIAS, settings.minimum_bias),
    ]
    .into_iter()
    .find_map(|(key, value)| (!value.is_finite() || value <= 0.0).then_some(key))
}

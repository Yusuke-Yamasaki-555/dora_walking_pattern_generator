//! 線形倒立振子モデル（LIPM）を用いた重心軌道生成。
//!
//! 公開する入出力の形は ROS 2 のプラグイン基底クラスに合わせている。一方で計算自体は
//! ROS 2・Dora に依存しない。そのため、単体テストで検証し、将来 Dora ノードへ組み込んでも
//! 同じ計算結果を利用できる。

// `fmt` はエラー値を人間が読める文字列に変換する `Display` の実装で使用する。
use std::fmt;

/// 2 次元の点。添字 0 は X、添字 1 は Y を表す。
pub type Point2 = [f64; 2];
/// 3 次元の点。添字 0, 1, 2 は順に X, Y, Z を表す。
pub type Point3 = [f64; 3];

/// C++ の `control_plugin_base::FootStep` に対応する入力。
#[derive(Clone, Debug, PartialEq)]
pub struct FootStep {
    /// 足先の着地位置列。各要素は `[x, y]` [m] である。
    pub foot_pos: Vec<Point2>,
    /// 腰の高さ [m]。移植元との入出力互換性のために保持する。
    pub waist_height: f64,
    /// 一歩の時間 [s]。移植元との入出力互換性のために保持する。
    pub walking_step_time: f64,
}

/// 以前は ROS 2 のパラメータサーバーから取得していた計算パラメータ。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LipmParameters {
    /// 制御周期 [s]。出力軌道のサンプリング間隔でもある。
    pub control_cycle: f64,
    /// 歩行周期 [s]。支持脚を切り替える周期である。
    pub walking_cycle: f64,
    /// LIPM の高さ [m]。時定数 `sqrt(z / g)` を求めるために使用する。
    pub waist_pos_z: f64,
}

/// C++ の `control_plugin_base::WalkingPattern` に対応する出力。
///
/// 移植元 C++ は 3 要素配列へ X/Y だけを初期化している。そのため互換性のため Z は `0.0`
/// のまま保持する。
#[derive(Clone, Debug, PartialEq)]
pub struct WalkingPattern {
    /// 各制御周期における重心位置参照値 `[x, y, z]` [m]。
    pub cc_cog_pos_ref: Vec<Point3>,
    /// 各制御周期における重心速度参照値 `[vx, vy, vz]` [m/s]。
    pub cc_cog_vel_ref: Vec<Point3>,
    /// 各歩行素片で用いる補正済み着地位置 `[x, y]` [m]。
    pub wc_foot_land_pos_ref: Vec<Point2>,
}

/// 入力値が物理的・計算的に不正な場合に返すエラー。
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GenerationError {
    TooFewFootsteps,
    NonPositiveControlCycle,
    NonPositiveWalkingCycle,
    NonPositiveWaistHeight,
}

impl fmt::Display for GenerationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::TooFewFootsteps => "足位置は少なくとも 2 点必要です",
            Self::NonPositiveControlCycle => "control_cycle は正でなければなりません",
            Self::NonPositiveWalkingCycle => "walking_cycle は正でなければなりません",
            Self::NonPositiveWaistHeight => "waist_pos_z は正でなければなりません",
        };
        formatter.write_str(message)
    }
}

impl std::error::Error for GenerationError {}

/// 一つの歩行素片を評価するために必要な状態。
///
/// 外部へ公開しないため、生成処理内でのみ使う小さな値オブジェクトとして定義している。
#[derive(Clone, Copy)]
struct SegmentState {
    /// 現在の支持脚位置の添字。
    index: usize,
    /// 現在の歩行素片を開始してからの時刻 [s]。
    t: f64,
    /// この歩行素片の先頭における重心位置 `[x, y]`。
    cog_start: Point2,
    /// この歩行素片の先頭における重心速度 `[vx, vy]`。
    velocity_start: Point2,
    /// 最適化で求めた、この歩行素片の着地位置。
    foot_land: Point2,
}

/// 重心（CoM）参照軌道を生成する。
///
/// これは純粋関数である。設定はすべて引数で渡し、ファイル・時刻・ROS 2 クライアント・
/// Dora API にはアクセスしない。サンプル時刻と計算式は
/// `LinearInvertedPendulumModel.cpp` を再現している。ただし移植元は最終支持脚切替時に
/// `foot_pos` の範囲外を読むため、本実装は有効な軌道までを返す（詳細は移植元の
/// `DEFECTS.md` を参照）。
pub fn generate_com_trajectory(
    foot_step: &FootStep,
    parameters: LipmParameters,
) -> Result<WalkingPattern, GenerationError> {
    // `?` はエラーなら直ちに呼び出し元へ返し、成功時だけ次の処理へ進める。
    validate(foot_step, parameters)?;

    // C++ と同様に、3 番目以降の X 座標を局所座標からグローバル座標へ積算する。
    let global_foot_pos = globalize_x(&foot_step.foot_pos);
    // LIPM の時定数 Tc = sqrt(z / g) を求める。重力加速度 g は 9.81 m/s^2。
    let time_constant = (parameters.waist_pos_z / 9.81).sqrt();
    // 最初の支持脚と次の支持脚、および初期状態から最初の補正着地位置を計算する。
    let initial_landing = landing_position(
        global_foot_pos[0],
        global_foot_pos[1],
        [0.0, 0.0],
        [0.0, 0.0],
        parameters.walking_cycle,
        time_constant,
    );
    // 最初の歩行素片は、重心位置・速度ともに 0 から開始する。
    let initial = SegmentState {
        index: 0,
        t: 0.0,
        cog_start: [0.0, 0.0],
        velocity_start: [0.0, 0.0],
        foot_land: initial_landing,
    };

    // 移植元はループ時刻に `float` を使う。実際の力学計算は C++ と同じ f64 のままにしつつ、
    // 終了条件だけは f32 にしてサンプル数を互換にする。
    let max_time = (parameters.walking_cycle as f32) * ((global_foot_pos.len() - 1) as f32);
    let mut elapsed = 0.0_f32;
    let mut state = initial;
    let mut pattern = WalkingPattern {
        cc_cog_pos_ref: Vec::new(),
        cc_cog_vel_ref: Vec::new(),
        wc_foot_land_pos_ref: vec![initial_landing],
    };

    // 時刻が最終歩行時刻を超えるまで、一制御周期ずつ参照値を追加する。
    while elapsed <= max_time {
        // 現在の状態と時定数から、この時刻の重心位置・速度を評価する。
        let (position, velocity) = evaluate_segment(state, time_constant);
        // C++ の 3 要素配列と合わせ、未使用の Z 成分には 0.0 を格納する。
        pattern.cc_cog_pos_ref.push([position[0], position[1], 0.0]);
        pattern.cc_cog_vel_ref.push([velocity[0], velocity[1], 0.0]);

        if state.t >= parameters.walking_cycle - 0.01 {
            // この時点で次の歩行素片を作る。次の次の足位置がなければ移植元は範囲外を
            // 読むため、Rust では有効な最後のサンプルを残して終了する。
            if state.index + 2 >= global_foot_pos.len() {
                break;
            }
            // 支持脚の添字を一つ進め、現在の終端状態を次の素片の初期状態にする。
            let next_index = state.index + 1;
            let next_landing = landing_position(
                global_foot_pos[next_index],
                global_foot_pos[next_index + 1],
                position,
                velocity,
                parameters.walking_cycle,
                time_constant,
            );
            // 新しい補正着地位置を出力にも保存し、次の素片の状態を丸ごと置き換える。
            pattern.wc_foot_land_pos_ref.push(next_landing);
            state = SegmentState {
                index: next_index,
                // 移植元の応急処置を再現し、切替直後の時刻を 0.01 s とする。
                t: 0.01,
                cog_start: position,
                velocity_start: velocity,
                foot_land: next_landing,
            };
        } else {
            // 同じ歩行素片を継続する場合は、その素片内の時刻だけを進める。
            state.t += parameters.control_cycle;
        }
        // 次の出力サンプルのため、全体の経過時間を一制御周期進める。
        elapsed += parameters.control_cycle as f32;
    }

    Ok(pattern)
}

fn validate(foot_step: &FootStep, parameters: LipmParameters) -> Result<(), GenerationError> {
    // 2 点未満では「現在」と「次」の足位置を選べず、軌道を定義できない。
    if foot_step.foot_pos.len() < 2 {
        return Err(GenerationError::TooFewFootsteps);
    }
    if parameters.control_cycle <= 0.0 {
        return Err(GenerationError::NonPositiveControlCycle);
    }
    if parameters.walking_cycle <= 0.0 {
        return Err(GenerationError::NonPositiveWalkingCycle);
    }
    if parameters.waist_pos_z <= 0.0 {
        return Err(GenerationError::NonPositiveWaistHeight);
    }
    Ok(())
}

fn globalize_x(local_foot_pos: &[Point2]) -> Vec<Point2> {
    // `iter` は入力を変更せずに各要素を参照する。`enumerate` は添字も同時に返す。
    local_foot_pos
        .iter()
        .enumerate()
        // `scan` は前回の X 座標を状態として保持し、各要素から新しい点を返す。
        .scan(0.0, |previous_x, (index, position)| {
            // 最初の 2 点は絶対座標、それ以降は前のグローバル X に局所 X を足す。
            let x = if index > 1 {
                position[0] + *previous_x
            } else {
                position[0]
            };
            // 次の繰り返しで使うため、状態を今回のグローバル X で更新する。
            *previous_x = x;
            // Y は移植元と同じくそのまま引き継ぎ、生成した点を `Some` で返す。
            Some([x, position[1]])
        })
        .collect()
}

fn evaluate_segment(state: SegmentState, time_constant: f64) -> (Point2, Point2) {
    // LIPM の解析解で共通に使う sinh(t/Tc) と cosh(t/Tc) を先に計算する。
    let sinh = (state.t / time_constant).sinh();
    let cosh = (state.t / time_constant).cosh();
    // `point_zip` により X/Y へ同じ演算を適用し、(p0 - u) cosh(t/Tc) を求める。
    let position = point_zip(state.cog_start, state.foot_land, |start, land| {
        (start - land) * cosh
    });
    // 同様に、位置式を微分した最初の速度項を X/Y それぞれに求める。
    let velocity = point_zip(state.cog_start, state.foot_land, |start, land| {
        ((start - land) / time_constant) * sinh
    });
    // タプル `(位置, 速度)` として返す。配列の添字 0/1 は X/Y 成分である。
    (
        [
            position[0] + time_constant * state.velocity_start[0] * sinh + state.foot_land[0],
            position[1] + time_constant * state.velocity_start[1] * sinh + state.foot_land[1],
        ],
        [
            velocity[0] + state.velocity_start[0] * cosh,
            velocity[1] + state.velocity_start[1] * cosh,
        ],
    )
}

fn landing_position(
    current_foot: Point2,
    next_foot: Point2,
    cog_start: Point2,
    velocity_start: Point2,
    walking_cycle: f64,
    time_constant: f64,
) -> Point2 {
    // 一歩分の時間での双曲線関数を計算する。
    let sinh = (walking_cycle / time_constant).sinh();
    let cosh = (walking_cycle / time_constant).cosh();
    // C++ の最適化で使う位置・速度誤差の重みをそのまま使う。
    let position_weight = 10.0;
    let velocity_weight = 1.0;
    // 評価関数を微分して得られる、X/Y に共通の分母を計算する。
    let denominator =
        position_weight * (cosh - 1.0).powi(2) + velocity_weight * (sinh / time_constant).powi(2);
    // 現在と次の足の中点を、X/Y へ同じクロージャ（無名関数）を適用して求める。
    let midpoint = point_zip(current_foot, next_foot, |current, next| {
        (next - current) / 2.0
    });
    // 歩行素片終端で目標にする重心位置は、現在の足から中点へ進んだ位置である。
    let desired_position = point_zip(current_foot, midpoint, |current, offset| current + offset);
    let desired_velocity = [
        ((cosh + 1.0) / (time_constant * sinh)) * midpoint[0],
        ((cosh - 1.0) / (time_constant * sinh)) * midpoint[1],
    ];
    // `[0, 1].map` により、同じ最適化式を X（0）と Y（1）へ適用して配列で返す。
    [0, 1].map(|axis| {
        -((position_weight * (cosh - 1.0)) / denominator)
            * (desired_position[axis]
                - cosh * cog_start[axis]
                - time_constant * sinh * velocity_start[axis])
            - ((velocity_weight * sinh) / (time_constant * denominator))
                * (desired_velocity[axis]
                    - (sinh / time_constant) * cog_start[axis]
                    - cosh * velocity_start[axis])
    })
}

fn point_zip(left: Point2, right: Point2, operation: impl Fn(f64, f64) -> f64) -> Point2 {
    // 配列を手作業で二つ書き換えず、渡された関数を X/Y の各成分へ適用する。
    [operation(left[0], right[0]), operation(left[1], right[1])]
}

/// 移植元の既定フットステッププランナが出力する入力例。
pub fn default_foot_step() -> FootStep {
    FootStep {
        foot_pos: vec![
            [0.0, 0.0],
            [0.0, 0.037],
            [0.03, -0.037],
            [0.03, 0.037],
            [0.03, -0.037],
            [0.03, 0.037],
            [0.0, 0.0],
            [0.0, 0.0],
        ],
        waist_height: 0.171_856,
        walking_step_time: 0.8,
    }
}

/// `robot_bringup/config/param_control.yaml` に記載された既定パラメータ。
pub const DEFAULT_PARAMETERS: LipmParameters = LipmParameters {
    control_cycle: 0.01,
    walking_cycle: 0.8,
    waist_pos_z: 0.171_856,
};

// Dora ノード接続のひな形。Dora のデータ型・ポート名が決まるまでコメントアウトしている。
//
// #[dora::main]
// fn main() -> Result<(), Box<dyn std::error::Error>> {
//     let (mut node, mut events) = dora::Node::init_from_env()?;
//     while let Some(event) = events.recv() {
//         let foot_step: FootStep = decode_foot_step(event)?;
//         let pattern = generate_com_trajectory(&foot_step, DEFAULT_PARAMETERS)?;
//         node.send_output("walking_pattern", encode_walking_pattern(&pattern))?;
//     }
//     Ok(())
// }

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-12;

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < EPSILON,
            "expected {expected:.15}, got {actual:.15}"
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
    fn input_validation_is_explicit() {
        let error = generate_com_trajectory(
            &FootStep {
                foot_pos: vec![[0.0, 0.0]],
                waist_height: 0.1,
                walking_step_time: 0.8,
            },
            DEFAULT_PARAMETERS,
        )
        .unwrap_err();
        assert_eq!(error, GenerationError::TooFewFootsteps);
    }
}

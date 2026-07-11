//! 歩行制御モジュールが生成したデータの記録・可視化。
//!
//! このクレートはデータ生成を担当せず、受け取った値をプロットやログへ変換する。
//! 現在は重心軌道の PNG 出力のみを提供する。将来は Dora の独立ノードとしてデータを
//! subscribe し、CSV 保存とリアルタイム表示を行う境界として拡張する。

use std::path::Path;

use com_trajectory::WalkingPattern;
use gnuplot::{AxesCommon, Caption, Color, Figure, RGBString};

/// 重心軌道のプロットに必要な一次元系列。
///
/// `from_pattern` は副作用を持たず、制御モジュールの出力を描画ライブラリから独立した値へ
/// 変換する。CSV 保存や別の描画バックエンドからも同じ系列を再利用できる。
#[derive(Clone, Debug, PartialEq)]
pub struct ComTrajectorySeries {
    pub time: Vec<f64>,
    pub x: Vec<f64>,
    pub y: Vec<f64>,
}

impl ComTrajectorySeries {
    /// 軌道とサンプリング周期からプロット用の系列を構築する。
    pub fn from_pattern(pattern: &WalkingPattern, control_cycle: f64) -> Self {
        let time = (0..pattern.cc_cog_pos_ref.len())
            .map(|index| index as f64 * control_cycle)
            .collect();
        let x = pattern
            .cc_cog_pos_ref
            .iter()
            .map(|point| point[0])
            .collect();
        let y = pattern
            .cc_cog_pos_ref
            .iter()
            .map(|point| point[1])
            .collect();

        Self { time, x, y }
    }
}

/// 渡された重心軌道を PNG ファイルへ描画する。
///
/// データ生成は呼び出し側の責務とし、この関数は変換済み系列を gnuplot へ渡す副作用の境界
/// だけを担う。
pub fn plot_com_trajectory(
    pattern: &WalkingPattern,
    control_cycle: f64,
    output_path: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    let series = ComTrajectorySeries::from_pattern(pattern, control_cycle);
    let output = output_path.to_string_lossy();
    let mut figure = Figure::new();
    figure.set_terminal("pngcairo size 1000,900", &output);
    figure.set_multiplot_layout(2, 1);
    figure
        .axes2d()
        .set_title("重心軌道: X-Y", &[])
        .set_x_label("X [m]", &[])
        .set_y_label("Y [m]", &[])
        .lines(
            &series.x,
            &series.y,
            &[Caption("重心"), Color(RGBString("blue"))],
        );
    figure
        .axes2d()
        .set_title("重心軌道: 時間-Y", &[])
        .set_x_label("時間 [s]", &[])
        .set_y_label("Y [m]", &[])
        .lines(
            &series.time,
            &series.y,
            &[Caption("重心"), Color(RGBString("blue"))],
        );
    figure.show()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converts_a_pattern_into_plot_series() {
        let pattern = WalkingPattern {
            cc_cog_pos_ref: vec![[1.0, 2.0, 0.0], [3.0, 4.0, 0.0]],
            cc_cog_vel_ref: Vec::new(),
            wc_foot_land_pos_ref: Vec::new(),
        };

        assert_eq!(
            ComTrajectorySeries::from_pattern(&pattern, 0.1),
            ComTrajectorySeries {
                time: vec![0.0, 0.1],
                x: vec![1.0, 3.0],
                y: vec![2.0, 4.0],
            }
        );
    }
}

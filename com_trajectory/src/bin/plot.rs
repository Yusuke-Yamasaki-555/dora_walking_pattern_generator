use com_trajectory::{DEFAULT_PARAMETERS, default_foot_step, generate_com_trajectory};
use gnuplot::{AxesCommon, Caption, Color, Figure, RGBString};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 移植元と同じ既定入力・パラメータで、描画対象の軌道を一度だけ生成する。
    let pattern = generate_com_trajectory(&default_foot_step(), DEFAULT_PARAMETERS)?;
    // 各サンプル番号へ制御周期を掛け、横軸に使う時刻列を作る。
    let time: Vec<f64> = (0..pattern.cc_cog_pos_ref.len())
        .map(|index| index as f64 * DEFAULT_PARAMETERS.control_cycle)
        .collect();
    // 位置参照列から、グラフ用に X 成分だけを取り出す。
    let x: Vec<f64> = pattern
        .cc_cog_pos_ref
        .iter()
        .map(|point| point[0])
        .collect();
    // 同様に Y 成分だけを取り出し、二つのグラフで共用する。
    let y: Vec<f64> = pattern
        .cc_cog_pos_ref
        .iter()
        .map(|point| point[1])
        .collect();

    // `env!` はコンパイル時の Cargo プロジェクトの絶対パスを埋め込むマクロである。
    let output = concat!(env!("CARGO_MANIFEST_DIR"), "/plots/com_trajectory.png");
    // Figure は gnuplot へ渡すグラフ全体を表す値である。
    let mut figure = Figure::new();
    // PNG 出力先と画像サイズを設定する。
    figure.set_terminal("pngcairo size 1000,900", output);
    // 2 行 1 列のマルチプロットにし、同じ画像内で上下に配置する。
    figure.set_multiplot_layout(2, 1);
    figure
        .axes2d()
        .set_title("重心軌道: X-Y", &[])
        .set_x_label("X [m]", &[])
        .set_y_label("Y [m]", &[])
        .lines(&x, &y, &[Caption("重心"), Color(RGBString("blue"))]);
    figure
        .axes2d()
        .set_title("重心軌道: 時間-Y", &[])
        .set_x_label("時間 [s]", &[])
        .set_y_label("Y [m]", &[])
        .lines(&time, &y, &[Caption("重心"), Color(RGBString("blue"))]);
    // gnuplot を実行して PNG ファイルを作成する。失敗時は `?` でエラーを返す。
    figure.show()?;
    println!("出力しました: {output}");
    Ok(())
}

use std::path::PathBuf;

use com_trajectory::{DEFAULT_PARAMETERS, default_foot_step, generate_com_trajectory};
use walking_logger::plot_com_trajectory;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 現在は動作確認用に軌道を生成する。Dora 接続後は、この入力元だけを subscriber が
    // 受信したデータへ置き換え、描画処理はライブラリとしてそのまま利用する。
    let pattern = generate_com_trajectory(&default_foot_step(), DEFAULT_PARAMETERS)?;
    let output = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("plots")
        .join("com_trajectory.png");
    if let Some(parent) = output.parent() {
        std::fs::create_dir_all(parent)?;
    }
    plot_com_trajectory(&pattern, DEFAULT_PARAMETERS.control_cycle, &output)?;
    println!("出力しました: {}", output.display());
    Ok(())
}

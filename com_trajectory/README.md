# com_trajectory

ROS 2 の `LinearInvertedPendulumModel` と互換な、線形倒立振子モデル（LIPM）による重心軌道生成ライブラリです。計算は副作用のない `generate_com_trajectory` 関数に分離されています。

```bash
cargo test
cargo run --bin plot
```

後者は `plots/com_trajectory.png` に、上段 `X-Y`、下段 `time-Y` のグラフを出力します。入力例とパラメータは移植元の既定フットステッププランナと `param_control.yaml` に対応します。

`src/lib.rs` の末尾に、将来の Dora ノード接続箇所をコメント付きで示しています。Dora のデータ型・ポート名が確定した時点で、その境界だけを有効化すれば計算コアはそのまま使用できます。

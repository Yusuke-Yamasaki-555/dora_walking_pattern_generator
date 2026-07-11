# walking_logger

歩行制御モジュールが publish するデータを記録・可視化するためのクレートです。現在は、
`com_trajectory` の重心軌道をデモ入力として生成し、PNG へ描画できます。

```bash
cargo run -p walking_logger
```

出力先は `walking_logger/plots/com_trajectory.png` です。

## 将来の実行構成

`walking_logger` は Dora の独立ノードとして動作し、制御モジュールとはプロセスを分離します。
制御モジュールが publish したデータを subscribe し、次の処理を担う予定です。

- 軌道を PNG などへ描画する
- CSV などへ数値を保存する
- 実行中のデータを逐次記録する
- 複数の制御モジュールの出力を比較する

制御データの受信・保存周期と描画周期は分離します。すべての受信サンプルを履歴へ反映しつつ、
描画はより低い周期で行い、可視化の負荷を制御処理から隔離します。

# dora_walking_pattern_generator

RustとDoraへの移植を進めている二足歩行パターン生成・シミュレーション用workspaceです。
現在は、LIPMによる重心軌道生成、軌道の可視化、MuJoCo上のCassie暫定コントローラを
含みます。

## Workspace構成

- `com_trajectory`: 線形倒立振子モデルによる重心軌道生成
- `walking_logger`: 軌道データのロギング・PNG可視化
- `controller_sim`: MuJoCo Cassieへの制御入力と状態読み出し

## 必要な環境

現在の開発・動作確認環境はArch Linux、COSMIC、Waylandです。

- Rust toolchain（edition 2024対応）
- MuJoCo 3.9.0
- gnuplot
- Git

Arch LinuxのAUR版MuJoCoは`mujoco.pc`を提供しないため、`.cargo/config.toml`で
`MUJOCO_DYNAMIC_LINK_DIR=/usr/lib`を設定しています。`mujoco-rs 5.0.0`はMuJoCo 3.9.0を
対象としているため、異なるMuJoCoバージョンとの組み合わせは使用しません。

## 初回セットアップ

リポジトリをcloneします。

```bash
git clone git@github.com:Yusuke-Yamasaki-555/dora_walking_pattern_generator.git
cd dora_walking_pattern_generator
```

CassieのMJCFとアセットは本リポジトリへコピーせず、公式`mujoco_menagerie`から
git sparse-checkoutで取得します。

```bash
git clone --filter=blob:none --no-checkout \
  https://github.com/google-deepmind/mujoco_menagerie.git mujoco_menagerie
git -C mujoco_menagerie sparse-checkout set agility_cassie
git -C mujoco_menagerie checkout main
```

`mujoco_menagerie/`は`.gitignore`対象です。MJCF、アセット、LICENSEは公式リポジトリの
管理単位のままローカルで参照します。

## ビルドとテスト

workspace全体をビルドします。

```bash
cargo build --workspace
```

全テストとClippyを実行します。

```bash
cargo test --workspace
cargo clippy --workspace --all-targets --all-features -- -D warnings
```

## Cassieシミュレータの起動

Wayland上でCassie viewerを起動します。

```bash
cargo run -p controller_sim
```

シミュレーション周期はCassie MJCF既定の0.5 ms、コントローラ周期は10 msです。デモでは
`home`姿勢から開始し、左右の膝へ可動範囲近くまで使う正弦波目標を入力します。2秒後に終了し、
10 msごとの全能動関節角度を`controller_sim/logs/joint_angles.dat`へ出力します。

詳細は[`controller_sim/README.md`](controller_sim/README.md)を参照してください。

## 重心軌道の生成と描画

既定入力のLIPM重心軌道を生成し、PNGへ描画します。

```bash
cargo run -p walking_logger
```

出力先は`walking_logger/plots/com_trajectory.png`です。軌道生成ライブラリだけをテストする
場合は次を実行します。

```bash
cargo test -p com_trajectory
```

## Dora対応

計算コアはDoraから独立した値変換として実装しています。各クレートには、将来Arrowスキーマと
ポート名が確定した後に有効化するDoraノード境界のひな形をコメントアウトして配置しています。

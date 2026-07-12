# controller_sim

MuJoCo上のAgility Cassieへ関節目標を入力し、ロボット状態を読み出す暫定コントローラです。
制御則を純粋関数として分離し、MuJoCoの更新と描画を副作用境界に閉じ込めています。

## モデル

CassieのMJCFとアセットはライセンスおよび上流履歴を保持するため、本リポジトリへコピー
せず、`google-deepmind/mujoco_menagerie`を独立したローカルリポジトリとして取得します。
必要な`agility_cassie`だけを展開するため、Gitのsparse-checkoutを使用します。

```bash
git clone --filter=blob:none --no-checkout \
  https://github.com/google-deepmind/mujoco_menagerie.git mujoco_menagerie
git -C mujoco_menagerie sparse-checkout set agility_cassie
git -C mujoco_menagerie checkout main
```

`mujoco_menagerie/`は本リポジトリの`.gitignore`対象であり、モデルやLICENSEを重複して
コミットしません。使用するファイルは`mujoco_menagerie/agility_cassie/scene.xml`です。
MuJoCo 3.9.0と`mujoco-rs 5.0.0`を使用します。Arch LinuxのAUR版MuJoCoはpkg-config
ファイルを持たないため、workspaceの`.cargo/config.toml`で`/usr/lib`を指定しています。

## 周期

- コントローラ周期: 10 ms
- MJCF既定のシミュレーション周期: 0.5 ms
- 制御1周期あたりのシミュレーション: 20 step

## テスト

```bash
cargo test -p controller_sim
```

テストはモデルの読み込み、能動関節の名前対応、全センサの読み出し、周期、有限値、制御入力に
対する関節応答を検証します。ウィンドウ描画はディスプレイ環境に依存するため自動テストには
含めません。

## Wayland上での描画確認

```bash
cargo run -p controller_sim
```

デモはCassieを`home`キーフレームから開始し、左右のknee関節へ0.5 Hzの正弦波目標を
同相で与えます。振幅はMJCF可動半幅の95%で、2秒間に上下限近傍を一巡します。

10 msの制御周期ごとに全10能動関節の角度を記録し、2秒後に次の空白区切りdatファイルを
出力して終了します。

```text
controller_sim/logs/joint_angles.dat
```

先頭行は時刻と関節名のヘッダーで、以降の200行が`time [s]`と`joint angle [rad]`です。

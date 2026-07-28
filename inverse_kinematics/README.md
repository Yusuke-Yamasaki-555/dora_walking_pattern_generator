# inverse_kinematics

Cassieの片脚5能動関節を対象とする、杉原Levenberg-Marquardt法の数値IKクレートです。
目標はワールド座標系の足先6次元Poseで、5自由度で完全に到達できない場合も重み付き残差を
最小化した有限な近似解と終了状態を返します。

## 境界

- 距離は[m]、角度は[rad]です。
- Pose姿勢は`[roll, pitch, yaw]`で、右手系の`Rz(yaw) * Ry(pitch) * Rx(roll)`です。
- IKの基準は腰Poseです。
- `LegKinematics`は、関節名、可動範囲、候補角における足先Poseと6x5ヤコビアンを返します。
- Cassieの受動関節と閉リンク拘束は、Issue #8側が解いた有効運動学として渡します。
- このクレートはCassieを単純な5関節直列リンクとして近似しません。

Issue #8が未実装のため、現時点のテストでは同じ公開契約を満たす運動学スタブを使用します。
実Cassieへの接続完了には、Issue #8から閉リンク拘束整合済みの`LegKinematics`実装が必要です。

## 設定

既定の調整値は`config/ik.conf`にあります。ファイルI/OはDoraノードなど最上位の境界で行い、
純粋な`parse_settings`へ文字列を渡します。

```rust
let source = std::fs::read_to_string("inverse_kinematics/config/ik.conf")?;
let settings = inverse_kinematics::parse_settings(&source)?;
```

未知キー、重複、欠落、非有限値、非正値は拒否されます。

## 検証

```bash
cargo test -p inverse_kinematics
cargo clippy -p inverse_kinematics --all-targets -- -D warnings
```

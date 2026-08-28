# HGV機動対艦ミサイル開発プロジェクト

Stormworks上で、敵艦の防空ミサイルをレーダー警戒し、回避機動と再現可能な上下機動を行う対艦ミサイルを開発する独立プロジェクトです。完成後は対艦攻撃シナリオと防空システム評価の両方に利用します。

## 現在の状態

防空ミサイル側からコピーした `ADSV3 Missile.lua` と `ADSV3 MissileKF.lua` を参照元として、HGV専用の初版を実装しました。

- `HGV Radar.lua`：敵艦方位の3点探索、SAM判定、終末レーダー固定、シースキミング入力のパススルー
- `HGV Guidance.lua`：通常HGVのSEARCH/EVADE上下蛇行、SAM速度に直交するビーム機動、選択式シースキミングとポップアップ終末誘導

Lua 5.3構文、重大診断、オフライン状態遷移試験、LifeBoat圧縮後文字数は確認済みです。2026-08-27時点の現行ソースはシースキミング高度10 m、ポップアップ距離・高度をプロパティ化、DIVE距離700 mであり、初期合意値との差異は採用確認TBDです。Stormworks実機応答は未確認です。
2026-08-26の再試験ではSAM探知とEVADE遷移が動作しましたが、接近時にSAMとHGVの進路交差角が浅くなりました。ビーム基準をSAMへの視線からSAM推定水平速度へ変更し、常に直交方向を指令する版のStormworks再試験待ちです。

`AA Missile`とはプロジェクトを分離しています。HGV側の開発中に防空ミサイルを変更せず、相互試験が必要になった段階で試験条件だけを接続します。

## 採用した初期要求

- 敵艦位置・速度・距離はデータリンクから取得する
- レーダー有効射程は1900 m、探知間隔は1 tick
- レーダー指向範囲は方位±180度、仰角±45度
- ビーム全幅は水平21.6度、垂直75.6度
- 敵艦方向の約±30度を3 tickで探索する
- 250 m/s以上でHGV方向へ相対接近する目標をSAM候補とする
- SAM失探または最接近点通過で回避を終了する
- 通常HGVのEVADE中はSAM推定水平速度に対して約90度の水平進路を指令しながら上下蛇行する
- 通常HGVのSEARCH/EVADE中は高度2000 mを中心に90 tickごとに上下を反転する
- 通常モードでは敵艦までの水平距離が1500 m以下になったらTERMINALへ不可逆遷移する
- TERMINALではレーダーを敵艦方向へ固定し、終末誘導を優先する
- Radar Bool入力ch2でシースキミングを選択し、Radar Bool出力ch5からGuidance Bool入力ch5へ渡す
- シースキミング中はSEARCH、CANDIDATE、EVADEの高度目標を20 mとし、上下蛇行を停止する
- 目標水平距離1000 m以下で高度400 mへポップアップし、高度400 m到達または目標3D距離500 m未満で突入する

詳細は `docs/CONTROL_DESIGN.md` を参照してください。

## ファイル構成

- `AGENTS.md` — Codexがこのフォルダで守る作業ルール
- `HGV Radar.lua` — レーダー探索・脅威追跡・フェーズ管理
- `HGV Guidance.lua` — 飛行誘導・上下蛇行・ビーム機動
- `ADSV3 Missile*.lua` — 流用元の比較用コピー
- `docs/PROJECT_PLAN.md` — 目的、範囲、開発段階、未確定事項
- `docs/CONTROL_DESIGN.md` — 採用した制御要求と状態遷移
- `docs/INTERFACES.md` — Lua入出力とプロパティの台帳
- `docs/TOOLING.md` — Lua・圧縮・DebugViewの運用方法
- `docs/TEST_PLAN.md` — HGV単体試験と防空ミサイル連携試験
- `docs/DECISIONS.md` — 採用した設計判断
- `tests/HGV Offline Test.lua` — オフライン状態遷移試験
- `experiments/` — Stormworks内の試験記録

## 次に必要な情報

1. HGV機体のPhysics Sensor、レーダー、フィン、推進器の実配線
2. 機体ローカル軸とYaw/Pitchフィンの正負
3. `VERTICAL_MANEUVER_AMPLITUDE_M` の初回試験値
4. PPN / PNゲイン、重力補償ゲイン、最大フィン指令
5. Compositeノード数から求めた実測ロジック遅延
6. 高度20 m飛行とポップアップ時に必要な実機Pitch余裕、海面クリアランス、信管動作の確認

未確認値を機体性能として扱わず、Stormworks試験で順に確定します。

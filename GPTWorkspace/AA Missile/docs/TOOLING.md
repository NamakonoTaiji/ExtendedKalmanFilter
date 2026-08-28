# 開発・検証環境

## 現在利用できるもの

親ワークスペース `GPTWorkspace` には、次の設定があります。

- `.vscode/settings.json`
  - Stormworksマイクロコントローラープロジェクトとして設定
  - Lua 5.3として解析
  - LifeBoatAPIのStormworksライブラリを参照
- `.vscode/launch.json`
  - Luaファイル実行用の起動構成
- `selene.toml`
  - Stormworks標準ライブラリを使うlint設定

VS Codeでは `GPTWorkspace` をワークスペースルートとして開くことで、これらの設定が `AA Missile` にも適用されます。

## 現在確認できていないもの

この環境のPATH上では、次の単体コマンドを確認できていません。

- `lua`
- `luac`
- `selene`

VS Code拡張機能に同梱されたLua 5.3ランタイムとLifeBoat圧縮器は利用でき、構文、純粋関数の数値試験、圧縮後文字数をオフライン確認できる。Lua Language Server単体の診断はStormworks標準ライブラリを読み込めないため、多数の既知の未定義グローバル警告を出す。Stormworks固有APIを含む統合動作と誘導性能は、引き続きStormworks内で確認する必要がある。

## 現在の圧縮後文字数

LifeBoat既定設定で確認した値：

| Luaブロック | 圧縮後文字数 | 8192文字以内 |
|---|---:|---|
| `ADSV3 MissileKF.lua` | 8185 | はい |
| `ADSV3 MissileFOVDebug.lua` | 7416 | はい |
| `ADSV3 Missile.lua` | 8142 | はい |

## Lua変更時の暫定チェックリスト

- 構文上の `end`、括弧、コメントブロックの対応
- 未定義変数と意図しないグローバル変数
- `input.get*` と `output.set*` のチャンネル対応
- property名とStormworks側設定の一致
- turnsとradの変換
- tickと秒の変換
- Physics Sensor世界座標とミサイルローカル座標の変換
- 変更した誘導分岐がpursuitかPNか
- `LOGIC_DELAY` の補償位置
- 8192文字制限を考慮した配備方法

## KF状態診断

`ADSV3 MissileFOVDebug.lua`は計測開始後、`[AA_FOV] KF_STATE`を毎tick記録する。前tickのKF位置・速度による1 tick予測からの位置残差、KF速度の1 tick変化に加え、自機位置差分のAAミサイル実速度と2階差分の実加速度を含む。それぞれのLOS直交成分に加え、推定閉速度、相対横速度、速度ベクトル交差角、LOS仰角も記録する。HGV側`[HGV_DBG] KINEMATICS`とは個別tickではなくDebugView++左端の時刻で照合する。

診断配線は、FOVロガーnum 13–15へKF出力1–3、num 16–18へKF出力4–6、num 32へKF出力12、bool 2へKF出力bool 1を分岐する。FOVロガーnum 1–12は生レーダー目標1–4に使用し、デバッグ専用outputは追加しない。

## 今後追加を検討するもの

- Lua構文・lintを再現可能に実行するコマンド
- Stormworks入出力を模擬するオフライン試験ハーネス
- 60 Hzのシナリオ再生とテレメトリCSV出力
- FOV余裕、ロストtick、最接近距離を自動集計する仕組み

ツールを追加する場合は、導入方法と実行コマンドをこの文書へ追記し、`AGENTS.md` の検証手順も更新します。

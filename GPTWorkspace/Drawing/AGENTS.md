# Stormworks Microcontroller project

このプロジェクトは Stormworks のマイクロコントローラを
storm-microcontroller-language (storm-mcl) で XML から DSL に変換して管理している。

## ファイル

- `project.json`: マイコンプロジェクト定義
- `main.sw-net`: 回路・ノード・配線の正本
- `main.sw-mcl`: Stormworksエディタ上のレイアウト
- `scripts/*.lua`: Luaノードのコード

## 編集時のルール

- XMLを直接編集しない。
- 回路変更は `.sw-net` を編集する。
- Lua変更は `scripts/*.lua` を編集する。
- Stormworks Luaは1ノード8192文字以内。
- Stormworksは通常60 tick/sで動作する。
- compositeのチャンネル番号を勝手に変更しない。
- 変更後は以下を実行して検証する。

  pnpm cli check-dsl <project.json>
  pnpm cli typecheck-dsl <project.json>

- XML生成は以下を使用する。

  pnpm cli dsl2xml <project.json> --out <output.xml>
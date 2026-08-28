[h1] データリンク対応対艦ミサイル / Anti-Ship Missile with Datalink [/h1]

無線から目標座標を送って中間誘導ができる対艦ミサイルマイコンです。
初期のポップアップ高度、巡航高度等をプロパティから設定可能。
簡易的なアンチチャフ機能を搭載してます。

This is an anti-ship missile microcontroller capable of mid-course guidance by receiving target coordinates via radio link.
Initial pop-up altitude, cruise altitude, and other parameters can be configured via properties.
Equipped with a simple anti-chaff feature.

[hr][/hr]

[h2] プロパティ一覧 / Property List [/h2]

[list]
[*] [b]SKKIMG_ALT[/b]
 巡航高度
 Cruise Altitude

[*] [b]PN_FIN_STRENGTH[/b]
 終末誘導に使われる比例航法のフィンの効きの強さ
 Fin strength for proportional navigation (PN) during terminal guidance

[*] [b]PPN_FIN_STRENGTH[/b]
 中間誘導や初期のポップアップ時のフィンの強さ
 Fin strength during mid-course guidance and initial pop-up phase

[*] [b]GUIDANCE_START_ALTITUDE[/b]
 発射時に高度何mまでポップアップするか
 Pop-up altitude (m) reached after launch before transitioning guidance

[*] [b]DATALINK_GATE_RADIUS[/b]
 目標座標から半径何メートル以内の目標を捕捉するか
 Acquisition radius (m) around the datalink target coordinates
[/list]
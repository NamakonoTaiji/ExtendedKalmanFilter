-- プレイヤーごとのデータ保持用テーブル
local player_data = {}

function onTick(game_ticks)
    local players = server.getPlayers()

    for _, player in pairs(players) do
        local peer_id = player.id

        if peer_id ~= nil then
            local pos_matrix, is_success = server.getPlayerPos(peer_id)

            if is_success then
                local x, y, z = matrix.position(pos_matrix)

                -- 初回データ初期化
                if not player_data[peer_id] then
                    player_data[peer_id] = {
                        x = x,
                        y = y,
                        z = z,
                        tick_counter = 0,
                        unit = "ms", -- デフォルトの単位 ("kmh" または "ms")
                        text = string.format("ALT: %.1fm\nSPD: ---", y)
                    }
                end

                local p = player_data[peer_id]

                -- tick数をカウントアップ
                p.tick_counter = p.tick_counter + game_ticks

                -- 60 tick（約1秒）経過した瞬間に計算と表示テキストを更新
                if p.tick_counter >= 60 then
                    -- 1秒前の位置からの移動距離 (m)
                    local dx = x - p.x
                    local dy = y - p.y
                    local dz = z - p.z
                    local distance = math.sqrt(dx * dx + dy * dy + dz * dz)

                    -- 実際に経過した時間（秒）
                    local elapsed_sec = p.tick_counter / 60

                    -- 速度計算 (m/s)
                    local speed_ms = distance / elapsed_sec

                    -- 単位に応じたテキストの生成 (1行13文字制限に対応)
                    if p.unit == "ms" then
                        p.text = string.format("ALT: %.1fm\nSPD: %.1fm/s", y, speed_ms)
                    else
                        local speed_kmh = speed_ms * 3.6
                        p.text = string.format("ALT: %.1fm\nSPD: %.1fkm/h", y, speed_kmh)
                    end

                    -- 次回の計算に向けて位置とカウンターを更新
                    p.x = x
                    p.y = y
                    p.z = z
                    p.tick_counter = p.tick_counter - 60
                end

                -- 保持しているテキストを表示
                local ui_id = peer_id + 10000
                server.setPopupScreen(peer_id, ui_id, "HUD_Speed_Alt", true, p.text, 0.8, 0.8)
            end
        end
    end
end

-- チャット欄にコマンドが入力された時の処理
function onCustomCommand(full_message, peer_id, is_admin, is_auth, command, ...)
    local args = { ... }

    -- "?unit" または "?speed" コマンドを受け付け
    if command == "?unit" or command == "?speed" then
        if player_data[peer_id] then
            local p = player_data[peer_id]

            -- 引数がある場合（例: "?unit ms" や "?unit kmh"）
            if args[1] == "ms" then
                p.unit = "ms"
            elseif args[1] == "kmh" then
                p.unit = "kmh"
            else
                -- 引数がない場合はトグル切り替え (kmh <-> ms)
                if p.unit == "kmh" then
                    p.unit = "ms"
                else
                    p.unit = "kmh"
                end
            end

            -- 即座に切り替え通知を画面右下にポップアップ
            server.notify(peer_id, "HUD Unit Changed", "Display unit: " .. string.upper(p.unit), 7)
        end
    end
end

function onPlayerLeave(steam_id, name, peer_id, is_admin, is_auth)
    if peer_id ~= nil then
        player_data[peer_id] = nil
        server.removePopup(peer_id, peer_id + 10000)
    end
end
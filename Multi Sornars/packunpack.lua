--============================================================
-- Target Pack / Unpack
--
-- 2 Number channels = 48bit
--
-- packed1:
--   bit  0-11 : distance      12bit
--   bit 12-22 : age           11bit
--   bit 23    : azimuth MSB    1bit
--
-- packed2:
--   bit  0-11 : azimuth LSB   12bit
--   bit 12-23 : elevation     12bit
--
-- 精度:
-- distance  : 2m刻み       最大誤差 ±1m
-- azimuth   : 13bit        最大誤差 約±0.022度
-- elevation : 12bit        最大誤差 約±0.022度
-- age       : 1tick刻み    0～2047tick
--============================================================


--============================================================
-- PACK
--
-- distance          : 距離 [m]
-- azimuth           : 方位角 [rad] (-pi ～ +pi)
-- elevation         : 仰角 [rad]   (-pi/2 ～ +pi/2)
-- targetReachedTick : 目標位置に対応するtick
-- currentTick       : パックを行う現在tick
--
-- return:
-- packed1, packed2
--============================================================
function packTarget(
    distance,
    azimuth,
    elevation,
    targetReachedTick,
    currentTick
)
    local pi = math.pi
    local pi2 = pi * 2

    ----------------------------------------------------------
    -- 距離
    -- 12bit / 2m刻み
    -- 0 ～ 8190m
    ----------------------------------------------------------
    local d = math.floor(distance / 2 + 0.5)

    if d < 0 then
        d = 0
    elseif d > 4095 then
        d = 4095
    end


    ----------------------------------------------------------
    -- 観測の古さ
    -- 11bit / 1tick刻み
    -- 0 ～ 2047tick
    ----------------------------------------------------------
    local age = math.floor(
        currentTick - targetReachedTick + 0.5
    )

    if age < 0 then
        age = 0
    elseif age > 2047 then
        age = 2047
    end


    ----------------------------------------------------------
    -- 方位角
    -- 13bit
    -- -pi ～ +pi を 0 ～ 8191 に変換
    ----------------------------------------------------------
    local a = math.floor(
        ((azimuth + pi) % pi2)
        / pi2
        * 8192
        + 0.5
    ) % 8192


    ----------------------------------------------------------
    -- 仰角
    -- 12bit
    -- -pi/2 ～ +pi/2 を 0 ～ 4095 に変換
    ----------------------------------------------------------
    if elevation < -pi / 2 then
        elevation = -pi / 2
    elseif elevation > pi / 2 then
        elevation = pi / 2
    end

    local e = math.floor(
        (elevation + pi / 2)
        / pi
        * 4095
        + 0.5
    )


    ----------------------------------------------------------
    -- 方位角13bitを
    -- 上位1bit + 下位12bit に分割
    ----------------------------------------------------------
    local aLow = a % 4096
    local aHigh = math.floor(a / 4096)


    ----------------------------------------------------------
    -- 24bit整数 × 2ch
    ----------------------------------------------------------
    local packed1 =
        d
        + age * 4096
        + aHigh * 8388608

    local packed2 =
        aLow
        + e * 4096


    return packed1, packed2
end



--============================================================
-- UNPACK
--
-- packed1, packed2 : packTarget()から受信した値
-- currentTick      : 受信側の現在tick
-- transportDelay   : 送信Lua→受信Lua間の固定遅延 [tick]
--                    不要なら0または省略
--
-- return:
-- distance
-- azimuth
-- elevation
-- targetReachedTick
--
-- データなし(packed1=0, packed2=0)の場合 nil
--============================================================
function unpackTarget(
    packed1,
    packed2,
    currentTick,
    transportDelay
)
    local pi = math.pi
    local pi2 = pi * 2

    transportDelay = transportDelay or 0


    ----------------------------------------------------------
    -- データなし
    ----------------------------------------------------------
    if packed1 == 0 and packed2 == 0 then
        return nil
    end


    ----------------------------------------------------------
    -- Number伝送後の値を整数へ
    ----------------------------------------------------------
    packed1 = math.floor(packed1 + 0.5)
    packed2 = math.floor(packed2 + 0.5)


    ----------------------------------------------------------
    -- packed1 抽出
    ----------------------------------------------------------

    -- distance 12bit
    local d =
        packed1 % 4096

    -- age 11bit
    local age =
        math.floor(packed1 / 4096) % 2048

    -- azimuth 上位1bit
    local aHigh =
        math.floor(packed1 / 8388608) % 2


    ----------------------------------------------------------
    -- packed2 抽出
    ----------------------------------------------------------

    -- azimuth 下位12bit
    local aLow =
        packed2 % 4096

    -- elevation 12bit
    local e =
        math.floor(packed2 / 4096) % 4096


    ----------------------------------------------------------
    -- 方位角13bitを結合
    ----------------------------------------------------------
    local a =
        aHigh * 4096 + aLow


    ----------------------------------------------------------
    -- 実値へ復元
    ----------------------------------------------------------

    -- 距離
    local distance =
        d * 2

    -- 方位角
    local azimuth =
        a / 8192
        * pi2
        - pi

    -- 仰角
    local elevation =
        e / 4095
        * pi
        - pi / 2


    ----------------------------------------------------------
    -- targetReachedTick復元
    --
    -- ageは「送信時点で何tick前だったか」なので、
    -- 通信経路の遅延も追加で引く
    ----------------------------------------------------------
    local targetReachedTick =
        currentTick
        - age
        - transportDelay


    return
        distance,
        azimuth,
        elevation,
        targetReachedTick
end
-- LifeBoatAPI既定設定でHGVの配備用文字数を再現可能に計測する。
-- arg[1]: LifeBoatAPI assets/lua/Common
-- arg[2]: LifeBoatAPI assets/lua/MicroController/microcontroller.lua
-- arg[3]: LifeBoatAPI assets/lua/Addon/addon.lua
-- arg[4]: HGV Missileワークスペース絶対パス

package.path = arg[1] .. "/?.lua;" .. arg[1] .. "/?/init.lua;" .. package.path
require("LifeBoatAPI.Tools.Build.Builder")

local Filepath = LifeBoatAPI.Tools.Filepath
local workspace = Filepath:new(arg[4])
local output = Filepath:new(arg[4] .. "/_build/lifeboat", true)
local builder = LifeBoatAPI.Tools.Builder:new(
    { workspace },
    output,
    Filepath:new(arg[2]),
    Filepath:new(arg[3])
)

local params = {
    boilerPlate = "",
    reduceAllWhitespace = true,
    reduceNewlines = true,
    removeRedundancies = true,
    shortenVariables = true,
    shortenGlobals = true,
    shortenNumbers = true,
    forceNCBoilerplate = false,
    forceBoilerplate = false,
    shortenStringDuplicates = true,
    removeComments = true,
    skipCombinedFileOutput = false
}

for _, name in ipairs({ "HGV Radar.lua", "HGV Guidance.lua" }) do
    builder:buildMicrocontroller(
        name,
        Filepath:new(arg[4] .. "/" .. name),
        params
    )
end

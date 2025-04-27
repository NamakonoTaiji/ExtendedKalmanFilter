--[[
            currentLocalAzimuth = math.atan(targetLocal[1], targetLocal[2])
            horizontalDistance = math.sqrt(targetLocal[1] ^ 2 + targetLocal[2] ^ 2)
            currentLocalElevation = math.atan(targetLocal[3], horizontalDistance)
]]

targetLocal = { -85.802806090776, -27.629772809181, 1165.73640326 }
currentLocalAzimuth = math.atan(targetLocal[1], targetLocal[2])
horizontalDistance = math.sqrt(targetLocal[1] ^ 2 + targetLocal[2] ^ 2)
currentLocalElevation = math.atan(targetLocal[3], horizontalDistance)

print("currentLocalAzimuth: ", currentLocalAzimuth)
print("horizontalDistance: ", horizontalDistance)
print("currentLocalElevation: ", currentLocalElevation)

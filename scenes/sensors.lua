-- Get wheel angular position (in degrees)
function getWheelAngularPosition (handle)
    angle=sim.getJointPosition(handle)
    angPos = angle*180.0/math.pi
    angPos = -angPos - 180.0
    return angPos
end 

-- Compute increment of odemeters
function deltaWheelAngularPosition(curPos,lastPos,wSpeed)
    if wSpeed == 0.0 then
        deltaPos = 0.0
    else
        deltaPos = -(curPos - lastPos)  -- clock wise rotation (anti trigo)
    end
    if deltaPos < -180.0 then
        deltaPos = deltaPos + 360.0
    end
    if deltaPos > 180.0 then
        deltaPos = deltaPos - 360.0
    end
    return deltaPos
end

-- Take speed command and convert into V-REP motor command
--   tries to simulate left-right diffrences and starting current
function defineLeftSpeed (cmd)
    cmd1=cmd
    if cmd1 < 0 then
        cmd1 = -cmd1
    end
    if cmd1 < cmdDeadZone then
        cmd1 = 0
    end
    if cmd1 > 100 then 
        cmd1 = 100
    end
    vrepCmd = cmd1*vAngMax/100.0
    if cmd < 0 then
        vrepCmd = -vrepCmd
    end
    return vrepCmd
end

function defineRightSpeed (cmd)
    cmd1=cmd
    if cmd1 < 0 then
        cmd1 = -cmd1
    end
    if cmd1 < cmdDeadZone then
        cmd1 = 0
    end
    if cmd1 > 100 then 
        cmd1 = 100
    end
    vrepCmd = cmd1*vAngMax/100.0
    if cmd < 0 then
        vrepCmd = -vrepCmd
    end
    return vrepCmd
end

-- add noise to distance (bias, spike and gaussian)
function realDistanceNoise (dist,sonar)
   local rDist = dist+fidelSonar*gaussian()+justSonar
   --if math.random() < probaSpike then
   --   --rDist = rDist*1000.0
   --   rDist = math.random()*100.0
   --end
   if periodSpike ~= -1 then 
      cntSpike = cntSpike - 1
      if cntSpike <= 0 then
         cntSpike = math.floor(0.5*(periodSpike+periodSpike*math.random()))
         rDist = math.random()*10.0
         printToConsole ("cntSpike",cntSpike,rDist)
         --rDist = 25.0
      end
   end
   --print ("dist="..dist..", rDist="..rDist)
   rDist = roundDecimal(rDist,2)
   return rDist
end

function realDistance(dist,sonar)
   dist = roundDecimal(dist,2)
   --printToConsole ("dist="..dist..", sonar="..sonar)
   return dist
end


function getMeasuredHeading()
   local orient = sim.getObjectOrientation(car,-1)
   local heading = 90.0 - (orient[3]*180.0/math.pi)
   -- heading = heading+fidelCompass*gaussian()+justCompass
   heading = roundDecimal(heading,1)
   if heading < 0.0 then heading = heading+360.0 end
   if heading > 360.0 then heading = heading-360.0 end
   if heading == 360.0 then heading = 0.0 end
   return heading
end

require ("car_utils")
require ("sensors")
require ("simulation_loop")


clock = os.clock
sim.setThreadSwitchTiming(10) -- We wanna manually switch for synchronization purpose (and also not to waste processing time!)

frontRightMotor = sim.getObjectHandle("JointWheelRight")
frontLeftMotor = sim.getObjectHandle("JointWheelLeft")
sonarFront = sim.getObjectHandle("SonarFrontSensor")
sonarLeft = sim.getObjectHandle("SonarLeftSensor")
sonarRight = sim.getObjectHandle("SonarRightSensor")
sonarBack = sim.getObjectHandle("SonarBackSensor")
car = sim.getObjectHandle("RobotCenterMarker")


-- Socket Port number
portNb = 30100
serverOn = false
connexionTimeout = 0.01
cntTimeout = 0
socket=require("socket")
srv = nil
clt1 = nil

if periodSpike ~= -1 then
   cntSpike = math.floor(0.5*(periodSpike+periodSpike*math.random()))
else
   cntSpike = 1000000000
end
printToConsole ("cntSpike init",cntSpike)
cntSonar = 0
-- one new sonar measurment every measureSonar*4*0.05 seconds (1 -> 0.2 s)
measureSonar = 1 
numSonar = 0
distFront,distLeft,distRight,distBack = 0.0,0.0,0.0,0.0


nTicks = nTicksPerRevol -- get nticks per revol from setup
print ("nTicks = "..nTicks)
speedRight, speedLeft = 0.0,0.0
lastSpeedRight, lastSpeedLeft = 0.0,0.0
if setBatteryLevel == -1.0 then
   local mySeed = os.time()
   math.randomseed(mySeed)
   batteryLevel = 0.7+0.3*math.random()
else
   batteryLevel = setBatteryLevel
end
batteryLevel = math.floor(batteryLevel*100+0.5)/100.0
--batteryLevel = 1.0
printToConsole ("battery level : "..batteryLevel)
vAngMax = 10.0*batteryLevel

cmdDeadZone = 0
if not run_headless then
   sim.addStatusbarMessage ("battery coef = "..(batteryLevel*100).."% , vAngMax = "..vAngMax)
end
-- get orientation of Dart's body
lastRelativeOrientation={}
lastOrientationTime={}
wheelCnt={}
wheelId={}
wheels={"JointWheelLeft","JointWheelRight"}
for i=1,#wheels do
    handle=sim.getObjectHandle(wheels[i])
    wheelId[#wheelId+1]=handle
    lastRelativeOrientation[#lastRelativeOrientation+1]=getWheelAngularPosition(handle)
    lastOrientationTime[#lastOrientationTime+1]=sim.getSimulationTime()
    wheelCnt[#wheelCnt+1]=0.0
end
printToConsole ("run ...",scene,"robscene ...")


cntMotion = 0
cntNoMotion = 100  -- no motion for 5 s (100 x 50 ms)
motion = false


timeChallengeStart = gettime()

printToConsole ("end init ...")
printToConsole ("start thread ...")
-- Execute the thread function:
res=false
err=" not launched delibarately "
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
   if not run_headless then
      sim.addStatusbarMessage('Lua runtime error: '..err)
   end
end
--simFloatingViewRemove(camView)
printToConsole ("end of simulation !!!")

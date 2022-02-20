-- main simulation loop function (run every 50 ms)
function threadFunction()
   printToConsole ("sim state",sim.getSimulationState())
   printToConsole ("sim time",sim.getSimulationTime())

   while (sim.getSimulationState()~=sim.simulation_advancing_abouttostop) do
      local t0 = gettime()
      local simTime = sim.getSimulationTime()
      timeStamp = simTime..";"

      -- update or init (reinit) motion
      if motion then
         if math.abs(speedLeft) > 0 or math.abs(speedRight) > 0 then
            cntMotion = cntNoMotion
         else
            cntMotion = cntMotion -1 
            if cntMotion <= 0 then
               motion = false
            end
         end
      else
         -- start or restart
         if math.abs(speedLeft) > 0 or math.abs(speedRight) > 0 then
            motion = true
            cntMotion = cntNoMotion
         end
     end

      local t1 = gettime()
      local t2 = gettime()

      if cntSonar == 0 then
         --printToConsole ("measure sonar",(numSonar+1),'log',logOn,'logFile',logFile)
	
         if numSonar == 0 then
            local result,dist,dtPoint,dtObjHandle,dtSurfNorm = sim.handleProximitySensor(sonarFront)
            distFront = 0.0
            if result == 1 then distFront = realDistance(dist,"front") end
         end
         if numSonar == 1 then
            local result,dist,dtPoint,dtObjHandle,dtSurfNorm = sim.handleProximitySensor(sonarLeft)
            distLeft = 0.0
            if result == 1 then distLeft = realDistance(dist,"left") end
         end
         if numSonar == 2 then
            local result,dist,dtPoint,dtObjHandle,dtSurfNorm = sim.handleProximitySensor(sonarRight)
            distRight = 0.0
            if result == 1 then distRight = realDistance(dist,"right") end
         end
         if numSonar == 3 then
            local result,dist,dtPoint,dtObjHandle,dtSurfNorm = sim.handleProximitySensor(sonarBack)
            distBack = 0.0
            if result == 1 then distBack = realDistance(dist,"back") end
   
         end
         numSonar = numSonar + 1
         if numSonar == 4 then numSonar=0 end
      end
      cntSonar = cntSonar+1
      if cntSonar == measureSonar then cntSonar = 0 end

      local t3 = gettime()

      for iw=1,#wheelId do
         handle=wheelId[iw]
         relativeOrientation=getWheelAngularPosition(handle)
         currentOrientationTime=sim.getSimulationTime()
         if iw == 1  then wheelSpeed = speedLeft end -- left wheel
         if iw == 2  then wheelSpeed = speedRight end -- right wheel
         analogCnt = deltaWheelAngularPosition(relativeOrientation,lastRelativeOrientation[iw],wheelSpeed)
         wheelCnt[iw] = wheelCnt[iw] + (analogCnt*nTicks/360.0)
         lastRelativeOrientation[iw]=relativeOrientation
         lastOrientationTime[iw]=currentOrientationTime   
      end
      leftEncoder = math.floor(wheelCnt[1])
      rightEncoder = math.floor(wheelCnt[2])
      if leftEncoder < 0 then leftEncoder = leftEncoder + 1 end
      if rightEncoder < 0 then rightEncoder = rightEncoder + 1 end
      heading = getMeasuredHeading()

      local dataOut={distFront,distLeft,distRight,distBack,
                     distFrontLeft,distFrontRight,
                     leftEncoder,rightEncoder,heading}

      local t4 = gettime()
      local t5 = gettime()


      if not serverOn then
         printToConsole ("not connected")
	      printToConsole ("sim time",sim.getSimulationTime())
         srv = assert(socket.bind('127.0.0.1',portNb))
         if (srv==nil) then
            printToConsole ("bad connect")
         else
	    printToConsole ("get socket")
	    printToConsole ("sim time",sim.getSimulationTime())
            ip, port = srv:getsockname()
            printToConsole ("server ok at "..ip.." on port "..port)
	    printToConsole ("sim time",sim.getSimulationTime())
             serverOn = true
            --srv:settimeout(connexionTimeout)
            printToConsole ("connexion granted !!! ")
         end
      end
      --printToConsole (serverOn)
      if serverOn then
         srv:settimeout(connexionTimeout)
         clt1 = srv:accept()
         if clt1 == nil then
            cntTimeout = cntTimeout + 1
         else
            clt1:settimeout(connexionTimeout)
            dataIn = readSocketData(clt1)
            if dataIn ~= nil then
               targetCmd=sim.unpackFloatTable(dataIn)            
               speedLeft = defineLeftSpeed(targetCmd[2]) 
               speedRight = defineRightSpeed(targetCmd[3])
               dataPacked=sim.packFloatTable(dataOut)
               writeSocketData(clt1,dataPacked)
               clt1:send(dataIn)
            else
               printToConsole ("no data")
            end
            clt1:close()
         end
      end

      -- modify speed only when it has changed
      if lastSpeedRight ~= speedRight then
         sim.setJointTargetVelocity(frontRightMotor,speedRight)
         lastSpeedRight = speedRight
      end
      if lastSpeedLeft ~= speedLeft then
         sim.setJointTargetVelocity(frontLeftMotor,speedLeft)
         lastSpeedLeft = speedLeft
      end

      if cntMark == 0 then
         motionStatus="Off "
         if motion then
            motionStatus="On  "
         end

         if not run_headless then
            sim.addStatusbarMessage (msgMark)
         end
      end


      sim.switchThread() -- This thread will resume just before the main script is called again

   end

end

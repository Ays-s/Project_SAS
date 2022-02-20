-- Radian / degree convertion
radDeg = 180.0/math.pi
degRad = math.pi/180.0


socket=require("socket")
-- Following function writes data to the socket (only single packet data for simplicity sake):
function writeSocketData(client,data)
    local header=string.char(59,57,math.mod(#data,256),math.floor(#data/256),0,0)
    -- Packet header is (in this case): headerID (59,57), dataSize (WORD), packetsLeft (WORD) but not used here
    client:send(header..data)
end

-- Following function reads data from the socket (only single packet data for simplicity sake):
function readSocketData(client)
    -- Packet header is: headerID (59,57), dataSize (WORD), packetsLeft (WORD) but not used here
    local header=client:receive(6)
    if (header==nil) then
        return(nil) -- error
    end
    if (header:byte(1)==59)and(header:byte(2)==57) then
        local l=header:byte(3)+header:byte(4)*256
        local v=client:receive(l)
        return(v)
    else
        return(nil) -- error
    end
end

-- Use sleeping function of socket library
function sleep(sec)
    socket.select(nil,nil,sec)
end 

-- Get cuurent time (in sec) 
function gettime()
   return socket.gettime()
end

-- Round with to given number of decimal places
function roundDecimal(num, numDecimalPlaces)
   local mult = 10^(numDecimalPlaces or 0)
   return math.floor(num * mult + 0.5) / mult
end

function roundDecimal2(num, numDecimalPlaces)
   return tonumber(string.format("%." .. (numDecimalPlaces or 0) .. "f", num))
end

-- Simple round
function round(x)
  return x>=0 and math.floor(x+0.5) or math.ceil(x-0.5)
end

-- int() function
function signedFloorCeil(x)
  return x>=0 and math.floor(x) or math.ceil(x)
end

-- get random value using gaussian distribution
function gaussian()
   w = 1.1
   while w >= 1.0 do
      x1 = 2.0*math.random()-1.0
      x2 = 2.0*math.random()-1.0
      w = x1 * x1 + x2 * x2
   end
   w = math.sqrt( (-2.0 * math.log( w ) ) / w )
   y1 = x1 * w
   --y2 = x2 * w
   return y1
end


-- define start time of challenge
function resetElapsedTime()
   --timeChallengeStart = clock()
   timeChallengeStart = gettime()
end

function tableConcat(t1,t2)
   n1 = table.getn(t1)
   n2 = table.getn(t2)
   for i=1,n2 do
      t1[n1+i] = t2[i]
   end
   return t1
end


function printTable(t,nm)
   local n = table.getn(t)
   for i=1,n do
      printToConsole(nm,i,t[i])
   end
end

function uniformRandom()
   rnd = math.random()
   irnd = irnd+1
   printToConsole("rnd",irnd,rnd)
   return rnd
end

-- measure elapsed time since start time of the challenge
function getElapsedTime()
   --local tElaps = clock() - timeChallengeStart
   local tElaps = gettime() - timeChallengeStart
   local nMin = math.floor(tElaps/60.0)
   local nSec = math.floor(tElaps-nMin*60)
   local stElaps = "Time="..tostring(nMin)..":"
   if nSec < 10 then
      stElaps = stElaps.."0"
   end
   stElaps = stElaps..tostring(nSec)
   return stElaps
end

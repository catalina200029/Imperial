-- Generate a sample from a Gaussian distribution
function gaussian (mean, variance)
    val = math.sqrt(-2 * variance * math.log(math.random())) * math.cos(2 * math.pi * math.random()) + mean
    if (val ~= val) then 
        return 0
    end
    return val
end

function realGaussian(z, mean, variance)
    return math.exp(-((z - mean) ^ 2) / (2 * variance))
end

function createRandomBumpyFloor()
    print ("Generating new random bumpy floor.")
    sim.setThreadAutomaticSwitch(false)

    -- Remove existing bumpy floor if there already is one
    if (heightField ~= nil) then
        sim.setObjectPosition(heightField, heightField, {0.05, 0, 0})
        return
    end
    --  Create random bumpy floor for robot to drive on
    floorSize = 5
    --heightFieldResolution = 0.3
    --heightFieldNoise = 0.00000005weightArray
    heightFieldResolution = 0.1
    heightFieldNoise = 0.0000008
    cellsPerSide = floorSize / heightFieldResolution
    cellHeights = {}
    for i=1,cellsPerSide*cellsPerSide,1 do
        table.insert(cellHeights, gaussian(0, heightFieldNoise))
    end
    heightField=sim.createHeightfieldShape(0, 0, cellsPerSide, cellsPerSide, floorSize, cellHeights)
    -- Make the floor invisible
    sim.setObjectInt32Param(heightField,10,0)
    sim.setThreadAutomaticSwitch(true)
end


function calculateLikelihood(x, y, theta, z, idx)
    -- Find out which wall the sonar beam would hit if the robot is at position x, y, theta
    -- Calculate expected measurement
    closestWall = 0
    closestDistance = -1
    
    for i = 1, N_WALLS do
        wall = walls[i]
        m = expectedDepthCalculation(x, y, theta, wall)
        if (m >= 0 and inRange(x, y, theta, m, wall) and (closestDistance == -1 or m < closestDistance)) then
            closestDistance = m
            closestWall = i
        end
    end
    
    --print("The sensor measurement (z) is ", z)
    --print("Index and closest distance to a wall ", idx, closestDistance)
    --print("x, y, theta, wall", x, y, theta, walls[closestWall])
    --print("The z - closest distance is ", z - closestDistance)
    --return gaussian(closestDistance, 0.01)
    return realGaussian(z, closestDistance, 0.01)
end

function expectedDepthCalculation(x, y, theta, wall)
    diff_y = wall[4] - wall[2]
    diff_x = wall[3] - wall[1]
    
    divisor = (diff_y * math.cos(theta)) - (diff_x * math.sin(theta))
    
    if (divisor == 0) then
        return -1
    else
        return ((diff_y * (wall[1] - x)) - (diff_x * (wall[2] - y))) / divisor
    end
end

function lessThan(a, b)
    error = 0.001
    return a <= b or a > b and a - b <= error
end

function inRange(x, y, theta, m, wall) 
    xPosition = x + (m * math.cos(theta))
    yPosition = y + (m * math.sin(theta))
    
    inRangeX = false
    inRangeY = false
    
    if (wall[1] < wall[3]) then
        --inRangeX = wall[1] <= xPosition and xPosition <= wall[3]
        inRangeX = lessThan(wall[1], xPosition) and lessThan(xPosition, wall[3])
    else
        --inRangeX = wall[3] <= xPosition and xPosition <= wall[1]
        inRangeX = lessThan(wall[3], xPosition) and lessThan(xPosition, wall[1])
    end
    
    if (wall[2] < wall[4]) then
        --inRangeY = wall[2] <= yPosition and yPosition <= wall[4]
        inRangeY = lessThan(wall[2], yPosition) and lessThan(yPosition, wall[4])
    else
        --inRangeY = wall[4] <= yPosition and yPosition <= wall[2]
        inRangeY = lessThan(wall[4], yPosition) and lessThan(yPosition, wall[2])
    end
    
    return inRangeX and inRangeY
end

function get_walls()
    -- Disable error reporting
    local savedState=sim.getInt32Param(sim.intparam_error_report_mode)
    sim.setInt32Param(sim.intparam_error_report_mode,0)
    local N = 1
    while true do
        local handle = sim.getObjectHandle("Wall"..tostring(N))
        if handle <= 0 then
            break
        end

        -- Read position and shape of wall
        -- Assume here that it is thin and oriented either along the x axis or y axis

        -- We can now get the propertries of these walls, e.g....
        local pos = sim.getObjectPosition(handle, -1)
        local res,minx = sim.getObjectFloatParameter(handle,15)
        local res,maxx = sim.getObjectFloatParameter(handle,18)
        local res,miny = sim.getObjectFloatParameter(handle,16)
        local res,maxy = sim.getObjectFloatParameter(handle,19)
    
        --print("Position of Wall " .. tostring(N) .. ": " .. tostring(pos[1]) .. "," .. tostring(pos[2]) .. "," .. tostring(pos[3]))
        --print("minmax", minx, maxx, miny, maxy)
 
        local Ax, Ay, Bx, By
        if (maxx - minx > maxy - miny) then
            --print("Wall " ..tostring(N).. " along x axis")
            Ax = pos[1] + minx
            Ay = pos[2]
            Bx = pos[1] + maxx
            By = pos[2]
        else
            --print("Wall " ..tostring(N).. " along y axis")
            Ax = pos[1]
            Ay = pos[2] + miny
            Bx = pos[1]
            By = pos[2] + maxy
        end
        --print (Ax, Ay, Bx, By)

        walls[N] = {Ax, Ay, Bx, By}
        N = N + 1
    end
    -- enable error reporting
    sim.setInt32Param(sim.intparam_error_report_mode,savedState)

    return N - 1
end

-- This function performs resampling- the fourth step of the MCL algorithm
function resampling()
    cumulative_prob_distr = {}
    cumulative_prob_distr[1] = weightArray[1]
    
    for i=2, N do
        cumulative_prob_distr[i] = cumulative_prob_distr[i-1] + weightArray[i]
        --print("Index and cumulation: ", i, cumulative_prob_distr[i])
    end
    
    --for i=1, N do
    --    print("Weight, Cum", weightArray[i], cumulative_prob_distr[i])
    --end
    
    xArrayNew = {}
    yArrayNew = {}
    thetaArrayNew = {}
    
    for i=1, N do
        r = math.random()
        for j=1, N do
            if r <= cumulative_prob_distr[j] or j == N then
                --print("New index and random number: ", i, r)
                --print("New index and particle picked: ", i, j)
                --print("New index, old index: ", i, j)
                xArrayNew[i] = xArray[j]
                yArrayNew[i] = yArray[j]
                thetaArrayNew[i] = thetaArray[j]
                weightArray[i] = 1 / N
                sim.setObjectPosition(dummyArray[i], -1, {xArrayNew[i], yArrayNew[i], 0})
                sim.setObjectOrientation(dummyArray[i], -1, {0, 0, thetaArrayNew[i]})
                break
            end
        end
    end
   
    xArray = xArrayNew
    yArray = yArrayNew
    thetaArray = thetaArrayNew
end

-- This function is executed exactly once when the scene is initialised
function sysCall_init()

    tt = sim.getSimulationTime()
    print("Init hello", tt)
          
    robotBase=sim.getObjectHandle(sim.handle_self) -- robot handle
    leftMotor=sim.getObjectHandle("leftMotor") -- Handle of the left motor
    rightMotor=sim.getObjectHandle("rightMotor") -- Handle of the right motor
    turretMotor=sim.getObjectHandle("turretMotor") -- Handle of the turret motor
    turretSensor=sim.getObjectHandle("turretSensor")
     
    stepCounter = 0
    waypointCounter = 1
    
    -- Usual rotation rate for wheels (radians per second)
    speedBase = 5
    speedBaseL = 0
    speedBaseR = 0
    stepCompletedFlag = false

    -- Create bumpy floor for robot to drive on
    createRandomBumpyFloor()

    stepList = {}
    stepList[1] = {"set_waypoint"}
    stepList[2] = {"turn"}
    stepList[3] = {"stop"}
    stepList[4] = {"readjust"}
    stepList[5] = {"forward"}
    stepList[6] = {"stop"}
    stepList[7] = {"readjust"}

    N_WAYPOINTS = 26
    currentWaypoint = 0
    waypoints = {}
    waypoints[1] = {0.5,0}
    waypoints[2] = {1,0}
    waypoints[3] = {1,0.5}
    waypoints[4] = {1,1}
    waypoints[5] = {1,1.5}
    waypoints[6] = {1,2}
    waypoints[7] = {0.5,2}
    waypoints[8] = {0,2}
    waypoints[9] = {-0.5,2}
    waypoints[10] = {-1,2}
    waypoints[11] = {-1,1.5}
    waypoints[12] = {-1,1}
    waypoints[13] = {-1.5,1}
    waypoints[14] = {-2,1}
    waypoints[15] = {-2,0.5}
    waypoints[16] = {-2,0}
    waypoints[17] = {-2,-0.5}
    waypoints[18] = {-1.5,-1}
    waypoints[19] = {-1,-1.5}
    waypoints[20] = {-0.5,-1.5}
    waypoints[21] = {0,-1.5}
    waypoints[22] = {0.5,-1.5}
    waypoints[23] = {1,-1.5} 
    waypoints[24] = {1,-1}
    waypoints[25] = {0.5,-0.5}
    waypoints[26] = {0,0}

    -- Create and initialise arrays for particles, and display them with dummies
    xArray = {}
    yArray = {}
    thetaArray = {}
    weightArray = {}
    dummyArray = {}
    N = 100
    for i=1, N do
        xArray[i] = 0
        yArray[i] = 0
        thetaArray[i] = 0
        weightArray[i] = 1/N
        dummyArray[i] = sim.createDummy(0.05)
        sim.setObjectPosition(dummyArray[i], -1, {0,0,0})
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,0})
    end
    
    target_x = 0
    target_y = 0
    
    angle_to_turn = 0
    last_command = ""
    distance_to_move = 0

    -- Data structure for walls
    walls = {}
    -- Fill it by parsing the scene in the GUI
    N_WALLS = get_walls()
    -- walls now is an array of arrays with the {Ax, Ay, Bx, By} wall coordinates
  
    sensorStandardDeviation = 0.1
    sensorVariance = sensorStandardDeviation^2
    noisyDistance = 0
    
    motorAnglePerMetre = 24.8
    motorAnglePerRadian = 3.05
    motorAngleTargetL = 0
    motorAngleTargetR = 0
end

function sysCall_sensing()
    
end

function getMaxMotorAngleFromTarget(posL, posR)

    -- How far are the left and right motors from their targets? Find the maximum
    maxAngle = 0
    if (speedBaseL > 0) then
        remaining = motorAngleTargetL - posL
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseL < 0) then
        remaining = posL - motorAngleTargetL
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseR > 0) then
        remaining = motorAngleTargetR - posR
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseR < 0) then
        remaining = posR - motorAngleTargetR
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end

    return maxAngle
end

function sysCall_actuation() 
    tt = sim.getSimulationTime()
    -- print("actuation hello", tt)
    
    -- Get and plot current angles of motor joints
    posL = sim.getJointPosition(leftMotor)
    posR = sim.getJointPosition(rightMotor)
    
    -- Start new step?
    if (stepCompletedFlag == true or stepCounter == 0) then
        stepCounter = stepCounter + 1
        stepCompletedFlag = false

        newStepType = stepList[stepCounter][1]
        
        waypoint = waypoints[waypointCounter]
        
        if (newStepType == "set_waypoint") then
            --print("A new waypoint is being set!")
            target_x = waypoint[1]
            target_y = waypoint[2]
        elseif (newStepType == "forward") then
            -- Forward step: set new joint targets
            --print("Preparing to move forward")
            last_command = "forward"
            
            x_sum = 0
            for i=1, N do
                x_sum = x_sum + (weightArray[i] * xArray[i])
            end
            curr_x = x_sum
            
            y_sum = 0
            for i=1, N do
                y_sum = y_sum + (weightArray[i] * yArray[i])
            end
            curr_y = y_sum
            
            newStepAmount = math.sqrt((target_x - curr_x)^2 + (target_y - curr_y)^2)

            --print("Newstepamount", newStepAmount)
            --print(curr_x, curr_y)

            distance_to_move = newStepAmount

            motorAngleTargetL = posL + (newStepAmount * motorAnglePerMetre)
            motorAngleTargetR = posR + (newStepAmount * motorAnglePerMetre)
        elseif (newStepType == "turn") then
            -- Turn step: set new targets
            --print("Preparing to turn")

            last_command = "turn"
            theta_sum = 0
            for i=1, table.getn(thetaArray) do
                theta_sum = theta_sum + (weightArray[i] * thetaArray[i])
            end
            curr_theta = theta_sum
            
            x_sum = 0
            for i=1, table.getn(xArray) do
                x_sum = x_sum + (weightArray[i] * xArray[i])
            end
            curr_x = x_sum
            
            y_sum = 0   
            for i=1, table.getn(yArray) do
                y_sum = y_sum + (weightArray[i] * yArray[i])
            end
            curr_y = y_sum
            
            dy = target_y - curr_y
            dx = target_x - curr_x
            alpha = math.atan2(dy, dx)
            
            -- In case alpha is NAN
            if (alpha ~= alpha) then
                if (dy < 0) then
                    alpha = math.rad(-90)
                else 
                    alpha = math.rad(90)
                end
            end

            beta = alpha - curr_theta
            
            --print("")
            --print("beta ", beta)
            --print("alpha ", alpha)
            --print("curr_theta ", curr_theta)
            
            while (beta > math.rad(180)) do
                beta = beta - math.rad(360)
            end
            
            while (beta < (-1 * math.rad(180))) do
                beta = beta + math.rad(360)
            end
            
            angle_to_turn = beta
            newStepAmount = beta
            
            motorAngleTargetL = posL - (angle_to_turn * motorAnglePerRadian)
            motorAngleTargetR = posR + (angle_to_turn * motorAnglePerRadian)
        elseif (newStepType == "stop") then
            --print("Preparing to stop, need to readjust with the help of MCL")
                        
            for i=1, N do
                if (last_command == "forward") then
                    --print("I WILL DO SOME READJUSTING NOW")
                    e = gaussian(0, distance_to_move * 0.001)
                    f = gaussian(0, distance_to_move * 0.001)
                    xArray[i] = xArray[i] + ((distance_to_move + e) * math.cos(thetaArray[i]))
                    yArray[i] = yArray[i] + ((distance_to_move + e) * math.sin(thetaArray[i]))
                    thetaArray[i] = thetaArray[i] + f
                elseif (last_command == "turn") then
                    if (angle_to_turn ~= 0) then
                        g = gaussian(0, (angle_to_turn / math.rad(90)) * 0.0007)
                        thetaArray[i] = thetaArray[i] + angle_to_turn + g
                    end
                end
            
                sim.setObjectPosition(dummyArray[i], -1, {xArray[i], yArray[i], 0})
                sim.setObjectOrientation(dummyArray[i], -1, {0, 0, thetaArray[i]})
            end
        elseif (newStepType == "readjust") then
            -- Get measurement from sensor and add noise
            result,cleanDistance=sim.readProximitySensor(turretSensor)
            if (result > 0) then
                noisyDistance = cleanDistance + gaussian(0.0, sensorVariance)
            end
            --print("Noisy distance is ", noisyDistance)
            
            for i = 1, N do
                -- Calculate the likelihood for each particle
                if (result > 0) then
                    likelihood = calculateLikelihood(xArray[i], yArray[i], thetaArray[i], noisyDistance, i)
                    weightArray[i] = weightArray[i] * likelihood
                end
            end
            
            sumOfWeights = 0
            for j = 1, N do
                sumOfWeights = sumOfWeights + weightArray[j]
            end
        
            for k = 1, N do
                weightArray[k] = weightArray[k] / sumOfWeights
                --print("Index and weight", k, weightArray[k])
            end
            
            --print("WEIGHTS BEFORE RESAMPLING")
            
            for x=1, N do
                --print("Index and weight: ", x, weightArray[x])
            end
            
            -- Perform resampling to create 100 new dummies whose spatial distribution now reflects the probability density
            resampling()
        end
    end


    -- Handle current ongoing step
    stepType = stepList[stepCounter][1]

    if (stepType == "turn") then
        --print("Performing a turn, need to turn", newStepAmount / math.pi)
        if (angle_to_turn >= 0) then
            speedBaseL = -speedBase
            speedBaseR = speedBase
        else
            speedBaseL = speedBase
            speedBaseR = -speedBase
        end
        motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        -- Slow down when close
        if (motorAngleFromTarget < 3) then
            speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        if (motorAngleFromTarget == 0) then
            stepCompletedFlag = true
        end
    elseif (stepType == "forward") then
        --print("Proceeding forward")
        speedBaseL = speedBase
        speedBaseR = speedBase
        motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        distance = distance_to_move
        -- Slow down when close
        if (motorAngleFromTarget < 3) then
            speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        if (motorAngleFromTarget == 0) then
            stepCompletedFlag = true
        end
    elseif (stepType == "stop") then
        --print("Stopping and checking that I have stopped")
        speedBaseL = 0
        speedBaseR = 0

        -- Check to see if the robot is stationary to within a small threshold
        linearVelocity,angularVelocity=sim.getVelocity(robotBase)
        vLin = math.sqrt(linearVelocity[1]^2 + linearVelocity[2]^2 + linearVelocity[3]^2)
        vAng = math.sqrt(angularVelocity[1]^2 + angularVelocity[2]^2 + angularVelocity[3]^2)
        --print ("stop", linearVelocity, vLin, vAng)
    
        if (vLin < 0.001 and vAng < 0.01) then
            stepCompletedFlag = true
        end
    elseif (stepType == "set_waypoint") then
        --print("The waypoint has been set")
        stepCompletedFlag = true
    end

    if (newStepType == "readjust") then
        -- Loop back to the first step
        if (stepCounter == 7) then
            stepCounter = 0
            waypointCounter = waypointCounter + 1
            if (waypointCounter > N_WAYPOINTS) then
                waypointCounter = 1
            end
        end
        stepCompletedFlag = true
    end

    -- Set the motor velocities for the current step
    sim.setJointTargetVelocity(leftMotor,speedBaseL)
    sim.setJointTargetVelocity(rightMotor,speedBaseR)        


end

function sysCall_cleanup()
    --simUI.destroy(ui)
end 

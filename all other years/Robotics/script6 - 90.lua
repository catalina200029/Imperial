-- Generate a sample from a Gaussian distribution
function gaussian (mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random() + 0.00001)) *
            math.cos(2 * math.pi * math.random()) + mean
end

function realGaussian(z, mean, variance)
    return math.exp(-((z - mean) ^ 2) / (2 * variance))
end


-- Move robot to a location (only for use in random setup, not from your code!)
function setRobotPose(handle, x, y, theta)
    allModelObjects = sim.getObjectsInTree(handle) -- get all objects in the model
    sim.setThreadAutomaticSwitch(false)
    for i=1,#allModelObjects,1 do
        sim.resetDynamicObject(allModelObjects[i]) -- reset all objects in the model
    end
    pos = sim.getObjectPosition(handle, -1)
    sim.setObjectPosition(handle, -1, {x, y, pos[3]})
    sim.setObjectOrientation(handle, -1, {0, 0, theta})
    sim.setThreadAutomaticSwitch(true)
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
    --heightFieldNoise = 0.00000005
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
            print("Wall " ..tostring(N).. " along x axis")
            Ax = pos[1] + minx
            Ay = pos[2]
            Bx = pos[1] + maxx
            By = pos[2]
        else
            print("Wall " ..tostring(N).. " along y axis")
            Ax = pos[1]
            Ay = pos[2] + miny
            Bx = pos[1]
            By = pos[2] + maxy
        end
        print (Ax, Ay, Bx, By)

        walls[N] = {Ax, Ay, Bx, By}
        N = N + 1
    end
    -- enable error reporting
    sim.setInt32Param(sim.intparam_error_report_mode,savedState)

    return N - 1
end


function get_goals()
    -- Disable error reporting
    local savedState=sim.getInt32Param(sim.intparam_error_report_mode)
    sim.setInt32Param(sim.intparam_error_report_mode,0)
    local N = 1
    goalHandles = {}
    while true do
        local handle = sim.getObjectHandle("Goal"..tostring(N))
        if handle <= 0 then
            break
        end
        
        goalHandles[N] = handle

        -- Read position of goal
        local pos = sim.getObjectPosition(handle, -1)
    
        print("Position of Goal " .. tostring(N) .. ": " .. tostring(pos[1]) .. "," .. tostring(pos[2]) .. "," .. tostring(pos[3]))

        goals[N] = {pos[1], pos[2]}
        N = N + 1
    end
    -- enable error reporting
    sim.setInt32Param(sim.intparam_error_report_mode,savedState)

    return N - 1
end



-- Robot should call this function when it thinks it has reached goal N
-- Second argument is the robot's handle
function reachedGoal(N, handle)

    green = {0, 1, 0}
    yellow = {1, 1, 0}
    blue = {0, 0, 1}
    grey = {0.5, 0.5, 0.5}
    
    local pos = sim.getObjectPosition(handle, -1)
    local xerr = pos[1] - goals[N][1]
    local yerr = pos[2] - goals[N][2]
    local err = math.sqrt(xerr^2 + yerr^2)
    local localpts = 0
    local colour = grey
    if (err < 0.05) then
        localpts = 3
        colour = green
    elseif (err < 0.1) then
        localpts = 2
        colour = yellow
    elseif (err < 0.2) then
        localpts = 1
        colour = blue
    end
 
    -- Colour the goal
    --local goalHandle = sim.getObjectHandle("Goal" .. tostring(N))
    sim.setShapeColor(goalHandles[N], nil, sim.colorcomponent_ambient_diffuse, colour)
 
    -- if we're not at final goal (which is where we started)
    if (localpts > 0 and goalsReached[N] == false) then
        goalsReached[N] = true
        totalPoints = totalPoints + localpts
        print ("Reached Goal" ..tostring(N).. " with error " ..tostring(err).. ": Points: " ..tostring(localpts))
    end

    -- at final goal: have we reached all goals?
    if (N == startGoal and localpts > 0) then
        local allGoalsReached = true
        for i=1,N_GOALS do
            if (goalsReached[i] == false) then
                allGoalsReached = false
            end
        end
        -- Yes... all goals achieved so calculate time
        if (allGoalsReached == true) then
            tt = sim.getSimulationTime() 
            timeTaken = tt - startTime
            timePoints = 0
            if (timeTaken < 60) then
                timePoints = 5
            elseif (timeTaken < 90) then
                timePoints = 4
            elseif (timeTaken < 120) then
                timePoints = 3
            elseif (timeTaken < 180) then
                timePoints = 2
            elseif (timeTaken < 240) then
                timePoints = 1
            end
            totalPoints = totalPoints + timePoints
            print ("FINISH at time" ..tostring(timeTaken).. " with total points " ..tostring(totalPoints))

            sim.pauseSimulation()
        end
    end

end


-- This function is executed exactly once when the scene is initialised
function sysCall_init()

    startTime = sim.getSimulationTime()
    print("Start Time", startTime)
          
    robotBase=sim.getObjectHandle(sim.handle_self) -- robot handle
    leftMotor=sim.getObjectHandle("leftMotor") -- Handle of the left motor
    rightMotor=sim.getObjectHandle("rightMotor") -- Handle of the right motor
    turretMotor=sim.getObjectHandle("turretMotor") -- Handle of the turret motor
    turretSensor=sim.getObjectHandle("turretSensor")
     
    -- Please use noisyDistance= cleanDistance + gaussian(0.0, sensorVariance) for all sonar sensor measurements
    sensorStandardDeviation = 0.1
    sensorVariance = sensorStandardDeviation^2
    noisyDistance = 0

    -- Create bumpy floor for robot to drive on
    createRandomBumpyFloor()
 
    -- Data structure for walls (your program can use this)
    walls = {}
    -- Fill it by parsing the scene in the GUI
    N_WALLS = get_walls()
    -- walls now is an array of arrays with the {Ax, Ay, Bx, By} wall coordinates



    -- Data structure for goals (your program can use this)
    goals = {}
    -- Fill it by parsing the scene in the GUI
    N_GOALS = get_goals()
    -- goals now is an array of arrays with the {Gx, Gy} goal coordinates

    for g=1,N_GOALS do
        print ("Goal" ..tostring(g).. " Gx " ..tostring(goals[g][1]).. " Gy " ..tostring(goals[g][2]))
    end

 

    -- Randomise robot start position to one of the goals with random orientation
    startGoal = math.random(N_GOALS)
    startx = goals[startGoal][1]
    starty = goals[startGoal][2]
    startOrientation = math.random() * 2 * math.pi
    setRobotPose(robotBase, startx, starty, startOrientation)
 
 
    -- These variables are for keeping score, and they will be changed by reachedGoal() --- don't change them directly!
    totalPoints = 0
    goalsReached = {}
    for i=1,N_GOALS do
        goalsReached[i] = false
    end
  
  
       
    -- Your code here!

    stepCounter = 1
    setupCounter = 1
    waypointCounter = 1
    isSetup = true
    
    -- Create and initialise arrays for particles, and display them with dummies
    xArray = {}
    yArray = {}
    thetaArray = {}
    weightArray = {}
    dummyArray = {}
    N = 110
    
    target_x = 0
    target_y = 0
    
    angle_to_turn = 0
    last_command = ""
    distance_to_move = 0
    noisyDistance = 0

    M = N / 11

    placeDummies(goals[1][1], goals[1][2], math.rad(180), M, 1)

    placeDummies(goals[2][1], goals[2][2], math.rad(0), M, M + 1)
    placeDummies(goals[2][1], goals[2][2], math.rad(90), M, 2 * M + 1)
    placeDummies(goals[2][1], goals[2][2], math.rad(180), M, 3 * M + 1)

    placeDummies(goals[3][1], goals[3][2], math.rad(0), M, 4 * M + 1)
    placeDummies(goals[3][1], goals[3][2], math.rad(90), M, 5 * M + 1)

    placeDummies(goals[4][1], goals[4][2], math.rad(0), M, 6 * M + 1)
    placeDummies(goals[4][1], goals[4][2], math.rad(-90), M, 7 * M + 1)
    placeDummies(goals[4][1], goals[4][2], math.rad(180), M, 8 * M + 1)

    placeDummies(goals[5][1], goals[5][2], math.rad(-90), M, 9 * M + 1)
    placeDummies(goals[5][1], goals[5][2], math.rad(180), M, 10 * M + 1)

    setup_amount_turned = 0
    
    -- Usual rotation rate for wheels (radians per second)
    speedBase = 5
    speedBaseL = 0
    speedBaseR = 0
    stepCompletedFlag = false

    setupList = {}
    setupList[1] = {"turn_by_angle"}
    setupList[2] = {"stop"}
    setupList[3] = {"measure_if_90"}
    setupList[4] = {"find_position"}
    setupList[5] = {"find_waypoint"}

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
    
    -- EXAMPLE: student thinks they have reached a goal
    -- reachedGoal(1, robotBase)

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

function placeDummies(x, y, theta, M, idx) 
    for i=idx, M + idx - 1 do
        xArray[i] = x
        yArray[i] = y
        thetaArray[i] = theta
        weightArray[i] = 1/N
        dummyArray[i] = sim.createDummy(0.05)
        sim.setObjectPosition(dummyArray[i], -1, {x,y,0})
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,theta})
    end
end 

function turn_by(amount) 
    -- Readjust to nearest 90 degree 0.5m away
    angle_to_turn = amount
    
    newStepAmount = angle_to_turn
    
    motorAngleTargetL = posL - (angle_to_turn * motorAnglePerRadian)
    motorAngleTargetR = posR + (angle_to_turn * motorAnglePerRadian)

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

    posL = sim.getJointPosition(leftMotor)
    posR = sim.getJointPosition(rightMotor)

    -- result,cleanDistance=sim.readProximitySensor(turretSensor)
    -- if (result>0) then
    --     noisyDistance= cleanDistance + gaussian(0.0, sensorVariance)
    --     --print ("Depth sensor reading ", noisyDistance)
    -- end


    -- PREPARE TO MOVE

    if (isSetup) then
        currentStep = setupList[setupCounter][1]
    else
        currentStep = stepList[setupCounter][1]
    end

    print("CURRENT STEP IS ", currentStep)

    if (currentStep == "turn_by_angle") then
        print("Preparing to a turn, need to turn")
        turn_by(math.rad(2))
    elseif (currentStep == "stop") then
        speedBaseL = 0
        speedBaseR = 0
    elseif (currentStep == "find_position") then
        turn_by(math.rad(22.5))
        setup_amount_turned = setup_amount_turned + math.rad(22.5) 
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
        
        -- Perform resampling to create 110 new dummies whose spatial distribution now reflects the probability density
        resampling()

        -- Maybe ensure all dummies in one place instead
        if (setup_amount_turned >= math.rad(360)) then
            stepCounter = stepCounter + 1
        end
    
    end

 

    -- result,cleanDistance=sim.readProximitySensor(turretSensor)
    -- if (result>0) then
    --     noisyDistance= cleanDistance + gaussian(0.0, sensorVariance)
    --     --print ("Depth sensor reading ", noisyDistance)
    -- end


   -- MOVE

    if (isSetup) then
        if (currentStep == "turn_by_angle") then
            print("Performing a turn, need to turn", newStepAmount / math.pi)
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
            
            setupCounter = setupCounter + 1
        elseif (currentStep == "stop") then
            print(setupCounter)
            setupCounter = setupCounter + 1
            print(setupCounter)
        elseif (currentStep == "measure_if_90") then
            cleanDistanceSum = 0
            numberOfCleanDistances = 0
            for i = 1, 20 do
        
                result,cleanDistance=sim.readProximitySensor(turretSensor)
                if (result > 0) then
                    cleanDistanceSum = cleanDistanceSum + cleanDistance
                    numberOfCleanDistances = numberOfCleanDistances + 1
                end

                -- if (result > 0) then
                --     noisyDistance = cleanDistance + gaussian(0.0, sensorVariance)
                -- end
            end
        
            print("Final cleanDistance ", cleanDistance)

            cleanDistance = cleanDistanceSum / numberOfCleanDistances
            if (math.abs(cleanDistance - 0.5) < 0.0025) then
                setupCounter = setupCounter + 1
            else
                setupCounter = 1
            end
        end
    end

    -- Set the motor velocities for the current step
    sim.setJointTargetVelocity(leftMotor,speedBaseL)
    sim.setJointTargetVelocity(rightMotor,speedBaseR)


end

function sysCall_cleanup()
    for g=1,N_GOALS do
        sim.setShapeColor(goalHandles[g], nil, sim.colorcomponent_ambient_diffuse, {1, 0, 0})
    end
end 


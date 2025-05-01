from controller import Robot, DistanceSensor, Motor

#-------------------------------------------------------
""" WARNING: This obstacle avoiding controller for the robot 
takes a bit of time once it detects the bottle. 
Please be a bit patient until it does :)
After a few iterations it exhibits chaotic behaviour that I could not identify why :( """

TIME_STEP = 64
MAX_SPEED = 6.28

robot = Robot()

timestep = int(robot.getBasicTimeStep())   # [ms]

states = [
    'forward', 'turn_right', 'turn_left', 
    'turn_90deg_left', 'forward2', 'turn_left_2', 'forward3'
]
current_state = 'forward'
last_state = current_state

counter = 0
COUNTER_MAX = 5
OBSTACLE_THRESHOLD = 100
NOT_IN_LINE = 0 

ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7'
]
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

# ground sensors
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)

# motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

#-------------------------------------------------------
# Main loop
while robot.step(timestep) != -1:
    psValues = [ps[i].getValue() for i in range(8)]
    gsValues = [gs[i].getValue() for i in range(3)]

    line_right = gsValues[0] > 500
    line_left = gsValues[2] > 500
    obstacle_detected = any(sensor_value > OBSTACLE_THRESHOLD for sensor_value in psValues[:2])  # assuming front sensors are ps0, ps1, ps2

    leftSpeed = 0
    rightSpeed = 0

    if current_state == 'forward':
        leftSpeed = MAX_SPEED
        rightSpeed = MAX_SPEED
        if line_right and not line_left:
            current_state = 'turn_right'
            counter = 0
        elif line_left and not line_right:
            current_state = 'turn_left'
            counter = 0
        elif obstacle_detected:
            current_state = 'turn_90deg_left'
            counter = 0

    elif current_state == 'turn_right':
        leftSpeed = 0.8 * MAX_SPEED
        rightSpeed = 0.4 * MAX_SPEED
        if counter >= COUNTER_MAX:
            current_state = 'forward'

    elif current_state == 'turn_left':
        leftSpeed = 0.4 * MAX_SPEED
        rightSpeed = 0.8 * MAX_SPEED
        if counter >= COUNTER_MAX:
            current_state = 'forward'

    elif current_state == 'turn_90deg_left':
        leftSpeed = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
        if counter >= 25: 
            current_state = 'forward2'
            counter = 0
            
    elif current_state == 'turn_90deg_right':
        leftSpeed = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
        if counter >= 15: 
            current_state = 'forward2'
            counter = 0

    elif current_state == 'forward2':
        leftSpeed = MAX_SPEED
        rightSpeed = MAX_SPEED
        if counter >= 10: 
            current_state = 'turn_right_2'
            counter = 0

    elif current_state == 'turn_right_2':
        leftSpeed = 0.5 * MAX_SPEED
        rightSpeed = -0.5 * MAX_SPEED
        if counter >= 5:  
            current_state = 'forward3'
            counter = 0

    elif current_state == 'forward3':
        leftSpeed = MAX_SPEED
        rightSpeed = MAX_SPEED
        if counter >= 5:  
            current_state = 'turn_right_2'
            counter = 0
            
        if line_right and line_left:
            NOT_IN_LINE = 1
            
        if NOT_IN_LINE == 1:
            if not line_right and not line_left:
                print('stop')
                current_state = 'turn_left'
                counter = -20
                NOT_IN_LINE = 0

    counter += 1

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    print('Counter: '+ str(counter) + '. Current state: ' + current_state, NOT_IN_LINE)
    print(gsValues)
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)






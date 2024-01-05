#falling states---------------------------------------------
'''
fallingState = 4 #0:launching 1:falling (para open) 2:falling (para closed) 3:landing (para open) 4:idle (won't do checks)
valuesInLine = 0
prevAlt = presAltitude
motorTime = 0
closeHeight = 800
openHeight = 770
def StateMachineFalling(state):
    global altitude, presAltitude, valuesInLine, prevAlt, cycleCount, motorTime, openHeight, closeHeight

    if state == 0:
        if my_gps.fix_stat > 0:
            if valuesInLine >= 4:
                return 1
            elif prevAlt > altitude: 
                valuesInLine += 1
                prevAlt = altitude
                return state
            else:
                valuesInLine = 0
                prevAlt = altitude
                return state
        else:
            if valuesInLine >= 4:
                return 1
            elif prevAlt > presAltitude: 
                valuesInLine += 1
                prevAlt = presAltitude
                return state
            else:
                valuesInLine = 0
                prevAlt = presAltitude
                return state

    elif state == 1:
        if my_gps.fix_stat > 0:
            if altitude <= closeHeight:
                motor_on(1)
                motorTime = cycleCount
                return 2
            else:
                return state
        else:
            if presAltitude <= closeHeight:
                motor_on(1)
                motorTime = cycleCount
                return 2
            else:
                return state
    elif state == 2:
        if motorTime + PULL_TIME <= cycleCount:
            motor_off()
        if my_gps.fix_stat > 0:
            if altitude <= openHeight:
                motor_on(-1)
                motorTime = cycleCount
                return 3
            else:
                return state
        else:
            if presAltitude <= openHeight:
                motor_on(-1)
                motorTime = cycleCount
                return 3
            else:
                return state
    
    elif state == 3:
        if motorTime + PULL_TIME - 1700 <= cycleCount:
            motor_off()
            return 4
        else:
            return state
    else:
        return state
'''
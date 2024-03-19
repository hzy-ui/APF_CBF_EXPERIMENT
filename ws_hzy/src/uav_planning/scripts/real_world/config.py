# mission mode
WAITING     = 0
ARM         = 1
TAKEOFF     = 2
LAND        = 3
TRACK       = 4
HOVER       = 5
DISARM      = 6
TASK_BEGIN  = 9
TASK_END    = 919

RIGHT_GOING = 10
LEFT_GOING  = 11
TURN_LEFT   = 20
TURN_RIGHT  = 21
TURN_BACK   = 22
TURN_AHEAD  = 23

MODE_DIC = {WAITING     :"WATING", 
            ARM         :"ARM",
            TAKEOFF     :"TAKEOFF",
            LAND        :"LAND   ",
            TRACK       :"TRACK  ",
            HOVER       :"HOVER  ",
            DISARM      :"DISARM",
            RIGHT_GOING :"RIGHT_GOING",
            LEFT_GOING  :"LEFT_GOING",
            TASK_BEGIN  :"TASK_BEGIN",
            TASK_END    :"TASK_END"
            }

def print_mode_tip():
    for key in MODE_DIC:
        print(f"{key}:{MODE_DIC.get(key)}")


if __name__ == "__main__":
    print_mode_tip()

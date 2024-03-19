import math

b = 0
t_0 = 0
t_1 =0
t_pre = 0
t_pre_1 = 0
gamma_0 = 0
gamma_inf = 0
l = 0
b_t = 0

def gamma_param(x_initial, x_goal, R, t_star):
    '''
    gamma_param is to calculate the parameter of gamma
        x_initial: initial position;
        x_goal: goal position 
        R: radius of goal area
        t_star: important time period -- t_star = t^{*} -t_{a} for F/G_[a,b]
    '''
    global gamma_0, gamma_inf, l

    delta = 0.1 # intial inside the forward invariant set 
    r = 0.1  # the robustness of the formulas 
    l_max = 0.9 # bound for l
    gamma_inf = 0.3
    h_0 = R - math.sqrt( (x_initial[0] - x_goal[0])**2 + (x_initial[1] - x_goal[1])**2 )
    gamma_0 = h_0 - delta
    # l =  ( -math.log( (r-gamma_inf)/(gamma_0 -gamma_inf) ) ) / t_star 


    if gamma_inf > R:
        gamma_inf = R
    else:
        gamma_inf = gamma_inf

    if t_star > 0:
        if l <= l_max:
            l =  ( -math.log( (r-gamma_inf)/(gamma_0 -gamma_inf) ) ) / t_star 
            if l > l_max:
                l = l_max 
        else:
            l = l_max 
        # 
        # if l >l_max:
        #     l = l_max
    else:
        l = l_max
        # print("cccccccccccccccccccccccccccccccccccccccccccc")

    return gamma_0, gamma_inf, l



def CBF_generator_updates(x, x_initial, x_goal, R, t_star, t):
    '''
    partial_calculate is to calculate partial derivatives of CBF   
    R: 
    x: position;
    x_goal: goal position 
    R: radius of goal area
    t: time period t = t-t_begin
    t_star: important time period -- t_star = t^{*} -t_{a} for F/G_[a,b]
    '''
    global gamma_0, gamma_inf, l, b, t_0, t_pre, b_t
    # print(l)
    delta_t = t - t_pre
    t_pre = t
    p_h_x1 = (-1/2) * ( ( (x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2 ) **(-1/2) ) * 2 * (x[0] - x_goal[0])
    p_h_x2 = (-1/2) * ( ( (x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2 ) **(-1/2) ) * 2 * (x[1] - x_goal[1])
    p_b_x1 = p_h_x1
    p_b_x2 = p_h_x2

    h = R - math.sqrt((x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2)

    if t_0 == 0:
        t_0 = t
        gamma_0, gamma_inf, l = gamma_param(x, x_goal, R, t_star - t_0)
        gamma = (gamma_0 - gamma_inf) * math.exp(-l * (t - t_0)) + gamma_inf 
        h = R - math.sqrt((x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2)
        b = h - gamma 
        # b_t = - ( -l * (gamma_0 - gamma_inf) * math.exp(-l*(t - t_0)) )

    else:
        if b < 0:
            t_0 = t
            gamma_0, gamma_inf, l = gamma_param(x, x_goal, R, t_star - t_0)
            gamma = (gamma_0 - gamma_inf) * math.exp(-l * (t - t_0)) + gamma_inf
            h = R - math.sqrt((x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2)
            b = h - gamma
            # b_t = - ( -l * (gamma_0 - gamma_inf) * math.exp(-l*(t - t_0)) )
            # print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        else:
            # if b > 3:
            #     t_0 = t
            #     gamma_0, gamma_inf, l = gamma_param(x, x_goal, R, t_star - t_0)
            #     gamma = (gamma_0 - gamma_inf) * math.exp(-l * (t - t_0)) + gamma_inf 
            #     h = R - math.sqrt((x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2)
            #     b = h - gamma 
            # else:
            h = R - math.sqrt((x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2)
            gamma = (gamma_0 - gamma_inf) * math.exp(-l * (t - t_0)) + gamma_inf
            b = h - gamma
            # b_t = - ( -l * (gamma_0 - gamma_inf) * math.exp(-l*(t - t_0)) )
            if b < 0 or delta_t < 0:
                t_0 = t
                gamma_0, gamma_inf, l = gamma_param(x, x_goal, R, t_star - t_0)
                gamma = (gamma_0 - gamma_inf) * math.exp(-l * (t - t_0)) + gamma_inf 
                h = R - math.sqrt((x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2)
                b = h - gamma 
                # b_t = - ( -l * (gamma_0 - gamma_inf) * math.exp(-l*(t - t_0)) )
                # print("bbbbbbbbbbbbbbbbbbbbbbbbbbbb")
    b_t = - ( -l * (gamma_0 - gamma_inf) * math.exp(-l*(t - t_0)) )
    return h, gamma, b, p_b_x1, p_b_x2, b_t
# t = 0 
# x_inital = (0, 0)
# x= (0.2,0.2)  
# x_goal = (5, 5)  
# R = 1.414
# t_star =50 
# h , gamma, b = CBF_generator(x, x_inital, x_goal, R, t_star, t) 
# print(h) 
# print(gamma) 
# print(b) 

def partial_calculate_updates(x, x_initial, x_goal, R, t_star, t):
    '''
    partial_calculate is to calculate partial derivatives of CBF   
    x: position;
    x_goal: goal position 
    t: time period t = t-t_{0}
    '''
    global gamma_0, gamma_inf, l, b, t_1, t_pre_1

    delta_t = t - t_pre_1
    t_pre_1 = t

    p_h_x1 = (-1/2) * ( ( (x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2 ) **(-1/2) ) * 2 * (x[0] - x_goal[0])
    p_h_x2 = (-1/2) * ( ( (x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2 ) **(-1/2) ) * 2 * (x[1] - x_goal[1])

    p_b_x1 = p_h_x1
    p_b_x2 = p_h_x2
    if t_1 == 0:
        t_1 = t
        gamma_0, gamma_inf, l = gamma_param(x, x_goal, R, t_star-t_1) 
        b_t = - ( -l * (gamma_0 - gamma_inf) * math.exp(-l*(t - t_1)) )
    else:
        if b < 0:
            t_1 = t
            # h = R - math.sqrt( (x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2 )
            gamma_0, gamma_inf, l = gamma_param(x, x_goal, R, t_star-t_1) 
            # gamma = (gamma_0 - gamma_inf) * math.exp(-l * (t - t_0)) + gamma_inf
            # b = h - gamma
            b_t = - ( -l * (gamma_0 - gamma_inf) * math.exp(-l*(t - t_1)) )
        else:
            b_t = - ( -l * (gamma_0 - gamma_inf) * math.exp(-l*(t - t_1)) )
            if b < 0 or delta_t < 0:
                t_1 = t
                gamma_0, gamma_inf, l = gamma_param(x, x_goal, R, t_star-t_1) 
                b_t = - ( -l * (gamma_0 - gamma_inf) * math.exp(-l*(t - t_1)) )




    return p_b_x1, p_b_x2, b_t
import math
'''
These functions construct the parameter for CBF.

'''
t_pre = 0
t_pre_1 = 0
x_initial = (0,0)
t_0 = 0
gamma_0 = 0
gamma_inf = 0
l = 0

def gamma_param(x_goal, R, t_star):
    '''
    gamma_param is to calculate the parameter of gamma
        x_initial: initial position;
        x_goal: goal position 
        R: radius of goal area
        t_star: important time period -- t_star = t^{*} -t_{a} for F/G_[a,b]
    '''
    global x_initial
    delta = 1 # intial inside the forward invariant set 
    r = 0.01  # the robustness of the formulas 
    l_max = 10 # bound for l
    gamma_inf = 0.3
    h_0 = R - math.sqrt( (x_initial[0] - x_goal[0])**2 + (x_initial[1] - x_goal[1])**2 )
    gamma_0 = h_0 - delta

    if gamma_inf > R:
        gamma_inf = R
    else:
        gamma_inf = gamma_inf

    if t_star > 0:
        l =  ( -math.log( (r-gamma_inf)/(gamma_0 -gamma_inf) ) ) / t_star 
        if l >l_max:
            l = l_max
    else:
        l = l_max

    return gamma_0, gamma_inf, l

# x_inital = (0, 0)  
# x_goal = (90, 90)  
# R = 10 
# t_star =15 
# gamma_0, gamma_inf, l = gamma_param(x_inital, x_goal, R, t_star) 
# print(gamma_0) 
# print(gamma_inf) 
# print(l) 

def CBF_generator(x, x_goal, R, t_star, t):
    '''
    partial_calculate is to calculate partial derivatives of CBF   
    R: 
    x: position;
    x_goal: goal position 
    R: radius of goal area
    t: time period t = t-t_begin
    t_star: important time period -- t_star = t^{*} -t_{a} for F/G_[a,b]
    '''
    global  t_pre, x_initial, t_0, gamma_0, gamma_inf, l

    delta_t = t - t_pre
    t_pre = t
    h = R - math.sqrt( (x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2 )
    if delta_t < 0:
        t_0 = t
        x_initial = (x[0], x[1])
        gamma_0, gamma_inf, l = gamma_param(x_goal, R, t_star) 
        gamma = (gamma_0 - gamma_inf) * math.exp(-l * (t-t_0)) + gamma_inf
        b = h - gamma

    # h = R - math.sqrt( (x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2 )
    gamma_0, gamma_inf, l = gamma_param(x_goal, R, t_star) 
    gamma = (gamma_0 - gamma_inf) * math.exp(-l * (t-t_0)) + gamma_inf
    b = h - gamma
    # print(gamma) 
    return h , gamma, b
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


def partial_calculate(x, x_goal, R, t_star, t):
    '''
    partial_calculate is to calculate partial derivatives of CBF   
    x: position;
    x_goal: goal position 
    t: time period t = t-t_{0}
    '''
    global t_pre_1, x_initial, t_0, gamma_0, gamma_inf, l

    delta_t = t - t_pre_1
    t_pre_1 = t
    if delta_t < 0:
        t_0 = t
        x_initial = (x[0], x[1])
        gamma_0, gamma_inf, l = gamma_param(x_goal, R, t_star) 
        b_t = - ( -l * (gamma_0 - gamma_inf) * math.exp(-l*(t - t_0)) )


    p_h_x1 = (-1/2) * ( ( (x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2 ) **(-1/2) ) * 2 * (x[0] - x_goal[0])
    p_h_x2 = (-1/2) * ( ( (x[0] - x_goal[0])**2 + (x[1] - x_goal[1])**2 ) **(-1/2) ) * 2 * (x[1] - x_goal[1])

    p_b_x1 = p_h_x1
    p_b_x2 = p_h_x2


    gamma_0, gamma_inf, l = gamma_param(x_goal, R, t_star) 
    # print(l)

    b_t = - ( -l * (gamma_0 - gamma_inf) * math.exp(-l*(t-t_0)) )

    return p_b_x1, p_b_x2, b_t

# t = 10
# x_inital = (0, 0)
# x= (85,85)  
# x_goal = (90, 90)  
# R = 10 
# t_star =15 
# p_b_x1, p_b_x2, b_t= partial_calculate(x, x_inital, x_goal, R, t_star, t) 
# print(p_b_x1) 
# print(p_b_x2) 
# print(b_t) 
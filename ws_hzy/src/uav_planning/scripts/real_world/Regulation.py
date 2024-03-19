import math 

def regulation_function(v1_cpf, v2_cpf, v1_cbf, v2_cbf, x,obstacles):
    eta = 10
    beta = 0.1
    h_in = 0
    for obstacle in obstacles:
        ox, oy, oR = obstacle
        R_star = oR + 0.5
        h_star =  math.sqrt( (x[0] - ox)**2 + (x[1] - oy)**2 ) - R_star
        # print(math.sqrt( (x[0] - ox)**2 + (x[1] - oy)**2 ) - oR)
        h_in +=  math.exp(-eta * h_star)
    
    h_obs = (-1/eta) * math.log( h_in )
    h_obs_positive = max(h_obs,0)
    lambda_s = 1 - math.exp(-beta * h_obs_positive) 
    # print("lambda"+ str(lambda_s))
    u1 = lambda_s * v1_cbf + (1- lambda_s) * v1_cpf
    u2 = lambda_s * v2_cbf + (1- lambda_s) * v2_cpf
    return u1, u2



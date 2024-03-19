import math 

def laser_data_process(laser_data, angle_increment, angle_min, UAV_x, UAV_y):
    index_first_para=[]
    paragraphs = []
    current_paragraph = []
    all_inf = True
    laser_data_list = list(laser_data)

    for i,num in enumerate(laser_data_list):
        if laser_data_list[i] > 10:
            laser_data_list[i] = float('inf') 
    # print(laser_data_list)

     

    for i,num in enumerate(laser_data_list):
        if num != float('inf') or (laser_data_list[i-1] !=float('inf') and laser_data_list[(i+1)%(len(laser_data_list))] !=float('inf')) :
            if (laser_data_list[i] == float('inf') ):
                # print(i)
                num = (laser_data_list[i-1] + laser_data_list[(i+1)%(len(laser_data_list))])/2 
                laser_data_list[i] = num

            current_paragraph.append(num)
            all_inf = False
        else:
            if current_paragraph :
                # for j in current_paragraph:
                #     if current_paragraph[j] == float('inf'):
                #         current_paragraph[j] = (current_paragraph[j-1]+current_paragraph[j+1])/2
                paragraphs.append(current_paragraph)
                current_paragraph = []

    if laser_data_list[-1] != float('inf') :
        paragraphs.append(current_paragraph)

    # print(laser_data_list)
    # print(paragraphs)
    # print(current_paragraph)


    
    # find the index of the first number after inf
    for i, num in enumerate(laser_data_list):
        if num == float('inf'):
            check_index = i + 1
            if check_index <= len(laser_data_list)-1:
                if laser_data_list[check_index] != float('inf'):
                    index_first_para.append(check_index)
            else:
                check_index = 0
                if laser_data_list[check_index] != float('inf'):
                    index_first_para.append(check_index)


    # print(index_first_para)
    # print(len())


    # Handle cyclic connection between first and last numbers
    if laser_data_list[0] != float('inf') and laser_data_list[-1] != float('inf'):
        # print(paragraphs)
        # print(current_paragraph)
        current_paragraph.extend(paragraphs[0])
        # print(paragraphs)
        paragraphs = paragraphs[1:]
        # paragraphs.append(current_paragraph)
        # print(current_paragraph)
        # print(paragraphs)
    # Find the largest and smallest numbers within each paragraph
    obs_positions = []
    smallest_index_all = []

    for i, paragraph in enumerate(paragraphs):
        largest = max(paragraph)
        smallest = min(paragraph)
        smallest_index_paragraph = paragraph.index(smallest)  # Get the index of the smallest number within the paragraph
        if laser_data_list[0] != float('inf') and laser_data_list[-1] != float('inf'):
            smallest_index = index_first_para[i] + smallest_index_paragraph 
            if smallest_index > len(laser_data_list) -1:
                smallest_index = (smallest_index % (len(laser_data_list)-1))-1
        else:
            if laser_data_list[0] != float('inf'):
                if i == 0: # in this senario, 0 will be the last component in index_first_para
                    smallest_index = index_first_para[-1] + smallest_index_paragraph 
                else:
                    smallest_index = index_first_para[i-1] + smallest_index_paragraph 
            else:
                smallest_index =index_first_para[i] + smallest_index_paragraph 

        smallest_index_all.append(smallest_index)

        angle = angle_min + angle_increment * smallest_index

        R = ((largest + 0.08)**2 -(smallest)**2)/ (2*smallest)
        if R <= 0.2 or R >= 2:
            continue

        delta_x = (R + smallest) * math.cos(angle)
        delta_y = (R + smallest) * math.sin(angle)

        x = UAV_x + delta_x
        y = UAV_y + delta_y

        obs_positions.append((x, y, R))
        # largest_smallest_pairs.append((delta_x, delta_y))
    
    # for i in range(len(obs_positions)):
    #     x1, y1, R1 = obs_positions[i]
    #     for j in range(i+1, len(obs_positions)):
    #         x2, y2, R2 = obs_positions[j]
            
    #         distance = euclidean_distance((x1, y1), (x2, y2))
    #         if distance <=2 and abs(R1-R2)<=1:
    #             x_new = ( x1+x2 ) / 2
    #             y_new = ( y1+y2 ) / 2
    #             R_new = ( R1+R2 ) / 2


    # print(smallest_index_all)

    # Handle the case when all numbers are infinite
    if all_inf:
        obs_positions.append((float('inf'), 0, float('inf')))

    return obs_positions

# my_tuple = (float('inf'), float('inf'), float('inf'), float('inf'))
# my_tuple = (20, 1, 3, 2, float('inf'), float('inf'), 30, 10, 20, float('inf'), 4, 3, 9, 2, 2)

# print(my_tuple[-18%len(my_tuple)])
# print(len(my_tuple))
# largest_smallest_pairs = laser_data_process(my_tuple, 0.017, -3.14,0,0)
# print(largest_smallest_pairs)
# print(largest)
# print(smallest)
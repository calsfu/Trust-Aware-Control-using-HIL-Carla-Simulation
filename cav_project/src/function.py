from cav_project.msg import limo_state, limo_state_matrix
from cav_class import CAV



def calc_qp_info(order_list, limo_num, zone):
    front_num = search_ahead(order_list, limo_num)
    collision_num1, collision_num2 = search_collision(order_list, limo_num)
    limo = order_list[limo_num]
    collision_pt1 = limo.current_collision_pt1
    collision_pt2 = limo.current_collision_pt2
    starting_pt = limo.turning_pts[limo.current]


    d0 = calc_distance((limo.position_x, limo.position_z), starting_pt)

    #constraints for front cav
    if front_num == -1:
        d1 = -1
        v1 = -1
    else:
        d1 = calc_manhattan_d1(order_list, limo_num, front_num)
        v1 = order_list[front_num].velocity

    #constraints for collision cavs
    if collision_num1 == -1:
        d2 = -1
        v2 = -1
        l2 = -1
    elif collision_num1 == front_num:
        d2 = -1
        v2 = -1
        l2 = -1
    else:
        collision_cav1 = order_list[collision_num1]
        dk = calc_manhattan_distance(order_list, collision_num1, collision_pt1)
        di = calc_manhattan_distance(order_list, limo_num, collision_pt1)
        d2 = di - dk
        v2 = order_list[collision_num1].velocity
        l2 = calc_distance(collision_cav1.turning_pts[collision_cav1.current], collision_pt1)

    if collision_num2 == -1:
        d3 = -1
        v3 = -1
        l3 = -1
    elif collision_num2 == front_num:
        d3 = -1
        v3 = -1
        l3 = -1
    elif collision_num2 == collision_num1:
        d3 = -1
        v3 = -1
        l3 = -1
    else:
        collision_cav2 = order_list[collision_num2]
        dk = calc_manhattan_distance(order_list, collision_num2, collision_pt2)
        di = calc_manhattan_distance(order_list, limo_num, collision_pt2)
        d3 = di - dk
        v3 = order_list[collision_num2].velocity
        l3 = calc_distance(collision_cav2.turning_pts[collision_cav2.current], collision_pt2)

    #constraints for vd
    if limo.within_critical_range == True:
        vd = 1
    else:
        vd = 1

    #building the message
    limo_state_msg = limo_state()
    limo_state_msg.limoID = limo.ID
    limo_state_msg.vel = limo.velocity
    limo_state_msg.d0 = d0
    limo_state_msg.d1 = d1
    limo_state_msg.v1 = v1
    limo_state_msg.d2 = d2
    limo_state_msg.v2 = v2
    limo_state_msg.l2 = l2
    limo_state_msg.d3 = d3
    limo_state_msg.v3 = v3
    limo_state_msg.l3 = l3
    limo_state_msg.vd = vd
    return limo_state_msg


#below are helper functions for calc_qp_info() for rear end constraint
def search_ahead(order_list, limo_num):
    front_limo = -1
    for i in range(limo_num-1, -1, -1): # check all limos in front of the queue
        #if another limo is on the same line, check if you are closer to the next collision point or the limo in front of you
        if order_list[limo_num].current_line == order_list[i].current_line:
            limo_dist = calc_distance(order_list[limo_num].current_position, order_list[i].current_position)
            if order_list[limo_num].current_collision_pt1 != (-1, -1):
                crit_dist =  calc_distance(order_list[limo_num].current_position, order_list[limo_num].current_collision_pt1)
                #crit_dist = calc_manhattan_distance(order_list, limo_num, order_list[limo_num].current_collision_pt1)
                if crit_dist > limo_dist:
                    front_limo = i
                    break
            else:
                front_limo = i
                break

    return front_limo



#below are helper functions for calc_qp_info() for rear end constraint
def search_collision(order_list, limo_num, zone):
    front_limo = -1
    for i in range(limo_num-1, -1, -1): # check all limos in front of the queue
        #if another limo is on the same line, check if you are closer to the next collision point or the limo in front of you
        if order_list[limo_num].current_line == order_list[i].current_line:
            limo_dist = calc_distance(order_list[limo_num].current_position, order_list[i].current_position)
            if order_list[limo_num].current_collision_pt1 != (-1, -1):
                crit_dist =  calc_distance(order_list[limo_num].current_position, order_list[limo_num].current_collision_pt1)
                #crit_dist = calc_manhattan_distance(order_list, limo_num, order_list[limo_num].current_collision_pt1)
                if crit_dist > limo_dist:
                    front_limo = i
                    break
            else:
                front_limo = i
                break

    return front_limo

# search for lateral constraint
def search_collision(order_list, limo_num):
    limo = order_list[limo_num]
    collision_limo1 = -1
    collision_limo2 = -1
    starting_pt_ind = limo.all_pts.index(limo.turning_pts[limo.current])
    ending_pt_ind = limo.all_pts.index(limo.current_end_pt)

    #check all points on the same line, only consider collision points if they are on the same line
    for j in range(starting_pt_ind, ending_pt_ind+1):
        if limo.all_pts[j] == limo.current_collision_pt1 and limo.current_collision_pt1 != (-1, -1):
            for i in range(limo_num-1, -1, -1):
                #check if another limo who is in front of the queue shares a collision point
                if limo.current_collision_pt1 == order_list[i].current_collision_pt1 or limo.current_collision_pt1 == order_list[i].current_collision_pt2:
                    collision_limo1 = i
                    break
    for j in range(starting_pt_ind, ending_pt_ind+1):
        if limo.all_pts[j] == limo.current_collision_pt2 and limo.current_collision_pt2 != (-1, -1):
            for i in range(limo_num-1, -1, -1):
                if limo.current_collision_pt2 == order_list[i].current_collision_pt1 or limo.current_collision_pt2 == order_list[i].current_collision_pt2 :
                    collision_limo2 = i
                    break
    if limo.current_collision_pt1 == limo.current_collision_pt2:
        return collision_limo1, -1
    return collision_limo1, collision_limo2

def calc_distance(pt_1, pt_2):
    distance = ((pt_1[0]- pt_2[0]) ** 2 + (pt_1[1] - pt_2[1]) ** 2) ** 0.5
    return distance

def calc_manhattan_d1(order_list, limonum1, limonum2):
    limo2 = order_list[limonum2]
    limo1 = order_list[limonum1]

    if limo2.current_line != limo1.current_line:
        distance = calc_distance((limo1.position_x, limo1.position_y), limo1.current_end_pt)
        for i in range (limo1.next, limo2.current):
            distance += limo1.dist[i]

    else:
        distance = calc_distance((limo1.position_x, limo1.position_z), (limo2.position_x, limo2.position_z))

    return distance


def calc_manhattan_distance(order_list, limo_num, point):
    limo = order_list[limo_num]
    for i in range(len(limo.all_pts)):
        if limo.all_pts[i] == limo.current_end_pt or limo.all_pts[i] == point:
            dist = calc_distance(limo.current_position, limo.all_pts[i])
            if limo.all_pts[i] == point:
                return dist
            else:
                for j in range(len(limo.all_pts), i):
                    if limo.all_pts[j] != point:
                        dist += limo.dist[i]
                        return dist
            break
    return dist

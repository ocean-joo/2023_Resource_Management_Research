import math

experiment = '230417_opt_v07_full_02'
center_line_path = f'../results/{experiment}/0/center_line.csv'
center_offset_path = f'../results/{experiment}/0/center_offset.csv'
new_center_offset_path = f'../results/{experiment}/0/new_center_offset.csv'

def dis(wp1, wp2):
    return math.sqrt((wp1[0] - wp2[0]) * (wp1[0] - wp2[0]) + (wp1[1] - wp2[1]) * (wp1[1] - wp2[1]))

def find_closest_point(map_wp_list, wp):
    min_distance = 500
    min_wp = [0, 0]

    for map_wp in map_wp_list:
        if dis(map_wp, wp) < min_distance:
            min_distance = dis(map_wp, wp)
            min_wp = map_wp
    
    return min_wp, min_distance

def get_column_idx_from_csv(line):
    output = {}
    for i, v in enumerate(line):
        output[v] = i
    return output

map_wp_list = []
with open(center_line_path, "r") as f:
    for i, line in enumerate(f):
        if i == 0: continue
        x, y = map(float, line.split(','))
        map_wp_list.append([x, y])

new_center_offset = []
prev_dis = 0

with open(center_offset_path, "r") as f:
    for i, line in enumerate(f):
        line_lst = line.split(',')
        if i == 0: 
            column_idx = get_column_idx_from_csv(line_lst)
            continue
        x = line_lst[column_idx['current_pose_x']]
        y = line_lst[column_idx['current_pose_y']]

        min_wp, min_dis = find_closest_point(map_wp_list, [float(x), float(y)])

        if abs(prev_dis) > 1.5 and min_dis * prev_dis < 0:
            min_dis *= 1
        prev_dis = min_dis
        new_center_offset.append(min_dis)

with open(new_center_offset_path, "w") as f:
    for v in new_center_offset:
        f.write(f'{v}\n')






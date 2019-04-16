import sys
import os
import copy
import dijkstra
import dispatcher


road_to_direction = [4 * [None] for i in range(4)]
road_to_direction[0][1] = 'left'
road_to_direction[0][2] = 'direct'
road_to_direction[0][3] = 'right'

road_to_direction[1][0] = 'right'
road_to_direction[1][2] = 'left'
road_to_direction[1][3] = 'direct'

road_to_direction[2][0] = 'direct'
road_to_direction[2][1] = 'right'
road_to_direction[2][3] = 'left'

road_to_direction[3][0] = 'left'
road_to_direction[3][1] = 'direct'
road_to_direction[3][2] = 'right'

def update_x(x, x_new, method):
    if x is None:
        return x_new
    elif method == 'max' and x < x_new:
        return x_new
    elif method == 'min' and x > x_new:
        return x_new
    return x

class Car:
    def __init__(self, id, start, to, speed, planTime, is_priority, is_preset, start_time, graph, shorest_path_road=None):
        self.id = id
        self.start = start
        self.to = to
        self.speed = speed
        self.planTime = planTime
        self.is_priority = is_priority
        self.is_preset = is_preset
        self.start_time = start_time
        self.graph = graph

        self.state = None

        self.shorest_path_road, self.shorest_path_cross = shorest_path_road, None
        self.current_cross_start, self.current_cross_to = None, None
        self.current_road, self.current_channel, self.current_position = None, None, None
        self.next_cross, self.next_road = None, None
        self.direction = None
    
    def init_info(self):
        self.update_info(is_init=True)

    def update_path_info(self):
        self.update_info(is_init=True, only_path=True)

    def update_info_by_cp(self, channel, position):
        self.update_info(channel=channel, position=position)

    def update_info(self, channel=None, position=None, is_init=False, only_path=False):
        if is_init:
            if self.shorest_path_road is None:
                if self.graph.path_matrix[self.start][self.to] is None:
                    dijk = dijkstra.Dijkstra(self.graph)
                    dijk.run(self.start)
                self.shorest_path_cross = self.graph.path_matrix[self.start][self.to][0]
                self.shorest_path_road = self.graph.path_matrix[self.start][self.to][1]
            elif self.shorest_path_cross is None:
                self.shorest_path_cross = self.get_cross_path_by_road()
            self.current_cross_start = self.start
            self.current_cross_to = self.shorest_path_cross[1]
        else:
            self.current_cross_start = self.current_cross_to
            self.current_cross_to = self.next_cross

        if not only_path:
            
            self.current_road = self.graph.adjacent_matrix[self.current_cross_start][self.current_cross_to]
            if channel is None or position is None:
                self.current_channel, self.current_position = self.current_road.get_available_channel(
                    self)
            else:
                self.current_channel, self.current_position = channel, position

            self.next_cross = self.get_next_cross()
            if self.next_cross is None:
                self.next_road == None
            else:
                self.next_road = self.graph.adjacent_matrix[self.current_cross_to][self.next_cross]
            self.direction = self.get_direction_by_road(
                self.current_road, self.next_road)

            self.state = 'stop'

            if self.current_channel is not None and self.current_position is not None:
                self.current_channel[self.current_position] = self.id

    def get_cross_path_by_road(self):
        # 根据road_path生成cross_path
        cross_path = []
        cross_path.append(self.start)
        for road_id in self.shorest_path_road:
            road_data = self.graph.road_data[self.graph.road_id_list[road_id]]
            cross1 = self.graph.cross_id_list[road_data[4]]
            cross2 = self.graph.cross_id_list[road_data[5]]
            last_cross = copy.copy(cross_path[-1])
            if cross1 != last_cross:
                cross_path.append(cross1)
            if cross2 != last_cross:
                cross_path.append(cross2)
        return cross_path

    def wait_car(self):
        self.state = 'wait'

    def is_reach_end(self):
        return self.next_cross is None

    def get_next_cross(self):
        # 遍历用路口表示最短路list，根据当前路口，找到下一个路口
        len_path = len(self.shorest_path_cross)
        for i in range(len_path):
            if self.shorest_path_cross[i] == self.current_cross_to:
                if i+1 == len_path:
                    return None
                else:
                    return self.shorest_path_cross[i+1]

    # 获取能过路口的车 在下一条路 能够行驶的距离
    def get_t_dist_next_road(self):
        this_road = self.current_road
        traveled_distance = this_road.length - 1 - self.current_position
        next_road = self.next_road
        next_road_travelable_distance = min(self.speed, next_road.speed)
        return (next_road_travelable_distance - traveled_distance)

    def is_ok_enter_next_road(self):
        # 检查可行驶距离能否穿过路口
        dist_is_ok = self.get_t_dist_next_road() > 0

        # 检查下一道路是否有车道可进
        if dist_is_ok:
            next_road = self.next_road
            next_channel, _ = next_road.get_available_channel(self)
            if next_channel is not None:
                return True
        return False

    # 根据道路获取车辆方向
    def get_direction_by_road(self, road_start, road_to):
        if road_to is None:
            return None
        cross_i = -1
        if road_start.start == road_to.start or road_start.start == road_to.to:
            cross_i = road_start.start
        if road_start.to == road_to.start or road_start.to == road_to.to:
            cross_i = road_start.to
        if cross_i != -1:
            roads_in_cross = self.graph.cross_data[cross_i]
            road_start_i = roads_in_cross.index(road_start.id) - 1
            road_to_i = roads_in_cross.index(road_to.id) - 1
            return road_to_direction[road_start_i][road_to_i]
        return None

    # 获取车辆的方向优先级
    def get_direction_priority(self):
        if self.is_reach_end():
            return 2
        direct_priority_list = ['right', 'left', 'direct']
        return direct_priority_list.index(self.direction)


class Road:
    def __init__(self, id, length, speed, num_channel, start, to, isDuplex, graph):
        self.id = id
        self.length = length
        self.speed = speed
        self.num_channel = num_channel
        self.start = start
        self.to = to
        self.isDuplex = isDuplex
        self.cost = sys.maxsize
        self.direction = None
        self.graph = graph

        self.channel_list = [self.length * [None]
                             for i in range(self.num_channel)]
        
        self.num_cars = 0

        self.carport = []
        self.build_carport()

    def build_carport(self):
        if self.id not in self.graph.road_car_dict:
            return
        for car_id in self.graph.road_car_dict[self.id]:
            car = self.graph.car_object[car_id]
            car.update_path_info()
            if car.shorest_path_cross[0] == self.start and car.shorest_path_cross[1] == self.to:
                self.carport.append(car.id)
        self.carport.sort(key=lambda i: (
            -self.graph.car_object[i].is_priority, self.graph.car_object[i].start_time, i))
    
    def cars_enter_road(self, time_slice, only_priority=False):
        for car_id in copy.copy(self.carport):
            car = self.graph.car_object[car_id]
            if only_priority and car.is_priority == 0:
                return
            if car.start_time <= time_slice:
                car.init_info()
                if car.current_channel is None or car.current_position is None:  # 若这个时间片无法上车道 则下一个时间片发车
                    # print('no channel!')
                    return
                dispatcher.on_road_car_list.append(car_id)
                self.carport.remove(car_id)

    def get_available_channel(self, car, leave_one_place=False):
        for channel_i in range(self.num_channel):
            channel = self.channel_list[channel_i]
            if channel[0] == None:  # 若车道入车口第一个位置没有车，则车道可进入
                # 如果是车辆上路时，最后一个车道留出第一个位置
                if (leave_one_place and channel_i == self.num_channel-1):
                    if channel[1] is None:  # 只有第二个位置没车才可以进
                        position = self.get_available_position_in_channel(car,
                                                                          channel)
                        return (channel, position)
                else:
                    position = self.get_available_position_in_channel(
                        car, channel)
                    return (channel, position)
            # 若车道入车口有车，且车的状态为等待状态，也可进入
            elif self.graph.car_object[channel[0]].state == 'wait':
                return (channel, None)
        # 找不到可进入的车道
        return (None, None)

    def get_available_position_in_channel(self, car, channel):
        travelable_distance = min(car.speed, self.speed)
        obstacle_car_id = dispatcher.get_first_obstacle_front(
            channel, -1, travelable_distance)

        # 获取当前位置
        position = travelable_distance - 1
        if obstacle_car_id is not None:
            obstacle_car = self.graph.car_object[obstacle_car_id]
            if obstacle_car.state == 'wait':
                return None
            position = obstacle_car.current_position - 1
        return position

    def get_first_priority_car_id(self):
        first_not_priority_car_id = None
        for i in range(self.length):  # 按照 出路口到入路口、车道号从小到大 的顺序进行调度
            for channel_i in range(self.num_channel):
                channel = self.channel_list[channel_i]
                position = self.length - i - 1
                car_id = channel[position]
                if car_id is not None:  # 这是一辆车
                    car = self.graph.car_object[car_id]
                    if car.state == 'wait':
                        if car.is_priority == 1:  # 若是等待优先车辆
                            obstacle_car_id = dispatcher.get_first_obstacle_front(
                                channel, position, self.length-1-position)
                            if obstacle_car_id is None:  # 　若优先车辆前没有其他车阻挡
                                return car_id
                        elif first_not_priority_car_id is None:  # 记录第一辆等待非优先车辆
                            first_not_priority_car_id = car_id
        return first_not_priority_car_id


class Cross:

    def __init__(self, id, roadNId, roadEId, roadSId, roadWId, graph):
        #self.cross_data = cross_data
        self.id = id
        self.roadNId = roadNId
        self.roadEId = roadEId
        self.roadSId = roadSId
        self.roadWId = roadWId

        self.graph = graph

        self.p_carport = []
        self.n_carport = []

        self.build_carport()

    def build_carport(self):
        if self.id not in self.graph.cross_car_dict:
            return
        for car_id in self.graph.cross_car_dict[self.id]:
            car = self.graph.car_object[car_id]
            if car.is_priority:
                self.p_carport.append(car_id)
            else:
                self.n_carport.append(car_id)
        # 将两个车库进行排序
        self.p_carport.sort(key=lambda i: (-self.graph.car_object[i].speed, self.graph.car_object[i].planTime, i))
        self.n_carport.sort(key=lambda i: (-self.graph.car_object[i].speed, self.graph.car_object[i].planTime, i))


class Graph:

    def __init__(self, car_data_path, road_data_path, cross_data_path, preset_answer_path, answer_path):
        self.answer_file = open(answer_path, 'w')
        
        self.adjacent_matrix = []
        self.car_data, self.road_data, self.cross_data, self.presetAnswer_data, self.answer_data = self.load_data(
            car_data_path, road_data_path, cross_data_path, preset_answer_path, answer_path)

        self.car_object = {}
        self.cross_object = {}  # 记录所有的路口信息 （路口车库信息）

        self.car_id_list = {car[0]: i for i, car in enumerate(self.car_data)}
        self.road_id_list = {road[0]: i for i,
                             road in enumerate(self.road_data)}
        self.cross_id_list = {cross[0]: i for i,
                              cross in enumerate(self.cross_data)}

        self.num_cars = len(self.car_data)
        self.num_crosses = len(self.cross_data)
        self.num_roads = len(self.road_data)
        self.num_p_cars = 0
        self.car_max_speed = None
        self.car_min_speed = None
        self.p_car_max_speed = None
        self.p_car_min_speed = None
        self.first_car_plan_time = None
        self.last_car_plan_time = None
        self.first_p_car_plan_time = None
        self.last_p_car_plan_time = None
        self.start_distribution = set()
        self.start_p_distribution = set()
        self.end_distribution = set()
        self.end_p_distribution = set()
        
        self.path_matrix = [self.num_crosses * [None]
                            for i in range(self.num_crosses)]
        self.road_car_dict = {}
        self.cross_car_dict = {}

        self.load_car_to_carport()
        self.init_graph()
        self.parameter_A = self.get_parameter_A()

        # # 存放最终结果
        # self.result = []

    def load_data(self, car_data_path, road_data_path, cross_data_path, presetAnswer_data_path, answer_data_path):
        car_data, road_data, cross_data, presetAnswer_data, answer_data = [], [], [], [], []
        for file_name in ['car', 'road', 'cross', 'presetAnswer', 'answer']:
            if file_name == 'answer' and dispatcher.run_type == 'dispatcher':
                continue
            data_file_path = eval(file_name+'_data_path')
            if data_file_path is None:
                continue
            lines = open(data_file_path).read().splitlines()
            for line in lines:
                if len(line) < 3:
                    continue
                if line[0] == '#':
                    continue
                spts = line.strip()[1:-1].split(',')
                tmp = [int(i) for i in spts]
                eval(file_name + "_data").append(tmp)
        return car_data, road_data, cross_data, presetAnswer_data, answer_data

    # 初始化图，读取数据填充二维邻接矩阵 graph_matrix
    def init_graph(self):
        # 邻接矩阵初始化为 大小为路口数*路口数 值为None的二维数组
        self.adjacent_matrix = [self.num_crosses * [None]
                                for i in range(self.num_crosses)]
        # 初始化道路
        self.init_road()
        self.init_cross()
        
    def init_road(self):
        for road_item in self.road_data:
            road_id, length, speed, channel = road_item[0], road_item[1], road_item[2], road_item[3]
            # 路口从1开始，而邻接矩阵从0开始，故 start 和 to 减1
            start, to, isDuplex = self.cross_id_list[road_item[4]
                                                     ], self.cross_id_list[road_item[5]], road_item[6]
            road = Road(road_id, length, speed,
                        channel, start, to, isDuplex, self)
            road.cost = length / (channel)
            self.adjacent_matrix[start][to] = road
            if isDuplex == 1:  # 双向车道，在图中为双向
                self.adjacent_matrix[start][to].direction = 'forward'
                reverse_road = Road(road_id, length, speed,
                                    channel, to, start, isDuplex, self)
                reverse_road.cost = 1 / (channel * length)
                self.adjacent_matrix[to][start] = reverse_road
                self.adjacent_matrix[to][start].direction = 'reverse'

    # 实例化所有Cross对象，返回一个字典
    def init_cross(self):
        for cross_item in self.cross_data:
            self.cross_object[cross_item[0]] = Cross(
                self.cross_id_list[cross_item[0]], cross_item[1], cross_item[2], cross_item[3], cross_item[4], self)

    def load_car_to_carport(self):
        if dispatcher.run_type == 'dispatcher':
            data_list = [self.car_data, self.presetAnswer_data]
        else:
            data_list = [self.presetAnswer_data, self.answer_data]
        
        for i, data in enumerate(data_list):
            for line in data:
                if dispatcher.run_type == 'dispatcher' and i == 0: # 是 car_data
                    car_id, start_time = line[0], 0
                    road_path = None
                    car_data = line
                else:
                    car_id, start_time = line[0], line[1]
                    road_path = [road_id for road_id in line[2:]]
                    car_data = self.car_data[self.car_id_list[car_id]]
                start, to = self.cross_id_list[car_data[1]
                                            ], self.cross_id_list[car_data[2]]
                speed, plan_time, is_priority, is_preset = \
                    car_data[3], car_data[4], car_data[5], car_data[6]
                
                car = Car(car_id, start, to, speed, plan_time,
                        is_priority, is_preset, start_time, self, road_path)
                # car = Car(car_id, start, to, speed, plan_time, 0, 0, start_time, self, road_path)

                self.car_object[car_id] = car
                if road_path is not None: # 预置车辆 (或answer车辆)
                    if road_path[0] not in self.road_car_dict:
                        self.road_car_dict[road_path[0]] = [car_id]
                    else:
                        self.road_car_dict[road_path[0]].append(car_id)
                elif car.is_preset == 0: # 非预置车辆
                    if start not in self.cross_car_dict:
                        self.cross_car_dict[start] = [car_id]
                    else:
                        self.cross_car_dict[start].append(car_id)
                    

                # 记录所有车辆信息
                self.car_max_speed = update_x(self.car_max_speed, speed, 'max')
                self.car_min_speed = update_x(self.car_min_speed, speed, 'min')
                self.first_car_plan_time = update_x(self.first_car_plan_time, plan_time, 'min')
                self.last_car_plan_time = update_x(self.last_car_plan_time, plan_time, 'max')
                self.start_distribution.add(start)
                self.end_distribution.add(to)

                # 记录优先车辆信息
                if is_priority == 1:
                    if dispatcher.run_type == 'dispatcher' and i == 0: # 是 car_data
                        self.num_p_cars += 1
                    self.p_car_max_speed = update_x(self.p_car_max_speed, speed, 'max')
                    self.p_car_min_speed = update_x(self.p_car_min_speed, speed, 'min')
                    self.first_p_car_plan_time = update_x(self.first_p_car_plan_time, plan_time, 'min')
                    self.last_p_car_plan_time = update_x(self.last_p_car_plan_time, plan_time, 'max')
                    self.start_p_distribution.add(start)
                    self.end_p_distribution.add(to)

    def get_parameter_A(self):
        num_factor = round(self.num_cars / self.num_p_cars, 5) * 0.05
        cars_speed_factor = round(self.car_max_speed/self.car_min_speed, 5)
        p_cars_speed_factor = round(self.p_car_max_speed/self.p_car_min_speed, 5)
        speed_factor = round(cars_speed_factor/p_cars_speed_factor, 5) * 0.2375
        cars_plantime_factor = round(self.first_car_plan_time/self.last_car_plan_time, 5)
        p_cars_plantime_factor = round(self.first_p_car_plan_time/self.last_p_car_plan_time, 5)
        plantime_factor = round(cars_plantime_factor/p_cars_plantime_factor, 5) * 0.2375
        start_dist_factor = round(len(self.start_distribution)/len(self.start_p_distribution), 5) * 0.2375
        end_dist_factor = round(len(self.end_distribution)/len(self.end_p_distribution), 5) * 0.2375
        result = num_factor + speed_factor + plantime_factor + start_dist_factor + end_dist_factor
        return round(result, 5)
            
    def show_graph(self):
        # 记录每个路口有多少条路
        num_roads_in_cross = []
        for line in self.adjacent_matrix:
            # 将邻接矩阵的一行映射成 0 或 cost（方便查看），0表示不连通
            temp = []
            count = 0
            for item in line:
                if item is None:
                    temp.append(0)
                else:
                    temp.append(item.cost)
                    count += 1
            num_roads_in_cross.append(count)
            print(temp)
        # print(num_roads_in_cross)
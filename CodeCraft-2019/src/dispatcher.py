import gamemap
import copy
import time
import dijkstra
import math

run_type = 'dispatcher'


end_car_list = []  # 记录到达终点的车辆
end_p_car_list = [] # 记录到达终点的优先车辆
on_road_car_list = []  # 记录在路上的车
p_car_dispatch_time = None
test = []

class Dispatcher:

    def __init__(self, graph):
        self.graph = graph
        self.time_slice = 0  # 时间片
        self.is_dead_lock = True  # 死锁flag

    def get_enter_cars_num(self):
        max_num_cars_on_graph = 3500
        num_cars_enter = (max_num_cars_on_graph - len(on_road_car_list)) / self.graph.num_crosses
        num_cars_enter = math.floor(num_cars_enter)
        return num_cars_enter

    def choose_enter_cars(self):
        # 遍历所有路口的优先车库，把所有优先车放进道路车库
        for cross_id in sorted(list(self.graph.cross_id_list.keys())):
            carport = self.graph.cross_object[cross_id].p_carport
            while len(carport) > 0:
                car_id = carport[0]
                car = self.graph.car_object[car_id]
                car.update_path_info()
                # if car.planTime > 100 or car.speed < 4:
                #     test.append(car.id)
                car.start_time = car.planTime * 5
                start = car.shorest_path_cross[0]
                to = car.shorest_path_cross[1]
                self.update_cost(car, 'add')
                road = self.graph.adjacent_matrix[start][to]
                road.carport.append(car.id)
                road.carport.sort(key=lambda i: (
                    -self.graph.car_object[i].is_priority, self.graph.car_object[i].start_time, i))
                self.graph.cross_object[cross_id].p_carport.remove(carport[0])
                
                # 写入result
                write_string = '({},{}'.format(car.id, car.start_time)
                for road_id in car.shorest_path_road:
                    write_string = write_string + ',' + str(road_id)
                write_string = write_string + ')' + '\n'
                self.graph.answer_file.write(write_string)

        if self.time_slice < 300:
            return
        time_wait = 20
        num_cars = self.get_enter_cars_num()
        if self.time_slice % time_wait != 1:
            return
        for cross_id in self.graph.cross_id_list.keys():
            carport = self.graph.cross_object[cross_id].n_carport
            for i in range(num_cars):
                if len(carport) < 1:
                    break
                car_id = carport[0]
                car = self.graph.car_object[car_id]
                car.update_path_info()
                start = car.shorest_path_cross[0]
                to = car.shorest_path_cross[1]
                car.start_time = self.time_slice
                self.update_cost(car, 'add')
                road = self.graph.adjacent_matrix[start][to]
                road.carport.append(car.id)
                road.carport.sort(key=lambda i: (
                    -self.graph.car_object[i].is_priority, self.graph.car_object[i].start_time, i))
                self.graph.cross_object[cross_id].n_carport.remove(carport[0])
                
                # 写入result
                write_string = '({},{}'.format(car.id, car.start_time)
                for road_id in car.shorest_path_road:
                    write_string = write_string + ',' + str(road_id)
                write_string = write_string + ')' + '\n'
                self.graph.answer_file.write(write_string)

    def run(self):
        global run_type
        run_type = 'dispatcher'
        while len(end_car_list) < self.graph.num_cars:
            self.time_slice += 1
            for car_id in on_road_car_list:
                car = self.graph.car_object[car_id]
                car.state = 'wait'
            self.choose_enter_cars()
            self.first_round_dispatch()
            self.all_road_cars_enter(only_priority=True)
            self.second_round_dispatch()
            self.all_road_cars_enter()

            print('time_slice {}: {} cars enter roads, {} cars on roads, {} cars reach end'
                  .format(self.time_slice, len(on_road_car_list)+len(end_car_list),
                          len(on_road_car_list), len(end_car_list)))
            #print(test)
                          
        print('dispatch finished!')
        print('priority_car_num: {}'.format(self.graph.num_p_cars))
        print('total time slice: {}'.format(self.time_slice))
        print('priority cars time slice: {}'.format(p_car_dispatch_time))
        print('final time slice: {}'.format(math.floor(self.time_slice + p_car_dispatch_time * self.graph.parameter_A)))


    # 输入result.txt 跑判题器
    def run_judger(self):
        global run_type
        run_type = 'judger'

        # car_list.sort()

        # 开始调度
        while len(end_car_list) < self.graph.num_cars:
            for car_id in on_road_car_list:
                car = self.graph.car_object[car_id]
                car.state = 'wait'
            self.first_round_dispatch()
            self.all_road_cars_enter(only_priority=True)
            self.second_round_dispatch()
            self.all_road_cars_enter()

            self.path_matrix = [self.graph.num_crosses * [None] for i in range(self.graph.num_crosses)]

            print('time_slice {}: {} cars enter roads, {} cars on roads, {} cars reach end'
                  .format(self.time_slice, len(on_road_car_list)+len(end_car_list),
                          len(on_road_car_list), len(end_car_list)))
            self.time_slice += 1
        self.time_slice -= 1  # 最后一轮多加了1
        print('judger finished!')
        print('total time slice: {}'.format(self.time_slice))
        print('priority cars time slice: {}'.format(p_car_dispatch_time))
        print('final time slice: {}'.format(math.floor(self.time_slice + p_car_dispatch_time * self.graph.parameter_A)))


    # 道路上车辆的第一轮调度：调度不通过路口的车辆
    def first_round_dispatch(self):
        for cross_i in range(self.graph.num_crosses):  # 每个路口
            for road in self.graph.adjacent_matrix[cross_i]:  # 每条道路
                if road is not None:
                    for channel_i in range(road.num_channel):  # 每个车道
                        for i in range(road.length):  # 车道的每个位置
                            position = road.length - i - 1  # 从出车口开始
                            car_id = road.channel_list[channel_i][position]
                            if car_id is not None:  # 这是一辆车
                                car = self.graph.car_object[car_id]
                                self.dispatch_car_on_channel(car)


    # 道路上车辆的第二轮调度：调度路口车辆
    def second_round_dispatch(self):
        if self.is_all_car_stop():
            return
        
        cross_index_dispatched_list = [v[1] for v in sorted(
            self.graph.cross_id_list.items(), key=lambda d:d[0])]  # id升序排列将要调度的路口
        while(len(cross_index_dispatched_list) > 0): # 所有路口遍历完才结束
            cross_index_not_dispatched = []  # 记录调度未完成的路口
            self.is_dead_lock = True

            for cross_i in cross_index_dispatched_list:
                roads_in_cross = [
                    roads[cross_i] for roads in self.graph.adjacent_matrix if roads[cross_i] is not None]
                road_dispatch_list = [v for v in sorted(
                    roads_in_cross, key=lambda road:road.id)]  # id升序排列将要调度的路

                # 遍历路口还未调度过的所有路
                self.road_not_dispatched = set()  # 记录调度未完成的路
                for road in road_dispatch_list:
                    while 1:
                        first_priority_car_id = road.get_first_priority_car_id()
                        if first_priority_car_id is None:
                            # continue
                            break
                        else:
                            car = self.graph.car_object[first_priority_car_id]

                            # 若发生冲突 终止该道路调度 开始下一道路
                            if self.is_conflict(car):
                                # print('conflict!')
                                self.road_not_dispatched.add(road)
                                # continue
                                break
                            # 未发生冲突
                            else:
                                # 检查下条路是否有前车阻挡 以及 前车状态
                                if car.is_reach_end():  # 该车将在这条路到达终点
                                    self.reach_end(car)
                                    self.dispatch_channel(
                                            car.current_channel, car.current_position - 1)
                                    car.current_road.cars_enter_road(time_slice=self.time_slice, only_priority=True)
                                else:
                                    next_road = car.next_road
                                    next_channel, _ = next_road.get_available_channel(
                                        car)
                                    t_dist_next_road = car.get_t_dist_next_road()
                                    if t_dist_next_road <= 0 or next_channel is None:  # 如果 可行驶距离无法驶入下一车道 或 下一个车道满了
                                        self.move_car(
                                            car.current_channel, car, car.current_road.length - 1)
                                        # 一辆车终止后 调度整个车道
                                        self.dispatch_channel(
                                            car.current_channel, car.current_position - 1)

                                        car.current_road.cars_enter_road(time_slice=self.time_slice, only_priority=True)
                                        # self.priority_cars_enter_road()        
                                    else:
                                        obstacle_car_id = get_first_obstacle_front(
                                            next_channel, -1, t_dist_next_road)  # -1 表示该车还未进入车道
                                        if obstacle_car_id is None:  # 下条路无车阻挡，直接移动
                                            self.move_car(
                                                next_channel, car, t_dist_next_road - 1)
                                        else:
                                            obstacle_car = self.graph.car_object[obstacle_car_id]
                                            if obstacle_car.state == 'stop':  # 下条路的阻挡前车状态为停止，该车移动并停止
                                                self.move_car(
                                                    next_channel, car, obstacle_car.current_position - 1)
                                            elif obstacle_car.state == 'wait':  # 下条路的阻挡前车状态为等待，该车也等待
                                                car.wait_car()
                                                self.road_not_dispatched.add(road)
                                                # continue
                                                break
                            # first_priority_car_id = road.get_first_priority_car_id()
                            # if first_priority_car_id is not None:
                            #     self.road_not_dispatched.add(road)
                if len(self.road_not_dispatched) > 0:
                    cross_index_not_dispatched.append(cross_i)

            cross_index_dispatched_list = cross_index_not_dispatched
            if self.is_dead_lock:
                print('dead_lock in {}'.format(cross_index_not_dispatched))
                exit(0)

    def dispatch_channel(self, channel, start_position=None):
        if start_position is None:
            start_position = len(channel) - 1
        for i in range(start_position + 1):
            position = start_position - i
            car_id = channel[position]
            if car_id is not None:  # 这是一辆车
                car = self.graph.car_object[car_id]
                if car.state == 'wait':
                    self.dispatch_car_on_channel(car)
                    if car.state == 'wait':  # 该车依然走不了 等下一轮调度
                        return
                else:
                    return

    def dispatch_car_on_channel(self, car):
        channel = car.current_channel
        position = car.current_position
        road = car.current_road

        # 得到可行驶距离
        travelable_distance = min(
            car.speed, road.speed)

        # 获取可行驶距离内阻挡该车的第一辆前车
        obstacle_car_id = get_first_obstacle_front(
            channel, position, travelable_distance)

        # 有车 检查车辆状态
        if obstacle_car_id is not None:
            obstacle_car = self.graph.car_object[obstacle_car_id]
            if obstacle_car.state == 'wait':
                car.wait_car()

            elif obstacle_car.state == 'stop':
                self.move_car(
                    channel, car, obstacle_car.current_position - 1)
        # 无车
        else:
            position_after_move = position + travelable_distance
            # 能行驶过路口
            if position_after_move >= road.length:
                car.wait_car()
            # 不能行驶过路口
            else:
                self.move_car(
                    channel, car, position_after_move)

    def move_car(self, next_channel, car, position_move_to):
        if position_move_to < 0 or position_move_to >= len(next_channel):
            print('position {} out of range'.format(position_move_to))
            exit(0)
        last_channel = car.current_channel  # 记录上一个channel 用于后面调度
        last_position = car.current_position
        last_road = car.current_road
        self.is_dead_lock = False

        # 更新车道中的信息
        last_channel[last_position] = None
        next_channel[position_move_to] = car.id
        car.current_position = position_move_to

        # 改变车辆状态
        car.state = 'stop'
        if car.current_channel != next_channel:  # 若穿过路口到达下一路

            # 车辆通过路口 更新车辆信息
            car.update_info_by_cp(next_channel, position_move_to)

            # 一辆车过路口后 调度整个车道
            self.dispatch_channel(last_channel)

            last_road.cars_enter_road(time_slice=self.time_slice, only_priority=True)

    def is_conflict(self, car):
        # 获取目标路口的所有路
        road_list = []
        roads_in_cross = [roads[car.current_cross_to]
                          for roads in self.graph.adjacent_matrix]
        for road in roads_in_cross:
            if road is not None and road != car.current_road and road != car.next_road:
                road_list.append(road)

        for road in road_list:
            first_priority_car_id = road.get_first_priority_car_id()  # 车道第一优先级车辆
            if first_priority_car_id is not None:  # 这是一辆车
                first_priority_car = self.graph.car_object[first_priority_car_id]
                #　判断状态是否为等待
                if first_priority_car.state == 'wait':
                    # 判断前进道路是否相同
                    if car.next_cross == first_priority_car.next_cross:
                        # 判断车辆优先级是否更高:
                        if car.is_priority < first_priority_car.is_priority:
                            return True
                        elif car.is_priority > first_priority_car.is_priority:
                            return False
                        # 判断方向优先级是否更高 (在优先级相同的情况下判断)
                        car_priority = car.get_direction_priority()
                        fp_car_priority = first_priority_car.get_direction_priority()
                        if fp_car_priority > car_priority:
                            return True
        return False

    def is_all_car_stop(self):
        num_stop_cars = 0
        for car_id in on_road_car_list:
            car_state = self.graph.car_object[car_id].state
            if car_state == 'stop':
                num_stop_cars += 1
        if num_stop_cars == len(on_road_car_list):
            return True
        return False

    def reach_end(self, car):
        # 更新车道信息
        car.current_channel[car.current_position] = None

        # 将车标记为到达终点
        end_car_list.append(car.id)
        on_road_car_list.remove(car.id)
        car.state = None
        self.is_dead_lock = False

        if car.is_priority == 1:
            end_p_car_list.append(car.id)

        global p_car_dispatch_time
        if p_car_dispatch_time is None and len(end_p_car_list) == self.graph.num_p_cars:
            p_car_dispatch_time = self.time_slice - self.graph.first_p_car_plan_time

        # 更新邻接矩阵cost
        # self.update_cost(car, method='sub')

    def update_cost(self, car, method):
        cost_increment = 0.5
        for cross_i in range(len(car.shorest_path_cross)-1):
            cross_start = car.shorest_path_cross[cross_i]
            cross_end = car.shorest_path_cross[cross_i+1]
            if method == 'add':
                # self.graph.adjacent_matrix[cross_start][cross_end].cost += cost_increment
                road = self.graph.adjacent_matrix[cross_start][cross_end]
                road.cost += (1 / road.num_channel)
            else:
                self.graph.adjacent_matrix[cross_start][cross_end].cost -= cost_increment


    # def cars_enter_road(self, is_priority=0):
    #     # 之前time_slice未能进入的车辆先进入
    #     for car_id in copy.copy(self.car_not_enter_list[is_priority]):
    #         car = self.graph.car_object[car_id]
    #         car.update_info(is_init=True)
    #         if car.current_channel is None or car.current_position is None:
    #             continue
    #         else:
    #             on_road_car_list.append(car_id)
    #             self.car_not_enter_list[is_priority].remove(car_id)

    #     for cross_id in sorted(self.cross_object.keys()):
    #         if self.time_slice in self.cross_object[cross_id].carport_cars:
    #             for car_id in sorted(self.cross_object[cross_id].carport_cars[self.time_slice][is_priority]):
    #                 car = self.graph.car_object[car_id]
    #                 car.update_info(is_init=True)
    #                 if car.current_channel is None or car.current_position is None:  # 若这个时间片无法上车道 则下一个时间片发车
    #                     print('no channel!')
    #                     self.car_not_enter_list[car.is_priority].append(car_id)
    #                     self.cross_object[cross_id].carport_cars[self.time_slice][is_priority].remove(
    #                         car_id)
    #                     continue
    #                 on_road_car_list.append(car_id)
    #                 self.cross_object[cross_id].carport_cars[self.time_slice][is_priority].remove(
    #                     car_id)

    def all_road_cars_enter(self, only_priority=False):
        for cross_i in range(self.graph.num_crosses):
            for road in self.graph.adjacent_matrix[cross_i]:
                if road is not None:
                    road.cars_enter_road(time_slice=self.time_slice, only_priority=only_priority)
    
def get_first_obstacle_front(channel, position, travelable_distance, is_reversed=False):
    start_position = position + 1
    end_position = position + travelable_distance

    if end_position > len(channel):
        end_position = len(channel)

    # 遍历该位置后一个到末尾的车道，判断每个位置是否有车
    channel = channel[start_position: end_position + 1]
    if is_reversed:
        channel = reversed(channel)
    for car_id in channel:
        if car_id is not None:
            return car_id
    return None

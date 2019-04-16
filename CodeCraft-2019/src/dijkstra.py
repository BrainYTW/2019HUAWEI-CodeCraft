import sys

class Dijkstra:
    def __init__(self, graph):
        self.graph = graph

    # 输入源点路口id，终点路口id，返回源点到终点最短路径，保存在path中
    def run(self, start_cross, end_cross=None):
        # d 保存各点到源点的最短距离
        # parent 保存一个结点到源点的路径上的 上一个结点（-1表示路径上已经没有结点）

        self.d = []
        
        for road in self.graph.adjacent_matrix[start_cross]:
            if road is not None:
                self.d.append(road.cost)
            else:
                self.d.append(sys.maxsize)
        
        self.parent = [-1 if self.d[i] ==
                       sys.maxsize else start_cross for i in range(self.graph.num_crosses)]

        # visited_list 存放已访问过的结点
        visited_list = []
        visited_list.append(start_cross)

        # 循环访问所有的结点
        while len(visited_list) < self.graph.num_crosses:

            # 找到ｄ中当前 未访问过的 最小的 结点
            min_dist = sys.maxsize
            min_index = start_cross
            for i, i_dist in enumerate(self.d):
                if i not in visited_list and min_dist > i_dist:
                    min_dist = i_dist
                    min_index = i

            # 通过上面找到的结点对所有边进行松弛
            for i, i_dist in enumerate(self.d):
                if self.graph.adjacent_matrix[min_index][i] is not None:
                    new_dist = self.d[min_index] + \
                        self.graph.adjacent_matrix[min_index][i].cost

                    # 松弛操作
                    if i not in visited_list and i_dist > new_dist:
                        self.d[i] = new_dist
                        # 如果松弛成功，即最短路径需要通过该结点，将该结点设为被松弛结点的父节点
                        self.parent[i] = min_index

            # 将该结点加入已访问过的结点list
            visited_list.append(min_index)
        
        # 构建从源点到所有路口的路径
        for cross in range(self.graph.num_crosses):
            self.buildShortestPath(start_cross, cross)


    # 根据终点获取最短路径
    def buildShortestPath(self, start_cross, end_cross):
        cross_path = []
        road_path = []
        index = end_cross
        # 将终点插入cross_path
        cross_path.insert(0, index)
        # print(self.parent)
        # 从终点开始访问结点的父节点，直到起点（起点的父节点为-1）
        while not self.parent[index] == -1:
            cross_path.insert(0, self.parent[index])  # 每次从头部插入
            index = self.parent[index]
        for i in range(len(cross_path) - 1):
            row_index = cross_path[i]
            col_index = cross_path[i+1]
            road = self.graph.adjacent_matrix[row_index][col_index]
            if road is not None:
                road_path.append(road.id)
        self.graph.path_matrix[start_cross][end_cross] = [cross_path, road_path]

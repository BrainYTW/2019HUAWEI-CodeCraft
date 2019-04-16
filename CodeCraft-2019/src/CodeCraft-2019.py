import os
import logging
import sys
import time
import dijkstra
import gamemap
import dispatcher

# logging.basicConfig(level=logging.DEBUG,
#                     filename='../logs/CodeCraft-2019.log',
#                     format='[%(asctime)s] %(levelname)s [%(funcName)s: %(filename)s, %(lineno)d] %(message)s',
#                     datefmt='%Y-%m-%d %H:%M:%S',
#                     filemode='a')


def main():
    # if len(sys.argv) != 6:
    #     # logging.info('please input args: car_path, road_path, cross_path, answerPath')
    #     exit(1)

    # car_path = sys.argv[1]
    # road_path = sys.argv[2]
    # cross_path = sys.argv[3]
    # preset_answer_path = sys.argv[4]
    # answer_path = sys.argv[5]

    # logging.info("car_path is %s" % (car_path))
    # logging.info("road_path is %s" % (road_path))
    # logging.info("cross_path is %s" % (cross_path))
    # logging.info("answer_path is %s" % (answer_path))

    # car_path = r"CodeCraft-2019/config_r2/car.txt"
    # road_path = r"CodeCraft-2019/config_r2/road.txt"
    # cross_path = r"CodeCraft-2019/config_r2/cross.txt"
    # preset_answer_path = r"CodeCraft-2019/config_r2/presetAnswer.txt"
    # answer_path = r"CodeCraft-2019/config_r2/answer.txt"

    car_path = r"../2-map-exam-1/car.txt"
    road_path = r"../2-map-exam-1/road.txt"
    cross_path = r"../2-map-exam-1/cross.txt"
    preset_answer_path = r"../2-map-exam-1/presetAnswer.txt"
    answer_path = r"../2-map-exam-1/answer.txt"

    dispatcher.run_type = 'dispatcher'
    # initilize graph
    graph = gamemap.Graph(car_path, road_path, cross_path, preset_answer_path, answer_path)

    # start to dispatch
    dper = dispatcher.Dispatcher(graph)
    dper.run()
    # dper.run_judger()
    
    time_write = time.time()
    
    # to write output file
    # for result_item in graph.result:
    #     write_string = '({},{}'.format(result_item[0], result_item[1])
    #     for road in result_item[2:]:
    #         write_string = write_string + ',' + str(road)
    #     write_string = write_string + ')' + '\n'
    #     write_file.write(write_string)
    # print("write time: {}s".format(time.time()-time_write))

if __name__ == "__main__":
    time_begin = time.time()
    main()
    print("total time: {}s".format(time.time()-time_begin))
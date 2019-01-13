import argparse
import os
import time
from heuristic import Solver


def main():
    parser = argparse.ArgumentParser(
        description='Optimization of public transport garage schedule.')
    parser.add_argument('inputfile', metavar='input_file',
                        help='Name of input file stored in data/ folder')
    args = parser.parse_args()

    instance = args.inputfile[8]
    instance_data = load_instance(args.inputfile)
    solver = Solver(*instance_data)

    print('Initial solution')
    print('First global goal:', solver.global_goal_first(solver.initial_solution))
    print('Second global goal:', solver.global_goal_second(solver.initial_solution))
    print('Fitness function:', solver.fitness_func(solver.initial_solution))
    print('Unscheduled vehicles count:', len(solver.initial_solution.unscheduled_vehicles))
    is_valid, message = solver.is_valid(solver.initial_solution)
    print('Is valid:', is_valid)
    print(message)
    print(solver.initial_solution)
    # write formatted result into output file
    write_result(str(solver.initial_solution), 'initial', instance)

    # neighbourhood = solver.generate_neighbourhood(solver.initial_solution, 1)
    # print_neighbourhood(neighbourhood)
    start = time.time()
    taboo_best_solution = solver.taboo_search(50, 1, 2)
    end = time.time()
    print()
    print('Taboo solution')
    print('Code execution: ', end - start)
    print('Unscheduled vehicles count:', len(taboo_best_solution.unscheduled_vehicles))
    print('Is taboo valid: ', solver.is_valid(taboo_best_solution)) 
    print('First global goal:', solver.global_goal_first(taboo_best_solution))
    print('Second global goal:', solver.global_goal_second(taboo_best_solution))   
    print('Taboo fitness function:', solver.fitness_func(taboo_best_solution))
    print(taboo_best_solution)
    
    write_result(str(taboo_best_solution), '5m', instance)

def print_neighbourhood(neighbourhood):
    i = 0
    print('Neighbourhood:')
    for solution in neighbourhood:
        print('Neighbour', i+1)
        print(solution)
        i+=1

def load_instance(filename):
    file_path = 'data/{}'.format(filename)
    with open(file_path, 'r') as f:
        try:
            vehicle_count = int(f.readline())
            track_count = int(f.readline())
            f.readline()

            vehicle_lengths = [int(x) for x in f.readline().strip().split(' ')]
            f.readline()

            vehicle_series = [int(x) for x in f.readline().strip().split(' ')]
            f.readline()

            vehicle_restrictions = []
            for _ in range(vehicle_count):
                vehicle_restriction = [True if x == '1' else False
                                       for x in f.readline().strip().split(' ')]
                vehicle_restrictions.append(vehicle_restriction)
            f.readline()

            track_lengths = [int(x) for x in f.readline().strip().split(' ')]
            f.readline()

            departure_times = [int(x) for x in f.readline().strip().split(' ')]
            f.readline()

            schedule_type = [int(x) for x in f.readline().strip().split(' ')]
            f.readline()

            blocking_tracks = {}
            for line in f.readlines():
                blocking_record = [int(x) for x in line.strip().split(' ')]
                blocking_tracks[blocking_record[0]] = blocking_record[1:]

            return (
                vehicle_count, track_count, vehicle_lengths, vehicle_series, vehicle_restrictions,
                track_lengths, departure_times, schedule_type, blocking_tracks
            )
        except ValueError:
            raise Exception('Instance input file is incorrectly formatted!')


def write_result(result_string, time, instance):
    filename = 'res-{}-i{}.txt'.format(time, instance)
    file_path = 'output/{}'.format(filename)

    directory_name = os.path.join(os.getcwd(), 'output')
    if not os.path.isdir(directory_name):
        os.mkdir(directory_name)

    with open(file_path, 'w+') as f:
        f.write(result_string)


if __name__ == "__main__":
    main()

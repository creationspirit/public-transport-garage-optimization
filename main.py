import argparse

from heuristic import Solver


def main():
    parser = argparse.ArgumentParser(
        description='Optimization of public transport garage schedule.')
    parser.add_argument('inputfile', metavar='input_file',
                        help='Name of input file stored in data/ folder')
    args = parser.parse_args()

    instance_data = load_instance(args.inputfile)
    solver = Solver(*instance_data)
    print(solver.generate_initial_solution())
    print(solver.global_goal_first())
    print(solver.global_goal_second())
    print(solver.fitness_func())
    # print(solver.track_length_sum)
    # print(solver.vehicle_length_sum)
    # print(list(solver.blocking_tracks.keys()))
    # print(solver.nonblocking_tracks)
    # print(solver.vehicle_restrictions[:, 25])


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
    with open(file_path, 'w') as f:
        f.write(result_string)


if __name__ == "__main__":
    main()

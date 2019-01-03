class Solver:
    def __init__(self, vehicle_count, track_count, vehicle_lengths, vehicle_series,
                 vehicle_restrictions, track_lengths, departure_times,
                 schedule_type, blocking_tracks):

        #########################
        # Load of instance data #
        #########################
        self.vehicle_count = vehicle_count
        self.track_count = track_count
        self.vehicle_lengths = vehicle_lengths
        self.vehicle_series = vehicle_series
        self.vehicle_restrictions = vehicle_restrictions
        self.track_lengths = track_lengths
        self.departure_times = departure_times
        self.schedule_type = schedule_type
        self.blocking_tracks = blocking_tracks

        ##################################
        # Initialize solution attributes #
        ##################################

        # List notes which series of vehicle is parked on which track
        self.series_on_track = [None] * self.track_count
        self.used_tracks_count = 0

        # List of empty space left in each track
        self.unused_track_capacity = self.track_lengths.copy()

        self.track_length_sum = sum(self.track_lengths)
        self.vehicle_length_sum = sum(self.vehicle_lengths)

        # list of tracks where every element is list of vehicles assigned to track
        self.schedule_result = [[] for _ in range(self.track_count)]

    def global_goal_first(self):
        # first subfunction
        f_1 = 0
        temp_first = None
        for first, second in zip(self.series_on_track, self.series_on_track[1:]):
            if first is not None and second is not None and first != second:
                f_1 += 1
            elif first is not None and second is None:
                temp_first = first
            elif first is None and second is not None and temp_first:
                if temp_first != second:
                    f_1 += 1
                temp_first = None
        p_1 = 1.0 / (self.used_tracks_count - 1)

        # second subfunction
        f_2 = self.used_tracks_count
        p_2 = 1.0 / self.track_count

        # third subfunction
        f_3 = 0
        for used, leftover in zip(self.series_on_track, self.unused_track_capacity):
            if used is not None:
                f_3 += leftover
        p_3 = 1.0 / (self.track_length_sum - self.vehicle_length_sum)

        return (p_1 * f_1) + (p_2 * f_2) + (p_3 * f_3)

    def global_goal_second(self):
        # first subfunction
        g_1 = 0
        for track_schedule in self.schedule_result:
            if len(track_schedule) > 1:
                for first, second in zip(track_schedule, track_schedule[1:]):
                    if self.schedule_type[first] == self.schedule_type[second]:
                        g_1 += 1
        r_1 = 1.0 / (self.vehicle_count - self.used_tracks_count)

        # second subfunction
        g_2 = 0
        temp_first = None
        for first, second in zip(self.schedule_result, self.schedule_result[1:]):
            if (len(first) != 0 and len(second) != 0 and
                    self.schedule_type(first[-1]) == self.schedule_type(second[0])):
                g_2 += 1
            elif len(first) != 0 and len(second) == 0:
                temp_first = first[-1]
            elif len(first) == 0 and len(second) != 0 and temp_first:
                if self.schedule_type(temp_first) == self.schedule_type(second[0]):
                    g_2 += 1
                temp_first = None
        r_2 = 1.0 / (self.used_tracks_count - 1)

        # third subfunction
        g_3 = 0
        pair_counter = 0
        for track_schedule in self.schedule_result:
            if len(track_schedule) > 1:
                for first, second in zip(track_schedule, track_schedule[1:]):
                    g_3 += self.__get_vehicle_departure_gap_factor(first, second)
                    pair_counter += 1
        r_3 = 1.0 / (15 * pair_counter)

        return (r_1 * g_1) + (r_2 * g_2) + (r_3 * g_3)

    def __get_vehicle_departure_gap_factor(self, vehicle_1, vehicle_2):
        deprature_diff = self.departure_times[vehicle_2] - self.departure_times[vehicle_1]
        if deprature_diff >= 10 or deprature_diff <= 20:
            return 15
        elif deprature_diff > 20:
            return 10
        else:
            return -4 * (10 - deprature_diff)

    def fitness_func(self):
        return self.global_goal_second() + (1.0 / self.global_goal_first())

    def generate_initial_solution(self):
        pass

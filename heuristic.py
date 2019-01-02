class Solver:
    def __init__(self, vehicle_count, track_count, vehicle_lengths, vehicle_series,
                 vehicle_restrictions, track_lengths, departure_times,
                 schedule_type, blocking_tracks):

        ############################
        # Loading of instance data #
        ############################
        self.vehicle_count = vehicle_count
        self.track_count = track_count
        self.vehicle_lengths = vehicle_lengths
        self.vehicle_series = vehicle_series
        self.vehicle_restrictions = vehicle_restrictions
        self.track_lengths = track_lengths
        self.departure_times = departure_times
        self.schedule_type = schedule_type
        self.blocking_tracks = blocking_tracks

        #########################################
        # Initialization of solution attributes #
        #########################################

        # List notes which series of vehicle is parked on which track
        self.series_on_track = [None] * self.track_count
        self.used_tracks_count = 0

        # List of empty space left in each track
        self.unused_track_capacity = self.track_lengths.copy()

    def global_fitness_first(self):

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

        return (p_1 * f_1) + (p_2 * f_2)

    def global_fitness_second(self):
        pass

    def fitness_func(self):
        return self.global_fitness_second() + (1.0 / self.global_fitness_first())

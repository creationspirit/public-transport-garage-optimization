import numpy as np
import random
import copy

class Solver:
    def __init__(self, vehicle_count, track_count, vehicle_lengths, vehicle_series,
                 vehicle_restrictions, track_lengths, departure_times,
                 schedule_type, blocking_tracks):

        #########################
        # Load instance data #
        #########################
        self.vehicle_count = vehicle_count
        self.track_count = track_count
        self.vehicle_lengths = vehicle_lengths
        self.vehicle_series = vehicle_series
        self.vehicle_restrictions = np.array(vehicle_restrictions)
        self.track_lengths = track_lengths
        self.departure_times = departure_times
        self.schedule_type = schedule_type
        self.blocking_tracks = blocking_tracks

        self.track_length_sum = sum(self.track_lengths)
        self.vehicle_length_sum = sum(self.vehicle_lengths)

        # tracks that are not blocking and are not blocked
        blocked = [item - 1 for sublist in self.blocking_tracks.values() for item in sublist]
        blocking = [item - 1 for item in self.blocking_tracks.keys()]
        self.nonblocking_tracks = [t for t in list(range(self.track_count))
                                   if t not in blocked + blocking]

        # initial solution
        self.initial_solution = self.generate_initial_solution()

    def global_goal_first(self, solution):
        # first subfunction
        f_1 = 0
        temp_first = None
        for first, second in zip(solution.series_on_track, solution.series_on_track[1:]):
            if first is not None and second is not None and first != second:
                f_1 += 1
            elif first is not None and second is None:
                temp_first = first
            elif first is None and second is not None and temp_first:
                if temp_first != second:
                    f_1 += 1
                temp_first = None
        p_1 = 1.0 / (solution.used_tracks_count - 1)

        # second subfunction
        f_2 = solution.used_tracks_count
        p_2 = 1.0 / self.track_count

        # third subfunction
        f_3 = 0
        for used, leftover in zip(solution.series_on_track, solution.unused_track_capacity):
            if used is not None:
                f_3 += leftover
        p_3 = 1.0 / (self.track_length_sum - self.vehicle_length_sum)

        return (p_1 * f_1) + (p_2 * f_2) + (p_3 * f_3)

    def global_goal_second(self, solution):
        # first subfunction
        g_1 = 0
        for track_schedule in solution.schedule:
            if len(track_schedule) > 1:
                for first, second in zip(track_schedule, track_schedule[1:]):
                    if self.schedule_type[first] == self.schedule_type[second]:
                        g_1 += 1
        r_1 = 1.0 / (self.vehicle_count - solution.used_tracks_count)

        # second subfunction
        g_2 = 0
        temp_first = None
        for first, second in zip(solution.schedule, solution.schedule[1:]):
            if (len(first) != 0 and len(second) != 0 and
                    self.schedule_type[first[-1]] == self.schedule_type[second[0]]):
                g_2 += 1
            elif len(first) != 0 and len(second) == 0:
                temp_first = first[-1]
            elif len(first) == 0 and len(second) != 0 and temp_first:
                if self.schedule_type[temp_first] == self.schedule_type[second[0]]:
                    g_2 += 1
                temp_first = None
        r_2 = 1.0 / (solution.used_tracks_count - 1)

        # third subfunction
        g_3 = 0
        pair_counter = 0
        for track_schedule in solution.schedule:
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

    def fitness_func(self, solution):
        return self.global_goal_second(solution) / self.global_goal_first(solution)

    def generate_initial_solution(self):
        s = Solution(self.track_count, self.track_lengths)

        # generate vehicle list and sort it by departure time (this is priority!)
        vehicles = list(range(self.vehicle_count))
        vehicles_sorted = self.__sort_by_departure_time(vehicles)

        tracks = list(range(self.track_count))

        for vehicle in vehicles_sorted:
            track_availability = self.vehicle_restrictions[vehicle]
            # Get all tracks that have current vehicle series assigned to them
            # and find the best one that vehicle fits in if it exists
            assigned_tracks = [t for t in tracks
                               if s.series_on_track[t] == self.vehicle_series[vehicle]]
            best_capacity = None
            best_track = None
            for t in assigned_tracks:
                if not track_availability[t]:
                    continue
                # check for blocking tracks constraint validation
                blocked_tracks = self.blocking_tracks.get(t + 1)
                if blocked_tracks:
                    invalid_flag = False
                    for bt in blocked_tracks:
                        if len(s.schedule[bt - 1]) > 0:
                            if (self.departure_times[s.schedule[bt - 1][0]] <
                                    self.departure_times[vehicle]):
                                invalid_flag = True
                    if invalid_flag:
                        continue

                new_capacity = s.unused_track_capacity[t] - self.vehicle_lengths[vehicle] - 0.5
                if new_capacity < 0:
                    continue
                elif best_capacity is None or best_capacity > new_capacity:
                    best_capacity = new_capacity
                    best_track = t

            if best_track is not None:
                s.unused_track_capacity[best_track] = best_capacity
                s.schedule[best_track].append(vehicle)

            # no track in assigned track was found, need to assign new track to this vehicle series
            else:
                # generate list of tracks that current vehicle can park on
                available_tracks = [t for t in tracks
                                    if track_availability[t] and s.series_on_track[t] is None]
                # get blocking and non blocking tracks first, if there is no such track left,
                # only then take from blocked tracks
                blocking_tracks = [t - 1 for t in self.blocking_tracks.keys()]
                usable_tracks = [t for t in available_tracks
                                 if t in self.nonblocking_tracks + blocking_tracks]
                if len(usable_tracks) == 0:
                    usable_tracks = available_tracks
                # find track that can store smallest number of vehicle series and use that one
                # this prioritizes less flexible tracks for vehicles that can go into them
                best_can_hold_types = self.vehicle_count + 1
                best_track = None
                for t in usable_tracks:
                    can_hold_types = list(self.vehicle_restrictions[:, t]).count(True)
                    if (can_hold_types < best_can_hold_types and
                            self.vehicle_lengths[vehicle] <= self.track_lengths[t]):
                        best_can_hold_types = can_hold_types
                        best_track = t

                if best_track is not None:
                    s.unused_track_capacity[best_track] -= self.vehicle_lengths[vehicle]
                    s.series_on_track[best_track] = self.vehicle_series[vehicle]
                    s.used_tracks_count += 1
                    s.schedule[best_track].append(vehicle)
                else:
                    s.unscheduled_vehicles.add(vehicle)

        return s

    def __sort_by_departure_time(self, target_list):
        zipped_pairs = zip(self.departure_times, target_list)
        z = [x for _, x in sorted(zipped_pairs)]
        return z

    def is_valid(self, solution):
        """This function checks if solution respects all of constraints."""
        tracks = list(range(self.track_count))
        for track, track_index in zip(solution.schedule, tracks):
            if len(track) > 1:
                for first, second in zip(track, track[1:]):
                        if self.departure_times[first] > self.departure_times[second]:
                            return (False,
                                    'Vehicle {} departs later than vehicle {}!'.format(first + 1,
                                                                                       second + 1))
                        if self.vehicle_series[first] != self.vehicle_series[second]:
                            return (False,
                                    'Vehicle {} is not same series as vehicle {}!'.format(first + 1,
                                                                                          second + 1))
            for vehicle in track:
                if not self.vehicle_restrictions[vehicle][track_index]:
                    return (False,
                            'Vehicle {} is restricted to park on track {}!'.format(vehicle + 1,
                                                                                   track_index + 1))
            if solution.unused_track_capacity[track_index] < 0:
                return (False,
                        'Track {} is over its capacity!'.format(track_index + 1))
        for blocking_track in self.blocking_tracks.keys():
            for blocked_track in self.blocking_tracks[blocking_track]:
                blocking_schedule = solution.schedule[blocking_track - 1]
                blocked_schedule = solution.schedule[blocked_track - 1]
                if len(blocked_schedule) > 0 and len(blocking_schedule) > 0:
                    if (self.departure_times[blocking_schedule[-1]] >
                            self.departure_times[blocked_schedule[0]]):
                        #print(self.departure_times[blocking_schedule[-1]])
                        #print(self.departure_times[blocked_schedule[0]])
                        return (False,
                                'First vehicle in blocked track {} departs sooner than last vehicle in blocking track {}'.format(blocked_track,
                                                                                                                                 blocking_track))
        return (True, '')

    def generate_neighbourhood(self, initial_solution, neighbourhood_length):
        neighbourhood = set()

        while len(neighbourhood) < neighbourhood_length:
            s = copy.deepcopy(initial_solution)
            
            if len(self.initial_solution.unscheduled_vehicles) > 0:
                # add unscheduled vehicles to scheduled
                s.schedule.append(list(s.unscheduled_vehicles))

            tracks_count = len(s.schedule)

            # find random track
            selected_track1_index = random.randrange(tracks_count)
            # randomly find track and cannot choose two empty tracks
            selected_track2_index = random.randrange(tracks_count)
            while len(s.schedule[selected_track1_index]) == 0 and len(s.schedule[selected_track2_index]) == 0:
                selected_track2_index = random.randrange(tracks_count)            

            selected_track2_count = len(s.schedule[selected_track2_index]) 
            selected_track1_count = len(s.schedule[selected_track1_index])
            if selected_track1_count > 0 and selected_track2_count > 0:
                
                selected_vehicle1_index = random.randrange(selected_track1_count)
                selected_vehicle2_index = random.randrange(selected_track2_count)

                # swap vehicles
                tmp = s.schedule[selected_track1_index][selected_vehicle1_index]
                s.schedule[selected_track1_index][selected_vehicle1_index] = s.schedule[selected_track2_index][selected_vehicle2_index] 
                s.schedule[selected_track2_index][selected_vehicle2_index] = tmp
            elif selected_track1_count == 0:
                # first track is empty
                selected_vehicle2_index = random.randrange(selected_track2_count)
                selected_vehicle2 = s.schedule[selected_track2_index].pop(selected_vehicle2_index)
                s.schedule[selected_track1_index].append(selected_vehicle2) 
            elif selected_track2_count == 0:
                # second track is empty
                selected_vehicle1_index = random.randrange(selected_track1_count)
                selected_vehicle1 = s.schedule[selected_track1_index].pop(selected_vehicle1_index)
                s.schedule[selected_track2_index].append(selected_vehicle1)
            # update solution
            # remove unscheduled vehicles
            if len(s.unscheduled_vehicles) > 0:
                unscheduled_vehicles =  set(s.schedule.pop())
                s.unscheduled_vehicles = unscheduled_vehicles

            s.used_tracks_count = self.count_used_tracks(s)
            s.series_on_track = self.initialize_series_on_track(s)
            s.unused_track_capacity = self.update_unused_track_capacity(s)
            if self.is_valid(s)[0] != False and s.schedule != self.initial_solution.schedule:
                neighbourhood.add(s)

        return neighbourhood

    def count_used_tracks(self, solution):
        count = 0
        for track in solution.schedule:
            if len(track) != 0:
                count += 1
        return count

    def initialize_series_on_track(self, solution):
        series_on_track = [None] * self.track_count
        for i in range(0, self.track_count):
            if (len(solution.schedule[i]) > 0):
                series_on_track[i] = self.vehicle_series[solution.schedule[i][0]]
        return series_on_track

    def update_unused_track_capacity(self, solution):
        track_lengths = self.track_lengths.copy()
        unused_tracks_capacity = []
        for track, unused_track in zip(solution.schedule, track_lengths):
            for vehicle in track:
                unused_track -= self.vehicle_lengths[vehicle]
            unused_tracks_capacity.append(unused_track)
        return unused_tracks_capacity

    def taboo_search(self, taboo_duration, iterations, neighbourhood_length):
        taboo_list = []
        best_solution = self.initial_solution

        current_iteration = 0
        while current_iteration < iterations:
            
            neighbourhood = self.generate_neighbourhood(best_solution, neighbourhood_length)

            for neighbour in neighbourhood:
                # print('test: ', self.fitness_func(neighbour))
                if not (neighbour in taboo_list) and self.fitness_func(best_solution) < self.fitness_func(neighbour):
                    best_solution = neighbour
                    taboo_list.insert(0, neighbour)
                    if taboo_duration == len(taboo_list):
                        taboo_list.pop()    
            current_iteration += 1
        return best_solution


class Solution:
    def __init__(self, track_count, track_lengths):

        ##################################
        # Initialize solution attributes #
        ##################################

        # List notes which series of vehicle is parked on which track
        self.series_on_track = [None] * track_count
        self.used_tracks_count = 0

        # List of empty space left in each track
        self.unused_track_capacity = track_lengths.copy()

        # Result data structure - list of tracks where every element is list of vehicles
        self.schedule = [[] for _ in range(track_count)]

        # Set of vehicles that couldn't fit anywhere
        self.unscheduled_vehicles = set()

    def __str__(self):
        string_schedule = []
        for track in self.schedule:
            string_schedule.append(' '.join([str(v + 1) for v in track]) if len(track) > 0 else '')
        return '\n'.join(string_schedule)
        
    def __eq__(self, other):
        if isinstance(other, Solution):
            return ((self.schedule == other.schedule) and (self.unscheduled_vehicles == other.unscheduled_vehicles) and (self.series_on_track == other.series_on_track)
            and self.used_tracks_count == other.used_tracks_count and self.unused_track_capacity == other.unused_track_capacity)
        else:
            return False
    def __hash__(self):
        return hash((tuple(self.series_on_track), self.used_tracks_count,(tuple(val) for val in self.schedule), tuple(self.unused_track_capacity), tuple(self.unscheduled_vehicles)))
    pass

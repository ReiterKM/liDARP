from util import *

# nodeset: assignment from physical station to DARP station => nodeset[DARP station] = physical station

# TODO: seperate Requests into Request class, make travelRequest a collection of Request?

class TravelRequests:
    def _initialize(self):
        self.generate_directional_requests()

    def read_requests(self, requests: REQUEST_LIST_TYPE, num_stations: int) -> None:
        self.num_stations = num_stations
        self.requests = requests
        self.num_requests = len(requests)
        self._initialize()

    def _transform_line(self, line: str, int_idx: List = []) -> List:
        values = line.strip().split()
        values = [float(v) for v in values]
        for i in int_idx:
            values[i] = int(values[i])
        return values

    def _read_input_file(self, infile: str):
        file = open(infile, "r")
        first_line = file.readline()

        overall_service_time = 10**6

        num_vehicles, self.num_requests, self.num_stations, max_time, \
            vehicle_capacity, alpha, beta = self._transform_line(first_line, [0, 1, 2, 4])
        raw_requests = [[0, 0, None, None] for _ in range(self.num_requests)]

        count = 0

        for line in file.readlines():
            idx, origin, destination, service_time, demand, \
                earliest_start_time, latest_arrival_time = self._transform_line(line, [0,1,2,4])

            if idx > self.num_requests:
                raise ValueError(
                    "More requests read from file than indicated.")
            
            if (service_time < 0):
                raise ValueError("Invalid service time.")
            else:
                overall_service_time = min(overall_service_time, service_time)

            if (origin < 0) or (origin > self.num_stations):
                raise ValueError(
                    "Origin of request {0} is out of range at location {1}".format(idx, origin))
            if (destination < 0) or (destination > self.num_stations):
                raise ValueError(
                    "Destination of request {0} is out of range at location {1}".format(idx, destination))
            if demand > 1:
                raise ValueError("Demand > 1 is not yet implemented.")

            raw_requests[idx][ORIGIN_IDX] = origin
            raw_requests[idx][DESTINATION_IDX] = destination

            if not isvalid(earliest_start_time) and not isvalid(latest_arrival_time):
                raise ValueError(
                    "Request {0} does not have any time information.".format(idx))

            if earliest_start_time < 0:
                raise ValueError("Start time of request {0} is before time 0.".format(idx))
            else:
                raw_requests[idx][EARLIEST_START_TIME_IDX] = earliest_start_time

            if (latest_arrival_time < 0):
                raise ValueError(
                    "End time of request {0} is after the max time.".format(idx))
            else:
                raw_requests[idx][LATEST_ARRIVAL_TIME_IDX] = latest_arrival_time                

            count += 1

        if count != self.num_requests:
            raise ValueError("Less requests read from file than indicated.")

        file.close()

        self.requests = raw_requests
        self._initialize()

        return num_vehicles, max_time, vehicle_capacity, self.num_stations, overall_service_time, alpha, beta

    def read_file(self, instance_file: str, consider_shortcuts: bool, station_location_file: str = None, distance_matrix_file: str = None) -> List[int]:
        output = self._read_input_file(infile=instance_file)
        
        if all([station_location_file, distance_matrix_file]):
            raise ValueError("Too many distance inputs given.")

        if (station_location_file is not None):
            self._read_station_locations(infile=station_location_file, consider_shortcuts=consider_shortcuts)
        elif (distance_matrix_file is not None):
            self._read_distance_matrix(infile=distance_matrix_file)
        else:
            self.generate_line_distances()
        
        if self.distances.sum() == 0:
            raise SyntaxError("Distances could not be initialized.")

        return output

    def generate_line_distances(self):
        # assuming stations are on a 1D line with distance 1 between each station
        stations = {i : (i, 0) for i in range(1, self.num_stations+1)}
        self._generate_euclidean_line_distances(stations, consider_shortcuts=False)

    def _generate_euclidean_line_distances(self, locs: dict, consider_shortcuts:bool):
        self.distances = np.zeros(shape = (self.num_stations+1, self.num_stations+1))
        self.direct_distances = np.zeros(shape = self.num_requests)
        for i in range(1,self.num_stations+1):
            for j in range(1,self.num_stations+1):#
                if (i == j):
                    self.distances[i][j] == 0
                else:
                    if consider_shortcuts:
                        self.distances[i][j] = math.sqrt((locs[j][0] - locs[i][0])**2 + (locs[j][1] - locs[i][1])**2)
                    else:
                        if abs(i - j) == 1:
                            self.distances[i][j] = math.sqrt((locs[j][0] - locs[i][0])**2 + (locs[j][1] - locs[i][1])**2)
                        else:
                            dist = 0
                            start = min(i,j)
                            end = max(i,j)
                            for k in range(start, end):
                                dist += math.sqrt((locs[k+1][0] - locs[k][0])**2 + (locs[k+1][1] - locs[k][1])**2)
                            self.distances[i][j] = dist
        for idx, r in enumerate(self.requests):
            i = r[ORIGIN_IDX]
            j = r[DESTINATION_IDX]
            self.direct_distances[idx] = math.sqrt((locs[j][0] - locs[i][0])**2 + (locs[j][1] - locs[i][1])**2)

    def _read_station_locations(self, infile: str, consider_shortcuts: bool):
        station_location = {}
        i = 0
        with open(infile, "r+") as file:
            for line in file:
                id, x, y = self._transform_line(line, int_idx = [0])
                station_location[id] = (x,y)
                i += 1
        if i < self.num_stations:
            raise ValueError("Number of stations in distance file is not sufficient for request file.")
        self._generate_euclidean_line_distances(station_location, consider_shortcuts)

    def _read_distance_matrix(self, infile:str):
        self.distances = np.zeros(shape = (self.num_stations+1, self.num_stations+1))
        self.direct_distances = np.zeros(shape = (self.num_requests))
        i = 1
        with open(infile, "r+") as file:
            for line in file:
                self.distances[i] = [0] + self._transform_line(line=line)
                i += 1
        if i != self.num_stations + 1:
            raise ValueError("Not enough distances input, please check matrix.")
        for idx, r in enumerate(self.requests):
            self.direct_distances[idx] = self.distances[r[ORIGIN_IDX]][r[DESTINATION_IDX]]

    def generate_time_windows(self, travel_speed: float, boarding_time: float,
                              max_time: float,
                              alpha: float, beta: float) -> List:
        time_windows = []
        for request in self.requests:
            service_promise_made = [False, False]  # pick-up, drop-off
            origin, destination, earliest_pick_up, latest_drop_off = request
            direct_travel_time = self.distances[origin][destination] * travel_speed
            max_travel_time = alpha * direct_travel_time

            if isvalid(earliest_pick_up) and not isvalid(latest_drop_off):
                # inbound requests
                service_promise_made[0] = True

                latest_pick_up = earliest_pick_up + beta
                earliest_drop_off = max(0, earliest_pick_up + boarding_time + direct_travel_time)
                latest_drop_off = min(max_time, latest_pick_up + max_travel_time + boarding_time)
                
            elif not isvalid(earliest_pick_up) and isvalid(latest_drop_off):
                # outbound requests
                service_promise_made[1] = True

                earliest_drop_off = latest_drop_off - beta
                earliest_pick_up = max(0, earliest_drop_off - max_travel_time - boarding_time)
                latest_pick_up = min(max_time, latest_drop_off - direct_travel_time - boarding_time)
                
            else:
                # no waiting time promise made for these passengers
                latest_pick_up = latest_drop_off - direct_travel_time - boarding_time
                earliest_drop_off = earliest_pick_up + direct_travel_time + boarding_time

            if earliest_pick_up > max_time:
                raise ValueError(
                    "Earliest pick up time is past the time frame.")
            if earliest_drop_off > max_time:
                raise ValueError(
                    "Earliest drop off time is past the time frame.")

            request_origin_time_window = [earliest_pick_up, latest_pick_up]
            request_destination_time_window = [
                earliest_drop_off, latest_drop_off]
            time_windows.append(
                [request_origin_time_window, request_destination_time_window, service_promise_made])
        return time_windows

    def generate_directional_requests(self):
        self.asc_requests = []
        self.desc_requests = []

        for idx, r in enumerate(self.requests):
            start = r[ORIGIN_IDX]
            end = r[DESTINATION_IDX]
            if start < end:
                self.asc_requests.append(idx)
            elif start > end:
                self.desc_requests.append(idx)
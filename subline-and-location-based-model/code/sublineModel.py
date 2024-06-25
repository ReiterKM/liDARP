from util import *
from model import Model

class SublineModel(Model):
    def __init__(self, boarding_time = 3, *args, **kwargs):
        self.boarding_time = boarding_time
        self.name = "subline-based"

        super(SublineModel, self).__init__(*args, **kwargs)

    def _initModel(self): 
        return gp.Model(self.name)
    
    def _calculateConstants(self):
        self.R_asc = self.requests.asc_requests
        self.R_desc = self.requests.desc_requests

        self.num_requests = self.requests.num_requests

        self.origins = [r[ORIGIN_IDX] for r in self.requests.requests]
        self.destinations = [r[DESTINATION_IDX] for r in self.requests.requests]

        self.H = range(1, self.num_stations + 1)
        self.K = range(self.num_busses)
        self.R = range(self.num_requests)

        self.b = {r : self.boarding_time for r in self.R}

        self.turn_edges = [(i,i) for i in self.H]
        self.drive_edges = list(itertools.permutations(self.H, 2))
        self.edges = self.drive_edges + self.turn_edges

        self.asc_edges = [(i,j) for (i,j) in self.drive_edges if i < j]
        self.desc_edges = [(i,j) for (i,j) in self.drive_edges if i > j]
        
        self.c = self.requests.distances
        self.c_direct = self.requests.direct_distances
        self.t = self.c * self.speed
        np.fill_diagonal(self.t, self.t_turn)
        
        # time windows
        self._calculate_time_windows()

        self.num_services = 2 * self.num_requests

        self.S = range(self.num_services)
        self.S_asc = range(0,self.num_services,2)
        self.S_desc = range(1,self.num_services,2)

        # max. time window
        self.T = max(max(self.drop_off_time_windows_per_request.values())) + (self.num_services - 1) * self.t_turn

        # big M
        self.big_M = self.T + (2 * self.num_services - 1) * self.boarding_time
        self.big_M_pickup = self.max_travel_minutes

        self.origin_requests_to_be_serviced_before_at_origin, self.destination_requests_to_be_serviced_before_at_origin,\
              self.origin_requests_to_be_serviced_before_at_destination, self.destination_requests_to_be_serviced_before_at_destination = self._find_requests_sharing_stations()

    def _find_requests_sharing_stations(self):
        origin_requests_to_be_serviced_before_at_origin = {}
        destination_requests_to_be_serviced_before_at_origin = {}
        origin_requests_to_be_serviced_before_at_destination = {}
        destination_requests_to_be_serviced_before_at_destination = {}


        # find all requests that need to be serviced *before* the current request r
        # if two requests have the same  e.g. latest arrival time, we need to break the tie to determine who waits for whom
        # this is done by 1. earliest departure time, then 2. request number
        for request_set in [self.R_asc, self.R_desc]:
            for r in request_set:
                origin_requests_to_be_serviced_before_at_origin[r] = []
                destination_requests_to_be_serviced_before_at_origin[r] = []
                origin_requests_to_be_serviced_before_at_destination[r] = []
                destination_requests_to_be_serviced_before_at_destination[r] = []

                for p in request_set:
                    if r == p: continue
                    # same origin
                    if self.origins[r] == self.origins[p]:
                        # same origin, different time
                        if self.pickup_time_windows_per_request[r][0] > self.pickup_time_windows_per_request[p][0]:
                            origin_requests_to_be_serviced_before_at_origin[r].append(p)
                        # same origin, same time
                        elif self.pickup_time_windows_per_request[r][0] == self.pickup_time_windows_per_request[p][0]:
                            if self.drop_off_time_windows_per_request[r][1] > self.drop_off_time_windows_per_request[p][1]:
                                origin_requests_to_be_serviced_before_at_origin[r].append(p)
                            # check tie breaker
                            elif self.drop_off_time_windows_per_request[r][1] == self.drop_off_time_windows_per_request[p][1]:
                                if r > p:
                                    origin_requests_to_be_serviced_before_at_destination[r].append(p)
                    # same destination
                    if self.destinations[r] == self.destinations[p]:
                        # same destination, different time
                        if self.pickup_time_windows_per_request[r][0] > self.pickup_time_windows_per_request[p][0]:
                            destination_requests_to_be_serviced_before_at_destination[r].append(p)
                        # same destination, same time
                        elif self.pickup_time_windows_per_request[r][0] == self.pickup_time_windows_per_request[p][0]:
                            if self.drop_off_time_windows_per_request[r][1] > self.drop_off_time_windows_per_request[p][1]:
                                destination_requests_to_be_serviced_before_at_destination[r].append(p)
                            elif self.drop_off_time_windows_per_request[r][1] == self.drop_off_time_windows_per_request[p][1]:
                                if r > p:
                                    destination_requests_to_be_serviced_before_at_destination[r].append(p)
                    # origin of r == destination of p
                    if self.origins[r] == self.destinations[p]:
                        destination_requests_to_be_serviced_before_at_origin[r].append(p)
        return origin_requests_to_be_serviced_before_at_origin, destination_requests_to_be_serviced_before_at_origin, origin_requests_to_be_serviced_before_at_destination, destination_requests_to_be_serviced_before_at_destination

    def _calculate_time_windows(self):
        self.pickup_time_windows_per_request = {}
        self.drop_off_time_windows_per_request = {}
        self.service_promises = {}

        time_windows = self.requests.generate_time_windows(travel_speed = self.speed, boarding_time = self.boarding_time,
                                                                max_time = self.max_travel_minutes, alpha = self.alpha, beta = self.beta)

        for idx, (origin_time_window, destination_time_window, service_promise) in enumerate(time_windows):
            self.pickup_time_windows_per_request[idx] = origin_time_window
            self.drop_off_time_windows_per_request[idx] = destination_time_window
            self.service_promises[idx] = service_promise

    def createModel(self, obj_weights: List, **kwargs):
        start_time = datetime.datetime.now()
        self.objWeights = obj_weights

        # define variables
        self.y = self.model.addVars(self.H, self.S, self.K, vtype=GRB.BINARY, name = "y")
        self.start_node = self.model.addVars(self.H, self.K, vtype=GRB.BINARY, name = "start")
        self.end_node = self.model.addVars(self.H, self.K, vtype=GRB.BINARY, name = "end")
        self.x = self.model.addVars(self.edges, self.S, self.K, vtype=GRB.BINARY, name = "x")
        self.z = self.model.addVars(self.K, vtype=GRB.BINARY, name="z") # indicator variable
        self.w_asc = self.model.addVars(self.R_asc, self.R_asc, self.S_asc, self.K, vtype = GRB.BINARY, name = "w_asc")
        self.w_desc = self.model.addVars(self.R_desc, self.R_desc, self.S_desc, self.K, vtype = GRB.BINARY, name = "w_desc")
        self.depTime = self.model.addVars(self.H, self.S, self.K, vtype=GRB.CONTINUOUS, name="depTime", lb = 0, ub=self.T)
        self.arrTime = self.model.addVars(self.H, self.S, self.K, vtype=GRB.CONTINUOUS, name = "arrTime", lb = 0, ub=self.T)
        self.assign_asc = self.model.addVars(self.R_asc, self.S_asc, self.K, vtype = GRB.BINARY, name = "assign_asc")
        self.assign_desc = self.model.addVars(self.R_desc, self.S_desc, self.K, vtype = GRB.BINARY, name = "assign_desc")
        self.pickupTime = self.model.addVars(self.R, vtype = GRB.CONTINUOUS, name = "pickupTime", lb = 0, ub=self.T)
        self.dropoffTime = self.model.addVars(self.R, vtype = GRB.CONTINUOUS, name = "dropoffTime", lb = 0, ub=self.T)

        self._setObjective()

        for k in self.K:
            self.model.addConstr(gp.quicksum(self.start_node[i,k] for i in self.H) <= 1, name="use_each_bus_max_once") # each bus used max. once
            self.model.addConstr(gp.quicksum(self.end_node[i,k] for i in self.H) == gp.quicksum(self.start_node[i,k] for i in self.H), name="bus_origin_=_destination") # each bus ends if it is started
            self.model.addConstr(self.z[k] == gp.quicksum(self.start_node[i,k] for i in self.H), name="define_z") # define variable z
            if k < self.num_busses - 1:
                self.model.addConstr(self.z[k+1] <= self.z[k], name="symm_break_smaller_buses_first") # symmetry breaking: use smaller numbered busses first

            for s in self.S_asc:
                for (i,j) in self.desc_edges:
                    self.model.addConstr(self.x[i,j,s,k] == 0, name="S_asc_flow") # ensure S_asc flows left to right

                for r in self.R_asc:
                    self.model.addConstr(2 * self.assign_asc[r,s,k] <= self.y[self.origins[r],s,k] + self.y[self.destinations[r],s,k], name="assign_if_pickup_and_dropoff") # assignment only if pax is picked up and and dropped off
                    self.model.addConstr(self.assign_asc[r,s,k] * (self.pickup_time_windows_per_request[r][0] + self.b[r]) <= self.depTime[self.origins[r],s,k], name="pickup_after_o.e") # pick-up after earliest pick-up time
                    self.model.addConstr(self.drop_off_time_windows_per_request[r][1] + self.T * (1-self.assign_asc[r,s,k]) >= self.arrTime[self.destinations[r],s,k], name="dropoff_before_d.l") # drop off before latest drop-off time

                    # service requests which have to be picked up or dropped off before the current request, at its origin
                    for p in self.origin_requests_to_be_serviced_before_at_origin[r]:
                        self.model.addConstr(self.b[p] + self.pickupTime[p] - self.pickupTime[r] <= self.big_M_pickup *(1 - gp.quicksum(self.w_asc[p, r, s, k] for s in self.S_asc for k in self.K)), name="order_of_pickup_service_at_origin")
                    for p in self.destination_requests_to_be_serviced_before_at_origin[r]:
                        self.model.addConstr(self.b[p] + self.dropoffTime[p] - self.pickupTime[r] <= self.big_M_pickup *(1 - gp.quicksum(self.w_asc[p, r, s, k] for s in self.S_asc for k in self.K)), name="order_of_dropoff_service_at_origin")

                    # service requests which have to be picked up or dropped off before the current request, at its destination
                    for p in self.destination_requests_to_be_serviced_before_at_destination[r]:
                        self.model.addConstr(self.b[p] + self.dropoffTime[p] - self.dropoffTime[r] <= self.big_M_pickup * (1 - gp.quicksum(self.w_asc[p, r, s, k] for s in self.S_asc for k in self.K)), name="order_of_dropoff_at_destination") # de-boarding time for all earlier passengers
                    for p in self.origin_requests_to_be_serviced_before_at_destination[r]:
                        self.model.addConstr(self.b[p] + self.pickupTime[p] - self.dropoffTime[r]  <= self.big_M_pickup * (1 - gp.quicksum(self.w_asc[p, r, s, k] for s in self.S_asc for k in self.K)), name="order_of_pickup_at_destination")
                    
                    self.model.addConstr(self.depTime[self.origins[r], s, k] >= self.assign_asc[r,s,k] * (self.pickupTime[r] + self.b[r]), name="asc_boarding_time") # boarding time added for each pax.
                    self.model.addConstr(self.depTime[self.destinations[r], s, k] >= self.assign_asc[r,s,k] * (self.dropoffTime[r] + self.b[r]), name="asc_de-boarding_time") # de-boarding time added for each pax.

                    for r2 in self.R_asc:
                        if r == r2: continue
                        self.model.addConstr(self.w_asc[r, r2, s, k] <= self.assign_asc[r,s,k], name="requests_share_service_1") # indicator variable if two requests are on the same bus service
                        self.model.addConstr(self.assign_asc[r,s,k] + self.assign_asc[r2,s,k] - 1 <= self.w_asc[r, r2, s, k], name="requests_share_service_2")

                for i in self.H[:-1]:
                    relevant_asc_requests = [r for r in self.R_asc if (self.origins[r] <= i) and (self.destinations[r] > i)]
                    self.model.addConstr(gp.quicksum(self.assign_asc[r,s,k] for r in relevant_asc_requests) <= self.Q_max, name="capacity_restriction") # capacity restriction

            for s in self.S_desc:
                for (i,j) in self.asc_edges:
                    self.model.addConstr(self.x[i,j,s,k] == 0, name="S_desc_flow") #ensure S_desc flow right to left
                for r in self.R_desc:
                    self.model.addConstr(2 * self.assign_desc[r,s,k] <= self.y[self.origins[r],s,k] + self.y[self.destinations[r],s,k], name="assign_if_pickup_and_dropoff") # assignment only if pax is picked up and and dropped off
                    self.model.addConstr(self.assign_desc[r,s,k] * (self.pickup_time_windows_per_request[r][0] + self.b[r]) <= self.depTime[self.origins[r],s,k], name="pickup_after_o.e") # pick-up after earliest pick-up time plus get-on-time
                    self.model.addConstr(self.drop_off_time_windows_per_request[r][1] + self.T * (1-self.assign_desc[r,s,k]) >= self.arrTime[self.destinations[r],s,k], name="dropoff_before_d.l") # drop off before latest drop-off time
                    
                    # service requests which have to be picked up or dropped off before the current request, at its origin
                    for p in self.origin_requests_to_be_serviced_before_at_origin[r]:
                        self.model.addConstr(self.b[p] + self.pickupTime[p] - self.pickupTime[r] <= self.big_M_pickup *(1 - gp.quicksum(self.w_desc[p, r, s, k] for s in self.S_desc for k in self.K)), name="order_of_pickup_service_at_origin")
                    for p in self.destination_requests_to_be_serviced_before_at_origin[r]:
                        self.model.addConstr(self.b[p] + self.dropoffTime[p] - self.pickupTime[r] <= self.big_M_pickup *(1 - gp.quicksum(self.w_desc[p, r, s, k] for s in self.S_desc for k in self.K)), name="order_of_dropoff_service_at_origin")

                    # service requests which have to be picked up or dropped off before the current request, at its destination
                    for p in self.destination_requests_to_be_serviced_before_at_destination[r]:
                        self.model.addConstr(self.b[p] + self.dropoffTime[p] - self.dropoffTime[r] <= self.big_M_pickup * (1 - gp.quicksum(self.w_desc[p, r, s, k] for s in self.S_desc for k in self.K)), name="order_of_dropoff_at_destination") # de-boarding time for all earlier passengers
                    for p in self.origin_requests_to_be_serviced_before_at_destination[r]:
                        self.model.addConstr(self.b[p] + self.pickupTime[p] - self.dropoffTime[r]  <= self.big_M_pickup * (1 - gp.quicksum(self.w_desc[p, r, s, k] for s in self.S_desc for k in self.K)), name="order_of_pickup_at_destination")

                    self.model.addConstr(self.depTime[self.origins[r], s, k] >= self.assign_desc[r,s,k] * (self.pickupTime[r] + self.b[r]), name="desc_boarding_time") # boarding time added for each pax.
                    self.model.addConstr(self.depTime[self.destinations[r], s, k] >= self.assign_desc[r,s,k] * (self.dropoffTime[r] + self.b[r]), name="desc_de-boarding_time") # de-boarding time added for each pax.

                    for r2 in self.R_desc:
                        if r == r2: continue
                        self.model.addConstr(self.w_desc[r, r2, s, k] <= self.assign_desc[r,s,k], name="requests_share_service_1") # indicator variable if two requests are on the same bus service
                        self.model.addConstr(self.assign_desc[r,s,k] + self.assign_desc[r2,s,k] - 1 <= self.w_desc[r, r2, s, k], name="requests_share_service_2")

                for i in self.H[1:]:
                    relevant_desc_requests = [r for r in self.R_desc if (self.origins[r] >= i) and (self.destinations[r] < i)]
                    self.model.addConstr(gp.quicksum(self.assign_desc[r,s,k] for r in relevant_desc_requests) <= self.Q_max, name="capacity_restriction") # capacity restriction

            for s in self.S_asc:
                for (i,j) in self.asc_edges:
                    self.model.addConstr(self.arrTime[j,s,k] >= self.depTime[i,s,k] + self.t[(i,j)] * self.x[i,j,s,k], name="arrival_time_consistency") # arrival time consistency

            for s in self.S_desc:
                for (i,j) in self.desc_edges:
                    self.model.addConstr(self.arrTime[j,s,k] >= self.depTime[i,s,k] + self.t[(i,j)] * self.x[i,j,s,k], name="arrival_time_consistency") # arrival time consistency

            for s in self.S[:-1]:
                self.model.addConstr(gp.quicksum(self.x[i,i,s,k] for i in self.H) == gp.quicksum(self.start_node[i,k] for i in self.H), name="turn_only_if_started") # ensure all turning busses have been started

            for (i,j) in self.edges:
                self.model.addConstr(self.z[k] >= self.x[i,j,s,k], name="drive_only_if_used") # vehicle cannot drive if not in use

            for i in self.H:
                stations_desc_from_i = [j for j in self.H if j < i]
                stations_asc_from_i = [j for j in self.H if j > i]
                

                self.model.addConstr(self.x[i,i,self.S[-1],k] == 0, name="last_bus_does_not_turn") # last bus does not turn
                self.model.addConstr(self.y[i,0,k] == self.start_node[i,k] + gp.quicksum(self.x[j,i,0,k] for j in stations_desc_from_i), name="first_service_has_predecessor_or_is_start_node") # each stop on the first service has predecessor or is start_node
                self.model.addConstr(self.y[i,self.S[-1],k] == self.end_node[i,k] + gp.quicksum(self.x[i,j,self.S[-1],k] for j in stations_desc_from_i), name="last_service_ends_at_end") # last service ends at end_node

                for s in self.S:
                    self.model.addConstr(self.depTime[i,s,k] >= self.arrTime[i,s,k], name="departure_time_consistency") # departure time consistency
                    self.model.addConstr(self.z[k] >= self.y[i,s,k], name="no_stop_if_unused") # vehicle cannot stop if not in use
                    self.model.addConstr(self.big_M * self.z[k] >= self.arrTime[i,s,k], name="no_arrival_if_unused") # vehicle cannot arrive if not in use
                    self.model.addConstr(self.big_M * self.z[k] >= self.depTime[i,s,k], name="no_depature_if_unused") # vehicle cannot depart if not in use


                for s in self.S[1:]:
                    self.model.addConstr(1 + self.end_node[i,k] >= self.x[i,i,s,k] + self.x[i,i,s-1,k], name="strengthen_end_after_2_turns") # strengthening: end node is placed after two turns
                    next_services_except_last = [t for t in self.S[:-1] if t > s]
                    for t in next_services_except_last:
                        self.model.addConstr(1 + self.x[i,i,t,k] >= self.x[i,i,s,k] + self.x[i,i,s-1,k], name="strengthen_park_after_2_turns") # strengthening: bus stays parked after two turns     

                for s in self.S_asc:
                    self.model.addConstr(self.y[i,s,k] == self.x[i,i,s,k] + gp.quicksum(self.x[i,j,s,k] for j in stations_asc_from_i), name="services_have_successor_or_turn") # subsequent service have successor or turn

                for s in self.S_asc[1:]:
                    self.model.addConstr(self.y[i,s,k] == self.x[i,i,s-1,k] + gp.quicksum(self.x[j,i,s,k] for j in stations_desc_from_i), name="services_have_predecessor_or_turn") # subsequent services have predecessor or prev. turns
                    self.model.addConstr(self.arrTime[i,s,k] >= self.depTime[i,s-1,k] + self.t_turn * self.x[i,i,s-1,k], name="arrival_time_of_new_service") # arrival time of new service

                for s in self.S_desc:
                    self.model.addConstr(self.y[i,s,k] == self.x[i,i,s-1,k] + gp.quicksum(self.x[j,i,s,k] for j in stations_asc_from_i), name="services_have_predecesor_or_turn") # subsequent services have predecessor or prev. turns
                    self.model.addConstr(self.arrTime[i,s,k] >= self.depTime[i,s-1,k] + self.t_turn * self.x[i,i,s-1,k],name="arrival_time_of_new_service") # arrival time of new service

                for s in self.S_desc[:-1]:
                    self.model.addConstr(self.y[i,s,k] == self.x[i,i,s,k] + gp.quicksum(self.x[i,j,s,k] for j in stations_desc_from_i), name="services_have_successor_or_turn") # subsequent services have successor or turn

        for r in self.R_asc:
            self.model.addConstr(gp.quicksum(self.assign_asc[r,s,k] for s in self.S_asc for k in self.K) <= 1, name="pax_picked_up_max_once") # passengers picked up max. once
            self.model.addConstr(self.dropoffTime[r] - self.b[r] - self.pickupTime[r] <= self.alpha * self.t[self.origins[r], self.destinations[r]], name="max_travel_time") # travel time service promise
            self.model.addConstr(self.pickupTime[r] >= gp.quicksum(self.assign_asc[r,s,k] * self.arrTime[self.origins[r], s,k] for s in self.S_asc for k in self.K), name="define_pickupTime") # define help variable
            self.model.addConstr(self.dropoffTime[r] >= gp.quicksum(self.assign_asc[r,s,k] * self.arrTime[self.destinations[r], s,k] for s in self.S_asc for k in self.K), name="define_dropoffTime") # define help variable
            for s in self.S_asc:
                self.model.addConstr(self.z[k] >= self.assign_asc[r,s,k], name="no_pickup_if_unused") # vehicle cannot pick up passengers if not in use

        for r in self.R_desc:
            self.model.addConstr(gp.quicksum(self.assign_desc[r,s,k] for s in self.S_desc for k in self.K) <= 1, name="pax_picked_up_max_once") # passengers picked up max. once
            self.model.addConstr(self.dropoffTime[r] - self.b[r] - self.pickupTime[r] <= self.alpha * self.t[self.origins[r], self.destinations[r]], name="max_travel_time") # travel time service promise
            self.model.addConstr(self.pickupTime[r] >= gp.quicksum(self.assign_desc[r,s,k] * self.arrTime[self.origins[r], s,k] for s in self.S_desc for k in self.K), name="define_pickupTime") # define help variable
            self.model.addConstr(self.dropoffTime[r] >= gp.quicksum(self.assign_desc[r,s,k] * self.arrTime[self.destinations[r], s,k] for s in self.S_desc for k in self.K), name="define_dropoffTime") # define help variable
            for s in self.S_desc:
                self.model.addConstr(self.z[k] >= self.assign_desc[r,s,k], name="no_pickup_if_unused") # vehicle cannot pick up passengers if not in use

        for r in self.R:
            # time constraints
            self.model.addConstr(self.pickupTime[r] >= self.pickup_time_windows_per_request[r][0], name="lb_on_pickup")
            self.model.addConstr(self.pickupTime[r] <= self.pickup_time_windows_per_request[r][1], name="ub_on_pickup")
            self.model.addConstr(self.dropoffTime[r] >= self.drop_off_time_windows_per_request[r][0], name="lb_on_dropoff")
            self.model.addConstr(self.dropoffTime[r] <= self.drop_off_time_windows_per_request[r][1], name="ub_on_dropoff")

        for i in self.H:
            prev_stations = [j for j in self.H if j < i]
            for k in self.K[:-1]:
                self.model.addConstr(self.start_node[i,k] >= gp.quicksum(self.start_node[j,k+1] for j in prev_stations), name="symm_breaking:smaller_vehicles_start_earlier") # symmetry breaking: smaller vehicles start at smaller stations
    
        end_time = datetime.datetime.now()
        self.Buildtime = (end_time - start_time).total_seconds()

    def _setObjective(self):
        self.num_pax_accepted =  gp.quicksum(self.assign_asc[r,s,k] for r in self.R_asc for s in self.S_asc for k in self.K) + \
                        gp.quicksum(self.assign_desc[r,s,k] for r in self.R_desc for s in self.S_desc for k in self.K)
        
        self.pax_km = gp.quicksum(self.assign_asc[r,s,k] * self.c_direct[r] 
                             for r in self.R_asc for s in self.S_asc for k in self.K) + \
                             gp.quicksum(self.assign_desc[r,s,k] * self.c_direct[r] 
                             for r in self.R_desc for s in self.S_desc for k in self.K)
        self.total_distance = gp.quicksum(self.x[i,j,s,k] * self.c[i,j] for k in self.K for s in self.S for (i,j) in self.drive_edges)

        self.saved_distance = self.pax_km - self.total_distance

        # define objective function
        obj_func = self.objWeights[0] * self.num_pax_accepted + self.objWeights[1] * self.saved_distance
                

        self.model.setObjective(obj_func, GRB.MAXIMIZE)

    def postprocessing(self, verbose = True):
        verboseprint = print if verbose else lambda *a, **b: None

        if self.model.Status == GRB.INFEASIBLE:
            verboseprint("Subline-Based model is infeasible.")

        elif self.model.Status == GRB.UNBOUNDED:
            verboseprint("Subline-Based model is unbounded.")

        elif self.model.SolCount == 0:
            verboseprint("No incumbent solution found.")

        else:
            # get obj values
            self.total_distance = round(self.total_distance.getValue(),1)
            self.num_pax_accepted = self.num_pax_accepted.getValue()
            self.required_busses = self.z.sum().getValue()
            self.pax_km = self.pax_km.getValue()

            self.passenger_assignment, self.bus_assignment = self._calculateAssignment()
            self.waiting_time_per_pax = self._calculateWaitingTimes()
            self.num_empty_services = self._calculateEmptyServices()
            self.activeServices = self._countActiveServices()

            start_node_per_bus = self._calculateNode(self.start_node)
            end_node_per_bus = self._calculateNode(self.end_node)

            self.paths = self._calculatePath()

            self.calculateStatistics()
            
            verboseprint(SEPERATOR)
            verboseprint("RESULTS")
            verboseprint(SEPERATOR)
            verboseprint("Objective Value:", self.model.getObjective().getValue())
            verboseprint(SEPERATOR)
            verboseprint("Buildtime:", round(self.Buildtime,4))
            verboseprint("Runtime:", round(self.model.Runtime, 4))
            verboseprint("MIP Gap:", self.model.MIPGap)
            verboseprint(SEPERATOR)
            verboseprint("Number of busses required:", self.required_busses)
            verboseprint("Number of passengers served overall:", self.num_pax_accepted)
            verboseprint("Distance driven:", self.total_distance)
            verboseprint("Passenger km saved:", self.pax_km)
            verboseprint("Passengers transported:", [i for i,k in self.passenger_assignment.items() if k is not None])
            verboseprint("Passengers rejected:", [i for i,k in self.passenger_assignment.items() if k is None])
            verboseprint("Passenger waiting times:", self.waiting_time_per_pax)

            print(SEPERATOR)
            print("STATISTICS")
            print(SEPERATOR)
            self._printStatistics()

            for k in self.K:
                verboseprint(SEPERATOR)
                verboseprint("SOLUTION FOR BUS", k)
                verboseprint(SEPERATOR)
                if self.z[k].X == 0:
                    verboseprint("Bus parked.")
                    continue

                verboseprint("Start node:",start_node_per_bus[k])
                verboseprint("End node:", end_node_per_bus[k])
                verboseprint("Path:", list(itertools.chain.from_iterable(self.paths[k].values())))
                

    def calculateStatistics(self):
        # average trip length in solution
        avg_waiting_time = 0
        avg_ride_time = 0
        avg_transportation_time = 0
        empty_mileage = 0
        pax_km_driven = 0
        pax_km_booked = 0
        pooling_factor = 0

        if self.num_pax_accepted == 0:
            super().calculateStatistics()
            return

        for r in self.R:
            if self.passenger_assignment[r] is not None:
                avg_waiting_time += self.waiting_time_per_pax[r]
                avg_ride_time += self.dropoffTime[r].X - self.pickupTime[r].X
                avg_transportation_time += self.dropoffTime[r].X - self.pickup_time_windows_per_request[r][0]

        self.avg_waiting_time = round(avg_waiting_time / self.num_pax_accepted, 2)
        self.avg_ride_time = round(avg_ride_time / self.num_pax_accepted, 2)
        self.avg_transportation_time = round(avg_transportation_time / self.num_pax_accepted, 2)

        # Note: the following calculations will have to change when implementing shortcuts or loads != 1.

        for k in self.K:
            for s in self.S:
                if len(self.bus_assignment[k][s]) == 0:
                    empty_mileage += sum(self.c[e] for e in self.paths[k][s])
                else:
                    pax_on_service = self.bus_assignment[k][s]
                    pax_km_booked += sum(self.c[(self.origins[pax],self.destinations[pax])] for pax in pax_on_service)
                    pooling_factor += sum(self.c[(self.origins[pax],self.destinations[pax])] for pax in pax_on_service)
                    for e in self.paths[k][s]:
                        i,j = e
                        num_pax_on_board = 0
                        for pax in pax_on_service:
                            if s in self.S_asc:
                                if (self.origins[pax] <= i) and (self.destinations[pax] >= j) and (i != j):
                                    # pax on board
                                    num_pax_on_board += 1
                            elif s in self.S_desc:
                                if (self.origins[pax] >= i) and (self.destinations[pax] <= j) and (i != j):
                                    num_pax_on_board += 1

                        if num_pax_on_board == 0:
                            empty_mileage += self.c[e]
                        else:
                            pax_km_driven += self.c[e] * num_pax_on_board
    
        if self.total_distance == 0:
            self.pooling_factor = 0
            self.avg_detour_factor = 0
            self.mean_occupancy = 0
            self.share_empty_mileage = 0
            self.system_efficiency = 0
            
        else:
            self.pooling_factor = pooling_factor / self.total_distance # gebuchte km / total routing costs
            self.avg_detour_factor = pax_km_driven / pax_km_booked
            self.mean_occupancy = pax_km_driven / (self.total_distance - empty_mileage)
            self.share_empty_mileage = empty_mileage / self.total_distance
            self.system_efficiency = pax_km_booked / self.total_distance

            if abs(self.system_efficiency - 1/ self.avg_detour_factor * self.mean_occupancy * (1 - self.share_empty_mileage)) > EPSILON:
                raise ValueError("Error in system efficiency!")

    def _calculateAssignment(self):
        passenger_assignment = self._calculatePassengerAssignment()
        bus_assignment = self._calculateBusAssignment(passenger_assignment)
        return passenger_assignment, bus_assignment

    def _calculatePassengerAssignment(self):
        assignment = {r: None for r in self.R}
        assignment = self._getNonzeroAssignment(assignment, self.assign_asc)
        assignment = self._getNonzeroAssignment(assignment, self.assign_desc)

        return assignment

    def _getNonzeroAssignment(self, assignment: dict, assignment_vars: dict):
        for (r,s,b), var in assignment_vars.items():
            if round(var.X) == 1:
                assignment[r] = (b,s)
        return assignment
    
    def _calculateBusAssignment(self, passenger_assignment):
        assignment = {k:{s:[] for s in self.S} for k in self.K}
        for r, val in passenger_assignment.items():
            if val is not None:
                k,s = val
                assignment[k][s].append(r)

        return assignment
    
    def _calculateEmptyServices(self):
        num_empty_services = 0
        for k in self.K:
            num_empty_services += sum([1 for l in self.bus_assignment[k].values() if not l])
        return num_empty_services
    
    def _calculateNode(self, var):
        return {k:i for k in self.K for i in self.H if round(var[i,k].X, 0) == 1}
    
    def _calculatePath(self):
        path = {}
        for k in self.K:
            desc_service = False
            path[k] = {}
            for s in self.S:
                edges = [(i,j) for (i,j) in self.edges if round(self.x[i,j,s,k].X) == 1]
                sorted_edges = sorted(edges, key = lambda x: x[0], reverse = desc_service)
                path[k][s] = sorted_edges
                desc_service = not desc_service
        return path

    def _calculateWaitingTimes(self):
        pax_wait_time = {}
        for r in self.R:
            # calculate only if pax was picked up
            if self.passenger_assignment[r]:
                b,s = self.passenger_assignment[r]
                bus_arrival_time = self.arrTime[self.origins[r], s, b].X

                if self.service_promises[r][0] and (self.pickup_time_windows_per_request[r][0] < bus_arrival_time): # pax must wait at origin station
                    pax_on_board_time = self.pickupTime[r].X
                    pax_wait_time[r] = pax_on_board_time - self.pickup_time_windows_per_request[r][0]
                elif self.service_promises[r][1] and (self.drop_off_time_windows_per_request[r][1] > bus_arrival_time): # pax is early at destination station
                    pax_off_board_time = self.dropoffTime[r].X
                    pax_wait_time[r] = self.drop_off_time_windows_per_request[r][1] - pax_off_board_time
                else:
                    pax_wait_time[r] = 0
            else:
                pax_wait_time[r] = 0 # rejected
        return pax_wait_time
    
    def _countActiveServices(self):
        active_services_by_bus = {}
        for k in self.K:
            active_services_by_bus[k] = self.num_services
            for s in self.S[::-1]:
                if len(self.bus_assignment[k][s]) == 0:
                    active_services_by_bus[k] -= 1
                else:
                    break
            # account for last turning service
            if active_services_by_bus[k] % 2 == 1:
                active_services_by_bus[k] += 1
        return active_services_by_bus

    def _prepareOutput(self, id):
        self.total_waiting_time = sum(self.waiting_time_per_pax.values())
        if self.num_pax_accepted == 0:
            self.avg_waiting_time = 0
        else:
            self.avg_waiting_time = self.total_waiting_time / self.num_pax_accepted
        return super()._prepareOutput(id)
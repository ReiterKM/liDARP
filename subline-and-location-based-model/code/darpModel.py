from util import *
from model import Model
from darpGraph import DarpGraph
from travelRequests import TravelRequests

class DARPModel(Model):
    def __init__(self, boarding_time = 3, *args, **kwargs):
        self.boarding_time = boarding_time
        self.name = "location-based"
        super().__init__(*args, **kwargs)

    def _initModel(self): 
        return gp.Model(self.name)

    def _calculateConstants(self):
        self.n = self.requests.num_requests
        Graph = DarpGraph(speed=self.speed, t_turn = self.t_turn, max_time = self.max_travel_minutes, boarding_time=self.boarding_time, travel_requests=self.requests, alpha=self.alpha, beta = self.beta)
        Graph.generate_graph()
        
        self.nodes = Graph.nodes
        self.asc_nodes, self.desc_nodes = Graph.get_directional_nodesets(as_labels=True)
        self.edges = Graph.get_edges()

        # calculate parameters
        self.K = range(self.num_busses)
        self.num_nodes_incl_depots = len(self.nodes)
        self.num_nodes = self.num_nodes_incl_depots - 2
        self.start_depot, self.end_depot = [n.name for n in self.nodes[-2:]]

        self.P = range(self.n)
        self.D = range(self.n, 2*self.n)
        self.P_bar = range(2*self.n,3*self.n)
        self.D_bar = range(3*self.n, 4*self.n)
        self.N = range(self.num_nodes)
        self.H_R = range(self.num_nodes_incl_depots)

        # verify parameters
        assert self.num_nodes == 4*self.n
        assert self.num_busses > 0

        # define subsets of edges
        self.edges_from_P_to_HR = [(i,j) for (i,j) in self.edges if (i in self.P) and (j in self.H_R)]
        self.turn_edges = [(i,j) for (i,j) in self.edges if (i in self.asc_nodes) and 
                       (j in self.desc_nodes)] + [(i,j) for (i,j) in self.edges if (i in self.desc_nodes) and (j in self.asc_nodes)] # turn-around edges
        self.edges_from_N_to_N = [(i,j) for (i,j) in self.edges if (i in self.N) and (j in self.N)]

        if len(self.edges_from_P_to_HR) == 0: raise ValueError("No edges from P to H_R found.")
        if len(self.turn_edges) == 0: raise ValueError("No turn edges found.")
        if len(self.edges_from_N_to_N) == 0: raise ValueError("No edges from N to N found.")

        # bigM constant
        self.big_M = len(self.edges)

        self.stations_after_start_depot = list(itertools.chain(self.P,self.P_bar)) + [self.end_depot]
        self.stations_before_end_depot = list(itertools.chain(self.D, self.D_bar)) + [self.start_depot]

        # direct drive distance and time
        self.c = Graph.get_edge_distances()
        self.c_direct = Graph.request_direct_distances
        self.t = {e: self.t_turn if e in self.turn_edges else self.c[e] * self.speed for e in self.c}

        # time windows
        self.e = [node.e for node in Graph.nodes]
        self.l = [node.l for node in Graph.nodes]

        # num pax boarding
        self.q = np.zeros(self.num_nodes_incl_depots)
        for i in self.P:
            self.q[i] = 1 # P
            self.q[i + self.n] = -1 # D

        # service duration (only for stations in P and D)
        self.b = np.zeros(self.num_nodes_incl_depots) 
        for i in itertools.chain(self.P, self.D):
            self.b[i] = self.boarding_time

        # max drive time
        self.L_max = np.zeros(self.n)
        for i in self.P:
            self.L_max[i] = self.alpha * self.t[i, i+self.n]

    def createModel(self, obj_weights:List[float], **kwargs):
        start_time = datetime.datetime.now()
        self.objWeights=obj_weights

        # define variables
        self.x = self.model.addVars(self.edges, self.K, vtype=GRB.BINARY,name="x") # decision variable
        self.L = self.model.addVars(self.n, vtype=GRB.CONTINUOUS, name="L", ub=self.max_travel_minutes) # ride time of request i
        self.B = self.model.addVars(self.num_nodes_incl_depots, vtype=GRB.CONTINUOUS, name="B", ub=self.max_travel_minutes) # service start time at station i
        self.B_start_depot = self.model.addVars(self.K, vtype=GRB.CONTINUOUS, name = "B_delta_s", ub = self.max_travel_minutes) # service start time for each bus at start depot
        self.B_end_depot = self.model.addVars(self.K, vtype=GRB.CONTINUOUS, name = "B_delta_e", ub = self.max_travel_minutes) # service start time for each bus at end depot
        self.Q = self.model.addVars(self.num_nodes, vtype=GRB.CONTINUOUS, name="Q", ub=self.Q_max) # passenger load after departing station i
        self.z = self.model.addVars(self.K, vtype=GRB.BINARY, name="z") # if bus b is used or not
        
        # define objective function
        self._setObjective()

        for o in self.N:
            incoming_edges_to_o = [(i,j) for (i,j) in self.edges if j == o]
            outgoing_edges_from_o = [(i,j) for (i,j) in self.edges if i == o]
            for k in self.K:
                self.model.addConstr((gp.quicksum(self.x[j,i,k] for (j,i) in incoming_edges_to_o) - gp.quicksum(self.x[i,j,k] for (i,j) in outgoing_edges_from_o)) == 0, name = "arc_flow_" + str(k)) # arc flow constr.

            outgoing_nodes_from_o = [j for (i,j) in outgoing_edges_from_o]
            self.model.addConstr(self.Q[o] <= self.Q_max * gp.quicksum(self.x[o,j,k] for j in outgoing_nodes_from_o for k in self.K), name = "ensure_empty_if_not_used") #ensure busses are empty if not driving
            
        for o in self.N:
            incoming_nodes_to_o = [i for (i,j) in self.edges if j == o]
            outgoing_nodes_from_o = [j for (i,j) in self.edges if i == o]

            max_for_lower_bound_on_B_per_j = {j: max(0, self.e[j] - self.e[o] + self.b[j] + self.t[j,o]) for j in incoming_nodes_to_o}
            self.model.addConstr(self.B[o] >= self.e[o] + gp.quicksum(max_for_lower_bound_on_B_per_j[j] * self.x[j,o,k] for j in incoming_nodes_to_o for k in self.K), name = "strenghten_lb_at_B") # strengthened lower bound on start of service time

            max_for_upper_bound_on_B_per_j = {j: max(0, self.l[o] - self.l[j] + self.b[o] + self.t[o,j]) for j in outgoing_nodes_from_o}
            self.model.addConstr(self.B[o] <= self.l[o] -  gp.quicksum(max_for_upper_bound_on_B_per_j[j] * self.x[o,j,k] for j in outgoing_nodes_from_o for k in self.K), name = "strenghten_ub_at_B") # strengthened upper bound on start of service time

        for (i,j) in self.turn_edges:
            self.model.addConstr(self.Q[i] <= self.Q_max * (1 - gp.quicksum(self.x[i,j,k] for k in self.K)), name = "load_when_turning_is_zero_1") # load when turning is zero
            self.model.addConstr(self.Q[i] >= -self.Q_max * (1 - gp.quicksum(self.x[i,j,k] for k in self.K)), name = "load_when_turning_is_zero_2") # load when turning is zero

        for k in self.K[:-1]:
            self.model.addConstr(self.z[k] >= self.z[k+1], name = "symm_breaking") # symmetry breaking

        for o in self.P:
            outgoing_edges = [(i,j) for (i,j) in self.edges if i == o]
            self.model.addConstr(gp.quicksum(self.x[i,j,k] for (i,j) in outgoing_edges for k in self.K) <= 1, name = "serve_request_max_once")
            self.model.addConstr(self.L[o] == self.B[o+self.n] - (self.B[o] + self.b[o]), name="ride_time") # ride time per request
            self.model.addConstr(self.L[o] >= self.t[o, o+self.n], name = "min_ride_time") # min ride time
            self.model.addConstr(self.L[o] <= self.L_max[o], name = "max_ride_time") # max. ride time

        for (i,j) in self.edges_from_N_to_N:
            self.model.addConstr(self.Q[j] >= (self.Q[i] + self.q[j]) * gp.quicksum(self.x[i,j,k] for k in self.K), name="load_when_leaving") # load upon leaving each station
            self.model.addConstr(self.B[j] >= (self.B[i] + self.b[i] + self.t[i,j]) * gp.quicksum(self.x[i,j,k] for k in self.K), name="min_dep_time") # min departure time at each station

        for k in self.K:
            self.model.addConstr(gp.quicksum(self.x[self.start_depot,j,k] for j in self.stations_after_start_depot) == 1, name = "buses_leave_depot") # busses leave the depot
            self.model.addConstr(gp.quicksum(self.x[i,self.end_depot,k] for i in self.stations_before_end_depot) == 1, name = "buses_enter_depot") # busses end at depot
            self.model.addConstr(self.z[k] >= (1/(len(self.N)**2)) * gp.quicksum(self.x[i,j,k] for (i,j) in self.edges_from_N_to_N), name="connect_z") # connect variable z
            self.model.addConstr(1 - gp.quicksum(self.x[i,j,k] for (i,j) in self.edges_from_N_to_N) <= self.big_M * self.x[self.start_depot, self.end_depot, k], name="ensure_depot_to_depot") # ensure bus only goes from start_depot to end_depot if no other node is visited
            for o in self.P:
                self.model.addConstr(gp.quicksum(self.x[i,j,k] for (i,j) in self.edges if i==o) - gp.quicksum(self.x[i,j,k] for (i,j) in self.edges if i==self.n+o) == 0, name = "pickup_and_deliver") # every customer picked up is also delivered
                
            for i in [*self.P, *self.P_bar]:
                self.model.addConstr(self.Q[i] >= self.q[i] * self.x[self.start_depot, i, k], name = "load_leaving_depot") # load upon leaving start depot
                self.model.addConstr(self.B[i] >= (self.B_start_depot[k] + self.b[self.start_depot] + self.t[self.start_depot, i]) * self.x[self.start_depot, i, k], name = "B_after_depot") # start of service time after leaving the start depot

            for i in [*self.D, *self.D_bar]:
                self.model.addConstr(0 >= (self.Q[i] + self.q[self.end_depot]) * self.x[i, self.end_depot, k], name="load_entering_depot") # load upon entering end depot
                self.model.addConstr(self.B_end_depot[k] >= (self.B[i] + self.b[i] + self.t[i, self.end_depot]) * self.x[i, self.end_depot, k], name = "B_entering_depot") # start of service time when entering the end depot

        self.model.update()

        end_time = datetime.datetime.now()
        self.Buildtime = (end_time - start_time).total_seconds()

    def _setObjective(self):
        self.num_pax_accepted = gp.quicksum(self.x[i,j,k] for k in self.K for (i,j) in self.edges_from_P_to_HR)
        self.total_distance = gp.quicksum(self.c[(i,j)] * self.x[i,j,k] for k in self.K for (i,j) in self.edges)

        self.pax_km = gp.LinExpr()
        for o in self.P:
            outgoing_nodes_from_i = [j for (i,j) in self.edges if i==o]
            self.pax_km += self.c_direct[o] * gp.quicksum(self.x[o, j, k] for j in outgoing_nodes_from_i for k in self.K)

        self.saved_distance = self.pax_km - self.total_distance

        # define objective function
        obj_func = self.objWeights[0] * self.num_pax_accepted + self.objWeights[1] * self.saved_distance

        self.model.setObjective(obj_func, GRB.MAXIMIZE)

    def postprocessing(self, verbose = True):

        # TODO alles überarbeiten

        verboseprint = print if verbose else lambda *a, **k: None

        if self.model.Status == GRB.INFEASIBLE:
            verboseprint("Location-Based Model is infeasible.")

        elif self.model.Status == GRB.UNBOUNDED:
            verboseprint("Location-Based Model is unbounded.")

        else:
            self.saved_distance = self.saved_distance.getValue()
            self.num_pax_accepted = self.num_pax_accepted.getValue()
            self.total_distance = self.total_distance.getValue()
            self.pax_km = self.pax_km.getValue()
            
            self.paths = self._calculatePath()
            self.next_nodes_path = self._calculateLinkedPath()
            self.visited_nodes_by_bus, self.visited_nodes = self._calculateVisitedNodes()
            self.service_times = self._calculateServiceTimes()
            self.loads = self._calculateLoad()
            self.departure_times = self._calculateDepartureTimes()
            self.waiting_time_per_pax = self._calculateWaitTime()

            self._calculatePassengerAssignment()
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
            verboseprint("Number of busses required:", self.num_vehicles)
            verboseprint("Number of passengers served overall:", self.num_pax_accepted)
            verboseprint("Distance driven:", self.total_distance)
            verboseprint("Passenger km saved:", self.pax_km)
            verboseprint("Passengers transported:", [i for i,k in self.pax_to_bus.items() if k is not None])
            verboseprint("Passengers rejected:", [i for i,k in self.pax_to_bus.items() if k is None])
            verboseprint("Passenger waiting times:", self.waiting_time_per_pax)

            print(SEPERATOR)
            print("STATISTICS")
            print(SEPERATOR)
            self._printStatistics()

            verboseprint(SEPERATOR)
            for k in self.K:
                verboseprint("SOLUTION FOR BUS", k)
                verboseprint(SEPERATOR)
                verboseprint("Edge path:")
                verboseprint(self.paths[k])
                verboseprint("Service times:")
                verboseprint(self.service_times[k])
                verboseprint(SEPERATOR)

    def calculateStatistics(self):
        # average trip length in solution
        avg_waiting_time = 0
        avg_ride_time = 0
        avg_transportation_time = 0
        pax_km_booked = 0
        pooling_factor = 0
        pax_km_driven = 0
        empty_mileage = 0
        self.num_vehicles = 0

        if self.num_pax_accepted == 0:
            super().calculateStatistics()
            return
        
        for i in self.P:
            if i in self.visited_nodes:
                if isvalid(self.waiting_time_per_pax[i]):
                    avg_waiting_time += self.waiting_time_per_pax[i]
                avg_ride_time += self.B[i + self.n].X - self.departure_times[i]
                avg_transportation_time += self.B[i + self.n].X - self.e[i]

        self.avg_waiting_time = round(avg_waiting_time / self.num_pax_accepted, 2)
        self.avg_ride_time = round(avg_ride_time / self.num_pax_accepted, 2)
        self.avg_transportation_time = round(avg_transportation_time / self.num_pax_accepted, 2)

        for k in self.K:
            for e in self.paths[k]:
                i,j = e
                if self.loads[i] == 0:
                    empty_mileage += self.c[e]
            if self.z[k].X > 0.9:
                self.num_vehicles += 1

        for i in self.P:
            if i in self.visited_nodes:
                pax_km_booked += self.q[i] * self.c[(i, i+self.n)]
                pooling_factor += self.c[(i, i + self.n)]

                k = self.pax_to_bus[i]

                current_node = i
                next_node = self.next_nodes_path[k][i]
                while next_node != (i + self.n):
                    pax_km_driven += self.q[i] * self.c[(current_node, next_node)]
                    current_node = next_node
                    next_node = self.next_nodes_path[k][current_node]
                pax_km_driven += self.q[i] * self.c[(current_node, next_node)]

        if (pax_km_driven != pax_km_booked):
            raise ValueError("Passenger KM driven and booked should be equal without shortcuts.")
        
        if (pax_km_driven != pooling_factor):
            raise ValueError("Pooling factor should be equal to pax km driven without shortcuts.")

        self.pooling_factor = pooling_factor / self.total_distance # gebuchte km / total routing costs

        self.avg_detour_factor = pax_km_driven / pax_km_booked
        self.mean_occupancy = pax_km_driven / (self.total_distance - empty_mileage)
        self.share_empty_mileage = empty_mileage / self.total_distance
        self.system_efficiency = pax_km_booked / self.total_distance

        if abs(self.system_efficiency - 1/ self.avg_detour_factor * self.mean_occupancy * (1 - self.share_empty_mileage)) > EPSILON:
            raise ValueError("Error in system efficiency!")

    def _calculatePassengerAssignment(self):
        pax_to_bus = {i: None for i in self.P}
        bus_to_pax = {k:[] for k in self.K}

        for k in self.K:
            for i in self.visited_nodes_by_bus[k]:
                if i in self.P:
                    pax_to_bus[i] = k
                    bus_to_pax[k].append(i)

        self.pax_to_bus = pax_to_bus
        self.bus_to_pax = bus_to_pax

    def _calculateDepartureTimes(self):
        departure_time = {i.name: None for i in self.nodes}
        for i in self.visited_nodes:
            departure_time[i] = self.B[i].X + self.b[i]
            
        return departure_time

    def _calculateWaitTime(self):
        waiting_time = {}
    	
        for request in self.P:
            if self.nodes[request].has_departure_service_promise:
                waiting_time[request] = round(self.B[request].X - self.nodes[request].e, 2)
            elif self.nodes[request + self.n].has_arrival_service_promise:
                waiting_time[request] = round(self.nodes[request + self.n].l - self.B[request + self.n].X, 2)
            else:
                waiting_time[request] = None

            if isvalid(waiting_time[request]):
                if waiting_time[request] < 0:
                    raise ValueError("Waiting time of request {0} is negative.".format(request))
        return waiting_time
    
    def sortEdgesByStationOrder(self, edges: np.array) -> np.array:
        # add first edge
        sorted_edges = [(i,j) for (i,j) in edges if i == self.start_depot]
        if len(sorted_edges) != 1:
            raise ValueError("Edgelist missing single edge from start depot.")
        edges.remove(sorted_edges[0])

        while len(edges) > 0:
            start_station = sorted_edges[-1][1]
            next_edge = [(i,j) for (i,j) in edges if i == start_station][0]
            sorted_edges.append(next_edge)
            edges.remove(next_edge)
        
        if sorted_edges[-1][1] != self.end_depot:
            raise ValueError("Edgelist missing edge to the end depot.")
        return sorted_edges

    def _calculatePath(self):
        path = {}
        for k in self.K:
            edges = [(i,j) for (i,j) in self.edges if round(self.x[i,j,k].X,0) == 1]
            sorted_edges = self.sortEdgesByStationOrder(edges=edges)
            path[k] = sorted_edges
        return path
    
    def _calculateLinkedPath(self):
        linked_path = {}
        for k in self.K:
            linked_bus_path = {}
            for e in self.paths[k]:
                i,j = e
                linked_bus_path[i] = j
            linked_path[k] = linked_bus_path
        return linked_path
    
    def _calculateVisitedNodes(self):
        all_nodes = []
        nodes_by_bus = {}
        for k in self.K:
            nodes_by_bus[k] = [self.paths[k][0][0]] + [j for (i,j) in self.paths[k][:-1]] + [self.paths[k][-1][1]]
            all_nodes += [self.paths[k][0][0]] + [j for (i,j) in self.paths[k][:-1]] + [self.paths[k][-1][1]]
        return nodes_by_bus, all_nodes
    
    def _calculateServiceTimes(self):
        service_time_by_bus = {}
        for k in self.K:
            service_time_by_bus[k] = {i: round(self.B[i].X, 4) for i in self.visited_nodes_by_bus[k]}
        return service_time_by_bus

    def _calculateLoad(self):
        load = {self.start_depot:0, self.end_depot:0}
        for k in self.K:
            load |= {i: round(self.Q[i].X) for i in self.visited_nodes_by_bus[k] if i not in [self.start_depot,self.end_depot]}
        return load
    
    def _prepareOutput(self, id):
        self.S = []
        self.num_empty_services = None
        self.activeServices = None

        self.total_waiting_time = sum(filter(None, self.waiting_time_per_pax.values()))
        if self.num_pax_accepted == 0:
            self.avg_waiting_time = 0
        else:
            self.avg_waiting_time = self.total_waiting_time / self.num_pax_accepted

        return super()._prepareOutput(id)
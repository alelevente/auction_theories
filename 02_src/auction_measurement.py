import traci

import numpy as np
import pandas as pd

import argparse
import sys
import os
import json
import re

import auction

SIM_ROOT = "../01_simulation/"
SIMULATION = "../01_simulation/stable.sumocfg"
STABLE_STATES_ROOT = "../01_simulation/stable_states/"
SUMO_CMD = ["sumo", "-c", SIMULATION, "--no-step-log", "--stop-output", "stop_data.out.xml"]
TIME_STEP = 15
PARKING_DEFS = "../01_simulation/parking_areas.add.xml"
STABLE_STATES_ROOT = "../01_simulation/stable_states/"
STARTING_PRICE_DEF = "../01_simulation/starting_prices.json"

AUCTIONS_TILL = 381600


def create_parking_mtx(veh_destinations, parking_distance_map, parking_edges):
    answer_mtx = {}
    for veh in veh_destinations:
        new_row = {}
        for p_edge in parking_edges:
            new_row[p_edge] = parking_distance_map[veh_destinations[veh]][p_edge]
        answer_mtx[veh] = new_row
    return answer_mtx

def calc_free_parkings(free_parkings, reservations):
    for res in reservations:
        free_parkings[res] = max(0, free_parkings[res] - reservations[res])
    return free_parkings


def calculate_route_distance(vehicle):
    route = traci.vehicle.getRoute(vehicle)
    dep_pos = traci.vehicle.getLanePosition(vehicle)
    arr_pos = traci.lane.getLength(f"{route[-1]}_0")
    distance = 0
    if len(route)>1:
        distance = traci.vehicle.getDrivingDistance(vehicle, route[1], 0)
    for i in range(1, len(route)-1):
        distance += traci.simulation.getDistanceRoad(route[i], 0,
                                                     route[i+1], 0,
                                                     isDriving=True)
    distance += arr_pos
    return distance

def make_auctions(free_parkings: dict, veh_destinations: dict, parking_distance_map: dict,
                  starting_prices: dict, parking_edges: list, beta_values, beta_probabilities):
    
    p_mtx = create_parking_mtx(veh_destinations, parking_distance_map, parking_edges)        
    betas_ = np.random.choice(beta_values, size=len(veh_destinations), p=beta_probabilities)
    beta_per_vehicle = {}
    for i, bi in enumerate(veh_destinations):
        beta_per_vehicle[bi] = betas_[i]
        
    auctions, buyers = auction.init_auction_method(free_parkings, veh_destinations, starting_prices,
                                                   p_mtx, betas=beta_per_vehicle)
    if len(buyers)>0:
        print(f"{len(auctions)} auctions started with {len(buyers)} buyers")
    auction_results = auction.run_auctions(auctions, buyers, r_max=100)
    return auction_results

def get_parking_edge(parking_id):
    if "garage" in parking_id:
        edge = parking_id.split("_")[-1]
    else:
        edge = parking_id.split("pa")[-1]
    return edge

def parse_beta_config(path):
    with open(path) as f:
        beta_conf = json.load(f)
    return beta_conf["values"], beta_conf["probabilities"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("seed", help="seed", type=str)
    parser.add_argument("--name", help="name of the simulation", type=str)
    parser.add_argument("--config", help="beta config file", type=str)
    
    args = parser.parse_args()
    seed = args.seed
    
    beta_values, beta_probabilities = parse_beta_config(args.config)
    
    parking_df = pd.read_xml(PARKING_DEFS, xpath="parkingArea").set_index("id")
    with open(STARTING_PRICE_DEF) as f:
        starting_prices = json.load(f)
    
    sumo_cmd = SUMO_CMD + ["--seed", seed, "--output-prefix", args.name,
                          "--load-state", f"{STABLE_STATES_ROOT}stable{seed}.xml"]
    traci.start(sumo_cmd)
    
    #initialization:
    parking_distance_map = {}
    parking_edges = []

    for i,r in parking_df.iterrows():
        parking_edges.append(r.lane.split("_")[0])

    for p_i in parking_edges:
        parking_distance_map[p_i] = {}
        for p_j in parking_edges:
            parking_distance_map[p_i][p_j] = traci.simulation.getDistanceRoad(p_i, 0, p_j, 0, True)
        
        
    controlled_vehicles = set()
    to_auction = set()
    active_vehicles = set()
    vehicle_data = {}
    unsuccessfuls = 0

    auction_outcomes = []
    park_ends = ()
    free_parkings = {}

    reservations = {}
    for pid in parking_df.index:
        reservations[pid] = 0

    p_ids, p_times, p_occups = [],[],[]
    
    end_time = traci.simulation.getEndTime()
    #end_time = 368000
    time = traci.simulation.getTime()

    #Main simulation loop with auctions:
    while time < AUCTIONS_TILL:
        time = traci.simulation.getTime()

        #with lower frequency:
        if time%TIME_STEP == 0:
            print(f"time: {time}")
            #parking occupancies:
            for parking_area in parking_df.index:
                occup = traci.parkingarea.getVehicleCount(parking_area)
                p_ids.append(parking_area)
                p_times.append(time)
                p_occups.append(occup/parking_df["roadsideCapacity"][parking_area])
                free_parkings[parking_area] = parking_df["roadsideCapacity"][parking_area] - occup

            #auctions:
            dests = {}
            for veh in to_auction:
                dests[veh] = vehicle_data[veh]["original_position"]
            free_parkings = calc_free_parkings(free_parkings, reservations)
            auction_result = make_auctions(free_parkings, dests, parking_distance_map,
                                           starting_prices.copy(), parking_edges,
                                           beta_values, beta_probabilities)
            for ar in auction_result:
                veh = auction_result[ar]["winner"]
                if veh!="":
                    print(ar, auction_result[ar])
                    controlled_vehicles.add(veh)
                    new_dest = re.sub("_[0-9]*$", "", ar)
                    auction_outcomes.append({
                        "parking_edge": new_dest,
                        "auction_id": ar,
                        "price": auction_result[ar]["price"],
                        "vehicle": veh,
                        "distance": traci.simulation.getDistanceRoad(
                                                        dests[veh], 0,
                                                        get_parking_edge(new_dest), 0, True),
                        "time": time
                    })
                    vehicle_data[veh]["controlled"] = True
                    #flag 65: parking @ a parking area
                    try:
                        next_stop = traci.vehicle.getStops(veh, 1)[0]
                        traci.vehicle.replaceStop(veh, 0, f"{new_dest}", flags=65, duration=next_stop.duration)
                        reservations[f"{new_dest}"] += 1
                    except:
                        unsuccessfuls += 1
                        controlled_vehicles.remove(auction_result[ar]["winner"])
            to_auction = set()

        #departing vehicles:
        departed_vehicles = traci.simulation.getDepartedIDList()
        #print(departed_vehicles)
        for dpv in departed_vehicles+park_ends:
            next_stop = traci.vehicle.getStops(dpv)
            if len(next_stop)>0: #not through vehicle
                pos = get_parking_edge(next_stop[0].stoppingPlaceID)
                vehicle_data[dpv] = {
                    "original_position": pos,
                    "controlled": False,
                    "occupied_reserved": False
                }
                to_auction.add(dpv)
                active_vehicles.add(dpv)
                
        #parkEndingVehicles:
        park_ends = traci.simulation.getParkingEndingVehiclesIDList()

        #stopping vehicles:
        stopping_vehicles = traci.simulation.getStopStartingVehiclesIDList()
        for spv in stopping_vehicles:
            stop_stat = traci.vehicle.getStops(spv)[0]
            parking_edge = get_parking_edge(stop_stat.stoppingPlaceID)
            #for uncontrolled or occupied
            if spv in controlled_vehicles:
                reserved_edge = parking_edge
                reservations[f"pa{reserved_edge}"] -= 1
                
        active_vehicles = active_vehicles - set(stopping_vehicles)     

        traci.simulation.step()
        
    #finishing up the simulation:
    traci.simulation.step(end_time)
                
    traci.close()            
    
    occupancy_results = pd.DataFrame()
    occupancy_results["parking_id"] = p_ids
    occupancy_results["time"] = p_times
    occupancy_results["occupancy"] = p_occups
    
    try:
        if not os.path.exists(f"../03_results/{args.name}"):
            os.mkdir(f"../03_results/{args.name}")
        os.replace(f"./{args.name}stop_data.out.xml",
                   f"../03_results/{args.name}/stop_data.out.xml")
        
        with open(f"../03_results/{args.name}/auction_results.json", "w") as f:
            json.dump({"outcomes": auction_outcomes,
                       "unsuccessful": unsuccessfuls}, f)
            
    except Exception as e:
        print(e)
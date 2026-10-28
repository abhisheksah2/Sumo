import asyncio
import json
import math
import traci
import websockets

CONNECTED_CLIENTS = set()
MAX_VEHICLES_LIMIT = 150

sumo_cmd = [
    "sumo",
    "-c", "osm.sumocfg",
    "--scale", "0.7",
    "--max-num-vehicles", str(MAX_VEHICLES_LIMIT),
    "--time-to-teleport", "60",
    "--no-step-log", "true",
    "--no-warnings", "true"
]

SIGNAL_POINTS = []
ADAPTIVE_SWITCH_COUNT = 0


async def ws_handler(websocket):
    CONNECTED_CLIENTS.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        CONNECTED_CLIENTS.discard(websocket)


def compute_stop_line_geometry(shape):
    """
    Given a lane's (x, y) shape, returns the stop-line center (lon, lat),
    the road heading angle in degrees, and a perpendicular SUMO stop-bar GeoJSON line.
    """
    if len(shape) >= 2:
        x1, y1 = shape[-2]
        x2, y2 = shape[-1]
    else:
        x1, y1 = shape[0]
        x2, y2 = shape[0]

    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    if length < 1e-3:
        ux, uy = 0.0, 1.0
    else:
        ux, uy = dx / length, dy / length

    # Perpendicular vector for a 3.6m wide SUMO stop-bar across the lane
    px, py = -uy * 1.8, ux * 1.8

    lon_c, lat_c = traci.simulation.convertGeo(x2, y2)
    lon_l, lat_l = traci.simulation.convertGeo(x2 + px, y2 + py)
    lon_r, lat_r = traci.simulation.convertGeo(x2 - px, y2 - py)

    angle_deg = (math.degrees(math.atan2(ux, uy)) + 360.0) % 360.0

    return {
        "lon": round(lon_c, 6),
        "lat": round(lat_c, 6),
        "angle": round(angle_deg, 1),
        "bar": [
            [round(lon_l, 6), round(lat_l, 6)],
            [round(lon_r, 6), round(lat_r, 6)]
        ]
    }


def init_tls_metadata():
    """
    Initializes traffic light stop-lines across ALL intersections and junctions in Sector 22.
    """
    global SIGNAL_POINTS, ADAPTIVE_SWITCH_COUNT
    SIGNAL_POINTS = []
    ADAPTIVE_SWITCH_COUNT = 0
    seen_lanes = set()

    # 1. Native SUMO Traffic Lights
    for tls_id in traci.trafficlight.getIDList():
        controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
        for idx, lane_id in enumerate(controlled_lanes):
            if lane_id in seen_lanes or lane_id.startswith(":"):
                continue
            seen_lanes.add(lane_id)
            try:
                shape = traci.lane.getShape(lane_id)
                geom = compute_stop_line_geometry(shape)
            except Exception:
                continue

            SIGNAL_POINTS.append({
                "id": f"{tls_id}_{idx}",
                "tls_id": tls_id,
                "tls_index": idx,
                "lane_id": lane_id,
                "is_native_tls": True,
                "lon": geom["lon"],
                "lat": geom["lat"],
                "angle": geom["angle"],
                "bar": geom["bar"],
                "phase_state": "green",
                "last_switch_step": 0
            })

    # 2. Cover ALL other junctions/intersections in Sector 22
    for junc_id in traci.junction.getIDList():
        if junc_id.startswith(":"):
            continue
        try:
            jx, jy = traci.junction.getPosition(junc_id)
            jlon, jlat = traci.simulation.convertGeo(jx, jy)
        except Exception:
            continue

        # Strict Sector 22 bounds check
        if not (76.7620 <= jlon <= 76.7860 and 30.7220 <= jlat <= 30.7420):
            continue

        inc_edges = traci.junction.getIncomingEdges(junc_id)
        for edge_id in inc_edges:
            if edge_id.startswith(":"):
                continue
            try:
                lane_ids = traci.edge.getLaneIDs(edge_id)
            except Exception:
                continue

            for lane_id in lane_ids:
                if lane_id in seen_lanes or lane_id.startswith(":"):
                    continue
                seen_lanes.add(lane_id)
                try:
                    shape = traci.lane.getShape(lane_id)
                    geom = compute_stop_line_geometry(shape)
                except Exception:
                    continue

                SIGNAL_POINTS.append({
                    "id": f"sig_{junc_id}_{lane_id}",
                    "tls_id": junc_id,
                    "tls_index": 0,
                    "lane_id": lane_id,
                    "is_native_tls": False,
                    "lon": geom["lon"],
                    "lat": geom["lat"],
                    "angle": geom["angle"],
                    "bar": geom["bar"],
                    "phase_state": "green",
                    "last_switch_step": 0
                })

    print(f"Initialized {len(SIGNAL_POINTS)} Adaptive Traffic Signal stop-lines across ALL intersections in Sector 22.")


def run_adaptive_traffic_control(step):
    global ADAPTIVE_SWITCH_COUNT
    output_signals = []

    native_states = {}
    for tls_id in traci.trafficlight.getIDList():
        native_states[tls_id] = traci.trafficlight.getRedYellowGreenState(tls_id)

    for sig in SIGNAL_POINTS:
        lane_id = sig["lane_id"]
        try:
            halting = traci.lane.getLastStepHaltingNumber(lane_id)
            veh_count = traci.lane.getLastStepVehicleNumber(lane_id)
        except Exception:
            continue

        if sig["is_native_tls"]:
            tls_id = sig["tls_id"]
            state_str = native_states.get(tls_id, "")
            idx = sig["tls_index"]
            ch = state_str[idx].lower() if idx < len(state_str) else "g"

            if ch == "y":
                color = "yellow"
            elif ch == "g":
                color = "green"
            else:
                color = "red"

            if step - sig["last_switch_step"] >= 20 and color == "red" and halting >= 2:
                logic_list = traci.trafficlight.getAllProgramLogics(tls_id)
                if logic_list:
                    phases = logic_list[0].phases
                    next_phase = (traci.trafficlight.getPhase(tls_id) + 1) % len(phases)
                    traci.trafficlight.setPhase(tls_id, next_phase)
                    sig["last_switch_step"] = step
                    ADAPTIVE_SWITCH_COUNT += 1
        else:
            elapsed = step - sig["last_switch_step"]
            current = sig["phase_state"]

            if current == "yellow" and elapsed >= 6:
                sig["phase_state"] = "red"
                sig["last_switch_step"] = step
            elif current == "red":
                if (halting >= 1 and elapsed >= 10) or (veh_count >= 2 and elapsed >= 12) or elapsed >= 30:
                    sig["phase_state"] = "green"
                    sig["last_switch_step"] = step
                    ADAPTIVE_SWITCH_COUNT += 1
            elif current == "green":
                if (veh_count == 0 and elapsed >= 15) or elapsed >= 35:
                    sig["phase_state"] = "yellow"
                    sig["last_switch_step"] = step
                    ADAPTIVE_SWITCH_COUNT += 1

            color = sig["phase_state"]

        output_signals.append({
            "id": sig["id"],
            "lon": sig["lon"],
            "lat": sig["lat"],
            "angle": sig["angle"],
            "bar": sig["bar"],
            "color": color,
            "queue": halting
        })

    return output_signals


def run_simulation_step(step):
    traci.simulationStep()

    if step > 20 and traci.simulation.getMinExpectedNumber() == 0:
        return None

    tls_telemetry = run_adaptive_traffic_control(step)

    all_veh_ids = traci.vehicle.getIDList()
    active_ids = all_veh_ids[:MAX_VEHICLES_LIMIT]

    vehicles = []
    for veh_id in active_ids:
        x, y = traci.vehicle.getPosition(veh_id)
        lon, lat = traci.simulation.convertGeo(x, y)
        angle = traci.vehicle.getAngle(veh_id)
        speed = traci.vehicle.getSpeed(veh_id)

        vehicles.append({
            "id": veh_id,
            "lon": round(lon, 6),
            "lat": round(lat, 6),
            "angle": round(angle, 1),
            "speed": round(speed, 2)
        })

    return {
        "step": step,
        "active_vehicles": len(vehicles),
        "vehicles": vehicles,
        "traffic_lights": tls_telemetry,
        "adaptive_switches": ADAPTIVE_SWITCH_COUNT
    }


async def simulation_loop():
    traci.start(["sumo", "-c", "osm.sumocfg"])
    await asyncio.to_thread(init_tls_metadata)
    step = 0

    try:
        while True:
            payload = await asyncio.to_thread(run_simulation_step, step)

            if payload is None:
                print("All trips finished! Restarting traffic loop from Step 0...")
                await asyncio.to_thread(traci.load, sumo_cmd[1:])
                await asyncio.to_thread(init_tls_metadata)
                step = 0
                continue

            if CONNECTED_CLIENTS:
                message = json.dumps(payload)
                await asyncio.gather(
                    *[client.send(message) for client in CONNECTED_CLIENTS],
                    return_exceptions=True
                )

            step += 1
            await asyncio.sleep(0.05)
    finally:
        traci.close()


async def main():
    # Read the dynamic port assigned by Render, defaulting to 8765 locally
    port = int(os.environ.get("PORT", 8765))

    server = await websockets.serve(ws_handler, "0.0.0.0", port)
    print(f"3D Digital Twin WebSocket Server active on port {port}")
    await asyncio.gather(server.wait_closed(), simulation_loop())


if __name__ == "__main__":
    asyncio.run(main())

tls_list = traci.trafficlight.getIDList()
print(f"Total traffic lights controlled: {len(tls_list)}")
print("Traffic Light IDs:", tls_list)

from __future__ import annotations

from dataclasses import dataclass, field

from rcbranch.geometry.active_set import VehicleOnPath
from rcbranch.mpc.constraints import longitudinal_step
from rcbranch.planners.proposed_dual_priced_branching import PlannerMemory, run_mpc_cycle


@dataclass(slots=True)
class ClosedLoopTrace:
    ego_accel: list[float] = field(default_factory=list)
    planner_mode: list[str] = field(default_factory=list)


def run_simple_closed_loop(
    vehicles: list[VehicleOnPath],
    *,
    cycles: int,
    dt: float = 0.2,
) -> ClosedLoopTrace:
    memory = PlannerMemory()
    trace = ClosedLoopTrace()
    current = vehicles
    for _ in range(cycles):
        ego_a, _ = run_mpc_cycle(current, memory)
        trace.ego_accel.append(ego_a)
        trace.planner_mode.append(memory.planner_mode)
        next_vehicles: list[VehicleOnPath] = []
        for vehicle in current:
            accel = ego_a if vehicle.is_ego else vehicle.a_obs
            s1, v1 = longitudinal_step(vehicle.s0, vehicle.v0, accel, dt)
            next_vehicles.append(
                VehicleOnPath(
                    obstacle_id=vehicle.obstacle_id,
                    is_ego=vehicle.is_ego,
                    route_id=vehicle.route_id,
                    incoming_id=vehicle.incoming_id,
                    ref_path=vehicle.ref_path,
                    s0=s1,
                    v0=v1,
                    a_obs=accel,
                    length=vehicle.length,
                    width=vehicle.width,
                    desired_speed=vehicle.desired_speed,
                    max_accel=vehicle.max_accel,
                    max_brake=vehicle.max_brake,
                    comfort_brake=vehicle.comfort_brake,
                )
            )
        current = next_vehicles
    return trace

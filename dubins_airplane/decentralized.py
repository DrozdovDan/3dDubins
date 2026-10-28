"""Peer-to-peer prioritized planning in an unreliable, event-driven simulation.

Each LocalPlanner owns its route, proposals and received peer reservations.
The transport only delivers messages; it never chooses routes/departures.
Initial native searches run in parallel threads. Confirmed flights are immutable
for this mission; this is batch coordination while airborne on separate circles,
not a flight controller or an arbitrary in-flight replanning/network stack.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import asdict, dataclass, replace
from heapq import heappop, heappush
from math import ceil, cos, isfinite, pi, sin, tau
from random import Random
from time import perf_counter

from .core import Pose, plan_dubins_airplane
from .fleet import (AgentRequest, FleetPlan, NoScheduleError, ScheduledFlight,
                    check_separation, departure_phase_delay, loiter_disk_distance)
from .rrt_star import NoPathError, PlanningResult, plan_safe_rrt_star
from .rrt_star import _validate_relative_gap
from .traffic import plan_safe_rrt_star_traffic


@dataclass(frozen=True)
class NetworkSettings:
    minimum_delay: float = 0.02
    maximum_delay: float = 0.15
    loss: float = 0.1
    duplicates: float = 0.1
    retry_interval: float = 0.2
    seed: int = 17
    blocked_links: frozenset[tuple[str, str]] = frozenset()

    def __post_init__(self):
        if (not all(isfinite(v) for v in (self.minimum_delay, self.maximum_delay,
                                         self.loss, self.duplicates, self.retry_interval))
                or self.minimum_delay < 0 or self.maximum_delay < self.minimum_delay
                or not 0 <= self.loss <= 1 or not 0 <= self.duplicates <= 1
                or self.retry_interval <= 0):
            raise ValueError("invalid network delays, loss, duplication or retry interval")


@dataclass(frozen=True)
class Message:
    kind: str
    sender: str
    receiver: str
    version: int
    flight: ScheduledFlight | None = None
    packet_id: int = 0  # Assigned by the transport; retries get new IDs.


@dataclass(frozen=True)
class ProtocolEvent:
    time: float
    agent: str
    kind: str
    peer: str = ""
    version: int = 0
    detail: str = ""
    packet_id: int = 0
    delivery_time: float | None = None


@dataclass(frozen=True)
class RouteSnapshot:
    """Observer-only local route, captured when READY is actually processed."""

    time: float
    agent: str
    attempt: int
    flight: ScheduledFlight


@dataclass(frozen=True)
class PlanningCall:
    """Observer-only wall time of one full local search, including failures."""

    agent: str
    attempt: int
    mode: str
    elapsed_ms: float
    succeeded: bool
    iteration_budget: int = 600
    iterations: int | None = None  # Actual work for successful searches; None on failure.


@dataclass(frozen=True)
class DecentralizedResult:
    plan: FleetPlan
    events: tuple[ProtocolEvent, ...]
    negotiation_time: float  # Virtual seconds, includes route readiness/network delays.
    wall_ms: float  # End-to-end simulator wall time, includes final audit.
    local_routing_ms: tuple[tuple[str, float], ...]
    route_attempts: tuple[tuple[str, int], ...]
    agreement_complete: bool = True
    planning_window: float = 5.0
    network: NetworkSettings | None = None
    initial_flights: tuple[ScheduledFlight, ...] = ()  # Observer-only independent proposals.
    route_history: tuple[RouteSnapshot, ...] = ()  # Not a source of peer knowledge.
    planning_calls: tuple[PlanningCall, ...] = ()

    @property
    def messages_sent(self):
        return sum(e.kind == "SEND" for e in self.events)

    @property
    def messages_lost(self):
        return sum(e.kind == "DROP" for e in self.events)

    def metadata(self):
        commits = {e.agent: e.time for e in self.events if e.kind == "COMMIT"}
        packets = [{"id": e.packet_id, "sender": e.agent, "receiver": e.peer,
                    "kind": e.detail, "version": e.version, "sent_s": e.time,
                    "delivery_s": e.delivery_time, "lost": e.kind == "DROP"}
                   for e in self.events if e.kind in ("IN_FLIGHT", "DROP")]
        network = self.network
        return {"coordination": "decentralized", "negotiation_time_s": self.negotiation_time,
                "conflict_resolution": "time_aware_spatial_replan",
                "agreement_complete": self.agreement_complete,
                "planning_window_s": self.planning_window, "commit_times_s": commits,
                "packets": packets,
                "network": ({"minimum_delay_s": network.minimum_delay,
                             "maximum_delay_s": network.maximum_delay, "loss": network.loss,
                             "duplicates": network.duplicates, "retry_interval_s": network.retry_interval,
                             "seed": network.seed, "blocked_links": sorted(network.blocked_links)}
                            if network else None),
                "wall_ms": self.wall_ms, "messages_sent": self.messages_sent,
                "messages_lost": self.messages_lost, "local_routing_ms": dict(self.local_routing_ms),
                "route_attempts": dict(self.route_attempts),
                "planning_calls": [asdict(c) for c in self.planning_calls],
                "conflicts_rejected": sum(e.kind == "CONFLICT" for e in self.events),
                "departure_delays": [asdict(e) for e in self.events if e.kind == "DELAY"],
                "route_history": [{"time_s": r.time, "agent": r.agent, "attempt": r.attempt,
                                   "departure_s": r.flight.departure, "arrival_s": r.flight.arrival,
                                   "points": [[p.x, p.y, p.z] for p in r.flight.route.sample(5)]}
                                  for r in self.route_history],
                "events": [asdict(e) for e in self.events]}


class DecentralizedPlanningError(NoScheduleError):
    """The mission failed; reservations describe what agents may still execute.

    Confirmed flights are NOT rolled back on timeout. Unconfirmed agents
    continue their original circles. Inspect events to diagnose the failure.
    """

    def __init__(self, reason, reservations, events, result=None):
        super().__init__(reason + "; confirmed flights remain reserved; unconfirmed agents keep their start circles")
        self.reservations = reservations
        self.events = tuple(events)
        self.result = result  # Audited partial execution for diagnostic visualization.


def holding_flight(request):
    """A real, indefinitely flown initial circle, not a stationary placeholder."""
    phase, c = request.initial_phase, request.start
    if phase is None:
        raise ValueError("decentralized agents require an initial phase")
    pose = Pose(c.x + c.radius * cos(phase), c.y + c.radius * sin(phase), c.z, phase + pi / 2)
    held = replace(request, goal=c, earliest_departure=0)
    path = plan_dubins_airplane(pose, pose, request.radius, request.max_angle)
    route = PlanningResult((path,), c, c, 0, 0, 0, ())
    return ScheduledFlight(held, route, 0)


class SimulatedTransport:
    """Unordered, lossy delivery plus local timers. No planning decisions here."""

    def __init__(self, settings):
        self.settings = settings
        self.rng = Random(settings.seed)
        self.now = 0.0
        self.queue = []
        self.sequence = 0
        self.packet_sequence = 0
        self.events = []
        self.route_history = []  # Append-only diagnostics; planners never read it.

    def log(self, agent, kind, peer="", version=0, detail="", *, packet_id=0, delivery_time=None):
        self.events.append(ProtocolEvent(self.now, agent, kind, peer, version, detail,
                                         packet_id, delivery_time))

    def local(self, at, receiver, payload):
        self.sequence += 1
        heappush(self.queue, (at, self.sequence, receiver, payload))

    def send(self, message):
        self.packet_sequence += 1
        message = replace(message, packet_id=self.packet_sequence)
        self.log(message.sender, "SEND", message.receiver, message.version, message.kind,
                 packet_id=message.packet_id)
        s = self.settings
        if ((message.sender, message.receiver) in s.blocked_links or self.rng.random() < s.loss):
            self.log(message.sender, "DROP", message.receiver, message.version, message.kind,
                     packet_id=message.packet_id)
            return
        copies = 2 if self.rng.random() < s.duplicates else 1
        for _ in range(copies):
            at = self.now + self.rng.uniform(s.minimum_delay, s.maximum_delay)
            self.log(message.sender, "IN_FLIGHT", message.receiver, message.version, message.kind,
                     packet_id=message.packet_id, delivery_time=at)
            self.local(at, message.receiver, message)


class LocalPlanner:
    """One agent's state machine. Only received messages reveal peer plans.

    IDs define the same strict priority on every node, regardless of roster
    input order. Higher-priority commits are awaited; still-uncommitted peers
    are conservatively represented by indefinitely reserved start circles.
    No proposal is flown. Votes and immutable commits make execution safe even
    if a subsequent COMMIT packet is lost: that peer keeps its protected circle.
    """

    def __init__(self, world, request, roster, transport, *, separation, max_delay,
                 planning_window, horizon, iterations, seed, max_route_attempts, retry_iterations=None,
                 relative_gap=0.0):
        self.world, self.request, self.transport = world, request, transport
        self.roster = dict(roster)  # Mission metadata only, never shared planning state.
        self.name = request.name
        self.peers = tuple(sorted(set(self.roster) - {self.name}))
        self.higher = tuple(p for p in self.peers if p < self.name)
        self.holds = {name: holding_flight(r) for name, r in self.roster.items()}
        self.received = {}  # Peer COMMIT messages, local immutable reservations.
        self.failed_peers = set()  # A final FAILED message retains that peer's initial circle.
        self.votes = {}  # Latest acknowledged (version, exact flight) for each peer.
        self.acks = set()
        self.confirmations = set()
        self.committed = None
        self.pending = None
        self.version = 0
        self.route = None
        self.fixed_start = None
        self.fixed_departure = None
        self.traffic_replan = False
        self.failed = ""
        self.busy = True
        self.attempts = 0
        self.routing_ms = 0.0
        self.planning_calls = []
        self.separation, self.max_delay = separation, max_delay
        self.window, self.horizon = planning_window, horizon
        self.iterations, self.seed = iterations, seed
        _validate_relative_gap(relative_gap)
        self.relative_gap = relative_gap
        self.retry_iterations = iterations if retry_iterations is None else retry_iterations
        self.search_failed = False
        self.max_attempts = max_route_attempts
        self.minimum_departure = max(planning_window, request.earliest_departure)
        if not world.validate_loiter(request.start) or not world.validate_loiter(request.goal):
            raise ValueError(f"{self.name}: unsafe start/goal loiter")

    def build_route(self):
        began = perf_counter()
        self.attempts += 1
        budget = self.retry_iterations if self.search_failed else self.iterations
        result = None
        try:
            if self.traffic_replan:
                result = plan_safe_rrt_star_traffic(self.world, self.request.start, self.request.goal,
                    self.request.radius, self.request.max_angle, start_pose=self.fixed_start,
                    departure=self.fixed_departure, speed=self.request.speed,
                    reservations=tuple(self._reservation(peer) for peer in self.peers),
                    separation=self.separation, horizon=self.horizon, iterations=budget,
                    relative_gap=self.relative_gap,
                    seed=self.seed + 1009 * (self.attempts - 1))
            else:
                result = plan_safe_rrt_star(self.world, self.request.start, self.request.goal,
                                          self.request.radius, self.request.max_angle,
                                          iterations=budget,
                                          relative_gap=self.relative_gap,
                                          seed=self.seed + 1009 * (self.attempts - 1), start_direction="CCW")
            return result
        except NoPathError as error:
            self.search_failed = True
            return error
        finally:
            elapsed = 1000 * (perf_counter() - began)
            self.routing_ms += elapsed
            self.planning_calls.append(PlanningCall(self.name, self.attempts,
                                       "traffic" if self.traffic_replan else "static", elapsed,
                                       result is not None, budget,
                                       result.iterations if result is not None else None))

    def _send(self, kind, peer, version=None, flight=None):
        self.transport.send(Message(kind, self.name, peer,
                                    self.version if version is None else version, flight))

    def _broadcast(self, kind, flight):
        for peer in self.peers:
            self._send(kind, peer, flight=flight)

    def _reservation(self, peer):
        return self.received[peer].flight if peer in self.received else self.holds[peer]

    def _safe(self, a, b):
        return (loiter_disk_distance(a.request.goal, b.request.goal) > self.separation + 1e-4
                and check_separation(a, b, self.separation, self.horizon).safe)

    def _fail(self, reason):
        self.failed = reason
        self.pending = None
        self.transport.log(self.name, "FAILED", detail=reason)
        self._broadcast("FAILED", None)

    def _retry_route(self):
        if self.attempts >= self.max_attempts:
            self._fail("no safe spatial route at the fixed departure within the local attempt budget")
            return
        self.traffic_replan = self.fixed_start is not None
        known = ",".join(f"{p}:v{m.version}" for p, m in sorted(self.received.items()))
        self.transport.log(self.name, "REPLAN", detail=(
            f"time-aware RRT*; fixed_departure={self.fixed_departure:.6f}; received={known or 'none'}"
            if self.traffic_replan else "retry initial static route search"))
        self.busy = True
        before = self.routing_ms
        route = self.build_route()
        self.transport.local(self.transport.now + (self.routing_ms - before) / 1000,
                             self.name, ("READY", route))

    def _propose(self):
        if (self.busy or self.route is None
                or any(p not in self.received and p not in self.failed_peers for p in self.higher)):
            return
        if self.fixed_departure > self.max_delay:
            self._fail("earliest phase-compatible departure exceeds max_delay")
            return
        flight = ScheduledFlight(self.request, self.route, self.fixed_departure)
        conflict = next((peer for peer in self.peers if not self._safe(flight, self._reservation(peer))), None)
        if flight.arrival > self.horizon or conflict is not None:
            self.transport.log(self.name, "CONFLICT", conflict or "",
                               detail=f"departure={flight.departure:.6f}; attempt={self.attempts}")
            self._retry_route()  # Geometry changes; departure state/time DO NOT.
            return
        self.version += 1
        self.pending, self.acks = flight, set()
        self.transport.log(self.name, "PROPOSE", version=self.version,
                           detail=f"departure={flight.departure:.6f}")
        self._broadcast("PROPOSE", flight)
        if not self.peers:
            self._commit()

    def _commit(self):
        self.committed, self.pending = self.pending, None
        self.transport.log(self.name, "COMMIT", version=self.version)
        self._broadcast("COMMIT", self.committed)

    def receive(self, message):
        """Process one event without inspecting another node's objects."""
        if message.sender not in self.peers or message.receiver != self.name:
            return
        self.transport.log(self.name, "RECEIVE", message.sender, message.version, message.kind,
                           packet_id=message.packet_id)
        peer = message.sender
        if message.kind == "PROPOSE":
            flight = message.flight
            prior = self.votes.get(peer)
            if (flight is None or flight.request != self.roster[peer]
                    or flight.departure < self.window or flight.arrival > self.horizon
                    or (prior and message.version < prior[0])):
                return
            if prior and message.version == prior[0] and flight != prior[1]:
                return  # Never acknowledge different content under the same version.
            own = self.committed or self.holds[self.name]
            if (peer > self.name and self.committed is None and not self.failed) or not self._safe(flight, own):
                self._send("NACK", peer, message.version)
            else:
                self.votes[peer] = (message.version, flight)
                self._send("ACK", peer, message.version)
        elif message.kind == "ACK":
            if self.pending is not None and message.version == self.version:
                self.acks.add(peer)
                if self.acks == set(self.peers):
                    self._commit()
        elif message.kind == "NACK":
            if self.pending is not None and message.version == self.version:
                self.pending = None
                self.acks.clear()
                self._retry_route()
        elif message.kind == "COMMIT":
            prior = self.votes.get(peer)
            old = self.received.get(peer)
            if (prior == (message.version, message.flight)
                    and (old is None or (old.version == message.version and old.flight == message.flight))):
                self.received[peer] = message
                self._send("RECORDED", peer, message.version)
        elif message.kind == "RECORDED":
            if self.committed is not None and message.version == self.version:
                self.confirmations.add(peer)
        elif message.kind == "FAILED" and peer not in self.received:
            self.failed_peers.add(peer)

    def tick(self):
        if self.failed:
            self._broadcast("FAILED", None)
            return
        if self.committed is not None:
            for peer in self.peers:
                if peer not in self.confirmations:
                    self._send("COMMIT", peer, flight=self.committed)
        elif self.pending is not None:
            for peer in self.peers:
                if peer not in self.acks:
                    self._send("PROPOSE", peer, flight=self.pending)
        else:
            self._propose()

    def ready(self, route):
        self.busy = False
        if isinstance(route, NoPathError):
            self._retry_route()
        else:
            self.route = route
            if self.fixed_start is None:
                self.fixed_start = route.paths[0].start
                base = departure_phase_delay(self.request, route)
                period = tau * self.request.start.radius / self.request.speed
                self.fixed_departure = base + max(0, ceil((self.minimum_departure-base)/period)) * period
            self.transport.route_history.append(RouteSnapshot(
                self.transport.now, self.name, self.attempts,
                ScheduledFlight(self.request, route, self.fixed_departure)))
            self.transport.log(self.name, "ROUTE_READY", detail=f"attempt={self.attempts}")
            self.tick()


def simulate_decentralized(world, requests, *, separation=40.0, max_delay=120.0,
                           planning_window=5.0, horizon=600.0, iterations=600, seed=7,
                           max_route_attempts=3, retry_iterations=None, network=None,
                           relative_gap=0.0) -> DecentralizedResult:
    """Simulate independent planners and peer messages, with a final observer audit.

    All starts must have separated safe disks, reserved from t=0 indefinitely.
    Each missing initial_phase defaults explicitly to zero (east point, CCW).
    The execution window begins at planning_window, allowing negotiation while
    agents keep circling. Timeout does not undo commits. Candidate routes are
    spatially replanned with received reservations inside the RRT* edge/goal
    checks; rewiring revalidates descendant times. Departure is fixed after the
    initial route, so conflicts cannot be hidden by extra laps. This is not a
    complete ADPP implementation or an online flight controller.
    horizon must cover every accepted arrival; terminal disks certify beyond it.
    retry_iterations optionally increases the budget after a search returns no
    path, including subsequent traffic replans. It never restarts the group or
    changes departures. Every call and its actual budget remain in the result.
    relative_gap optionally stops each local search after a safe route reaches
    a physical length bound; zero retains the fixed-budget reference behavior.
    """
    began = perf_counter()
    _validate_relative_gap(relative_gap)
    requests = tuple(replace(r, initial_phase=0.0) if r.initial_phase is None else r for r in requests)
    if (not requests or len({r.name for r in requests}) != len(requests)
            or not all(isfinite(v) for v in (separation, max_delay, planning_window, horizon))
            or separation <= 0 or not 0 < planning_window <= max_delay < horizon
            or not isinstance(iterations, int) or iterations <= 0
            or (retry_iterations is not None and (not isinstance(retry_iterations, int) or retry_iterations < iterations))
            or not isinstance(max_route_attempts, int) or max_route_attempts <= 0):
        raise ValueError("invalid agents, times, separation or search budgets")
    roster = {r.name: r for r in requests}
    for i, r in enumerate(requests):
        if r.earliest_departure > max_delay:
            raise ValueError("earliest departure exceeds max_delay")
        for other in requests[:i]:
            if (loiter_disk_distance(r.start, other.start) <= separation + 1e-4
                    or loiter_disk_distance(r.goal, other.goal) <= separation + 1e-4):
                raise ValueError("initial and terminal loiter disks must be pairwise separated")
    transport = SimulatedTransport(network or NetworkSettings())
    nodes = {name: LocalPlanner(world, roster[name], roster, transport, separation=separation,
                               max_delay=max_delay, planning_window=planning_window, horizon=horizon,
                               iterations=iterations, seed=seed + i, max_route_attempts=max_route_attempts,
                               retry_iterations=retry_iterations, relative_gap=relative_gap)
             for i, name in enumerate(sorted(roster))}
    # Only the local object executes each search. No shared route/decision table.
    routing_started = perf_counter()
    initial_flights = []
    with ThreadPoolExecutor(max_workers=min(8, len(nodes))) as workers:
        futures = {workers.submit(node.build_route): node for node in nodes.values()}
        for future in as_completed(futures):
            node = futures[future]
            route = future.result()
            transport.local(perf_counter() - routing_started, node.name, ("READY", route))
            if not isinstance(route, NoPathError):
                base = departure_phase_delay(node.request, route)
                period = tau * node.request.start.radius / node.request.speed
                laps = max(0, ceil((node.minimum_departure - base) / period))
                initial_flights.append(ScheduledFlight(node.request, route, base + laps * period))
    routing_ms = 1000 * (perf_counter() - routing_started)
    negotiation_started = perf_counter()
    for name in nodes:
        transport.local(0.0, name, "TICK")
    while transport.queue:
        at, _, name, payload = heappop(transport.queue)
        if at >= planning_window:
            transport.now = planning_window
            break
        transport.now = at
        node = nodes[name]
        if payload == "TICK":
            node.tick()
            transport.local(at + transport.settings.retry_interval, name, "TICK")
        elif isinstance(payload, Message):
            node.receive(payload)
        else:
            node.ready(payload[1])
        if all(n.failed or (n.committed is not None and n.confirmations == set(n.peers))
               for n in nodes.values()):
            break
    scheduling_ms = 1000 * (perf_counter() - negotiation_started)
    flights = tuple(n.committed or n.holds[n.name] for n in nodes.values())
    plan = FleetPlan(flights, separation, horizon, routing_ms, scheduling_ms)
    if not plan.validate(world):
        raise RuntimeError("independent final reservation audit failed")
    complete = all(not n.failed and n.committed is not None and n.confirmations == set(n.peers)
                   for n in nodes.values())
    if not complete:
        for node in nodes.values():
            if not node.failed and (node.committed is None or node.confirmations != set(node.peers)):
                transport.log(node.name, "TIMEOUT", detail="execution window reached; reservation retained")
    result = DecentralizedResult(plan, tuple(transport.events), transport.now,
                                1000 * (perf_counter() - began),
                                tuple((n.name, n.routing_ms) for n in nodes.values()),
                                tuple((n.name, n.attempts) for n in nodes.values()),
                                agreement_complete=complete, planning_window=planning_window,
                                network=transport.settings,
                                initial_flights=tuple(sorted(initial_flights, key=lambda f: f.request.name)),
                                route_history=tuple(transport.route_history),
                                planning_calls=tuple(c for n in nodes.values() for c in n.planning_calls))
    if not complete:
        reason = "; ".join(f"{n.name}: {n.failed or 'agreement/receipts not completed before execution window'}"
                           for n in nodes.values() if n.failed or n.committed is None
                           or n.confirmations != set(n.peers))
        raise DecentralizedPlanningError(reason, plan, transport.events, result=result)
    return result

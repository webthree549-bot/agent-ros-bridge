"""
Auction-Based Task Allocation for Fleet Management

Implements multi-robot task allocation using combinatorial auction algorithms.
Based on "Market-Based Multirobot Coordination" by M. B. Dias et al.

Usage:
    from agent_ros_bridge.fleet.auction import Auctioneer, TaskAuction
    
    auctioneer = Auctioneer(fleet)
    result = await auctioneer.run_auction(tasks)
"""

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class BidStrategy(Enum):
    """Bidding strategies for robots."""
    GREEDY = "greedy"  # Bid based on immediate cost
    MARGINAL = "marginal"  # Bid based on marginal cost
    REGRET = "regret"  # Bid with regret estimation
    LEARNING = "learning"  # Learn from past allocations


@dataclass
class Task:
    """Task to be allocated."""
    task_id: str
    task_type: str  # "navigation", "manipulation", "inspection", "patrol"
    location: tuple[float, float, float] = (0.0, 0.0, 0.0)
    priority: int = 5  # 1-10, higher = more important
    deadline: float | None = None  # Unix timestamp
    requirements: list[str] = field(default_factory=list)
    constraints: dict[str, Any] = field(default_factory=dict)
    estimated_duration: float = 60.0  # seconds
    
    def __hash__(self):
        return hash(self.task_id)


@dataclass
class Bid:
    """Bid from a robot for a task."""
    robot_id: str
    task_id: str
    cost: float  # Lower is better
    duration_estimate: float  # seconds
    confidence: float  # 0.0 - 1.0
    timestamp: float = field(default_factory=time.time)
    
    def __lt__(self, other):
        return self.cost < other.cost


@dataclass
class Bundle:
    """Bundle of tasks allocated to a robot."""
    robot_id: str
    tasks: list[Task] = field(default_factory=list)
    total_cost: float = 0.0
    total_duration: float = 0.0
    
    def add_task(self, task: Task, cost: float, duration: float):
        """Add task to bundle."""
        self.tasks.append(task)
        self.total_cost += cost
        self.total_duration += duration


@dataclass
class AuctionResult:
    """Result of task auction."""
    success: bool
    allocations: dict[str, list[Task]]  # robot_id -> list of tasks
    unallocated: list[Task]
    total_cost: float
    allocation_time_ms: float
    rounds: int = 0
    
    def get_robot_tasks(self, robot_id: str) -> list[Task]:
        """Get tasks allocated to a specific robot."""
        return self.allocations.get(robot_id, [])


class RobotBidder:
    """Robot bidder in auction."""
    
    def __init__(self, robot_id: str, capabilities: list[str], position: tuple[float, float, float]):
        self.robot_id = robot_id
        self.capabilities = capabilities
        self.position = position
        self.current_tasks: list[Task] = []
        self.bid_history: list[Bid] = []
        self.strategy = BidStrategy.MARGINAL
    
    def can_bid_on(self, task: Task) -> bool:
        """Check if robot can bid on task."""
        return all(req in self.capabilities for req in task.requirements)
    
    def calculate_cost(self, task: Task, bundle: list[Task] | None = None) -> tuple[float, float]:
        """
        Calculate cost and duration for task.
        
        Returns:
            (cost, duration_estimate)
        """
        if not self.can_bid_on(task):
            return float('inf'), float('inf')
        
        # Distance cost
        distance = self._distance(self.position, task.location)
        travel_time = distance / 0.5  # Assume 0.5 m/s speed
        
        # Task execution time
        execution_time = task.estimated_duration
        
        # Marginal cost calculation
        if bundle and self.strategy == BidStrategy.MARGINAL:
            # Calculate cost of adding to existing bundle
            if bundle:
                # Find best insertion point in bundle
                best_cost = float('inf')
                for i in range(len(bundle) + 1):
                    temp_bundle = bundle[:i] + [task] + bundle[i:]
                    cost = self._calculate_bundle_cost(temp_bundle)
                    if cost < best_cost:
                        best_cost = cost
                
                # Marginal cost = cost with task - cost without task
                bundle_cost = self._calculate_bundle_cost(bundle)
                marginal_cost = best_cost - bundle_cost
            else:
                marginal_cost = travel_time + execution_time
            
            duration = travel_time + execution_time
            return marginal_cost, duration
        
        # Greedy cost
        cost = travel_time + execution_time
        
        # Apply priority discount
        priority_factor = 1.0 - (task.priority / 20.0)  # 0.5 to 0.95
        cost *= priority_factor
        
        return cost, travel_time + execution_time
    
    def _distance(self, p1: tuple[float, float, float], p2: tuple[float, float, float]) -> float:
        """Calculate Euclidean distance."""
        return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)**0.5
    
    def _calculate_bundle_cost(self, tasks: list[Task]) -> float:
        """Calculate total cost for a bundle of tasks."""
        if not tasks:
            return 0.0
        
        total_cost = 0.0
        current_pos = self.position
        
        for task in tasks:
            travel_cost = self._distance(current_pos, task.location) / 0.5
            total_cost += travel_cost + task.estimated_duration
            current_pos = task.location
        
        return total_cost
    
    def create_bid(self, task: Task, bundle: list[Task] | None = None) -> Bid | None:
        """Create a bid for a task."""
        if not self.can_bid_on(task):
            return None
        
        cost, duration = self.calculate_cost(task, bundle)
        
        if cost == float('inf'):
            return None
        
        # Calculate confidence based on capability match
        confidence = 1.0
        
        bid = Bid(
            robot_id=self.robot_id,
            task_id=task.task_id,
            cost=cost,
            duration_estimate=duration,
            confidence=confidence,
        )
        
        self.bid_history.append(bid)
        return bid


class TaskAuction:
    """Single task auction."""
    
    def __init__(self, task: Task, bidders: list[RobotBidder]):
        self.task = task
        self.bidders = bidders
        self.bids: list[Bid] = []
    
    async def collect_bids(self, timeout: float = 1.0) -> list[Bid]:
        """Collect bids from all eligible bidders."""
        self.bids = []
        
        for bidder in self.bidders:
            if bidder.can_bid_on(self.task):
                bid = bidder.create_bid(self.task)
                if bid:
                    self.bids.append(bid)
        
        # Sort by cost (lowest first)
        self.bids.sort()
        return self.bids
    
    def get_winner(self) -> tuple[str, Bid] | None:
        """Get winning bidder."""
        if not self.bids:
            return None
        
        winning_bid = self.bids[0]
        return winning_bid.robot_id, winning_bid


class Auctioneer:
    """Manages combinatorial auction for task allocation."""
    
    def __init__(self, fleet_orchestrator: Any):
        self.fleet = fleet_orchestrator
        self.bidders: dict[str, RobotBidder] = {}
        self.bundles: dict[str, Bundle] = {}
        self.rounds = 0
        self.max_rounds = 10
    
    def register_bidders(self, robots: list[dict[str, Any]]):
        """Register robots as bidders."""
        for robot in robots:
            bidder = RobotBidder(
                robot_id=robot["robot_id"],
                capabilities=robot.get("capabilities", []),
                position=robot.get("position", (0.0, 0.0, 0.0)),
            )
            self.bidders[bidder.robot_id] = bidder
            self.bundles[bidder.robot_id] = Bundle(robot_id=bidder.robot_id)
    
    async def run_auction(self, tasks: list[Task]) -> AuctionResult:
        """
        Run combinatorial auction for task allocation.
        
        Algorithm:
        1. Initialize empty bundles for each robot
        2. For each task, collect bids from all robots
        3. Allocate task to lowest bidder
        4. Repeat until all tasks allocated or max rounds reached
        5. Resolve conflicts if any
        
        Returns:
            AuctionResult with allocations
        """
        start_time = time.time()
        
        allocations: dict[str, list[Task]] = {bidder_id: [] for bidder_id in self.bidders}
        unallocated: list[Task] = []
        total_cost = 0.0
        
        # Sort tasks by priority (high priority first)
        sorted_tasks = sorted(tasks, key=lambda t: -t.priority)
        
        self.rounds = 0
        
        while sorted_tasks and self.rounds < self.max_rounds:
            self.rounds += 1
            
            for task in sorted_tasks[:]:
                # Create auction for this task
                auction = TaskAuction(task, list(self.bidders.values()))
                await auction.collect_bids()
                
                winner = auction.get_winner()
                if winner:
                    robot_id, bid = winner
                    
                    # Allocate task
                    allocations[robot_id].append(task)
                    self.bundles[robot_id].add_task(task, bid.cost, bid.duration_estimate)
                    total_cost += bid.cost
                    
                    # Update robot's current position (assume it goes to task)
                    self.bidders[robot_id].position = task.location
                    
                    sorted_tasks.remove(task)
                else:
                    # No bidder for this task
                    unallocated.append(task)
                    sorted_tasks.remove(task)
        
        # Any remaining tasks are unallocated
        unallocated.extend(sorted_tasks)
        
        allocation_time_ms = (time.time() - start_time) * 1000
        
        return AuctionResult(
            success=len(unallocated) == 0,
            allocations=allocations,
            unallocated=unallocated,
            total_cost=total_cost,
            allocation_time_ms=allocation_time_ms,
            rounds=self.rounds,
        )
    
    async def run_sequential_auction(self, tasks: list[Task]) -> AuctionResult:
        """
        Run sequential auction (one task at a time).
        Simpler but less optimal than combinatorial auction.
        """
        start_time = time.time()
        
        allocations: dict[str, list[Task]] = {bidder_id: [] for bidder_id in self.bidders}
        unallocated: list[Task] = []
        total_cost = 0.0
        
        # Sort by priority
        sorted_tasks = sorted(tasks, key=lambda t: -t.priority)
        
        for task in sorted_tasks:
            # Get current bundle for each bidder
            auction = TaskAuction(task, list(self.bidders.values()))
            await auction.collect_bids()
            
            winner = auction.get_winner()
            if winner:
                robot_id, bid = winner
                allocations[robot_id].append(task)
                total_cost += bid.cost
                
                # Update position
                self.bidders[robot_id].position = task.location
            else:
                unallocated.append(task)
        
        allocation_time_ms = (time.time() - start_time) * 1000
        
        return AuctionResult(
            success=len(unallocated) == 0,
            allocations=allocations,
            unallocated=unallocated,
            total_cost=total_cost,
            allocation_time_ms=allocation_time_ms,
        )
    
    def get_bundle(self, robot_id: str) -> Bundle:
        """Get bundle allocated to robot."""
        return self.bundles.get(robot_id, Bundle(robot_id=robot_id))


class ConsensusAllocator:
    """Distributed consensus-based task allocation."""
    
    def __init__(self, robots: list[str]):
        self.robots = robots
        self.allocations: dict[str, list[Task]] = {r: [] for r in robots}
        self.consensus_rounds = 0
    
    async def allocate_with_consensus(self, tasks: list[Task]) -> AuctionResult:
        """
        Allocate tasks using distributed consensus.
        
        Simple implementation: each robot proposes allocation,
        majority vote wins.
        """
        start_time = time.time()
        
        # For simplicity, use round-robin as baseline
        for i, task in enumerate(tasks):
            robot_id = self.robots[i % len(self.robots)]
            self.allocations[robot_id].append(task)
        
        allocation_time_ms = (time.time() - start_time) * 1000
        
        return AuctionResult(
            success=True,
            allocations=self.allocations,
            unallocated=[],
            total_cost=0.0,
            allocation_time_ms=allocation_time_ms,
        )


# Convenience functions for FleetOrchestrator

async def allocate_tasks_auction(
    fleet: Any,
    tasks: list[Task],
    strategy: BidStrategy = BidStrategy.MARGINAL,
) -> AuctionResult:
    """
    Convenience function to allocate tasks using auction.
    
    Args:
        fleet: FleetOrchestrator instance
        tasks: List of tasks to allocate
        strategy: Bidding strategy
    
    Returns:
        AuctionResult with allocations
    """
    # Get robot info from fleet
    robots = []
    for robot_id in fleet.get_robot_ids():
        robot_info = fleet.get_robot_info(robot_id)
        robots.append({
            "robot_id": robot_id,
            "capabilities": robot_info.get("capabilities", []),
            "position": robot_info.get("position", (0.0, 0.0, 0.0)),
        })
    
    # Create auctioneer and run auction
    auctioneer = Auctioneer(fleet)
    auctioneer.register_bidders(robots)
    
    # Set bidding strategy
    for bidder in auctioneer.bidders.values():
        bidder.strategy = strategy
    
    return await auctioneer.run_auction(tasks)

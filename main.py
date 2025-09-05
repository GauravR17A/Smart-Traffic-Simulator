import pygame
import random
import math
from enum import Enum
from collections import deque
import json
import os
import sys

# Initialize Pygame
pygame.init()

# ===================== Configuration =====================
class Config:
    # Display
    WINDOW_WIDTH = 1200
    WINDOW_HEIGHT = 800
    FPS = 60  # render FPS

    # Colors
    ROAD_COLOR = (60, 60, 60)
    LANE_COLOR = (255, 255, 255)
    GRASS_COLOR = (34, 139, 34)
    CAR_COLORS = [(255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 165, 0), (128, 0, 128)]
    AMBULANCE_COLOR = (255, 255, 255)

    # Signal colors
    RED = (255, 0, 0)
    YELLOW = (255, 255, 0)
    GREEN = (0, 255, 0)
    SIGNAL_OFF = (50, 50, 50)

    # UI colors
    TEXT_COLOR = (255, 255, 255)
    BUTTON_COLOR = (70, 70, 70)
    BUTTON_HOVER = (100, 100, 100)

    # Road geometry
    ROAD_WIDTH = 200
    LANE_WIDTH = 40
    INTERSECTION_SIZE = 200

    # Vehicle dynamics (baseline @ 60 FPS; scaled by dt_sim*FPS)
    CAR_WIDTH = 30
    CAR_HEIGHT = 15
    CAR_MAX_SPEED = 4.0       # px/frame baseline
    CAR_ACCELERATION = 0.10   # px/frame^2 baseline
    SAFE_DISTANCE = 35

    # Spawn settings (legacy per-frame converted to per-second internally)
    BASE_SPAWN_RATE = 0.02
    MAIN_ROAD_MULTIPLIER = 1.5
    SIDE_ROAD_MULTIPLIER = 1.0

    # Signal timing (seconds)
    FIXED_GREEN_TIME = 20
    FIXED_YELLOW_TIME = 3
    FIXED_ALL_RED_TIME = 2

    # AI Controller
    AI_UPDATE_INTERVAL = 2.0  # seconds
    MIN_GREEN_TIME = 12
    MAX_GREEN_TIME = 50
    COOLDOWN_TIME = 6
    ANTI_STARVATION_TIME = 120
    PRESSURE_ALPHA = 0.3  # arrival rate weight
    PRESSURE_BETA  = 0.4  # wait time weight
    HYSTERESIS_MARGIN = 0.5

class SignalState(Enum):
    GREEN = "GREEN"
    YELLOW = "YELLOW"
    RED = "RED"
    ALL_RED = "ALL_RED"

class Direction(Enum):
    NORTH = 0
    SOUTH = 1
    EAST = 2
    WEST = 3

# ===================== Car =====================
class Car:
    def __init__(self, x, y, direction, is_ambulance=False, spawn_sim_time=0.0):
        self.x = float(x)
        self.y = float(y)
        self.direction = direction
        self.speed = 0.0
        self.max_speed = Config.CAR_MAX_SPEED
        self.width = Config.CAR_WIDTH
        self.height = Config.CAR_HEIGHT
        self.color = Config.AMBULANCE_COLOR if is_ambulance else random.choice(Config.CAR_COLORS)
        self.is_ambulance = is_ambulance
        self.spawn_time = spawn_sim_time
        self.waiting_time = 0.0
        self.is_stopped = False
        self.exit_time = None

    def update(self, cars, signal_controller, dt_sim):
        # Keep per-frame tuned params consistent under variable dt
        step_scale = dt_sim * Config.FPS

        old_x, old_y = self.x, self.y

        # Signal compliance (ambulances preempt)
        should_stop = self.should_stop_at_signal(signal_controller)
        if self.is_ambulance:
            should_stop = False

        # Car-following
        car_ahead = self.get_car_ahead(cars)

        if should_stop or car_ahead:
            self.speed = max(0.0, self.speed - Config.CAR_ACCELERATION * 2.0 * step_scale)
            self.is_stopped = (self.speed <= 1e-3)
        else:
            self.speed = min(self.max_speed, self.speed + Config.CAR_ACCELERATION * step_scale)
            self.is_stopped = False

        # Integrate position
        delta = self.speed * step_scale
        if self.direction == Direction.NORTH:
            self.y -= delta
        elif self.direction == Direction.SOUTH:
            self.y += delta
        elif self.direction == Direction.EAST:
            self.x += delta
        elif self.direction == Direction.WEST:
            self.x -= delta

        # Waiting time (sim-time)
        if self.is_stopped and (abs(self.x - old_x) < 1e-3 and abs(self.y - old_y) < 1e-3):
            self.waiting_time += dt_sim

    def should_stop_at_signal(self, signal_controller):
        cx = Config.WINDOW_WIDTH // 2
        cy = Config.WINDOW_HEIGHT // 2
        half = Config.INTERSECTION_SIZE // 2
        stop_band = 50  # window around stop line

        if self.direction == Direction.NORTH:
            stop_y = cy + half
            if (stop_y - stop_band) < self.y < (stop_y + stop_band):
                return signal_controller.get_signal_state('main') != SignalState.GREEN
        elif self.direction == Direction.SOUTH:
            stop_y = cy - half
            if (stop_y - stop_band) < self.y < (stop_y + stop_band):
                return signal_controller.get_signal_state('main') != SignalState.GREEN
        elif self.direction == Direction.EAST:
            stop_x = cx - half
            if (stop_x - stop_band) < self.x < (stop_x + stop_band):
                return signal_controller.get_signal_state('side') != SignalState.GREEN
        elif self.direction == Direction.WEST:
            stop_x = cx + half
            if (stop_x - stop_band) < self.x < (stop_x + stop_band):
                return signal_controller.get_signal_state('side') != SignalState.GREEN

        return False

    def get_car_ahead(self, cars):
        for other in cars:
            if other is self or other.direction != self.direction:
                continue

            if self.direction == Direction.NORTH:
                if other.y < self.y and abs(other.x - self.x) < Config.LANE_WIDTH:
                    if 0 < (self.y - other.y) < Config.SAFE_DISTANCE:
                        return other
            elif self.direction == Direction.SOUTH:
                if other.y > self.y and abs(other.x - self.x) < Config.LANE_WIDTH:
                    if 0 < (other.y - self.y) < Config.SAFE_DISTANCE:
                        return other
            elif self.direction == Direction.EAST:
                if other.x > self.x and abs(other.y - self.y) < Config.LANE_WIDTH:
                    if 0 < (other.x - self.x) < Config.SAFE_DISTANCE:
                        return other
            elif self.direction == Direction.WEST:
                if other.x < self.x and abs(other.y - self.y) < Config.LANE_WIDTH:
                    if 0 < (self.x - other.x) < Config.SAFE_DISTANCE:
                        return other
        return None

    def draw(self, screen):
        rect = pygame.Rect(
            int(self.x) - self.width // 2,
            int(self.y) - self.height // 2,
            self.width,
            self.height
        )
        if self.direction in (Direction.NORTH, Direction.SOUTH):
            rect.width, rect.height = self.height, self.width

        pygame.draw.rect(screen, self.color, rect)
        pygame.draw.rect(screen, (0, 0, 0), rect, 2)

        if self.is_ambulance:
            s = 8
            pygame.draw.line(screen, Config.RED, (int(self.x - s//2), int(self.y)), (int(self.x + s//2), int(self.y)), 3)
            pygame.draw.line(screen, Config.RED, (int(self.x), int(self.y - s//2)), (int(self.x), int(self.y + s//2)), 3)

    def is_off_screen(self):
        m = 100
        return (self.x < -m or self.x > Config.WINDOW_WIDTH + m or
                self.y < -m or self.y > Config.WINDOW_HEIGHT + m)

# ===================== Signal Controller =====================
class SignalController:
    def __init__(self, controller_type="fixed", start_sim_time=0.0):
        self.controller_type = controller_type
        self.main_signal = SignalState.GREEN
        self.side_signal = SignalState.RED

        # Sim-time clocks
        self.signal_timer = 0.0
        self.current_green_time = 0.0
        self.last_ai_update = start_sim_time
        self.last_switch_time = start_sim_time - Config.COOLDOWN_TIME

        # Last green timestamps
        self.main_last_green = start_sim_time
        self.side_last_green = start_sim_time

        # Switching FSM
        self.switching = False
        self.switch_stage = 0
        self.switch_start_time = start_sim_time

        # Arrival history (sim-time)
        self.arrival_history = {'main': deque(maxlen=200), 'side': deque(maxlen=200)}

    def update(self, cars, dt_sim, sim_time):
        self.signal_timer += dt_sim
        self.current_green_time += dt_sim

        if self.controller_type == "fixed":
            self.update_fixed_signals()
        else:
            self.update_ai_signals(cars, sim_time)

    def update_fixed_signals(self):
        cycle = (Config.FIXED_GREEN_TIME * 2 +
                 Config.FIXED_YELLOW_TIME * 2 +
                 Config.FIXED_ALL_RED_TIME * 2)
        pos = self.signal_timer % cycle

        if pos < Config.FIXED_GREEN_TIME:
            self.main_signal = SignalState.GREEN
            self.side_signal = SignalState.RED
        elif pos < Config.FIXED_GREEN_TIME + Config.FIXED_YELLOW_TIME:
            self.main_signal = SignalState.YELLOW
            self.side_signal = SignalState.RED
        elif pos < Config.FIXED_GREEN_TIME + Config.FIXED_YELLOW_TIME + Config.FIXED_ALL_RED_TIME:
            self.main_signal = SignalState.ALL_RED
            self.side_signal = SignalState.ALL_RED
        elif pos < Config.FIXED_GREEN_TIME * 2 + Config.FIXED_YELLOW_TIME + Config.FIXED_ALL_RED_TIME:
            self.main_signal = SignalState.RED
            self.side_signal = SignalState.GREEN
        elif pos < Config.FIXED_GREEN_TIME * 2 + Config.FIXED_YELLOW_TIME * 2 + Config.FIXED_ALL_RED_TIME:
            self.main_signal = SignalState.RED
            self.side_signal = SignalState.YELLOW
        else:
            self.main_signal = SignalState.ALL_RED
            self.side_signal = SignalState.ALL_RED

    def update_ai_signals(self, cars, sim_time):
        if self.switching:
            self.handle_signal_switching(sim_time)
            return

        if (sim_time - self.last_ai_update) >= Config.AI_UPDATE_INTERVAL:
            self.make_ai_decision(cars, sim_time)
            self.last_ai_update = sim_time

        # Anti-starvation
        active = 'main' if self.main_signal == SignalState.GREEN else 'side'
        waiting = 'side' if active == 'main' else 'main'
        waiting_last = self.main_last_green if waiting == 'main' else self.side_last_green
        waiting_time = sim_time - waiting_last

        if waiting_time > Config.ANTI_STARVATION_TIME and self.current_green_time > Config.MIN_GREEN_TIME:
            self.initiate_signal_switch(sim_time)

    def make_ai_decision(self, cars, sim_time):
        main_p = self.calculate_pressure(cars, 'main', sim_time)
        side_p = self.calculate_pressure(cars, 'side', sim_time)

        active_is_main = (self.main_signal == SignalState.GREEN)
        active_p = main_p if active_is_main else side_p
        waiting_p = side_p if active_is_main else main_p

        should_switch = False
        time_since_switch = sim_time - self.last_switch_time

        if (self.current_green_time >= Config.MIN_GREEN_TIME and
            time_since_switch >= Config.COOLDOWN_TIME):
            if waiting_p > (active_p + Config.HYSTERESIS_MARGIN):
                should_switch = True

        if self.current_green_time >= Config.MAX_GREEN_TIME:
            should_switch = True

        if should_switch:
            self.initiate_signal_switch(sim_time)

    def calculate_pressure(self, cars, road, sim_time):
        q = self.get_queue_length(cars, road)
        arr_rate = self.get_arrival_rate(road, sim_time)   # per second
        avg_wait = self.get_average_wait_time(cars, road)  # seconds
        avg_wait_norm = min(avg_wait / 60.0, 1.0)
        return q + (Config.PRESSURE_ALPHA * arr_rate * 10.0) + (Config.PRESSURE_BETA * avg_wait_norm * 10.0)

    def get_queue_length(self, cars, road):
        cx = Config.WINDOW_WIDTH // 2
        cy = Config.WINDOW_HEIGHT // 2
        count = 0
        for car in cars:
            if road == 'main':
                if car.direction in (Direction.NORTH, Direction.SOUTH):
                    if car.is_stopped and abs(car.x - cx) < Config.LANE_WIDTH * 1.5:
                        count += 1
            else:
                if car.direction in (Direction.EAST, Direction.WEST):
                    if car.is_stopped and abs(car.y - cy) < Config.LANE_WIDTH * 1.5:
                        count += 1
        return count

    def get_arrival_rate(self, road, sim_time):
        hist = self.arrival_history[road]
        if len(hist) < 2:
            return 0.0
        recent = sum(1 for t in hist if sim_time - t < 10.0)  # last 10 sec
        return recent / 10.0

    def get_average_wait_time(self, cars, road):
        waits = []
        cx = Config.WINDOW_WIDTH // 2
        cy = Config.WINDOW_HEIGHT // 2
        for car in cars:
            if road == 'main' and car.direction in (Direction.NORTH, Direction.SOUTH):
                if abs(car.x - cx) < Config.LANE_WIDTH * 2:
                    waits.append(car.waiting_time)
            elif road == 'side' and car.direction in (Direction.EAST, Direction.WEST):
                if abs(car.y - cy) < Config.LANE_WIDTH * 2:
                    waits.append(car.waiting_time)
        return (sum(waits) / len(waits)) if waits else 0.0

    def initiate_signal_switch(self, sim_time):
        self.switching = True
        self.switch_stage = 0
        self.switch_start_time = sim_time
        self.last_switch_time = sim_time
        # record who had green last
        if self.main_signal == SignalState.GREEN:
            self.main_last_green = sim_time
        elif self.side_signal == SignalState.GREEN:
            self.side_last_green = sim_time
        self.current_green_time = 0.0

    def handle_signal_switching(self, sim_time):
        elapsed = sim_time - self.switch_start_time

        if self.switch_stage == 0:  # Yellow
            if self.main_signal == SignalState.GREEN:
                self.main_signal = SignalState.YELLOW
            elif self.side_signal == SignalState.GREEN:
                self.side_signal = SignalState.YELLOW
            if elapsed >= Config.FIXED_YELLOW_TIME:
                self.switch_stage = 1
                self.switch_start_time = sim_time

        elif self.switch_stage == 1:  # All red
            self.main_signal = SignalState.ALL_RED
            self.side_signal = SignalState.ALL_RED
            if elapsed >= Config.FIXED_ALL_RED_TIME:
                self.switch_stage = 2
                self.switch_start_time = sim_time

        elif self.switch_stage == 2:  # Grant green to the road that wasn't just green
            if self.side_last_green < self.main_last_green:
                self.side_signal = SignalState.GREEN
                self.main_signal = SignalState.RED
            else:
                self.main_signal = SignalState.GREEN
                self.side_signal = SignalState.RED
            self.switching = False
            self.current_green_time = 0.0

    def get_signal_state(self, road):
        return self.main_signal if road == 'main' else self.side_signal

    def record_arrival(self, road, sim_time):
        self.arrival_history[road].append(sim_time)

    def get_time_remaining(self):
        if self.controller_type == "fixed":
            cycle = (Config.FIXED_GREEN_TIME * 2 +
                     Config.FIXED_YELLOW_TIME * 2 +
                     Config.FIXED_ALL_RED_TIME * 2)
            pos = self.signal_timer % cycle
            if pos < Config.FIXED_GREEN_TIME:
                return Config.FIXED_GREEN_TIME - pos
            elif pos < Config.FIXED_GREEN_TIME + Config.FIXED_YELLOW_TIME:
                return Config.FIXED_GREEN_TIME + Config.FIXED_YELLOW_TIME - pos
            elif pos < Config.FIXED_GREEN_TIME + Config.FIXED_YELLOW_TIME + Config.FIXED_ALL_RED_TIME:
                return Config.FIXED_GREEN_TIME + Config.FIXED_YELLOW_TIME + Config.FIXED_ALL_RED_TIME - pos
            elif pos < Config.FIXED_GREEN_TIME * 2 + Config.FIXED_YELLOW_TIME + Config.FIXED_ALL_RED_TIME:
                return Config.FIXED_GREEN_TIME * 2 + Config.FIXED_YELLOW_TIME + Config.FIXED_ALL_RED_TIME - pos
            elif pos < Config.FIXED_GREEN_TIME * 2 + Config.FIXED_YELLOW_TIME * 2 + Config.FIXED_ALL_RED_TIME:
                return Config.FIXED_GREEN_TIME * 2 + Config.FIXED_YELLOW_TIME * 2 + Config.FIXED_ALL_RED_TIME - pos
            else:
                return cycle - pos
        else:
            return self.current_green_time

# ===================== Metrics =====================
class MetricsCollector:
    def __init__(self):
        self.reset(0.0)

    def reset(self, start_sim_time):
        self.total_cars_spawned = 0
        self.total_cars_exited = 0
        self.total_wait_time = 0.0
        self.sim_start_time = start_sim_time
        self.throughput_history = deque(maxlen=400)

    def record_car_spawn(self):
        self.total_cars_spawned += 1

    def record_car_exit(self, car, sim_time):
        self.total_cars_exited += 1
        self.total_wait_time += car.waiting_time
        car.exit_time = sim_time
        self.throughput_history.append(sim_time)

    def get_average_wait_time(self):
        return 0.0 if self.total_cars_exited == 0 else (self.total_wait_time / self.total_cars_exited)

    def get_throughput_per_minute(self, sim_time):
        recent = [t for t in self.throughput_history if sim_time - t < 60.0]
        return len(recent)

    def get_simulation_time(self, sim_time):
        return sim_time - self.sim_start_time

# ===================== Simulation =====================
class TrafficSimulation:
    def __init__(self):
        self.screen = pygame.display.set_mode((Config.WINDOW_WIDTH, Config.WINDOW_HEIGHT))
        pygame.display.set_caption("Traffic Signal Simulation")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 36)
        self.small_font = pygame.font.Font(None, 24)

        self.cars = []
        self.signal_controller = None
        self.metrics = MetricsCollector()
        self.running = False
        self.simulation_speed = 1.0
        self.incident_active = False
        self.ambulance_mode = False

        # UI state
        self.show_menu = True
        self.buttons = []
        self.cached_fixed_metrics = self.load_cached_metrics()

        # Simulation clock
        self.sim_time = 0.0
        self.dt_sim = 0.0

        # Incident region (visual + slowdown)
        cx = Config.WINDOW_WIDTH // 2
        cy = Config.WINDOW_HEIGHT // 2
        self.incident_rect = pygame.Rect(cx + 20, cy - 60, 40, 30)  # same as drawn

    # ---------- Persistence ----------
    def load_cached_metrics(self):
        try:
            if os.path.exists('fixed_metrics.json'):
                with open('fixed_metrics.json', 'r') as f:
                    return json.load(f)
        except:
            pass
        return None

    def save_fixed_metrics(self):
        if self.signal_controller and self.signal_controller.controller_type == "fixed":
            data = {
                'avg_wait_time': self.metrics.get_average_wait_time(),
                'throughput': self.metrics.get_throughput_per_minute(self.sim_time),
                'total_cars': self.metrics.total_cars_exited,
                'simulation_time': self.metrics.get_simulation_time(self.sim_time)
            }
            try:
                with open('fixed_metrics.json', 'w') as f:
                    json.dump(data, f)
            except:
                pass

    # ---------- UI ----------
    def create_menu_buttons(self):
        w, h = 200, 50
        cx = Config.WINDOW_WIDTH // 2 - w // 2
        sy = Config.WINDOW_HEIGHT // 2 - 100
        self.buttons = [
            {'rect': pygame.Rect(cx, sy, w, h), 'text': 'Fixed Timer Mode',
             'action': lambda: self.start_simulation('fixed')},
            {'rect': pygame.Rect(cx, sy + 70, w, h), 'text': 'AI Controller Mode',
             'action': lambda: self.start_simulation('ai')},
            {'rect': pygame.Rect(cx, sy + 140, w, h), 'text': 'Quit', 'action': self.quit_game}
        ]

    def start_simulation(self, mode):
        # reset cars/metrics and seed timers to current sim_time
        self.signal_controller = SignalController(mode, start_sim_time=self.sim_time)
        self.metrics.reset(self.sim_time)
        self.cars.clear()
        self.show_menu = False
        self.running = True
        self.incident_active = False
        self.ambulance_mode = False

    def quit_game(self):
        pygame.quit()
        sys.exit(0)

    # ---------- Core ----------
    def spawn_cars(self):
        # Convert legacy per-frame BASE to per-second rate
        base_per_sec = Config.BASE_SPAWN_RATE * Config.FPS
        main_rate = base_per_sec * Config.MAIN_ROAD_MULTIPLIER   # cars/sec
        side_rate = base_per_sec * Config.SIDE_ROAD_MULTIPLIER   # cars/sec

        def event(dt, rate_per_sec):
            p = 1.0 - math.exp(-rate_per_sec * dt)
            return random.random() < p

        # Main road spawn (N/S)
        if event(self.dt_sim, main_rate):
            direction = random.choice([Direction.NORTH, Direction.SOUTH])
            is_ambulance = self.ambulance_mode and (random.random() < 0.10)
            cx = Config.WINDOW_WIDTH // 2
            if direction == Direction.NORTH:
                car = Car(cx - Config.LANE_WIDTH // 2, Config.WINDOW_HEIGHT + 50, direction, is_ambulance, self.sim_time)
            else:
                car = Car(cx + Config.LANE_WIDTH // 2, -50, direction, is_ambulance, self.sim_time)
            self.cars.append(car)
            self.metrics.record_car_spawn()
            self.signal_controller.record_arrival('main', self.sim_time)

        # Side road spawn (E/W)
        if event(self.dt_sim, side_rate):
            direction = random.choice([Direction.EAST, Direction.WEST])
            is_ambulance = self.ambulance_mode and (random.random() < 0.10)
            cy = Config.WINDOW_HEIGHT // 2
            if direction == Direction.EAST:
                car = Car(-50, cy - Config.LANE_WIDTH // 2, direction, is_ambulance, self.sim_time)
            else:
                car = Car(Config.WINDOW_WIDTH + 50, cy + Config.LANE_WIDTH // 2, direction, is_ambulance, self.sim_time)
            self.cars.append(car)
            self.metrics.record_car_spawn()
            self.signal_controller.record_arrival('side', self.sim_time)

    def update_cars(self):
        # Light incident effect: cars crossing the incident region slow down (not ambulances)
        for car in self.cars[:]:
            # Restore base max speed
            car.max_speed = Config.CAR_MAX_SPEED

            if self.incident_active and not car.is_ambulance:
                # If the car is near the incident box, reduce its max speed
                expand = 30  # leniency
                inc = self.incident_rect.inflate(expand, expand)
                if inc.collidepoint(int(car.x), int(car.y)):
                    car.max_speed = min(car.max_speed, 1.5)

            car.update(self.cars, self.signal_controller, self.dt_sim)

            if car.is_off_screen():
                self.metrics.record_car_exit(car, self.sim_time)
                self.cars.remove(car)

    # ---------- Events ----------
    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                if self.running:
                    self.save_fixed_metrics()
                return False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    if self.running:
                        self.save_fixed_metrics()
                        self.show_menu = True
                        self.running = False
                elif event.key == pygame.K_SPACE and self.running:
                    self.simulation_speed = 2.0 if self.simulation_speed == 1.0 else 1.0
                elif event.key == pygame.K_i and self.running:
                    self.incident_active = not self.incident_active
                elif event.key == pygame.K_a and self.running:
                    self.ambulance_mode = not self.ambulance_mode

            if event.type == pygame.MOUSEBUTTONDOWN and self.show_menu:
                mx, my = pygame.mouse.get_pos()
                for b in self.buttons:
                    if b['rect'].collidepoint((mx, my)):
                        b['action']()
        return True

    # ---------- Drawing ----------
    def draw_road(self):
        self.screen.fill(Config.GRASS_COLOR)

        cx = Config.WINDOW_WIDTH // 2
        cy = Config.WINDOW_HEIGHT // 2

        # Main (N-S)
        main_rect = pygame.Rect(cx - Config.ROAD_WIDTH // 2, 0, Config.ROAD_WIDTH, Config.WINDOW_HEIGHT)
        pygame.draw.rect(self.screen, Config.ROAD_COLOR, main_rect)

        # Side (E-W)
        side_rect = pygame.Rect(0, cy - Config.ROAD_WIDTH // 2, Config.WINDOW_WIDTH, Config.ROAD_WIDTH)
        pygame.draw.rect(self.screen, Config.ROAD_COLOR, side_rect)

        # Lane dashes (skip intersection)
        dash, gap = 20, 30
        for y in range(0, Config.WINDOW_HEIGHT, dash + gap):
            if not (cy - Config.INTERSECTION_SIZE//2 < y < cy + Config.INTERSECTION_SIZE//2):
                pygame.draw.rect(self.screen, Config.LANE_COLOR, pygame.Rect(cx - 2, y, 4, dash))
        for x in range(0, Config.WINDOW_WIDTH, dash + gap):
            if not (cx - Config.INTERSECTION_SIZE//2 < x < cx + Config.INTERSECTION_SIZE//2):
                pygame.draw.rect(self.screen, Config.LANE_COLOR, pygame.Rect(x, cy - 2, dash, 4))

        # Stop lines
        t = 4
        half = Config.INTERSECTION_SIZE // 2
        # North approach line at y = cy - half
        pygame.draw.line(self.screen, Config.LANE_COLOR,
                         (cx - Config.ROAD_WIDTH//2, cy - half),
                         (cx, cy - half), t)
        # South approach at y = cy + half
        pygame.draw.line(self.screen, Config.LANE_COLOR,
                         (cx, cy + half),
                         (cx + Config.ROAD_WIDTH//2, cy + half), t)
        # East approach at x = cx + half
        pygame.draw.line(self.screen, Config.LANE_COLOR,
                         (cx + half, cy - Config.ROAD_WIDTH//2),
                         (cx + half, cy), t)
        # West approach at x = cx - half
        pygame.draw.line(self.screen, Config.LANE_COLOR,
                         (cx - half, cy),
                         (cx - half, cy + Config.ROAD_WIDTH//2), t)

        # Incident indicator
        if self.incident_active:
            pygame.draw.rect(self.screen, (255, 165, 0), self.incident_rect)
            pygame.draw.rect(self.screen, (255, 0, 0), self.incident_rect, 3)

    def draw_signals(self):
        cx = Config.WINDOW_WIDTH // 2
        cy = Config.WINDOW_HEIGHT // 2
        main_state = self.signal_controller.get_signal_state('main')
        side_state = self.signal_controller.get_signal_state('side')

        self.draw_traffic_light((cx - 30, cy - Config.INTERSECTION_SIZE//2 - 40), main_state)  # North
        self.draw_traffic_light((cx + 30, cy + Config.INTERSECTION_SIZE//2 + 40), main_state)  # South
        self.draw_traffic_light((cx + Config.INTERSECTION_SIZE//2 + 40, cy - 30), side_state)  # East
        self.draw_traffic_light((cx - Config.INTERSECTION_SIZE//2 - 40, cy + 30), side_state)  # West

    def draw_traffic_light(self, pos, state):
        w, h, r = 20, 60, 8
        rect = pygame.Rect(pos[0] - w//2, pos[1] - h//2, w, h)
        pygame.draw.rect(self.screen, (40, 40, 40), rect)
        pygame.draw.rect(self.screen, (0, 0, 0), rect, 2)

        red_pos = (pos[0], pos[1] - 15)
        yel_pos = (pos[0], pos[1])
        grn_pos = (pos[0], pos[1] + 15)

        pygame.draw.circle(self.screen, Config.RED if state in (SignalState.RED, SignalState.ALL_RED) else Config.SIGNAL_OFF, red_pos, r)
        pygame.draw.circle(self.screen, Config.YELLOW if state == SignalState.YELLOW else Config.SIGNAL_OFF, yel_pos, r)
        pygame.draw.circle(self.screen, Config.GREEN if state == SignalState.GREEN else Config.SIGNAL_OFF, grn_pos, r)

    def draw_ui_overlay(self):
        overlay = pygame.Surface((400, 300)); overlay.set_alpha(200); overlay.fill((0, 0, 0))
        self.screen.blit(overlay, (10, 10))

        y = 20
        lh = 25

        mode_text = f"Mode: {'Fixed Timer' if self.signal_controller.controller_type == 'fixed' else 'AI Controller'}"
        self.screen.blit(self.font.render(mode_text, True, Config.TEXT_COLOR), (20, y)); y += 35

        tval = self.signal_controller.get_time_remaining()
        ttxt = f"Time Remaining: {tval:.1f}s" if self.signal_controller.controller_type == "fixed" else f"Current Green: {tval:.1f}s"
        self.screen.blit(self.small_font.render(ttxt, True, Config.TEXT_COLOR), (20, y)); y += lh

        main_state = self.signal_controller.get_signal_state('main')
        side_state = self.signal_controller.get_signal_state('side')
        active = "Main Road" if main_state == SignalState.GREEN else ("Side Road" if side_state == SignalState.GREEN else "Switching")
        self.screen.blit(self.small_font.render(f"Active: {active}", True, Config.TEXT_COLOR), (20, y)); y += lh

        m_q = self.signal_controller.get_queue_length(self.cars, 'main')
        s_q = self.signal_controller.get_queue_length(self.cars, 'side')
        self.screen.blit(self.small_font.render(f"Main Queue: {m_q}", True, Config.TEXT_COLOR), (20, y)); y += lh
        self.screen.blit(self.small_font.render(f"Side Queue: {s_q}", True, Config.TEXT_COLOR), (20, y)); y += lh

        avg_wait = self.metrics.get_average_wait_time()
        throughput = self.metrics.get_throughput_per_minute(self.sim_time)
        self.screen.blit(self.small_font.render(f"Avg Wait: {avg_wait:.1f}s", True, Config.TEXT_COLOR), (20, y)); y += lh
        self.screen.blit(self.small_font.render(f"Throughput: {throughput}/min", True, Config.TEXT_COLOR), (20, y)); y += lh
        self.screen.blit(self.small_font.render(f"Total Cars: {self.metrics.total_cars_exited}", True, Config.TEXT_COLOR), (20, y)); y += lh

        # Comparison vs cached fixed-timer
        if (self.signal_controller.controller_type == "ai" and
            self.cached_fixed_metrics and
            self.metrics.total_cars_exited > 10):
            fixed_avg = self.cached_fixed_metrics.get('avg_wait_time', 0) or 0
            if fixed_avg > 0:
                imp = ((fixed_avg - avg_wait) / fixed_avg) * 100.0
                color = Config.GREEN if imp > 0 else Config.RED
                self.screen.blit(self.small_font.render(f"vs Fixed: {imp:+.1f}%", True, color), (20, y))

        # Controls
        y = Config.WINDOW_HEIGHT - 120
        for line in ["ESC: Menu", "SPACE: Speed (2x)", "I: Incident (slows traffic)", "A: Ambulance Preemption"]:
            self.screen.blit(self.small_font.render(line, True, Config.TEXT_COLOR), (20, y))
            y += 20

        # Status flags
        if self.simulation_speed > 1.0:
            self.screen.blit(self.small_font.render("FAST MODE", True, (255, 255, 0)), (Config.WINDOW_WIDTH - 120, 20))
        if self.incident_active:
            self.screen.blit(self.small_font.render("INCIDENT", True, (255, 0, 0)), (Config.WINDOW_WIDTH - 120, 45))
        if self.ambulance_mode:
            self.screen.blit(self.small_font.render("AMBULANCE", True, Config.AMBULANCE_COLOR), (Config.WINDOW_WIDTH - 120, 70))

    def draw_menu(self):
        self.screen.fill((30, 30, 50))
        title = self.font.render("Traffic Signal Simulation", True, Config.TEXT_COLOR)
        self.screen.blit(title, title.get_rect(center=(Config.WINDOW_WIDTH//2, 150)))

        sub = self.small_font.render("Choose simulation mode:", True, Config.TEXT_COLOR)
        self.screen.blit(sub, sub.get_rect(center=(Config.WINDOW_WIDTH//2, 200)))

        mouse = pygame.mouse.get_pos()
        for b in self.buttons:
            hovered = b['rect'].collidepoint(mouse)
            pygame.draw.rect(self.screen, Config.BUTTON_HOVER if hovered else Config.BUTTON_COLOR, b['rect'])
            pygame.draw.rect(self.screen, Config.TEXT_COLOR, b['rect'], 2)
            txt = self.small_font.render(b['text'], True, Config.TEXT_COLOR)
            self.screen.blit(txt, txt.get_rect(center=b['rect'].center))

        lines = [
            "Fixed Timer: Traditional signal with preset timing",
            "AI Controller: Adaptive signal based on queues, arrivals, and wait",
            "",
            "Controls:",
            "ESC - Return to menu",
            "SPACE - Toggle 2x speed",
            "I - Toggle incident slowdown",
            "A - Toggle ambulance preemption"
        ]
        y = 450
        for line in lines:
            if line:
                s = self.small_font.render(line, True, Config.TEXT_COLOR)
                self.screen.blit(s, s.get_rect(center=(Config.WINDOW_WIDTH//2, y)))
            y += 25

    # ---------- Main loop ----------
    def run(self):
        self.create_menu_buttons()

        while True:
            if not self.handle_events():
                break

            # Fixed render FPS; scale sim-time by speed
            dt_real = self.clock.tick(Config.FPS) / 1000.0
            self.dt_sim = dt_real * self.simulation_speed
            self.sim_time += self.dt_sim

            if self.show_menu:
                self.draw_menu()
            else:
                if self.running:
                    self.spawn_cars()
                    self.update_cars()
                    self.signal_controller.update(self.cars, self.dt_sim, self.sim_time)

                self.draw_road()
                for car in self.cars:
                    car.draw(self.screen)
                self.draw_signals()
                self.draw_ui_overlay()

            pygame.display.flip()

        if self.signal_controller and self.signal_controller.controller_type == "fixed":
            self.save_fixed_metrics()
        pygame.quit()

def main():
    """
    Traffic Signal Simulation (Fixed vs AI)

    Controls:
      ESC   - Menu
      SPACE - 2x speed toggle
      I     - Incident slowdown (affects cars near the orange box)
      A     - Ambulance preemption (white cars ignore red)

    Notes:
      - Timekeeping is sim-time based; metrics remain stable under speed toggles.
      - Spawns are Poisson per-second; doubling speed ≈ doubles arrivals and throughput.
    """
    TrafficSimulation().run()

if __name__ == "__main__":
    main()

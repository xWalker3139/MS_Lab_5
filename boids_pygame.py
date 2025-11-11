import pygame
import random
import math
from pygame.math import Vector2

# --- Simulation Constants ---
WIDTH, HEIGHT = 1000, 700
BOID_COUNT = 80
OBSTACLE_COUNT = 3
MAX_SPEED = 4
MAX_FORCE = 0.05

# --- Utility ---
def limit(vec, max_value):
    if vec.length() > max_value:
        vec.scale_to_length(max_value)
    return vec


# --- Boid Class ---
class Boid:
    def __init__(self, x, y, leader=False):
        self.pos = Vector2(x, y)
        angle = random.uniform(0, 2 * math.pi)
        self.vel = Vector2(math.cos(angle), math.sin(angle))
        self.acc = Vector2(0, 0)
        self.max_speed = MAX_SPEED
        self.max_force = MAX_FORCE
        self.leader = leader
        self.base_speed = random.uniform(1.5, 3.5)
        self.color = (255, 255, 255)

    def update(self):
        self.vel += self.acc
        limit(self.vel, self.max_speed)
        self.pos += self.vel
        self.acc *= 0
        self.wrap()

    def apply_force(self, force):
        self.acc += force

    def wrap(self):
        if self.pos.x < 0:
            self.pos.x += WIDTH
        elif self.pos.x > WIDTH:
            self.pos.x -= WIDTH
        if self.pos.y < 0:
            self.pos.y += HEIGHT
        elif self.pos.y > HEIGHT:
            self.pos.y -= HEIGHT

    def draw(self, screen):
        # --- Color based on speed (Blue → White → Red) ---
        speed = self.vel.length()
        t = min(max(speed / self.max_speed, 0), 1)  # normalized 0–1

        if t < 0.5:
            # Blue → White
            r = int(510 * t)
            g = int(155 * t + 100)
            b = 255
        else:
            # White → Red
            r = 255
            g = int(255 * (1 - (t - 0.5) * 2))
            b = int(255 * (1 - (t - 0.5) * 2))

        # Leaders stay cyan
        if self.leader:
            self.color = (50, 200, 255)
        else:
            self.color = (r, g, b)

        # Draw triangle pointing in velocity direction
        angle = self.vel.angle_to(Vector2(1, 0))
        points = [
            self.pos + Vector2(10, 0).rotate(-angle),
            self.pos + Vector2(-6, 4).rotate(-angle),
            self.pos + Vector2(-6, -4).rotate(-angle),
        ]
        pygame.draw.polygon(screen, self.color, points)


# --- Obstacle Class ---
class Obstacle:
    def __init__(self, x, y, radius, dynamic=False):
        self.pos = Vector2(x, y)
        self.radius = radius
        self.dynamic = dynamic
        self.vel = Vector2(random.uniform(-1, 1), random.uniform(-1, 1)) if dynamic else Vector2(0, 0)

    def update(self):
        if self.dynamic:
            self.pos += self.vel
            if self.pos.x < 0 or self.pos.x > WIDTH:
                self.vel.x *= -1
            if self.pos.y < 0 or self.pos.y > HEIGHT:
                self.vel.y *= -1

    def draw(self, screen):
        pygame.draw.circle(screen, (100, 100, 100), (int(self.pos.x), int(self.pos.y)), int(self.radius))


# --- Flock Class ---
class Flock:
    def __init__(self):
        self.boids = [Boid(random.uniform(0, WIDTH), random.uniform(0, HEIGHT)) for _ in range(BOID_COUNT)]
        # Make 2 leader boids
        for i in range(2):
            self.boids[i].leader = True
        # Add some obstacles
        self.obstacles = [
            Obstacle(random.uniform(100, WIDTH - 100), random.uniform(100, HEIGHT - 100),
                     random.uniform(30, 50), dynamic=(i == 0))
            for i in range(OBSTACLE_COUNT)
        ]
        # Simulation parameters
        self.separation_weight = 1.5
        self.alignment_weight = 1.0
        self.cohesion_weight = 1.0
        self.perception_radius = 60
        self.avoidance_radius = 40

    def step(self):
        for ob in self.obstacles:
            ob.update()

        for boid in self.boids:
            if boid.leader:
                # Leader moves in a circular pattern
                t = pygame.time.get_ticks() / 1000
                target = Vector2(WIDTH / 2 + 200 * math.cos(t + id(boid) % 5),
                                 HEIGHT / 2 + 200 * math.sin(t + id(boid) % 5))
                desired = (target - boid.pos).normalize() * boid.max_speed
                steer = limit(desired - boid.vel, boid.max_force)
                boid.apply_force(steer)
            else:
                sep = self.separation(boid) * self.separation_weight
                ali = self.alignment(boid) * self.alignment_weight
                coh = self.cohesion(boid) * self.cohesion_weight
                obs = self.avoid_obstacles(boid) * 2.0

                # Variable speed based on local density
                density = len(self.get_neighbors(boid)) / 10.0
                boid.max_speed = max(1.5, 4.0 - density)

                total_force = sep + ali + coh + obs
                limit(total_force, boid.max_force)
                boid.apply_force(total_force)

            boid.update()

    def get_neighbors(self, boid):
        return [other for other in self.boids if other is not boid and boid.pos.distance_to(other.pos) < self.perception_radius]

    def separation(self, boid):
        steer = Vector2(0, 0)
        for other in self.boids:
            d = boid.pos.distance_to(other.pos)
            if 0 < d < self.avoidance_radius:
                diff = (boid.pos - other.pos).normalize() / d
                steer += diff
        if steer.length() > 0:
            steer = steer.normalize() * boid.max_speed - boid.vel
        return steer

    def alignment(self, boid):
        avg_vel = Vector2(0, 0)
        neighbors = self.get_neighbors(boid)
        if neighbors:
            for other in neighbors:
                avg_vel += other.vel
            avg_vel /= len(neighbors)
            avg_vel = avg_vel.normalize() * boid.max_speed
            steer = avg_vel - boid.vel
            return steer
        return Vector2(0, 0)

    def cohesion(self, boid):
        center_mass = Vector2(0, 0)
        neighbors = self.get_neighbors(boid)
        if neighbors:
            for other in neighbors:
                center_mass += other.pos
            center_mass /= len(neighbors)
            desired = (center_mass - boid.pos).normalize() * boid.max_speed
            return desired - boid.vel
        return Vector2(0, 0)

    def avoid_obstacles(self, boid):
        steer = Vector2(0, 0)
        for ob in self.obstacles:
            d = boid.pos.distance_to(ob.pos)
            if d < ob.radius + 40:
                diff = (boid.pos - ob.pos).normalize() * (1 / (d + 1))
                steer += diff
        return steer

    def draw(self, screen):
        for ob in self.obstacles:
            ob.draw(screen)
        for b in self.boids:
            b.draw(screen)


# --- Main Loop ---
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("🐦 Boids Simulation with Interactive Controls")
    clock = pygame.time.Clock()
    flock = Flock()
    font = pygame.font.SysFont(None, 24)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False

            # --- Interactive Controls ---
            if event.type == pygame.KEYDOWN:
                # Rule weights
                if event.key == pygame.K_1: flock.separation_weight += 0.1
                if event.key == pygame.K_q: flock.separation_weight = max(0, flock.separation_weight - 0.1)
                if event.key == pygame.K_2: flock.alignment_weight += 0.1
                if event.key == pygame.K_w: flock.alignment_weight = max(0, flock.alignment_weight - 0.1)
                if event.key == pygame.K_3: flock.cohesion_weight += 0.1
                if event.key == pygame.K_e: flock.cohesion_weight = max(0, flock.cohesion_weight - 0.1)
                # Perception & Avoidance distance
                if event.key == pygame.K_4: flock.perception_radius += 5
                if event.key == pygame.K_r: flock.perception_radius = max(10, flock.perception_radius - 5)
                if event.key == pygame.K_5: flock.avoidance_radius += 5
                if event.key == pygame.K_t: flock.avoidance_radius = max(10, flock.avoidance_radius - 5)

        flock.step()
        screen.fill((20, 20, 30))
        flock.draw(screen)

        # Display HUD with current parameters
        hud_lines = [
            f"Sep:{flock.separation_weight:.2f} (1/Q)",
            f"Ali:{flock.alignment_weight:.2f} (2/W)",
            f"Coh:{flock.cohesion_weight:.2f} (3/E)",
            f"Perception:{flock.perception_radius:.0f} (4/R)",
            f"AvoidDist:{flock.avoidance_radius:.0f} (5/T)"
        ]
        for i, line in enumerate(hud_lines):
            text = font.render(line, True, (200, 200, 200))
            screen.blit(text, (10, 10 + i * 20))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()

import pygame
import math

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Ackermann Steering Vehicle Simulation")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)

# Simulation clock
clock = pygame.time.Clock()

# Vehicle parameters
class Vehicle:
    def __init__(self, x, y, length, width):
        self.x = x  # x-position of the vehicle
        self.y = y  # y-position of the vehicle
        self.theta = 0  # orientation angle of the vehicle (in radians)
        self.length = length  # length of the vehicle
        self.width = width  # width of the vehicle

        # Ackermann steering model
        self.wheel_base = 50  # wheelbase (distance between front and rear axles)
        self.steering_angle = 0  # steering angle (radians)
        self.velocity = 0  # vehicle velocity
        self.max_velocity = 10  # maximum speed
        self.max_steering = math.radians(30)  # max steering angle
        self.acceleration = 0.2  # acceleration rate
        self.friction = 0.02  # friction coefficient to simulate drag

    def update(self, dt):
        # Apply velocity and steering angle to move the vehicle
        if self.velocity > 0:
            self.velocity -= self.friction  # Apply friction to slow down the vehicle

        # Kinematic bicycle model for Ackermann steering
        if self.steering_angle != 0:
            turning_radius = self.wheel_base / math.tan(self.steering_angle)
            angular_velocity = self.velocity / turning_radius
        else:
            angular_velocity = 0

        # Update the vehicle's position and orientation
        self.theta += angular_velocity * dt
        self.x += self.velocity * math.cos(self.theta) * dt
        self.y += self.velocity * math.sin(self.theta) * dt

    def draw(self, screen):
        # Draw the vehicle as a simple rectangle (with rotation)
        car_image = pygame.Surface((self.length, self.width))
        car_image.fill(RED)
        rotated_car = pygame.transform.rotate(car_image, -math.degrees(self.theta))

        # Calculate the new top-left corner after rotation
        rect = rotated_car.get_rect(center=(self.x, self.y))
        screen.blit(rotated_car, rect.topleft)

    def handle_input(self, keys):
        # Handle vehicle movement (keyboard arrow keys)
        if keys[pygame.K_UP]:
            if self.velocity < self.max_velocity:
                self.velocity += self.acceleration  # Accelerate forward
        if keys[pygame.K_DOWN]:
            if self.velocity > -self.max_velocity:
                self.velocity -= self.acceleration  # Accelerate backward

        if keys[pygame.K_LEFT]:
            if self.steering_angle < self.max_steering:
                self.steering_angle += math.radians(2)  # Turn left

        if keys[pygame.K_RIGHT]:
            if self.steering_angle > -self.max_steering:
                self.steering_angle -= math.radians(2)  # Turn right

        if not (keys[pygame.K_LEFT] or keys[pygame.K_RIGHT]):
            # If no steering input, straighten the wheels
            self.steering_angle *= 0.9

# Main simulation loop
def main():
    running = True
    vehicle = Vehicle(WIDTH // 2, HEIGHT // 2, length=60, width=30)

    while running:
        dt = clock.tick(100) / 1000.0  # Time in seconds per frame
        screen.fill(WHITE)

        # Handle user input
        keys = pygame.key.get_pressed()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update vehicle dynamics
        vehicle.handle_input(keys)
        vehicle.update(dt)

        # Draw the vehicle
        vehicle.draw(screen)

        # Update the screen
        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()

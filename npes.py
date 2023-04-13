#the library
import pygame
import numpy as np
import math
import random

# Settings
screen_width = 800
screen_height = 600

G = 6.6743e-11
G2 = 1000
ga = 9.81
elasticity = 0.9
gravity = True
screen_bounce = True
collision = True

# For vectors calculation
class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def add(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def subtract(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    def scale(self, scalar):
        return Vector(self.x * scalar, self.y * scalar)

    def magnitude(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def normalize(self):
        mag = self.magnitude()
        return Vector(self.x / mag, self.y / mag) if mag != 0 else Vector(0, 0)

class Particle:
    def __init__(self, pos, vel, mass, radius):
        self.pos = np.array(pos, dtype=np.float64)
        self.vel = np.array(vel, dtype=np.float64)
        self.mass = mass
        self.radius = radius

    def update(self, dt, force):
        # Update the particle's velocity and position based on the force acting on it
        acceleration = force / self.mass
        self.vel += acceleration * dt
        self.pos += self.vel * dt

class Wall:
    def __init__(self, start, end):
        self.start = np.array(start, dtype=np.float64)
        print(self.start)
        self.end = np.array(end, dtype=np.float64)
        print(self.end)
        a = (self.start[0]-self.end[0]) / (self.start[1]-self.end[1])
        b = self.start[1] - (a*self.start[0])
        

    def has_crossed(self, particle):
        #Check particle collision
        a = (self.start[0]-self.end[0]) / (self.start[1]-self.end[1])
        b = self.start[1] - (a*self.start[0])
        x = particle.pos[0]
        y = particle.pos[1]
        if math.floor(y) == math.floor((a*x) + b):
            print('wall collision detected')
            return True

    def reflect(self, particle):
        # Reflect the particle's velocity off the wall
        particle.vel = -(particle.vel)



class Simulation:
    def __init__(self, particles, walls, dt=0.01):
        self.particles = particles
        self.walls = walls
        self.dt = dt

    def step(self):
        # Calculate the net force acting on each particle
        if gravity==True: forces = [np.array([0, (particle.mass * ga)]) for particle in self.particles]
        else: forces = []
        for i, particle1 in enumerate(self.particles):
            force = np.array([0, 0], dtype=np.float64)
            for j, particle2 in enumerate(self.particles):
                if i != j:
                    # Calculate the gravitational force between the particles
                    distance = np.linalg.norm(particle2.pos - particle1.pos)
                    force_direction = (particle2.pos - particle1.pos) / distance
                    force_magnitude = G * particle1.mass * particle2.mass / distance**2
                    #force_magnitude = 0
                    force += force_direction * force_magnitude

                    # Check if the particles have collided
                    if collision == True:
                        if (distance - particle1.radius - particle2.radius) <= 0.1:
                            # Handle the collision by updating the velocities of both particles
                            particle1.vel, particle2.vel = particle2.vel, particle1.vel
                            #particle1.vel *= elasticity
                            #particle2.vel *= elasticity
                            particle1.update(self.dt, np.zeros(2))
                            particle2.update(self.dt, np.zeros(2))

            #Check collision with screen edge
            if screen_bounce == True:
                if (particle1.pos[1] + particle1.radius) > screen_height or (particle1.pos[1] - particle1.radius) < 0:
                    particle1.vel[1] = -(particle1.vel[1])
                    particle1.vel *= elasticity
                    particle1.update(self.dt, np.zeros(2))
                if (particle1.pos[0] + particle1.radius) > screen_width or (particle1.pos[0] - particle1.radius) < 0:
                    particle1.vel[0] = -(particle1.vel[0])
                    particle1.vel *= elasticity
                    particle1.update(self.dt, np.zeros(2))

            forces.append(force)

        # Update each particle's velocity and position
        for i, particle in enumerate(self.particles):
            # Update the particle's velocity and position based on the force acting on it
            particle.update(self.dt, forces[i])

            # Check if the particle has collided with any walls
            for wall in self.walls:
                for particle in self.particles:
                    if wall.has_crossed(particle):
                        # Reflect the particle's velocity off the wall
                        wall.reflect(particle)

                        # Update the particle's position again
                        particle.update(self.dt, np.zeros(2))


    def draw(self, screen):
        # Draw each particle as a circle on the screen
        for particle in self.particles:
            #pos = particle.pos.astype(int)
            pygame.draw.circle(screen, (255, 0, 0), particle.pos, particle.radius)

        # Draw each wall as a line on the screen
        for wall in self.walls:
            start = wall.start
            end = wall.end
            start = start.astype(int)
            end = end.astype(int)
            pygame.draw.line(screen, (0, 0, 255), start, end, 2)



#testing program
# Set up the Pygame display
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Newtonian Physics Simulation")

r = random.randint(50,500)

# Create three particles with initial positions, velocities, masses and radius
particle1 = Particle([r, r], [r, r], 1000, 30)
particle2 = Particle([200, 300], [40, -20], 50, 20)
particle3 = Particle([300, 250], [-25, 45], 50, 10)

# Create a simulation with the particles, walls, and a time step of 0.01 seconds
simulation = Simulation([particle1, particle2, particle3], [], dt=0.01)

# Run the simulation until the user closes the window
running = True
while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Update the simulation
    simulation.step()

    # Clear the screen
    screen.fill((255, 255, 255))

    # Draw the particles on the screen
    simulation.draw(screen)

    # Update the display
    pygame.display.flip()

# Clean up
pygame.quit()

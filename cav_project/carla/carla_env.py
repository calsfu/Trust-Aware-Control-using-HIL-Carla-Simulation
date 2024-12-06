import carla
import carla.command
import carla_cav_class
import numpy as np
import random
import pygame
import math

class CAVEnv:
    def __init__(self,
                num_cavs=11, 
                map_name='Town01',
                carla_port=2000,
                render_display=False
                ):
        
        # init
        self.num_cavs = num_cavs
        self.map_name = map_name
        self.render_display = render_display
                # initialize rendering
        if self.render_display:
            pygame.init()
            self.render_display = pygame.display.set_mode((800, 600), pygame.HWSURFACE | pygame.DOUBLEBUF)
            self.clock = pygame.time.Clock()

        self.cav_ids = [
            "limo155", 
            "limo813", 
            "limo777", 
            "limo793", 
            "limo795", 
            "limo789", 
            "limo780", 
            "limo799", 
            "limo808", 
            "limo787", 
            "limo770"
            ]

        # connect to client
        self.client = carla.Client('localhost', carla_port)
        self.client.set_timeout(4.0)

        # create world and map
        self.world = self.client.load_world(self.map_name)
        self.map = self.world.get_map()

        # create cavs
        self.carla_ros_pairs= {}
        # self.carla_cavs = []
        # self.ros_cavs = []
        self.create_cav(num_cavs)

        # create ego
        self.ego_carla_cav = None
        self.ego_ros_cav = None
        self.create_ego()

        # render display

        if self.render_display: #not worth to do rn
            self.actor_list = []
            self.create_display()



    def create_cav(self, num_cavs : int):
        '''
        Will create num_cavs cavs in CARLA, connect them to ros_cav nodes
        '''
        init_transforms = self.world.get_map().get_spawn_points()
        init_transforms = np.random.choice(init_transforms, num_cavs)

        # maybe change to make all non-ego one vehicle so its easier to see?
        blueprints = self.world.get_blueprint_library().filter('vehicle.*')
        blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]

        # spawn vehicles
        batch = []
        for transform in init_transforms:
            transform.location.z += 0.1  # otherwise can collide with the road it starts on
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(carla.command.SpawnActor(blueprint, transform).then(
                carla.command.SetAutopilot(carla.command.FutureActor, True)))

        # for response in self.client.apply_batch_sync(batch, False):
        #     self.carla_cavs.append(response.actor_id)

        # save created cav and link to carla_cav ros node
        for i, response in enumerate(self.client.apply_batch_sync(batch)):
            if response.error:
                pass
            else:
                cav_id = self.cav_ids[i]
                ros_cav = carla_cav_class(cav_id)
                self.carla_ros_pairs[cav_id] = (response.actor_id, ros_cav)        

    def create_ego(self):
        init_transforms = self.world.get_map().get_spawn_points()
        vehicle_init_transform = random.choice(init_transforms)
        blueprint_library = self.world.get_blueprint_library()
        vehicle_blueprint = blueprint_library.find('vehicle.' + 'tesla.cybertruck')
        if vehicle_blueprint.has_attribute('color'):
            color = random.choice(vehicle_blueprint.get_attribute('color').recommended_values)
            vehicle_blueprint.set_attribute('color', color)
        self.vehicle = self.world.spawn_actor(vehicle_blueprint, vehicle_init_transform)

    def create_display(self):
        # spawn camera for rendering
        blueprint_library = self.world.get_blueprint_library()
        self.camera_display = self.world.spawn_actor(
            blueprint_library.find('sensor.camera.rgb'),
            carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
            attach_to=self.ego_carla_cav)
        self.actor_list.append(self.camera_display)

    def step(self):
        if self.render_display:
            self.clock.tick()
            
            # draw_image(self.render_display, display_image)
            pygame.display.flip()

        # non-ego cavs
        batch = []
        for key, value in self.carla_ros_pairs.items():
            cav_id = key
            actor_id = value[0]
            ros_cav = value[1]
            
            # update value in carla_cav
            actor = self.world.get_actor(actor_id)
            velocity = actor.get_velocity()
            transform = actor.get_transform()
            # TODO: ASK SABBIR ABOUT STATE
            state = -1
            ros_cav.update_and_publish(velocity, state, transform)

            # update CARLA values
            steering_angle, desired_velocity, control = ros_cav.get_control_data()
            
            # TODO: FIX THROTTLE/BRAKE: Could maybe use with set_target_velocity
            steering = steering_angle / 7000 # max is 7000, measured in 1/100th angle maybe?
            speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            
            ackermann_control = carla.VehicleAckermannControl()
            ackermann_control.steer = steering_angle/7000
            ackermann_control.speed = desired_velocity
            
            batch.append(carla.command.ApplyVehicleAckermannControl(actor_id, ackermann_control))


        
        # update non-go cav
        responses = self.client.apply_batch_sync(batch)

        # ego cav
        # mocap stuff
        # location, rotation = mocap
        self.ego_carla_cav.set_transform(transform)

        self.world.tick()

def draw_image(surface, image, blend=False):
    array = np.frombuffer(image.raw_data, dtype=np.dtype('uint8'))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))

def main():
    env = CAVEnv()

if __name__ == '__main__':
    main()

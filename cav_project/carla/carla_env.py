import carla
# import carla_cav
import numpy as np
import random

class CAVEnv:
    def __init__(self,
                num_cavs=11, 
                map_name='Town01',
                carla_port=2000,
                ):
        
        # init
        self.num_cavs = num_cavs
        self.map_name = map_name

        # connect to client
        self.client = carla.Client('localhost', carla_port)
        self.client.set_timeout(4.0)

        # create world and map
        self.world = self.client.load_world(self.map_name)
        self.map = self.world.get_map()

        # create cavs
        self.carla_cavs = []
        self.ros_cavs = []
        self.create_cav(num_cavs)

        # create ego
        self.ego_carla_cav = None
        self.ego_ros_cav = None
        self.create_ego()

        # render display
        self.actor_list = []
        self.create_display()



    def create_cav(self, num_cavs : int):
        '''
        Will create num_cavs cavs in CARLA, connect them to ros_cav nodes
        '''
        # for i in range(num_cavs):
        #     # cav = carla_cav() add
        #     self.cav_list.append(cav)
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

        for response in self.client.apply_batch_sync(batch, False):
            self.carla_cavs.append(response.actor_id)

        for response in self.client.apply_batch_sync(batch):
            if response.error:
                pass
            else:
                self.carla_cavs.append(response.actor_id)

        # connect cav ros nodes to existing vehicles

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
        # non-ego cavs
        for i in range(self.carla_cavs):
            throttle, steer, brake = self.ros_cavs[i].getData()
            vehicle_control = carla.VehicleControl(
                throttle=float(throttle),
                steer=float(steer),
                brake=float(brake),
                hand_brake=False,
                reverse=False,
                manual_gear_shift=False
            )
            self.carla_cavs[i].apply_control(vehicle_control)

        # ego cav
        # mocap stuff
        # location = mocap
        self.ego_carla_cav.set_transform(location)



def main():
    env = CAVEnv()

if __name__ == '__main__':
    main()

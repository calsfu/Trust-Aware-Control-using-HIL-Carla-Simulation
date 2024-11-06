#!/usr/bin/env python3
import rospy
from cav_class import CAV
from cav_project.msg import limo_state_matrix
from function import calc_qp_info, calc_manhattan_distance, calc_distance

class MainCoordinator:
    def __init__(self):
        rospy.init_node('main_coordinator', anonymous=True)
        self.limo_state_matrix_pub = rospy.Publisher('/limo_state_matrix', limo_state_matrix, queue_size=10)
        self.rate = rospy.Rate(15)

        # Define the list of CAVs with their corresponding entry and exit points
        cav_configs = {
            "limo155": ('g', 'u'),  # cav2
            "limo813": ('n', 'o'),
            "limo777": ('n', 'o'),  # cav3
            "limo793": ('t', 'u'),  # cav7
            "limo795": ('m', 'u')
               # cav7
        }

        # Initialize all CAVs
        self.cavs = {ID: CAV(ID, enter) for ID, (enter, exit) in cav_configs.items()}

        # Initial order list for each zone
        self.order_list = {
            "Intersection Zone": [],
            "Merging Path Zone": [],
            "Link Zone": []  # Link Zone as a single list
        }

        # Manually set the initial order for each zone
        self.manual_initial_order()

        # Print initial orders
        for zone in self.order_list:
            print(f"Initial order in {zone}: {[c.ID for c in self.order_list[zone]]}")

    def manual_initial_order(self):
        # Manually set the initial order for CAVs in specific zones
        intersection_order = [
            self.cavs["limo795"],
            self.cavs["limo155"], # cav3
            self.cavs["limo813"],
            self.cavs["limo777"],  # cav2
            self.cavs["limo793"]   # cav7
        ]
        for cav in intersection_order:
            cav.zone = ["Intersection Zone"]
            self.order_list["Intersection Zone"].append(cav)

        # Merging Path Zone
        merging_order = []
        for cav in merging_order:
            cav.zone = ["Merging Path Zone"]
            self.order_list["Merging Path Zone"].append(cav)

        # Link Zone
        link_zone_order = []
        for cav in link_zone_order:
            cav.zone = ["Link Zone"]
            self.order_list["Link Zone"].append(cav)

    def update_order_list(self, cav):
        # Remove CAV from all zone lists first, but only if it is actually moving zones
        for zone in self.order_list:
            if zone not in cav.zone and cav in self.order_list[zone]:
                self.order_list[zone].remove(cav)
                print(f"Removed CAV {cav.ID} from {zone}, updated list: {[c.ID for c in self.order_list[zone]]}")

        # Add CAV to the appropriate list based on its current zones
        for zone in cav.zone:
            if cav not in self.order_list[zone]:
                self.order_list[zone].append(cav)
                print(f"Added CAV {cav.ID} to {zone}")
                # Sort the Link Zone based on the new logic
                if "Link Zone" in cav.zone:
                    self.insert_cav_in_link_zone(cav)

    def insert_cav_in_link_zone(self, new_cav):
        # Get the next collision point for the current CAV
        next_collision_point = new_cav.current_collision_pt1

        # Identify the conflicting CAVs in the current order list based on the next collision point
        conflicting_cavs = [cav for cav in self.cavs.values() if next_collision_point in cav.collision_pts and cav.ID != new_cav.ID]
        conflicting_cavs.append(new_cav)  # Include the new arriving CAV

        # Sort the conflicting CAVs by Manhattan distance
        for cav in conflicting_cavs:
            manhattan_distance = calc_manhattan_distance(self.cavs, cav.ID, next_collision_point)
            print(f"Manhattan distance for {cav.ID} to {next_collision_point}: {manhattan_distance}")

        conflicting_cavs.sort(key=lambda cav: calc_manhattan_distance(self.cavs, cav.ID, next_collision_point))

        # Remove the older instance of the new CAV if it already exists in the Link Zone
        self.order_list["Link Zone"] = [cav for cav in self.order_list["Link Zone"] if cav.ID != new_cav.ID]

        # Insert the sorted conflicting CAVs into the Link Zone
        temp = conflicting_cavs.index(new_cav)  # Get the index of the new CAV in the sorted list

        if temp == len(conflicting_cavs) - 1:
            # If the new CAV is last in the sorted list, append it to the end of the Link Zone
            self.order_list["Link Zone"].append(new_cav)
        else:
            # Insert the new CAV before the next conflicting CAV
            next_conflicting_cav = conflicting_cavs[temp + 1]
            for index, cav in enumerate(self.order_list["Link Zone"]):
                if cav.ID == next_conflicting_cav.ID:
                    self.order_list["Link Zone"].insert(index, new_cav)
                    break

        # Ensure the order list contains only unique CAVs
        unique_link_zone = []
        seen_cavs = set()

        for cav in self.order_list["Link Zone"]:
            if cav.ID not in seen_cavs:
                unique_link_zone.append(cav)
                seen_cavs.add(cav.ID)

        # Update the Link Zone with the final unique sorted order
        self.order_list["Link Zone"] = unique_link_zone

        # Print the updated order for debugging purposes
        print(f"Final order in Link Zone after sorting and inserting: {[cav.ID for cav in self.order_list['Link Zone']]}")

    def check_zone_transition(self):
        # Check for CAVs transitioning between zones
        for cav_id, cav in self.cavs.items():
            previous_zones = cav.zone
            cav.update_zone(self)

            # If transitioning to the Link Zone, calculate order based on collision points
            if "Link Zone" in cav.zone and "Link Zone" not in previous_zones:
                self.insert_cav_in_link_zone(cav)
            elif set(previous_zones) != set(cav.zone):
                print(f"CAV {cav.ID} has moved zones from {previous_zones} to {cav.zone}")
                self.update_order_list(cav)

    def run(self):
        while not rospy.is_shutdown():
            self.check_zone_transition()

            limo_state_mat = limo_state_matrix()

            for zone in self.order_list:
                if len(self.order_list[zone]) > 0:
                    print(f"Solving QP for CAVs in {zone}: {[c.ID for c in self.order_list[zone]]}")

                    for i, cav in enumerate(self.order_list[zone]):
                        limo_state_msg = calc_qp_info(self.order_list[zone], i, zone)
                        limo_state_mat.limos.append(limo_state_msg)
                        cav.run()

            self.limo_state_matrix_pub.publish(limo_state_mat)
            self.rate.sleep()

if __name__ == '__main__':
    coordinator = MainCoordinator()
    coordinator.run()

import logging

import lanelet2
from lanelet2.core import AttributeMap, Lanelet

from . import constants

logger = logging.getLogger(__name__)


class Preprocessing:
    """
    This class does preprocess tasks to prepare the main behavior space derivation of the framework. Thus, relevant
    lanelets are being identified and a RoutingGraph containing every lanelet of the Lanelet2 map is created.

    Attributes
    ----------
        map_lanelet : LaneletMap
            Layered lanelet2 map that contains all lanelet2 objects of a loaded map.
        traffic_rules : traffic_rules
            traffic rules object from lanelet2 for participant = vehicle

    Methods
    -------
        get_routing_graph_all():
            creating RoutingGraph object that contains every lanelet of a map
        find_relevant_lanelets():
            Find for bssd relevant lanelets in lanelet2 map.
        get_relevant_bicycle_lanelets():
            Distinguishes relevance of bicycle lanelets and returns list of relevant bicycle lanelets.
        find_usages_and_remove_self(ll, side):
            Finds direct neighbors on one side of a lanelet.
    """

    def __init__(self, map_lanelet):
        self.map_lanelet = map_lanelet
        self.traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                           lanelet2.traffic_rules.Participants.Vehicle)

    def get_routing_graph_all(self):
        """
        Creates a Lanelet2-RoutingGraph object for the participant 'vehicle'. Before the creation every lanelet of the
        map is made drivable by vehicles by setting overrride tags. This enables a RoutingGraph that contains every
        lanelet of a map, which is useful for the operations within this framework. Therefore, this RoutingGraph
        shouldn't be used for actual routing purposes.

        Returns:
            graph (RoutingGraph):RoutingGraph object, which contains lanelets as nodes and their connections as edges.
        """

        # Setup a variable to save the edited lanelets and their values in
        edited = {}

        # For-loop for every lanelet in a map
        for lanelet in self.map_lanelet.laneletLayer:
            # Check, if a lanelet is passable for vehicles. This way, only the non-passable lanelets will receive
            # override tags
            if not self.traffic_rules.canPass(lanelet):
                # check if the override tag for vehicle is already existing to save its value to the edited dictionary
                if 'participant:vehicle' in lanelet.attributes:
                    edited[lanelet] = lanelet.attributes['participant:vehicle']
                # If the tag doesn't exist, it will be created and set to yes. Adding the lanelet to the dictionary
                else:
                    edited[lanelet] = None
                lanelet.attributes['participant:vehicle'] = 'yes'

        # Creating the actual RoutingGraph object for the edited map
        graph = lanelet2.routing.RoutingGraph(self.map_lanelet, self.traffic_rules)

        # Running a for-loop for the edited lanelets to reverse the changes
        for lanelet, value in edited.items():
            # If a previous value existed, this one will be reset
            if value:
                lanelet.attributes['participant:vehicle'] = value
            # If no previous value existed the tag will be deleted entirely
            else:
                del lanelet.attributes['participant:vehicle']

        return graph

    def find_relevant_lanelets(self) -> list:
        """
        This function is going through every lanelet of the map and checking different criteria to determine whether
        a lanelet belongs to the roadway and therefore is relevant for the behavior space derivation.

        Returns:
            relevant_lanelets (list):List of every relevant lanelet of a Lanelet2 map.
        """

        # First, filter lanelets for passability of motorized vehicles
        relevant_lanelets = [lanelet.id for lanelet in self.map_lanelet.laneletLayer
                             if is_lanelet_relevant(lanelet.attributes)]
        # Second, add a list of relevant bicycle lanelets and return both lists combined
        return relevant_lanelets + self.get_relevant_bicycle_lanelets()

    def get_relevant_bicycle_lanelets(self) -> list:
        """
        This function filters every bicycle lanelet of a Lanelet2 map for relevance. This means that conditions need to
        be met to consider a lanelet part of the roadway. Currently, the conditions are that a bicycle lanelet has
        neigbors that are generally considered relevant (using the 'is_ll_relevant' function).

        Returns:
            relevant_bicycle_list (list):List of every relevant bicycle lanelet of a Lanelet2 map.
        """

        # Filter lanelet map for all lanelets that are tagged as 'bicycle_lane' or include the overriding tag
        # 'participant:bicycle' which is set to 'yes'
        list_bicycle = [lanelet for lanelet in self.map_lanelet.laneletLayer
                        if lanelet.attributes['subtype'] == 'bicycle_lane'
                        or ('participant:bicycle' in lanelet.attributes
                            and lanelet.attributes['participant:bicycle'] == 'yes')
                        ]
        relevant_bicycle_list = []

        # For-loop for every bicycle lane lanelet
        for lanelet in list_bicycle:

            # Find neighbors on both sides of the lanelet
            neighbors_left = self.find_usages_and_remove_self(lanelet, 'l')
            neighbors_right = self.find_usages_and_remove_self(lanelet, 'r')

            # Check if the neighbors on one of the sides allow the conclusion that this lanelet is relevant
            if is_bicycle_lanelet_relevant(neighbors_left, lanelet.leftBound.attributes) \
                    or is_bicycle_lanelet_relevant(neighbors_right, lanelet.rightBound.attributes):
                logger.debug(f' Lanelet {lanelet.id} identified as relevant bicycle lane')
                lanelet.attributes['relevant_bicycle_lane'] = 'yes'

                relevant_bicycle_list.append(lanelet.id)

        return relevant_bicycle_list

    def find_usages_and_remove_self(self, lanelet: Lanelet, side: str) -> list:
        """
        Finds all the direct neighbors of a given lanelet for the left or right side. This is accomplished by using the
        FindUsages function of the Lanelet2-Framework. This returns a list from which the original lanelet is being
        removed.

        Parameters:
            lanelet (lanelet):The lanelet that neighbors are being searched for.
            side (str):'l' for left and 'r' for right.

        Returns:
            neighbors (list):True if conditions are met, otherwise False.
        """

        neighbors = []

        # Distinguish between sites and search for the respective boundary linestring for usages
        if side == 'r':
            neighbors = self.map_lanelet.laneletLayer.findUsages(lanelet.rightBound)
        elif side == 'l':
            neighbors = self.map_lanelet.laneletLayer.findUsages(lanelet.leftBound)

        # Remove the lanelet from which the search has been started.
        neighbors.remove(lanelet)

        # Return the list of neighbors
        return neighbors


def is_lanelet_relevant(lanelet_attributes: AttributeMap) -> bool:
    """
    Determine the relevance of a lanelet by first checking its subtype (for instance: shouldn't be "stairs")
    and second if any overriding 'participant'-tags are being used

        Parameters:
            lanelet_attributes (AttributeMap):Attributes of a lanelet.

        Returns:
            relevant (bool):True if lanelet is relevant according to the selected criteria.
    """

    # Check if the lanelet subtype is in the list of subtypes that are considered to be part of the roadway
    # A second condition is used checks if the tag relevant bicycle lane is set.
    if lanelet_attributes['subtype'] in constants.SUBTYPE_TAGS \
            or ('relevant_bicycle_lane' in lanelet_attributes and lanelet_attributes['relevant_bicycle_lane'] == 'yes'):

        # If overriding tags are set, they override the meaning of the lanelet subtypes. Thus, a check is necessary that
        # makes sure that overriding tags show a passability for vehicles
        if any('participant' in key.lower() for key, value in lanelet_attributes.items()):

            if any(value == 'yes' for key, value in lanelet_attributes.items() if 'participant:vehicle' in key.lower()):
                relevant = True
            else:
                relevant = False

        # If no overriding tags are used and the subtype is matching the mentioned condition, the lanelet is relevant
        else:
            relevant = True

    # If the subtype does not match, the lanelet is considered not relevant.
    else:
        relevant = False

    return relevant


def is_bicycle_lanelet_relevant(neighbors: list, linestring_attributes: AttributeMap) -> bool:
    """
    This function checks for a given lateral linestring of a bicycle lanelet and the neighbors next to that linestring
    whether a bicycle lanelet is relevant.

        Parameters:
            neighbors (list):List of lanelets that border the considered bicycle lanelet.
            linestring_attributes (AttributeMap):Attributes of the lateral boundary linestring of a bicycle lanelet.

        Returns:
            relevant (bool):True if bicycle lanelet is relevant according to the selected criteria.
    """

    # First condition is that the lanelet has neighbors, if not, the bicycle lanelet is considered not to be on the
    # roadway. Second, neighbors are being checked themselves for relevance. It is otherwise possible that a bicycle
    # lanelet lies next to a walkway lanelet. The third condition is to check the linestring type that divides the
    # bicycle lanelet from its neighbor(s). If this linestring is not making it impossible to cross, a motorized vehicle
    # could theoretically reach the bicycle lanelet and it is therefore considered relevant.
    if neighbors and any(neighbor for neighbor in neighbors if is_lanelet_relevant(neighbor.attributes))\
            and linestring_attributes['type'] in constants.RELEVANT_BICYCLE_TAGS:
        return True
    else:
        return False

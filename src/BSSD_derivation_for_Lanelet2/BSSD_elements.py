from __future__ import annotations
import logging

from lanelet2.core import getId
from lanelet2.core import Lanelet, LineString2d, LineString3d
from bssd.core import mutable

logger = logging.getLogger('framework.classes')


class BssdMap:
    """
    This class is implemented to store the elements of BSSD in dictionaries for each element type. The elements are
    stored here to be accessible via ID and through this can be written one by one at the end of the framework.
    Furthermore, this class offers methods to create a new BehaviorSpace element including all of its subelements.

    Attributes
    ----------
        BehaviorSpaceLayer : dictionary
            Storing BehaviorSpace elements as values using their ID as the key.
        BehaviorLayer : dictionary
            Storing Behavior elements as values using their ID as the key.
        ReservationLayer : dictionary
            Storing Reservation elements as values using their ID as the key.
        BoundaryLatLayer : dictionary
            Storing BoundaryLat elements as values using their ID as the key.
        BoundaryLongLayer : dictionary
            Storing BoundaryLong elements as values using their ID as the key.

    Methods
    -------
        __init__():
            Initiates the dictionaries for each layer.
        add(BssdElement):
            adds an element to its respective layer and returning the element for further usage
        create_placeholder(lanelet=None, long_boundary_along=None, long_boundary_against=None):
            Creates a placeholder BehaviorSpace and gives the opportunity to add longitudinal boundaries as well as lanelets.
        create_behavior(left_boundary, right_boundary, long_boundary):
            Creates a placeholder Behavior and aggregates the lateral boundaries and the longitudinal boundary.
    """

    def __init__(self):
        self.BehaviorSpaceLayer = {}
        self.BehaviorLayer = {}
        self.ReservationLayer = {}
        self.BoundaryLatLayer = {}
        self.BoundaryLongLayer = {}

    def __iter__(self):
        for attr, value in self.__dict__.items():
            yield attr, value

    def add(self, bssd_object: BssdElement) -> BssdElement:
        # For a given instance of a BSSD element, this function adds that instance
        # to the respective layer of the BSSD map.

        if isinstance(bssd_object, BehaviorSpace):
            self.BehaviorSpaceLayer[bssd_object.id] = bssd_object
        elif isinstance(bssd_object, Behavior):
            self.BehaviorLayer[bssd_object.id] = bssd_object
        elif isinstance(bssd_object, Reservation):
            self.ReservationLayer[bssd_object.id] = bssd_object
        elif isinstance(bssd_object, BoundaryLat):
            self.BoundaryLatLayer[bssd_object.id] = bssd_object
        elif isinstance(bssd_object, BoundaryLong):
            self.BoundaryLongLayer[bssd_object.id] = bssd_object
        else:
            logger.warning(f'Non-BSSD-Object (ID: {bssd_object.id}) attempted to add to map_bssd')

        return bssd_object

    def create_placeholder(self, lanelet=None, long_boundary_along=None, long_boundary_against=None):
        """
        Function that creates an empty placeholderf or a behavior space and all subelements that are belonging
        to a behavior space in the BSSD.

        Parameters:
            lanelet (lanelet):Optional reference lanelet for the creation of a BehaviorSpace object.
            long_boundary_along (linestring):Optional linestring that represents the long boundary along the reference direction.
            long_boundary_against (linestring):Optional linestring that represents the long boundary against the reference direction.

        Returns:
            BehaviorSpace(BehaviorSpace):The created BehaviorSpace object.
        """

        # Check, if a lanelet is given
        if not lanelet:  # If not, not linestrings will be linked for the lateral boundaries.
            left_boundary_of_lanelet = None
            right_boundary_of_lanelet = None
        else:  # If yes, the linestrings will be linked to the objects of the lateral boundaries.
            left_boundary_of_lanelet = lanelet.leftBound
            right_boundary_of_lanelet = lanelet.rightBound

        # Creating behavior elements for both directions and adding them immediately to the BSSD map class
        # Also, the longitudinale boundary linestring and lateral linestrings are handed over to link them
        behavior_against = self.add(self.create_behavior(right_boundary_of_lanelet, left_boundary_of_lanelet, long_boundary_against))
        behavior_along = self.add(self.create_behavior(left_boundary_of_lanelet, right_boundary_of_lanelet, long_boundary_along))

        # Creating the BehaviorSpace element and adding it to the BSSD map class
        return self.add(BehaviorSpace(behavior_against, behavior_along, lanelet))

    def create_behavior(self, left_boundary, right_boundary, long_boundary):
        """
        Joins two dictionaries. Intended for dictionaries with partially mutual keys. This way the values of
        the two dictionaries for the same key are being combined in a list. This function is used for the segment search.

            Parameters:
                left_boundary (BoundaryLat):Lateral boundary object for the left boundary.
                right_boundary (BoundaryLat):Lateral boundary object for the right boundary.
                long_boundary (BoundaryLong):Longitudinal boundary object.

            Returns:
                Behavior(Behavior):The created Behavior object.
        """

        # Create objects for the lateral boundary elements and give them the linestring objects as an argument
        # for optional linkage.
        boundary_object_right = self.add(BoundaryLat(right_boundary))
        boundary_object_left = self.add(BoundaryLat(left_boundary))
        # Create an empty Reservation object
        reservation_object = self.add(Reservation())

        # Check, if a longitudinal boundary linestring is given.
        if long_boundary:  # If yes, create a longitudinal boundary object.
            b_long = self.add(BoundaryLong(long_boundary))
        else:  # If not, no longitudinal boundary object will be created
            b_long = None

        # Creating the Behavior element and adding it to the BSSD map class and eventually returning it
        return Behavior(boundary_left=boundary_object_left, boundary_right=boundary_object_right,
                        boundary_long=b_long, reservation=reservation_object)


class BssdElement:
    """
    This class is an abstract class for BSSD elements. The specific BSSD element classes inherit from this one.

    Attributes
    ----------
        attributes : BSSD Core object
            Creating an empty attribute attributes. For this the BSSD Core objects
            will be assigned in the specific functions.
        id : int
            The identification number for the bssd element
        visible : bool
            Specifies the visibility of an OSM object.
        version : int
            The version of an OSM object

    Methods
    -------
        __init__():
            This method is being inherited by the specific BSSD objects.
            It sets an unique ID and by OSM required information like visible and version.
        assign_to_attributes():
            assigns the attributes ID, visible and version to the BSSD Core object that is aggregated as attributes.
    """

    def __init__(self):
        self.attributes = None
        self.id = getId()
        self.visible = True
        self.version = 1

    def assign_to_attributes(self):
        self.attributes.visible = self.visible
        self.attributes.version = self.version
        self.attributes.id = self.id


class BehaviorSpace(BssdElement):
    """
    This class is a being used to represent BehaviorSpace objects. It inherits from the abstract BssdElement class.

    Attributes
    ----------
        alongBehavior : Behavior
            The Behavior object for along the reference direction.
        againstBehavior : Behavior
            The Behavior object for against the reference direction.
        ref_lanelet : Lanelet
            The lanelet object that the BehaviorSpace refers to.
        attributes : BehaviorSpace from BSSD Core
            Mutable BehaviorSpace object from BSSD Core.

    Methods
    -------
        __init__():
            If given, assigns behavior and lanelet objects. Assigns attributs that are given through BssdElements class
            to BSSD Core class which is added beforehand.
        assign_along(behavior):
            Assigns a given Behavior object to both the attribute and the BSSD Core object in 'attributes'. For the
            latter, it uses the ID and the add_along method.
        assign_agst(behavior):
            Assigns a given Behavior object to both the attribute and the BSSD Core object in 'attributes'. For the
            latter, it uses the ID and the add_against method.
        assign_lanelet(ll):
            assigns a given lanelet object to both the attribute and the BSSD Core object in 'attributes'. For the
            latter, it uses the ID and the add_lanelet method.
    """
    def __init__(self, behavior_against=None, behavior_along=None, lanelet=None):
        super().__init__()
        self.alongBehavior = None
        self.againstBehavior = None
        self.ref_lanelet = None
        self.attributes = mutable.BehaviorSpace()
        self.assign_to_attributes()

        if behavior_against:
            self.assign_against(behavior_against)

        if behavior_along:
            self.assign_along(behavior_along)

        if lanelet:
            self.assign_lanelet(lanelet)

    def __str__(self):
        return f'id: {self.id}, id behavior along: {self.alongBehavior.id},' \
               f' id behavior against: {self.againstBehavior.id}'

    def assign_along(self, behavior: Behavior):
        self.alongBehavior = behavior
        self.attributes.add_along(behavior.id)

    def assign_against(self, behavior: Behavior):
        self.againstBehavior = behavior
        self.attributes.add_against(behavior.id)

    def assign_lanelet(self, lanelet: Lanelet):
        self.ref_lanelet = lanelet
        self.attributes.add_lanelet(lanelet.id)


class Behavior(BssdElement):
    """
    This class is a being used to represent Behavior objects. It inherits from the abstract BssdElement class.

    Attributes
    ----------
        reservation : list
            A list that contains Reservation objects. Multiple Reservation objects can occur for a behavior.
        longBound : BoundaryLong
            The BoundaryLong object that represents the longitudinal boundary.
        leftBound : BoundaryLat
            The BoundaryLat object that represents the lateral left boundary.
        rightBound : BoundaryLat
            The BoundaryLat object that represents the lateral left boundary.
        attributes : Behavior from BSSD Core
            Mutable Behavior object from BSSD Core.

    Methods
    -------
        __init__(reservation, boundary_long, boundary_left, boundary_right):
            If given, assigns a Reservation and the three boundary objects. Assigns attributes that are given through
            BssdElements class to BSSD Core class which is added beforehand.
        assign_left_boundary(BoundaryLat):
            Assigns a given BoundaryLat object to both the attribute and the BSSD Core object in 'attributes'.
        assign_right_boundary(BoundaryLat):
            Assigns a given BoundaryLat object to both the attribute and the BSSD Core object in 'attributes'.
        assign_long_boundary(BoundaryLong):
            assigns a given BoundaryLong object to both the attribute and the BSSD Core object in 'attributes'.
        assign_reservation(Reservation):
            assigns a given Reservation object to both the attribute and the BSSD Core object in 'attributes'.
    """
    def __init__(self, reservation=None, boundary_long=None, boundary_left=None, boundary_right=None):
        super().__init__()
        self.reservation = []
        self.longBound = None
        self.leftBound = None
        self.rightBound = None
        self.attributes = mutable.Behavior()
        self.assign_to_attributes()

        if reservation:
            self.assign_reservation(reservation)

        if boundary_long:
            self.assign_long_boundary(boundary_long)

        if boundary_left:
            self.assign_left_boundary(boundary_left)

        if boundary_right:
            self.assign_right_boundary(boundary_right)

    def __str__(self):
        return f'id: {self.id}, id long boundary: {self.longBound.id}, ' \
               f'id left bound: {self.leftBound.id}, id right bound:  {self.rightBound.id}'

    def assign_left_boundary(self, boundary_linestring: BoundaryLat):
        self.leftBound = boundary_linestring
        self.attributes.add_boundary_left(boundary_linestring.id)

    def assign_right_boundary(self, boundary_linestring: BoundaryLat):
        self.rightBound = boundary_linestring
        self.attributes.add_boundary_right(boundary_linestring.id)

    def assign_long_boundary(self, boundary_linestring: BoundaryLong):
        self.longBound = boundary_linestring
        self.attributes.add_boundary_long(boundary_linestring.id)

    def assign_reservation(self, reservation: Reservation):
        self.reservation.append(reservation)
        self.attributes.add_reservation(reservation.id)


class Reservation(BssdElement):
    """
        This class is a being used to represent Reservation objects. It inherits from the abstract BssdElement class.

        Attributes
        ----------
            attributes : Reservation from BSSD Core
                Mutable Reservation object from BSSD Core.

        Methods
        -------
            __init__():
                Creates an empty BSSD Core Reservation object and saves it as 'attributes'
        """
    def __init__(self):
        super().__init__()
        self.attributes = mutable.Reservation()
        self.assign_to_attributes()

    def __str__(self):
        return f'id: {self.id}'


class BoundaryLat(BssdElement):
    """
    This class is a being used to represent BoundaryLat objects. It inherits from the abstract BssdElement class.

    Attributes
    ----------
        lineString : LineString2d | LineString3d
            A list that contains Reservation objects. Multiple Reservation objects can occur for a behavior.
        attributes : BoundaryLat from BSSD Core
            Mutable BoundaryLat object from BSSD Core.

    Methods
    -------
        __init__():
            If given, assigns a linestring object as the boundary. Assigns attributes that are given through
            BssdElements class to BSSD Core class which is added beforehand.
        assign_linestring(LineString2d | LineString3d):
            Assigns a given Linestring object to both the attribute and the BSSD Core object in 'attributes'.
    """
    def __init__(self, boundary_linestring=None):
        super().__init__()
        self.lineString = None
        self.attributes = mutable.BoundaryLat()
        self.assign_to_attributes()

        if boundary_linestring:
            self.assign_linestring(boundary_linestring)

    def __str__(self):
        return f'id: {self.id}, id linestring: {self.lineString.id}'

    def assign_linestring(self, linestring: LineString2d | LineString3d):
        self.lineString = linestring
        self.attributes.add_boundary(linestring.id)


class BoundaryLong(BssdElement):
    """
    This class is a being used to represent BoundaryLong objects. It inherits from the abstract BssdElement class.

    Attributes
    ----------
        lineString : LineString2d | LineString3d
            A list that contains Reservation objects. Multiple Reservation objects can occur for a behavior.
        attributes : BoundaryLong from BSSD Core
            Mutable Behavior object from BSSD Core.

    Methods
    -------
        __init__():
            If given, assigns a linestring object as the boundary. Assigns attributes that are given through
            BssdElements class to BSSD Core class which is added beforehand.
        assign_linestring(Linestring2d | Linestring3d):
            Assigns a given Linestring object to both the attribute and the BSSD Core object in 'attributes'.
    """
    def __init__(self, boundary_linestring=None):
        super().__init__()
        self.lineString = None
        self.ref_line = None
        self.attributes = mutable.BoundaryLong()
        self.assign_to_attributes()

        if boundary_linestring:
            self.assign_linestring(boundary_linestring)

    def __str__(self):
        return f'id: {self.id}, id linestring: {self.lineString.id}'

    def assign_linestring(self, linestring: LineString2d | LineString3d):
        self.lineString = linestring
        self.attributes.add_boundary(linestring.id)

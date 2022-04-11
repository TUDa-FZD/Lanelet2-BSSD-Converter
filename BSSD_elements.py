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
        create_placeholder(lanelet=None, bdr_alg=None, bdr_agst=None):
            Creates a placeholder BehaviorSpace and gives the opportunity to add longitudinal boundaries as well as lanelets.
        create_behavior(leftBdr, rightBdr, longBdr):
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

    def create_placeholder(self, lanelet=None, bdr_alg=None, bdr_agst=None):
        """
        Function that creates an empty placeholderf or a behavior space and all subelements that are belonging
        to a behavior space in the BSSD.

        Parameters:
            lanelet (lanelet):Optional reference lanelet for the creation of a BehaviorSpace object.
            bdr_alg (linestring):Optional linestring that represents the long boundary along the reference direction.
            bdr_agst (linestring):Optional linestring that represents the long boundary against the reference direction.

        Returns:
            BehaviorSpace(BehaviorSpace):The created BehaviorSpace object.
        """

        # Check, if a lanelet is given
        if not lanelet:  # If not, not linestrings will be linked for the lateral boundaries.
            lB_ll = None
            rB_ll = None
        else:  # If yes, the linestrings will be linked to the objects of the lateral boundaries.
            lB_ll = lanelet.leftBound
            rB_ll = lanelet.rightBound

        # Creating behavior elements for both directions and adding them immediately to the BSSD map class
        # Also, the longitudinale boundary linestring and lateral linestrings are handed over to link them
        behavior_agst = self.add(self.create_behavior(lB_ll, rB_ll, bdr_agst))
        behavior_alg = self.add(self.create_behavior(rB_ll, lB_ll, bdr_alg))

        # Creating the BehaviorSpace element and adding it to the BSSD map class
        return self.add(BehaviorSpace(b_agst=behavior_agst, b_alg=behavior_alg, ll=lanelet))

    def create_behavior(self, leftBdr, rightBdr, longBdr):
        '''
        Joins two dictionaries. Intended for dictionaries with partially mutual keys. This way the values of
        the two dictionaries for the same key are being combined in a list. This function is used for the segment search.

            Parameters:
                leftBdr (BoundaryLat):Lateral boundary object for the left boundary.
                rightBdr (BoundaryLat):Lateral boundary object for the right boundary.
                longBdr (BoundaryLong):Longitudinal boundary object.

            Returns:
                Behavior(Behavior):The created Behavior object.
        '''
        # Create objects for the lateral boundary elements and give them the linestring objects as an argument
        # for optional linkage.
        b_right = self.add(BoundaryLat(rightBdr))
        b_left = self.add(BoundaryLat(leftBdr))
        # Create an empty Reservation object
        res = self.add(Reservation())

        # Check, if a longitudinal boundary linestring is given.
        if longBdr:  # If yes, create a longitudinal boundary object.
            b_long = self.add(BoundaryLong(longBdr))
        else:  # If not, no longitudinal boundary object will be created
            b_long = None

        # Creating the Behavior element and adding it to the BSSD map class and eventually returning it
        return Behavior(bdr_left=b_left, bdr_right=b_right, bdr_long=b_long, res=res)


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
        assign_along(bhvr):
            Assigns a given Behavior object to both the attribute and the BSSD Core object in 'attributes'. For the
            latter, it uses the ID and the add_along method.
        assign_agst(bhvr):
            Assigns a given Behavior object to both the attribute and the BSSD Core object in 'attributes'. For the
            latter, it uses the ID and the add_against method.
        assign_lanelet(ll):
            assigns a given lanelet object to both the attribute and the BSSD Core object in 'attributes'. For the
            latter, it uses the ID and the add_lanelet method.
    """
    def __init__(self, b_agst=None, b_alg=None, ll=None):
        super().__init__()
        self.alongBehavior = None
        self.againstBehavior = None
        self.ref_lanelet = None
        self.attributes = mutable.BehaviorSpace()
        self.assign_to_attributes()

        if b_agst:
            self.assign_agst(b_agst)

        if b_alg:
            self.assign_along(b_alg)

        if ll:
            self.assign_lanelet(ll)

    def __str__(self):
        # Print ID and Behavior ID for agst and alg
        id_str = 'id: ' + str(self.id)
        behavior_agst = ', id behavior along: ' + str(self.alongBehavior.id)
        behavior_alg = ', id behavior against: ' + str(self.againstBehavior.id)

        # return str([id_str, behavior_agst, behavior_alg])
        return id_str + behavior_agst + behavior_alg

    def assign_along(self, bhvr: Behavior):
        self.alongBehavior = bhvr
        self.attributes.add_along(bhvr.id)

    def assign_agst(self, bhvr: Behavior):
        self.againstBehavior = bhvr
        self.attributes.add_against(bhvr.id)

    def assign_lanelet(self, ll: Lanelet):
        self.ref_lanelet = ll
        self.attributes.add_lanelet(ll.id)


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
        __init__(res, bdr_long, bdr_left, bdr_right):
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
    def __init__(self, res=None, bdr_long=None, bdr_left=None, bdr_right=None):
        super().__init__()
        self.reservation = []
        self.longBound = None
        self.leftBound = None
        self.rightBound = None
        self.attributes = mutable.Behavior()
        self.assign_to_attributes()

        # Todo: Give an option for multiple reservation instances

        if res:
            self.assign_reservation(res)

        if bdr_long:
            self.assign_long_boundary(bdr_long)

        if bdr_left:
            self.assign_left_boundary(bdr_left)

        if bdr_right:
            self.assign_right_boundary(bdr_right)


    def __str__(self):
        # Print ID and
        id_str = 'id: ' + str(self.id)
        # bs = ', id behavior space: ' + str(self.id_bs)

        return id_str

    def assign_left_boundary(self, bound: BoundaryLat):
        self.leftBound = bound
        self.attributes.add_boundary_left(bound.id)

    def assign_right_boundary(self, bound: BoundaryLat):
        self.rightBound = bound
        self.attributes.add_boundary_right(bound.id)

    def assign_long_boundary(self, bound: BoundaryLong):
        self.longBound = bound
        self.attributes.add_boundary_long(bound.id)

    def assign_reservation(self, res: Reservation):
        self.reservation.append(res)
        self.attributes.add_reservation(res.id)


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
        # Print ID and
        id_str = 'id: ' + str(self.id)
        # b = ', id behavior: ' + str(self.id_b)
        b = ', id behavior: ' + str(0)

        return id_str + b


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
        assign_ls(LineString2d | LineString3d):
            Assigns a given Linestring object to both the attribute and the BSSD Core object in 'attributes'.
    """
    def __init__(self, bdr=None):
        super().__init__()
        self.lineString = None
        self.attributes = mutable.BoundaryLat()
        self.assign_to_attributes()

        if bdr:
            self.assign_ls(bdr)

    def __str__(self):
        # Print ID and
        id_str = 'id: ' + str(self.id)
        bdr = ', id linestring: ' + str(self.lineString.id)
        return id_str + bdr

    def assign_ls(self, ls: LineString2d | LineString3d):
        self.lineString = ls
        self.attributes.add_boundary(ls.id)


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
        assign_ls(Linestring2d | Linestring3d):
            Assigns a given Linestring object to both the attribute and the BSSD Core object in 'attributes'.
    """
    def __init__(self, bdr=None):
        super().__init__()
        self.lineString = None
        self.ref_line = None
        self.attributes = mutable.BoundaryLong()
        self.assign_to_attributes()

        if bdr:
            self.assign_ls(bdr)

    def __str__(self):
        # Print ID and
        id_str = 'id: ' + str(self.id)
        bdr = ', id linestring: ' + str(self.lineString.id)
        return id_str+bdr

    def assign_ls(self, ls: LineString2d | LineString3d):
        self.lineString = ls
        self.attributes.add_boundary(ls.id)

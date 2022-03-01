from lanelet2.core import AttributeMap, TrafficLight, Lanelet, LineString3d, Point2d, Point3d, getId, \
    LaneletMap, BoundingBox2d, BasicPoint2d
import constants
import logging
from bssd.core import mutable
import bssd
import osmium
logger = logging.getLogger(__name__)


class bssdClass:

    def __init__(self):
        self.BehaviorSpaceLayer = {}
        self.BehaviorLayer = {}
        self.ReservationLayer = {}
        self.BoundaryLatLayer = {}
        self.BoundaryLongLayer = {}

    def __iter__(self):
        for attr, value in self.__dict__.items():
            yield attr, value

    def add(self, bssdObject):
        # For a given instance of a BSSD class, this function adds that instance
        # to the respective layer of the BSSD map. Furthermore it looks for all subclasses
        # and adds them to their respective classe, too.

        def get_all_subelements(cls):
            all_subelements = [cls]

            for subelement in cls.get_subelements():
                all_subelements.append(subelement)
                all_subelements.extend(get_all_subelements(subelement))

            return all_subelements

        for bssd_class in get_all_subelements(bssdObject):
            if isinstance(bssd_class, BehaviorSpace):
                self.BehaviorSpaceLayer[bssd_class.id] = bssd_class
            elif isinstance(bssd_class, Behavior):
                self.BehaviorLayer[bssd_class.id] = bssd_class
            elif isinstance(bssd_class, Reservation):
                self.ReservationLayer[bssd_class.id] = bssd_class
            elif isinstance(bssd_class, BoundaryLat):
                self.BoundaryLatLayer[bssd_class.id] = bssd_class
            elif isinstance(bssd_class, BoundaryLong):
                self.BoundaryLongLayer[bssd_class.id] = bssd_class


class BSSD_element():

    def __init__(self):
        self.id = getId()
        self.visible = True
        self.version = 1
        self.tags = {}

    def __str__(self):
        pass

    def __eq__(self, other):
        # Function for comparing behavior
        if not type(self) == type(other):
            return False
        return self.tags == other.tags

    def get_subelements(self):
        return []


class BehaviorSpace(BSSD_element):

    def __init__(self, b_agst, b_alg, ll=None):
        super().__init__()
        self.alongBehavior = b_agst
        self.againstBehavior = b_alg
        self.ref_lanelet = ll
        self.tags = {'type': 'behavior_space', 'subtype': 'along'}
        self.members = [('r', self.alongBehavior.id, 'along'),
                        ('r', self.againstBehavior.id, 'against')]

        if ll:
            self.members.append(('r', ll, 'ref_lanelet'))

    def __str__(self):
        # Print ID and Behavior ID for agst and alg
        id_str = 'id: ' + str(self.id)
        behavior_agst = ', id behavior along: ' + str(self.alongBehavior.id)
        behavior_alg = ', id behavior against: ' + str(self.againstBehavior.id)

        # return str([id_str, behavior_agst, behavior_alg])
        return id_str + behavior_agst + behavior_alg

    def __eq__(self, other):
        # Function for comparing behavior of two behavior spaces
        if not type(self) == type(other):
            return False
        elif (not self.alongBehavior == other.alongBehavior or
              not self.againstBehavior == other.againstBehavior):
            return False
        return self.tags == other.tags

    def __add__(self, other):
        # Function for joining two behavior spaces

        # 1. Replace long Boundary of self by longBoundary of other

        # 2. Create new linestrings by concatenate linestrings from left/right boundary

        # 3. Assign new lateral boundaries to self.Behavior.leftBoundary/rightBoundary

        # 4. Reference both underlying lanelets

        # 5. Kill instances of other (including Behaviors and boundaries)
        pass

    def get_subelements(self):
        return [self.alongBehavior, self.againstBehavior]


class Behavior(BSSD_element):

    def __init__(self, res, bdr_long, bdr_left, bdr_right):
        super().__init__()
        self.reservation = res
        self.longBound = bdr_long
        self.leftBound = bdr_left
        self.rightBound = bdr_right
        self.tags.update(constants.BEHAVIOR_TAGS)
        # Todo: Give an option for multiple reservation instances
        self.members = [('r', self.reservation.id, 'reservation'),
                        ('r', self.leftBound.id, 'boundary_left'),
                        ('r', self.rightBound.id, 'boundary_right')
                        ]

        if bdr_long:
            self.members.append(('r', self.longBound.id, 'boundary_long'))

    def __str__(self):
        # Print ID and
        id_str = 'id: ' + str(self.id)
        # bs = ', id behavior space: ' + str(self.id_bs)

        return id_str

    def __eq__(self, other):
        # Function for comparing behavior
        if not type(self) == type(other):
            return False
        elif (not self.leftBound == other.leftBound or
              not self.rightBound == other.rightBound or
              not self.longBound == other.longBound or
              not self.reservation == other.reservation):
            return False
        return self.tags == other.tags

    def get_subelements(self):
        element_list = [self.reservation, self.leftBound, self.rightBound]
        if self.longBound:
            element_list.append(self.longBound)
        return element_list

    def assign_long_ref(self, ref, direction):
        if self.longBound:
            self.longBound.ref_line = ref
            #logger.debug(f'Behavior {self.id} ({direction}) long ref: {ref}')


class Reservation(BSSD_element):

    def __init__(self):
        super().__init__()
        # self.id_b = id_b
        self.tags.update(constants.RESERVATION_TAGS)

    def __str__(self):
        # Print ID and
        id_str = 'id: ' + str(self.id)
        # b = ', id behavior: ' + str(self.id_b)
        b = ', id behavior: ' + str(0)

        return id_str + b


class BoundaryLat(BSSD_element):

    def __init__(self, bdr=None):
        super().__init__()
        self.lineString = bdr
        self.tags.update(constants.BOUNDARY_LAT_TAGS)

        if bdr:
            self.members = [('w', self.lineString.id, 'boundary')]
        else:
            self.members = [('w', -1, 'boundary')]

    def __str__(self):
        # Print ID and
        id_str = 'id: ' + str(self.id)
        bdr = ', id linestring: ' + str(self.lineString.id)

        return id_str + bdr


class BoundaryLong(BSSD_element):

    def __init__(self, bdr=None):
        super().__init__()
        self.lineString = bdr
        self.tags.update(constants.BOUNDARY_LONG_TAGS)
        self.ref_line = None

        if bdr:
            self.members = [('w', self.lineString.id, 'boundary')]
        else:
            self.members = [('w', -1, 'boundary')]

    def __str__(self):
        # Print ID and
        id_str = 'id: ' + str(self.id)
        bdr = ', id linestring: ' + str(self.lineString.id)


def create_placeholder(lanelet=None, bdr_agst=None, bdr_alg=None):
    # Function that calls code to create empty placeholders for all objects that
    # are necessary for a behavior space in the BSSD.
    def create_behavior(leftBdr, rightBdr, longBdr):
        if longBdr:
            longBound = BoundaryLong(longBdr)
        else:
            longBound = None
        return Behavior(Reservation(), longBound, BoundaryLat(leftBdr), BoundaryLat(rightBdr))

    if not lanelet:
        lB_ll = None
        rB_ll = None
        id_ll = None
    else:
        lB_ll = lanelet.leftBound
        rB_ll = lanelet.rightBound
        id_ll = lanelet.id

    behavior_agst = create_behavior(lB_ll, rB_ll, bdr_agst)
    behavior_alg = create_behavior(rB_ll, lB_ll, bdr_alg)

    return BehaviorSpace(behavior_agst, behavior_alg, id_ll)

from lanelet2.core import AttributeMap, TrafficLight, Lanelet, LineString3d, Point2d, Point3d, getId, \
    LaneletMap, BoundingBox2d, BasicPoint2d
import constants
import logging
from bssd.core import mutable
import bssd
import osmium
from bssd.core._types import CrossingType as ct
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

        if isinstance(bssdObject, BehaviorSpace):
            self.BehaviorSpaceLayer[bssdObject.id] = bssdObject
        elif isinstance(bssdObject, Behavior):
            self.BehaviorLayer[bssdObject.id] = bssdObject
        elif isinstance(bssdObject, Reservation):
            self.ReservationLayer[bssdObject.id] = bssdObject
        elif isinstance(bssdObject, BoundaryLat):
            self.BoundaryLatLayer[bssdObject.id] = bssdObject
        elif isinstance(bssdObject, BoundaryLong):
            self.BoundaryLongLayer[bssdObject.id] = bssdObject
        else:
            logger.warning(f'Non-BSSD-Object (ID: {bssdObject.id}) attempted to add to map_bssd')

        return bssdObject

    def create_placeholder(self, lanelet=None, bdr_agst=None, bdr_alg=None):
        # Function that calls code to create empty placeholders for all objects that
        # are necessary for a behavior space in the BSSD.

        if not lanelet:
            lB_ll = None
            rB_ll = None
        else:
            lB_ll = lanelet.leftBound
            rB_ll = lanelet.rightBound

        behavior_agst = self.add(self.create_behavior(lB_ll, rB_ll, bdr_agst))
        behavior_alg = self.add(self.create_behavior(rB_ll, lB_ll, bdr_alg))

        return self.add(BehaviorSpace(b_agst=behavior_agst, b_alg=behavior_alg, ll=lanelet))

    def create_behavior(self, leftBdr, rightBdr, longBdr):

        b_right = self.add(BoundaryLat(rightBdr))
        b_left = self.add(BoundaryLat(leftBdr))
        res = self.add(Reservation())

        if longBdr:
            b_long = self.add(BoundaryLong(longBdr))
        else:
            b_long = None

        return Behavior(bdr_left=b_left, bdr_right=b_right, bdr_long=b_long, res=res)


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


class BehaviorSpace(mutable.BehaviorSpace):

    def __init__(self, b_agst=None, b_alg=None, ll=None):
        super().__init__()
        self.alongBehavior = None
        self.againstBehavior = None
        self.ref_lanelet = None
        set_initial_values(self)

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

    def __eq__(self, other):
        # Function for comparing behavior of two behavior spaces
        if not type(self) == type(other):
            return False
        elif (not self.alongBehavior == other.alongBehavior or
              not self.againstBehavior == other.againstBehavior):
            return False
        return self.tags == other.tags

    def assign_along(self, bhvr):
        self.alongBehavior = bhvr
        self.add_along(bhvr.id)

    def assign_agst(self, bhvr):
        self.againstBehavior = bhvr
        self.add_against(bhvr.id)

    def assign_lanelet(self, ll):
        self.ref_lanelet = ll
        self.add_lanelet(ll.id)


class Behavior(mutable.Behavior):

    def __init__(self, res=None, bdr_long=None, bdr_left=None, bdr_right=None):
        super().__init__()
        self.reservation_sub = []
        self.longBound = None
        self.leftBound = None
        self.rightBound = None
        set_initial_values(self)
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

    def assign_long_ref(self, ref, direction):
        if self.longBound:
            self.longBound.ref_line = ref
            #logger.debug(f'Behavior {self.id} ({direction}) long ref: {ref}')

    def assign_left_boundary(self, bound):
        self.leftBound = bound
        self.add_boundary_left(bound.id)

    def assign_right_boundary(self, bound):
        self.rightBound = bound
        self.add_boundary_right(bound.id)

    def assign_long_boundary(self, bound):
        self.longBound = bound
        self.add_boundary_long(bound.id)

    def assign_reservation(self, res):
        self.reservation_sub.append(res)
        self.add_reservation(res.id)


class Reservation(mutable.Reservation):

    def __init__(self):
        super().__init__()
        set_initial_values(self)

    def __str__(self):
        # Print ID and
        id_str = 'id: ' + str(self.id)
        # b = ', id behavior: ' + str(self.id_b)
        b = ', id behavior: ' + str(0)

        return id_str + b


class BoundaryLat(mutable.BoundaryLat):

    def __init__(self, bdr=None):
        super().__init__()
        self.lineString = None
        set_initial_values(self)
        #self.crossing = ct.PROHIBITED
        if bdr:
            self.assign_ls(bdr)

    def __str__(self):
        # Print ID and
        id_str = 'id: ' + str(self.id)
        bdr = ', id linestring: ' + str(self.lineString.id)

        return id_str + bdr

    def assign_ls(self, ls):
        self.lineString = ls
        self.add_boundary(ls.id)


class BoundaryLong(mutable.BoundaryLong):

    def __init__(self, bdr=None):
        super().__init__()
        self.lineString = None
        self.ref_line = None
        set_initial_values(self)

        if bdr:
            self.assign_ls(bdr)

    def __str__(self):
        # Print ID and
        id_str = 'id: ' + str(self.id)
        bdr = ', id linestring: ' + str(self.lineString.id)

    def assign_ls(self, ls):
        self.lineString = ls
        self.add_boundary(ls.id)


# def create_placeholder(lanelet=None, bdr_agst=None, bdr_alg=None):
#     # Function that calls code to create empty placeholders for all objects that
#     # are necessary for a behavior space in the BSSD.
#
#     if not lanelet:
#         lB_ll = None
#         rB_ll = None
#     else:
#         lB_ll = lanelet.leftBound
#         rB_ll = lanelet.rightBound
#
#     behavior_agst = create_behavior(lB_ll, rB_ll, bdr_agst)
#     behavior_alg = create_behavior(rB_ll, lB_ll, bdr_alg)
#
#     return BehaviorSpace(b_agst=behavior_agst, b_alg=behavior_alg, ll=lanelet)
#
#
# def create_behavior(leftBdr, rightBdr, longBdr):
#
#     b_right = BoundaryLat(rightBdr)
#     b_left = BoundaryLat(leftBdr)
#     res = Reservation()
#
#     if longBdr:
#         b_long = BoundaryLong(longBdr)
#     else:
#         b_long = None
#
#     return Behavior(bdr_left=b_left, bdr_right=b_right, bdr_long=b_long, res=res)


def set_initial_values(rel):
    rel.id = getId()
    rel.visible = True
    rel.version = 1

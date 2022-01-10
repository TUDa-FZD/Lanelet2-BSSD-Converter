from lanelet2.core import AttributeMap, TrafficLight, Lanelet, LineString3d, Point2d, Point3d, getId, \
    LaneletMap, BoundingBox2d, BasicPoint2d
import constants


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
        if isinstance(bssdObject, BehaviorSpace):
            self.BehaviorSpaceLayer[bssdObject.id] = bssdObject
        elif isinstance(bssdObject, Behavior):
            self.BehaviorLayer[bssdObject.id] = bssdObject
        elif isinstance(bssdObject, Reservation):
            self.ReservationLayer[bssdObject.id] = bssdObject
        elif isinstance(bssdObject, Boundary_lat):
            self.BoundaryLatLayer[bssdObject.id] = bssdObject
        elif isinstance(bssdObject, Boundary_long):
            self.BoundaryLongLayer[bssdObject.id] = bssdObject


class BSSD_element:

    def __init__(self):
        self.id = getId()
        self.visible = True
        self.version = 1

    def __str__(self):
        pass


class BehaviorSpace(BSSD_element):

    def __init__(self, b_fwd, b_bwd, ll=None):
        super().__init__()
        self.B_fwd_id = b_fwd
        self.B_bwd_id = b_bwd
        self.ref_lanelet = ll
        self.tags = {'type': 'behavior_space', 'subtype': 'forward'}
        self.members = [('r', self.B_fwd_id, 'forward'), ('r', self.B_bwd_id, 'backward')]

        if ll:
            self.members.append(('r', id_ll, 'ref lanelet'))

    def __str__(self):
        # Print ID and Behavior ID for fwd and bwd
        id_str = 'id: ' + str(self.id)
        behavior_fwd = ', id behavior forward: ' + str(self.B_fwd_id)
        behavior_bwd = ', id behavior backward: ' + str(self.B_bwd_id)

        # return str([id_str, behavior_fwd, behavior_bwd])
        return id_str + behavior_fwd + behavior_bwd

    def __add__(self, bs_other):
        # Function for joining two behavior spaces
        pass


class Behavior(BSSD_element):

    def __init__(self, id_res, id_bd_long, id_bd_l, id_bd_r):
        super().__init__()
        self.id_res = id_res
        self.id_bd_long = id_bd_long
        self.id_bd_l = id_bd_l
        self.id_bd_r = id_bd_r
        self.tags = constants.BEHAVIOR_TAGS

        self.members = [('r', self.id_res, 'reservation'),
                        ('r', self.id_bd_r, 'boundary_left'),
                        ('r', self.id_bd_r, 'boundary_left')
                        ]

        if id_bd_long:
            self.members.append(('r', self.id_bd_long, 'boundary_long'))


    def __str__(self):
        # Print ID and
        id_str = 'id: ' + str(self.id)
        # bs = ', id behavior space: ' + str(self.id_bs)

        return id_str


class Reservation(BSSD_element):

    def __init__(self):
        super().__init__()
        # self.id_b = id_b
        self.tags = constants.RESERVATION_TAGS

    def __str__(self):
        # Print ID and
        id_str = 'id: ' + str(self.id)
        # b = ', id behavior: ' + str(self.id_b)
        b = ', id behavior: ' + str(0)

        return id_str + b


class Boundary_lat(BSSD_element):

    def __init__(self, id_bdr=0):
        super().__init__()
        self.id_bdr = id_bdr
        self.tags = constants.BOUNDARY_LAT_TAGS
        self.members = [('w', self.id_bdr, 'boundary')]

    def __str__(self):
        # Print ID and
        id_str = 'id: ' + str(self.id)
        bdr = ', id boundary: ' + str(self.id_bdr)

        return id_str + bdr


class Boundary_long(BSSD_element):

    def __init__(self, id_bdr=0):
        super().__init__()
        self.id_bdr = id_bdr
        self.tags = constants.BOUNDARY_LONG_TAGS
        self.members = [('w', self.id_bdr, 'boundary')]

    def __str__(self):
        # Print ID and
        id_str = 'id: ' + str(self.id)
        # b = ', id behavior: ' + str(self.id_b)
        b = ', id behavior: ' + str(0)
        bdr = ', id boundary: ' + str(self.id_bdr)

        return id_str + b + bdr


def create_placeholder(bssd_map, lanelet=None, id_bdr_fwd=-1, id_bdr_bwd=-1):
    # Function that calls code to create empty placeholders for all objects that are necessary for
    # a behavior space in the BSSD.
    def create_behavior(id_leftBdr, id_rightBdr, id_longBdr):
        leftBound = Boundary_lat(id_leftBdr)
        bssd_map.add(leftBound)
        rightBound = Boundary_lat(id_rightBdr)
        bssd_map.add(rightBound)
        if id_longBdr:
            longBound = Boundary_long(id_longBdr)
            bssd_map.add(longBound)
            id_relation_longBdr = longBound.id
        else:
            id_relation_longBdr = None
        reservation = Reservation()
        bssd_map.add(reservation)
        behavior = Behavior(reservation.id, id_relation_longBdr, leftBound.id, rightBound.id)
        bssd_map.add(behavior)
        return behavior

    if not lanelet:
        id_lB_ll = 0
        id_rB_ll = 0
        id_ll = 0
    else:
        id_lB_ll = lanelet.leftBound.id
        id_rB_ll = lanelet.rightBound.id
        id_ll = lanelet.id

    behavior_fwd = create_behavior(id_lB_ll, id_rB_ll, id_bdr_fwd)
    behavior_bwd = create_behavior(id_rB_ll, id_lB_ll, id_bdr_bwd)

    bs = BehaviorSpace(behavior_fwd.id, behavior_bwd.id, id_ll)
    bssd_map.add(bs)

    return bs
from BSSD_derivation_for_Lanelet2 import io_handler
from BSSD_derivation_for_Lanelet2 import data_handler
from BSSD_derivation_for_Lanelet2 import BSSD_elements

io = io_handler.IoHandler('test/DA_Nieder-Ramst-Mühlstr-Hochstr.osm')
map_lanelet = io.load_map()
data = data_handler.DataHandler(map_lanelet, [], None)


def test_identify_longitudinal_boundary():
    """
    Check, if stopline is being identified and linestrings are derived correctly.
    """
    point_1 = map_lanelet.pointLayer[1246]
    point_2 = map_lanelet.pointLayer[1248]

    linestring, ref_line = data.identify_longitudinal_boundary(point_1, point_2, False, None)

    assert ref_line == 1366

    point_1 = map_lanelet.pointLayer[1136]
    point_2 = map_lanelet.pointLayer[1171]

    linestring, ref_line = data.identify_longitudinal_boundary(point_1, point_2, False, None)

    assert ref_line == 1344
    assert point_1 in linestring and point_2 in linestring
    assert len(linestring) == 2


def test_derive_behavior_boundary_lateral():
    """
    Check, parking areas are identified and the CrossingType is derived correctly.
    """

    boundary_left = BSSD_elements.BoundaryLat(map_lanelet.lineStringLayer[1325])
    boundary_right = BSSD_elements.BoundaryLat(map_lanelet.lineStringLayer[1417])
    behavior_1 = BSSD_elements.Behavior(boundary_left=boundary_left, boundary_right=boundary_right)
    behavior_2 = BSSD_elements.Behavior(boundary_left=boundary_right, boundary_right=boundary_left)

    data.derive_behavior_boundary_lateral(behavior_1, behavior_2, 'left')
    data.derive_behavior_boundary_lateral(behavior_2, behavior_1, 'right')

    assert behavior_1.leftBound.attributes.parking_only
    assert not behavior_1.rightBound.attributes.parking_only

    assert behavior_1.rightBound.attributes.crossing == 'prohibited'


def test_find_adjacent():
    """
    Check, if lanelets of a segment are identified correctly.
    """
    lanelets = data.find_adjacent(map_lanelet.laneletLayer[1450], 0, None)

    assert len(lanelets[1]) == 2


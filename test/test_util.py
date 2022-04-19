from lanelet2.core import Lanelet, LineString3d, Point3d
from BSSD_derivation_for_Lanelet2 import util


def test_angle_between_lanelets():
    """
    Check, if angles between lanelets are calculated correctly based on their centerlines.
    """

    p1 = Point3d(2, 1, 0, 0)
    p2 = Point3d(3, 1, 1, 0)
    p3 = Point3d(4, 0, 0, 0)
    p4 = Point3d(5, 0, 1, 0)
    lanelet_1 = Lanelet(1, LineString3d(6, [p1, p2]), LineString3d(7, [p3, p4]))
    lanelet_2 = Lanelet(2, LineString3d(6, [p3, p1]), LineString3d(7, [p4, p2]))

    assert round(util.angle_between_lanelets(lanelet_1, lanelet_2)) == 90
    lanelet_2 = Lanelet(2, LineString3d(6, [p2, p1]), LineString3d(7, [p4, p3]))
    assert round(util.angle_between_lanelets(lanelet_1, lanelet_2)) == 180
    assert round(util.angle_between_lanelets(lanelet_1, lanelet_1)) == 0


def test_angle_between_linestrings():
    """
    Check, if angles between linestrings are calculated correctly.
    """
    p1 = Point3d(2, 1, 0, 0)
    p2 = Point3d(3, 1, 1, 0)
    p3 = Point3d(4, 0, 0, 0)
    linestring_1 = LineString3d(6, [p1, p2])
    linestring_2 = LineString3d(6, [p3, p1])

    assert round(util.angle_between_linestrings(linestring_1, linestring_2)) == 90
    linestring_3 = LineString3d(6, [p3, p2])
    assert round(util.angle_between_linestrings(linestring_1, linestring_3)) == 45

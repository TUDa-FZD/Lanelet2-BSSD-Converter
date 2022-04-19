from lanelet2.core import AttributeMap, Lanelet, LineString3d, Point3d
from BSSD_derivation_for_Lanelet2 import io_handler
from BSSD_derivation_for_Lanelet2 import preprocessing

io = io_handler.IoHandler('test/DA_Nieder-Ramst-Mühlstr-Hochstr.osm')
map_lanelet = io.load_map()


def test_is_lanelet_relevant():
    """
    Check, if  different lanelets are considered relevant or not.
    """

    attributes = AttributeMap({'type': 'lanelet', 'subtype': 'road'})
    assert preprocessing.is_lanelet_relevant(attributes)
    attributes = AttributeMap({'type': 'lanelet',
                               'subtype': 'road',
                               'participant:bicycle': 'yes',
                               'participant:vehicle': 'no',
                               })
    assert not preprocessing.is_lanelet_relevant(attributes)


def test_is_bicycle_lanelet_relevant():
    """
    Create test bicycle lanelets and check if they are considered as relevant.
    """

    attributes = AttributeMap({'type': 'lanelet', 'subtype': 'road'})
    p = Point3d(2, 1, 0, 0)
    p2 = Point3d(3, 1, 1, 0)
    p3 = Point3d(4, 0, 0, 0)
    p4 = Point3d(5, 0, 1, 0)
    lanelet = Lanelet(1, LineString3d(6, [p, p2]), LineString3d(7, [p3, p4]), attributes)
    neighbors = [lanelet]
    linestring_attributes = AttributeMap({'type': 'line_thin', 'subtype': 'dashed'})
    assert preprocessing.is_bicycle_lanelet_relevant(neighbors, linestring_attributes)
    attributes = AttributeMap({'type': 'lanelet',
                               'subtype': 'road',
                               'participant:bicycle': 'yes',
                               'participant:vehicle': 'no',
                               })
    lanelet = Lanelet(8, LineString3d(6, [p, p2]), LineString3d(7, [p3, p4]), attributes)

    assert not preprocessing.is_bicycle_lanelet_relevant([lanelet], linestring_attributes)


def test_routing_graph_all():
    """
    Check, whether pedestrian and bus lanelets are part of the RoutingGraph.
    """
    preprocessor = preprocessing.Preprocessing(map_lanelet)
    routing_graph = preprocessor.get_routing_graph_all()

    assert len(routing_graph.following(map_lanelet.laneletLayer[1480])) > 0
    assert len(routing_graph.following(map_lanelet.laneletLayer[1479])) > 0

import lanelet2
import os
import time
from lanelet2.core import AttributeMap, TrafficLight, Lanelet, LineString3d, Point2d, Point3d, getId, \
    LaneletMap, BoundingBox2d, BoundingBox3d, BasicPoint2d, BasicPoint3d
from lanelet2.geometry import distance as dist
import BSSD_elements
from BSSD_elements import create_placeholder
import constants
import io_data
import math


def main():
    # Process Lanelet2 map and derive behavior spaces

    # ----------- INPUT --------------
    # Load example file from lanelet2
    filename = "res/rwth_campus-melaten_UNICARagil_lanelet2_edit.osm"
    example_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
    pos_ids_file_path = "/home/jannik/Documents/testws/src/lanelet2/lanelet2_python/scripts/make_ids_positive.py"
    os.system(f"python3 {pos_ids_file_path} -i {example_file}")
    map_ll = io_data.load_map(example_file)

    # ------- PREPROCESSING --------------
    # Make list with all IDs of lanelets that are relevant
    start = time.perf_counter()
    ll_rel = [item.id for item in map_ll.laneletLayer if ll_relevant(item.attributes)]
    ll_bicycle = [item for item in map_ll.laneletLayer if item.attributes['subtype'] == 'bicycle_lane']

    for ll in ll_bicycle:
        nbrs_left = map_ll.laneletLayer.findUsages(ll.leftBound)
        nbrs_left.remove(ll)
        nbrs_right = map_ll.laneletLayer.findUsages(ll.rightBound)
        nbrs_right.remove(ll)

        if len(nbrs_left) > 1 or len(nbrs_right) > 1:
            print(f"For bicycle_lane with ID {ll.id}: Multiple neighbors have been found")

        if (nbrs_left and ls_of_pbl(ll.leftBound.attributes) and ll_relevant(nbrs_left[0].attributes)) or \
                (nbrs_right and ls_of_pbl(ll.rightBound.attributes) and ll_relevant(nbrs_right[0].attributes)):
            ll.attributes['subtype_alt'] = 'bicycle_lane_protected'
            ll.attributes['participant:vehicle'] = 'yes'
            ll.attributes['participant:bicycle'] = 'yes'
            ll_rel.append(ll.id)
            ll_bicycle.remove(ll)

    graph = io_data.get_RoutingGraph(map_ll)

    BSSD = BSSD_elements.bssdClass()
    end = time.perf_counter()
    print(f"Preprocessing done. Elapsed time: {round(end - start, 2)}")

    # ----------- PROCESSING --------------
    # Recursively loop through all lanelets to perform desired actions for each (e.g. derive long. boundary)
    start = time.perf_counter()
    while ll_rel:
        # print('start')
        BSSD, ll_rel = ll_recursion(map_ll.laneletLayer[ll_rel[0]], ll_rel, graph, map_ll, BSSD)
    end = time.perf_counter()
    print(f"Processing done. Elapsed time: {round(end - start, 2)}")

    # ----------- OUTPUT --------------
    # Save edited .osm-map to desired file
    start = time.perf_counter()
    file1 = "map_ll.osm"
    file2 = "map_bssd.osm"
    io_data.save_map(map_ll, file1)

    io_data.write_bssd_elements(BSSD, file2)
    io_data.merge_files(file1, file2, filename[3:])
    end = time.perf_counter()
    print(f"BSSD file is written. Elapsed time: {round(end - start, 2)}")


def ll_recursion(ll, list_relevant_ll, graph, map_ll, map_bssd, direction=None, ls=None):

    if ll.id in list_relevant_ll:
        # Remove current lanelet from list of relevant lanelets to keep track which lanelets still have to be done
        list_relevant_ll.remove(ll.id)

        # Perform derivation of behavior space from current lanelet
        # atm only long. boundary
        # fwd_ls, bwd_ls, map_ll = derive_abs_geom(ll, map_ll, direction, ls)
        map_ll, fwd_ls = get_long_bdr(map_ll, ll.leftBound[-1], ll.rightBound[-1], 'fwd', direction, ls)
        map_ll, bwd_ls = get_long_bdr(map_ll, ll.leftBound[0], ll.rightBound[0], 'bwd', direction, ls)
        ## derive abs behavior
        new_bs = create_placeholder(ll, fwd_ls, bwd_ls)
        map_bssd.add(new_bs)
        # Call function in itself for the succeeding and preceding lanelet and hand over information
        # about already created boundaries
        for successor in graph.following(ll):
            map_bssd, list_relevant_ll = ll_recursion(successor, list_relevant_ll, graph, map_ll, map_bssd, 'fwd', fwd_ls)
        for predecessor in graph.previous(ll):
            map_bssd, list_relevant_ll = ll_recursion(predecessor,  list_relevant_ll, graph, map_ll, map_bssd, 'bwd', bwd_ls)
            
    return map_bssd, list_relevant_ll


def ll_relevant(att):
    # Determine the relevance of a lanelet by first checking its subtype (for instance: shouldn't be "stairs")
    # and second if any overriding participant-tags are being used

    if att['subtype'] in constants.SUBTYPE_TAGS:

        if any('participant' in key.lower() for key, value in att.items()):

            if any(value == 'yes' for key, value in att.items() if 'participant:vehicle' in key.lower()):
                relevant = True
            else:
                relevant = False

        else:
            relevant = True

    else:
        relevant = False

    return relevant


def ls_of_pbl(att):
    if ('line' in att['type'] and att['subtype'] == 'dashed') or att['type'] == 'virtual':
        return True
    else:
        return False


def get_long_bdr(map_lanelet, pt_left, pt_right, side, direction, d_id):

    ls = None

    llLayer = map_lanelet.laneletLayer
    lsLayer = map_lanelet.lineStringLayer
    ptLayer = map_lanelet.pointLayer

    if not direction == side and direction:
        # Use previously created linestring
        ls = d_id
    elif pt_left.id == pt_right.id:
        # No longitudinal boundary exists
        # print(ll.id)
        pass
    else:
        # Check for existing lineStrings (e.g. stop_line)
        lines = {'exact': None,
                 'overarching': None,
                 'Insufficient_half': None,
                 'Insufficient_full': None
                 }

        # condition: linestrings shouldn't be part of lanelets as lat. boundary
        lsList_pt_left = find_usage_special(llLayer, lsLayer, point=pt_left)
        lsList_pt_right = find_usage_special(llLayer, lsLayer, point=pt_right)

        mutual_ls = list(set.intersection(set(lsList_pt_left), set(lsList_pt_right)))
        lsList_pt_left = list(set(lsList_pt_left) - set(mutual_ls))
        lsList_pt_right = list(set(lsList_pt_right) - set(mutual_ls))

        ### EXACT OR overarching ###
        for line in mutual_ls:
            # line_id.append(line.id)
            if ((line[0].id == pt_left.id and line[-1].id == pt_right.id) or
                    (line[-1].id == pt_left.id and line[0].id == pt_right.id)):
                lines['exact'] = line.id

            elif line.attributes['type'] == 'stop_line' or 'pedestrian_marking':
                pt_list = [el for el in line]
                idx_l = pt_list.index(pt_left)
                idx_r = pt_list.index(pt_right)
                if idx_l < idx_r:
                    lines['overarching'] = pt_list[idx_l:idx_r+1]
                else:
                    lines['overarching'] = pt_list[idx_r:idx_l+1]
            else:
                pass
                # print(mutual_ls, pt_left, pt_right)

        ### insuFFICIENT
        pt_list_left = find_line_insufficient(lsList_pt_left, pt_left, pt_right)
        pt_list_right = find_line_insufficient(lsList_pt_right, pt_right, pt_left)
        if pt_list_left:
            lines['Insufficient_half'] = pt_list_left
        elif pt_list_right:
            lines['Insufficient_half'] = pt_list_right

        ### Both sides are not matching
        # v = [pt_right.x - pt_left.x, pt_right.y - pt_left.y]
        v_orth = [pt_right.y - pt_left.y, -(pt_right.x - pt_left.x)]
        length_v = math.sqrt(v_orth[0] * v_orth[0] + v_orth[1] * v_orth[1])
        v_orth = [el / length_v for el in v_orth]
        min_pt = BasicPoint2d(min(pt_left.x + v_orth[0], pt_right.x - v_orth[0]),
                              min(pt_left.y + v_orth[1], pt_right.y - v_orth[1]))
        max_pt = BasicPoint2d(max(pt_left.x + v_orth[0], pt_right.x - v_orth[0]),
                              max(pt_left.y + v_orth[1], pt_right.y - v_orth[1]))

        searchBox = BoundingBox2d(min_pt, max_pt)
        near_ls = lsLayer.search(searchBox)
        near_ls = list((set(near_ls) - set(mutual_ls)))
        near_ls = list((set(near_ls) - set(lsList_pt_left)))
        near_ls = list((set(near_ls) - set(lsList_pt_right)))
        near_ls = find_usage_special(llLayer, ls_list=near_ls)
        for line in near_ls:
            if line.attributes['type'] == 'stop_line':
                # Distinguish line inside and outside of lanelet
                near_pts = ptLayer.search(searchBox)
                if line[0] in near_pts and line[-1] in near_pts:
                    print('NEAR', line.id)
                    lines['Insufficient_full'] = [el for el in line]
                    if dist(line[0], pt_left) < dist(line[0], pt_right):
                        lines['Insufficient_full'].insert(0, pt_left)
                        lines['Insufficient_full'].append(pt_right)
                    else:
                        lines['Insufficient_full'].insert(0, pt_right)
                        lines['Insufficient_full'].append(pt_left)

        # In Case multiple linestrings have been found, write an error
        if len([v for k, v in lines.items() if v]) > 1:
            print(f'Multiple possible long. boundaries found for points {pt_left} and {pt_right}')

        if lines['exact']:
            ls = map_lanelet.lineStringLayer[lines['exact']]
        # Create new line, if necessary
        else:
            if lines['overarching']:
                pt_pairs = lines['overarching']
            elif lines['Insufficient_half']:
                pt_pairs = lines['Insufficient_half']
            elif lines['Insufficient_full']:
                pt_pairs = lines['Insufficient_full']
            else:
                pt_pairs = [pt_left, pt_right]
            pt_pairs = [ptLayer[pt.id] for pt in pt_pairs]
            ls = LineString3d(getId(), pt_pairs, {'type': 'BSSD', 'subtype': 'Boundary'})
            map_lanelet.add(ls)

            if lines['overarching']:
                # Todo: Integrate into log
                ref_line = lines['overarching']

    return map_lanelet, ls


def find_line_insufficient(ls_list, point_matching, point_free):
    # Find the points for a new longitudinal boundary in case there is an existing stop line
    # that doesn't contain BOTH of the given endpoints of the lanelets lateral boundaries.
    # Instead, the linestring contains only one of the points and has a loose end. Goal of this
    # function is to find the list of points between the matching point and loose end and add the
    # open end on the other lateral boundary. This list of points can be used to create a new
    # linestring which represents the longitudinal boundary of the behavior space.
    # Todo: Find better function name
    for line in ls_list:
        if line.attributes['type'] == 'stop_line':
            # ls shows to the right
            pt_list = [el for el in line]
            if dist(line[-1], point_free) < dist(line[-1], point_matching):
                pts_for_ls = pt_list[pt_list.index(point_matching):]
                pts_for_ls.append(point_free)
                print('A', pts_for_ls)
                print(line.id)
                return pts_for_ls

            # ls shows to the left
            elif dist(line[0], point_free) < dist(line[0], point_matching):
                pts_for_ls = pt_list[:pt_list.index(point_matching)+1]
                pts_for_ls.insert(0, point_free)
                print('B', pts_for_ls)
                print(line.id)
                return pts_for_ls

    return []


def find_usage_special(ll_layer, ls_layer=None, point=None, ls_list=None):

    if point:
        linestring_list = ls_layer.findUsages(point)
    else:
        linestring_list = ls_list

    linestring_list = [ls for ls in linestring_list
                       if not ll_layer.findUsages(ls)
                       and not ll_layer.findUsages(ls.invert())]

    return linestring_list


if __name__ == '__main__':
    main()

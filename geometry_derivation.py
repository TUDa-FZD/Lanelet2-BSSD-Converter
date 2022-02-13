from lanelet2.geometry import distance as dist
import math
from lanelet2.core import AttributeMap, TrafficLight, Lanelet, LineString3d, Point2d, Point3d, getId, \
    LaneletMap, BoundingBox2d, BasicPoint2d


def get_long_bdr(map_lanelet, pt_left, pt_right, side, direction, d_id):

    ls = None
    ref_line = None

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
        lines = {'exact': [None],
                 'overarching': [None, None],
                 'insufficient_half': [None, None],
                 'insufficient_full': [None, None]
                 }

        # condition: linestrings shouldn't be part of lanelets as lat. boundary
        lsList_pt_left = find_usage_special(llLayer, lsLayer, point=pt_left)
        lsList_pt_right = find_usage_special(llLayer, lsLayer, point=pt_right)

        mutual_ls = list(set.intersection(set(lsList_pt_left), set(lsList_pt_right)))
        lsList_pt_left = list(set(lsList_pt_left) - set(mutual_ls))
        lsList_pt_right = list(set(lsList_pt_right) - set(mutual_ls))

        ### EXACT OR overarching ###
        lines.update(find_flush_bdr(pt_left, pt_right, mutual_ls))

        ### insuFFICIENT
        pt_list_left = find_line_insufficient(lsList_pt_left, pt_left, pt_right)
        pt_list_right = find_line_insufficient(lsList_pt_right, pt_right, pt_left)
        if pt_list_left:
            lines['insufficient_half'] = pt_list_left
        elif pt_list_right:
            lines['insufficient_half'] = pt_list_right

        ### Both sides are not matching
        searchBox = make_orth_bounding_box(pt_left, pt_right)

        near_ls = lsLayer.search(searchBox)
        near_ls = list((set(near_ls) - set(mutual_ls)))
        near_ls = list((set(near_ls) - set(lsList_pt_left)))
        near_ls = list((set(near_ls) - set(lsList_pt_right)))
        near_ls = find_usage_special(llLayer, ls_list=near_ls)

        lines.update(find_inside_lines(ptLayer, searchBox, near_ls, pt_left, pt_right))

        # In case multiple linestrings have been found, write an error
        if len([v for k, v in lines.items() if v[0]]) > 1:
            print(f'Multiple possible long. boundaries found for points {pt_left} and {pt_right}')

        if lines['exact'][0]:
            ls = map_lanelet.lineStringLayer[lines['exact'][0]]
            ref_line = lines['exact'][0]
        # Create new line, if necessary
        else:
            if lines['overarching'][0]:
                pt_pairs = lines['overarching'][1]
                ref_line = lines['overarching'][0]
            elif lines['insufficient_half'][0]:
                pt_pairs = lines['insufficient_half'][1]
                ref_line = lines['insufficient_half'][0]
            elif lines['insufficient_full'][0]:
                pt_pairs = lines['insufficient_full'][1]
                ref_line = lines['insufficient_full'][0]
            else:
                pt_pairs = [pt_left, pt_right]

            pt_pairs = [ptLayer[pt.id] for pt in pt_pairs]
            ls = LineString3d(getId(), pt_pairs, {'type': 'BSSD', 'subtype': 'boundary'})
            map_lanelet.add(ls)

    return map_lanelet, ls, ref_line


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
                return [line.id, pts_for_ls]

            # ls shows to the left
            elif dist(line[0], point_free) < dist(line[0], point_matching):
                pts_for_ls = pt_list[:pt_list.index(point_matching)+1]
                pts_for_ls.insert(0, point_free)
                return [line.id, pts_for_ls]

    return [None, None]


def find_usage_special(ll_layer, ls_layer=None, point=None, ls_list=None):

    if point:
        linestring_list = ls_layer.findUsages(point)
    else:
        linestring_list = ls_list

    linestring_list = [ls for ls in linestring_list
                       if not ll_layer.findUsages(ls)
                       and not ll_layer.findUsages(ls.invert())]

    return linestring_list


def make_orth_bounding_box(pt_left, pt_right):
    # vector v from left point to right point
    # v = [pt_right.x - pt_left.x, pt_right.y - pt_left.y]
    # create orthogonal vector from vector v
    v_orth = [pt_right.y - pt_left.y, -(pt_right.x - pt_left.x)]
    # calculate length of vector v/v_orth
    length_v = math.sqrt(v_orth[0] * v_orth[0] + v_orth[1] * v_orth[1])
    # normalize vector v_orth
    v_orth = [el / length_v for el in v_orth]

    # add orthogonal vector to coordinates of left point and substract from coordinates of the right point
    # from new coordinates: find min and max x and y values and initalize new points
    min_pt = BasicPoint2d(min(pt_left.x + v_orth[0], pt_right.x - v_orth[0]),
                          min(pt_left.y + v_orth[1], pt_right.y - v_orth[1]))
    max_pt = BasicPoint2d(max(pt_left.x + v_orth[0], pt_right.x - v_orth[0]),
                          max(pt_left.y + v_orth[1], pt_right.y - v_orth[1]))

    # from previously created points initalize BoundingBox2d and return it
    return BoundingBox2d(min_pt, max_pt)


def find_flush_bdr(pt_left, pt_right, list_mutual):

    lines_local = {'exact': [None],
                   'overarching': [None, None]}

    for line in list_mutual:
        # line_id.append(line.id)
        if ((line[0].id == pt_left.id and line[-1].id == pt_right.id) or
                (line[-1].id == pt_left.id and line[0].id == pt_right.id)):
            lines_local['exact'] = [line.id]

        elif line.attributes['type'] == 'stop_line' or 'pedestrian_marking' or 'zebra_marking':
            pt_list = [pt for pt in line]
            idx_l = pt_list.index(pt_left)
            idx_r = pt_list.index(pt_right)
            if idx_l < idx_r:
                lines_local['overarching'] = [line.id, pt_list[idx_l:idx_r + 1]]
            else:
                lines_local['overarching'] = [line.id, pt_list[idx_r:idx_l + 1]]
        else:
            pass
            # print(mutual_ls, pt_left, pt_right)

    return lines_local


def find_inside_lines(ptLayer, searchBox, near_ls, pt_left, pt_right):

    ls_id = None
    ls_pts = None

    for line in near_ls:
        if line.attributes['type'] == 'stop_line':
            # Distinguish line inside and outside of lanelet
            near_pts = ptLayer.search(searchBox)
            if line[0] in near_pts and line[-1] in near_pts:
                print('NEAR', line.id)
                ls_pts = [el for el in line]
                ls_id = line.id
                if dist(line[0], pt_left) < dist(line[0], pt_right):
                    ls_pts.insert(0, pt_left)
                    ls_pts.append(pt_right)
                else:
                    ls_pts.insert(0, pt_right)
                    ls_pts.append(pt_left)

    return {'insufficient_full': [ls_id, ls_pts]}

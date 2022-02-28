import constants
import logging
logger = logging.getLogger(__name__)


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


def protected_b_lane(nbrs, bound_att):
    if nbrs and \
            ls_of_pbl(bound_att) and\
            ll_relevant(nbrs[0].attributes):
            #('line' in att['type'] and att['subtype'] == 'dashed') or att['type'] == 'virtual':
        return True
    else:
        return False


def get_relevant_bicycle_lls(ll_layer, list_relevant):

    list_bicycle = [item for item in ll_layer if item.attributes['subtype'] == 'bicycle_lane']

    for ll in list_bicycle:
        nbrs_left = find_usages_and_remove_self(ll_layer, ll, 'l')
        nbrs_right = find_usages_and_remove_self(ll_layer, ll, 'r')

        if len(nbrs_left) > 1 or len(nbrs_right) > 1:
            logger.warning(f"For bicycle_lane with ID {ll.id}: Multiple neighbors have been found.\
            left: {nbrs_left}, right {nbrs_right}")

        # if (nbrs_left and ls_of_pbl(ll.leftBound.attributes) and ll_relevant(nbrs_left[0].attributes)) or \
        #         (nbrs_right and ls_of_pbl(ll.rightBound.attributes) and ll_relevant(nbrs_right[0].attributes)):
        if protected_b_lane(nbrs_left, ll.leftBound.attributes)\
                or protected_b_lane(nbrs_right, ll.rightBound.attributes):
            logger.debug(f' Lanelet {ll.id} identified as protected bicycle lane')
            ll.attributes['subtype_alt'] = 'bicycle_lane_protected'
            ll.attributes['participant:vehicle'] = 'yes'
            ll.attributes['participant:bicycle'] = 'yes'
            list_relevant.append(ll.id)
            list_bicycle.remove(ll)

    return list_relevant, list_bicycle


# def assign_long_ref_line(behavior, ref):
#     if behavior.longBound:
#         behavior.longBound.ref_line = ref


def find_usages_and_remove_self(ll_layer, ll, side):

    nbrs = []
    if side == 'r':
        nbrs = ll_layer.findUsages(ll.rightBound)
    elif side == 'l':
        nbrs = ll_layer.findUsages(ll.leftBound)
    nbrs.remove(ll)

    return nbrs
import numpy as np
# stolen from https://stackoverflow.com/questions/32791911/fast-calculation-of-pareto-front-in-python
def is_pareto_efficient_indexed(costs, return_mask=True):  # <- Fastest for many points
    """
    :param costs: An (n_points, n_costs) array
    :param return_mask: True to return a mask, False to return integer indices of efficient points.
    :return: An array of indices of pareto-efficient points.
            If return_mask is True, this will be an (n_points, ) boolean array
            Otherwise it will be a (n_efficient_points, ) integer array of indices.
    """
    is_efficient = np.arange(costs.shape[0])
    n_points = costs.shape[0]
    next_point_index = 0  # Next index in the is_efficient array to search for

    while next_point_index < len(costs):
        nondominated_point_mask = np.any(costs <= costs[next_point_index], axis=1)
        is_efficient = is_efficient[nondominated_point_mask]  # Remove dominated points
        costs = costs[nondominated_point_mask]
        next_point_index = np.sum(nondominated_point_mask[:next_point_index])+1

    if return_mask:
        is_efficient_mask = np.zeros(n_points, dtype=bool)
        is_efficient_mask[is_efficient] = True
        return is_efficient_mask
    else:
        return is_efficient


def pareto_truncate(designs, outcomes):
    # truncate the pop
    dominant_designs = []
    dominant_outcomes = []
    nondominant_designs = []
    nondominant_outcomes = []
    pareto_mask = is_pareto_efficient_indexed(np.array(outcomes), True)
    for i,dominant in enumerate(pareto_mask):
        if dominant == True:
            dominant_designs.append(designs[i])
            dominant_outcomes.append(outcomes[i])
        else:
            nondominant_designs.append(designs[i])
            nondominant_outcomes.append(designs[i])
    return dominant_designs, dominant_outcomes, nondominant_designs, nondominant_outcomes

def pareto_sort(designs, outcomes):
    # I'm a little unclear whether python passes lists by reference or value
    # just in case, let's copy
    temp_designs = designs[:]
    temp_outcomes = outcomes[:]
    tiered_designs= [] # list of pareto fronts
    tiered_outcomes = [] # parallel outcomes
    while len(temp_designs) > 0:
        dd,do,nd,no = pareto_truncate(temp_designs, temp_outcomes)
        temp_designs = nd
        temp_outcomes = no
        tiered_designs.append(dd)
        tiered_outcomes.append(do)
    return tiered_designs, tiered_outcomes
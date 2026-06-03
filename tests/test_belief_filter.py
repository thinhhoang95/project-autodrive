from rcbranch.belief.crossing_order_filter import BeliefFilterConfig, CrossingOrderBeliefFilter
from rcbranch.belief.features import PairFeatures


def test_belief_update_reacts_to_j_yielding_for_i_first():
    filt = CrossingOrderBeliefFilter(BeliefFilterConfig(w_near=0.0))
    features = PairFeatures(
        t_i_in_cv=1.0,
        t_i_out_cv=2.0,
        t_j_in_cv=3.0,
        t_j_out_cv=4.0,
        b_i_req_stop=1.0,
        b_j_req_stop=1.0,
        a_i_obs=0.5,
        a_j_obs=-2.0,
        gap_if_i_first=0.3,
        gap_if_j_first=-3.7,
    )

    belief = filt.update(1, 2, features)

    assert belief.p_i_first > belief.p_j_first
    assert belief.p_i_first > belief.p_unresolved

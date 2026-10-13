"""Unit tests for local dynamic synchronisation: helpers.compute_sync_groups."""
from whca_controller.helpers import compute_sync_groups


def test_disjoint_paths_get_separate_groups():
    a = [(0, 0), (1, 0), (2, 0)]
    b = [(0, 5), (1, 5), (2, 5)]
    assert compute_sync_groups([a, b]) == [[0], [1]]


def test_shared_cell_at_different_times_joins_groups():
    # b passes through (2, 0) two steps after a has left it: an ordering
    # constraint, so they must share a clock.
    a = [(2, 0), (3, 0), (4, 0)]
    b = [(0, 0), (1, 0), (2, 0)]
    assert compute_sync_groups([a, b]) == [[0, 1]]


def test_chain_is_transitive():
    # A and C never meet, but both meet B, so all three are one group.
    a = [(0, 0), (1, 0)]
    b = [(1, 0), (1, 1), (2, 1)]
    c = [(2, 1), (3, 1)]
    d = [(9, 9), (9, 8)]
    assert compute_sync_groups([a, b, c, d]) == [[0, 1, 2], [3]]


def test_robot_revisiting_its_own_cell_stays_alone():
    a = [(0, 0), (1, 0), (0, 0), (0, 0)]
    assert compute_sync_groups([a]) == [[0]]


def test_parked_robot_joins_robot_that_uses_its_cell():
    parked = [(5, 5)] * 5                      # waits on its goal all window
    passer = [(4, 5), (4, 5), (4, 4), (5, 4)]  # never enters (5, 5)
    stepper = [(5, 6), (5, 6), (5, 6), (5, 5)] # enters (5, 5) at the end
    assert compute_sync_groups([parked, passer]) == [[0], [1]]
    assert compute_sync_groups([parked, stepper]) == [[0, 1]]


def test_groups_partition_every_robot_exactly_once():
    import random
    rng = random.Random(0)
    scheds = [[(rng.randrange(8), rng.randrange(8)) for _ in range(5)] for _ in range(30)]
    groups = compute_sync_groups(scheds)
    flat = sorted(i for g in groups for i in g)
    assert flat == list(range(30))
    # members of different groups never share a cell
    cells = [set(s) for s in scheds]
    for gi, g in enumerate(groups):
        for h in groups[gi + 1:]:
            for i in g:
                for j in h:
                    assert not (cells[i] & cells[j])


def test_empty():
    assert compute_sync_groups([]) == []

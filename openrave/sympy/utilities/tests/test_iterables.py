from sympy import symbols, Integral, Tuple, Dummy, Basic
from sympy.utilities.iterables import (postorder_traversal, preorder_traversal,
    flatten, group, take, subsets, variations, cartes, numbered_symbols,
    dict_merge, prefixes, postfixes, sift, topological_sort, rotate_left,
    rotate_right, multiset_partitions, partitions, binary_partitions,
    generate_bell, generate_involutions, generate_derangements,
    unrestricted_necklace, generate_oriented_forest)

from sympy.core.singleton import S
from sympy.functions.elementary.piecewise import Piecewise, ExprCondPair
from sympy.utilities.pytest import raises

w,x,y,z= symbols('w,x,y,z')

def test_postorder_traversal():
    expr = z+w*(x+y)
    expected1 = [z, w, y, x, x + y, w*(x + y), z + w*(x + y)]
    expected2 = [z, w, x, y, x + y, w*(x + y), z + w*(x + y)]
    expected3 = [w, y, x, x + y, w*(x + y), z, z + w*(x + y)]
    assert list(postorder_traversal(expr)) in [expected1, expected2, expected3]

    expr = Piecewise((x,x<1),(x**2,True))
    assert list(postorder_traversal(expr)) == [
        x, x, 1, x < 1, ExprCondPair(x, x < 1), x, 2, x**2, True,
        ExprCondPair(x**2, True), Piecewise((x, x < 1), (x**2, True))
    ]
    assert list(preorder_traversal(Integral(x**2, (x, 0, 1)))) == [
        Integral(x**2, (x, 0, 1)), x**2, x, 2, Tuple(x, 0, 1), x, 0, 1
    ]
    assert list(preorder_traversal(('abc', ('d', 'ef')))) == [
        ('abc', ('d', 'ef')), 'abc', ('d', 'ef'), 'd', 'ef']



def test_preorder_traversal():
    expr = z+w*(x+y)
    expected1 = [z + w*(x + y), z, w*(x + y), w, x + y, y, x]
    expected2 = [z + w*(x + y), z, w*(x + y), w, x + y, x, y]
    expected3 = [z + w*(x + y), w*(x + y), w, x + y, y, x, z]
    assert list(preorder_traversal(expr)) in [expected1, expected2, expected3]

    expr = Piecewise((x,x<1),(x**2,True))
    assert list(preorder_traversal(expr)) == [
        Piecewise((x, x < 1), (x**2, True)), ExprCondPair(x, x < 1), x, x < 1,
        x, 1, ExprCondPair(x**2, True), x**2, x, 2, True
    ]
    assert list(postorder_traversal(Integral(x**2, (x, 0, 1)))) == [
        x, 2, x**2, x, 0, 1, Tuple(x, 0, 1),
        Integral(x**2, Tuple(x, 0, 1))
    ]
    assert list(postorder_traversal(('abc', ('d', 'ef')))) == [
        'abc', 'd', 'ef', ('d', 'ef'), ('abc', ('d', 'ef'))]

    expr = (x**(y**z)) ** (x**(y**z))
    expected = [(x**(y**z))**(x**(y**z)), x**(y**z), x**(y**z)]
    result = []
    pt = preorder_traversal(expr)
    for i in pt:
        result.append(i)
        if i == x**(y**z):
            pt.skip()
    assert result == expected

def test_flatten():
    assert flatten((1, (1,))) == [1, 1]
    assert flatten((x, (x,))) == [x, x]

    ls = [[(-2, -1), (1, 2)], [(0, 0)]]

    assert flatten(ls, levels=0) == ls
    assert flatten(ls, levels=1) == [(-2, -1), (1, 2), (0, 0)]
    assert flatten(ls, levels=2) == [-2, -1, 1, 2, 0, 0]
    assert flatten(ls, levels=3) == [-2, -1, 1, 2, 0, 0]

    raises(ValueError, "flatten(ls, levels=-1)")

    class MyOp(Basic):
        pass

    assert flatten([MyOp(x, y), z]) == [MyOp(x, y), z]
    assert flatten([MyOp(x, y), z], cls=MyOp) == [x, y, z]

def test_group():
    assert group([]) == []
    assert group([], multiple=False) == []

    assert group([1]) == [[1]]
    assert group([1], multiple=False) == [(1, 1)]

    assert group([1,1]) == [[1,1]]
    assert group([1,1], multiple=False) == [(1, 2)]

    assert group([1,1,1]) == [[1,1,1]]
    assert group([1,1,1], multiple=False) == [(1, 3)]

    assert group([1,2,1]) == [[1],[2],[1]]
    assert group([1,2,1], multiple=False) == [(1, 1), (2, 1), (1, 1)]

    assert group([1,1,2,2,2,1,3,3]) == [[1,1], [2,2,2], [1], [3,3]]
    assert group([1,1,2,2,2,1,3,3], multiple=False) == [(1, 2), (2, 3), (1, 1), (3, 2)]

def test_subsets():
    # combinations
    assert list(subsets([1, 2, 3], 0)) == [[]]
    assert list(subsets([1, 2, 3], 1)) == [[1], [2], [3]]
    assert list(subsets([1, 2, 3], 2)) == [[1, 2], [1,3], [2, 3]]
    assert list(subsets([1, 2, 3], 3)) == [[1, 2, 3]]
    l = range(4)
    assert list(subsets(l, 0, repetition=True)) == [[]]
    assert list(subsets(l, 1, repetition=True)) == [[0], [1], [2], [3]]
    assert list(subsets(l, 2, repetition=True)) == [[0, 0], [0, 1], [0, 2],
                                                    [0, 3], [1, 1], [1, 2],
                                                    [1, 3], [2, 2], [2, 3],
                                                    [3, 3]]
    assert list(subsets(l, 3, repetition=True)) == [[0, 0, 0], [0, 0, 1],
                                                    [0, 0, 2], [0, 0, 3],
                                                    [0, 1, 1], [0, 1, 2],
                                                    [0, 1, 3], [0, 2, 2],
                                                    [0, 2, 3], [0, 3, 3],
                                                    [1, 1, 1], [1, 1, 2],
                                                    [1, 1, 3], [1, 2, 2],
                                                    [1, 2, 3], [1, 3, 3],
                                                    [2, 2, 2], [2, 2, 3],
                                                    [2, 3, 3], [3, 3, 3]]
    assert len(list(subsets(l, 4, repetition=True))) == 35

    assert list(subsets(l[:2], 3, repetition=False)) == []
    assert list(subsets(l[:2], 3, repetition=True)) == [[0, 0, 0],
                                                        [0, 0, 1],
                                                        [0, 1, 1],
                                                        [1, 1, 1]]
    assert list(subsets([1, 2], repetition=True)) == \
           [[], [1], [2], [1, 1], [1, 2], [2, 2]]
    assert list(subsets([1, 2], repetition=False)) == \
           [[], [1], [2], [1, 2]]
    assert list(subsets([1, 2, 3], 2)) == \
           [[1, 2], [1, 3], [2, 3]]
    assert list(subsets([1, 2, 3], 2, repetition=True)) == \
           [[1, 1], [1, 2], [1, 3], [2, 2], [2, 3], [3, 3]]

def test_variations():
    # permutations
    l = range(4)
    assert list(variations(l, 0, repetition=False)) == [[]]
    assert list(variations(l, 1, repetition=False)) == [[0], [1], [2], [3]]
    assert list(variations(l, 2, repetition=False)) == [[0, 1], [0, 2], [0, 3], [1, 0], [1, 2], [1, 3], [2, 0], [2, 1], [2, 3], [3, 0], [3, 1], [3, 2]]
    assert list(variations(l, 3, repetition=False)) == [[0, 1, 2], [0, 1, 3], [0, 2, 1], [0, 2, 3], [0, 3, 1], [0, 3, 2], [1, 0, 2], [1, 0, 3], [1, 2, 0], [1, 2, 3], [1, 3, 0], [1, 3, 2], [2, 0, 1], [2, 0, 3], [2, 1, 0], [2, 1, 3], [2, 3, 0], [2, 3, 1], [3, 0, 1], [3, 0, 2], [3, 1, 0], [3, 1, 2], [3, 2, 0], [3, 2, 1]]
    assert list(variations(l, 0, repetition=True)) == [[]]
    assert list(variations(l, 1, repetition=True)) == [[0], [1], [2], [3]]
    assert list(variations(l, 2, repetition=True)) == [[0, 0], [0, 1], [0, 2],
                                                       [0, 3], [1, 0], [1, 1],
                                                       [1, 2], [1, 3], [2, 0],
                                                       [2, 1], [2, 2], [2, 3],
                                                       [3, 0], [3, 1], [3, 2],
                                                       [3, 3]]
    assert len(list(variations(l, 3, repetition=True))) == 64
    assert len(list(variations(l, 4, repetition=True))) == 256
    assert list(variations(l[:2], 3, repetition=False)) == []
    assert list(variations(l[:2], 3, repetition=True)) == [[0, 0, 0], [0, 0, 1],
                                                           [0, 1, 0], [0, 1, 1],
                                                           [1, 0, 0], [1, 0, 1],
                                                           [1, 1, 0], [1, 1, 1]]

def test_cartes():
    assert list(cartes([1, 2], [3, 4, 5])) == \
           [[1, 3], [1, 4], [1, 5], [2, 3], [2, 4], [2, 5]]
    assert list(cartes()) == [[]]

def test_numbered_symbols():
    s = numbered_symbols(cls=Dummy)
    assert isinstance(s.next(), Dummy)

def test_sift():
    assert sift(range(5), lambda _: _%2) == {1: [1, 3], 0: [0, 2, 4]}
    assert sift(x + y, lambda _: _.has(x)) == {False: [y], True: [x]}
    assert sift(x*y, lambda _: _.has(x)) == {False: [y], True: [x]}
    assert sift(S.One, lambda _: _.has(x)) == {False: [1]}

def test_take():
    X = numbered_symbols()

    assert take(X, 5) == list(symbols('x0:5'))
    assert take(X, 5) == list(symbols('x5:10'))

    assert take([1,2,3,4,5], 5) == [1,2,3,4,5]

def test_dict_merge():
    assert dict_merge({}, {1: x, y: z}) == {1: x, y: z}
    assert dict_merge({1: x, y: z}, {}) == {1: x, y: z}

    assert dict_merge({2: z}, {1: x, y: z}) == {1: x, 2: z, y: z}
    assert dict_merge({1: x, y: z}, {2: z}) == {1: x, 2: z, y: z}

    assert dict_merge({1: y, 2: z}, {1: x, y: z}) == {1: x, 2: z, y: z}
    assert dict_merge({1: x, y: z}, {1: y, 2: z}) == {1: y, 2: z, y: z}

def test_prefixes():
    assert list(prefixes([])) == []
    assert list(prefixes([1])) == [[1]]
    assert list(prefixes([1, 2])) == [[1], [1, 2]]

    assert list(prefixes([1,2,3,4,5])) == \
        [[1], [1, 2], [1, 2, 3], [1, 2, 3, 4], [1, 2, 3, 4, 5]]

def test_postfixes():
    assert list(postfixes([])) == []
    assert list(postfixes([1])) == [[1]]
    assert list(postfixes([1, 2])) == [[2], [1, 2]]

    assert list(postfixes([1,2,3,4,5])) == \
        [[5], [4, 5], [3, 4, 5], [2, 3, 4, 5], [1, 2, 3, 4, 5]]

def test_topological_sort():
    V = [2, 3, 5, 7, 8, 9, 10, 11]
    E = [(7, 11), (7, 8), (5, 11), (3, 8), (3, 10), (11, 2), (11, 9), (11, 10), (8, 9)]

    assert topological_sort((V, E)) == [3, 5, 7, 8, 11, 2, 9, 10]
    assert topological_sort((V, E), key=lambda v: -v) == [7, 5, 11, 3, 10, 8, 9, 2]

    raises(ValueError, "topological_sort((V, E + [(10, 7)]))")

def test_rotate():
    A = [0, 1, 2, 3, 4]

    assert rotate_left(A, 2) == [2, 3, 4, 0, 1]
    assert rotate_right(A, 1) == [4, 0, 1, 2, 3]

def test_multiset_partitions():
    A = [0, 1, 2, 3, 4]

    assert list(multiset_partitions(A, 5)) == [[[0], [1], [2], [3], [4]]]
    assert len(list(multiset_partitions(A, 4))) == 10
    assert len(list(multiset_partitions(A, 3))) == 25


    assert list(multiset_partitions([1,1,1,2,2], 2)) == [[[1, 1, 1, 2], [2]],\
    [[1, 1, 2], [1, 2]], [[1, 1], [1, 2, 2]], [[1], [1, 1, 2, 2]], [[1, 2],\
    [1, 1, 2]], [[1, 1, 2, 2], [1]], [[1, 2, 2], [1, 1]]]

    assert list(multiset_partitions([1,1,2,2], 2)) == [[[1, 1, 2], [2]], \
    [[1, 2], [1, 2]], [[1], [1, 2, 2]], [[1, 1], [2, 2]], [[1, 2, 2], [1]]]

    assert list(multiset_partitions([1,2,3,4], 2)) == [[[1, 2, 3], [4]], [[1, 3], \
    [2, 4]], [[1], [2, 3, 4]], [[1, 2], [3, 4]], [[1, 2, 4], [3]], \
    [[1, 4], [2, 3]], [[1, 3, 4], [2]]]

    assert list(multiset_partitions([1,2,2], 2)) == [[[1, 2], [2]],
                                                     [[1], [2, 2]]]

def test_partitions():
    assert [p.copy() for p in partitions(6, k=2)] == [{2: 3}, \
    {1: 2, 2: 2}, {1: 4, 2: 1}, {1: 6}]

    assert [p.copy() for p in partitions(6, k=3)] == [{3: 2}, \
    {1: 1, 2: 1, 3: 1}, {1: 3, 3: 1}, {2: 3}, {1: 2, 2: 2}, \
    {1: 4, 2: 1}, {1: 6}]

    assert [p.copy() for p in partitions(6, k=2, m=2)] == []

    assert [p.copy() for p in partitions(8, k=4, m=3)] == [{4: 2},\
    {1: 1, 3: 1, 4: 1}, {2: 2, 4: 1}, {2: 1, 3: 2}]

def test_binary_partitions():
    assert [i[:] for i in binary_partitions(10)] == [[8, 2], [8, 1, 1], \
    [4, 4, 2], [4, 4, 1, 1], [4, 2, 2, 2], [4, 2, 2, 1, 1], [4, 2, 1, 1, 1, 1], \
    [4, 1, 1, 1, 1, 1, 1], [2, 2, 2, 2, 2], [2, 2, 2, 2, 1, 1], \
    [2, 2, 2, 1, 1, 1, 1], [2, 2, 1, 1, 1, 1, 1, 1], [2, 1, 1, 1, 1, 1, 1, 1, 1], \
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]

    assert len([j[:] for j in binary_partitions(16)]) == 36

def test_bell_perm():
    assert [len(generate_bell(i)) for i in xrange(1, 7)] == [1, 2, 5, 15, 52, 203]
    assert list(generate_bell(4)) == [(0, 1, 2, 3), (0, 1, 3, 2), (0, 2, 1, 3),
                                     (0, 3, 1, 2), (0, 3, 2, 1), (1, 0, 2, 3),
                                     (1, 0, 3, 2), (2, 0, 1, 3), (2, 1, 0, 3),
                                     (2, 3, 0, 1), (3, 0, 1, 2), (3, 0, 2, 1),
                                     (3, 1, 0, 2), (3, 1, 2, 0), (3, 2, 1, 0)]

def test_involutions():
    assert [len(generate_involutions(n)) for n in range(1, 7)] == [1, 2, 4, 10, 26, 76]
    assert generate_involutions(4) == [(0, 1, 2, 3), (0, 1, 3, 2),
                                       (0, 2, 1, 3), (0, 3, 2, 1),
                                       (1, 0, 2, 3), (2, 1, 0, 3),
                                       (3, 0, 2, 1), (3, 1, 0, 2),
                                       (3, 1, 2, 0), (3, 2, 1, 0)]

def test_derangements():
    assert len(list(generate_derangements([0, 1, 2, 3, 4, 5]))) == 265
    assert list(generate_derangements([0, 1, 2, 3])) == [[1, 0, 3, 2], \
    [1, 2, 3, 0], [1, 3, 0, 2], [2, 0, 3, 1], [2, 3, 0, 1], [2, 3, 1, 0], \
    [3, 0, 1, 2], [3, 2, 0, 1], [3, 2, 1, 0]]
    assert list(generate_derangements([0, 1, 2, 2])) == [[2, 2, 0, 1], \
                                                        [2, 2, 1, 0]]

def test_unrestricted_necklaces():
    assert [i[:] for i in unrestricted_necklace(4, 5)] == [[0, 0, 0, 0], \
    [0, 0, 1, 0], [0, 0, 2, 0], [0, 0, 3, 0], [0, 0, 4, 0], [0, 1, 1, 1], \
    [0, 1, 2, 1], [0, 1, 3, 1], [0, 1, 4, 1], [0, 2, 2, 2], [0, 2, 3, 2], \
    [0, 2, 4, 2], [0, 3, 3, 3], [0, 3, 4, 3], [0, 4, 4, 4]]
    assert [i[:] for i in unrestricted_necklace(6, 3)] == [[0, 0, 0, 0, 0, 0],\
    [0, 0, 0, 1, 0, 0], [0, 0, 0, 2, 0, 0], [0, 0, 1, 0, 1, 0], \
    [0, 0, 1, 1, 0, 1], [0, 0, 1, 2, 0, 1], [0, 0, 2, 0, 2, 0], \
    [0, 0, 2, 1, 0, 2], [0, 0, 2, 2, 0, 2], [0, 1, 1, 1, 1, 1], \
    [0, 1, 1, 2, 1, 1], [0, 1, 2, 1, 2, 1], [0, 1, 2, 2, 1, 2], \
    [0, 2, 2, 2, 2, 2]]
    assert len(list(unrestricted_necklace(20, 2))) == 111

def test_generate_oriented_forest():
    assert list(generate_oriented_forest(5)) == [[0, 1, 2, 3, 4], \
    [0, 1, 2, 3, 3], [0, 1, 2, 3, 2], [0, 1, 2, 3, 1], [0, 1, 2, 3, 0], \
    [0, 1, 2, 2, 2], [0, 1, 2, 2, 1], [0, 1, 2, 2, 0], [0, 1, 2, 1, 2], \
    [0, 1, 2, 1, 1], [0, 1, 2, 1, 0], [0, 1, 2, 0, 1], [0, 1, 2, 0, 0], \
    [0, 1, 1, 1, 1], [0, 1, 1, 1, 0], [0, 1, 1, 0, 1], [0, 1, 1, 0, 0], \
    [0, 1, 0, 1, 0], [0, 1, 0, 0, 0], [0, 0, 0, 0, 0]]
    assert len(list(generate_oriented_forest(10))) == 1842

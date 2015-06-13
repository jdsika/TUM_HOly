from sympy import S
from sympy.core.symbol import Symbol
from sympy.utilities.pytest import raises
from sympy.functions.elementary.miscellaneous import Min, Max
from sympy import I, cos, sin, oo

def test_Min():
    from sympy.abc import x, y, z
    n = Symbol('n', negative=True)
    n_ = Symbol('n_', negative=True)
    nn = Symbol('nn', nonnegative=True)
    nn_ = Symbol('nn_', nonnegative=True)
    p = Symbol('p', positive=True)
    p_ = Symbol('p_', positive=True)
    np = Symbol('np', nonpositive=True)
    np_ = Symbol('np_', nonpositive=True)

    assert Min(5, 4) == 4
    assert Min(-oo, -oo) == -oo
    assert Min(-oo, n) == -oo
    assert Min(n, -oo) == -oo
    assert Min(-oo, np) == -oo
    assert Min(np, -oo) == -oo
    assert Min(-oo, 0) == -oo
    assert Min(0, -oo) == -oo
    assert Min(-oo, nn) == -oo
    assert Min(nn, -oo) == -oo
    assert Min(-oo, p) == -oo
    assert Min(p, -oo) == -oo
    assert Min(-oo, oo) == -oo
    assert Min(oo, -oo) == -oo
    assert Min(n, n) == n
    assert Min(n, np) == Min(n, np)
    assert Min(np, n) == Min(np, n)
    assert Min(n, 0) == n
    assert Min(0, n) == n
    assert Min(n, nn) == n
    assert Min(nn, n) == n
    assert Min(n, p) == n
    assert Min(p, n) == n
    assert Min(n, oo) == n
    assert Min(oo, n) == n
    assert Min(np, np) == np
    assert Min(np, 0) == np
    assert Min(0, np) == np
    assert Min(np, nn) == np
    assert Min(nn, np) == np
    assert Min(np, p) == np
    assert Min(p, np) == np
    assert Min(np, oo) == np
    assert Min(oo, np) == np
    assert Min(0, 0) == 0
    assert Min(0, nn) == 0
    assert Min(nn, 0) == 0
    assert Min(0, p) == 0
    assert Min(p, 0) == 0
    assert Min(0, oo) == 0
    assert Min(oo, 0) == 0
    assert Min(nn, nn) == nn
    assert Min(nn, p) == Min(nn, p)
    assert Min(p, nn) == Min(p, nn)
    assert Min(nn, oo) == nn
    assert Min(oo, nn) == nn
    assert Min(p, p) == p
    assert Min(p, oo) == p
    assert Min(oo, p) == p
    assert Min(oo, oo) == oo

    assert Min(n, n_).func is Min
    assert Min(nn, nn_).func is Min
    assert Min(np, np_).func is Min
    assert Min(p, p_).func is Min

    # lists
    raises(ValueError, 'Min()')
    assert Min(x, y) == Min(y, x)
    assert Min(x, y, z) == Min(z, y, x)
    assert Min(x, Min(y, z)) == Min(z, y, x)
    assert Min(x, Max(y, -oo)) == Min(x, y)
    assert Min(p, oo, n,  p, p, p_) == n
    assert Min(n, oo, -7, p,  p, 2) == Min(n, -7)
    assert Min(2, x, p, n, oo, n_,  p, 2, -2, -2) == Min(-2, x, n, n_)
    assert Min(0, x, 1, y) == Min(0, x, y)
    assert Min(1000, 100, -100, x, p, n) == Min(n, x, -100)
    assert Min(cos(x), sin(x)) == Min(cos(x), sin(x))
    assert Min(cos(x), sin(x)).subs(x, 1) == cos(1)
    assert Min(cos(x), sin(x)).subs(x, S(1)/2) == sin(S(1)/2)
    raises(ValueError, 'Min(cos(x), sin(x)).subs(x, I)')
    raises(ValueError, 'Min(I)')
    raises(ValueError, 'Min(I, x)')
    raises(ValueError, 'Min(S.ComplexInfinity, x)')


def test_Max():
    from sympy.abc import x, y, z
    n = Symbol('n', negative=True)
    n_ = Symbol('n_', negative=True)
    nn = Symbol('nn', nonnegative=True)
    nn_ = Symbol('nn_', nonnegative=True)
    p = Symbol('p', positive=True)
    p_ = Symbol('p_', positive=True)
    np = Symbol('np', nonpositive=True)
    np_ = Symbol('np_', nonpositive=True)

    assert Max(5, 4) == 5

    # lists

    raises(ValueError, 'Max()')
    assert Max(x, y) == Max(y, x)
    assert Max(x, y, z) == Max(z, y, x)
    assert Max(x, Max(y, z)) == Max(z, y, x)
    assert Max(x, Min(y, oo)) == Max(x, y)
    assert Max(n, -oo, n_,  p, 2) == Max(p, 2)
    assert Max(n, -oo, n_,  p) == p
    assert Max(2, x, p, n, -oo, S.NegativeInfinity, n_,  p, 2) == Max(2, x, p)
    assert Max(0, x, 1, y) == Max(1, x, y)
    assert Max(x, x + 1, x - 1) == 1 + x
    assert Max(1000, 100, -100, x, p, n) == Max(p, x, 1000)
    assert Max(cos(x), sin(x)) == Max(sin(x), cos(x))
    assert Max(cos(x), sin(x)).subs(x, 1) == sin(1)
    assert Max(cos(x), sin(x)).subs(x, S(1)/2) == cos(S(1)/2)
    raises(ValueError, 'Max(cos(x), sin(x)).subs(x, I)')
    raises(ValueError, 'Max(I)')
    raises(ValueError, 'Max(I, x)')
    raises(ValueError, 'Max(S.ComplexInfinity, 1)')
    # interesting:
    # Max(n, -oo, n_,  p, 2) == Max(p, 2)
    # True
    # Max(n, -oo, n_,  p, 1000) == Max(p, 1000)
    # False

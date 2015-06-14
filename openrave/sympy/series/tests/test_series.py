from sympy import sin, cos, exp, E, series, oo, S, Derivative, O, Integral, \
                  Function, log, sqrt, Symbol
from sympy.abc import x, y, n, k
from sympy.utilities.pytest import raises

def test_sin():
    e1 = sin(x).series(x, 0)
    e2 = series(sin(x), x, 0)
    assert e1 == e2

def test_cos():
    e1 = cos(x).series(x, 0)
    e2 = series(cos(x), x, 0)
    assert e1 == e2

def test_exp():
    e1 = exp(x).series(x, 0)
    e2 = series(exp(x), x, 0)
    assert e1 == e2

def test_exp2():
    e1 = exp(cos(x)).series(x, 0)
    e2 = series(exp(cos(x)),x,0)
    assert e1 == e2


def test_2124():
    assert series(1, x) == 1
    assert S(0).lseries(x).next() == 0
    assert cos(x).series() == cos(x).series(x)
    raises(ValueError, 'cos(x+y).series()')
    raises(ValueError, 'x.series(dir="")')

    assert (cos(x).series(x, 1).removeO().subs(x, x - 1) -
            cos(x + 1).series(x).removeO().subs(x, x - 1)).expand() == 0
    e = cos(x).series(x, 1, n=None)
    assert [e.next() for i in range(2)] == [cos(1), -((x - 1)*sin(1))]
    e = cos(x).series(x, 1, n=None, dir='-')
    assert [e.next() for i in range(2)] == [cos(1), (1 - x)*sin(1)]
    # the following test is exact so no need for x -> x - 1 replacement
    assert abs(x).series(x, 1, dir='-') == x
    assert exp(x).series(x, 1, dir='-', n=3).removeO().subs(x, x - 1) == \
           E + E*(x - 1) + E*(x - 1)**2/2

    D = Derivative
    assert D(x**2 + x**3*y**2, x, 2, y, 1).series(x).doit() == 12*x*y
    assert D(cos(x), x).lseries().next() == D(1, x)
    assert D(exp(x), x).series(n=3) == D(1, x) + D(x, x) + D(x**2/2, x) + O(x**3)

    assert Integral(x, (x, 1, 3),(y, 1, x)).series(x) == -4 + 4*x

    assert (1 + x + O(x**2)).getn() == 2
    assert (1 + x).getn() == None

    assert ((1/sin(x))**oo).series() == oo
    logx = Symbol('logx')
    assert ((sin(x))**y).nseries(x, n=1, logx = logx) \
           == exp(y*logx) + O(x*exp(y*logx), x)

    raises(NotImplementedError, 'series(Function("f")(x))')

    assert sin(1/x).series(x, oo, n=5) == 1/x - 1/(6*x**3)
    assert abs(x).series(x, oo, n=5, dir='+') == x
    assert abs(x).series(x, -oo, n=5, dir='-') == -x
    assert abs(-x).series(x, oo, n=5, dir='+') == x
    assert abs(-x).series(x, -oo, n=5, dir='-') == -x

    assert exp(x*log(x)).series(n=3) == \
           1 + x*log(x) + x**2*log(x)**2/2 + O(x**3*log(x)**3)
    # XXX is this right? If not, fix "ngot > n" handling in expr.
    p = Symbol('p', positive=True)
    assert exp(sqrt(p)**3*log(p)).series(n=3) == \
        1 + p**S('3/2')*log(p) + O(p**3*log(p)**3)

    assert exp(sin(x)*log(x)).series(n=2) == 1 + x*log(x) + O(x**2*log(x)**2)

from sympy.series.acceleration import richardson, shanks
from sympy import Sum, Integer

def test_acceleration():
    e = (1 + 1/n)**n
    assert round(richardson(e, n, 10, 20).evalf(), 10) == round(E.evalf(), 10)

    A = Sum(Integer(-1)**(k+1) / k, (k, 1, n))
    assert round(shanks(A, n, 25).evalf(), 4) == round(log(2).evalf(), 4)
    assert round(shanks(A, n, 25, 5).evalf(), 10) == round(log(2).evalf(), 10)

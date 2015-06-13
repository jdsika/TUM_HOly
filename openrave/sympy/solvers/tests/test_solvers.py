from sympy import (Matrix, Symbol, solve, exp, log, cos, acos, Rational, Eq,
    sqrt, oo, LambertW, pi, I, sin, asin, Function, diff, Derivative, symbols,
    S, sympify, var, simplify, Integral, sstr, Wild, solve_linear, Interval,
    And, Or, Lt, Gt, Q, re, im, expand, zoo, tan)

from sympy.solvers import solve_linear_system, solve_linear_system_LU,dsolve,\
     tsolve, solve_undetermined_coeffs

from sympy.solvers.solvers import guess_solve_strategy, GS_POLY, GS_POLY_CV_1, GS_POLY_CV_2,\
    GS_TRANSCENDENTAL, GS_RATIONAL, GS_RATIONAL_CV_1

from sympy.utilities.pytest import XFAIL, raises

from sympy.abc import x

def NS(e, n=15, **options):
    return sstr(sympify(e).evalf(n, **options), full_prec=True)

def test_free_symbols():
    x = Symbol('x')
    f = Function('f')
    raises(NotImplementedError, "solve(Eq(log(f(x)), Integral(x, (x, 1, f(x)))), f(x))")

def test_swap_back():
    f, g = map(Function, 'fg')
    x, y = symbols('x,y')
    a, b = f(x), g(x)
    assert solve([a + y - 2, a - b - 5], a, y, b) == \
                 {a: b + 5, y: -b - 3}
    assert solve(a + b*x - 2, [a, b]) == {a: 2, b: 0}
    assert solve(a + b**2*x - y, [a, b]) == {a: y - b**2*x}

def test_guess_poly():
    """
    See solvers.guess_solve_strategy
    """
    x, y, a = symbols('x,y,a')

    # polynomial equations
    assert guess_solve_strategy( S(4), x ) == GS_POLY
    assert guess_solve_strategy( x, x ) == GS_POLY
    assert guess_solve_strategy( x + a, x ) == GS_POLY
    assert guess_solve_strategy( 2*x, x ) == GS_POLY
    assert guess_solve_strategy( x + sqrt(2), x) == GS_POLY
    assert guess_solve_strategy( x + 2**Rational(1,4), x) == GS_POLY
    assert guess_solve_strategy( x**2 + 1, x ) == GS_POLY
    assert guess_solve_strategy( x**2 - 1, x ) == GS_POLY
    assert guess_solve_strategy( x*y + y, x ) == GS_POLY
    assert guess_solve_strategy( x*exp(y) + y, x) == GS_POLY
    assert guess_solve_strategy( (x - y**3)/(y**2*(1 - y**2)**(S(1)/2)), x) == GS_POLY

def test_guess_poly_cv():
    x, y = symbols('x,y')
    # polynomial equations via a change of variable
    assert guess_solve_strategy( x**Rational(1,2) + 1, x ) == GS_POLY_CV_1
    assert guess_solve_strategy( x**Rational(1,3) + x**Rational(1,2) + 1, x ) == GS_POLY_CV_1
    assert guess_solve_strategy( 4*x*(1 - sqrt(x)), x ) == GS_POLY_CV_1

    # polynomial equation multiplying both sides by x**n
    assert guess_solve_strategy( x + 1/x + y, x ) == GS_POLY_CV_2

def test_guess_rational_cv():
    # rational functions
    x, y = symbols('x,y')
    assert guess_solve_strategy( (x+1)/(x**2 + 2), x) == GS_RATIONAL
    assert guess_solve_strategy( (x - y**3)/(y**2*(1 - y**2)**(S(1)/2)), y) == GS_RATIONAL_CV_1

    # rational functions via the change of variable y -> x**n
    assert guess_solve_strategy( (x**Rational(1,2) + 1)/(x**Rational(1,3) + x**Rational(1,2) + 1), x ) \
                                == GS_RATIONAL_CV_1

def test_guess_transcendental():
    x, y, a, b = symbols('x,y,a,b')
    #transcendental functions
    assert guess_solve_strategy( exp(x) + 1, x ) == GS_TRANSCENDENTAL
    assert guess_solve_strategy( 2*cos(x)-y, x ) == GS_TRANSCENDENTAL
    assert guess_solve_strategy( exp(x) + exp(-x) - y, x ) == GS_TRANSCENDENTAL
    assert guess_solve_strategy(3**x-10, x) == GS_TRANSCENDENTAL
    assert guess_solve_strategy(-3**x+10, x) == GS_TRANSCENDENTAL

    assert guess_solve_strategy(a*x**b-y, x) == GS_TRANSCENDENTAL

def test_solve_args():
    a, b, x, y = symbols('a,b,x,y')
    #implicit symbol to solve for
    assert set(int(tmp) for tmp in solve(x**2-4)) == set([2,-2])
    assert solve([x+y-3,x-y-5]) == {x: 4, y: -1}
    #no symbol to solve for
    assert solve(42) == []
    assert solve([1,2]) is None
    #multiple symbols: take the first linear solution
    assert solve(x + y - 3, [x, y]) == {x: 3 - y}
    # unless it is an undetermined coefficients system
    assert solve(a + b*x - 2, [a, b]) == {a: 2, b: 0}
    #symbol is not a symbol or function
    raises(TypeError, "solve(x**2-pi, pi)")

def test_solve_polynomial1():
    x, y, a = symbols('x,y,a')

    assert solve(3*x-2, x) == [Rational(2,3)]
    assert solve(Eq(3*x, 2), x) == [Rational(2,3)]

    assert solve(x**2-1, x) in [[-1, 1], [1, -1]]
    assert solve(Eq(x**2, 1), x) in [[-1, 1], [1, -1]]

    assert solve( x - y**3, x) == [y**3]
    assert sorted(solve( x - y**3, y)) == sorted([
        (-x**Rational(1,3))/2 + I*sqrt(3)*x**Rational(1,3)/2,
        x**Rational(1,3),
        (-x**Rational(1,3))/2 - I*sqrt(3)*x**Rational(1,3)/2,
    ])

    a11,a12,a21,a22,b1,b2 = symbols('a11,a12,a21,a22,b1,b2')

    assert solve([a11*x + a12*y - b1, a21*x + a22*y - b2], x, y) == \
        {
        x : (a22*b1 - a12*b2)/(a11*a22 - a12*a21),
        y : (a11*b2 - a21*b1)/(a11*a22 - a12*a21),
        }

    solution = {y: S.Zero, x: S.Zero}

    assert solve((x-y, x+y),  x, y ) == solution
    assert solve((x-y, x+y), (x, y)) == solution
    assert solve((x-y, x+y), [x, y]) == solution

    assert solve( x**3 - 15*x - 4, x) == [-2 + 3**Rational(1,2),
                                           4,
                                           -2 - 3**Rational(1,2) ]

    assert sorted(solve((x**2 - 1)**2 - a, x)) == \
           sorted([(1 + a**S.Half)**S.Half, -(1 + a**S.Half)**S.Half,
                   (1 - a**S.Half)**S.Half, -(1 - a**S.Half)**S.Half])

def test_solve_polynomial2():
    x = Symbol('x')
    assert solve(4, x) == []

def test_solve_polynomial_cv_1a():
    """
    Test for solving on equations that can be converted to a polynomial equation
    using the change of variable y -> x**Rational(p, q)
    """

    x = Symbol('x')
    assert solve( x**Rational(1,2) - 1, x) == [1]
    assert solve( x**Rational(1,2) - 2, x) == [4]
    assert solve( x**Rational(1,4) - 2, x) == [16]
    assert solve( x**Rational(1,3) - 3, x) == [27]
    # XXX there are imaginary roots that are being missed
    assert solve(x**Rational(1,2)+x**Rational(1,3)+x**Rational(1,4),x) == [0]

def test_solve_polynomial_cv_1b():
    x, a = symbols('x a')
    assert set(solve(4*x*(1 - a*x**(S(1)/2)), x)) == set([S(0), 1/a**2])
    assert set(solve(x * (x**(S(1)/3) - 3), x)) == set([S(0), S(27)])

def test_solve_polynomial_cv_2():
    """
    Test for solving on equations that can be converted to a polynomial equation
    multiplying both sides of the equation by x**m
    """
    x = Symbol('x')

    assert solve(x + 1/x - 1, x) in \
        [[ Rational(1,2) + I*sqrt(3)/2, Rational(1,2) - I*sqrt(3)/2],
         [ Rational(1,2) - I*sqrt(3)/2, Rational(1,2) + I*sqrt(3)/2]]

def test_solve_rational():
    """Test solve for rational functions"""
    x, y, a, b = symbols('x,y,a,b')

    assert solve( ( x - y**3 )/( (y**2)*sqrt(1 - y**2) ), x) == [y**3]

def test_linear_system():
    x, y, z, t, n, a = symbols('x,y,z,t,n,a')

    assert solve([x-1, x-y, x-2*y, y-1], [x,y]) is None

    assert solve([x-1, x-y, x-2*y, x-1], [x,y]) is None
    assert solve([x-1, x-1, x-y, x-2*y], [x,y]) is None

    assert solve([x+5*y-2, -3*x+6*y-15], x, y) == {x: -3, y: 1}

    M = Matrix([[0,0,n*(n+1),(n+1)**2,0],
                [n+1,n+1,-2*n-1,-(n+1),0],
                [-1, 0, 1, 0, 0]])

    assert solve_linear_system(M, x, y, z, t) == \
           {y: 0, z: t*(-n - 1)/n, x: t*(-n - 1)/n}

    assert solve([x + y + z + t, -z-t], x, y, z, t) == {x: -y, z: -t}

    assert solve([a(0, 0) + a(0, 1) + a(1, 0) + a(1, 1), -a(1, 0) - a(1, 1)],
        a(0, 0), a(0, 1), a(1, 0), a(1, 1)) == {a(1, 0): -a(1, 1), a(0, 0): -a(0, 1)}

def test_linear_systemLU():
    x, y, z, n = symbols('x,y,z,n')

    M = Matrix([[1,2,0,1],[1,3,2*n,1],[4,-1,n**2,1]])

    assert solve_linear_system_LU(M, [x,y,z]) == {z: -3/(n**2+18*n),
                                                  x: 1-12*n/(n**2+18*n),
                                                  y: 6*n/(n**2+18*n)}

# Note: multiple solutions exist for some of these equations, so the tests
# should be expected to break if the implementation of the solver changes
# in such a way that a different branch is chosen
def test_tsolve():
    a, b = symbols('a,b')
    x, y, z = symbols('x,y,z')
    assert solve(exp(x)-3, x) == [log(3)]
    assert solve((a*x+b)*(exp(x)-3), x) == [-b/a, log(3)]
    assert solve(cos(x)-y, x) == [acos(y)]
    assert solve(2*cos(x)-y,x)== [acos(y/2)]
    raises(NotImplementedError, "solve(Eq(cos(x), sin(x)), x)")

    assert solve(exp(x) + exp(-x) - y, x, simplified=False) == [
        log(y**2/2 + y*sqrt(y**2 - 4)/2 - 1)/2,
        log(y**2/2 - y*sqrt(y**2 - 4)/2 - 1)/2]
    assert solve(exp(x)-3, x) == [log(3)]
    assert solve(Eq(exp(x), 3), x) == [log(3)]
    assert solve(log(x)-3, x) == [exp(3)]
    assert solve(sqrt(3*x)-4, x) == [Rational(16,3)]
    assert solve(3**(x+2), x) == [zoo]
    assert solve(3**(2-x), x) == [zoo]
    assert solve(4*3**(5*x+2)-7, x) == [(-2*log(3) - 2*log(2) + log(7))/(5*log(3))]
    assert solve(x+2**x, x) == [-LambertW(log(2))/log(2)]
    assert solve(3*x+5+2**(-5*x+3), x) in [
        [Rational(-5, 3) + LambertW(log(2**(-10240*2**(Rational(1, 3))/3)))/(5*log(2))],
        [-Rational(5,3) + LambertW(-10240*2**Rational(1,3)*log(2)/3)/(5*log(2))],
        [(-25*log(2) + 3*LambertW(-10240*2**(Rational(1, 3))*log(2)/3))/(15*log(2))],
        [-((25*log(2) - 3*LambertW(-10240*2**(Rational(1, 3))*log(2)/3)))/(15*log(2))],
        [-(25*log(2) - 3*LambertW(log(2**(-10240*2**Rational(1, 3)/3))))/(15*log(2))],
        [(25*log(2) - 3*LambertW(log(2**(-10240*2**Rational(1, 3)/3))))/(-15*log(2))]
        ]
    assert solve(5*x-1+3*exp(2-7*x), x) == \
        [Rational(1,5) + LambertW(-21*exp(Rational(3,5))/5)/7]
    assert solve(2*x+5+log(3*x-2), x) == \
        [Rational(2,3) + LambertW(2*exp(-Rational(19,3))/3)/2]
    assert solve(3*x+log(4*x), x) == [LambertW(Rational(3,4))/3]
    assert solve((2*x+8)*(8+exp(x)), x) == [-4, log(8) + pi*I]
    assert solve(2*exp(3*x+4)-3, x) in [
        [Rational(-4, 3) + log(2**(Rational(2, 3))*3**(Rational(1, 3))/2)],
        [-Rational(4,3)+log(Rational(3,2))/3],
        [Rational(-4, 3) - log(2)/3 + log(3)/3],
        ]
    assert solve(2*log(3*x+4)-3, x) == [(exp(Rational(3,2))-4)/3]
    assert solve(exp(x)+1, x) == [pi*I]
    assert solve(x**2 - 2**x, x) == [2]
    assert solve(x**3 - 3**x, x) == [-3*LambertW(-log(3)/3)/log(3)]

    A = -7*2**Rational(4, 5)*6**Rational(1, 5)*log(7)/10
    B = -7*3**Rational(1, 5)*log(7)/5

    result = solve(2*(3*x+4)**5 - 6*7**(3*x+9), x)

    assert len(result) == 1 and expand(result[0]) in [
        Rational(-4, 3) - 5/log(7)/3*LambertW(A),
        Rational(-4, 3) - 5/log(7)/3*LambertW(B),
    ]

    assert solve(z*cos(x)-y, x)      == [acos(y/z)]
    assert solve(z*cos(2*x)-y, x)    == [acos(y/z)/2]
    assert solve(z*cos(sin(x))-y, x) == [asin(acos(y/z))]

    assert solve(z*cos(x), x)        == [acos(0)]

    # issue #1409
    assert solve(y - b*x/(a+x), x) in [[-a*y/(y - b)], [a*y/(b - y)]]
    assert solve(y - b*exp(a/x), x) == [a/log(y/b)]
    # issue #1408
    assert solve(y-b/(1+a*x), x) in [[(b - y)/(a*y)], [-((y - b)/(a*y))]]
    # issue #1407
    assert solve(y-a*x**b , x) == [(y/a)**(1/b)]
    # issue #1406
    assert solve(z**x - y, x) == [log(y)/log(z)]
    # issue #1405
    assert solve(2**x - 10, x) == [log(10)/log(2)]

def test_solve_for_functions_derivatives():
    t = Symbol('t')
    x = Function('x')(t)
    y = Function('y')(t)
    a11,a12,a21,a22,b1,b2 = symbols('a11,a12,a21,a22,b1,b2')

    soln = solve([a11*x + a12*y - b1, a21*x + a22*y - b2], x, y)
    assert soln == {
        x : (a22*b1 - a12*b2)/(a11*a22 - a12*a21),
        y : (a11*b2 - a21*b1)/(a11*a22 - a12*a21),
        }

    assert solve(x - 1, x) == [1]
    assert solve(3*x - 2, x) == [Rational(2, 3)]

    soln = solve([a11*x.diff(t) + a12*y.diff(t) - b1, a21*x.diff(t) +
            a22*y.diff(t) - b2], x.diff(t), y.diff(t))
    assert soln == { y.diff(t) : (a11*b2 - a21*b1)/(a11*a22 - a12*a21),
            x.diff(t) : (a22*b1 - a12*b2)/(a11*a22 - a12*a21) }

    assert solve(x.diff(t)-1, x.diff(t)) == [1]
    assert solve(3*x.diff(t)-2, x.diff(t)) == [Rational(2,3)]

    eqns = set((3*x - 1, 2*y-4))
    assert solve(eqns, set((x,y))) == { x : Rational(1, 3), y: 2 }
    x = Symbol('x')
    f = Function('f')
    F = x**2 + f(x)**2 - 4*x - 1
    assert solve(F.diff(x), diff(f(x), x)) == [-((x - 2)/f(x))]

    # Mixed cased with a Symbol and a Function
    x = Symbol('x')
    y = Function('y')(t)

    soln = solve([a11*x + a12*y.diff(t) - b1, a21*x +
            a22*y.diff(t) - b2], x, y.diff(t))
    assert soln == { y.diff(t) : (a11*b2 - a21*b1)/(a11*a22 - a12*a21),
            x : (a22*b1 - a12*b2)/(a11*a22 - a12*a21) }

def test_issue626():
    x = Symbol("x")
    f = Function("f")
    F = x**2 + f(x)**2 - 4*x - 1
    e = F.diff(x)
    assert solve(e, f(x).diff(x)) in [[(2 - x)/f(x)], [-((x - 2)/f(x))]]

def test_solve_linear():
    x, y = symbols('x y')
    w = Wild('w')
    assert solve_linear(x, x) == (0, 1)
    assert solve_linear(x, y - 2*x) in [(x, y/3), (y, 3*x)]
    assert solve_linear(x, y - 2*x, exclude=[x]) ==(y, 3*x)
    assert solve_linear(3*x - y, 0) in [(x, y/3), (y, 3*x)]
    assert solve_linear(3*x - y, 0, [x]) == (x, y/3)
    assert solve_linear(3*x - y, 0, [y]) == (y, 3*x)
    assert solve_linear(x**2/y, 1) == (y, x**2)
    assert solve_linear(w, x) in [(w, x), (x, w)]
    assert solve_linear(cos(x)**2 + sin(x)**2 + 2 + y) == \
           (y, -2 - cos(x)**2 - sin(x)**2)
    assert solve_linear(cos(x)**2 + sin(x)**2 + 2 + y, x=[x]) == (0, 1)

def test_solve_undetermined_coeffs():
    a, b, c, x = symbols('a, b, c, x')

    assert solve_undetermined_coeffs(a*x**2 + b*x**2 + b*x  + 2*c*x + c + 1, [a, b, c], x) == \
        {a: -2, b: 2, c: -1}
    # Test that rational functions work
    assert solve_undetermined_coeffs(a/x  + b/(x + 1) - (2*x + 1)/(x**2 + x), [a, b], x) == \
        {a: 1, b: 1}
    # Test cancellation in rational functions
    assert solve_undetermined_coeffs(((c + 1)*a*x**2 + (c + 1)*b*x**2 +
    (c + 1)*b*x  + (c + 1)*2*c*x + (c + 1)**2)/(c + 1), [a, b, c], x) == \
        {a: -2, b: 2, c: -1}

def test_solve_inequalities():
    system = [Lt(x**2 - 2, 0), Gt(x**2 - 1, 0)]

    assert solve(system) == \
        And(Or(And(Lt(-sqrt(2), re(x)), Lt(re(x), -1)),
               And(Lt(1, re(x)), Lt(re(x), sqrt(2)))), Eq(im(x), 0))
    assert solve(system, assume=Q.real(x)) == \
        Or(And(Lt(-sqrt(2), x), Lt(x, -1)), And(Lt(1, x), Lt(x, sqrt(2))))

def test_issue_1694():
    x, y = symbols('x,y')
    assert solve(1/x) == []
    assert solve(x*(1 - 5/x)) == [5]
    assert solve(x + sqrt(x) - 2) == [1]
    assert solve(-(1 + x)/(2 + x)**2 + 1/(2 + x)) == []
    assert solve(-x**2 - 2*x + (x + 1)**2 - 1) == []
    assert solve((x/(x + 1) + 3)**(-2)) == []
    assert solve(x/sqrt(x**2 + 1),x) == [0]
    assert solve(exp(x) - y,x) == [log(y)]
    assert solve(exp(x)) == [zoo]
    assert solve(x**2 + x + sin(y)**2 + cos(y)**2 - 1, x) in [[0, -1], [-1, 0]]
    assert solve(4*3**(5*x + 2) - 7, x) == [(-2*log(3) - log(4) + log(7))/(5*log(3))]
    assert solve(x**2 - y**2/exp(x), x, y) == [x*exp(x/2), -x*exp(x/2)]
    # 2072
    assert solve(sqrt(x)) == solve(sqrt(x**3)) == [0]
    assert solve(sqrt(x - 1)) == [1]
    # 1363
    a = symbols('a')
    assert solve(-3*a/sqrt(x),x) == []
    # 1387
    assert solve(2*x/(x + 2) - 1,x) == [2]
    # 1397
    assert solve((x**2/(7 - x)).diff(x)) == [14, 0]
    # 1596
    f = Function('f')
    assert solve((3 - 5*x/f(x))*f(x), f(x)) == [5*x/3]
    # 1398
    #assert solve(1/(5 + x)**(S(1)/5) - 9, x) == [-295244/S(59049)]

from sympy import jn, yn, symbols, sin, cos, pi, S, jn_zeros, besselj, \
                  bessely, besseli, besselk, hankel1, hankel2, expand_func, \
                  latex, sqrt
from sympy.functions.special.bessel import fn
from sympy.utilities.pytest import raises, skip
from sympy.utilities.randtest import \
        random_complex_number as randcplx, \
        test_numerically as tn, \
        test_derivative_numerically as td
from sympy.abc import z, n, k

def test_bessel_rand():
    assert td(besselj(randcplx(), z), z)
    assert td(bessely(randcplx(), z), z)
    assert td(besseli(randcplx(), z), z)
    assert td(besselk(randcplx(), z), z)
    assert td(hankel1(randcplx(), z), z)
    assert td(hankel2(randcplx(), z), z)
    assert td(jn(randcplx(), z), z)
    assert td(yn(randcplx(), z), z)

def test_diff():
    assert besselj(n, z).diff(z) == besselj(n-1, z)/2 - besselj(n+1, z)/2
    assert bessely(n, z).diff(z) == bessely(n-1, z)/2 - bessely(n+1, z)/2
    assert besseli(n, z).diff(z) == besseli(n-1, z)/2 + besseli(n+1, z)/2
    assert besselk(n, z).diff(z) == -besselk(n-1, z)/2 - besselk(n+1, z)/2
    assert hankel1(n, z).diff(z) == hankel1(n-1, z)/2 - hankel1(n+1, z)/2
    assert hankel2(n, z).diff(z) == hankel2(n-1, z)/2 - hankel2(n+1, z)/2

def test_rewrite():
    assert besselj(n, z).rewrite(jn) == sqrt(2*z/pi)*jn(n - S(1)/2, z)
    assert bessely(n, z).rewrite(yn) == sqrt(2*z/pi)*yn(n - S(1)/2, z)

def test_fn():
    x, z = symbols("x z")
    assert fn(1, z) == 1/z**2
    assert fn(2, z) == -1/z + 3/z**3
    assert fn(3, z) == -6/z**2 + 15/z**4
    assert fn(4, z) == 1/z - 45/z**3 + 105/z**5

def mjn(n, z): return expand_func(jn(n,z))
def myn(n, z): return expand_func(yn(n,z))

def test_jn():
    z = symbols("z")
    assert mjn(0, z) == sin(z)/z
    assert mjn(1, z) == sin(z)/z**2 - cos(z)/z
    assert mjn(2, z) == (3/z**3-1/z)*sin(z) - (3/z**2) * cos(z)
    assert mjn(3, z) == (15/z**4 - 6/z**2)*sin(z) + (1/z - 15/z**3)*cos(z)
    assert mjn(4, z) == (1/z + 105/z**5 - 45/z**3)*sin(z) + \
                (-105/z**4 + 10/z**2)*cos(z)
    assert mjn(5, z) == (945/z**6 - 420/z**4 + 15/z**2)*sin(z) + \
                (-1/z - 945/z**5 + 105/z**3)*cos(z)
    assert mjn(6, z) == (-1/z + 10395/z**7 - 4725/z**5 + 210/z**3)*sin(z) + \
                (-10395/z**6 + 1260/z**4 - 21/z**2)*cos(z)

def test_yn():
    z = symbols("z")
    assert myn(0, z) == -cos(z)/z
    assert myn(1, z) == -cos(z)/z**2-sin(z)/z
    assert myn(2, z) == -((3/z**3-1/z)*cos(z)+(3/z**2)*sin(z))

def test_sympify_yn():
    assert S(15) in myn(3, pi).atoms()
    assert myn(3, pi) == 15/pi**4 - 6/pi**2

def eq(a, b, tol=1e-6):
    for x, y in zip(a, b):
        if not (abs(x-y) < tol):
            return False
    return True

def test_jn_zeros():
    assert eq(jn_zeros(0, 4), [3.141592, 6.283185, 9.424777, 12.566370])
    assert eq(jn_zeros(1, 4), [4.493409, 7.725251, 10.904121, 14.066193])
    assert eq(jn_zeros(2, 4), [5.763459, 9.095011, 12.322940, 15.514603])
    assert eq(jn_zeros(3, 4), [6.987932, 10.417118, 13.698023, 16.923621])
    assert eq(jn_zeros(4, 4), [8.182561, 11.704907, 15.039664, 18.301255])

def test_jn_zeros_mpmath():
    try:
        from mpmath import besseljzero
    except ImportError:
        skip("Cannot import mpmath.besseljzero.")
    zeros = lambda n, k: jn_zeros(n, k, method='mpmath')
    assert eq(zeros(0, 4), [3.141592, 6.283185, 9.424777, 12.566370])
    assert eq(zeros(1, 4), [4.493409, 7.725251, 10.904121, 14.066193])
    assert eq(zeros(2, 4), [5.763459, 9.095011, 12.322940, 15.514603])
    assert eq(zeros(3, 4), [6.987932, 10.417118, 13.698023, 16.923621])
    assert eq(zeros(4, 4), [8.182561, 11.704907, 15.039664, 18.301255])


# -*- coding: utf-8 -*-
from sympy import (Basic, Matrix, Piecewise, Ne, symbols, sqrt, Function,
    Rational, conjugate, Derivative, tan, Function, log, floor, Symbol,
    pprint, sqrt, factorial, binomial, pi, sin, ceiling, pprint_use_unicode,
    I, S, Limit, oo, cos, Pow, Integral, exp, Eq, Lt, Gt, Ge, Le, gamma, Abs,
    RootOf, RootSum, Lambda, Not, And, Or, Xor, Nand, Nor, Implies, Equivalent,
    Sum, Subs, FF, ZZ, QQ, RR, O, uppergamma, lowergamma, hyper, meijerg)

from sympy.printing.pretty import pretty as xpretty
from sympy.printing.pretty import pprint

from sympy.physics.units import joule

from sympy.utilities.pytest import raises

a, b, x, y, z, k = symbols('a,b,x,y,z,k')
th = Symbol('theta')
ph = Symbol('phi')

"""
Expressions whose pretty-printing is tested here:
(A '#' to the right of an expression indicates that its various acceptable
orderings are accounted for by the tests.)


BASIC EXPRESSIONS:

oo
(x**2)
1/x
y*x**-2
x**Rational(-5,2)
(-2)**x
Pow(3, 1, evaluate=False)
(x**2 + x + 1)  #
1-x  #
1-2*x  #
x/y
-x/y
(x+2)/y  #
(1+x)*y  #3
-5*x/(x+10)  # correct placement of negative sign
1 - Rational(3,2)*(x+1)
-(-x + 5)*(-x - 2*2**(1/S(2)) + 5) - (-y + 5)*(-y + 5) # Issue 2425


ORDERING:

x**2 + x + 1
1 - x
1 - 2*x
2*x**4 + y**2 - x**2 + y**3


RELATIONAL:

Eq(x, y)
Lt(x, y)
Gt(x, y)
Le(x, y)
Ge(x, y)
Ne(x/(y+1), y**2)  #


RATIONAL NUMBERS:

y*x**-2
y**Rational(3,2) * x**Rational(-5,2)
sin(x)**3/tan(x)**2


FUNCTIONS (ABS, CONJ, EXP, FUNCTION BRACES, FACTORIAL, FLOOR, CEILING):

(2*x + exp(x))  #
Abs(x)
Abs(x/(x**2+1)) #
Abs(1 / (y - Abs(x)))
factorial(n)
factorial(2*n)
factorial(factorial(factorial(n)))
factorial(n+1) #
conjugate(x)
conjugate(f(x+1)) #
f(x)
f(x, y)
f(x/(y+1), y) #
sin(x)**2
conjugate(a+b*I)
conjugate(exp(a+b*I))
conjugate( f(1 + conjugate(f(x))) ) #
f(x/(y+1), y)  # denom of first arg
floor(1 / (y - floor(x)))
ceiling(1 / (y - ceiling(x)))


SQRT:

sqrt(2)
2**Rational(1,3)
2**Rational(1,1000)
sqrt(x**2 + 1)
(1 + sqrt(5))**Rational(1,3)
2**(1/x)
sqrt(2+pi)
(2+(1+x**2)/(2+x))**Rational(1,4)+(1+x**Rational(1,1000))/sqrt(3+x**2)


DERIVATIVES:

Derivative(log(x), x, evaluate=False)
Derivative(log(x), x, evaluate=False) + x  #
Derivative(log(x) + x**2, x, y, evaluate=False)
Derivative(2*x*y, y, x, evaluate=False) + x**2  #
beta(alpha).diff(alpha)


INTEGRALS:

Integral(log(x), x)
Integral(x**2, x)
Integral((sin(x))**2 / (tan(x))**2)
Integral(x**(2**x), x)
Integral(x**2, (x,1,2))
Integral(x**2, (x,Rational(1,2),10))
Integral(x**2*y**2, x,y)
Integral(x**2, (x, None, 1))
Integral(x**2, (x, 1, None))
Integral(sin(th)/cos(ph), (th,0,pi), (ph, 0, 2*pi))


MATRICES:

Matrix([[x**2+1, 1], [y, x+y]])  #
Matrix([[x/y, y, th], [0, exp(I*k*ph), 1]])


PIECEWISE:

Piecewise((x,x<1),(x**2,True))


SEQUENCES (TUPLES, LISTS, DICTIONARIES):

()
[]
{}
(1/x,)
[x**2, 1/x, x, y, sin(th)**2/cos(ph)**2]
(x**2, 1/x, x, y, sin(th)**2/cos(ph)**2)
{x: sin(x)}
{1/x: 1/y, x: sin(x)**2}  #
[x**2]
(x**2,)
{x**2: 1}


LIMITS:

Limit(x, x, oo)
Limit(x**2, x, 0)
Limit(1/x, x, 0)
Limit(sin(x)/x, x, 0)


UNITS:

joule => kg*m**2/s


SUBS:

Subs(f(x), x, ph**2)
Subs(f(x).diff(x), x, 0)
Subs(f(x).diff(x)/y, (x, y), (0, Rational(1, 2)))

"""

def pretty(expr, order=None):
    """ASCII pretty-printing"""
    return xpretty(expr, order=order, use_unicode=False)


def upretty(expr, order=None):
    """Unicode pretty-printing"""
    return xpretty(expr, order=order, use_unicode=True)


def test_pretty_ascii_str():
    """Different acceptable outputs are listed here because of a disagreement
    between Python 2.4 and 2.6 on the output of repr('xxx').
    """
    assert pretty( 'xxx' ) in ["'xxx'", 'xxx']
    assert pretty( "xxx" ) in ["'xxx'", 'xxx']
    assert pretty( 'xxx\'xxx' ) in ['"xxx\'xxx"', 'xxx\'xxx']
    assert pretty( 'xxx"xxx' ) in ["'xxx\"xxx'", 'xxx\"xxx']
    assert pretty( 'xxx\"xxx' ) in ["'xxx\"xxx'", 'xxx\"xxx']
    assert pretty( "xxx'xxx" ) in ['"xxx\'xxx"', 'xxx\'xxx']
    assert pretty( "xxx\'xxx" ) in ['"xxx\'xxx"', 'xxx\'xxx']
    assert pretty( "xxx\"xxx" ) in ["'xxx\"xxx'", 'xxx\"xxx']
    assert pretty( "xxx\"xxx\'xxx" ) in ['\'xxx"xxx\\\'xxx\'', 'xxx"xxx\'xxx']
    assert pretty( "xxx\nxxx" ) in ["'xxx\nxxx'", 'xxx\nxxx']


def test_pretty_unicode_str():
    """Different acceptable outputs are listed here because of a disagreement
    between Python 2.4 and 2.6 on the output of repr(u'xxx').
    """
    assert pretty( u'xxx' ) in [u"'xxx'", u'xxx']
    assert pretty( u"xxx" ) in [u"'xxx'", u'xxx']
    assert pretty( u'xxx\'xxx' ) in [u'"xxx\'xxx"', u'xxx\'xxx']
    assert pretty( u'xxx"xxx' ) in [u"'xxx\"xxx'", u'xxx\"xxx']
    assert pretty( u'xxx\"xxx' ) in [u"'xxx\"xxx'", u'xxx\"xxx']
    assert pretty( u"xxx'xxx" ) in [u'"xxx\'xxx"', u'xxx\'xxx']
    assert pretty( u"xxx\'xxx" ) in [u'"xxx\'xxx"', u'xxx\'xxx']
    assert pretty( u"xxx\"xxx" ) in [u"'xxx\"xxx'", u'xxx\"xxx']
    assert pretty( u"xxx\"xxx\'xxx" ) in [u'\'xxx"xxx\\\'xxx\'', u'xxx"xxx\'xxx']
    assert pretty( u"xxx\nxxx" ) in [u"'xxx\nxxx'", u'xxx\nxxx']


def test_upretty_greek():
    assert upretty( oo ) == u'∞'
    assert upretty( Symbol('alpha^+_1') )   ==  u'α⁺₁'
    assert upretty( Symbol('beta') )    == u'β'
    assert upretty(Symbol('lambda')) == u'λ'


def test_upretty_multiindex():
    assert upretty( Symbol('beta12') )  == u'β₁₂'
    assert upretty( Symbol('Y00') )     == u'Y₀₀'
    assert upretty( Symbol('Y_00') )    == u'Y₀₀'
    assert upretty( Symbol('F^+-') )    == u'F⁺⁻'


def test_upretty_sub_super():
    assert upretty( Symbol('beta_1_2') )  == u'β₁ ₂'
    assert upretty( Symbol('beta^1^2') )  == u'β¹ ²'
    assert upretty( Symbol('beta_1^2') )  == u'β²₁'
    assert upretty( Symbol('beta_10_20') )  == u'β₁₀ ₂₀'
    assert upretty( Symbol('beta_ax_gamma^i') )  == u'βⁱₐₓ ᵧ'
    assert upretty( Symbol("F^1^2_3_4") )  == u'F¹ ²₃ ₄'
    assert upretty( Symbol("F_1_2^3^4") )  == u'F³ ⁴₁ ₂'
    assert upretty( Symbol("F_1_2_3_4") )  == u'F₁ ₂ ₃ ₄'
    assert upretty( Symbol("F^1^2^3^4") )  == u'F¹ ² ³ ⁴'


def test_upretty_subs_missingin_24():
    assert upretty( Symbol('F_beta') )  == u'Fᵦ'
    assert upretty( Symbol('F_gamma') ) == u'Fᵧ'
    assert upretty( Symbol('F_rho') )   == u'Fᵨ'
    assert upretty( Symbol('F_phi') )   == u'Fᵩ'
    assert upretty( Symbol('F_chi') )   == u'Fᵪ'

    assert upretty( Symbol('F_a') )     == u'Fₐ'
    assert upretty( Symbol('F_e') )     == u'Fₑ'
    assert upretty( Symbol('F_i') )     == u'Fᵢ'
    assert upretty( Symbol('F_o') )     == u'Fₒ'
    assert upretty( Symbol('F_u') )     == u'Fᵤ'
    assert upretty( Symbol('F_r') )     == u'Fᵣ'
    assert upretty( Symbol('F_v') )     == u'Fᵥ'
    assert upretty( Symbol('F_x') )     == u'Fₓ'


def test_pretty_basic():
    assert pretty( -Rational(1)/2 ) == '-1/2'
    assert pretty( -Rational(13)/22 ) == \
"""\
  13\n\
- --\n\
  22\
"""
    expr = oo
    ascii_str = \
"""\
oo\
"""
    ucode_str = \
u"""\
∞\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = (x**2)
    ascii_str = \
"""\
 2\n\
x \
"""
    ucode_str = \
u"""\
 2\n\
x \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = 1/x
    ascii_str = \
"""\
1\n\
-\n\
x\
"""
    ucode_str = \
u"""\
1\n\
─\n\
x\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = y*x**-2
    ascii_str = \
"""\
y \n\
--\n\
 2\n\
x \
"""
    ucode_str = \
u"""\
y \n\
──\n\
 2\n\
x \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = x**Rational(-5,2)
    ascii_str = \
"""\
 1  \n\
----\n\
 5/2\n\
x   \
"""
    ucode_str = \
u"""\
 1  \n\
────\n\
 5/2\n\
x   \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = (-2)**x
    ascii_str = \
"""\
    x\n\
(-2) \
"""
    ucode_str = \
u"""\
    x\n\
(-2) \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    # See issue 1824
    expr = Pow(3, 1, evaluate=False)
    ascii_str = \
"""\
 1\n\
3 \
"""
    ucode_str = \
u"""\
 1\n\
3 \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = (x**2 + x + 1)
    ascii_str_1 = \
"""\
         2\n\
1 + x + x \
"""
    ascii_str_2 = \
"""\
 2        \n\
x  + x + 1\
"""
    ascii_str_3 = \
"""\
 2        \n\
x  + 1 + x\
"""
    ucode_str_1 = \
u"""\
         2\n\
1 + x + x \
"""
    ucode_str_2 = \
u"""\
 2        \n\
x  + x + 1\
"""
    ucode_str_3 = \
u"""\
 2        \n\
x  + 1 + x\
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2, ascii_str_3]
    assert upretty(expr) in [ucode_str_1, ucode_str_2, ucode_str_3]

    expr = 1-x
    ascii_str_1 = \
"""\
1 - x\
"""
    ascii_str_2 = \
"""\
-x + 1\
"""
    ucode_str_1 = \
u"""\
1 - x\
"""
    ucode_str_2 = \
u"""\
-x + 1\
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    expr = 1-2*x
    ascii_str_1 = \
"""\
1 - 2*x\
"""
    ascii_str_2 = \
"""\
-2*x + 1\
"""
    ucode_str_1 = \
u"""\
1 - 2⋅x\
"""
    ucode_str_2 = \
u"""\
-2⋅x + 1\
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    expr = x/y
    ascii_str = \
"""\
x\n\
-\n\
y\
"""
    ucode_str = \
u"""\
x\n\
─\n\
y\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = -x/y
    ascii_str = \
"""\
-x\n\
--\n\
y \
"""
    ucode_str = \
u"""\
-x\n\
──\n\
y \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = (x+2)/y
    ascii_str_1 = \
"""\
2 + x\n\
-----\n\
  y  \
"""
    ascii_str_2 = \
"""\
x + 2\n\
-----\n\
  y  \
"""
    ucode_str_1 = \
u"""\
2 + x\n\
─────\n\
  y  \
"""
    ucode_str_2 = \
u"""\
x + 2\n\
─────\n\
  y  \
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    expr = (1+x)*y
    ascii_str_1 = \
"""\
y*(1 + x)\
"""
    ascii_str_2 = \
"""\
(1 + x)*y\
"""
    ascii_str_3 = \
"""\
y*(x + 1)\
"""
    ucode_str_1 = \
u"""\
y⋅(1 + x)\
"""
    ucode_str_2 = \
u"""\
(1 + x)⋅y\
"""
    ucode_str_3 = \
u"""\
y⋅(x + 1)\
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2, ascii_str_3]
    assert upretty(expr) in [ucode_str_1, ucode_str_2, ucode_str_3]

    # Test for correct placement of the negative sign
    expr = -5*x/(x+10)
    ascii_str_1 = \
"""\
 -5*x \n\
------\n\
10 + x\
"""
    ascii_str_2 = \
"""\
 -5*x \n\
------\n\
x + 10\
"""
    ucode_str_1 = \
u"""\
 -5⋅x \n\
──────\n\
10 + x\
"""
    ucode_str_2 = \
u"""\
 -5⋅x \n\
──────\n\
x + 10\
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    expr = -S(1)/2 - 3*x
    ascii_str = \
"""\
-3*x - 1/2\
"""
    ucode_str = \
u"""\
-3⋅x - 1/2\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = S(1)/2 - 3*x
    ascii_str = \
"""\
-3*x + 1/2\
"""
    ucode_str = \
u"""\
-3⋅x + 1/2\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = -S(1)/2 - 3*x/2
    ascii_str = \
"""\
  3*x   1\n\
- --- - -\n\
   2    2\
"""
    ucode_str = \
u"""\
  3⋅x   1\n\
- ─── - ─\n\
   2    2\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = S(1)/2 - 3*x/2
    ascii_str = \
"""\
  3*x   1\n\
- --- + -\n\
   2    2\
"""
    ucode_str = \
u"""\
  3⋅x   1\n\
- ─── + ─\n\
   2    2\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

def test_issue_2425():
    assert pretty(-(-x + 5)*(-x - 2*2**(1/S(2)) + 5) - (-y + 5)*(-y + 5)) == \
"""\
        /         ___    \\           2\n\
(x - 5)*\\-x - 2*\\/ 2  + 5/ - (-y + 5) \
"""

    assert upretty(-(-x + 5)*(-x - 2*2**(1/S(2)) + 5) - (-y + 5)*(-y + 5)) == \
u"""\
        ⎛         ⎽⎽⎽    ⎞           2\n\
(x - 5)⋅⎝-x - 2⋅╲╱ 2  + 5⎠ - (-y + 5) \
"""

def test_pretty_ordering():
    assert pretty(x**2 + x + 1, order='lex') == \
"""\
 2        \n\
x  + x + 1\
"""
    assert pretty(x**2 + x + 1, order='rev-lex') == \
"""\
         2\n\
1 + x + x \
"""
    assert pretty(1 - x, order='lex') == '-x + 1'
    assert pretty(1 - x, order='rev-lex') == '1 - x'

    assert pretty(1 - 2*x, order='lex') == '-2*x + 1'
    assert pretty(1 - 2*x, order='rev-lex') == '1 - 2*x'

    f = 2*x**4 + y**2 - x**2 + y**3
    assert pretty(f, order=None) == \
"""\
   4    2    3    2\n\
2*x  - x  + y  + y \
"""
    assert pretty(f, order='lex') == \
"""\
   4    2    3    2\n\
2*x  - x  + y  + y \
"""
    assert pretty(f, order='rev-lex') == \
"""\
 2    3    2      4\n\
y  + y  - x  + 2*x \
"""

    expr = x - x**3/6 + x**5/120 + O(x**6)

    ascii_str = \
"""\
     3     5          \n\
    x     x           \n\
x - -- + --- + O(x**6)\n\
    6    120          \
"""
    ucode_str = \
u"""\
     3     5          \n\
    x     x           \n\
x - ── + ─── + O(x**6)\n\
    6    120          \
"""

    assert  pretty(expr, order=None) == ascii_str
    assert upretty(expr, order=None) == ucode_str

    assert  pretty(expr, order='lex') == ascii_str
    assert upretty(expr, order='lex') == ucode_str

    assert  pretty(expr, order='rev-lex') == ascii_str
    assert upretty(expr, order='rev-lex') == ucode_str

def test_pretty_relational():
    expr = Eq(x, y)
    ascii_str = \
"""\
x = y\
"""
    ucode_str = \
u"""\
x = y\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Lt(x, y)
    ascii_str = \
"""\
x < y\
"""
    ucode_str = \
u"""\
x < y\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Gt(x, y)
    ascii_str = \
"""\
y < x\
"""
    ucode_str = \
u"""\
y < x\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Le(x, y)
    ascii_str = \
"""\
x <= y\
"""
    ucode_str = \
u"""\
x ≤ y\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Ge(x, y)
    ascii_str = \
"""\
y <= x\
"""
    ucode_str = \
u"""\
y ≤ x\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Ne(x/(y+1), y**2)
    ascii_str_1 = \
"""\
  x       2\n\
----- != y \n\
1 + y      \
"""
    ascii_str_2 = \
"""\
  x       2\n\
----- != y \n\
y + 1      \
"""
    ucode_str_1 = \
u"""\
  x      2\n\
───── ≠ y \n\
1 + y     \
"""
    ucode_str_2 = \
u"""\
  x      2\n\
───── ≠ y \n\
y + 1     \
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]


def test_pretty_rational():
    expr = y*x**-2
    ascii_str = \
"""\
y \n\
--\n\
 2\n\
x \
"""
    ucode_str = \
u"""\
y \n\
──\n\
 2\n\
x \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = y**Rational(3,2) * x**Rational(-5,2)
    ascii_str = \
"""\
 3/2\n\
y   \n\
----\n\
 5/2\n\
x   \
"""
    ucode_str = \
u"""\
 3/2\n\
y   \n\
────\n\
 5/2\n\
x   \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = sin(x)**3/tan(x)**2
    ascii_str = \
"""\
   3   \n\
sin (x)\n\
-------\n\
   2   \n\
tan (x)\
"""
    ucode_str = \
u"""\
   3   \n\
sin (x)\n\
───────\n\
   2   \n\
tan (x)\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str


def test_pretty_functions():
    """Tests for Abs, conjugate, exp, function braces, and factorial."""
    expr = (2*x + exp(x))
    ascii_str_1 = \
"""\
       x\n\
2*x + e \
"""
    ascii_str_2 = \
"""\
 x      \n\
e  + 2*x\
"""
    ucode_str_1 = \
u"""\
       x\n\
2⋅x + ℯ \
"""
    ucode_str_2 = \
u"""\
 x     \n\
ℯ + 2⋅x\
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    expr = Abs(x)
    ascii_str = \
"""\
|x|\
"""
    ucode_str = \
u"""\
│x│\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Abs(x/(x**2+1))
    ascii_str_1 = \
"""\
|  x   |\n\
|------|\n\
|     2|\n\
|1 + x |\
"""
    ascii_str_2 = \
"""\
|  x   |\n\
|------|\n\
| 2    |\n\
|x  + 1|\
"""
    ucode_str_1 = \
u"""\
│  x   │\n\
│──────│\n\
│     2│\n\
│1 + x │\
"""
    ucode_str_2 = \
u"""\
│  x   │\n\
│──────│\n\
│ 2    │\n\
│x  + 1│\
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    expr = Abs(1 / (y - Abs(x)))
    ascii_str = \
"""\
|   1   |\n\
|-------|\n\
|y - |x||\
"""
    ucode_str = \
u"""\
│   1   │\n\
│───────│\n\
│y - │x││\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    n = Symbol('n', integer=True)
    expr = factorial(n)
    ascii_str = \
"""\
n!\
"""
    ucode_str = \
u"""\
n!\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str


    expr = factorial(2*n)
    ascii_str = \
"""\
(2*n)!\
"""
    ucode_str = \
u"""\
(2⋅n)!\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = factorial(factorial(factorial(n)))
    ascii_str = \
"""\
((n!)!)!\
"""
    ucode_str = \
u"""\
((n!)!)!\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = factorial(n+1)
    ascii_str_1 = \
"""\
(1 + n)!\
"""
    ascii_str_2 = \
"""\
(n + 1)!\
"""
    ucode_str_1 = \
u"""\
(1 + n)!\
"""
    ucode_str_2 = \
u"""\
(n + 1)!\
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    expr = 2*binomial(n, k)
    ascii_str = \
"""\
  /n\\\n\
2*| |\n\
  \k/\
"""
    ucode_str = \
u"""\
  ⎛n⎞\n\
2⋅⎜ ⎟\n\
  ⎝k⎠\
"""

    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = 2*binomial(2*n, k)
    ascii_str = \
"""\
  /2*n\\\n\
2*|   |\n\
  \ k /\
"""
    ucode_str = \
u"""\
  ⎛2⋅n⎞\n\
2⋅⎜   ⎟\n\
  ⎝ k ⎠\
"""

    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = 2*binomial(n**2, k)
    ascii_str = \
"""\
  / 2\\\n\
  |n |\n\
2*|  |\n\
  \k /\
"""
    ucode_str = \
u"""\
  ⎛ 2⎞\n\
  ⎜n ⎟\n\
2⋅⎜  ⎟\n\
  ⎝k ⎠\
"""

    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = conjugate(x)
    ascii_str = \
"""\
_\n\
x\
"""
    ucode_str = \
u"""\
⎽\n\
x\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    f = Function('f')
    expr = conjugate(f(x+1))
    ascii_str_1 = \
"""\
________\n\
f(1 + x)\
"""
    ascii_str_2 = \
"""\
________\n\
f(x + 1)\
"""
    ucode_str_1 = \
u"""\
⎽⎽⎽⎽⎽⎽⎽⎽\n\
f(1 + x)\
"""
    ucode_str_2 = \
u"""\
⎽⎽⎽⎽⎽⎽⎽⎽\n\
f(x + 1)\
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    expr = f(x)
    ascii_str = \
"""\
f(x)\
"""
    ucode_str = \
u"""\
f(x)\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = f(x, y)
    ascii_str = \
"""\
f(x, y)\
"""
    ucode_str = \
u"""\
f(x, y)\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = f(x/(y+1), y)
    ascii_str_1 = \
"""\
 /  x     \\\n\
f|-----, y|\n\
 \\1 + y   /\
"""
    ascii_str_2 = \
"""\
 /  x     \\\n\
f|-----, y|\n\
 \\y + 1   /\
"""
    ucode_str_1 = \
u"""\
 ⎛  x     ⎞\n\
f⎜─────, y⎟\n\
 ⎝1 + y   ⎠\
"""
    ucode_str_2 = \
u"""\
 ⎛  x     ⎞\n\
f⎜─────, y⎟\n\
 ⎝y + 1   ⎠\
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    expr = sin(x)**2
    ascii_str = \
"""\
   2   \n\
sin (x)\
"""
    ucode_str = \
u"""\
   2   \n\
sin (x)\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = conjugate(a+b*I)
    ascii_str = \
"""\
_     _\n\
a - I*b\
"""
    ucode_str = \
u"""\
⎽     ⎽\n\
a - ⅈ⋅b\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = conjugate(exp(a+b*I))
    ascii_str = \
"""\
 _     _\n\
 a - I*b\n\
e       \
"""
    ucode_str = \
u"""\
 ⎽     ⎽\n\
 a - ⅈ⋅b\n\
ℯ       \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = conjugate( f(1 + conjugate(f(x))) )
    ascii_str_1 = \
"""\
___________\n\
 /    ____\\\n\
f\\1 + f(x)/\
"""
    ascii_str_2 = \
"""\
___________\n\
 /____    \\\n\
f\\f(x) + 1/\
"""
    ucode_str_1 = \
u"""\
⎽⎽⎽⎽⎽⎽⎽⎽⎽⎽⎽\n\
 ⎛    ⎽⎽⎽⎽⎞\n\
f⎝1 + f(x)⎠\
"""
    ucode_str_2 = \
u"""\
⎽⎽⎽⎽⎽⎽⎽⎽⎽⎽⎽\n\
 ⎛⎽⎽⎽⎽    ⎞\n\
f⎝f(x) + 1⎠\
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    expr = f(x/(y+1), y)
    ascii_str_1 = \
"""\
 /  x     \\\n\
f|-----, y|\n\
 \\1 + y   /\
"""
    ascii_str_2 = \
"""\
 /  x     \\\n\
f|-----, y|\n\
 \\y + 1   /\
"""
    ucode_str_1 = \
u"""\
 ⎛  x     ⎞\n\
f⎜─────, y⎟\n\
 ⎝1 + y   ⎠\
"""
    ucode_str_2 = \
u"""\
 ⎛  x     ⎞\n\
f⎜─────, y⎟\n\
 ⎝y + 1   ⎠\
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    expr = floor(1 / (y - floor(x)))
    ascii_str = \
"""\
     /     1      \\\n\
floor|------------|\n\
     \y - floor(x)/\
"""
    ucode_str = \
u"""\
⎢   1   ⎥\n\
⎢───────⎥\n\
⎣y - ⌊x⌋⎦\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = ceiling(1 / (y - ceiling(x)))
    ascii_str = \
"""\
       /      1       \\\n\
ceiling|--------------|\n\
       \y - ceiling(x)/\
"""
    ucode_str = \
u"""\
⎡   1   ⎤\n\
⎢───────⎥\n\
⎢y - ⌈x⌉⎥\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str


def test_pretty_sqrt():
    expr = sqrt(2)
    ascii_str = \
"""\
  ___\n\
\/ 2 \
"""
    ucode_str = \
u"""\
  ⎽⎽⎽\n\
╲╱ 2 \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = 2**Rational(1,3)
    ascii_str = \
"""\
3 ___\n\
\/ 2 \
"""
    ucode_str = \
u"""\
3 ⎽⎽⎽\n\
╲╱ 2 \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = 2**Rational(1,1000)
    ascii_str = \
"""\
1000___\n\
  \/ 2 \
"""
    ucode_str = \
u"""\
1000⎽⎽⎽\n\
  ╲╱ 2 \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = sqrt(x**2 + 1)
    ascii_str = \
"""\
   ________\n\
  /  2     \n\
\/  x  + 1 \
"""
    ucode_str = \
u"""\
   ⎽⎽⎽⎽⎽⎽⎽⎽\n\
  ╱  2     \n\
╲╱  x  + 1 \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = (1 + sqrt(5))**Rational(1,3)
    ascii_str = \
"""\
   ___________\n\
3 /       ___ \n\
\/  1 + \/ 5  \
"""
    ucode_str = \
u"""\
   ⎽⎽⎽⎽⎽⎽⎽⎽⎽⎽⎽\n\
3 ╱       ⎽⎽⎽ \n\
╲╱  1 + ╲╱ 5  \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = 2**(1/x)
    ascii_str = \
"""\
x ___\n\
\/ 2 \
"""
    ucode_str = \
u"""\
x ⎽⎽⎽\n\
╲╱ 2 \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = sqrt(2+pi)
    ascii_str = \
"""\
  ________\n\
\/ 2 + pi \
"""
    ucode_str = \
u"""\
  ⎽⎽⎽⎽⎽⎽⎽\n\
╲╱ 2 + π \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = (2+(1+x**2)/(2+x))**Rational(1,4)+(1+x**Rational(1,1000))/sqrt(3+x**2)
    ascii_str = \
"""\
     ____________              \n\
    /      2        1000___    \n\
   /      x  + 1      \/ x  + 1\n\
4 /   2 + ------  + -----------\n\
\/        x + 2        ________\n\
                      /  2     \n\
                    \/  x  + 3 \
"""
    ucode_str = \
u"""\
     ⎽⎽⎽⎽⎽⎽⎽⎽⎽⎽⎽⎽              \n\
    ╱      2        1000⎽⎽⎽    \n\
   ╱      x  + 1      ╲╱ x  + 1\n\
4 ╱   2 + ──────  + ───────────\n\
╲╱        x + 2        ⎽⎽⎽⎽⎽⎽⎽⎽\n\
                      ╱  2     \n\
                    ╲╱  x  + 3 \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

def test_pretty_lambda():
    expr = Lambda(x, x)
    ascii_str = \
"""\
Lambda(x, x)\
"""
    ucode_str = \
u"""\
Λ(x, x)\
"""

    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Lambda(x, x**2)
    ascii_str = \
"""\
      /    2\\\n\
Lambda\\x, x /\
"""
    ucode_str = \
u"""\
 ⎛    2⎞\n\
Λ⎝x, x ⎠\
"""

    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Lambda(x, x**2)**2
    ascii_str = \
"""\
      2       \n\
       /    2\\\n\
Lambda \\x, x /\
"""
    ucode_str = \
u"""\
 2       \n\
  ⎛    2⎞\n\
Λ ⎝x, x ⎠\
"""

    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Lambda((x, y), x)
    ascii_str = \
"""\
Lambda((x, y), x)\
"""
    ucode_str = \
u"""\
Λ((x, y), x)\
"""

    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Lambda((x, y), x**2)
    ascii_str = \
"""\
      /         2\\\n\
Lambda\\(x, y), x /\
"""
    ucode_str = \
u"""\
 ⎛         2⎞\n\
Λ⎝(x, y), x ⎠\
"""

    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

def test_pretty_derivatives():
    # Simple
    expr = Derivative(log(x), x, evaluate=False)
    ascii_str = \
"""\
d         \n\
--(log(x))\n\
dx        \
"""
    ucode_str = \
u"""\
d         \n\
──(log(x))\n\
dx        \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Derivative(log(x), x, evaluate=False) + x
    ascii_str_1 = \
"""\
    d         \n\
x + --(log(x))\n\
    dx        \
"""
    ascii_str_2 = \
"""\
d             \n\
--(log(x)) + x\n\
dx            \
"""
    ucode_str_1 = \
u"""\
    d         \n\
x + ──(log(x))\n\
    dx        \
"""
    ucode_str_2 = \
u"""\
d             \n\
──(log(x)) + x\n\
dx            \
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    # Multiple symbols
    expr = Derivative(log(x) + x**2, x, y)
    ascii_str_1 = \
"""\
   2              \n\
  d  /          2\\\n\
-----\log(x) + x /\n\
dy dx             \
"""
    ascii_str_2 = \
"""\
   2              \n\
  d  / 2         \\\n\
-----\\x  + log(x)/\n\
dy dx             \
"""
    ucode_str_1 = \
u"""\
   2              \n\
  d  ⎛          2⎞\n\
─────⎝log(x) + x ⎠\n\
dy dx             \
"""
    ucode_str_2 = \
u"""\
   2              \n\
  d  ⎛ 2         ⎞\n\
─────⎝x  + log(x)⎠\n\
dy dx             \
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    expr = Derivative(2*x*y, y, x) + x**2
    ascii_str_1 = \
"""\
   2             \n\
  d             2\n\
-----(2*x*y) + x \n\
dx dy            \
"""
    ascii_str_2 = \
"""\
        2        \n\
 2     d         \n\
x  + -----(2*x*y)\n\
     dx dy       \
"""
    ucode_str_1 = \
u"""\
   2             \n\
  d             2\n\
─────(2⋅x⋅y) + x \n\
dx dy            \
"""
    ucode_str_2 = \
u"""\
        2        \n\
 2     d         \n\
x  + ─────(2⋅x⋅y)\n\
     dx dy       \
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    expr = Derivative(2*x*y, x, x)
    ascii_str = \
"""\
  2       \n\
 d        \n\
---(2*x*y)\n\
  2       \n\
dx        \
"""
    ucode_str = \
u"""\
  2       \n\
 d        \n\
───(2⋅x⋅y)\n\
  2       \n\
dx        \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Derivative(2*x*y, x, 17)
    ascii_str = \
"""\
 17        \n\
d          \n\
----(2*x*y)\n\
  17       \n\
dx         \
"""
    ucode_str = \
u"""\
 17        \n\
d          \n\
────(2⋅x⋅y)\n\
  17       \n\
dx         \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Derivative(2*x*y, x, x, y)
    ascii_str = \
"""\
   3         \n\
  d          \n\
------(2*x*y)\n\
     2       \n\
dy dx        \
"""
    ucode_str = \
u"""\
   3         \n\
  d          \n\
──────(2⋅x⋅y)\n\
     2       \n\
dy dx        \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    # Greek letters
    alpha = Symbol('alpha')
    beta  = Function('beta')
    expr = beta(alpha).diff(alpha)
    ascii_str = \
"""\
  d                \n\
------(beta(alpha))\n\
dalpha             \
"""
    ucode_str = \
u"""\
d       \n\
──(β(α))\n\
dα      \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

def test_pretty_integrals():
    expr = Integral(log(x), x)
    ascii_str = \
"""\
  /         \n\
 |          \n\
 | log(x) dx\n\
 |          \n\
/           \
"""
    ucode_str = \
u"""\
⌠          \n\
⎮ log(x) dx\n\
⌡          \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Integral(x**2, x)
    ascii_str = \
"""\
  /     \n\
 |      \n\
 |  2   \n\
 | x  dx\n\
 |      \n\
/       \
"""
    ucode_str = \
u"""\
⌠      \n\
⎮  2   \n\
⎮ x  dx\n\
⌡      \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Integral((sin(x))**2 / (tan(x))**2)
    ascii_str = \
"""\
  /          \n\
 |           \n\
 |    2      \n\
 | sin (x)   \n\
 | ------- dx\n\
 |    2      \n\
 | tan (x)   \n\
 |           \n\
/            \
"""
    ucode_str = \
u"""\
⌠           \n\
⎮    2      \n\
⎮ sin (x)   \n\
⎮ ─────── dx\n\
⎮    2      \n\
⎮ tan (x)   \n\
⌡           \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Integral(x**(2**x), x)
    ascii_str = \
"""\
  /        \n\
 |         \n\
 |  / x\   \n\
 |  \\2 /   \n\
 | x     dx\n\
 |         \n\
/          \
"""
    ucode_str = \
u"""\
⌠         \n\
⎮  ⎛ x⎞   \n\
⎮  ⎝2 ⎠   \n\
⎮ x     dx\n\
⌡         \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Integral(x**2, (x,1,2))
    ascii_str = \
"""\
  2      \n\
  /      \n\
 |       \n\
 |   2   \n\
 |  x  dx\n\
 |       \n\
/        \n\
1        \
"""
    ucode_str = \
u"""\
2      \n\
⌠      \n\
⎮  2   \n\
⎮ x  dx\n\
⌡      \n\
1      \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Integral(x**2, (x,Rational(1,2),10))
    ascii_str = \
"""\
 10      \n\
  /      \n\
 |       \n\
 |   2   \n\
 |  x  dx\n\
 |       \n\
/        \n\
1/2      \
"""
    ucode_str = \
u"""\
 10      \n\
 ⌠       \n\
 ⎮   2   \n\
 ⎮  x  dx\n\
 ⌡       \n\
1/2      \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Integral(x**2*y**2, x,y)
    ascii_str = \
"""\
  /  /           \n\
 |  |            \n\
 |  |  2  2      \n\
 |  | x *y  dx dy\n\
 |  |            \n\
/  /             \
"""
    ucode_str = \
u"""\
⌠ ⌠            \n\
⎮ ⎮  2  2      \n\
⎮ ⎮ x ⋅y  dx dy\n\
⌡ ⌡            \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Integral(sin(th)/cos(ph), (th,0,pi), (ph, 0, 2*pi))
    ascii_str = \
"""\
 2*pi pi                           \n\
   /   /                           \n\
  |   |                            \n\
  |   |  sin(theta)                \n\
  |   |  ---------- d(theta) d(phi)\n\
  |   |   cos(phi)                 \n\
  |   |                            \n\
 /   /                             \n\
 0   0                             \
"""
    ucode_str = \
u"""\
2⋅π π             \n\
 ⌠  ⌠             \n\
 ⎮  ⎮ sin(θ)      \n\
 ⎮  ⎮ ────── dθ dφ\n\
 ⎮  ⎮ cos(φ)      \n\
 ⌡  ⌡             \n\
 0  0             \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str


def test_pretty_matrix():
    # Empty Matrix
    expr = Matrix()
    ascii_str = "[]"
    unicode_str = "[]"
    assert pretty(expr) == ascii_str
    assert upretty(expr) == unicode_str
    expr = Matrix([[x**2+1, 1], [y, x+y]])
    ascii_str_1 = \
"""\
[     2       ]
[1 + x     1  ]
[             ]
[  y     x + y]\
"""
    ascii_str_2 = \
"""\
[ 2           ]
[x  + 1    1  ]
[             ]
[  y     x + y]\
"""
    ucode_str_1 = \
u"""\
⎡     2       ⎤
⎢1 + x     1  ⎥
⎢             ⎥
⎣  y     x + y⎦\
"""
    ucode_str_2 = \
u"""\
⎡ 2           ⎤
⎢x  + 1    1  ⎥
⎢             ⎥
⎣  y     x + y⎦\
"""
    assert  pretty(expr) in [ascii_str_1, ascii_str_2]
    assert upretty(expr) in [ucode_str_1, ucode_str_2]

    expr = Matrix([[x/y, y, th], [0, exp(I*k*ph), 1]])
    ascii_str = \
"""\
[x                 ]
[-     y      theta]
[y                 ]
[                  ]
[    I*k*phi       ]
[0  e           1  ]\
"""
    ucode_str = \
u"""\
⎡x           ⎤
⎢─    y     θ⎥
⎢y           ⎥
⎢            ⎥
⎢    ⅈ⋅k⋅φ   ⎥
⎣0  ℯ       1⎦\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str


def test_pretty_piecewise():
    expr = Piecewise((x,x<1),(x**2,True))
    ascii_str = \
"""\
/x   for x < 1\n\
|             \n\
< 2           \n\
|x   otherwise\n\
\             \
"""
    ucode_str = \
u"""\
⎧x   for x < 1\n\
⎪             \n\
⎨ 2           \n\
⎪x   otherwise\n\
⎩             \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str


def test_pretty_seq():
    expr = ()
    ascii_str = \
"""\
()\
"""
    ucode_str = \
u"""\
()\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = []
    ascii_str = \
"""\
[]\
"""
    ucode_str = \
u"""\
[]\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = {}
    ascii_str = \
"""\
{}\
"""
    ucode_str = \
u"""\
{}\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = (1/x,)
    ascii_str = \
"""\
 1  \n\
(-,)\n\
 x  \
"""
    ucode_str = \
u"""\
⎛1 ⎞\n\
⎜─,⎟\n\
⎝x ⎠\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = [x**2, 1/x, x, y, sin(th)**2/cos(ph)**2]
    ascii_str = \
"""\
                 2        \n\
  2  1        sin (theta) \n\
[x , -, x, y, -----------]\n\
     x            2       \n\
               cos (phi)  \
"""
    ucode_str = \
u"""\
⎡                2   ⎤\n\
⎢ 2  1        sin (θ)⎥\n\
⎢x , ─, x, y, ───────⎥\n\
⎢    x           2   ⎥\n\
⎣             cos (φ)⎦\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = (x**2, 1/x, x, y, sin(th)**2/cos(ph)**2)
    ascii_str = \
"""\
                 2        \n\
  2  1        sin (theta) \n\
(x , -, x, y, -----------)\n\
     x            2       \n\
               cos (phi)  \
"""
    ucode_str = \
u"""\
⎛                2   ⎞\n\
⎜ 2  1        sin (θ)⎟\n\
⎜x , ─, x, y, ───────⎟\n\
⎜    x           2   ⎟\n\
⎝             cos (φ)⎠\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = {x: sin(x)}
    ascii_str = \
"""\
{x: sin(x)}\
"""
    ucode_str = \
u"""\
{x: sin(x)}\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = {1/x: 1/y, x: sin(x)**2}
    ascii_str = \
"""\
 1  1        2    \n\
{-: -, x: sin (x)}\n\
 x  y             \
"""
    ucode_str = \
u"""\
⎧1  1        2   ⎫\n\
⎨─: ─, x: sin (x)⎬\n\
⎩x  y            ⎭\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    # There used to be a bug with pretty-printing sequences of even height.
    expr = [x**2]
    ascii_str = \
"""\
  2 \n\
[x ]\
"""
    ucode_str = \
u"""\
⎡ 2⎤\n\
⎣x ⎦\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = (x**2,)
    ascii_str = \
"""\
  2  \n\
(x ,)\
"""
    ucode_str = \
u"""\
⎛ 2 ⎞\n\
⎝x ,⎠\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = {x**2: 1}
    ascii_str = \
"""\
  2    \n\
{x : 1}\
"""
    ucode_str = \
u"""\
⎧ 2   ⎫\n\
⎨x : 1⎬\n\
⎩     ⎭\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

def test_any_object_in_sequence():
    # Cf. issue 2207
    b1 = Basic()
    b2 = Basic(Basic())

    expr = [b2, b1]
    assert pretty(expr) == "[Basic(Basic()), Basic()]"
    assert upretty(expr) == u"[Basic(Basic()), Basic()]"

    expr = set([b2, b1])
    assert pretty(expr) == "set(Basic(), Basic(Basic()))"
    assert upretty(expr) == u"set(Basic(), Basic(Basic()))"

    expr = {b2:b1, b1:b2}
    assert pretty(expr) == "{Basic(): Basic(Basic()), Basic(Basic()): Basic()}"
    assert upretty(expr) == u"{Basic(): Basic(Basic()), Basic(Basic()): Basic()}"

def test_pretty_limits():
    expr = Limit(x, x, oo)
    ascii_str = \
"""\
 lim x\n\
x->oo \
"""
    ucode_str = \
u"""\
lim x\n\
x->∞ \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Limit(x**2, x, 0)
    ascii_str = \
"""\
     2\n\
lim x \n\
x->0  \
"""
    ucode_str = \
u"""\
     2\n\
lim x \n\
x->0  \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Limit(1/x, x, 0)
    ascii_str = \
"""\
    1\n\
lim -\n\
x->0x\
"""
    ucode_str = \
u"""\
    1\n\
lim ─\n\
x->0x\
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = Limit(sin(x)/x, x, 0)
    ascii_str = \
"""\
    sin(x)\n\
lim ------\n\
x->0  x   \
"""
    ucode_str = \
u"""\
    sin(x)\n\
lim ──────\n\
x->0  x   \
"""
    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

def test_pretty_RootOf():
    expr = RootOf(x**5 + 11*x - 2, 0)
    ascii_str = \
"""\
      / 5              \\\n\
RootOf\\x  + 11*x - 2, 0/\
"""
    ucode_str = \
u"""\
      ⎛ 5              ⎞\n\
RootOf⎝x  + 11⋅x - 2, 0⎠\
"""

    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

def test_pretty_RootSum():
    expr = RootSum(x**5 + 11*x - 2, auto=False)
    ascii_str = \
"""\
       / 5           \\\n\
RootSum\\x  + 11*x - 2/\
"""
    ucode_str = \
u"""\
       ⎛ 5           ⎞\n\
RootSum⎝x  + 11⋅x - 2⎠\
"""

    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = RootSum(x**5 + 11*x - 2, Lambda(z, exp(z)))
    ascii_str = \
"""\
       / 5                   /    z\\\\\n\
RootSum\\x  + 11*x - 2, Lambda\\z, e //\
"""
    ucode_str = \
u"""\
       ⎛ 5              ⎛    z⎞⎞\n\
RootSum⎝x  + 11⋅x - 2, Λ⎝z, ℯ ⎠⎠\
"""

    assert  pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

def test_pretty_Boolean():
    expr = Not(x, evaluate=False)

    assert  pretty(expr) == "Not(x)"
    assert upretty(expr) == u"¬ x"

    expr = And(x, y)

    assert  pretty(expr) == "And(x, y)"
    assert upretty(expr) == u"x ∧ y"

    expr = Or(x, y)

    assert  pretty(expr) == "Or(x, y)"
    assert upretty(expr) == u"x ∨ y"

    expr = Xor(x, y, evaluate=False)

    assert  pretty(expr) == "Xor(x, y)"
    assert upretty(expr) == u"x ⊻ y"

    expr = Nand(x, y, evaluate=False)

    assert  pretty(expr) == "Nand(x, y)"
    assert upretty(expr) == u"x ⊼ y"

    expr = Nor(x, y, evaluate=False)

    assert  pretty(expr) == "Nor(x, y)"
    assert upretty(expr) == u"x ⊽ y"

    expr = Implies(x, y, evaluate=False)

    assert  pretty(expr) == "Implies(x, y)"
    assert upretty(expr) == u"x → y"

    expr = Equivalent(x, y, evaluate=False)

    assert  pretty(expr) == "Equivalent(x, y)"
    assert upretty(expr) == u"x ≡ y"

def test_pretty_Domain():
    expr = FF(23)

    assert  pretty(expr) == "GF(23)"
    assert upretty(expr) == u"ℤ₂₃"

    expr = ZZ

    assert  pretty(expr) == "ZZ"
    assert upretty(expr) == u"ℤ"

    expr = QQ

    assert  pretty(expr) == "QQ"
    assert upretty(expr) == u"ℚ"

    expr = RR

    assert  pretty(expr) == "RR"
    assert upretty(expr) == u"ℝ"

    expr = QQ[x]

    assert  pretty(expr) == "QQ[x]"
    assert upretty(expr) == u"ℚ[x]"

    expr = QQ[x, y]

    assert  pretty(expr) == "QQ[x, y]"
    assert upretty(expr) == u"ℚ[x, y]"

    expr = ZZ.frac_field(x)

    assert  pretty(expr) == "ZZ(x)"
    assert upretty(expr) == u"ℤ(x)"

    expr = ZZ.frac_field(x, y)

    assert  pretty(expr) == "ZZ(x, y)"
    assert upretty(expr) == u"ℤ(x, y)"

def test_pretty_prec():
    assert xpretty(S("0.3"), full_prec=True) == "0.300000000000000"
    assert xpretty(S("0.3"), full_prec="auto") == "0.300000000000000"
    assert xpretty(S("0.3"), full_prec=False) == "0.3"
    assert xpretty(S("0.3")*x, full_prec=True, use_unicode=False) in [
            "0.300000000000000*x",
            "x*0.300000000000000"
            ]
    assert xpretty(S("0.3")*x, full_prec="auto", use_unicode=False) in [
            "0.3*x",
            "x*0.3"
            ]
    assert xpretty(S("0.3")*x, full_prec=False, use_unicode=False) in [
            "0.3*x",
            "x*0.3"
            ]


def test_pprint():
    import StringIO, sys
    fd = StringIO.StringIO()
    sso = sys.stdout
    sys.stdout = fd
    try:
        #FIXME-py3k: sympy/printing/pretty/stringpict.py", line 286, in terminal_width
        #FIXME-py3k: curses.setupterm()
        #FIXME-py3k: io.UnsupportedOperation: fileno
        pprint(pi, use_unicode=False)
    finally:
        sys.stdout = sso
    assert fd.getvalue() == 'pi\n'


def test_pretty_class():
    """Test that the printer dispatcher correctly handles classes."""
    class C: pass   # C has no .__class__ and this was causing problems
    class D(object): pass

    assert pretty( C ) == str( C )
    assert pretty( D ) == str( D )


def test_pretty_no_wrap_line():
    huge_expr = 0
    for i in range(20):
        huge_expr += i*sin(i+x)
    assert xpretty(huge_expr            ).find('\n') != -1
    assert xpretty(huge_expr, wrap_line=False).find('\n') == -1


def test_settings():
    raises(TypeError, 'pretty(S(4), method="garbage")')

def test_pretty_sum():
    from sympy.abc import x, a, b, k, m, n

    expr = Sum(x, (x, 0, oo))
    ascii_str = \
"""\
  oo   \n\
 __    \n\
 \\ `   \n\
  )   x\n\
 /_,   \n\
x = 0  \
"""

    assert  pretty(expr) == ascii_str
    #assert upretty(expr) == ucode_str

    expr = Sum(x**2, (x, 0, oo))
    ascii_str = \
"""\
  oo    \n\
 ___    \n\
 \\  `   \n\
  \\    2\n\
  /   x \n\
 /__,   \n\
x = 0   \
"""

    assert  pretty(expr) == ascii_str
    #assert upretty(expr) == ucode_str

    expr = Sum(x/2, (x, 0, oo))
    ascii_str = \
"""\
  oo   \n\
 ___   \n\
 \\  `  \n\
  \\   x\n\
   )  -\n\
  /   2\n\
 /__,  \n\
x = 0  \
"""

    assert  pretty(expr) == ascii_str
    #assert upretty(expr) == ucode_str

    expr = Sum(x**3/2, (x, 0, oo))
    ascii_str = \
"""\
  oo    \n\
____    \n\
\\   `   \n\
 \\     3\n\
  \\   x \n\
  /   --\n\
 /    2 \n\
/___,   \n\
x = 0   \
"""

    assert  pretty(expr) == ascii_str
    #assert upretty(expr) == ucode_str

    expr = Sum((x**3*y**(x/2))**n, (x, 0, oo))
    ascii_str = \
"""\
  oo          \n\
____          \n\
\\   `         \n\
 \\           n\n\
  \\   /    x\\ \n\
   )  |    -| \n\
  /   | 3  2| \n\
 /    \\x *y / \n\
/___,         \n\
x = 0         \
"""

    assert  pretty(expr) == ascii_str
    #assert upretty(expr) == ucode_str

    expr = Sum(1/x**2, (x, 0, oo))
    ascii_str = \
"""\
  oo    \n\
____    \n\
\\   `   \n\
 \\    1 \n\
  \\   --\n\
  /    2\n\
 /    x \n\
/___,   \n\
x = 0   \
"""

    assert  pretty(expr) == ascii_str
    #assert upretty(expr) == ucode_str

    expr = Sum(1/y**(a/b), (x, 0, oo))
    ascii_str = \
"""\
  oo     \n\
____     \n\
\\   `    \n\
 \\     -a\n\
  \\    --\n\
  /    b \n\
 /    y  \n\
/___,    \n\
x = 0    \
"""

    assert  pretty(expr) == ascii_str
    #assert upretty(expr) == ucode_str

    expr = Sum(1/y**(a/b), (x, 0, oo), (y,1,2))
    ascii_str = \
"""\
  2     oo     \n\
____  ____     \n\
\\   ` \\   `    \n\
 \\     \\     -a\n\
  \\     \\    --\n\
  /     /    b \n\
 /     /    y  \n\
/___, /___,    \n\
y = 1 x = 0    \
"""

    assert  pretty(expr) == ascii_str
    #assert upretty(expr) == ucode_str

    expr = Sum(1/(1 + 1/(1 + 1/k)) + 1, (k, 111, 1 + 1/n), (k, 1/(1+m), oo)) + 1/(1 + 1/k)
    ascii_str = \
"""\
               1                         \n\
           1 + -                         \n\
    oo         n                         \n\
  _____    _____                         \n\
  \\    `   \\    `                        \n\
   \\        \\     /        1    \\        \n\
    \\        \\    |1 + ---------|        \n\
     \\        \\   |          1  |     1  \n\
      )        )  |    1 + -----| + -----\n\
     /        /   |            1|       1\n\
    /        /    |        1 + -|   1 + -\n\
   /        /     \\            k/       k\n\
  /____,   /____,                        \n\
      1   k = 111                        \n\
k = -----                                \n\
    m + 1                                \
"""

    assert  pretty(expr) == ascii_str
    #assert upretty(expr) == ucode_str

def test_units():
    # issue 2461
    expr = joule
    ascii_str = \
"""\
    2\n\
kg*m \n\
-----\n\
   2 \n\
  s  \
"""

    unicode_str = \
u"""\
    2\n\
kg⋅m \n\
─────\n\
   2 \n\
  s  \
"""
    assert upretty(expr) == unicode_str
    assert pretty(expr) == ascii_str

def test_pretty_Subs():
    f = Function('f')
    expr = Subs(f(x), x, ph**2)
    ascii_str = \
"""\
(f(x))|     2\n\
      |x=phi \
"""
    unicode_str = \
u"""\
(f(x))│   2\n\
      │x=φ \
"""

    assert pretty(expr) == ascii_str
    assert upretty(expr) == unicode_str

    expr = Subs(f(x).diff(x), x, 0)
    ascii_str = \
"""\
/d       \\|   \n\
|--(f(x))||   \n\
\\dx      /|x=0\
"""
    unicode_str = \
u"""\
⎛d       ⎞│   \n\
⎜──(f(x))⎟│   \n\
⎝dx      ⎠│x=0\
"""

    assert pretty(expr) == ascii_str
    assert upretty(expr) == unicode_str

    expr = Subs(f(x).diff(x)/y, (x, y), (0, Rational(1, 2)))
    ascii_str = \
"""\
/d       \\|          \n\
|--(f(x))||          \n\
|dx      ||          \n\
|--------||          \n\
\\   y    /|x=0, y=1/2\
"""
    unicode_str = \
u"""\
⎛d       ⎞│          \n\
⎜──(f(x))⎟│          \n\
⎜dx      ⎟│          \n\
⎜────────⎟│          \n\
⎝   y    ⎠│x=0, y=1/2\
"""

    assert pretty(expr) == ascii_str
    assert upretty(expr) == unicode_str

def test_gammas():
    assert upretty(lowergamma(x, y)) == u"γ(x, y)"
    assert upretty(uppergamma(x, y)) == u"Γ(x, y)"

def test_hyper():
    expr = hyper((), (), z)
    ucode_str = \
u"""\
 ┌─  ⎛  │  ⎞\n\
 ├─  ⎜  │ z⎟\n\
0╵ 0 ⎝  │  ⎠\
"""
    ascii_str = \
"""\
  _         \n\
 |_  /  |  \\\n\
 |   |  | z|\n\
0  0 \\  |  /\
"""
    assert pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = hyper((), (1,), x)
    ucode_str = \
u"""\
 ┌─  ⎛  │  ⎞\n\
 ├─  ⎜  │ x⎟\n\
0╵ 1 ⎝1 │  ⎠\
"""
    ascii_str = \
"""\
  _         \n\
 |_  /  |  \\\n\
 |   |  | x|\n\
0  1 \\1 |  /\
"""
    assert pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = hyper([2], [1], x)
    ucode_str = \
u"""\
 ┌─  ⎛2 │  ⎞\n\
 ├─  ⎜  │ x⎟\n\
1╵ 1 ⎝1 │  ⎠\
"""
    ascii_str = \
"""\
  _         \n\
 |_  /2 |  \\\n\
 |   |  | x|\n\
1  1 \\1 |  /\
"""
    assert pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = hyper((pi/3, -2*k), (3, 4, 5, -3), x)
    ucode_str = \
u"""\
     ⎛  π         │  ⎞\n\
 ┌─  ⎜  ─, -2⋅k   │  ⎟\n\
 ├─  ⎜  3         │ x⎟\n\
2╵ 4 ⎜            │  ⎟\n\
     ⎝3, 4, 5, -3 │  ⎠\
"""
    ascii_str = \
"""\
                      \n\
  _  /  pi        |  \\\n\
 |_  |  --, -2*k  |  |\n\
 |   |  3         | x|\n\
2  4 |            |  |\n\
     \\3, 4, 5, -3 |  /\
"""
    assert pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = hyper((pi,S('2/3'),-2*k), (3,4,5,-3), x**2)
    ucode_str = \
u"""\
 ┌─  ⎛π, 2/3, -2⋅k │  2⎞\n\
 ├─  ⎜             │ x ⎟\n\
3╵ 4 ⎝3, 4, 5, -3  │   ⎠\
"""
    ascii_str = \
"""\
  _                      \n\
 |_  /pi, 2/3, -2*k |  2\\\n\
 |   |              | x |\n\
3  4 \\ 3, 4, 5, -3  |   /\
"""
    assert pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

def test_meijerg():
    expr = meijerg([pi, pi, x], [1], [0, 1], [1, 2, 3], z)
    ucode_str = \
u"""\
╭─╮2, 3 ⎛π, π, x     1    │  ⎞\n\
│╶┐     ⎜                 │ z⎟\n\
╰─╯4, 5 ⎝ 0, 1    1, 2, 3 │  ⎠\
"""
    ascii_str = \
"""\
 __2, 3 /pi, pi, x     1    |  \\\n\
/__     |                   | z|\n\
\_|4, 5 \\  0, 1     1, 2, 3 |  /\
"""
    assert pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

    expr = meijerg([1, pi/7], [2, pi, 5], [], [], z**2)
    ucode_str = \
u"""\
        ⎛   π          │   ⎞\n\
╭─╮0, 2 ⎜1, ─  2, π, 5 │  2⎟\n\
│╶┐     ⎜   7          │ z ⎟\n\
╰─╯5, 0 ⎜              │   ⎟\n\
        ⎝              │   ⎠\
"""
    ascii_str = \
"""\
        /   pi           |   \\\n\
 __0, 2 |1, --  2, pi, 5 |  2|\n\
/__     |   7            | z |\n\
\_|5, 0 |                |   |\n\
        \\                |   /\
"""
    assert pretty(expr) == ascii_str
    assert upretty(expr) == ucode_str

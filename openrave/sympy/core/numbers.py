from core import C
from sympify import converter, sympify, _sympify, SympifyError
from basic import Basic
from singleton import S, Singleton
from expr import Expr, AtomicExpr
from decorators import _sympifyit, deprecated
from cache import cacheit, clear_cache
import sympy.mpmath as mpmath
import sympy.mpmath.libmp as mlib
from sympy.mpmath.libmp import mpf_pow, mpf_pi, mpf_e, phi_fixed
from sympy.mpmath.ctx_mp import mpnumeric

import decimal


rnd = mlib.round_nearest


# TODO: we should use the warnings module
_errdict = {"divide": False}
def seterr(divide=False):
    """
    Should sympy raise an exception on 0/0 or return a nan?

    divide == True .... raise an exception
    divide == False ... return nan
    """
    if _errdict["divide"] != divide:
        clear_cache()
        _errdict["divide"] = divide

# (a,b) -> gcd(a,b)
_gcdcache = {}

# TODO caching with decorator, but not to degrade performance
def igcd(a, b):
    """Computes positive, integer greatest common divisor of two numbers.

       The algorithm is based on the well known Euclid's algorithm. To
       improve speed, igcd() has its own caching mechanism implemented.
    """
    try:
        return _gcdcache[(a,b)]
    except KeyError:
        if a and b:
            if b < 0:
                b = -b

            while b:
                a, b = b, a % b
        else:
            a = abs(a or b)

        _gcdcache[(a,b)] = a
        return a

def ilcm(a, b):
    """Computes integer least common multiple of two numbers. """
    if a == 0 and b == 0:
        return 0
    else:
        return a * b // igcd(a, b)

def igcdex(a, b):
    """Returns x, y, g such that g = x*a + y*b = gcd(a, b).

       >>> from sympy.core.numbers import igcdex
       >>> igcdex(2, 3)
       (-1, 1, 1)
       >>> igcdex(10, 12)
       (-1, 1, 2)

       >>> x, y, g = igcdex(100, 2004)
       >>> x, y, g
       (-20, 1, 4)
       >>> x*100 + y*2004
       4

    """
    if (not a) and (not b):
        return (0, 1, 0)

    if not a:
        return (0, b//abs(b), abs(b))
    if not b:
        return (a//abs(a), 0, abs(a))

    if a < 0:
        a, x_sign = -a, -1
    else:
        x_sign = 1

    if b < 0:
        b, y_sign = -b, -1
    else:
        y_sign = 1

    x, y, r, s = 1, 0, 0, 1

    while b:
        (c, q) = (a % b, a // b)
        (a, b, r, s, x, y) = (b, c, x-q*r, y-q*s, r, s)

    return (x*x_sign, y*y_sign, a)

class Number(AtomicExpr):
    """
    Represents any kind of number in sympy.

    Floating point numbers are represented by the Float class.
    Integer numbers (of any size), together with rational numbers (again, there
    is no limit on their size) are represented by the Rational class.

    If you want to represent, for example, ``1+sqrt(2)``, then you need to do::

      Rational(1) + sqrt(Rational(2))
    """
    is_commutative = True
    is_comparable = True
    is_bounded = True
    is_finite = True
    is_number = True

    __slots__ = []

    # Used to make max(x._prec, y._prec) return x._prec when only x is a float
    _prec = -1

    is_Number = True

    def __new__(cls, *obj):
        if len(obj)==1:
            obj=obj[0]
        if isinstance(obj, (int, long)):
            return Integer(obj)
        if isinstance(obj, tuple) and len(obj) == 2:
            return Rational(*obj)
        if isinstance(obj, (float, mpmath.mpf, decimal.Decimal)):
            return Float(obj)
        if isinstance(obj, str):
            val = sympify(obj)
            if isinstance(val, Number):
                return val
            else:
                raise ValueError('String "%s" does not denote a Number'%obj)
        if isinstance(obj, Number):
            return obj
        raise TypeError("expected str|int|long|float|Decimal|Number object but got %r" % (obj))

    def _as_mpf_val(self, prec):
        """Evaluation of mpf tuple accurate to at least prec bits."""
        raise NotImplementedError('%s needs ._as_mpf_val() method' % \
            (self.__class__.__name__))

    def _eval_evalf(self, prec):
        return Float._new(self._as_mpf_val(prec), prec)

    def _as_mpf_op(self, prec):
        prec = max(prec, self._prec)
        return self._as_mpf_val(prec), prec

    def __float__(self):
        return mlib.to_float(self._as_mpf_val(53))

    def _eval_conjugate(self):
        return self

    def _eval_order(self, *symbols):
        # Order(5, x, y) -> Order(1,x,y)
        return C.Order(S.One, *symbols)

    @classmethod
    def class_key(cls):
        return 1, 0, 'Number'

    def sort_key(self, order=None):
        return self.class_key(), (0, ()), (), self


    def __eq__(self, other):
        raise NotImplementedError('%s needs .__eq__() method' % (self.__class__.__name__))
    def __ne__(self, other):
        raise NotImplementedError('%s needs .__ne__() method' % (self.__class__.__name__))
    def __lt__(self, other):
        raise NotImplementedError('%s needs .__lt__() method' % (self.__class__.__name__))
    def __le__(self, other):
        raise NotImplementedError('%s needs .__le__() method' % (self.__class__.__name__))

    def __gt__(self, other):
        return _sympify(other).__lt__(self)
    def __ge__(self, other):
        return _sympify(other).__le__(self)

    def __hash__(self):
        return super(Number, self).__hash__()

    @property
    def is_number(self):
        return True

    def as_coeff_mul(self, *deps):
        # a -> c * t
        if self.is_Rational:
            return self, tuple()
        elif self.is_negative:
            return S.NegativeOne, (-self,)
        return S.One, (self,)

    def as_coeff_add(self, *deps):
        # a -> c + t
        if self.is_Rational:
            return self, tuple()
        return S.Zero, (self,)

    def gcd(self, other):
        """Compute greatest common divisor of input arguments. """
        _ = _sympify(other)
        return S.One

    def lcm(self, other):
        """Compute least common multiple of input arguments. """
        other = _sympify(other)
        return self*other

    def cofactors(self, other):
        """Compute GCD and cofactors of input arguments. """
        other = _sympify(other)
        return S.One, self, other

    def as_coeff_Mul(self):
        """Efficiently extract the coefficient of a product. """
        return self, S.One

class Float(Number):
    """
    Represents a floating point number. It is capable of representing
    arbitrary-precision floating-point numbers

    **Usage**

    ::

      Float(3.5)
      3.5 # (the 3.5 was converted from a python float)
      Float("3.0000000000000005")

    >>> from sympy import Float
    >>> Float((1,3,0,2)) # mpmath tuple: (-1)**1 * 3 * 2**0; 3 has 2 bits
    -3.00000000000000

    **Notes**

    - Float(x) with x being a Python int/long will return Integer(x)
    """
    is_real = True
    is_irrational = False
    is_integer = False

    __slots__ = ['_mpf_', '_prec']

    # mpz can't be pickled
    def __getnewargs__(self):
        return (mlib.to_pickable(self._mpf_),)

    def __getstate__(self):
        d = Expr.__getstate__(self).copy()
        del d["_mpf_"]
        return mlib.to_pickable(self._mpf_), d

    def __setstate__(self, state):
        _mpf_, d = state
        _mpf_ = mlib.from_pickable(_mpf_)
        self._mpf_ = _mpf_
        Expr.__setstate__(self, d)

    is_Float = True

    def floor(self):
        return C.Integer(int(mlib.to_int(mlib.mpf_floor(self._mpf_, self._prec))))

    def ceiling(self):
        return C.Integer(int(mlib.to_int(mlib.mpf_ceil(self._mpf_, self._prec))))

    @property
    def num(self):
        return mpmath.mpf(self._mpf_)

    def _as_mpf_val(self, prec):
        return self._mpf_

    def _as_mpf_op(self, prec):
        return self._mpf_, max(prec, self._prec)

    def __new__(cls, num, prec=15):
        prec = mlib.libmpf.dps_to_prec(prec)
        if isinstance(num, (int, long)):
            return Integer(num)
        if isinstance(num, (str, decimal.Decimal)):
            _mpf_ = mlib.from_str(str(num), prec, rnd)
        elif isinstance(num, tuple) and len(num) == 4:
            if type(num[1]) is str:
                # it's a hexadecimal (coming from a pickled object)
                # assume that it is in standard form
                num = list(num)
                num[1] = long(num[1], 16)
                _mpf_ = tuple(num)
            else:
                _mpf_ = mpmath.mpf(
                    S.NegativeOne ** num[0] * num[1] * 2 ** num[2])._mpf_
        else:
            _mpf_ = mpmath.mpf(num)._mpf_
        if not num:
            return C.Zero()
        obj = Expr.__new__(cls)
        obj._mpf_ = _mpf_
        obj._prec = prec
        return obj

    @classmethod
    def _new(cls, _mpf_, _prec):
        if _mpf_ == mlib.fzero:
            return S.Zero
        obj = Expr.__new__(cls)
        obj._mpf_ = _mpf_
        obj._prec = _prec
        return obj

    def _hashable_content(self):
        return (self._mpf_, self._prec)

    def _eval_is_positive(self):
        return self.num > 0

    def _eval_is_negative(self):
        return self.num < 0

    def __neg__(self):
        return Float._new(mlib.mpf_neg(self._mpf_), self._prec)

    @_sympifyit('other', NotImplemented)
    def __mul__(self, other):
        if isinstance(other, Number):
            rhs, prec = other._as_mpf_op(self._prec)
            return Float._new(mlib.mpf_mul(self._mpf_, rhs, prec, rnd), prec)
        return Number.__mul__(self, other)

    @_sympifyit('other', NotImplemented)
    def __mod__(self, other):
        if isinstance(other, Number):
            rhs, prec = other._as_mpf_op(self._prec)
            return Float._new(mlib.mpf_mod(self._mpf_, rhs, prec, rnd), prec)
        return Number.__mod__(self, other)

    @_sympifyit('other', NotImplemented)
    def __rmod__(self, other):
        if isinstance(other, Number):
            rhs, prec = other._as_mpf_op(self._prec)
            return Float._new(mlib.mpf_mod(rhs, self._mpf_, prec, rnd), prec)
        return Number.__rmod__(self, other)

    @_sympifyit('other', NotImplemented)
    def __add__(self, other):
        if (other is S.NaN) or (self is NaN):
            return S.NaN
        if isinstance(other, Number):
            rhs, prec = other._as_mpf_op(self._prec)
            return Float._new(mlib.mpf_add(self._mpf_, rhs, prec, rnd), prec)
        return Number.__add__(self, other)

    def _eval_power(self, e):
        """
        e is symbolic object but not equal to 0, 1

        (-p) ** r -> exp(r * log(-p)) -> exp(r * (log(p) + I*Pi)) ->
                  -> p ** r * (sin(Pi*r) + cos(Pi*r) * I)
        """
        if isinstance(e, Number):
            if isinstance(e, Integer):
                prec = self._prec
                return Float._new(mlib.mpf_pow_int(self._mpf_, e.p, prec, rnd), prec)
            e, prec = e._as_mpf_op(self._prec)
            b = self._mpf_
            try:
                y = mpf_pow(b, e, prec, rnd)
                return Float._new(y, prec)
            except mlib.ComplexResult:
                re, im = mlib.mpc_pow((b, mlib.fzero), (e, mlib.fzero), prec, rnd)
                return Float._new(re, prec) + Float._new(im, prec) * S.ImaginaryUnit

    def __abs__(self):
        return Float._new(mlib.mpf_abs(self._mpf_), self._prec)

    def __int__(self):
        return int(mlib.to_int(self._mpf_))

    def __eq__(self, other):
        try:
            other = _sympify(other)
        except SympifyError:
            return False    # sympy != other  -->  not ==
        if isinstance(other, NumberSymbol):
            if other.is_irrational: return False
            return other.__eq__(self)
        if isinstance(other, FunctionClass): #cos as opposed to cos(x)
            return False
        if isinstance(other, Number):
            return bool(mlib.mpf_eq(self._mpf_, other._as_mpf_val(self._prec)))
        return False    # Float != non-Number

    def __ne__(self, other):
        try:
            other = _sympify(other)
        except SympifyError:
            return True     # sympy != other
        if isinstance(other, NumberSymbol):
            if other.is_irrational: return True
            return other.__ne__(self)
        if isinstance(other, FunctionClass): #cos as opposed to cos(x)
            return True
        if isinstance(other, Number):
            return bool(not mlib.mpf_eq(self._mpf_, other._as_mpf_val(self._prec)))
        return True     # Float != non-Number

    def __lt__(self, other):
        try:
            other = _sympify(other)
        except SympifyError:
            return False    # sympy > other
        if isinstance(other, NumberSymbol):
            return other.__ge__(self)
        if other.is_comparable: other = other.evalf()
        if isinstance(other, Number):
            return bool(mlib.mpf_lt(self._mpf_, other._as_mpf_val(self._prec)))
        return Expr.__lt__(self, other)

    def __le__(self, other):
        try:
            other = _sympify(other)
        except SympifyError:
            return False    # sympy > other  -->  ! <=
        if isinstance(other, NumberSymbol):
            return other.__gt__(self)
        if other.is_comparable: other = other.evalf()
        if isinstance(other, Number):
            return bool(mlib.mpf_le(self._mpf_, other._as_mpf_val(self._prec)))
        return Expr.__le__(self, other)

    def __hash__(self):
        return super(Float, self).__hash__()

    def epsilon_eq(self, other, epsilon="10e-16"):
        return abs(self - other) < Float(epsilon)

    def _sage_(self):
        import sage.all as sage
        return sage.RealNumber(str(self))

# Add sympify converters
converter[float] = converter[decimal.Decimal] = Float

# this is here to work nicely in Sage
RealNumber = Float

@deprecated
def Real(*args, **kwargs):  # pragma: no cover
    """Deprecated alias for the Float constructor."""
    return Float(*args, **kwargs)

class Rational(Number):
    """Represents integers and rational numbers (p/q) of any size.

    **Examples**

    >>> from sympy import Rational
    >>> from sympy.abc import x, y
    >>> Rational(3)
    3
    >>> Rational(1,2)
    1/2
    >>> Rational(1.5)
    1

    Rational can also accept strings that are valid literals for reals:

    >>> Rational("1.23")
    123/100
    >>> Rational('1e-2')
    1/100
    >>> Rational(".1")
    1/10

    Parsing needs for any other type of string for which a Rational is desired
    can be handled with the rational=True option in sympify() which produces
    rationals from strings like '.[3]' (=1/3) and '3/10' (=3/10).

    **Low-level**

    Access numerator and denominator as .p and .q:

    >>> r = Rational(3,4)
    >>> r
    3/4
    >>> r.p
    3
    >>> r.q
    4

    Note that p and q return integers (not sympy Integers) so some care
    is needed when using them in expressions:

    >>> r.p/r.q
    0

    """
    is_real = True
    is_integer = False
    is_rational = True

    __slots__ = ['p', 'q']

    is_Rational = True

    @cacheit
    def __new__(cls, p, q=None):
        if q is None:
            if isinstance(p, Rational):
               return p
            if isinstance(p, basestring):
                try:
                    # we might have a Float
                    neg_pow, digits, expt = decimal.Decimal(p).as_tuple()
                    p = [1, -1][neg_pow] * int("".join(str(x) for x in digits))
                    if expt > 0:
                        # TODO: this branch needs a test
                        return Rational(p*Pow(10, expt), 1)
                    return Rational(p, Pow(10, -expt))
                except decimal.InvalidOperation:
                    import re
                    f = re.match('^([-+]?[0-9]+)/([0-9]+)$', p.replace(' ',''))
                    if f:
                        n, d = f.groups()
                        return Rational(int(n), int(d))
                    raise ValueError('invalid literal: %s' % p)
            elif not isinstance(p, Basic):
                return Rational(S(p))
            q = S.One
        if isinstance(q, Rational):
            p *= q.q
            q = q.p
        if isinstance(p, Rational):
            q *= p.q
            p = p.p
        p = int(p)
        q = int(q)
        if q == 0:
            if p == 0:
                if _errdict["divide"]:
                    raise ValueError("Indeterminate 0/0")
                else:
                    return S.NaN
            if p < 0:
                return S.NegativeInfinity
            return S.Infinity
        if q < 0:
            q = -q
            p = -p
        n = igcd(abs(p), q)
        if n > 1:
            p //= n
            q //= n
        if q == 1:
            return Integer(p)
        if p == 1 and q == 2:
            return S.Half
        obj = Expr.__new__(cls)
        obj.p = p
        obj.q = q
        #obj._args = (p, q)
        return obj

    def limit_denominator(self, max_denominator=1000000):
        """Closest Rational to self with denominator at most max_denominator.

        >>> from sympy import Rational
        >>> Rational('3.141592653589793').limit_denominator(10)
        22/7
        >>> Rational('3.141592653589793').limit_denominator(100)
        311/99

        """
        # Algorithm notes: For any real number x, define a *best upper
        # approximation* to x to be a rational number p/q such that:
        #
        #   (1) p/q >= x, and
        #   (2) if p/q > r/s >= x then s > q, for any rational r/s.
        #
        # Define *best lower approximation* similarly.  Then it can be
        # proved that a rational number is a best upper or lower
        # approximation to x if, and only if, it is a convergent or
        # semiconvergent of the (unique shortest) continued fraction
        # associated to x.
        #
        # To find a best rational approximation with denominator <= M,
        # we find the best upper and lower approximations with
        # denominator <= M and take whichever of these is closer to x.
        # In the event of a tie, the bound with smaller denominator is
        # chosen.  If both denominators are equal (which can happen
        # only when max_denominator == 1 and self is midway between
        # two integers) the lower bound---i.e., the floor of self, is
        # taken.

        if max_denominator < 1:
            raise ValueError("max_denominator should be at least 1")
        if self.q <= max_denominator:
            return self

        p0, q0, p1, q1 = 0, 1, 1, 0
        n, d = self.p, self.q
        while True:
            a = n//d
            q2 = q0+a*q1
            if q2 > max_denominator:
                break
            p0, q0, p1, q1 = p1, q1, p0+a*p1, q2
            n, d = d, n-a*d

        k = (max_denominator-q0)//q1
        bound1 = Rational(p0+k*p1, q0+k*q1)
        bound2 = Rational(p1, q1)
        if abs(bound2 - self) <= abs(bound1-self):
            return bound2
        else:
            return bound1

    def __getnewargs__(self):
        return (self.p, self.q)

    def _hashable_content(self):
        return (self.p, self.q)

    def _eval_is_positive(self):
        return self.p > 0

    def _eval_is_zero(self):
        return self.p == 0

    def __neg__(self):
        return Rational(-self.p, self.q)

    @_sympifyit('other', NotImplemented)
    def __mul__(self, other):
        if (other is S.NaN) or (self is S.NaN):
            return S.NaN
        if isinstance(other, Float):
            return other * self
        if isinstance(other, Rational):
            return Rational(self.p * other.p, self.q * other.q)
        return Number.__mul__(self, other)

    @_sympifyit('other', NotImplemented)
    def __mod__(self, other):
        if isinstance(other, Rational):
            n = (self.p*other.q) // (other.p*self.q)
            return Rational(self.p*other.q - n*other.p*self.q, self.q*other.q)
        if isinstance(other, Float):
            return self.evalf() % other
        return Number.__mod__(self, other)

    @_sympifyit('other', NotImplemented)
    def __rmod__(self, other):
        if isinstance(other, Rational):
            return Rational.__mod__(other, self)
        if isinstance(other, Float):
            return other % self.evalf()
        return Number.__rmod__(self, other)

    # TODO reorder
    @_sympifyit('other', NotImplemented)
    def __add__(self, other):
        if (other is S.NaN) or (self is S.NaN):
            return S.NaN
        if isinstance(other, Float):
            return other + self
        if isinstance(other, Rational):
            if self.is_unbounded:
                if other.is_bounded:
                    return self
                elif self==other:
                    return self
            else:
                if other.is_unbounded:
                    return other
            return Rational(self.p * other.q + self.q * other.p, self.q * other.q)
        return Number.__add__(self, other)

    def _eval_power(b, e):
        if (e is S.NaN): return S.NaN
        if isinstance(e, Number):
            if isinstance(e, Float):
                return b._eval_evalf(e._prec) ** e
            if e.is_negative:
                # (3/4)**-2 -> (4/3)**2
                ne = -e
                if (ne is S.One):
                    return Rational(b.q, b.p)
                if b < 0:
                    if e.q != 1:
                        return -(S.NegativeOne) ** ((e.p % e.q) / S(e.q)) * Rational(b.q, -b.p) ** ne
                    else:
                        return S.NegativeOne ** ne * Rational(b.q, -b.p) ** ne
                else:
                    return Rational(b.q, b.p) ** ne
            if (e is S.Infinity):
                if b.p > b.q:
                    # (3/2)**oo -> oo
                    return S.Infinity
                if b.p < -b.q:
                    # (-3/2)**oo -> oo + I*oo
                    return S.Infinity + S.Infinity * S.ImaginaryUnit
                return S.Zero
            if isinstance(e, Integer):
                # (4/3)**2 -> 4**2 / 3**2
                return Rational(b.p ** e.p, b.q ** e.p)
            if isinstance(e, Rational):
                if b.p != 1:
                    # (4/3)**(5/6) -> 4**(5/6) * 3**(-5/6)
                    return Integer(b.p) ** e * Integer(b.q) ** (-e)
                if b >= 0:
                    return Integer(b.q)**Rational(e.p * (e.q-1), e.q) / ( Integer(b.q) ** Integer(e.p))
                else:
                    return (-1)**e * (-b)**e

        c, t = b.as_coeff_mul()
        if e.is_even and isinstance(c, Number) and c < 0:
            return (-c * Mul(*t)) ** e

        return

    def _as_mpf_val(self, prec):
        return mlib.from_rational(self.p, self.q, prec, rnd)

    def _mpmath_(self, prec, rnd):
        return mpmath.make_mpf(mlib.from_rational(self.p, self.q, prec, rnd))

    def __abs__(self):
        return Rational(abs(self.p), self.q)

    def __int__(self):
        return int(float(self.p)/self.q)

    def __eq__(self, other):
        try:
            other = _sympify(other)
        except SympifyError:
            return False    # sympy != other  -->  not ==
        if isinstance(other, NumberSymbol):
            if other.is_irrational: return False
            return other.__eq__(self)
        if isinstance(other, FunctionClass): #cos as opposed to cos(x)
            return False
        if other.is_comparable and not isinstance(other, Rational):
            other = other.evalf()
        if isinstance(other, Number):
            if isinstance(other, Float):
                return bool(mlib.mpf_eq(self._as_mpf_val(other._prec), other._mpf_))
            return bool(self.p==other.p and self.q==other.q)

        return False    # Rational != non-Number

    def __ne__(self, other):
        try:
            other = _sympify(other)
        except SympifyError:
            return True     # sympy != other
        if isinstance(other, NumberSymbol):
            if other.is_irrational: return True
            return other.__ne__(self)
        if isinstance(other, FunctionClass): #cos as opposed to cos(x)
            return True
        if other.is_comparable and not isinstance(other, Rational):
            other = other.evalf()
        if isinstance(other, Number):
            if isinstance(other, Float):
                return bool(not mlib.mpf_eq(self._as_mpf_val(other._prec), other._mpf_))
            return bool(self.p!=other.p or self.q!=other.q)

        return True     # Rational != non-Number

    def __lt__(self, other):
        try:
            other = _sympify(other)
        except SympifyError:
            return False    # sympy > other  --> not <
        if isinstance(other, NumberSymbol):
            return other.__ge__(self)
        if other.is_comparable and not isinstance(other, Rational):
            other = other.evalf()
        if isinstance(other, Number):
            if isinstance(other, Float):
                return bool(mlib.mpf_lt(self._as_mpf_val(other._prec), other._mpf_))
            return bool(self.p * other.q < self.q * other.p)
        return Expr.__lt__(self, other)

    def __le__(self, other):
        try:
            other = _sympify(other)
        except SympifyError:
            return False    # sympy > other  -->  not <=
        if isinstance(other, NumberSymbol):
            return other.__gt__(self)
        if other.is_comparable and not isinstance(other, Rational):
            other = other.evalf()
        if isinstance(other, Number):
            if isinstance(other, Float):
                return bool(mlib.mpf_le(self._as_mpf_val(other._prec), other._mpf_))
            return bool(self.p * other.q <= self.q * other.p)
        return Expr.__le__(self, other)

    def __hash__(self):
        return super(Rational, self).__hash__()

    def factors(self, limit=None, use_trial=True,
                                  use_rho=False,
                                  use_pm1=False,
                                  verbose=False):
        """A wrapper to factorint which return factors of self that are
        smaller than limit (or cheap to compute). Special methods of
        factoring are disabled by default so that only trial division is used.
        """
        from sympy.ntheory import factorint

        f = factorint(self.p, limit=limit,
                              use_trial=use_trial,
                              use_rho=use_rho,
                              use_pm1=use_pm1,
                              verbose=verbose).copy()
        for p, e in factorint(self.q, limit=limit,
                              use_trial=use_trial,
                              use_rho=use_rho,
                              use_pm1=use_pm1,
                              verbose=verbose).items():
            try: f[p] += -e
            except KeyError: f[p] = -e

        if len(f)>1 and 1 in f: del f[1]
        return f

    def gcd(self, other):
        """Compute greatest common divisor of input arguments. """
        if type(other) in (int, long):
            p = igcd(self.p, other)

            if self.is_Integer:
                return Integer(p)
            else:
                return Rational(p, self.q)
        else:
            other = _sympify(other)

            if other.is_Rational:
                p = igcd(self.p, other.p)

                if other.is_Integer:
                    if self.is_Integer:
                        return Integer(p)
                    else:
                        return Rational(p, self.q)
                else:
                    if self.is_Integer:
                        return Rational(p, other.q)
                    else:
                        return Rational(p, ilcm(self.q, other.q))
            elif other.is_Number:
                return S.One
            else:
                raise TypeError("expected an integer or rational, got %s" % other)

    def lcm(self, other):
        """Compute least common multiple of input arguments. """
        if type(other) in (int, long):
            return Integer(ilcm(self.p, other))
        else:
            other = _sympify(other)

            if other.is_Rational:
                p = ilcm(self.p, other.p)

                if self.is_Integer or other.is_Integer:
                    return Integer(p)
                else:
                    return Rational(p, igcd(self.q, other.q))
            elif other.is_Number:
                return self*other
            else:
                raise TypeError("expected an integer or rational, got %s" % other)

    def cofactors(self, other):
        """Compute GCD and cofactors of input arguments. """
        other = _sympify(other)
        gcd = self.gcd(other)

        if gcd is S.One:
            return gcd, self, other
        else:
            return gcd, self/gcd, other/gcd

    def as_numer_denom(self):
        return Integer(self.p), Integer(self.q)

    def _sage_(self):
        import sage.all as sage
        return sage.Integer(self.p)/sage.Integer(self.q)

# int -> Integer
_intcache = {}


# TODO move this tracing facility to  sympy/core/trace.py  ?
def _intcache_printinfo():
    ints = sorted(_intcache.keys())
    nhit = _intcache_hits
    nmiss= _intcache_misses

    if nhit == 0 and nmiss == 0:
        print
        print 'Integer cache statistic was not collected'
        return

    miss_ratio = float(nmiss) / (nhit+nmiss)

    print
    print 'Integer cache statistic'
    print '-----------------------'
    print
    print '#items: %i' % len(ints)
    print
    print ' #hit   #miss               #total'
    print
    print '%5i   %5i (%7.5f %%)   %5i'    % (nhit, nmiss, miss_ratio*100, nhit+nmiss)
    print
    print ints

_intcache_hits   = 0
_intcache_misses = 0

def int_trace(f):
    import os
    if os.getenv('SYMPY_TRACE_INT', 'no').lower() != 'yes':
        return f

    def Integer_tracer(cls, i):
        global _intcache_hits, _intcache_misses

        try:
            _intcache_hits += 1
            return _intcache[i]
        except KeyError:
            _intcache_hits   -= 1
            _intcache_misses += 1

            return f(cls, i)


    # also we want to hook our _intcache_printinfo into sys.atexit
    import atexit
    atexit.register(_intcache_printinfo)

    return Integer_tracer




class Integer(Rational):

    q = 1
    is_integer = True

    is_Integer = True

    __slots__ = ['p']

    def _as_mpf_val(self, prec):
        return mlib.from_int(self.p)

    def _mpmath_(self, prec, rnd):
        return mpmath.make_mpf(self._as_mpf_val(prec))

    # TODO caching with decorator, but not to degrade performance
    @int_trace
    def __new__(cls, i):
        ival = int(i)

        try:
            return _intcache[ival]
        except KeyError:
            # We only work with well-behaved integer types. This converts, for
            # example, numpy.int32 instances.
            if ival == 0: obj = S.Zero
            elif ival == 1: obj = S.One
            elif ival == -1: obj = S.NegativeOne
            else:
                obj = Expr.__new__(cls)
                obj.p = ival

            _intcache[ival] = obj
            return obj

    def __getnewargs__(self):
        return (self.p,)

    # Arithmetic operations are here for efficiency
    def __int__(self):
        return self.p

    def __neg__(self):
        return Integer(-self.p)

    def __abs__(self):
        if self.p >= 0:
            return self
        else:
            return Integer(-self.p)

    def __divmod__(self, other):
        return divmod(self.p, other.p)

    # TODO make it decorator + bytecodehacks?
    def __add__(a, b):
        if isinstance(b, (int, long)):
            return Integer(a.p + b)
        elif isinstance(b, Integer):
            return Integer(a.p + b.p)
        return Rational.__add__(a, b)   # a,b -not- b,a

    def __radd__(a, b):
        if isinstance(b, (int, long)):
            return Integer(b + a.p)
        elif isinstance(b, Integer):
            return Integer(b.p + a.p)
        return Rational.__add__(a, b)

    def __sub__(a, b):
        if isinstance(b, (int, long)):
            return Integer(a.p - b)
        elif isinstance(b, Integer):
            return Integer(a.p - b.p)
        return Rational.__sub__(a, b)

    def __rsub__(a, b):
        if isinstance(b, (int, long)):
            return Integer(b - a.p)
        elif isinstance(b, Integer):
            return Integer(b.p - a.p)
        return Rational.__rsub__(a, b)

    def __mul__(a, b):
        if isinstance(b, (int, long)):
            return Integer(a.p * b)
        elif isinstance(b, Integer):
            return Integer(a.p * b.p)
        return Rational.__mul__(a, b)

    def __rmul__(a, b):
        if isinstance(b, (int, long)):
            return Integer(b * a.p)
        elif isinstance(b, Integer):
            return Integer(b.p * a.p)
        return Rational.__mul__(a, b)

    def __mod__(a, b):
        if isinstance(b, (int, long)):
            return Integer(a.p % b)
        elif isinstance(b, Integer):
            return Integer(a.p % b.p)
        return Rational.__mod__(a, b)

    def __rmod__(a, b):
        if isinstance(b, (int, long)):
            return Integer(b % a.p)
        elif isinstance(b, Integer):
            return Integer(b.p % a.p)
        return Rational.__rmod__(a, b)

    def __eq__(a, b):
        if isinstance(b, (int, long)):
            return (a.p == b)
        elif isinstance(b, Integer):
            return (a.p == b.p)
        return Rational.__eq__(a, b)

    def __ne__(a, b):
        if isinstance(b, (int, long)):
            return (a.p != b)
        elif isinstance(b, Integer):
            return (a.p != b.p)
        return Rational.__ne__(a, b)

    def __gt__(a, b):
        if isinstance(b, (int, long)):
            return (a.p >  b)
        elif isinstance(b, Integer):
            return (a.p >  b.p)
        return Rational.__gt__(a, b)

    def __lt__(a, b):
        if isinstance(b, (int, long)):
            return (a.p <  b)
        elif isinstance(b, Integer):
            return (a.p <  b.p)
        return Rational.__lt__(a, b)

    def __ge__(a, b):
        if isinstance(b, (int, long)):
            return (a.p >= b)
        elif isinstance(b, Integer):
            return (a.p >= b.p)
        return Rational.__ge__(a, b)

    def __le__(a, b):
        if isinstance(b, (int, long)):
            return (a.p <= b)
        elif isinstance(b, Integer):
            return (a.p <= b.p)
        return Rational.__le__(a, b)

    def __hash__(self):
        return super(Integer, self).__hash__()

    def __index__(self):
        return self.p

    ########################################

    def _eval_is_odd(self):
        return bool(self.p % 2)

    def _eval_power(b, e):
        """
        Tries to do some simplifications on b ** e, where b is
        an instance of Integer

        Returns None if no further simplifications can be done

        When exponent is a fraction (so we have for example a square root),
        we try to find a simpler representation by factoring the argument
        up to factors of 2**15, e.g.

          - 4**Rational(1,2) becomes 2
          - (-4)**Rational(1,2) becomes 2*I
          - (2**(3+7)*3**(6+7))**Rational(1,7) becomes 6*18**(3/7)

        Further simplification would require a special call to factorint on
        the argument which is not done here for sake of speed.

        """
        from sympy import perfect_power

        if e is S.NaN:
            return S.NaN
        if b is S.One:
            return S.One
        if b is S.NegativeOne:
            return
        if e is S.Infinity:
            if b > S.One:
                return S.Infinity
            if b is S.NegativeOne:
                return S.NaN
            # cases for 0 and 1 are done in their respective classes
            return S.Infinity + S.ImaginaryUnit * S.Infinity
        if not isinstance(e, Number):
            # simplify when exp is even
            # (-2) ** k --> 2 ** k
            c, t = b.as_coeff_mul()
            if e.is_even and isinstance(c, Number) and c < 0:
                return (-c*Mul(*t))**e
        if not isinstance(e, Rational):
            return
        if e is S.Half and b < 0:
            # we extract I for this special case since everyone is doing so
            return S.ImaginaryUnit*Pow(-b, e)
        if e < 0:
            # invert base and change sign on exponent
            ne = -e
            if b < 0:
                if e.q != 1:
                    return -(S.NegativeOne)**((e.p % e.q) /
                                             S(e.q)) * Rational(1, -b)**ne
                else:
                    return (S.NegativeOne)**ne*Rational(1, -b)**ne
            else:
                return Rational(1, b)**ne
        # see if base is a perfect root, sqrt(4) --> 2
        b_pos = int(abs(b))
        x, xexact = integer_nthroot(b_pos, e.q)
        if xexact:
            # if it's a perfect root we've finished
            result = Integer(x ** abs(e.p))
            if b < 0:
                result *= (-1)**e
            return result

        # The following is an algorithm where we collect perfect roots
        # from the factors of base.

        # if it's not an nth root, it still might be a perfect power
        p = perfect_power(b_pos)
        if p is not False:
            dict = {p[0]: p[1]}
        else:
            dict = Integer(b_pos).factors(limit=2**15)

        # now process the dict of factors
        if b.is_negative:
            dict[-1] = 1
        out_int = 1 # integer part
        out_rad = 1 # extracted radicals
        sqr_int = 1
        sqr_gcd = 0
        sqr_dict = {}
        for prime, exponent in dict.items():
            exponent *= e.p
            # remove multiples of e.q, e.g. (2**12)**(1/10) -> 2*(2**2)**(1/10)
            div_e, div_m = divmod(exponent, e.q)
            if div_e > 0:
                out_int *= prime**div_e
            if div_m > 0:
                # see if the reduced exponent shares a gcd with e.q
                # (2**2)**(1/10) -> 2**(1/5)
                g = igcd(div_m, e.q)
                if g != 1:
                    out_rad *= Pow(prime, Rational(div_m//g, e.q//g))
                else:
                    sqr_dict[prime] = div_m
        # identify gcd of remaining powers
        for p, ex in sqr_dict.iteritems():
            if sqr_gcd == 0:
                sqr_gcd = ex
            else:
                sqr_gcd = igcd(sqr_gcd, ex)
                if sqr_gcd == 1:
                    break
        for k, v in sqr_dict.iteritems():
            sqr_int *= k**(v//sqr_gcd)
        if sqr_int == b and out_int == 1 and out_rad == 1:
            result = None
        else:
            result = out_int*out_rad*Pow(sqr_int, Rational(sqr_gcd, e.q))
        return result

    def _eval_is_prime(self):
        if self.p < 0:
            return False

    def as_numer_denom(self):
        return self, S.One

    def __floordiv__(self, other):
        return Integer(self.p // Integer(other).p)

    def __rfloordiv__(self, other):
        return Integer(Integer(other).p // self.p)

    def factorial(a):
        """Compute factorial of `a`. """
        from sympy.functions.combinatorial.factorials import factorial
        return Integer(factorial(int(a)))

    def isqrt(a):
        """Compute integer square root of `a`. """
        return Integer(mlib.isqrt(int(a)))

    def half_gcdex(a, b):
        """Half Extended Euclidean Algorithm. """
        s, _, h = a.gcdex(b)
        return s, h

    def gcdex(a, b):
        """Extended Euclidean Algorithm. """
        if isinstance(b, (int, long)):
            return tuple(map(Integer, igcdex(int(a), b)))
        else:
            b = _sympify(b)

            if b.is_Integer:
                return tuple(map(Integer, igcdex(int(a), int(b))))
            else:
                raise ValueError("expected an integer, got %s" % b)

    def invert(a, b):
        """Invert `a` modulo `b`, if possible. """
        if isinstance(b, (int, long)):
            a = int(a)
        else:
            b = _sympify(b)

            if b.is_Integer:
                a, b = int(a), int(b)
            else:
                raise ValueError("expected an integer, got %s" % b)

        s, _, h = igcdex(a, b)

        if h == 1:
            return Integer(s % b)
        else:
            raise ZeroDivisionError("zero divisor")

# Add sympify converters
converter[int] = converter[long] = Integer

class RationalConstant(Rational):
    """
    Abstract base class for rationals with specific behaviors

    Derived classes must define class attributes p and q and should probably all
    be singletons.
    """
    __slots__ = []

    def __new__(cls):
        return AtomicExpr.__new__(cls)

class IntegerConstant(Integer):
    __slots__ = []

    def __new__(cls):
        return AtomicExpr.__new__(cls)


class Zero(IntegerConstant):
    __metaclass__ = Singleton

    p = 0
    q = 1
    is_positive = False
    is_negative = False
    is_finite = False
    is_zero = True
    is_prime = False
    is_composite = False

    __slots__ = []

    @staticmethod
    def __abs__():
        return S.Zero

    @staticmethod
    def __neg__():
        return S.Zero

    def _eval_power(b, e):
        if e.is_negative:
            return S.Infinity
        if e.is_positive:
            return b
        d = e.evalf()
        if isinstance(d, Number):
            if d.is_negative:
                return S.Infinity
            return b
        coeff, terms = e.as_coeff_mul()
        if coeff.is_negative:
            return S.Infinity ** Mul(*terms)
        if coeff is not S.One:
            return b ** Mul(*terms)

    def _eval_order(self, *symbols):
        # Order(0,x) -> 0
        return self

    def __nonzero__(self):
        return False

class One(IntegerConstant):
    __metaclass__ = Singleton

    p = 1
    q = 1

    is_prime = True

    __slots__ = []

    def _eval_evalf(self, prec):
        return self

    @staticmethod
    def __abs__():
        return S.One

    @staticmethod
    def __neg__():
        return S.NegativeOne

    def _eval_order(self, *symbols):
        return

    @staticmethod
    def factors():
        return {1: 1}

class NegativeOne(IntegerConstant):
    __metaclass__ = Singleton

    p = -1
    q = 1

    __slots__ = []

    def _eval_evalf(self, prec):
        return self

    @staticmethod
    def __abs__():
        return S.One

    @staticmethod
    def __neg__():
        return S.One

    def _eval_power(b, e):
        if e.is_odd: return S.NegativeOne
        if e.is_even: return S.One
        if isinstance(e, Number):
            if isinstance(e, Float):
                return Float(-1.0) ** e
            if e is S.NaN:
                return S.NaN
            if e is S.Infinity  or  e is S.NegativeInfinity:
                return S.NaN
            if e is S.Half:
                return S.ImaginaryUnit
            if isinstance(e, Rational):
                if e.q == 2:
                    return S.ImaginaryUnit ** Integer(e.p)
                q = Float(e).floor()
                if q:
                    q = Integer(q)
                    return b ** q * b ** (e - q)
        return

class Half(RationalConstant):
    __metaclass__ = Singleton

    p = 1
    q = 2

    __slots__ = []

    @staticmethod
    def __abs__():
        return S.Half


class Infinity(RationalConstant):
    __metaclass__ = Singleton

    p = 1
    q = 0

    __slots__ = []

    is_commutative = True
    is_positive = True
    is_bounded = False
    is_finite   = False
    is_infinitesimal = False
    is_integer  = None
    is_rational = None
    is_odd = None

    @staticmethod
    def __abs__():
        return S.Infinity

    @staticmethod
    def __neg__():
        return S.NegativeInfinity

    def _eval_power(b, e):
        """
        e is symbolic object but not equal to 0, 1

        oo ** nan -> nan
        oo ** (-p) -> 0, p is number, oo
        """
        if e.is_positive:
            return S.Infinity
        if e.is_negative:
            return S.Zero
        if isinstance(e, Number):
            if e is S.NaN:
                return S.NaN
        d = e.evalf()
        if isinstance(d, Number):
            return b ** d
        return

    def _as_mpf_val(self, prec):
        return mlib.finf

    def _sage_(self):
        import sage.all as sage
        return sage.oo

    def __gt__(a, b):
        if b is S.Infinity:
            return False
        return True

    def __lt__(a, b):
        return False

    def __ge__(a, b):
        return True

    def __le__(a, b):
        if b is S.Infinity:
            return True
        return False

    def __mod__(self, other):
        return S.NaN

    __rmod__ = __mod__
oo = S.Infinity

class NegativeInfinity(RationalConstant):
    __metaclass__ = Singleton

    p = -1
    q = 0

    __slots__ = []

    is_commutative = True
    is_real = True
    is_positive = False
    is_bounded = False
    is_finite = False
    is_infinitesimal = False
    is_integer  = None
    is_rational = None

    @staticmethod
    def __abs__():
        return S.Infinity

    @staticmethod
    def __neg__():
        return S.Infinity

    def _eval_power(b, e):
        """
        e is symbolic object but not equal to 0, 1

        (-oo) ** nan -> nan
        (-oo) ** oo  -> nan
        (-oo) ** (-oo) -> nan
        (-oo) ** e -> oo, e is positive even integer
        (-oo) ** o -> -oo, o is positive odd integer

        """
        if isinstance(e, Number):
            if (e is S.NaN)  or  (e is S.Infinity)  or  (e is S.NegativeInfinity):
                return S.NaN
            if isinstance(e, Integer):
                if e.is_positive:
                    if e.is_odd:
                        return S.NegativeInfinity
                    return S.Infinity
            return S.NegativeOne**e * S.Infinity ** e
        return

    def _as_mpf_val(self, prec):
        return mlib.fninf

    def _sage_(self):
        import sage.all as sage
        return -(sage.oo)

    def __gt__(a, b):
        return False

    def __lt__(a, b):
        if b is S.NegativeInfinity:
            return False
        return True

    def __ge__(a, b):
        if b is S.NegativeInfinity:
            return True
        return False

    def __le__(a, b):
        return True

class NaN(RationalConstant):
    __metaclass__ = Singleton

    p = 0
    q = 0

    is_commutative = True
    is_real = None
    is_rational = None
    is_integer  = None
    is_comparable = False
    is_finite   = None
    is_bounded = None
    #is_unbounded = False
    is_zero     = None
    is_prime    = None
    is_positive = None

    __slots__ = []

    def _as_mpf_val(self, prec):
        return mlib.fnan

    def _eval_power(b, e):
        if e is S.Zero:
            return S.One
        return b

    def _sage_(self):
        import sage.all as sage
        return sage.NaN
nan = S.NaN

class ComplexInfinity(AtomicExpr):
    __metaclass__ = Singleton
    is_commutative = True
    is_comparable = None
    is_bounded = False
    is_real = None
    is_number = True

    __slots__ = []

    def __new__(cls):
        return AtomicExpr.__new__(cls)

    @staticmethod
    def __abs__():
        return S.Infinity

    @staticmethod
    def __neg__():
        return S.ComplexInfinity

    def _eval_power(b, e):
        if e is S.ComplexInfinity:
            return S.NaN

        if isinstance(e, Number):
            if e is S.Zero:
                return S.NaN
            else:
                if e.is_positive:
                    return S.ComplexInfinity
                else:
                    return S.Zero
zoo = S.ComplexInfinity

class NumberSymbol(AtomicExpr):
    __metaclass__ = Singleton

    is_commutative = True
    is_comparable = True
    is_bounded = True
    is_finite = True
    is_number = True

    __slots__ = []

    is_NumberSymbol = True

    def __new__(cls):
        return AtomicExpr.__new__(cls)

    def approximation(self, number_cls):
        """ Return an interval with number_cls endpoints
        that contains the value of NumberSymbol.
        If not implemented, then return None.
        """

    def _eval_evalf(self, prec):
        return Float._new(self._as_mpf_val(prec), prec)

    def __eq__(self, other):
        try:
            other = _sympify(other)
        except SympifyError:
            return False    # sympy != other  -->  not ==
        if self is other:
            return True
        if isinstance(other, Number) and self.is_irrational:
            return False

        return False    # NumberSymbol != non-(Number|self)

    def __ne__(self, other):
        try:
            other = _sympify(other)
        except SympifyError:
            return True     # sympy != other
        if self is other:
            return False
        if isinstance(other, Number) and self.is_irrational:
            return True

        return True     # NumberSymbol != non(Number|self)

    def __lt__(self, other):
        try:
            other = _sympify(other)
        except SympifyError:
            return False    # sympy > other  --> not <
        if self is other:
            return False
        if isinstance(other, Number):
            approx = self.approximation_interval(other.__class__)
            if approx is not None:
                l,u = approx
                if other < l:
                    return False
                if other > u:
                    return True
            return self.evalf()<other
        if other.is_comparable:
            other = other.evalf()
            return self.evalf()<other
        return Expr.__lt__(self, other)

    def __le__(self, other):
        try:
            other = _sympify(other)
        except SympifyError:
            return False    # sympy > other  --> not <=
        if self is other:
            return True
        if other.is_comparable:
            other = other.evalf()
        if isinstance(other, Number):
            return self.evalf()<=other
        return Expr.__le__(self, other)

    def __gt__(self, other):
        return (-self) < (-other)

    def __ge__(self, other):
        return (-self) <= (-other)

    def __int__(self):
        return int(self.evalf(0))

    def __hash__(self):
        return super(NumberSymbol, self).__hash__()


class Exp1(NumberSymbol):
    __metaclass__ = Singleton

    is_real = True
    is_positive = True
    is_negative = False # XXX Forces is_negative/is_nonnegative
    is_irrational = True

    __slots__ = []

    @staticmethod
    def __abs__():
        return S.Exp1

    def _as_mpf_val(self, prec):
        return mpf_e(prec)

    def approximation_interval(self, number_cls):
        if issubclass(number_cls,Integer):
            return (Integer(2),Integer(3))
        elif issubclass(number_cls,Rational):
            pass

    def _eval_power(self, exp):
        return C.exp(exp)

    def _sage_(self):
        import sage.all as sage
        return sage.e
E = S.Exp1

class Pi(NumberSymbol):
    __metaclass__ = Singleton


    is_real = True
    is_positive = True
    is_negative = False
    is_irrational = True

    __slots__ = []

    @staticmethod
    def __abs__():
        return S.Pi

    def _as_mpf_val(self, prec):
        return mpf_pi(prec)

    def approximation_interval(self, number_cls):
        if issubclass(number_cls, Integer):
            return (Integer(3), Integer(4))
        elif issubclass(number_cls, Rational):
            return (Rational(223,71), Rational(22,7))

    def _sage_(self):
        import sage.all as sage
        return sage.pi
pi = S.Pi

class GoldenRatio(NumberSymbol):
    __metaclass__ = Singleton

    is_real = True
    is_positive = True
    is_negative = False
    is_irrational = True

    __slots__ = []

    def _as_mpf_val(self, prec):
        return mlib.from_man_exp(phi_fixed(prec+10), -prec-10)

    def _eval_expand_func(self, deep=True, **hints):
        return S.Half + S.Half*S.Sqrt(5)

    def approximation_interval(self, number_cls):
        if issubclass(number_cls, Integer):
            return (S.One, Rational(2))
        elif issubclass(number_cls, Rational):
            pass

    def _sage_(self):
        import sage.all as sage
        return sage.golden_ratio

class EulerGamma(NumberSymbol):
    __metaclass__ = Singleton

    is_real = True
    is_positive = True
    is_negative = False
    is_irrational = None

    __slots__ = []

    def _as_mpf_val(self, prec):
        return mlib.from_man_exp(mlib.libhyper.euler_fixed(
            prec+10), -prec-10)

    def approximation_interval(self, number_cls):
        if issubclass(number_cls, Integer):
            return (S.Zero, S.One)
        elif issubclass(number_cls, Rational):
            return (S.Half, Rational(3, 5))

    def _sage_(self):
        import sage.all as sage
        return sage.euler_gamma

class Catalan(NumberSymbol):
    __metaclass__ = Singleton

    is_real = True
    is_positive = True
    is_negative = False
    is_irrational = None

    __slots__ = []

    def _as_mpf_val(self, prec):
        return mlib.from_man_exp(mlib.catalan_fixed(prec+10), -prec-10)

    def approximation_interval(self, number_cls):
        if issubclass(number_cls, Integer):
            return (S.Zero, S.One)
        elif issubclass(number_cls, Rational):
            return (Rational(9, 10), S.One)

    def _sage_(self):
        import sage.all as sage
        return sage.catalan

class ImaginaryUnit(AtomicExpr):
    __metaclass__ = Singleton

    is_commutative = True
    is_imaginary = True
    is_bounded = True
    is_finite = True
    is_number = True

    __slots__ = []

    @staticmethod
    def __abs__():
        return S.One

    def _eval_evalf(self, prec):
        return self

    def _eval_conjugate(self):
        return -S.ImaginaryUnit

    def _eval_power(b, e):
        """
        b is I = sqrt(-1)
        e is symbolic object but not equal to 0, 1

        I ** r -> (-1)**(r/2) -> exp(r/2 * Pi * I) -> sin(Pi*r/2) + cos(Pi*r/2) * I, r is decimal
        I ** 0 mod 4 -> 1
        I ** 1 mod 4 -> I
        I ** 2 mod 4 -> -1
        I ** 3 mod 4 -> -I
        """


        if isinstance(e, Number):
            if isinstance(e, Integer):
                ei = e.p % 4
                if ei == 0:
                    return S.One
                if ei == 1:
                    return S.ImaginaryUnit
                if ei == 2:
                    return -S.One
                return -S.ImaginaryUnit
            return (S.NegativeOne) ** (e * S.Half)
        return

    def as_base_exp(self):
        return S.NegativeOne, S.Half

    def _sage_(self):
        import sage.all as sage
        return sage.I

I = S.ImaginaryUnit

try:
    # fractions is only available for python 2.6+
    import fractions

    def sympify_fractions(f):
        return Rational(f.numerator, f.denominator)

    converter[fractions.Fraction] = sympify_fractions
except ImportError:
    pass

try:
    import gmpy

    def sympify_mpz(x):
        return Integer(long(x))

    def sympify_mpq(x):
        return Rational(long(x.numer()), long(x.denom()))

    converter[type(gmpy.mpz(1))] = sympify_mpz
    converter[type(gmpy.mpq(1, 2))] = sympify_mpq
except ImportError:
    pass

def sympify_mpmath(x):
    return Expr._from_mpmath(x, x.context.prec)

converter[mpnumeric] = sympify_mpmath

def sympify_complex(a):
    real, imag = map(sympify, (a.real, a.imag))
    return real + S.ImaginaryUnit * imag

converter[complex] = sympify_complex

_intcache[0] = S.Zero
_intcache[1] = S.One
_intcache[-1]= S.NegativeOne

from function import FunctionClass
from power import Pow, integer_nthroot
from mul import Mul
Mul.identity = One()
from add import Add
Add.identity = Zero()

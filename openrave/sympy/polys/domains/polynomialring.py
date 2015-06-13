"""Implementation of :class:`PolynomialRing` class. """

from sympy.polys.domains.ring import Ring
from sympy.polys.domains.compositedomain import CompositeDomain
from sympy.polys.domains.characteristiczero import CharacteristicZero

from sympy.polys.polyclasses import DMP
from sympy.polys.polyerrors import GeneratorsNeeded, PolynomialError, CoercionFailed
from sympy.polys.polyutils import dict_from_basic, basic_from_dict, _dict_reorder

class PolynomialRing(Ring, CharacteristicZero, CompositeDomain):
    """A class for representing multivariate polynomial rings. """

    dtype        = DMP
    is_Poly      = True

    has_assoc_Ring         = True
    has_assoc_Field        = True

    def __init__(self, dom, *gens):
        if not gens:
            raise GeneratorsNeeded("generators not specified")

        lev = len(gens) - 1

        self.zero = self.dtype.zero(lev, dom)
        self.one  = self.dtype.one(lev, dom)

        self.dom  = dom
        self.gens = gens

    def __str__(self):
        return str(self.dom) + '[' + ','.join(map(str, self.gens)) + ']'

    def __hash__(self):
        return hash((self.__class__.__name__, self.dtype, self.dom, self.gens))

    def __call__(self, a):
        """Construct an element of `self` domain from `a`. """
        return DMP(a, self.dom, len(self.gens)-1)

    def __eq__(self, other):
        """Returns `True` if two domains are equivalent. """
        return self.dtype == other.dtype and self.dom == other.dom and self.gens == other.gens

    def __ne__(self, other):
        """Returns `False` if two domains are equivalent. """
        return not self.__eq__(other)

    def to_sympy(self, a):
        """Convert `a` to a SymPy object. """
        return basic_from_dict(a.to_sympy_dict(), *self.gens)

    def from_sympy(self, a):
        """Convert SymPy's expression to `dtype`. """
        try:
            rep, _ = dict_from_basic(a, gens=self.gens)
        except PolynomialError:
            raise CoercionFailed("can't convert %s to type %s" % (a, self))

        for k, v in rep.iteritems():
            rep[k] = self.dom.from_sympy(v)

        return self(rep)

    def from_ZZ_python(K1, a, K0):
        """Convert a Python `int` object to `dtype`. """
        return K1(K1.dom.convert(a, K0))

    def from_QQ_python(K1, a, K0):
        """Convert a Python `Fraction` object to `dtype`. """
        return K1(K1.dom.convert(a, K0))

    def from_ZZ_sympy(K1, a, K0):
        """Convert a SymPy `Integer` object to `dtype`. """
        return K1(K1.dom.convert(a, K0))

    def from_QQ_sympy(K1, a, K0):
        """Convert a SymPy `Rational` object to `dtype`. """
        return K1(K1.dom.convert(a, K0))

    def from_ZZ_gmpy(K1, a, K0):
        """Convert a GMPY `mpz` object to `dtype`. """
        return K1(K1.dom.convert(a, K0))

    def from_QQ_gmpy(K1, a, K0):
        """Convert a GMPY `mpq` object to `dtype`. """
        return K1(K1.dom.convert(a, K0))

    def from_RR_sympy(K1, a, K0):
        """Convert a SymPy `Real` object to `dtype`. """
        return K1(K1.dom.convert(a, K0))

    def from_RR_mpmath(K1, a, K0):
        """Convert a mpmath `mpf` object to `dtype`. """
        return K1(K1.dom.convert(a, K0))

    def from_AlgebraicField(K1, a, K0):
        """Convert a `ANP` object to `dtype`. """
        if K1.dom == K0:
            return K1(a)

    def from_PolynomialRing(K1, a, K0):
        """Convert a `DMP` object to `dtype`. """
        if K1.gens == K0.gens:
            if K1.dom == K0.dom:
                return a
            else:
                return a.convert(K1.dom)
        else:
            monoms, coeffs = _dict_reorder(a.to_dict(), K0.gens, K1.gens)

            if K1.dom != K0.dom:
                coeffs = [ K1.dom.convert(c, K0.dom) for c in coeffs ]

            return K1(dict(zip(monoms, coeffs)))

    def from_FractionField(K1, a, K0):
        """
        Convert a ``DMF`` object to ``DMP``.

        **Examples**

        >>> from sympy.polys.polyclasses import DMP, DMF
        >>> from sympy.polys.domains import ZZ
        >>> from sympy.abc import x

        >>> f = DMF(([ZZ(1), ZZ(1)], [ZZ(1)]), ZZ)
        >>> K = ZZ.frac_field(x)

        >>> F = ZZ[x].from_FractionField(f, K)

        >>> F == DMP([ZZ(1), ZZ(1)], ZZ)
        True
        >>> type(F)
        <class 'sympy.polys.polyclasses.DMP'>

        """
        if a.denom().is_one:
            return K1.from_PolynomialRing(a.numer(), K0)

    def get_field(self):
        """Returns a field associated with `self`. """
        from sympy.polys.domains import FractionField
        return FractionField(self.dom, *self.gens)

    def poly_ring(self, *gens):
        """Returns a polynomial ring, i.e. `K[X]`. """
        raise NotImplementedError('nested domains not allowed')

    def frac_field(self, *gens):
        """Returns a fraction field, i.e. `K(X)`. """
        raise NotImplementedError('nested domains not allowed')

    def is_positive(self, a):
        """Returns True if `LC(a)` is positive. """
        return self.dom.is_positive(a.LC())

    def is_negative(self, a):
        """Returns True if `LC(a)` is negative. """
        return self.dom.is_negative(a.LC())

    def is_nonpositive(self, a):
        """Returns True if `LC(a)` is non-positive. """
        return self.dom.is_nonpositive(a.LC())

    def is_nonnegative(self, a):
        """Returns True if `LC(a)` is non-negative. """
        return self.dom.is_nonnegative(a.LC())

    def gcdex(self, a, b):
        """Extended GCD of `a` and `b`. """
        return a.gcdex(b)

    def gcd(self, a, b):
        """Returns GCD of `a` and `b`. """
        return a.gcd(b)

    def lcm(self, a, b):
        """Returns LCM of `a` and `b`. """
        return a.lcm(b)

    def factorial(self, a):
        """Returns factorial of `a`. """
        return self.dtype(self.dom.factorial(a))

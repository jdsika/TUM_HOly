from expr import Expr
from evalf import EvalfMixin
from sympify import _sympify

def Rel(a, b, op):
    """
    A handy wrapper around the Relational class.
    Rel(a,b, op)

    Example:
    >>> from sympy import Rel
    >>> from sympy.abc import x, y
    >>> Rel(y, x+x**2, '==')
    y == x**2 + x

    """
    return Relational(a,b,op)

def Eq(a, b=0):
    """
    A handy wrapper around the Relational class.
    Eq(a,b)

    Example:
    >>> from sympy import Eq
    >>> from sympy.abc import x, y
    >>> Eq(y, x+x**2)
    y == x**2 + x

    """
    return Relational(a,b,'==')

def Ne(a, b):
    """
    A handy wrapper around the Relational class.
    Ne(a,b)

    Example:
    >>> from sympy import Ne
    >>> from sympy.abc import x, y
    >>> Ne(y, x+x**2)
    y != x**2 + x

    """
    return Relational(a,b,'!=')

def Lt(a, b):
    """
    A handy wrapper around the Relational class.
    Lt(a,b)

    Example:
    >>> from sympy import Lt
    >>> from sympy.abc import x, y
    >>> Lt(y, x+x**2)
    y < x**2 + x

    """
    return Relational(a,b,'<')

def Le(a, b):
    """
    A handy wrapper around the Relational class.
    Le(a,b)

    Example:
    >>> from sympy import Le
    >>> from sympy.abc import x, y
    >>> Le(y, x+x**2)
    y <= x**2 + x

    """
    return Relational(a,b,'<=')

def Gt(a, b):
    """
    A handy wrapper around the Relational class.
    Gt(a,b)

    Example:
    >>> from sympy import Gt
    >>> from sympy.abc import x, y
    >>> Gt(y, x+x**2)
    x**2 + x < y

    """
    return Relational(a,b,'>')

def Ge(a, b):
    """
    A handy wrapper around the Relational class.
    Ge(a,b)

    Example:
    >>> from sympy import Ge
    >>> from sympy.abc import x, y
    >>> Ge(y, x+x**2)
    x**2 + x <= y

    """
    return Relational(a,b,'>=')

class Relational(Expr, EvalfMixin):

    __slots__ = []

    is_Relational = True

    @staticmethod
    def get_relational_class(rop):
        if rop is None or rop in ['==','eq']: return Equality, False
        if rop in ['!=','<>','ne']: return Unequality, False
        if rop in ['<','lt']: return StrictInequality, False
        if rop in ['>','gt']: return StrictInequality, True
        if rop in ['<=','le']: return Inequality, False
        if rop in ['>=','ge']: return Inequality, True
        raise ValueError("Invalid relational operator symbol: %r" % (rop))

    def __new__(cls, lhs, rhs, rop=None, **assumptions):
        lhs = _sympify(lhs)
        rhs = _sympify(rhs)
        if cls is not Relational:
            rop_cls = cls
        else:
            rop_cls, swap = Relational.get_relational_class(rop)
            if swap: lhs, rhs = rhs, lhs
        if lhs.is_real and lhs.is_number and rhs.is_real and rhs.is_number:
            # Just becase something is a number, doesn't mean you can evalf it.
            Nlhs = lhs.evalf()
            if Nlhs.is_Number:
                # S.Zero.evalf() returns S.Zero, so test Number instead of Float
                Nrhs = rhs.evalf()
                if Nrhs.is_Number:
                    return rop_cls._eval_relation(Nlhs, Nrhs)

        obj = Expr.__new__(rop_cls, lhs, rhs, **assumptions)
        return obj

    @property
    def lhs(self):
        return self._args[0]

    @property
    def rhs(self):
        return self._args[1]

    def _eval_subs(self, old, new):
        if self == old:
            return new
        return self.__class__(self.lhs._eval_subs(old, new), self.rhs._eval_subs(old, new))

    def _eval_evalf(self, prec):
        return self.func(*[s._evalf(prec) for s in self.args])

class Equality(Relational):

    rel_op = '=='

    __slots__ = []

    is_Equality = True

    @classmethod
    def _eval_relation(cls, lhs, rhs):
        return lhs == rhs

    def __nonzero__(self):
        return self.lhs.compare(self.rhs)==0

class Unequality(Relational):

    rel_op = '!='

    __slots__ = []

    @classmethod
    def _eval_relation(cls, lhs, rhs):
        return lhs != rhs

    def __nonzero__(self):
        return self.lhs.compare(self.rhs)!=0

class StrictInequality(Relational):

    rel_op = '<'

    __slots__ = []

    @classmethod
    def _eval_relation(cls, lhs, rhs):
        return lhs < rhs

    def __nonzero__(self):
        return self.lhs.compare(self.rhs)==-1

class Inequality(Relational):

    rel_op = '<='

    __slots__ = []

    @classmethod
    def _eval_relation(cls, lhs, rhs):
        return lhs <= rhs

    def __nonzero__(self):
        return self.lhs.compare(self.rhs)<=0

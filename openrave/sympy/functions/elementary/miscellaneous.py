from sympy.core import S, C, sympify
from sympy.core.basic import Basic
from sympy.core.containers import Tuple
from sympy.core.operations import LatticeOp, ShortCircuit
from sympy.core.function import Application, Lambda
from sympy.core.expr import Expr
from sympy.core.singleton import Singleton

class IdentityFunction(Lambda):
    """The identity function

    >>> from sympy import Id, Symbol
    >>> x = Symbol('x')
    >>> Id(x)
    x
    """
    __metaclass__ = Singleton
    __slots__ = []
    nargs = 1
    def __new__(cls):
        x = C.Dummy('x')
        #construct "by hand" to avoid infinite loop
        return Expr.__new__(cls, Tuple(x), x)
Id = S.IdentityFunction

###############################################################################
############################# SQUARE ROOT FUNCTION ############################
###############################################################################

def sqrt(arg):
    # arg = sympify(arg) is handled by Pow
    return C.Pow(arg, S.Half)

###############################################################################
############################# MINIMUM and MAXIMUM #############################
###############################################################################

class MinMaxBase(LatticeOp):
    def __new__(cls, *args, **assumptions):
        if not args:
            raise ValueError("The Max/Min functions must have arguments.")

        args = (sympify(arg) for arg in args)

        # first standard filter, for cls.zero and cls.identity
        # also reshape Max(a, Max(b, c)) to Max(a, b, c)
        try:
            _args = frozenset(cls._new_args_filter(args))
        except ShortCircuit:
            return cls.zero

        # second filter
        # variant I: remove ones which can be removed
        # args = cls._collapse_arguments(set(_args), **assumptions)

        # variant II: find local zeros
        args = cls._find_localzeros(set(_args), **assumptions)

        _args = frozenset(args)

        if not _args:
            return cls.identity
        elif len(_args) == 1:
            return set(_args).pop()
        else:
            # base creation
            obj = Expr.__new__(cls, _args, **assumptions)
            obj._argset = _args
            return obj

    @classmethod
    def _new_args_filter(cls, arg_sequence):
        """
        Generator filtering args.

        first standard filter, for cls.zero and cls.identity.
        Also reshape Max(a, Max(b, c)) to Max(a, b, c),
        and check arguments for comparability
        """
        for arg in arg_sequence:

            # pre-filter, checking comparability of arguments
            if (arg.is_real == False) or (arg is S.ComplexInfinity):
                raise ValueError("The argument '%s' is not comparable." % arg)

            if arg == cls.zero:
                raise ShortCircuit(arg)
            elif arg == cls.identity:
                continue
            elif arg.func == cls:
                for x in arg.iter_basic_args():
                    yield x
            else:
                yield arg

    @classmethod
    def _find_localzeros(cls, values, **options):
        """
        Sequentially allocate values to localzeros.

        If value is greter than all of the localzeros, then it is new localzero
        and it is apending to them.

        if value is greter than one of the localzeros,
        then update localzero's set.
        """
        localzeros = set()
        for v in values:
            is_newzero = True
            for z in localzeros:
                if id(v) == id(z):
                    is_newzero = False
                elif cls._is_connected(v, z):
                    is_newzero = False
                    if cls._is_asneeded(v, z):
                        localzeros.remove(z)
                        localzeros.update([v])
                        break
            if is_newzero:
                localzeros.update([v])
        return localzeros

    @classmethod
    def _is_connected(cls, x, y):
        """
        Check if x and y are connected somehow.
        """
        if (x == y) or isinstance(x > y, bool) or isinstance(x < y, bool):
            return True
        if x.is_Number and y.is_Number:
            return True
        return False

    @classmethod
    def _is_asneeded(cls, x, y):
        """
        Check if x and y satisfy relation condition.

        The relation condition for Max function is x > y,
        for Min function is x < y. They are defined in children Max and Min
        classes through the method _rel(cls, x, y)
        """
        if (x == y):
            return False
        if x.is_Number and y.is_Number:
            if cls._rel(x, y):
                return True
        xy = cls._rel(x, y)
        if isinstance(xy, bool):
            if xy:
                return True
            return False
        yx = cls._rel_inversed(x, y)
        if isinstance(yx, bool):
            if yx:
                return False # never occurs?
            return True
        return False

class Max(MinMaxBase, Application, Basic):
    """
    Return, if possible, the maximum value of the list.

    When number of arguments is equal one, then
    return this argument.

    When number of arguments is equal two, then
    return, if possible, the value from (a, b) that is >= the other.

    In common case, when the length of list greater than 2, the task
    is more complicated. Return only the arguments, which are greater
    than others, if it is possible to determine directional relation.

    If is not possible to determine such a relation, return a partially
    evaluated result.

    Assumptions are used to make the decision too.

    Also, only comparable arguments are permitted.

    Example
    -------

    >>> from sympy import Max, Symbol, oo
    >>> from sympy.abc import x, y
    >>> p = Symbol('p', positive=True)
    >>> n = Symbol('n', negative=True)

    >>> Max(x, -2)                  #doctest: +SKIP
    Max(x, -2)

    >>> Max(x, -2).subs(x, 3)
    3

    >>> Max(p, -2)
    p

    >>> Max(x, y)                   #doctest: +SKIP
    Max(x, y)

    >>> Max(x, y) == Max(y, x)
    True

    >>> Max(x, Max(y, z))           #doctest: +SKIP
    Max(x, y, z)

    >>> Max(n, 8, p, 7, -oo)        #doctest: +SKIP
    Max(8, p)

    >>> Max (1, x, oo)
    oo

    Algorithm
    ---------
    The task can be considered as searching of supremums in the
    directed complete partial orders [1]_.

    The source values are sequentially allocated by the isolated subsets
    in which supremums are searched and result as Max arguments.

    If the resulted supremum is single, then it is returned.

    The isolated subsets are the sets of values which are only the comparable
    with each other in the current set. E.g. natural numbers are comparable with
    each other, but not comparable with the `x` symbol. Another example: the
    symbol `x` with negative assumption is comparable with a natural number.

    Also there are "least" elements, which are comparable with all others,
    and have a zero property (maximum or minimum for all elements). E.g. `oo`.
    In case of it the allocation operation is terminated and only this value is
    returned.

    Assumption:
       - if A > B > C then A > C
       - if A==B then B can be removed

    [1] http://en.wikipedia.org/wiki/Directed_complete_partial_order
    [2] http://en.wikipedia.org/wiki/Lattice_(order)

    See Also
    --------
    Min() : find minimum values

    """
    zero = S.Infinity
    identity = S.NegativeInfinity

    @classmethod
    def _rel(cls, x, y):
        """
        Check if x > y.
        """
        return (x > y)

    @classmethod
    def _rel_inversed(cls, x, y):
        """
        Check if x < y.
        """
        return (x < y)


class Min(MinMaxBase, Application, Basic):
    """
    Return, if possible, the minimum value of the list.

    Example
    -------

    >>> from sympy import Min, Symbol, oo
    >>> from sympy.abc import x, y
    >>> p = Symbol('p', positive=True)
    >>> n = Symbol('n', negative=True)

    >>> Min(x, -2)                  #doctest: +SKIP
    Min(x, -2)

    >>> Min(x, -2).subs(x, 3)
    -2

    >>> Min(p, -3)
    -3

    >>> Min(x, y)                   #doctest: +SKIP
    Min(x, y)

    >>> Min(n, 8, p, -7, p, oo)     #doctest: +SKIP
    Min(n, -7)

    See Also
    --------
    Max() : find maximum values
    """
    zero = S.NegativeInfinity
    identity = S.Infinity

    @classmethod
    def _rel(cls, x, y):
        """
        Check if x < y.
        """
        return (x < y)

    @classmethod
    def _rel_inversed(cls, x, y):
        """
        Check if x > y.
        """
        return (x > y)


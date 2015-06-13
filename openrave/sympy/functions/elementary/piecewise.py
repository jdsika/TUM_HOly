from sympy.core import Basic, S, Function, diff, Number, sympify
from sympy.core.relational import Equality, Relational
from sympy.logic.boolalg import Boolean
from sympy.core.sets import Set

class ExprCondPair(Function):
    """Represents an expression, condition pair."""

    def __new__(cls, *args, **assumptions):
        if isinstance(args[0], cls):
            expr = args[0].expr
            cond = args[0].cond
        elif len(args) == 2:
            expr = sympify(args[0])
            cond = sympify(args[1])
        else:
            raise TypeError("args must be a (expr, cond) pair")
        return Basic.__new__(cls, expr, cond, **assumptions)

    @property
    def expr(self):
        return self.args[0]

    @property
    def cond(self):
        return self.args[1]

    @property
    def is_commutative(self):
        return self.expr.is_commutative

    @property
    def free_symbols(self):
        # Overload Basic.free_symbols because self.args[1] may contain non-Basic
        result = self.expr.free_symbols
        if hasattr(self.cond, 'free_symbols'):
            result |= self.cond.free_symbols
        return result

    def __iter__(self):
        yield self.expr
        yield self.cond

class Piecewise(Function):
    """
    Represents a piecewise function.

    Usage
    =====
      Piecewise( (expr,cond), (expr,cond), ... )
        - Each argument is a 2-tuple defining a expression and condition
        - The conds are evaluated in turn returning the first that is True.
          If any of the evaluated conds are not determined explicitly False,
          e.g. x < 1, the function is returned in symbolic form.
        - If the function is evaluated at a place where all conditions are False,
          a ValueError exception will be raised.
        - Pairs where the cond is explicitly False, will be removed.

    Examples
    ========
      >>> from sympy import Piecewise, log
      >>> from sympy.abc import x
      >>> f = x**2
      >>> g = log(x)
      >>> p = Piecewise( (0, x<-1), (f, x<=1), (g, True))
      >>> p.subs(x,1)
      1
      >>> p.subs(x,5)
      log(5)

    """

    nargs = None
    is_Piecewise = True

    def __new__(cls, *args, **options):
        # (Try to) sympify args first
        newargs = []
        for ec in args:
            pair = ExprCondPair(*ec)
            cond_type = type(pair.cond)
            if not (cond_type is bool or issubclass(cond_type, Relational) or \
                    issubclass(cond_type, Number) or \
                    issubclass(cond_type, Set) or issubclass(cond_type, Boolean)):
                raise TypeError(
                    "Cond %s is of type %s, but must be a bool," \
                    " Relational, Number or Set" % (pair.cond, cond_type))
            newargs.append(pair)

        if options.pop('evaluate', True):
            r = cls.eval(*newargs)
        else:
            r = None

        if r is None:
            return Basic.__new__(cls, *newargs, **options)
        else:
            return r

    def __getnewargs__(self):
        # Convert ExprCondPair objects to tuples.
        args = []
        for expr, condition in self.args:
            args.append((expr, condition))
        return tuple(args)

    @classmethod
    def eval(cls, *args):
        # Check for situations where we can evaluate the Piecewise object.
        # 1) Hit an unevaluable cond (e.g. x<1) -> keep object
        # 2) Hit a true condition -> return that expr
        # 3) Remove false conditions, if no conditions left -> raise ValueError
        all_conds_evaled = True    # Do all conds eval to a bool?
        piecewise_again = False    # Should we pass args to Piecewise again?
        non_false_ecpairs = []
        for expr, cond in args:
            # Check here if expr is a Piecewise and collapse if one of
            # the conds in expr matches cond. This allows the collapsing
            # of Piecewise((Piecewise(x,x<0),x<0)) to Piecewise((x,x<0)).
            # This is important when using piecewise_fold to simplify
            # multiple Piecewise instances having the same conds.
            # Eventually, this code should be able to collapse Piecewise's
            # having different intervals, but this will probably require
            # using the new assumptions.
            if isinstance(expr, Piecewise):
                for e, c in expr.args:
                    # Don't collapse if cond is "True" as this leads to
                    # incorrect simplifications with nested Piecewises.
                    if c == cond and cond is not True:
                        expr = e
                        piecewise_again = True
            cond_eval = cls.__eval_cond(cond)
            if cond_eval is None:
                all_conds_evaled = False
                non_false_ecpairs.append( (expr, cond) )
            elif cond_eval:
                if all_conds_evaled:
                    return expr
                non_false_ecpairs.append( (expr, cond) )
        if len(non_false_ecpairs) != len(args) or piecewise_again:
            return Piecewise(*non_false_ecpairs)

        return None

    def doit(self, **hints):
        newargs = []
        for e, c in self.args:
            if hints.get('deep', True):
                if isinstance(e, Basic):
                    e = e.doit(**hints)
                if isinstance(c, Basic):
                    c = c.doit(**hints)
            newargs.append((e, c))
        return Piecewise(*newargs)

    def _eval_integral(self,x):
        from sympy.integrals import integrate
        return  Piecewise(*[(integrate(e, x), c) for e, c in self.args])

    def _eval_interval(self, sym, a, b):
        """Evaluates the function along the sym in a given interval ab"""
        # FIXME: Currently complex intervals are not supported.  A possible
        # replacement algorithm, discussed in issue 2128, can be found in the
        # following papers;
        #     http://portal.acm.org/citation.cfm?id=281649
        #     http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.70.4127&rep=rep1&type=pdf
        int_expr = []
        mul = 1
        if a > b:
            a, b, mul = b, a, -1
        default = None

        # Determine what intervals the expr,cond pairs affect.
        # 1) If cond is True, then log it as default
        # 1.1) Currently if cond can't be evaluated, throw NotImplentedError.
        # 2) For each inequality, if previous cond defines part of the interval
        #    update the new conds interval.
        #    -  eg x < 1, x < 3 -> [oo,1],[1,3] instead of [oo,1],[oo,3]
        # 3) Sort the intervals to make it easier to find correct exprs
        for expr, cond in self.args:
            if isinstance(cond, bool) or cond.is_Number:
                if cond:
                    default = expr
                    break
                else:
                    continue
            elif isinstance(cond, Equality):
                continue
            curr = list(cond.args)
            if cond.args[0].has(sym):
                curr[0] = S.NegativeInfinity
            elif cond.args[1].has(sym):
                curr[1] = S.Infinity
            else:
                raise NotImplementedError(\
                        "Unable handle interval evaluation of expression.")
            curr = [max(a, curr[0]), min(b, curr[1])]
            for n in xrange(len(int_expr)):
                if self.__eval_cond(curr[0] < int_expr[n][1]) and \
                        self.__eval_cond(curr[0] >= int_expr[n][0]):
                    curr[0] = int_expr[n][1]
                if self.__eval_cond(curr[1] > int_expr[n][0]) and \
                        self.__eval_cond(curr[1] <= int_expr[n][1]):
                    curr[1] = int_expr[n][0]
            if self.__eval_cond(curr[0] < curr[1]):
                int_expr.append(curr + [expr])
        int_expr.sort(key=lambda x:x[0])

        # Add holes to list of intervals if there is a default value,
        # otherwise raise a ValueError.
        holes = []
        curr_low = a
        for int_a, int_b, expr in int_expr:
            if curr_low < int_a:
                holes.append([curr_low, min(b, int_a), default])
            curr_low = int_b
            if curr_low > b:
                break
        if curr_low < b:
            holes.append([curr_low, b, default])

        if holes and default is not None:
            int_expr.extend(holes)
        elif holes and default == None:
            raise ValueError("Called interval evaluation over piecewise " \
                             "function on undefined intervals %s" % \
                             ", ".join([str((h[0], h[1])) for h in holes]))

        # Finally run through the intervals and sum the evaluation.
        ret_fun = 0
        for int_a, int_b, expr in int_expr:
            ret_fun += expr._eval_interval(sym,  max(a, int_a), min(b, int_b))
        return mul * ret_fun

    def _eval_derivative(self, s):
        return Piecewise(*[(diff(e, s), c) for e, c in self.args])

    def _eval_subs(self, old, new):
        if self == old:
            return new
        new_args = []
        for e, c in self.args:
            if isinstance(c, bool):
                new_args.append((e._eval_subs(old, new), c))
            elif isinstance(c, Set):
                # What do we do if there are more than one symbolic
                # variable. Which do we put pass to Set.contains?
                new_args.append((e._eval_subs(old, new),  c.contains(new)))
            else:
                new_args.append((e._eval_subs(old, new), c._eval_subs(old, new)))
        return Piecewise( *new_args )

    def _eval_nseries(self, x, n, logx):
        args = map(lambda ec: (ec.expr._eval_nseries(x, n, logx), ec.cond), \
                   self.args)
        return self.func(*args)

    @classmethod
    def __eval_cond(cls, cond):
        """Returns S.One if True, S.Zero if False, or None if undecidable."""
        if type(cond) == bool or cond.is_number:
            if cond:
                return S.One
            else:
                return S.Zero
        elif type(cond) == Set:
            return None
        return None

def piecewise_fold(expr):
    """
    Takes an expression containing a piecewise function and returns the
    expression in piecewise form.

    >>> from sympy import Piecewise, piecewise_fold
    >>> from sympy.abc import x
    >>> p = Piecewise((x, x < 1), (1, 1 <= x))
    >>> piecewise_fold(x*p)
    Piecewise((x**2, x < 1), (x, 1 <= x))

    """
    if not isinstance(expr, Basic) or not expr.has(Piecewise):
        return expr
    new_args = map(piecewise_fold, expr.args)
    if expr.func is ExprCondPair:
        return ExprCondPair(*new_args)
    piecewise_args = []
    for n, arg in enumerate(new_args):
        if arg.func is Piecewise:
            piecewise_args.append(n)
    if len(piecewise_args) > 0:
        n = piecewise_args[0]
        new_args = [(expr.func(*(new_args[:n] + [e] + new_args[n+1:])), c) \
                        for e, c in new_args[n].args]
        if len(piecewise_args) > 1:
            return piecewise_fold(Piecewise(*new_args))
    return Piecewise(*new_args)


from sympy import SYMPY_DEBUG

from sympy.core import (Basic, S, C, Add, Mul, Pow, Rational, Integer,
    Derivative, Wild, Symbol, sympify, expand, expand_mul, expand_func,
    Function, Equality, Dummy, Atom, count_ops)

from sympy.core.compatibility import iterable
from sympy.core.numbers import igcd
from sympy.core.function import expand_log

from sympy.utilities import flatten
from sympy.functions import gamma, exp, sqrt, log

from sympy.simplify.cse_main import cse

from sympy.polys import (Poly, together, reduced, cancel, factor,
    ComputationFailed, terms_gcd)

from sympy.core.compatibility import reduce

import sympy.mpmath as mpmath

def fraction(expr, exact=False):
    """Returns a pair with expression's numerator and denominator.
       If the given expression is not a fraction then this function
       will return the tuple (expr, 1).

       This function will not make any attempt to simplify nested
       fractions or to do any term rewriting at all.

       If only one of the numerator/denominator pair is needed then
       use numer(expr) or denom(expr) functions respectively.

       >>> from sympy import fraction, Rational, Symbol
       >>> from sympy.abc import x, y

       >>> fraction(x/y)
       (x, y)
       >>> fraction(x)
       (x, 1)

       >>> fraction(1/y**2)
       (1, y**2)

       >>> fraction(x*y/2)
       (x*y, 2)
       >>> fraction(Rational(1, 2))
       (1, 2)

       This function will also work fine with assumptions:

       >>> k = Symbol('k', negative=True)
       >>> fraction(x * y**k)
       (x, y**(-k))

       If we know nothing about sign of some exponent and 'exact'
       flag is unset, then structure this exponent's structure will
       be analyzed and pretty fraction will be returned:

       >>> from sympy import exp
       >>> fraction(2*x**(-y))
       (2, x**y)

       >>> fraction(exp(-x))
       (1, exp(x))

       >>> fraction(exp(-x), exact=True)
       (exp(-x), 1)

    """
    expr = sympify(expr)

    numer, denom = [], []

    for term in Mul.make_args(expr):
        if term.is_Pow or term.func is exp:
            b, ex = term.as_base_exp()
            if ex.is_negative:
                if ex is S.NegativeOne:
                    denom.append(b)
                else:
                    denom.append(Pow(b, -ex))
            elif not exact and ex.is_Mul:
                n, d = term.as_numer_denom()
                numer.append(n)
                denom.append(d)
            else:
                numer.append(term)
        elif term.is_Rational:
            n, d = term.as_numer_denom()
            numer.append(n)
            denom.append(d)
        else:
            numer.append(term)

    return Mul(*numer), Mul(*denom)

def numer(expr):
    return fraction(expr)[0]

def denom(expr):
    return fraction(expr)[1]

def fraction_expand(expr):
    a, b = fraction(expr)
    return a.expand() / b.expand()

def numer_expand(expr):
    a, b = fraction(expr)
    return a.expand() / b

def denom_expand(expr):
    a, b = fraction(expr)
    return a / b.expand()

def separate(expr, deep=False, force=False):
    """A wrapper to expand(power_base=True) which separates a power
       with a base that is a Mul into a product of powers, without performing
       any other expansions, provided that assumptions about the power's base
       and exponent allow.

       deep=True (default is False) will do separations inside functions.

       force=True (default is False) will cause the expansion to ignore
       assumptions about the base and exponent. When False, the expansion will
       only happen if the base is non-negative or the exponent is an integer.

       >>> from sympy.abc import x, y, z
       >>> from sympy import separate, sin, cos, exp

       >>> (x*y)**2
       x**2*y**2

       >>> (2*x)**y
       (2*x)**y
       >>> separate(_)
       2**y*x**y

       >>> separate((x*y)**z)
       (x*y)**z
       >>> separate((x*y)**z, force=True)
       x**z*y**z
       >>> separate(sin((x*y)**z))
       sin((x*y)**z)
       >>> separate(sin((x*y)**z), deep=True, force=True)
       sin(x**z*y**z)

       >>> separate((2*sin(x))**y + (2*cos(x))**y)
       2**y*sin(x)**y + 2**y*cos(x)**y

       >>> separate((2*exp(y))**x)
       2**x*exp(x*y)

       >>> separate((2*cos(x))**y)
       2**y*cos(x)**y

       Notice that summations are left untouched. If this is not the
       desired behavior, apply 'expand' to the expression:

       >>> separate(((x+y)*z)**2)
       z**2*(x + y)**2
       >>> (((x+y)*z)**2).expand()
       x**2*z**2 + 2*x*y*z**2 + y**2*z**2

       >>> separate((2*y)**(1+z))
       2**(z + 1)*y**(z + 1)
       >>> ((2*y)**(1+z)).expand()
       2*2**z*y*y**z

    """
    return sympify(expr).expand(deep=deep, mul=False, power_exp=False,\
    power_base=True, basic=False, multinomial=False, log=False, force=force)

def collect(expr, syms, evaluate=True, exact=False):
    """
        Collect additive terms with respect to a list of symbols up
        to powers with rational exponents. By the term symbol here
        are meant arbitrary expressions, which can contain powers,
        products, sums etc. In other words symbol is a pattern
        which will be searched for in the expression's terms.

        This function will not apply any redundant expanding to the
        input expression, so user is assumed to enter expression in
        final form. This makes 'collect' more predictable as there
        is no magic behind the scenes. However it is important to
        note, that powers of products are converted to products of
        powers using 'separate' function.

        There are two possible types of output. First, if 'evaluate'
        flag is set, this function will return a single expression
        or else it will return a dictionary with separated symbols
        up to rational powers as keys and collected sub-expressions
        as values respectively.

        >>> from sympy import collect, sympify, Wild
        >>> from sympy.abc import a, b, c, x, y, z

        This function can collect symbolic coefficients in polynomial
        or rational expressions. It will manage to find all integer or
        rational powers of collection variable:

        >>> collect(a*x**2 + b*x**2 + a*x - b*x + c, x)
        c + x**2*(a + b) + x*(a - b)

        The same result can be achieved in dictionary form:

        >>> d = collect(a*x**2 + b*x**2 + a*x - b*x + c, x, evaluate=False)
        >>> d[x**2]
        a + b
        >>> d[x]
        a - b
        >>> d[sympify(1)]
        c

        You can also work with multi-variate polynomials. However
        remember that this function is greedy so it will care only
        about a single symbol at time, in specification order:

        >>> collect(x**2 + y*x**2 + x*y + y + a*y, [x, y])
        x**2*(y + 1) + x*y + y*(a + 1)

        Also more complicated expressions can be used as patterns:

        >>> from sympy import sin, log
        >>> collect(a*sin(2*x) + b*sin(2*x), sin(2*x))
        (a + b)*sin(2*x)

        >>> collect(a*x*log(x) + b*(x*log(x)), x*log(x))
        x*(a + b)*log(x)

        You can use wildcards in the pattern

        >>> w = Wild('w1')
        >>> collect(a*x**y - b*x**y, w**y)
        x**y*(a - b)

        It is also possible to work with symbolic powers, although
        it has more complicated behavior, because in this case
        power's base and symbolic part of the exponent are treated
        as a single symbol:

        >>> collect(a*x**c + b*x**c, x)
        a*x**c + b*x**c

        >>> collect(a*x**c + b*x**c, x**c)
        x**c*(a + b)

        However if you incorporate rationals to the exponents, then
        you will get well known behavior:

        >>> collect(a*x**(2*c) + b*x**(2*c), x**c)
        (a + b)*(x**2)**c

        Note also that all previously stated facts about 'collect'
        function apply to the exponential function, so you can get:

        >>> from sympy import exp
        >>> collect(a*exp(2*x) + b*exp(2*x), exp(x))
        (a + b)*exp(2*x)

        If you are interested only in collecting specific powers
        of some symbols then set 'exact' flag in arguments:

        >>> collect(a*x**7 + b*x**7, x, exact=True)
        a*x**7 + b*x**7

        >>> collect(a*x**7 + b*x**7, x**7, exact=True)
        x**7*(a + b)

        You can also apply this function to differential equations, where
        derivatives of arbitrary order can be collected.  Note that if you
        collect with respect to a function or a derivative of a function,
        all derivatives of that function will also be collected. Use
        exact=True to prevent this from happening:

        >>> from sympy import Derivative as D, collect, Function
        >>> f = Function('f') (x)

        >>> collect(a*D(f,x) + b*D(f,x), D(f,x))
        (a + b)*Derivative(f(x), x)

        >>> collect(a*D(D(f,x),x) + b*D(D(f,x),x), f)
        (a + b)*Derivative(f(x), x, x)

        >>> collect(a*D(D(f,x),x) + b*D(D(f,x),x), D(f,x), exact=True)
        a*Derivative(f(x), x, x) + b*Derivative(f(x), x, x)

        >>> collect(a*D(f,x) + b*D(f,x) + a*f + b*f, f,x)
        (a + b)*f(x) + (a + b)*Derivative(f(x), x)

        Or you can even match both derivative order and exponent at the same
        time.

        >>> collect(a*D(D(f,x),x)**2 + b*D(D(f,x),x)**2, D(f,x))
        (a + b)*Derivative(f(x), x, x)**2

        Note: arguments are expected to be in expanded form, so you might have
        to call expand() prior to calling this function.
    """
    def make_expression(terms):
        product = []

        for term, rat, sym, deriv in terms:
            if deriv is not None:
                var, order = deriv

                while order > 0:
                    term, order = Derivative(term, var), order-1

            if sym is None:
                if rat is S.One:
                    product.append(term)
                else:
                    product.append(Pow(term, rat))
            else:
                product.append(Pow(term, rat*sym))

        return Mul(*product)

    def parse_derivative(deriv):
        # scan derivatives tower in the input expression and return
        # underlying function and maximal differentiation order
        expr, sym, order = deriv.expr, deriv.variables[0], 1

        for s in deriv.variables[1:]:
            if s == sym:
                order += 1
            else:
                raise NotImplementedError('Improve MV Derivative support in collect')

        while isinstance(expr, Derivative):
            s0 = expr.variables[0]

            for s in expr.variables:
                if s != s0:
                    raise NotImplementedError('Improve MV Derivative support in collect')

            if s0 == sym:
                expr, order = expr.expr, order+len(expr.variables)
            else:
                break

        return expr, (sym, Rational(order))

    def parse_term(expr):
        """Parses expression expr and outputs tuple (sexpr, rat_expo, sym_expo, deriv)
        where:
         - sexpr is the base expression
         - rat_expo is the rational exponent that sexpr is raised to
         - sym_expo is the symbolic exponent that sexpr is raised to
         - deriv contains the derivatives the the expression

         for example, the output of x would be (x, 1, None, None)
         the output of 2**x would be (2, 1, x, None)
        """
        rat_expo, sym_expo = S.One, None
        sexpr, deriv = expr, None

        if expr.is_Pow:
            if isinstance(expr.base, Derivative):
                sexpr, deriv = parse_derivative(expr.base)
            else:
                sexpr = expr.base

            if expr.exp.is_Rational:
                rat_expo = expr.exp
            elif expr.exp.is_Mul:
                coeff, tail = expr.exp.as_coeff_mul()

                if coeff.is_Rational:
                    rat_expo, sym_expo = coeff, expr.exp._new_rawargs(*tail)
                else:
                    sym_expo = expr.exp
            else:
                sym_expo = expr.exp
        elif expr.func is C.exp:
            arg = expr.args[0]
            if arg.is_Rational:
                sexpr, rat_expo = S.Exp1, arg
            elif arg.is_Mul:
                coeff, tail = arg.as_coeff_mul()

                if coeff.is_Rational:
                    sexpr, rat_expo = C.exp(arg._new_rawargs(*tail)), coeff
        elif isinstance(expr, Derivative):
            sexpr, deriv = parse_derivative(expr)

        return sexpr, rat_expo, sym_expo, deriv

    def parse_expression(terms, pattern):
        """Parse terms searching for a pattern.
        terms is a list of tuples as returned by parse_terms;
        pattern is an expression treated as a product of factors
        """
        pattern = Mul.make_args(pattern)

        if len(terms) < len(pattern):
            # pattern is longer than  matched product
            # so no chance for positive parsing result
            return None
        else:
            pattern = [parse_term(elem) for elem in pattern]

            terms = terms[:] # need a copy
            elems, common_expo, has_deriv = [], None, False

            for elem, e_rat, e_sym, e_ord in pattern:

                if elem.is_Number:
                    # a constant is a match for everything
                    continue

                for j in range(len(terms)):
                    if terms[j] is None:
                        continue

                    term, t_rat, t_sym, t_ord = terms[j]

                    # keeping track of whether one of the terms had
                    # a derivative or not as this will require rebuilding
                    # the expression later
                    if t_ord is not None:
                        has_deriv= True

                    if (term.match(elem) is not None and \
                            (t_sym == e_sym or t_sym is not None and \
                            e_sym is not None and \
                            t_sym.match(e_sym) is not None)):
                        if exact == False:
                            # we don't have to be exact so find common exponent
                            # for both expression's term and pattern's element
                            expo = t_rat / e_rat

                            if common_expo is None:
                                # first time
                                common_expo = expo
                            else:
                                # common exponent was negotiated before so
                                # there is no chance for a pattern match unless
                                # common and current exponents are equal
                                if common_expo != expo:
                                    common_expo = 1
                        else:
                            # we ought to be exact so all fields of
                            # interest must match in every details
                            if e_rat != t_rat or e_ord != t_ord:
                                continue

                        # found common term so remove it from the expression
                        # and try to match next element in the pattern
                        elems.append(terms[j])
                        terms[j] = None

                        break

                else:
                    # pattern element not found
                    return None

            return filter(None, terms), elems, common_expo, has_deriv

    if evaluate:
        if expr.is_Mul:
            ret = 1
            for term in expr.args:
                ret *= collect(term, syms, True, exact)
            return ret
        elif expr.is_Pow:
            b = collect(expr.base, syms, True, exact)
            return Pow(b, expr.exp)

    summa = [separate(i) for i in Add.make_args(sympify(expr))]

    if hasattr(syms, '__iter__') or hasattr(syms, '__getitem__'):
        syms = [separate(s) for s in syms]
    else:
        syms = [separate(syms)]

    collected, disliked = {}, S.Zero
    for product in summa:
        terms = [parse_term(i) for i in Mul.make_args(product)]

        for symbol in syms:
            if SYMPY_DEBUG:
                print "DEBUG: parsing of expression %s with symbol %s " % (str(terms), str(symbol))

            result = parse_expression(terms, symbol)

            if SYMPY_DEBUG:
                print "DEBUG: returned %s" %  str(result)

            if result is not None:
                terms, elems, common_expo, has_deriv = result

                # when there was derivative in current pattern we
                # will need to rebuild its expression from scratch
                if not has_deriv:
                    index = 1
                    for elem in elems:
                        index *= Pow(elem[0], elem[1])
                        if elem[2] is not None:
                            index **= elem[2]
                else:
                    index = make_expression(elems)
                terms = separate(make_expression(terms))
                index = separate(index)
                if index in collected.keys():
                    collected[index] += terms
                else:
                    collected[index] = terms

                break
        else:
            # none of the patterns matched
            disliked += product

    if disliked is not S.Zero:
        collected[S.One] = disliked

    if evaluate:
        return Add(*[a*b for a, b in collected.iteritems()])
    else:
        return collected

def rcollect(expr, *vars):
    """
    Recursively collect sums in an expression.

    Example
    =======

    >>> from sympy.simplify import rcollect
    >>> from sympy.abc import x, y

    >>> expr = (x**2*y + x*y + x + y)/(x + y)

    >>> rcollect(expr, y)
    (x + y*(x**2 + x + 1))/(x + y)

    """
    if expr.is_Atom or not expr.has(*vars):
        return expr
    else:
        expr = expr.__class__(*[ rcollect(arg, *vars) for arg in expr.args ])

        if expr.is_Add:
            return collect(expr, vars)
        else:
            return expr

def separatevars(expr, symbols=[], dict=False, force=False):
    """
    Separates variables in an expression, if possible.  By
    default, it separates with respect to all symbols in an
    expression and collects constant coefficients that are
    independent of symbols.

    If dict=True then the separated terms will be returned
    in a dictionary keyed to their corresponding symbols.
    By default, all symbols in the expression will appear as
    keys; if symbols are provided, then all those symbols will
    be used as keys, and any terms in the expression containing
    other symbols or non-symbols will be returned keyed to the
    string 'coeff'.

    If force=True, then power bases will only be separated if assumptions allow.

    Note: the order of the factors is determined by Mul, so that the
    separated expressions may not necessarily be grouped together.

    Examples:
    >>> from sympy.abc import x, y, z, alpha
    >>> from sympy import separatevars, sin
    >>> separatevars((x*y)**y)
    (x*y)**y
    >>> separatevars((x*y)**y, force=True)
    x**y*y**y
    >>> separatevars(2*x**2*z*sin(y)+2*z*x**2)
    2*x**2*z*(sin(y) + 1)

    >>> separatevars(2*x+y*sin(x))
    2*x + y*sin(x)
    >>> separatevars(2*x**2*z*sin(y)+2*z*x**2, symbols=(x, y), dict=True)
    {'coeff': 2*z, x: x**2, y: sin(y) + 1}
    >>> separatevars(2*x**2*z*sin(y)+2*z*x**2, [x, y, alpha], dict=True)
    {'coeff': 2*z, alpha: 1, x: x**2, y: sin(y) + 1}

    If the expression is not really separable, or is only partially
    separable, separatevars will do the best it can to separate it.

    >>> separatevars(x+x*y-3*(x**2))
    -x*(3*x - y - 1)

    If the expression is not separable then expr is returned unchanged
    or (if dict=True) then None is returned.

    >>> eq = 2*x+y*sin(x)
    >>> separatevars(eq) == eq
    True
    >>> separatevars(2*x+y*sin(x), symbols=(x, y), dict=True) == None
    True

    """

    if dict:
        return _separatevars_dict(_separatevars(expr, force), *symbols)
    else:
        return _separatevars(expr, force)

def _separatevars(expr, force):
    # get a Pow ready for expansion
    if expr.is_Pow:
        expr = Pow(separatevars(expr.base, force=force), expr.exp)

    # First try other expansion methods
    expr = expr.expand(mul=False, multinomial=False, force=force)

    _expr = expr.expand(power_exp=False, deep=False, force=force)

    if not force:
        # factor will expand bases so we mask them off now
        pows = [p for p in _expr.atoms(Pow) if p.base.is_Mul]
        dums = [Dummy(str(i)) for i in xrange(len(pows))]
        _expr = _expr.subs(dict(zip(pows, dums)))

    _expr = factor(_expr, expand=False)

    if not force:
        # and retore them
        _expr = _expr.subs(dict(zip(dums, pows)))



    if not _expr.is_Add:
        expr = _expr

    if expr.is_Add:

        nonsepar = sympify(0)
        # Find any common coefficients to pull out
        commoncsetlist = []
        for i in expr.args:
            if i.is_Mul:
                commoncsetlist.append(set(i.args))
            else:
                commoncsetlist.append(set((i,)))
        commoncset = set(flatten(commoncsetlist))
        commonc = sympify(1)

        for i in commoncsetlist:
            commoncset = commoncset.intersection(i)
        commonc = Mul(*commoncset)

        for i in expr.args:
            coe = i.extract_multiplicatively(commonc)
            if coe == None:
                nonsepar += sympify(1)
            else:
                nonsepar += coe
        if nonsepar == 0:
            return commonc
        else:
            return commonc*nonsepar

    else:
        return expr

def _separatevars_dict(expr, *symbols):
    if symbols:
        assert all((t.is_Atom for t in symbols)), "symbols must be Atoms."

    ret = dict(((i, S.One) for i in symbols + ('coeff',)))

    for i in Mul.make_args(expr):
        expsym = i.free_symbols
        intersection = set(symbols).intersection(expsym)
        if len(intersection) > 1:
            return None
        if len(intersection) == 0:
            # There are no symbols, so it is part of the coefficient
            ret['coeff'] *= i
        else:
            ret[intersection.pop()] *= i

    return ret

def ratsimp(expr):
    """Put an expression over a common denominator, cancel and reduce.

    == Examples ==
        >>> from sympy import ratsimp
        >>> from sympy.abc import x, y
        >>> ratsimp(1/x + 1/y)
        (x + y)/(x*y)
    """

    f, g = cancel(expr).as_numer_denom()
    try:
        Q, r = reduced(f, [g], field=True, expand=False)
    except ComputationFailed:
        return f/g

    return Add(*Q) + cancel(r/g)

def trigsimp(expr, deep=False, recursive=False):
    """
    == Usage ==

    trigsimp(expr) -> reduces expression by using known trig identities

    == Notes ==

    deep:
    - Apply trigsimp inside functions

    recursive:
    - Use common subexpression elimination (cse()) and apply
    trigsimp recursively (recursively==True is quite expensive
    operation if the expression is large)

    == Examples ==
        >>> from sympy import trigsimp, sin, cos, log
        >>> from sympy.abc import x, y
        >>> e = 2*sin(x)**2 + 2*cos(x)**2
        >>> trigsimp(e)
        2
        >>> trigsimp(log(e))
        log(2*sin(x)**2 + 2*cos(x)**2)
        >>> trigsimp(log(e), deep=True)
        log(2)

    """
    sin, cos, tan, cot = C.sin, C.cos, C.tan, C.cot
    if not expr.has(sin, cos, tan, cot):
        return expr

    if recursive:
        w, g = cse(expr)
        g = trigsimp_nonrecursive(g[0])

        for sub in reversed(w):
            g = g.subs(sub[0], sub[1])
            g = trigsimp_nonrecursive(g)
        result = g
    else:
        result = trigsimp_nonrecursive(expr, deep)

    return result


def trigsimp_nonrecursive(expr, deep=False):
    """
    A nonrecursive trig simplifier, used from trigsimp.

    == Usage ==
        trigsimp_nonrecursive(expr) -> reduces expression by using known trig
                                       identities

    == Notes ==

    deep ........ apply trigsimp inside functions

    == Examples ==
        >>> from sympy import cos, sin, log
        >>> from sympy.simplify.simplify import trigsimp, trigsimp_nonrecursive
        >>> from sympy.abc import x, y
        >>> e = 2*sin(x)**2 + 2*cos(x)**2
        >>> trigsimp(e)
        2
        >>> trigsimp_nonrecursive(log(e))
        log(2*sin(x)**2 + 2*cos(x)**2)
        >>> trigsimp_nonrecursive(log(e), deep=True)
        log(2)

    """
    sin, cos, tan, cot = C.sin, C.cos, C.tan, C.cot

    if expr.is_Function:
        if deep:
            return expr.func(trigsimp_nonrecursive(expr.args[0], deep))
    elif expr.is_Mul:
        # do some simplifications like sin/cos -> tan:
        a,b,c = map(Wild, 'abc')
        matchers = (
                (a*sin(b)**c/cos(b)**c, a*tan(b)**c),
                (a*tan(b)**c*cos(b)**c, a*sin(b)**c),
                (a*cot(b)**c*sin(b)**c, a*cos(b)**c),
                (a*tan(b)**c/sin(b)**c, a/cos(b)**c),
                (a*cot(b)**c/cos(b)**c, a/sin(b)**c),
        )
        for pattern, simp in matchers:
            res = expr.match(pattern)
            if res is not None:
                # if c is missing or zero, do nothing:
                if (not c in res) or res[c] == 0:
                    continue
                # if "a" contains any of sin("b"), cos("b"), tan("b") or cot("b),
                # skip the simplification:
                if res[a].has(cos(res[b]), sin(res[b]), tan(res[b]), cot(res[b])):
                    continue
                # simplify and finish:
                expr = simp.subs(res)
                break
        if not expr.is_Mul:
            return trigsimp_nonrecursive(expr, deep)
        ret = S.One
        for x in expr.args:
            ret *= trigsimp_nonrecursive(x, deep)
        return ret
    elif expr.is_Pow:
        return Pow(trigsimp_nonrecursive(expr.base, deep),
                trigsimp_nonrecursive(expr.exp, deep))
    elif expr.is_Add:
        # TODO this needs to be faster

        # The types of trig functions we are looking for
        a,b,c = map(Wild, 'abc')
        matchers = (
            (a*sin(b)**2, a - a*cos(b)**2),
            (a*tan(b)**2, a*(1/cos(b))**2 - a),
            (a*cot(b)**2, a*(1/sin(b))**2 - a)
        )

        # Scan for the terms we need
        ret = S.Zero
        for term in expr.args:
            term = trigsimp_nonrecursive(term, deep)
            res = None
            for pattern, result in matchers:
                res = term.match(pattern)
                if res is not None:
                    ret += result.subs(res)
                    break
            if res is None:
                ret += term

        # Reduce any lingering artifacts, such as sin(x)**2 changing
        # to 1-cos(x)**2 when sin(x)**2 was "simpler"
        artifacts = (
            (a - a*cos(b)**2 + c, a*sin(b)**2 + c, cos),
            (a - a*(1/cos(b))**2 + c, -a*tan(b)**2 + c, cos),
            (a - a*(1/sin(b))**2 + c, -a*cot(b)**2 + c, sin)
        )

        expr = ret
        for pattern, result, ex in artifacts:
            # Substitute a new wild that excludes some function(s)
            # to help influence a better match. This is because
            # sometimes, for example, 'a' would match sec(x)**2
            a_t = Wild('a', exclude=[ex])
            pattern = pattern.subs(a, a_t)
            result = result.subs(a, a_t)
            if expr.is_number:
                continue

            m = expr.match(pattern)
            while m is not None:
                if m[a_t] == 0 or -m[a_t] in m[c].args or m[a_t] + m[c] == 0:
                    break
                expr = result.subs(m)
                m = expr.match(pattern)

        return expr
    return expr

def radsimp(expr):
    """
    Rationalize the denominator.

    Examples:
        >>> from sympy import radsimp, sqrt, Symbol
        >>> radsimp(1/(2+sqrt(2)))
        -2**(1/2)/2 + 1
        >>> x,y = map(Symbol, 'xy')
        >>> e = ((2+2*sqrt(2))*x+(2+sqrt(8))*y)/(2+sqrt(2))
        >>> radsimp(e)
        2**(1/2)*x + 2**(1/2)*y

    """
    n,d = fraction(expr)
    a,b,c = map(Wild, 'abc')
    r = d.match(a+b*sqrt(c))
    if r is not None:
        a = r[a]
        if r[b] == 0:
            b,c = 0,0
        else:
            b,c = r[b],r[c]

        syms = list(n.atoms(Symbol))
        n = collect((n*(a-b*sqrt(c))).expand(), syms)
        d = a**2 - c*b**2

    return n/d

def posify(eq):
    """Return eq (with generic symbols made positive) and a restore dictionary.

    Any symbol that has positive=None will be replaced with a positive dummy
    symbol having the same name. This replacement will allow more symbolic
    processing of expressions, especially those involving powers and logarithms.

    A dictionary that can be sent to subs to restore eq to its original symbols
    is also returned.

    >>> from sympy import posify, Symbol, log
    >>> from sympy.abc import x
    >>> posify(x + Symbol('p', positive=True) + Symbol('n', negative=True))
    (_x + n + p, {_x: x})

    >> log(1/x).expand() # should be log(1/x) but it comes back as -log(x)
    log(1/x)

    >>> log(posify(1/x)[0]).expand() # take [0] and ignore replacements
    -log(_x)
    >>> eq, rep = posify(1/x)
    >>> log(eq).expand().subs(rep)
    -log(x)
    >>> posify([x, 1 + x])
    ([_x, _x + 1], {_x: x})
    """
    eq = sympify(eq)
    if iterable(eq):
        f = type(eq)
        eq = list(eq)
        syms = set()
        for e in eq:
            syms = syms.union(e.atoms(C.Symbol))
        reps = {}
        for s in syms:
            reps.update(dict((v, k) for k, v in posify(s)[1].items()))
        for i, e in enumerate(eq):
            eq[i] = e.subs(reps)
        return f(eq), dict([(r,s) for s, r in reps.iteritems()])

    reps = dict([(s, Dummy(s.name, positive=True))
                 for s in eq.atoms(Symbol) if s.is_positive is None])
    eq = eq.subs(reps)
    return eq, dict([(r,s) for s, r in reps.iteritems()])

def powdenest(eq, force=False):
    """
    Collect exponents on powers as assumptions allow.

    Given (bb**be)**e, this can be simplified as follows:
        o if bb is positive or e is an integer, bb**(be*e)
        o if be has an integer in the denominatory, then
          all integers from its numerator can be joined with e
    Given a product of powers raised to a power, (bb1**be1 * bb2**be2...)**e,
    simplification can be done as follows:
        o if e is positive, the gcd of all bei can be joined with e;
        o all non-negative bb can be separated from those that are negative
          and their gcd can be joined with e; autosimplification already
          handles this separation.
        o integer factors from powers that have integers in the denominator
          of the exponent can be removed from any term and the gcd of such
          integers can be joined with e

    Setting ``force`` to True will make symbols that are not explicitly
    negative behave as though they are positive, resulting in more
    denesting.

    When there are sums of logs in exp() then a product of powers may be
    obtained e.g. exp(3*(log(a) + 2*log(b))) - > a**3*b**6.

    Examples:

    >>> from sympy.abc import a, b, x, y, z
    >>> from sympy import Symbol, exp, log, sqrt, symbols, powdenest

    >>> powdenest((x**(2*a/3))**(3*x))
    (x**(a/3))**(6*x)
    >>> powdenest(exp(3*x*log(2)))
    2**(3*x)

    Assumptions may prevent expansion:

    >> powdenest(sqrt(x**2))  # activate when log rules are fixed
    (x**2)**(1/2)

    >>> p = symbols('p', positive=True)
    >>> powdenest(sqrt(p**2))
    p

    No other expansion is done.

    >>> i, j = symbols('i,j', integer=1)
    >>> powdenest((x**x)**(i + j)) # -X-> (x**x)**i*(x**x)**j
    x**(x*(i + j))

    But exp() will be denested by moving all non-log terms outside of
    the function; this may result in the collapsing of the exp to a power
    with a different base:

    >>> powdenest(exp(3*y*log(x)))
    x**(3*y)
    >>> powdenest(exp(y*(log(a) + log(b))))
    (a*b)**y
    >>> powdenest(exp(3*(log(a) + log(b))))
    a**3*b**3

    If assumptions allow, symbols can also be moved to the outermost exponent:

    >>> i = Symbol('i', integer=True)
    >>> p = Symbol('p', positive=True)
    >>> powdenest(((x**(2*i))**(3*y))**x)
    ((x**(2*i))**(3*y))**x
    >>> powdenest(((x**(2*i))**(3*y))**x, force=1)
    x**(6*i*x*y)

    >> powdenest(((p**(2*a))**(3*y))**x)  # activate when log rules are fixed
    p**(6*a*x*y)

    >>> powdenest(((x**(2*a/3))**(3*y/i))**x)
    ((x**(a/3))**(y/i))**(6*x)
    >>> powdenest((x**(2*i)*y**(4*i))**z,1)
    (x*y**2)**(2*i*z)

    >>> n = Symbol('n', negative=1)

    >> powdenest((x**i)**y, force=1)  # activate when log rules are fixed
    x**(i*y)
    >> powdenest((n**i)**x, force=1)  # activate when log rules are fixed
    (n**i)**x

    """

    if force:
        eq, rep = posify(eq)
        return powdenest(eq, force=0).subs(rep)

    eq = S(eq)
    if eq.is_Atom:
        return eq

    # handle everything that is not a power
    #   if subs would work then one could replace the following with
    #      return eq.subs(dict([(p, powdenest(p)) for p in eq.atoms(Pow)]))
    #   but subs expands (3**x)**2 to 3**x * 3**x so the 3**(5*x)
    #   is not recognized; in addition, that would take 2 passes through
    #   the expression (once to find Pows and again to replace them). The
    #   following does it in one pass. Which is more important, efficiency
    #   or simplicity? On the other hand, this only does a shallow replacement
    #   and doesn't enter Integrals or functions, etc... so perhaps the subs
    #   approach (or adding a deep flag) is the thing to do.
    if not eq.is_Pow and not eq.func is exp:
        args = list(Add.make_args(eq))
        rebuild = False
        for i, arg in enumerate(args):
            margs = list(Mul.make_args(arg))
            changed = False
            for j, m in enumerate(margs):
                if not m.is_Pow:
                    continue
                m = powdenest(m, force=force)
                if m != margs[j]:
                    changed = True
                    margs[j] = m
            if changed:
                rebuild = True
                args[i] = C.Mul(*margs)
        if rebuild:
            eq = eq.func(*args)
        return eq

    b, e = eq.as_base_exp()

    # denest exp with log terms in exponent
    if b is S.Exp1 and e.is_Mul:
        logs = []
        other = []
        efunc = C.Mul
        for ei in Mul.make_args(e):
            if any(aj.func is C.log for a in Mul.make_args(ei)
                   for ai in Add.make_args(a) for aj in Mul.make_args(ai)):
                logs.append(ei)
            else:
                other.append(ei)
        logs = logcombine(efunc(*logs), force=force)
        return Pow(C.exp(logs), efunc(*other))

    bb, be = b.as_base_exp()
    if be is S.One and not (b.is_Mul or b.is_Rational):
        return eq

    # denest eq which is either Pow**e or Mul**e
    if force or e.is_integer:
        # replace all non-explicitly negative symbols with positive dummies
        syms = eq.atoms(Symbol)
        rep = [(s, C.Dummy(s.name, positive=True)) for s in syms if not s.is_negative]
        sub = eq.subs(rep)
    else:
        rep = []
        sub = eq

    # if any factor is a bare symbol then there is nothing to be done
    b, e = sub.as_base_exp()
    if e is S.One or any(s.is_Symbol for s in Mul.make_args(b)):
        return sub.subs([(new, old) for old, new in rep])
    # let log handle the case of the base of the argument being a mul, e.g.
    # sqrt(x**(2*i)*y**(6*i)) -> x**i*y**(3**i)
    gcd = terms_gcd(log(b).expand(log=1))
    if gcd.func is C.log or not gcd.is_Mul:
        if hasattr(gcd.args[0], 'exp'):
            gcd = powdenest(gcd.args[0])
            c, _ = gcd.exp.as_coeff_mul()
            ok = c.p != 1
            if ok:
                ok = c.q != 1
                if not ok:
                    n, d = gcd.exp.as_numer_denom()
                    ok = d is not S.One and any(di.is_integer for di in Mul.make_args(d))
            if ok:
                return Pow(Pow(gcd.base, gcd.exp/c.p), c.p*e)
        elif e.is_Mul:
            return Pow(b, e).subs([(new, old) for old, new in rep])
        return eq
    else:
        add= []
        other = []
        for g in gcd.args:
            if g.is_Add:
                add.append(g)
            else:
                other.append(g)
        return powdenest(Pow(exp(logcombine(Mul(*add))), e*Mul(*other))).subs([(new, old) for old, new in rep])

def powsimp(expr, deep=False, combine='all', force=False):
    """
    == Usage ==
        powsimp(expr, deep) -> reduces expression by combining powers with
        similar bases and exponents.

    == Notes ==
        If deep is True then powsimp() will also simplify arguments of
        functions. By default deep is set to False.

        If force is True then bases will be combined without checking for
        assumptions, e.g. sqrt(x)*sqrt(y) -> sqrt(x*y) which is not true
        if x and y are both negative.

        You can make powsimp() only combine bases or only combine exponents by
        changing combine='base' or combine='exp'.  By default, combine='all',
        which does both.  combine='base' will only combine::

             a   a          a                          2x      x
            x * y  =>  (x*y)   as well as things like 2   =>  4

        and combine='exp' will only combine
        ::

             a   b      (a + b)
            x * x  =>  x

        combine='exp' will strictly only combine exponents in the way that used
        to be automatic.  Also use deep=True if you need the old behavior.

        When combine='all', 'exp' is evaluated first.  Consider the first
        example below for when there could be an ambiguity relating to this.
        This is done so things like the second example can be completely
        combined.  If you want 'base' combined first, do something like
        powsimp(powsimp(expr, combine='base'), combine='exp').

    == Examples ==
        >>> from sympy import powsimp, exp, log, symbols
        >>> from sympy.abc import x, y, z, n
        >>> powsimp(x**y*x**z*y**z, combine='all')
        x**(y + z)*y**z
        >>> powsimp(x**y*x**z*y**z, combine='exp')
        x**(y + z)*y**z
        >>> powsimp(x**y*x**z*y**z, combine='base', force=True)
        x**y*(x*y)**z

        >>> powsimp(x**z*x**y*n**z*n**y, combine='all', force=True)
        (n*x)**(y + z)
        >>> powsimp(x**z*x**y*n**z*n**y, combine='exp')
        n**(y + z)*x**(y + z)
        >>> powsimp(x**z*x**y*n**z*n**y, combine='base', force=True)
        (n*x)**y*(n*x)**z

        >>> x, y = symbols('x y', positive=True)
        >>> powsimp(log(exp(x)*exp(y)))
        log(exp(x)*exp(y))
        >>> powsimp(log(exp(x)*exp(y)), deep=True)
        x + y

    """
    if combine not in ['all', 'exp', 'base']:
        raise ValueError("combine must be one of ('all', 'exp', 'base').")
    y = Dummy('y')
    if expr.is_Pow:
        if deep:
            return powsimp(y*powsimp(expr.base, deep, combine, force)**powsimp(\
            expr.exp, deep, combine, force), deep, combine, force)/y
        else:
            return powsimp(y*expr, deep, combine, force)/y # Trick it into being a Mul
    elif expr.is_Function:
        if expr.func is exp and deep:
            # Exp should really be like Pow
            return powsimp(y*exp(powsimp(expr.args[0], deep, combine, force)), deep, combine, force)/y
        elif expr.func is exp and not deep:
            return powsimp(y*expr, deep, combine, force)/y
        elif deep:
            return expr.func(*[powsimp(t, deep, combine, force) for t in expr.args])
        else:
            return expr
    elif expr.is_Add:
        return Add(*[powsimp(t, deep, combine, force) for t in expr.args])

    elif expr.is_Mul:
        if combine in ('exp', 'all'):
            # Collect base/exp data, while maintaining order in the
            # non-commutative parts of the product
            if combine is 'all' and deep and any((t.is_Add for t in expr.args)):
                # Once we get to 'base', there is no more 'exp', so we need to
                # distribute here.
                return powsimp(expand_mul(expr, deep=False), deep, combine, force)
            c_powers = {}
            nc_part = []
            newexpr = sympify(1)
            for term in expr.args:
                if term.is_Add and deep:
                    newexpr *= powsimp(term, deep, combine, force)
                else:
                    if term.is_commutative:
                        b, e = term.as_base_exp()
                        if deep:
                            b, e = [powsimp(i, deep, combine, force) for i in  [b, e]]
                        c_powers.setdefault(b, []).append(e)
                    else:
                        # This is the logic that combines exponents for equal,
                        # but non-commutative bases: A**x*A**y == A**(x+y).
                        if nc_part:
                            b1, e1 = nc_part[-1].as_base_exp()
                            b2, e2 = term.as_base_exp()
                            if (b1 == b2 and
                                e1.is_commutative and e2.is_commutative):
                                nc_part[-1] = Pow(b1, Add(e1, e2))
                                continue
                        nc_part.append(term)

            # add up exponents of common bases
            for b, e in c_powers.iteritems():
                c_powers[b] = Add(*e)

            # check for base and inverted base pairs
            be = c_powers.items()
            skip = set() # skip if we already saw them
            for b, e in be:
                if b in skip:
                    continue
                bpos = b.is_positive
                if bpos:
                    binv = 1/b
                    if b != binv and binv in c_powers:
                        if b.as_numer_denom()[0] is S.One:
                            c_powers.pop(b)
                            c_powers[binv] -= e
                        else:
                            skip.add(binv)
                            e = c_powers.pop(binv)
                            c_powers[b] -= e

            newexpr = Mul(*([newexpr] + [Pow(b, e) for b, e in c_powers.iteritems()]))
            if combine is 'exp':
                return Mul(newexpr, Mul(*nc_part))
            else:
                # combine is 'all', get stuff ready for 'base'
                if deep:
                    newexpr = expand_mul(newexpr, deep=False)
                if newexpr.is_Add:
                    return powsimp(Mul(*nc_part), deep, combine='base', force=force) * \
                           Add(*[powsimp(i, deep, combine='base', force=force)
                                 for i in newexpr.args])
                else:
                    return powsimp(Mul(*nc_part), deep, combine='base', force=force)*\
                    powsimp(newexpr, deep, combine='base', force=force)

        else:
            # combine is 'base'
            if deep:
                expr = expand_mul(expr, deep=False)
            if expr.is_Add:
                return Add(*[powsimp(i, deep, combine, force) for i in expr.args])
            else:
                # Build c_powers and nc_part.  These must both be lists not
                # dicts because exp's are not combined.
                c_powers = []
                nc_part = []
                for term in expr.args:
                    if term.is_commutative:
                        c_powers.append(list(term.as_base_exp()))
                    else:
                        # This is the logic that combines bases that are
                        # different and non-commutative, but with equal and
                        # commutative exponents: A**x*B**x == (A*B)**x.
                        if nc_part:
                            b1, e1 = nc_part[-1].as_base_exp()
                            b2, e2 = term.as_base_exp()
                            if (e1 == e2 and e2.is_commutative):
                                nc_part[-1] = Pow(Mul(b1, b2), e1)
                                continue
                        nc_part.append(term)

            # Pull out numerical coefficients from exponent if assumptions allow
            # e.g., 2**(2*x) => 4**x
            for i in xrange(len(c_powers)):
                b, e = c_powers[i]
                if not (b.is_nonnegative or e.is_integer or force):
                    continue
                exp_c, exp_t = e.as_coeff_mul()
                if not (exp_c is S.One) and exp_t:
                    c_powers[i] = [Pow(b, exp_c), e._new_rawargs(*exp_t)]


            # Combine bases whenever they have the same exponent and
            # assumptions allow

            # first gather the potential bases under the common exponent
            c_exp = {}
            for b, e in c_powers:
                if deep:
                    e = powsimp(e, deep, combine, force)
                c_exp.setdefault(e, []).append(b)
            del c_powers

            # Merge back in the results of the above to form a new product
            c_powers = {}
            for e in c_exp:
                bases = c_exp[e]

                # calculate the new base for e
                if len(bases) == 1:
                    new_base = bases[0]
                elif e.is_integer or force:
                    new_base = Mul(*bases)
                else:
                    # see which ones can be joined
                    unk=[]
                    nonneg=[]
                    neg=[]
                    for bi in bases:
                        if not bi.is_negative is None: #then we know the sign
                            if bi.is_negative:
                                neg.append(bi)
                            else:
                                nonneg.append(bi)
                        else:
                            unk.append(bi)
                    if len(unk) == 1 and not neg or len(neg) == 1 and not unk:
                        # a single neg or a single unk can join the rest
                        nonneg.extend(unk + neg)
                        unk = neg = []
                    elif neg:
                        # their negative signs cancel in pairs
                        neg = [-w for w in neg]
                        if len(neg) % 2:
                            unk.append(S.NegativeOne)

                    # these shouldn't be joined
                    for b in unk:
                        c_powers.setdefault(b, []).append(e)
                    # here is a new joined base
                    new_base = Mul(*(nonneg + neg))

                c_powers.setdefault(new_base, []).append(e)

            # break out the powers from c_powers now
            c_part = []
            if combine == 'all':
                #...joining the exponents
                for b, e in c_powers.iteritems():
                    c_part.append(Pow(b, Add(*e)))
            else:
                #...joining nothing
                for b, e in c_powers.iteritems():
                    for ei in e:
                        c_part.append(Pow(b, ei))

            # we're done
            return Mul(*(c_part + nc_part))

    else:
        return expr

def hypersimp(f, k):
    """Given combinatorial term f(k) simplify its consecutive term ratio
       i.e. f(k+1)/f(k).  The input term can be composed of functions and
       integer sequences which have equivalent representation in terms
       of gamma special function.

       The algorithm performs three basic steps:

           (1) Rewrite all functions in terms of gamma, if possible.

           (2) Rewrite all occurrences of gamma in terms of products
               of gamma and rising factorial with integer,  absolute
               constant exponent.

           (3) Perform simplification of nested fractions, powers
               and if the resulting expression is a quotient of
               polynomials, reduce their total degree.

       If f(k) is hypergeometric then as result we arrive with a
       quotient of polynomials of minimal degree. Otherwise None
       is returned.

       For more information on the implemented algorithm refer to:

       [1] W. Koepf, Algorithms for m-fold Hypergeometric Summation,
           Journal of Symbolic Computation (1995) 20, 399-417
    """
    f = sympify(f)

    g = f.subs(k, k+1) / f

    g = g.rewrite(gamma)
    g = expand_func(g)
    g = powsimp(g, deep=True, combine='exp')

    if g.is_rational_function(k):
        return simplify(g)
    else:
        return None

def hypersimilar(f, g, k):
    """Returns True if 'f' and 'g' are hyper-similar.

       Similarity in hypergeometric sense means that a quotient of
       f(k) and g(k) is a rational function in k.  This procedure
       is useful in solving recurrence relations.

       For more information see hypersimp().

    """
    f, g = map(sympify, (f, g))

    h = (f/g).rewrite(gamma)
    h = h.expand(func=True, basic=False)

    return h.is_rational_function(k)

def combsimp(expr):
    r"""
    Simplify combinatorial expressions.

    This function takes as input an expression containing factorials,
    binomials, Pochhammer symbol and other "combinatorial" functions,
    and tries to minimize the number of those functions and reduce
    the size of their arguments. The result is be given in terms of
    binomials and factorials.

    The algorithm works by rewriting all combinatorial functions as
    expressions involving rising factorials (Pochhammer symbols) and
    applies recurrence relations and other transformations applicable
    to rising factorials, to reduce their arguments, possibly letting
    the resulting rising factorial to cancel. Rising factorials with
    the second argument being an integer are expanded into polynomial
    forms and finally all other rising factorial are rewritten in terms
    more familiar binomials and factorials.

    All transformation rules can be found (or was derived from) here:

    1. http://functions.wolfram.com/GammaBetaErf/Pochhammer/17/01/02/
    2. http://functions.wolfram.com/GammaBetaErf/Pochhammer/27/01/0005/

    **Examples**

    >>> from sympy.simplify import combsimp
    >>> from sympy import factorial, binomial
    >>> from sympy.abc import n, k

    >>> combsimp(factorial(n)/factorial(n - 3))
    n*(n - 2)*(n - 1)
    >>> combsimp(binomial(n+1, k+1)/binomial(n, k))
    (n + 1)/(k + 1)

    """
    factorial = C.factorial
    binomial = C.binomial
    gamma = C.gamma

    def as_coeff_Add(expr):
        if expr.is_Add:
            coeff, args = expr.args[0], expr.args[1:]

            if coeff.is_Number:
                if len(args) == 1:
                    return coeff, args[0]
                else:
                    return coeff, expr._new_rawargs(*args)

        return S.Zero, expr

    class rf(Function):
        @classmethod
        def eval(cls, a, b):
            if b.is_Integer:
                if not b:
                    return S.Zero

                n, result = int(b), S.One

                if n > 0:
                    for i in xrange(0, n):
                        result *= a + i

                    return result
                else:
                    for i in xrange(1, -n+1):
                        result *= a - i

                    return 1/result
            else:
                c, _b = as_coeff_Add(b)

                if c.is_Integer:
                    if c > 0:
                        return rf(a, _b)*rf(a+_b, c)
                    elif c < 0:
                        return rf(a, _b)/rf(a+_b+c, -c)

                c, _a = as_coeff_Add(a)

                if c.is_Integer:
                    if c > 0:
                        return rf(_a, b)*rf(_a+b, c)/rf(_a, c)
                    elif c < 0:
                        return rf(_a, b)*rf(_a+c, -c)/rf(_a+b+c, -c)

    expr = expr.replace(binomial,
        lambda n, k: rf((n-k+1).expand(), k.expand())/rf(1, k.expand()))
    expr = expr.replace(factorial,
        lambda n: rf(1, n.expand()))
    expr = expr.replace(gamma,
        lambda n: rf(1, (n-1).expand()))

    expr = expr.replace(rf,
        lambda a, b: binomial(a+b-1, b)*factorial(b))

    def rule(n, k):
        coeff, rewrite = S.One, False

        cn, _n = as_coeff_Add(n)
        ck, _k = as_coeff_Add(k)

        if cn.is_Integer and cn:
            coeff *= rf(_n + 1, cn)/rf(_n - k + 1, cn)
            rewrite = True
            n = _n

        if ck.is_Integer and ck:
            coeff *= rf(n - ck - _k + 1, ck)/rf(_k + 1, ck)
            rewrite = True
            k = _k

        if rewrite:
            return coeff*binomial(n, k)

    expr = expr.replace(binomial, rule)

    return factor(expr)

def simplify(expr, ratio=1.7):
    """Naively simplifies the given expression.

       Simplification is not a well defined term and the exact strategies
       this function tries can change in the future versions of SymPy. If
       your algorithm relies on "simplification" (whatever it is), try to
       determine what you need exactly  -  is it powsimp()?, radsimp()?,
       together()?, logcombine()?, or something else? And use this particular
       function directly, because those are well defined and thus your algorithm
       will be robust.

       In some cases, applying :func:`simplify` may actually result in some more
       complicated expression.
       By default ``ratio=1.7`` prevents more extreme cases:
       if (result length)/(input length) > ratio, then input is returned
       unmodified (:func:`count_ops` is used to measure length).

       For example, if ``ratio=1``, ``simplify`` output can't be longer
       than input.

       ::

            >>> from sympy import S, simplify, count_ops, oo
            >>> root = S("(5/2 + 21**(1/2)/2)**(1/3)*(1/2 - I*3**(1/2)/2)"
            ... "+ 1/((1/2 - I*3**(1/2)/2)*(5/2 + 21**(1/2)/2)**(1/3))")

       Since ``simplify(root)`` would result in a slightly longer expression,
       root is returned inchanged instead::

            >>> simplify(root, ratio=1) is root
            True

       If ``ratio=oo``, simplify will be applied anyway::

            >>> count_ops(simplify(root, ratio=oo)) > count_ops(root)
            True

       Note that the shortest expression is not necessary the simplest, so
       setting ``ratio`` to 1 may not be a good idea.
       Heuristically, default value ``ratio=1.7`` seems like a reasonable choice.

    """
    expr = sympify(expr)

    if not isinstance(expr, Basic): # XXX: temporary hack
        return expr

    if isinstance(expr, Atom):
        return expr

    if isinstance(expr, C.Relational):
        return expr.__class__(simplify(expr.lhs, ratio=ratio),
                              simplify(expr.rhs, ratio=ratio))

    # TODO: Apply different strategies, considering expression pattern:
    # is it a purely rational function? Is there any trigonometric function?...
    # See also https://github.com/sympy/sympy/pull/185.

    original_expr = expr

    if expr.is_commutative is False:
        return together(powsimp(expr))

    expr = together(cancel(powsimp(expr)).expand())

    if not isinstance(expr, Basic): # XXX: temporary hack
        return expr

    if expr.has(C.TrigonometricFunction):
        expr = trigsimp(expr)

    if expr.has(C.log):
        expr = min([expand_log(expr, deep=True), logcombine(expr)],
                       key=count_ops)

    if expr.has(C.CombinatorialFunction, gamma):
        expr = combsimp(expr)

    expr = powsimp(expr, combine='exp', deep=True)
    numer, denom = expr.as_numer_denom()

    if denom.is_Add:
        a, b, c = map(Wild, 'abc')

        r = denom.match(a + b*c**S.Half)

        if r is not None and r[b]:
            a, b, c = r[a], r[b], r[c]

            numer *= a-b*c**S.Half
            numer = numer.expand()

            denom = a**2 - c*b**2

            expr = numer/denom

    if expr.could_extract_minus_sign():
        n, d = expr.as_numer_denom()
        if d != 0:
            expr = -n/(-d)

    if count_ops(expr) > ratio*count_ops(original_expr):
        return original_expr

    return expr

def _real_to_rational(expr):
    """
    Replace all reals in expr with rationals.

    >>> from sympy import nsimplify
    >>> from sympy.abc import x

    >>> nsimplify(.76 + .1*x**.5, rational=1)
    x**(1/2)/10 + 19/25

    """
    p = sympify(expr)
    for r in p.atoms(C.Float):
        newr = nsimplify(r)
        if not newr.is_Rational or \
           r.is_finite and not newr.is_finite:
            newr = r
            if newr < 0:
                s = -1
                newr *= s
            else:
                s = 1
            d = Pow(10, int((mpmath.log(newr)/mpmath.log(10))))
            newr = s*Rational(str(newr/d))*d
        p = p.subs(r, newr)
    return p

def nsimplify(expr, constants=[], tolerance=None, full=False, rational=False):
    """
    Replace numbers with simple representations.

    If rational=True then numbers are simply replaced with their rational
    equivalents.

    If rational=False, a simple formula that numerically matches the
    given expression is sought (and the input should be possible to evalf
    to a precision of at least 30 digits).

    Optionally, a list of (rationally independent) constants to
    include in the formula may be given.

    A lower tolerance may be set to find less exact matches.

    With full=True, a more extensive search is performed
    (this is useful to find simpler numbers when the tolerance
    is set low).

    Examples:

        >>> from sympy import nsimplify, sqrt, GoldenRatio, exp, I, exp, pi
        >>> nsimplify(4/(1+sqrt(5)), [GoldenRatio])
        -2 + 2*GoldenRatio
        >>> nsimplify((1/(exp(3*pi*I/5)+1)))
        1/2 - I*(5**(1/2)/10 + 1/4)**(1/2)
        >>> nsimplify(I**I, [pi])
        exp(-pi/2)
        >>> nsimplify(pi, tolerance=0.01)
        22/7

    """
    if rational:
        return _real_to_rational(expr)

    expr = sympify(expr)

    prec = 30
    bprec = int(prec*3.33)

    constants_dict = {}
    for constant in constants:
        constant = sympify(constant)
        v = constant.evalf(prec)
        if not v.is_Float:
            raise ValueError("constants must be real-valued")
        constants_dict[str(constant)] = v._to_mpmath(bprec)

    exprval = expr.evalf(prec, chop=True)
    re, im = exprval.as_real_imag()

    # Must be numerical
    if not ((re.is_Float or re.is_Integer) and (im.is_Float or im.is_Integer)):
        return expr

    def nsimplify_real(x):
        orig = mpmath.mp.dps
        xv = x._to_mpmath(bprec)
        try:
            # We'll be happy with low precision if a simple fraction
            if not (tolerance or full):
                mpmath.mp.dps = 15
                rat = mpmath.findpoly(xv, 1)
                if rat is not None:
                    return Rational(-int(rat[1]), int(rat[0]))
            mpmath.mp.dps = prec
            newexpr = mpmath.identify(xv, constants=constants_dict,
                tol=tolerance, full=full)
            if not newexpr:
                raise ValueError
            if full:
                newexpr = newexpr[0]
            return sympify(newexpr)
        finally:
            mpmath.mp.dps = orig
    try:
        if re: re = nsimplify_real(re)
        if im: im = nsimplify_real(im)
    except ValueError:
        return expr

    return re + im*S.ImaginaryUnit


def logcombine(expr, force=False):
    """
    Takes logarithms and combines them using the following rules:

    - log(x)+log(y) == log(x*y)
    - a*log(x) == log(x**a)

    These identities are only valid if x and y are positive and if a is real, so
    the function will not combine the terms unless the arguments have the proper
    assumptions on them.  Use logcombine(func, force=True) to
    automatically assume that the arguments of logs are positive and that
    coefficients are real.  Note that this will not change any assumptions
    already in place, so if the coefficient is imaginary or the argument
    negative, combine will still not combine the equations.  Change the
    assumptions on the variables to make them combine.

    Examples:
    >>> from sympy import Symbol, symbols, log, logcombine
    >>> from sympy.abc import a, x, y, z
    >>> logcombine(a*log(x)+log(y)-log(z))
    a*log(x) + log(y) - log(z)
    >>> logcombine(a*log(x)+log(y)-log(z), force=True)
    log(x**a*y/z)
    >>> x,y,z = symbols('x,y,z', positive=True)
    >>> a = Symbol('a', real=True)
    >>> logcombine(a*log(x)+log(y)-log(z))
    log(x**a*y/z)

    """
    # Try to make (a+bi)*log(x) == a*log(x)+bi*log(x).  This needs to be a
    # separate function call to avoid infinite recursion.
    expr = expand_mul(expr, deep=False)
    return _logcombine(expr, force)

def _logcombine(expr, force=False):
    """
    Does the main work for logcombine, it's a separate function to avoid an
    infinite recursion. See the docstrings of logcombine() for help.
    """
    def _getlogargs(expr):
        """
        Returns the arguments of the logarithm in an expression.
        Example:
        _getlogargs(a*log(x*y))
        x*y
        """
        if expr.func is log:
            return [expr.args[0]]
        else:
            args = []
            for i in expr.args:
                if i.func is log:
                    args.append(_getlogargs(i))
            return flatten(args)
        return None

    if type(expr) in (int, float) or expr.is_Number or expr.is_Rational or \
        expr.is_NumberSymbol or type(expr) == C.Integral:
        return expr

    if isinstance(expr, Equality):
        retval = Equality(_logcombine(expr.lhs-expr.rhs, force),\
        Integer(0))
        # If logcombine couldn't do much with the equality, try to make it like
        # it was.  Hopefully extract_additively won't become smart enought to
        # take logs apart :)
        right = retval.lhs.extract_additively(expr.lhs)
        if right:
            return Equality(expr.lhs, _logcombine(-right, force))
        else:
            return retval

    if expr.is_Add:
        argslist = 1
        notlogs = 0
        coeflogs = 0
        for i in expr.args:
            if i.func is log:
                if (i.args[0].is_positive or (force and not \
                i.args[0].is_nonpositive)):
                    argslist *= _logcombine(i.args[0], force)
                else:
                    notlogs += i
            elif i.is_Mul and any(map(lambda t: getattr(t,'func', False)==log,\
            i.args)):
                largs = _getlogargs(i)
                assert len(largs) != 0
                loglargs = 1
                for j in largs:
                    loglargs *= log(j)

                if  all(getattr(t,'is_positive') for t in largs)\
                    and getattr(i.extract_multiplicatively(loglargs),'is_real', False)\
                    or (force\
                        and not all(getattr(t,'is_nonpositive') for t in largs)\
                        and not getattr(i.extract_multiplicatively(loglargs),\
                        'is_real')==False):

                    coeflogs += _logcombine(i, force)
                else:
                    notlogs += i
            elif i.has(log):
                notlogs += _logcombine(i, force)
            else:
                notlogs += i
        if notlogs + log(argslist) + coeflogs == expr:
            return expr
        else:
            alllogs = _logcombine(log(argslist) + coeflogs, force)
            return notlogs + alllogs

    if expr.is_Mul:
        a = Wild('a')
        x = Wild('x')
        coef = expr.match(a*log(x))
        if coef\
            and (coef[a].is_real\
                or expr.is_Number\
                or expr.is_NumberSymbol\
                or type(coef[a]) in (int, float)\
                or (force\
                and not coef[a].is_imaginary))\
            and (coef[a].func != log\
                or force\
                or (not getattr(coef[a],'is_real')==False\
                    and getattr(x, 'is_positive'))):

            return log(coef[x]**coef[a])
        else:
            return _logcombine(expr.args[0], force)*reduce(lambda x, y:\
             _logcombine(x, force)*_logcombine(y, force),\
             expr.args[1:], 1)

    if expr.is_Function:
        return expr.func(*map(lambda t: _logcombine(t, force), expr.args))

    if expr.is_Pow:
        return _logcombine(expr.args[0], force)**\
        _logcombine(expr.args[1], force)

    return expr

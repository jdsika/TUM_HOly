"""Algorithms for partial fraction decomposition of rational functions. """

from sympy.polys import Poly, RootSum, cancel, factor
from sympy.polys.polytools import parallel_poly_from_expr

from sympy.core import S, Add, sympify, Symbol, Function, Lambda, Dummy
from sympy.utilities import numbered_symbols, take, threaded

@threaded
def apart(f, x=None, full=False):
    """
    Compute partial fraction decomposition of a rational function.

    Given a rational function ``f`` compute partial fraction decomposition
    of ``f``. Two algorithms are available: one is based on undetermined
    coefficients method and the other is Bronstein's full partial fraction
    decomposition algorithm.

    **Examples**

    >>> from sympy.polys.partfrac import apart
    >>> from sympy.abc import x, y

    >>> apart(y/(x + 2)/(x + 1), x)
    -y/(x + 2) + y/(x + 1)

    """
    f = sympify(f)

    if f.is_Atom:
        return f
    else:
        P, Q = f.as_numer_denom()

    (P, Q), opt = parallel_poly_from_expr((P, Q), x)

    if P.is_multivariate:
        raise NotImplementedError("multivariate partial fraction decomposition")

    common, P, Q = P.cancel(Q)

    poly, P = P.div(Q, auto=True)
    P, Q = P.rat_clear_denoms(Q)

    if Q.degree() <= 1:
        partial = P/Q
    else:
        if not full:
            partial = apart_undetermined_coeffs(P, Q)
        else:
            partial = apart_full_decomposition(P, Q)

    terms = S.Zero

    for term in Add.make_args(partial):
        terms += factor(term)

    return common*(poly.as_expr() + terms)

def apart_undetermined_coeffs(P, Q):
    """Partial fractions via method of undetermined coefficients. """
    X = numbered_symbols(cls=Dummy)
    partial, symbols = [], []

    _, factors = Q.factor_list()

    for f, k in factors:
        n, q = f.degree(), Q

        for i in xrange(1, k+1):
            coeffs, q = take(X, n), q.quo(f)
            partial.append((coeffs, q, f, i))
            symbols.extend(coeffs)

    dom = Q.get_domain().inject(*symbols)
    F = Poly(0, Q.gen, domain=dom)

    for i, (coeffs, q, f, k) in enumerate(partial):
        h = Poly(coeffs, Q.gen, domain=dom)
        partial[i] = (h, f, k)
        q = q.set_domain(dom)
        F += h*q

    system, result = [], S(0)

    for (k,), coeff in F.terms():
        system.append(coeff - P.nth(k))

    from sympy.solvers import solve
    solution = solve(system, symbols)

    for h, f, k in partial:
        h = h.as_expr().subs(solution)
        result += h/f.as_expr()**k

    return result

def apart_full_decomposition(P, Q):
    """
    Bronstein's full partial fraction decomposition algorithm.

    Given a univariate rational function ``f``, performing only GCD
    operations over the algebraic closure of the initial ground domain
    of definition, compute full partial fraction decomposition with
    fractions having linear denominators.

    Note that no factorization of the initial denominator of ``f`` is
    performed. The final decomposition is formed in terms of a sum of
    :class:`RootSum` instances.

    **References**

    1. [Bronstein93]_

    """
    f, x, U = P/Q, P.gen, []

    u = Function('u')(x)
    a = Dummy('a')

    partial = S(0)

    for d, n in Q.sqf_list_include(all=True):
        b = d.as_expr()
        U += [ u.diff(x, n-1) ]

        h = cancel(f*b**n) / u**n

        H, subs = [h], []

        for j in range(1, n):
            H += [ H[-1].diff(x) / j ]

        for j in range(1, n+1):
            subs += [ (U[j-1], b.diff(x, j) / j) ]

        for j in range(0, n):
            P, Q = cancel(H[j]).as_numer_denom()

            for i in range(0, j+1):
                P = P.subs(*subs[j-i])

            Q = Q.subs(*subs[0])

            P = Poly(P, x)
            Q = Poly(Q, x)

            G = P.gcd(d)
            D = d.quo(G)

            B, g = Q.half_gcdex(D)
            b = (P * B.quo(g)).rem(D)

            numer = b.as_expr()
            denom = (x-a)**(n-j)

            expr = numer.subs(x, a) / denom

            partial += RootSum(D, Lambda(a, expr))

    return partial

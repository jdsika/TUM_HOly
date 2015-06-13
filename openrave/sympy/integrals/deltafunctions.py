import sympy
from sympy.functions import DiracDelta, Heaviside
from sympy.simplify.simplify import simplify
from sympy.solvers import solve
from sympy.utilities.misc import default_sort_key

def change_mul(node, x):
    """change_mul(node, x)

       Rearranges the operands of a product, bringing to front any simple
       DiracDelta expression.

       If no simple DiracDelta expression was found, then all the DiracDelta
       expressions are simplified (using DiracDelta.simplify).

       Return: (dirac, new node)
       Where:
         o dirac is either a simple DiracDelta expression or None (if no simple expression was found);
         o new node is either a simplified DiracDelta expressions or None (if it could not be simplified).

       Examples
       --------

       >>> from sympy import DiracDelta, cos
       >>> from sympy.integrals.deltafunctions import change_mul
       >>> from sympy.abc import x, y
       >>> change_mul(x*y*DiracDelta(x)*cos(x),x)
       (DiracDelta(x), x*y*cos(x))
       >>> change_mul(x*y*DiracDelta(x**2-1)*cos(x),x)
       (None, x*y*cos(x)*DiracDelta(x - 1)/2 + x*y*cos(x)*DiracDelta(x + 1)/2)
       >>> change_mul(x*y*DiracDelta(cos(x))*cos(x),x)
       (None, None)

    """
    if not node.is_Mul:
        return node
    new_args = []
    dirac = None
    sorted_args = list(node.args)
    sorted_args.sort(key=default_sort_key)

    for arg in sorted_args:
        if arg.func == DiracDelta and arg.is_simple(x) \
                and (len(arg.args) <= 1 or arg.args[1]==0):
            if dirac is None:
                dirac = arg
            else:
                new_args.append(arg)
        else:
            new_args.append(change_mul(arg, x))
    if not dirac: # there was no simple dirac
        new_args = []
        for arg in sorted_args:
            if arg.func == DiracDelta:
                new_args.append(arg.simplify(x))
            else:
                new_args.append(change_mul(arg, x))
        if new_args != sorted_args:
            nnode = node.__class__(*new_args).expand()
        else: #if the node didn't change there is nothing to do
            nnode = None
        return (None, nnode)
    return (dirac, node.func(*new_args))


def deltaintegrate(f, x):
    """The idea for integration is the following:
    -If we are dealing with a DiracDelta expression, i.e.:
    DiracDelta(g(x)), we try to simplify it.
    If we could simplify it, then we integrate the resulting expression.
    We already know we can integrate a simplified expression, because only
    simple DiracDelta expressions are involved.
    If we couldn't simplify it, there are two cases:
    1) The expression is a simple expression, then we return the integral
    Taking care if we are dealing with a Derivative or with a proper DiracDelta
    2) The expression is not simple(i.e. DiracDelta(cos(x))), we can do nothing at all

    -If the node is a multiplication node having a DiracDelta term
    First we expand it.
    If the expansion did work, the we try to integrate the expansion
    If not, we try to extract a simple DiracDelta term, then we have two cases
    1)We have a simple DiracDelta term, so we return the integral
    2)We didn't have a simple term, but we do have an expression with simplified
    DiracDelta terms, so we integrate this expression

    """
    if not f.has(DiracDelta):
        return None
    # g(x) = DiracDelta(h(x))
    if f.func == DiracDelta:
        h = f.simplify(x)
        if h == f:#can't simplify the expression
            #FIXME: the second term tells whether is DeltaDirac or Derivative
            #For integrating derivatives of DiracDelta we need the chain rule
            if f.is_simple(x):
                if (len(f.args) <= 1 or f.args[1]==0):
                    return Heaviside(f.args[0])
                else:
                    return (DiracDelta(f.args[0],f.args[1]-1)/ f.args[0].as_poly().LC())
        else:#let's try to integrate the simplified expression
            fh = sympy.integrals.integrate(h, x)
            return fh
    elif f.is_Mul: #g(x)=a*b*c*f(DiracDelta(h(x)))*d*e
        g = f.expand()
        if f != g:#the expansion worked
            fh = sympy.integrals.integrate(g, x)
            if fh and not isinstance(fh, sympy.integrals.Integral):
                return fh
        else:#no expansion performed, try to extract a simple DiracDelta term
            dg, rest_mult = change_mul(f, x)

            if not dg:
                if rest_mult:
                    fh = sympy.integrals.integrate(rest_mult, x)
                    return fh
            else:
                dg = dg.simplify(x)
                if dg.is_Mul: # Take out any extracted factors
                    dg, rest_mult_2 = change_mul(dg, x)
                    rest_mult = rest_mult*rest_mult_2
                point = solve(dg.args[0],x)[0]
                return (rest_mult.subs(x, point)*Heaviside(x - point))
    return None


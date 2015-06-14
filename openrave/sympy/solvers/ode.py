"""
This module contains dsolve() and different helper functions that it
uses.

dsolve() solves ordinary differential equations. See the docstring on
the various functions for their uses. Note that partial differential
equations support is in pde.py.  Note that ode_hint() functions have
docstrings describing their various methods, but they are intended for
internal use.  Use dsolve(ode, func, hint=hint) to solve an ode using a
specific hint.  See also the docstring on dsolve().

**Functions in this module**

    These are the user functions in this module:

    - dsolve() - Solves ODEs.
    - classify_ode() - Classifies ODEs into possible hints for dsolve().
    - checkodesol() - Checks if an equation is the solution to an ODE.
    - ode_order() - Returns the order (degree) of an ODE.
    - homogeneous_order() - Returns the homogeneous order of an
      expression.

    These are the non-solver helper functions that are for internal use.
    The user should use the various options to dsolve() to obtain the
    functionality provided by these functions:

    - odesimp() - Does all forms of ODE simplification.
    - ode_sol_simplicity() - A key function for comparing solutions by
      simplicity.
    - constantsimp() - Simplifies arbitrary constants.
    - constant_renumber() - Renumber arbitrary constants
    - _handle_Integral() - Evaluate unevaluated Integrals.

    See also the docstrings of these functions.

**Solving methods currently implemented**

The following methods are implemented for solving ordinary differential
equations.  See the docstrings of the various ode_hint() functions for
more information on each (run help(ode)):
    - 1st order separable differential equations
    - 1st order differential equations whose coefficients or dx and dy
      are functions homogeneous of the same order.
    - 1st order exact differential equations.
    - 1st order linear differential equations
    - 1st order Bernoulli differential equations.
    - 2nd order Liouville differential equations.
    - nth order linear homogeneous differential equation with constant
      coefficients.
    - nth order linear inhomogeneous differential equation with constant
      coefficients using the method of undetermined coefficients.
    - nth order linear inhomogeneous differential equation with constant
      coefficients using the method of variation of parameters.

**Philosophy behind this module**

This module is designed to make it easy to add new ODE solving methods
without having to mess with the solving code for other methods.  The
idea is that there is a classify_ode() function, which takes in an ODE
and tells you what hints, if any, will solve the ODE.  It does this
without attempting to solve the ODE, so it is fast.  Each solving method
is a hint, and it has its own function, named ode_hint.  That function
takes in the ODE and any match expression gathered by classify_ode and
returns a solved result.  If this result has any integrals in it, the
ode_hint function will return an unevaluated Integral class. dsolve(),
which is the user wrapper function around all of this, will then call
odesimp() on the result, which, among other things, will attempt to
solve the equation for the dependent variable (the function we are
solving for), simplify the arbitrary constants in the expression, and
evaluate any integrals, if the hint allows it.

**How to add new solution methods**

If you have an ODE that you want dsolve() to be able to solve, try to
avoid adding special case code here.  Instead, try finding a general
method that will solve your ODE, as well as others.  This way, the ode
module will become more robust, and unhindered by special case hacks.
WolphramAlpha and Maple's DETools[odeadvisor] function are two resources
you can use to classify a specific ODE.  It is also better for a method
to work with an nth order ODE instead of only with specific orders, if
possible.

To add a new method, there are a few things that you need to do.  First,
you need a hint name for your method.  Try to name your hint so that it
is unambiguous with all other methods, including ones that may not be
implemented yet.  If your method uses integrals, also include a
"hint_Integral" hint.  If there is more than one way to solve ODEs with
your method, include a hint for each one, as well as a "hint_best" hint.
Your ode_hint_best() function should choose the best using min with
ode_sol_simplicity as the key argument.  See
ode_1st_homogeneous_coeff_best(), for example. The function that uses
your method will be called ode_hint(), so the hint must only use
characters that are allowed in a Python function name (alphanumeric
characters and the underscore '_' character).  Include a function for
every hint, except for "_Integral" hints (dsolve() takes care of those
automatically).  Hint names should be all lowercase, unless a word is
commonly capitalized (such as Integral or Bernoulli). If you have a hint
that you do not want to run with "all_Integral" that doesn't have an
"_Integral" counterpart (such as a best hint that would defeat the
purpose of "all_Integral"), you will need to remove it manually in the
dsolve() code.  See also the classify_ode() docstring for guidelines on
writing a hint name.

Determine *in general* how the solutions returned by your method
compare with other methods that can potentially solve the same ODEs.
Then, put your hints in the allhints tuple in the order that they should
be called.  The ordering of this tuple determines which hints are
default. Note that exceptions are ok, because it is easy for the user to
choose individual hints with dsolve().  In general, "_Integral" variants
should go at the end of the list, and "_best" variants should go before
the various hints they apply to.  For example, the
"undetermined_coefficients" hint comes before the
"variation_of_parameters" hint because, even though variation of
parameters is more general than undetermined coefficients, undetermined
coefficients generally returns cleaner results for the ODEs that it can
solve than variation of parameters does, and it does not require
integration, so it is much faster.

Next, you need to have a match expression or a function that matches the
type of the ODE, which you should put in classify_ode() (if the match
function is more than just a few lines, like
_undetermined_coefficients_match(), it should go outside of
classify_ode()).  It should match the ODE without solving for it as much
as possible, so that classify_ode() remains fast and is not hindered by
bugs in solving code.  Be sure to consider corner cases. For example, if
your solution method involves dividing by something, make sure you
exclude the case where that division will be 0.

In most cases, the matching of the ODE will also give you the various
parts that you need to solve it. You should put that in a dictionary
(.match() will do this for you), and add that as matching_hints['hint']
= matchdict in the relevant part of classify_ode.  classify_ode will
then send this to dsolve(), which will send it to your function as the
match argument. Your function should be named ode_hint(eq, func, order,
match). If you need to send more information, put it in the match
dictionary.  For example, if you had to substitute in a dummy variable
in classify_ode to match the ODE, you will need to pass it to your
function using the match dict to access it.  You can access the
independent variable using func.args[0], and the dependent variable (the
function you are trying to solve for) as func.func.  If, while trying to
solve the ODE, you find that you cannot, raise NotImplementedError.
dsolve() will catch this error with the "all" meta-hint, rather than
causing the whole routine to fail.

Add a docstring to your function that describes the method employed.
Like with anything else in SymPy, you will need to add a doctest to the
docstring, in addition to real tests in test_ode.py.  Try to maintain
consistency with the other hint functions' docstrings.  Add your method
to the list at the top of this docstring.  Also, add your method to
ode.txt in the docs/src directory, so that the Sphinx docs will pull its
docstring into the main SymPy documentation.  Be sure to make the Sphinx
documentation by running "make html" from within the doc directory to
verify that the docstring formats correctly.

If your solution method involves integrating, use C.Integral() instead
of integrate().  This allows the user to bypass hard/slow integration by
using the "_Integral" variant of your hint.  In most cases, calling
.doit() will integrate your solution.  If this is not the case, you will
need to write special code in _handle_Integral().  Arbitrary constants
should be symbols named C1, C2, and so on.  All solution methods should
return an equality instance.  If you need an arbitrary number of
arbitrary constants, you can use constants =
numbered_symbols(prefix='C', cls=Symbol, start=1).  If it is
possible to solve for the dependent function in a general way, do so.
Otherwise, do as best as you can, but do not call solve in your
ode_hint() function.  odesimp() will attempt to solve the solution for
you, so you do not need to do that. Lastly, if your ODE has a common
simplification that can be applied to your solutions, you can add a
special case in odesimp() for it.  For example, solutions returned from
the "1st_homogeneous_coeff" hints often have many log() terms, so
odesimp() calls logcombine() on them (it also helps to write the
arbitrary constant as log(C1) instead of C1 in this case).  Also
consider common ways that you can rearrange your solution to have
constantsimp() take better advantage of it.  It is better to put
simplification in odesimp() than in your method, because it can then be
turned off with the simplify flag in dsolve(). If you have any
extraneous simplification in your function, be sure to only run it using
"if match.get('simplify', True):", especially if it can be slow or if it
can reduce the domain of the solution.

Finally, as with every contribution to SymPy, your method will need to
be tested. Add a test for each method in test_ode.py.  Follow the
conventions there, i.e., test the solver using dsolve(eq, f(x),
hint=your_hint), and also test the solution using checkodesol (you can
put these in a separate tests and skip/XFAIL if it runs too slow/doesn't
work).  Be sure to call your hint specifically in dsolve, that way the
test won't be broken simply by the introduction of another matching
hint. If your method works for higher order (>1) ODEs, you will need to
run sol = constant_renumber(sol, 'C', 1, order), for each solution, where
order is the order of the ODE. This is because constant_renumber renumbers
the arbitrary constants by printing order, which is platform dependent.
Try to test every corner case of your solver, including a range of
orders if it is a nth order solver, but if your solver is slow, auch as
if it involves hard integration, try to keep the test run time down.

Feel free to refactor existing hints to avoid duplicating code or
creating inconsistencies.  If you can show that your method exactly
duplicates an existing method, including in the simplicity and speed of
obtaining the solutions, then you can remove the old, less general
method. The existing code is tested extensively in test_ode.py, so if
anything is broken, one of those tests will surely fail.

"""
from sympy.core import Add, Basic, C, S, Mul, Pow, oo
from sympy.core.compatibility import iterable, cmp_to_key
from sympy.core.function import Derivative, diff, expand_mul
from sympy.core.multidimensional import vectorize
from sympy.core.relational import Equality, Eq
from sympy.core.symbol import Symbol, Wild, Dummy
from sympy.core.sympify import sympify

from sympy.functions import cos, exp, im, log, re, sin, tan, sign, sqrt
from sympy.matrices import wronskian
from sympy.polys import Poly, RootOf
from sympy.series import Order
from sympy.simplify import collect, logcombine, powsimp, separatevars, \
    simplify, trigsimp
from sympy.solvers import solve

from sympy.utilities import numbered_symbols

# This is a list of hints in the order that they should be applied.  That means
# that, in general, hints earlier in the list should produce simpler results
# than those later for ODEs that fit both.  This is just based on my own
# empirical observations, so if you find that *in general*, a hint later in
# the list is better than one before it, fell free to modify the list.  Note
# however that you can easily override the hint used in dsolve() for a specific ODE
# (see the docstring).  In general, "_Integral" hints should be grouped
# at the end of the list, unless there is a method that returns an unevaluable
# integral most of the time (which should surely go near the end of the list
# anyway).
# "default", "all", "best", and "all_Integral" meta-hints should not be
# included in this list, but "_best" and "_Integral" hints should be included.
allhints = ("separable", "1st_exact", "1st_linear", "Bernoulli", "Riccati_special_minus2",
"1st_homogeneous_coeff_best", "1st_homogeneous_coeff_subs_indep_div_dep",
"1st_homogeneous_coeff_subs_dep_div_indep", "nth_linear_constant_coeff_homogeneous",
"nth_linear_constant_coeff_undetermined_coefficients",
"nth_linear_constant_coeff_variation_of_parameters",
"Liouville", "separable_Integral", "1st_exact_Integral", "1st_linear_Integral",
"Bernoulli_Integral", "1st_homogeneous_coeff_subs_indep_div_dep_Integral",
"1st_homogeneous_coeff_subs_dep_div_indep_Integral",
"nth_linear_constant_coeff_variation_of_parameters_Integral",
"Liouville_Integral")

def sub_func_doit(eq, func, new):
    """When replacing the func with something else, we usually
    want the derivative evaluated, so this function helps in
    making that happen.

    To keep subs from having to look through all derivatives, we
    mask them off with dummy variables, do the func sub, and then
    replace masked off derivatives with their doit values.
    """
    reps = {}
    repu = {}
    for d in eq.atoms(Derivative):
        u = C.Dummy('u')
        repu[u] = d.subs(func, new).doit()
        reps[d] = u

    return eq.subs(reps).subs(func, new).subs(repu)

def dsolve(eq, func, hint="default", simplify=True, **kwargs):
    """
    Solves any (supported) kind of ordinary differential equation.

    **Usage**

        dsolve(eq, f(x), hint) -> Solve ordinary differential equation
        eq for function f(x), using method hint.


    **Details**

        ``eq`` can be any supported ordinary differential equation (see
            the ode docstring for supported methods).  This can either
            be an Equality, or an expression, which is assumed to be
            equal to 0.

        ``f(x)`` is a function of one variable whose derivatives in that
            variable make up the ordinary differential equation eq.

        ``hint`` is the solving method that you want dsolve to use.  Use
            classify_ode(eq, f(x)) to get all of the possible hints for
            an ODE.  The default hint, 'default', will use whatever hint
            is returned first by classify_ode().  See Hints below for
            more options that you can use for hint.
        ``simplify`` enables simplification by odesimp().  See its
            docstring for more information.  Turn this off, for example,
            to disable solving of solutions for func or simplification
            of arbitrary constants.  It will still integrate with this
            hint. Note that the solution may contain more arbitrary
            constants than the order of the ODE with this option
            enabled.

    **Hints**

        Aside from the various solving methods, there are also some
        meta-hints that you can pass to dsolve():

        "default":
                This uses whatever hint is returned first by
                classify_ode(). This is the default argument to
                dsolve().

        "all":
                To make dsolve apply all relevant classification hints,
                use dsolve(ODE, func, hint="all").  This will return a
                dictionary of hint:solution terms.  If a hint causes
                dsolve to raise the NotImplementedError, value of that
                hint's key will be the exception object raised.  The
                dictionary will also include some special keys:

                - order: The order of the ODE.  See also ode_order().
                - best: The simplest hint; what would be returned by
                  "best" below.
                - best_hint: The hint that would produce the solution
                  given by 'best'.  If more than one hint produces the
                  best solution, the first one in the tuple returned by
                  classify_ode() is chosen.
                - default: The solution that would be returned by
                  default.  This is the one produced by the hint that
                  appears first in the tuple returned by classify_ode().

        "all_Integral":
                This is the same as "all", except if a hint also has a
                corresponding "_Integral" hint, it only returns the
                "_Integral" hint.  This is useful if "all" causes
                dsolve() to hang because of a difficult or impossible
                integral.  This meta-hint will also be much faster than
                "all", because integrate() is an expensive routine.

        "best":
                To have dsolve() try all methods and return the simplest
                one.  This takes into account whether the solution is
                solvable in the function, whether it contains any
                Integral classes (i.e. unevaluatable integrals), and
                which one is the shortest in size.

        See also the classify_ode() docstring for more info on hints,
        and the ode docstring for a list of all supported hints.


    **Tips**
        - You can declare the derivative of an unknown function this way:
            >>> from sympy import Function, Derivative
            >>> from sympy.abc import x # x is the independent variable
            >>> f = Function("f")(x) # f is a function of x
            >>> # f_ will be the derivative of f with respect to x
            >>> f_ = Derivative(f, x)

        - See test_ode.py for many tests, which serves also as a set of
          examples for how to use dsolve().
        - dsolve always returns an Equality class (except for the case
          when the hint is "all" or "all_Integral").  If possible, it
          solves the solution explicitly for the function being solved
          for. Otherwise, it returns an implicit solution.
        - Arbitrary constants are symbols named C1, C2, and so on.
        - Because all solutions should be mathematically equivalent,
          some hints may return the exact same result for an ODE. Often,
          though, two different hints will return the same solution
          formatted differently.  The two should be equivalent. Also
          note that sometimes the values of the arbitrary constants in
          two different solutions may not be the same, because one
          constant may have "absorbed" other constants into it.
        - Do help(ode.ode_hintname) to get help more information on a
          specific hint, where hintname is the name of a hint without
          "_Integral".

    **Examples**

        >>> from sympy import Function, dsolve, Eq, Derivative, sin, cos
        >>> from sympy.abc import x
        >>> f = Function('f')
        >>> dsolve(Derivative(f(x),x,x)+9*f(x), f(x))
        f(x) == C1*cos(3*x) + C2*sin(3*x)
        >>> dsolve(sin(x)*cos(f(x)) + cos(x)*sin(f(x))*f(x).diff(x), f(x),
        ...     hint='separable')
        -log(sin(f(x))**2 - 1)/2 == C1 + log(sin(x)**2 - 1)/2
        >>> dsolve(sin(x)*cos(f(x)) + cos(x)*sin(f(x))*f(x).diff(x), f(x),
        ...     hint='1st_exact')
        f(x) == acos(C1/cos(x))
        >>> dsolve(sin(x)*cos(f(x)) + cos(x)*sin(f(x))*f(x).diff(x), f(x),
        ... hint='best')
        f(x) == acos(C1/cos(x))
        >>> # Note that even though separable is the default, 1st_exact produces
        >>> # a simpler result in this case.

    """
    # TODO: Implement initial conditions
    # See issue 1621.  We first need a way to represent things like f'(0).
    if isinstance(eq, Equality):
        if eq.rhs != 0:
            return dsolve(eq.lhs-eq.rhs, func, hint=hint, simplify=simplify, **kwargs)
        eq = eq.lhs

    # Magic that should only be used internally.  Prevents classify_ode from
    # being called more than it needs to be by passing its results through
    # recursive calls.
    if kwargs.get('classify', True):
        hints = classify_ode(eq, func, dict=True)
    else:
        # Here is what all this means:
        #
        # hint:    The hint method given to dsolve() by the user.
        # hints:   The dictionary of hints that match the ODE, along with
        #          other information (including the internal pass-through magic).
        # default: The default hint to return, the first hint from allhints
        #          that matches the hint.  This is obtained from classify_ode().
        # match:   The hints dictionary contains a match dictionary for each hint
        #          (the parts of the ODE for solving).  When going through the
        #          hints in "all", this holds the match string for the current
        #          hint.
        # order:   The order of the ODE, as determined by ode_order().
        hints = kwargs.get('hint',
                           {'default': hint,
                            hint: kwargs['match'],
                            'order': kwargs['order']})


    if hints['order'] == 0:
        raise ValueError(str(eq) + " is not a differential equation in " + str(func))

    if not hints['default']:
        # classify_ode will set hints['default'] to None if no hints match.
        raise NotImplementedError("dsolve: Cannot solve " + str(eq))

    if hint == 'default':
        return dsolve(eq, func, hint=hints['default'], simplify=simplify, classify=False,
        order=hints['order'], match=hints[hints['default']])
    elif hint in ('all', 'all_Integral', 'best'):
        retdict = {}
        failedhints = {}
        gethints = set(hints) - set(['order', 'default', 'ordered_hints'])
        if hint == 'all_Integral':
            for i in hints:
                if i.endswith('_Integral'):
                    gethints.remove(i[:-len('_Integral')])
            # special case
            if "1st_homogeneous_coeff_best" in gethints:
                gethints.remove("1st_homogeneous_coeff_best")
        for i in gethints:
            try:
                sol = dsolve(eq, func, hint=i, simplify=simplify, classify=False,
                   order=hints['order'], match=hints[i])
            except NotImplementedError, detail: # except NotImplementedError as detail:
                failedhints[i] = detail
            else:
                retdict[i] = sol
        retdict['best'] = min(retdict.values(), key=lambda x:
            ode_sol_simplicity(x, func, trysolving=not simplify))
        if hint == 'best':
            return retdict['best']
        for i in hints['ordered_hints']:
            if retdict['best'] == retdict.get(i, None):
                retdict['best_hint'] = i
                break
        retdict['default'] = hints['default']
        retdict['order'] = sympify(hints['order'])
        retdict.update(failedhints)
        return retdict
    elif hint not in allhints: # and hint not in ('default', 'ordered_hints'):
        raise ValueError("Hint not recognized: " + hint)
    elif hint not in hints:
        raise ValueError("ODE " + str(eq) + " does not match hint " + hint)
    elif hint.endswith('_Integral'):
        solvefunc = globals()['ode_' + hint[:-len('_Integral')]]
    else:
        solvefunc = globals()['ode_' + hint] # convert the string into a function
    # odesimp() will attempt to integrate, if necessary, apply constantsimp(),
    # attempt to solve for func, and apply any other hint specific simplifications
    if simplify:
        rv = odesimp(solvefunc(eq, func, order=hints['order'],
            match=hints[hint]), func, hints['order'], hint)
    else:
        # We still want to integrate (you can disable it separately with the hint)
        r = hints[hint]
        r['simplify'] = False # Some hints can take advantage of this option
        rv = _handle_Integral(solvefunc(eq, func, order=hints['order'],
            match=hints[hint]), func, hints['order'], hint)
    return rv

def classify_ode(eq, func, dict=False):
    """
    Returns a tuple of possible dsolve() classifications for an ODE.

    The tuple is ordered so that first item is the classification that
    dsolve() uses to solve the ODE by default.  In general,
    classifications at the near the beginning of the list will produce
    better solutions faster than those near the end, thought there are
    always exceptions.  To make dsolve use a different classification,
    use dsolve(ODE, func, hint=<classification>).  See also the dsolve()
    docstring for different meta-hints you can use.

    If dict is true, classify_ode() will return a dictionary of
    hint:match expression terms. This is intended for internal use by
    dsolve().  Note that because dictionaries are ordered arbitrarily,
    this will most likely not be in the same order as the tuple.

    You can get help on different hints by doing help(ode.ode_hintname),
    where hintname is the name of the hint without "_Integral".

    See sympy.ode.allhints or the sympy.ode docstring for a list of all
    supported hints that can be returned from classify_ode.

    **Notes on Hint Names**

    *"_Integral"*

        If a classification has "_Integral" at the end, it will return
        the expression with an unevaluated Integral class in it.  Note
        that a hint may do this anyway if integrate() cannot do the
        integral, though just using an "_Integral" will do so much
        faster.  Indeed, an "_Integral" hint will always be faster than
        its corresponding hint without "_Integral" because integrate()
        is an expensive routine.  If dsolve() hangs, it is probably
        because integrate() is hanging on a tough or impossible
        integral.  Try using an "_Integral" hint or "all_Integral" to
        get it return something.

        Note that some hints do not have "_Integral" counterparts.  This
        is because integrate() is not used in solving the ODE for those
        method. For example, nth order linear homogeneous ODEs with
        constant coefficients do not require integration to solve, so
        there is no "nth_linear_homogeneous_constant_coeff_Integrate"
        hint. You can easily evaluate any unevaluated Integrals in an
        expression by doing expr.doit().

    *Ordinals*

        Some hints contain an ordinal such as "1st_linear".  This is to
        help differentiate them from other hints, as well as from other
        methods that may not be implemented yet. If a hint has "nth" in
        it, such as the "nth_linear" hints, this means that the method
        used to applies to ODEs of any order.

    *"indep" and "dep"*

        Some hints contain the words "indep" or "dep".  These reference
        the independent variable and the dependent function,
        respectively. For example, if an ODE is in terms of f(x), then
        "indep" will refer to x and "dep" will refer to f.

    *"subs"*

        If a hints has the word "subs" in it, it means the the ODE is
        solved by substituting the expression given after the word
        "subs" for a single dummy variable.  This is usually in terms of
        "indep" and "dep" as above.  The substituted expression will be
        written only in characters allowed for names of Python objects,
        meaning operators will be spelled out.  For example, indep/dep
        will be written as indep_div_dep.

    *"coeff"*

        The word "coeff" in a hint refers to the coefficients of
        something in the ODE, usually of the derivative terms.  See the
        docstring for the individual methods for more info (help(ode)).
        This is contrast to "coefficients", as in
        "undetermined_coefficients", which refers to the common name of
        a method.

    *"_best"*

        Methods that have more than one fundamental way to solve will
        have a hint for each sub-method and a "_best"
        meta-classification. This will evaluate all hints and return the
        best, using the same considerations as the normal "best"
        meta-hint.


    **Examples**
        >>> from sympy import Function, classify_ode, Eq
        >>> from sympy.abc import x
        >>> f = Function('f')
        >>> classify_ode(Eq(f(x).diff(x), 0), f(x))
        ('separable', '1st_linear', '1st_homogeneous_coeff_best',
        '1st_homogeneous_coeff_subs_indep_div_dep',
        '1st_homogeneous_coeff_subs_dep_div_indep',
        'nth_linear_constant_coeff_homogeneous', 'separable_Integral',
        '1st_linear_Integral',
        '1st_homogeneous_coeff_subs_indep_div_dep_Integral',
        '1st_homogeneous_coeff_subs_dep_div_indep_Integral')
        >>> classify_ode(f(x).diff(x, 2) + 3*f(x).diff(x) + 2*f(x) - 4, f(x))
        ('nth_linear_constant_coeff_undetermined_coefficients',
        'nth_linear_constant_coeff_variation_of_parameters',
        'nth_linear_constant_coeff_variation_of_parameters_Integral')

    """
    from sympy import expand

    if len(func.args) != 1:
        raise ValueError("dsolve() and classify_ode() only work with functions " + \
            "of one variable")

    x = func.args[0]
    f = func.func
    y = Dummy('y')
    if isinstance(eq, Equality):
        if eq.rhs != 0:
            return classify_ode(eq.lhs-eq.rhs, func)
        eq = eq.lhs
    order = ode_order(eq, f(x))
    # hint:matchdict or hint:(tuple of matchdicts)
    # Also will contain "default":<default hint> and "order":order items.
    matching_hints = {"order": order}

    if not order:
        if dict:
            matching_hints["default"] = None
            return matching_hints
        else:
            return ()

    df = f(x).diff(x)
    a = Wild('a', exclude=[f(x)])
    b = Wild('b', exclude=[f(x)])
    c = Wild('c', exclude=[f(x)])
    d = Wild('d', exclude=[df, f(x).diff(x, 2)])
    e = Wild('e', exclude=[df])
    k = Wild('k', exclude=[df])
    n = Wild('n', exclude=[f(x)])
    c1 = Wild('c1', exclude=[x])
    a2 = Wild('a2', exclude=[x, f(x), df])
    b2 = Wild('b2', exclude=[x, f(x), df])
    c2 = Wild('c2', exclude=[x, f(x), df])
    d2 = Wild('d2', exclude=[x, f(x), df])

    eq = expand(eq)

    # Precondition to try remove f(x) from highest order derivative
    reduced_eq = None
    if eq.is_Add:
        deriv_coef = eq.coeff(f(x).diff(x, order))
        if deriv_coef != 1:
            r = deriv_coef.match(a*f(x)**c1)
            if r and r[c1]:
                den = f(x)**r[c1]
                reduced_eq = Add(*[arg/den for arg in eq.args])
    if not reduced_eq:
        reduced_eq = eq

    if order == 1:

        # Linear case: a(x)*y'+b(x)*y+c(x) == 0
        if eq.is_Add:
            ind, dep = reduced_eq.as_independent(f)
        else:
            u = Dummy('u')
            ind, dep = (reduced_eq + u).as_independent(f)
            ind, dep = [tmp.subs(u, 0) for tmp in [ind, dep]]
        r = {a: dep.coeff(df) or S.Zero, # if we get None for coeff, take 0
             b: dep.coeff(f(x)) or S.Zero, # ditto
             c: ind}
        # double check f[a] since the preconditioning may have failed
        if not r[a].has(f) and (r[a]*df + r[b]*f(x) + r[c]).expand() - reduced_eq == 0:
            r['a'] = a
            r['b'] = b
            r['c'] = c
            matching_hints["1st_linear"] = r
            matching_hints["1st_linear_Integral"] = r

        # Bernoulli case: a(x)*y'+b(x)*y+c(x)*y**n == 0
        r = collect(reduced_eq, f(x), exact = True).match(a*df + b*f(x) + c*f(x)**n)
        if r and r[c] != 0 and r[n] != 1: # See issue 1577
            r['a'] = a
            r['b'] = b
            r['c'] = c
            r['n'] = n
            matching_hints["Bernoulli"] = r
            matching_hints["Bernoulli_Integral"] = r

        # Riccati special n == -2 case: a2*y'+b2*y**2+c2*y/x+d2/x**2 == 0
        r = collect(reduced_eq, f(x), exact = True).match(a2*df + b2*f(x)**2 + c2*f(x)/x + d2/x**2)
        if r and r[b2] != 0 and (r[c2] != 0 or r[d2] != 0):
            r['a2'] = a2
            r['b2'] = b2
            r['c2'] = c2
            r['d2'] = d2
            matching_hints["Riccati_special_minus2"] = r

        # Exact Differential Equation: P(x,y)+Q(x,y)*y'=0 where dP/dy == dQ/dx
        # WITH NON-REDUCED FORM OF EQUATION
        r = collect(eq, df, exact = True).match(d + e * df)
        if r:
            r['d'] = d
            r['e'] = e
            r['y'] = y
            r[d] = r[d].subs(f(x),y)
            r[e] = r[e].subs(f(x),y)
            try:
                if r[d] != 0 and simplify(r[d].diff(y)) == simplify(r[e].diff(x)):
                    matching_hints["1st_exact"] = r
                    matching_hints["1st_exact_Integral"] = r
            except NotImplementedError:
                # Differentiating the coefficients might fail because of things
                # like f(2*x).diff(x).  See issue 1525 and issue 1620.
                pass

        # This match is used for several cases below; we now collect on
        # f(x) so the matching works.
        r = collect(reduced_eq, df, exact = True).match(d+e*df)
        if r:
            r['d'] = d
            r['e'] = e
            r['y'] = y
            r[d] = r[d].subs(f(x),y)
            r[e] = r[e].subs(f(x),y)

            # Separable Case: y' == P(y)*Q(x)
            r[d] = separatevars(r[d])
            r[e] = separatevars(r[e])
            # m1[coeff]*m1[x]*m1[y] + m2[coeff]*m2[x]*m2[y]*y'
            m1 = separatevars(r[d], dict=True, symbols=(x, y))
            m2 = separatevars(r[e], dict=True, symbols=(x, y))
            if m1 and m2:
                r1 = {'m1':m1, 'm2':m2, 'y':y}
                matching_hints["separable"] = r1
                matching_hints["separable_Integral"] = r1

            # First order equation with homogeneous coefficients:
            # dy/dx == F(y/x) or dy/dx == F(x/y)
            ordera = homogeneous_order(r[d], x, y)
            orderb = homogeneous_order(r[e], x, y)
            if ordera == orderb and ordera is not None:
                # u1=y/x and u2=x/y
                u1 = Dummy('u1')
                u2 = Dummy('u2')
                if simplify((r[d]+u1*r[e]).subs({x:1, y:u1})) != 0:
                    matching_hints["1st_homogeneous_coeff_subs_dep_div_indep"] = r
                    matching_hints["1st_homogeneous_coeff_subs_dep_div_indep_Integral"] = r
                if simplify((r[e]+u2*r[d]).subs({x:u2, y:1})) != 0:
                    matching_hints["1st_homogeneous_coeff_subs_indep_div_dep"] = r
                    matching_hints["1st_homogeneous_coeff_subs_indep_div_dep_Integral"] = r
                if "1st_homogeneous_coeff_subs_dep_div_indep" in matching_hints \
                and "1st_homogeneous_coeff_subs_indep_div_dep" in matching_hints:
                    matching_hints["1st_homogeneous_coeff_best"] = r

    if order == 2:
        # Liouville ODE f(x).diff(x, 2) + g(f(x))*(f(x).diff(x))**2 + h(x)*f(x).diff(x)
        # See Goldstein and Braun, "Advanced Methods for the Solution of
        # Differential Equations", pg. 98

        s = d*f(x).diff(x, 2) + e*df**2 + k*df
        r = reduced_eq.match(s)
        if r and r[d] != 0:
            y = Dummy('y')
            g = simplify(r[e]/r[d]).subs(f(x), y)
            h = simplify(r[k]/r[d])
            if h.has(f(x)) or g.has(x):
                pass
            else:
                r = {'g':g, 'h':h, 'y':y}
                matching_hints["Liouville"] = r
                matching_hints["Liouville_Integral"] = r


    if order > 0:
        # nth order linear ODE
        # a_n(x)y^(n) + ... + a_1(x)y' + a_0(x)y = F(x) = b

        r = _nth_linear_match(reduced_eq, func, order)

        # Constant coefficient case (a_i is constant for all i)
        if r and not any(r[i].has(x) for i in r if i >= 0):
            # Inhomogeneous case: F(x) is not identically 0
            if r[-1]:
                undetcoeff = _undetermined_coefficients_match(r[-1], x)
                matching_hints["nth_linear_constant_coeff_variation_of_parameters"] = r
                matching_hints["nth_linear_constant_coeff_variation_of_parameters" + \
                    "_Integral"] = r
                if undetcoeff['test']:
                    r['trialset'] = undetcoeff['trialset']
                    matching_hints["nth_linear_constant_coeff_undetermined_" + \
                        "coefficients"] = r
            # Homogeneous case: F(x) is identically 0
            else:
                matching_hints["nth_linear_constant_coeff_homogeneous"] = r


    # Order keys based on allhints.
    retlist = []
    for i in allhints:
        if i in matching_hints:
            retlist.append(i)


    if dict:
        # Dictionaries are ordered arbitrarily, so we need to make note of which
        # hint would come first for dsolve().  In Python 3, this should be replaced
        # with an ordered dictionary.
        matching_hints["default"] = None
        matching_hints["ordered_hints"] = tuple(retlist)
        for i in allhints:
            if i in matching_hints:
                matching_hints["default"] = i
                break
        return matching_hints
    else:
        return tuple(retlist)

@vectorize(0)
def odesimp(eq, func, order, hint):
    r"""
    Simplifies ODEs, including trying to solve for func and running
    constantsimp().

    It may use knowledge of the type of solution that that hint returns
    to apply additional simplifications.

    It also attempts to integrate any Integrals in the expression, if
    the hint is not an "_Integral" hint.

    This function should have no effect on expressions returned by
    dsolve(), as dsolve already calls odesimp(), but the individual hint
    functions do not call odesimp (because the dsolve() wrapper does).
    Therefore, this function is designed for mainly internal use.

    **Example**
        >>> from sympy import sin, symbols, dsolve, pprint, Function
        >>> from sympy.solvers.ode import odesimp
        >>> x , u2, C1= symbols('x,u2,C1')
        >>> f = Function('f')

        >>> eq = dsolve(x*f(x).diff(x) - f(x) - x*sin(f(x)/x), f(x),
        ... hint='1st_homogeneous_coeff_subs_indep_div_dep_Integral',
        ... simplify=False)
        >>> pprint(eq)
                      x
                     ----
                     f(x)
                       /
                      |
                      |   /      /1 \    \
                      |  -|u2*sin|--| + 1|
           /f(x)\     |   \      \u2/    /
        log|----| -   |  ----------------- d(u2) = 0
           \ C1 /     |       2    /1 \
                      |     u2 *sin|--|
                      |            \u2/
                      |
                     /

        >>  pprint(odesimp(eq, f(x), 1,
        ... hint='1st_homogeneous_coeff_subs_indep_div_dep'
        ... )) # (this is slow, so we skip)
            x
        --------- = C1
           /f(x)\
        tan|----|
           \2*x /

    """
    x = func.args[0]
    f = func.func
    C1 = Symbol('C1')

    # First, integrate if the hint allows it.
    eq = _handle_Integral(eq, func, order, hint)
    assert isinstance(eq, Equality)

    # Second, clean up the arbitrary constants.
    # Right now, nth linear hints can put as many as 2*order constants in an
    # expression.  If that number grows with another hint, the third argument
    # here should be raised accordingly, or constantsimp() rewritten to handle
    # an arbitrary number of constants.
    eq = constantsimp(eq, x, 2*order)

    # Lastly, now that we have cleaned up the expression, try solving for func.
    # When RootOf is implemented in solve(), we will want to return a RootOf
    # everytime instead of an Equality.

    # Get the f(x) on the left if possible.
    if eq.rhs == func and not eq.lhs.has(func):
        eq = [Eq(eq.rhs, eq.lhs)]

    # make sure we are working with lists of solutions in simplified form.
    if eq.lhs == func and not eq.rhs.has(func):
        # The solution is already solved
        eq = [eq]

        # special simplification of the rhs
        if hint.startswith("nth_linear_constant_coeff"):
            # Collect terms to make the solution look nice.
            # This is also necessary for constantsimp to remove unnecessary terms
            # from the particular solution from variation of parameters
            global collectterms
            assert len(eq) == 1 and eq[0].lhs == f(x)
            sol = eq[0].rhs
            sol = expand_mul(sol)
            for i, reroot, imroot in collectterms:
                sol = collect(sol, x**i*exp(reroot*x)*sin(abs(imroot)*x))
                sol = collect(sol, x**i*exp(reroot*x)*cos(imroot*x))
            for i, reroot, imroot in collectterms:
                sol = collect(sol, x**i*exp(reroot*x))
            del collectterms
            eq[0] = Eq(f(x), sol)

    else:
        # The solution is not solved, so try to solve it
        try:
            eqsol = solve(eq, func)
            if eqsol == []:
                raise NotImplementedError
        except NotImplementedError:
            eq = [eq]
        else:
            def _expand(expr):
                numer, denom = expr.as_numer_denom()

                if denom.is_Add:
                    return expr
                else:
                    return powsimp(expr.expand(), combine='exp', deep=True)

            # XXX: the rest of odesimp() expects each ``t`` to be in a
            # specific normal form: rational expression with numerator
            # expanded, but with combined exponential functions (at
            # least in this setup all tests pass).
            eq = [Eq(f(x), _expand(t)) for t in eqsol]

        # special simplification of the lhs.
        if hint.startswith("1st_homogeneous_coeff"):
            for j, eqi in enumerate(eq):
                newi = logcombine(eqi, force=True)
                if newi.lhs.is_Function and newi.lhs.func is log and newi.rhs == 0:
                    newi = Eq(newi.lhs.args[0]/C1, C1)
                eq[j] = newi

    # We cleaned up the costants before solving to help the solve engine with
    # a simpler expression, but the solved expression could have introduced
    # things like -C1, so rerun constantsimp() one last time before returning.
    for i, eqi in enumerate(eq):
        eq[i] = constant_renumber(constantsimp(eqi, x, 2*order), 'C', 1, 2*order)

    # If there is only 1 solution, return it;
    # otherwise return the list of solutions.
    if len(eq) == 1:
        eq = eq[0]

    return eq

def checkodesol(ode, func, sol, order='auto', solve_for_func=True):
    """
    Substitutes sol for func in ode and checks that the result is 0.

    This only works when func is one function, like f(x).  sol can be a
    single solution or a list of solutions.  Either way, each solution
    must be an Equality instance (e.g., Eq(f(x), C1*cos(x) +
    C2*sin(x))).  If it is a list of solutions, it will return a list of
    the checkodesol() result for each solution.

    It tries the following methods, in order, until it finds zero
    equivalence:

        1. Substitute the solution for f in the original equation.  This
           only works if the ode is solved for f.  It will attempt to solve
           it first unless solve_for_func == False
        2. Take n derivatives of the solution, where n is the order of
           ode, and check to see if that is equal to the solution.  This
           only works on exact odes.
        3. Take the 1st, 2nd, ..., nth derivatives of the solution, each
           time solving for the derivative of f of that order (this will
           always be possible because f is a linear operator).  Then back
           substitute each derivative into ode in reverse order.

    This function returns a tuple.  The first item in the tuple is True
    if the substitution results in 0, and False otherwise. The second
    item in the tuple is what the substitution results in.  It should
    always be 0 if the first item is True. Note that sometimes this
    function will False, but with an expression that is identically
    equal to 0, instead of returning True.  This is because simplify()
    cannot reduce the expression to 0.  If an expression returned by
    this function vanishes identically, then sol really is a solution to
    ode.

    If this function seems to hang, it is probably because of a hard
    simplification.

    To use this function to test, test the first item of the tuple.

    **Examples**
        >>> from sympy import Eq, Function, checkodesol, symbols
        >>> x, C1 = symbols('x,C1')
        >>> f = Function('f')
        >>> checkodesol(f(x).diff(x), f(x), Eq(f(x), C1))
        (True, 0)
        >>> assert checkodesol(f(x).diff(x), f(x), Eq(f(x), C1))[0]
        >>> assert not checkodesol(f(x).diff(x), f(x), Eq(f(x), x))[0]
        >>> checkodesol(f(x).diff(x, 2), f(x), Eq(f(x), x**2))
        (False, 2)

    """
    if type(sol) in (tuple, list, set):
        return type(sol)(map(lambda i: checkodesol(ode, func, i, order=order,
            solve_for_func=solve_for_func), sol))
    if not func.is_Function or len(func.args) != 1:
        raise ValueError("func must be a function of one variable, not " + str(func))
    x = func.args[0]
    s = True
    testnum = 0
    if not isinstance(ode, Equality):
        ode = Eq(ode, 0)
    if not isinstance(sol, Equality):
        raise ValueError("sol must be an Equality, got " + str(sol))
    if order == 'auto':
        order = ode_order(ode, func)
    if solve_for_func and not (sol.lhs == func and not sol.rhs.has(func)) and not \
        (sol.rhs == func and not sol.lhs.has(func)):
        try:
            solved = solve(sol, func)
            if solved == []:
                raise NotImplementedError
        except NotImplementedError:
            pass
        else:
            if len(solved) == 1:
                result = checkodesol(ode, func, Eq(func, solved[0]), \
                    order=order, solve_for_func=False)
            else:
                result = checkodesol(ode, func, [Eq(func, t) for t in solved],
                order=order, solve_for_func=False)

            return result

    while s:
        if testnum == 0:
            # First pass, try substituting a solved solution directly into the ode
            # This has the highest chance of succeeding.
            ode_diff = ode.lhs - ode.rhs

            if sol.lhs == func:
                s = sub_func_doit(ode_diff, func, sol.rhs)
            elif sol.rhs == func:
                s = sub_func_doit(ode_diff, func, sol.lhs)
            else:
                testnum += 1
                continue
            ss = simplify(s)
            if ss:
                # with the new numer_denom in power.py, if we do a simple
                # expansion then testnum == 0 verifies all solutions.
                s = ss.expand()
            else:
                s = 0
            testnum += 1
        elif testnum == 1:
            # Second pass. If we cannot substitute f, try seeing if the nth
            # derivative is equal, this will only work for odes that are exact,
            # by definition.
            s = simplify(trigsimp(diff(sol.lhs, x, order) - diff(sol.rhs, x, order)) - \
                trigsimp(ode.lhs) + trigsimp(ode.rhs))
#            s2 = simplify(diff(sol.lhs, x, order) - diff(sol.rhs, x, order) - \
#                ode.lhs + ode.rhs)
            testnum += 1
        elif testnum == 2:
            # Third pass. Try solving for df/dx and substituting that into the ode.
            # Thanks to Chris Smith for suggesting this method.  Many of the
            # comments below are his too.
            # The method:
            # - Take each of 1..n derivatives of the solution.
            # - Solve each nth derivative for d^(n)f/dx^(n)
            #   (the differential of that order)
            # - Back substitute into the ode in decreasing order
            #   (i.e., n, n-1, ...)
            # - Check the result for zero equivalence
            if sol.lhs == func and not sol.rhs.has(func):
                diffsols = {0:sol.rhs}
            elif sol.rhs == func and not sol.lhs.has(func):
                diffsols = {0:sol.lhs}
            else:
                diffsols = {}
            sol = sol.lhs - sol.rhs
            for i in range(1, order + 1):
                # Differentiation is a linear operator, so there should always
                # be 1 solution. Nonetheless, we test just to make sure.
                # We only need to solve once.  After that, we will automatically
                # have the solution to the differential in the order we want.
                if i == 1:
                    ds = sol.diff(x)
                    try:
                        sdf = solve(ds,func.diff(x, i))
                        if len(sdf) != 1:
                            raise NotImplementedError
                    except NotImplementedError:
                        testnum += 1
                        break
                    else:
                        diffsols[i] = sdf[0]
                else:
                    # This is what the solution says df/dx should be.
                    diffsols[i] = diffsols[i - 1].diff(x)

            # Make sure the above didn't fail.
            if testnum > 2:
                continue
            else:
                # Substitute it into ode to check for self consistency.
                lhs, rhs = ode.lhs, ode.rhs
                for i in range(order, -1, -1):
                    if i == 0 and 0 not in diffsols:
                        # We can only substitute f(x) if the solution was
                        # solved for f(x).
                        break
                    lhs = sub_func_doit(lhs, func.diff(x, i), diffsols[i])
                    rhs = sub_func_doit(rhs, func.diff(x, i), diffsols[i])
                    ode_or_bool = Eq(lhs,rhs)
                    ode_or_bool = simplify(ode_or_bool)

                    if isinstance(ode_or_bool, bool):
                        if ode_or_bool:
                            lhs = rhs = S.Zero
                    else:
                        lhs = ode_or_bool.lhs
                        rhs = ode_or_bool.rhs
                # No sense in overworking simplify--just prove the numerator goes to zero
                s = simplify(trigsimp((lhs-rhs).as_numer_denom()[0]))
                testnum += 1
        else:
            break

    if not s:
        return (True, s)
    elif s is True: # The code above never was able to change s
        raise NotImplementedError("Unable to test if " + str(sol) + \
            " is a solution to " + str(ode) + ".")
    else:
        return (False, s)


def ode_sol_simplicity(sol, func, trysolving=True):
    """
    Returns an extended integer representing how simple a solution to an
    ODE is.

    The following things are considered, in order from most simple to
    least:
    - sol is solved for func.
    - sol is not solved for func, but can be if passed to solve (e.g.,
    a solution returned by dsolve(ode, func, simplify=False)
    - If sol is not solved for func, then base the result on the length
    of sol, as computed by len(str(sol)).
    - If sol has any unevaluated Integrals, this will automatically be
    considered less simple than any of the above.

    This function returns an integer such that if solution A is simpler
    than solution B by above metric, then ode_sol_simplicity(sola, func)
    < ode_sol_simplicity(solb, func).

    Currently, the following are the numbers returned, but if the
    heuristic is ever improved, this may change.  Only the ordering is
    guaranteed.

    sol solved for func                        -2
    sol not solved for func but can be         -1
    sol is not solved or solvable for func     len(str(sol))
    sol contains an Integral                   oo

    oo here means the SymPy infinity, which should compare greater than
    any integer.

    If you already know solve() cannot solve sol, you can use
    trysolving=False to skip that step, which is the only potentially
    slow step.  For example, dsolve with the simplify=False flag should
    do this.

    If sol is a list of solutions, if the worst solution in the list
    returns oo it returns that, otherwise it returns len(str(sol)), that
    is, the length of the string representation of the whole list.

    **Examples**

    This function is designed to be passed to min as the key argument,
    such as min(listofsolutions, key=lambda i: ode_sol_simplicity(i, f(x))).

        >>> from sympy import symbols, Function, Eq, tan, cos, sqrt, Integral
        >>> from sympy.solvers.ode import ode_sol_simplicity
        >>> x, C1 = symbols('x,C1')
        >>> f = Function('f')

        >>> ode_sol_simplicity(Eq(f(x), C1*x**2), f(x))
        -2
        >>> ode_sol_simplicity(Eq(x**2 + f(x), C1), f(x))
        -1
        >>> ode_sol_simplicity(Eq(f(x), C1*Integral(2*x, x)), f(x))
        oo
        >>> # This is from dsolve(x*f(x).diff(x) - f(x) - x*sin(f(x)/x), \
        >>> # f(x), hint='1st_homogeneous_coeff_subs_indep_div_dep')
        >>> eq1 = Eq(x/tan(f(x)/(2*x)), C1)
        >>> # This is from the same ode with the
        >>> # '1st_homogeneous_coeff_subs_dep_div_indep' hint.
        >>> eq2 = Eq(x*sqrt(1 + cos(f(x)/x))/sqrt(-1 + cos(f(x)/x)), C1)
        >>> ode_sol_simplicity(eq1, f(x))
        23
        >>> min([eq1, eq2], key=lambda i: ode_sol_simplicity(i, f(x)))
        x/tan(f(x)/(2*x)) == C1

    """
    #TODO: write examples

    # See the docstring for the coercion rules.  We check easier (faster)
    # things here first, to save time.

    if iterable(sol):
        # See if there are Integrals
        for i in sol:
            if ode_sol_simplicity(i, func, trysolving=trysolving) == oo:
                return oo

        return len(str(sol))

    if sol.has(C.Integral):
        return oo

    # Next, try to solve for func.  This code will change slightly when RootOf
    # is implemented in solve().  Probably a RootOf solution should fall somewhere
    # between a normal solution and an unsolvable expression.

    # First, see if they are already solved
    if sol.lhs == func and not sol.rhs.has(func) or\
        sol.rhs == func and not sol.lhs.has(func):
            return -2
    # We are not so lucky, try solving manually
    if trysolving:
        try:
            sols = solve(sol, func)
            if sols == []:
                raise NotImplementedError
        except NotImplementedError:
            pass
        else:
            return -1

    # Finally, a naive computation based on the length of the string version
    # of the expression.  This may favor combined fractions because they
    # will not have duplicate denominators, and may slightly favor expressions
    # with fewer additions and subtractions, as those are separated by spaces
    # by the printer.

    # Additional ideas for simplicity heuristics are welcome, like maybe
    # checking if a equation has a larger domain, or if constantsimp has
    # introduced arbitrary constants numbered higher than the order of a
    # given ode that sol is a solution of.
    return len(str(sol))


@vectorize(0)
def constantsimp(expr, independentsymbol, endnumber, startnumber=1,
    symbolname='C'):
    """
    Simplifies an expression with arbitrary constants in it.

    This function is written specifically to work with dsolve(), and is
    not intended for general use.

    Simplification is done by "absorbing" the arbitrary constants in to
    other arbitrary constants, numbers, and symbols that they are not
    independent of.

    The symbols must all have the same name with numbers after it, for
    example, C1, C2, C3.  The symbolname here would be 'C', the
    startnumber would be 1, and the end number would be 3.  If the
    arbitrary constants are independent of the variable x, then the
    independent symbol would be x.  There is no need to specify the
    dependent function, such as f(x), because it already has the
    independent symbol, x, in it.

    Because terms are "absorbed" into arbitrary constants and because
    constants are renumbered after simplifying, the arbitrary constants
    in expr are not necessarily equal to the ones of the same name in
    the returned result.

    If two or more arbitrary constants are added, multiplied, or raised
    to the power of each other, they are first absorbed together into a
    single arbitrary constant.  Then the new constant is combined into
    other terms if necessary.

    Absorption is done naively.  constantsimp() does not attempt to
    expand or simplify the expression first to obtain better absorption.
    So for example, exp(C1)*exp(x) will be simplified to C1*exp(x), but
    exp(C1 + x) will be left alone.

    Use constant_renumber() to renumber constants after simplification.
    Without using that function, simplified constants may end up
    having any numbering to them.

    In rare cases, a single constant can be "simplified" into two
    constants.  Every differential equation solution should have as many
    arbitrary constants as the order of the differential equation.  The
    result here will be technically correct, but it may, for example,
    have C1 and C2 in an expression, when C1 is actually equal to C2.
    Use your discretion in such situations, and also take advantage of
    the ability to use hints in dsolve().

    **Examples**
        >>> from sympy import symbols
        >>> from sympy.solvers.ode import constantsimp
        >>> C1, C2, C3, x, y = symbols('C1,C2,C3,x,y')
        >>> constantsimp(2*C1*x, x, 3)
        C1*x
        >>> constantsimp(C1 + 2 + x + y, x, 3)
        C1 + x
        >>> constantsimp(C1*C2 + 2 + x + y + C3*x, x, 3)
        C2 + C3*x + x

    """
    # This function works recursively.  The idea is that, for Mul,
    # Add, Pow, and Function, if the class has a constant in it, then
    # we can simplify it, which we do by recursing down and
    # simplifying up.  Otherwise, we can skip that part of the
    # expression.

    constantsymbols = [Symbol(symbolname+"%d" % t) for t in range(startnumber,
    endnumber + 1)]
    constantsymbols_set = set(constantsymbols)
    x = independentsymbol

    if isinstance(expr, Equality):
        # For now, only treat the special case where one side of the equation
        # is a constant
        if expr.lhs in constantsymbols_set:
            return Eq(expr.lhs, constantsimp(expr.rhs + expr.lhs, x, endnumber,
            startnumber, symbolname) - expr.lhs)
            # this could break if expr.lhs is absorbed into another constant,
            # but for now, the only solutions that return Eq's with a constant
            # on one side are first order.  At any rate, it will still be
            # technically correct.  The expression will just have too many
            # constants in it
        elif expr.rhs in constantsymbols_set:
            return Eq(constantsimp(expr.lhs + expr.rhs, x, endnumber,
            startnumber, symbolname) - expr.rhs, expr.rhs)
        else:
            return Eq(constantsimp(expr.lhs, x, endnumber, startnumber,
                symbolname), constantsimp(expr.rhs, x, endnumber,
                startnumber, symbolname))

    if type(expr) not in (Mul, Add, Pow) and not expr.is_Function:
        # We don't know how to handle other classes
        # This also serves as the base case for the recursion
        return expr
    elif not expr.has(*constantsymbols):
        return expr
    else:
        newargs = []
        hasconst = False
        isPowExp = False
        reeval = False
        for i in expr.args:
            if i not in constantsymbols:
                newargs.append(i)
            else:
                newconst = i
                hasconst = True
                if expr.is_Pow and i == expr.exp:
                    isPowExp = True

        for i in range(len(newargs)):
            isimp = constantsimp(newargs[i], x, endnumber, startnumber,
            symbolname)
            if isimp in constantsymbols:
                reeval = True
                hasconst = True
                newconst = isimp
                if expr.is_Pow and i == 1:
                    isPowExp = True
            newargs[i] = isimp
        if hasconst:
            newargs = [i for i in newargs if i.has(x)]
            if isPowExp:
                newargs = newargs + [newconst] # Order matters in this case
            else:
                newargs = [newconst] + newargs
        if expr.is_Pow and len(newargs) == 1:
            newargs.append(S.One)
        if expr.is_Function:
            if (len(newargs) == 0 or hasconst and len(newargs) == 1):
                return newconst
            else:
                newfuncargs = [constantsimp(t, x, endnumber, startnumber,
                symbolname) for t in expr.args]
                return expr.func(*newfuncargs)
        else:
            newexpr = expr.func(*newargs)
            if reeval:
                return constantsimp(newexpr, x, endnumber, startnumber,
                symbolname)
            else:
                return newexpr

def constant_renumber(expr, symbolname, startnumber, endnumber):
    """
    Renumber arbitrary constants in expr.

    This is a simple function that goes through and renumbers any Symbol
    with a name in the form symbolname + num where num is in the range
    from startnumber to endnumber.

    Symbols are renumbered based on Basic._compare_pretty, so they
    should be numbered roughly in the order that they appear in the
    final, printed expression.  Note that this ordering is based in part
    on hashes, so it can produce different results on different
    machines.

    The structure of this function is very similar to that of
    constantsimp().

    **Example**
        >>> from sympy import symbols, Eq, pprint
        >>> from sympy.solvers.ode import constant_renumber
        >>> x, C1, C2, C3 = symbols('x,C1,C2,C3')
        >>> pprint(C2 + C1*x + C3*x**2)
                        2
        C1*x + C2 + C3*x
        >>> pprint(constant_renumber(C2 + C1*x + C3*x**2, 'C', 1, 3))
                        2
        C1 + C2*x + C3*x

    """
    if type(expr) in (set, list, tuple):
        return type(expr)(map(lambda i: constant_renumber(i, symbolname=symbolname,
            startnumber=startnumber, endnumber=endnumber), expr))
    global newstartnumber
    newstartnumber = 1

    def _constant_renumber(expr, symbolname, startnumber, endnumber):
        """
        We need to have an internal recursive function so that
        newstartnumber maintains its values throughout recursive calls.

        """
        constantsymbols = [Symbol(symbolname+"%d" % t) for t in range(startnumber,
        endnumber + 1)]
        global newstartnumber

        if isinstance(expr, Equality):
            return Eq(_constant_renumber(expr.lhs, symbolname, startnumber, endnumber),
            _constant_renumber(expr.rhs, symbolname, startnumber, endnumber))

        if type(expr) not in (Mul, Add, Pow) and not expr.is_Function and\
        not expr.has(*constantsymbols):
            # Base case, as above.  We better hope there aren't constants inside
            # of some other class, because they won't be renumbered.
            return expr
        elif expr in constantsymbols:
            # Renumbering happens here
            newconst = Symbol(symbolname + str(newstartnumber))
            newstartnumber += 1
            return newconst
        else:
            if expr.is_Function or expr.is_Pow:
                return expr.func(*[_constant_renumber(x, symbolname, startnumber,
                endnumber) for x in expr.args])
            else:
                sortedargs = list(expr.args)
                # make a mapping to send all constantsymbols to S.One and use
                # that to make sure that term ordering is not dependent on
                # the indexed value of C
                C_1 = [(ci, S.One) for ci in constantsymbols]
                sortedargs.sort(key=cmp_to_key(lambda x, y:\
                               Basic._compare_pretty(x.subs(C_1), y.subs(C_1))))
                return expr.func(*[_constant_renumber(x, symbolname, startnumber,
                endnumber) for x in sortedargs])


    return _constant_renumber(expr, symbolname, startnumber, endnumber)


def _handle_Integral(expr, func, order, hint):
    """
    Converts a solution with Integrals in it into an actual solution.

    For most hints, this simply runs expr.doit()

    """
    x = func.args[0]
    f = func.func
    if hint == "1st_exact":
        global exactvars
        x0 = exactvars['x0']
        y0 = exactvars['y0']
        y = exactvars['y']
        tmpsol = expr.lhs.doit()
        sol = 0
        assert tmpsol.is_Add
        for i in tmpsol.args:
            if x0 not in i and y0 not in i:
                sol += i
        assert sol != 0
        sol = Eq(sol.subs(y, f(x)),expr.rhs) # expr.rhs == C1
        del exactvars
    elif hint == "1st_exact_Integral":
        # FIXME: We still need to back substitute y
        # y = exactvars['y']
        # sol = expr.subs(y, f(x))
        # For now, we are going to have to return an expression with f(x) replaced
        # with y.  Substituting results in the y's in the second integral
        # becoming f(x), which prevents the integral from being evaluatable.
        # For example, Integral(cos(f(x)), (x, x0, x)).  If there were a way to
        # do inert substitution, that could maybe be used here instead.
        del exactvars
        sol = expr
    elif hint == "nth_linear_constant_coeff_homogeneous":
        sol = expr
    elif not hint.endswith("_Integral"):
        sol = expr.doit()
    else:
        sol = expr
    return sol

def ode_order(expr, func):
    """
    Returns the order of a given ODE with respect to func.

    This function is implemented recursively.

    **Examples**
        >>> from sympy import Function, ode_order
        >>> from sympy.abc import x
        >>> f, g = map(Function, ['f', 'g'])
        >>> ode_order(f(x).diff(x, 2) + f(x).diff(x)**2 +
        ... f(x).diff(x), f(x))
        2
        >>> ode_order(f(x).diff(x, 2) + g(x).diff(x, 3), f(x))
        2
        >>> ode_order(f(x).diff(x, 2) + g(x).diff(x, 3), g(x))
        3

    """
    a = Wild('a', exclude=[func])

    order = 0
    if isinstance(expr, Derivative) and expr.args[0] == func:
        order = len(expr.variables)
    else:
        for arg in expr.args:
            if isinstance(arg, Derivative) and arg.args[0] == func:
                order = max(order, len(arg.variables))
            elif expr.match(a):
                order = 0
            else :
                for arg1 in arg.args:
                    order = max(order, ode_order(arg1, func))

    return order

# FIXME: replace the general solution in the docstring with
# dsolve(equation, hint='1st_exact_Integral').  You will need to be able
# to have assumptions on P and Q that dP/dy = dQ/dx.
def ode_1st_exact(eq, func, order, match):
    r"""
    Solves 1st order exact ordinary differential equations.

    A 1st order differential equation is called exact if it is the total
    differential of a function. That is, the differential equation
    P(x, y)dx + Q(x, y)dy = 0 is exact if there is some function F(x, y)
    such that P(x, y) = dF/dx and Q(x, y) = dF/dy (d here refers to the
    partial derivative).  It can be shown that a necessary and
    sufficient condition for a first order ODE to be exact is that
    dP/dy = dQ/dx.  Then, the solution will be as given below::

        >>> from sympy import Function, Eq, Integral, symbols, pprint
        >>> x, y, t, x0, y0, C1= symbols('x,y,t,x0,y0,C1')
        >>> P, Q, F= map(Function, ['P', 'Q', 'F'])
        >>> pprint(Eq(Eq(F(x, y), Integral(P(t, y), (t, x0, x)) +
        ... Integral(Q(x0, t), (t, y0, y))), C1))
                    x                y
                    /                /
                   |                |
        F(x, y) =  |  P(t, y) dt +  |  Q(x0, t) dt = C1
                   |                |
                  /                /
                  x0               y0

    Where the first partials of P and Q exist and are continuous in a
    simply connected region.

    A note: SymPy currently has no way to represent inert substitution on
    an expression, so the hint '1st_exact_Integral' will return an integral
    with dy.  This is supposed to represent the function that you are
    solving for.

    **Example**
        >>> from sympy import Function, dsolve, cos, sin
        >>> from sympy.abc import x
        >>> f = Function('f')
        >>> dsolve(cos(f(x)) - (x*sin(f(x)) - f(x)**2)*f(x).diff(x),
        ... f(x), hint='1st_exact')
        x*cos(f(x)) + f(x)**3/3 == C1

    **References**
        - http://en.wikipedia.org/wiki/Exact_differential_equation
        - M. Tenenbaum & H. Pollard, "Ordinary Differential Equations",
          Dover 1963, pp. 73

        # indirect doctest

    """
    x = func.args[0]
    f = func.func
    r = match # d+e*diff(f(x),x)
    C1 = Symbol('C1')
    x0 = Dummy('x0')
    y0 = Dummy('y0')
    global exactvars # This is the only way to pass these dummy variables to
    # _handle_Integral
    exactvars = {'y0':y0, 'x0':x0, 'y':r['y']}
    # If we ever get a Constant class, x0 and y0 should be constants, I think
    sol = C.Integral(r[r['e']].subs(x,x0),(r['y'],y0,f(x)))+C.Integral(r[r['d']],(x,x0,x))
    return Eq(sol, C1)


def ode_1st_homogeneous_coeff_best(eq, func, order, match):
    r"""
    Returns the best solution to an ODE from the two hints
    '1st_homogeneous_coeff_subs_dep_div_indep' and
    '1st_homogeneous_coeff_subs_indep_div_dep'.

    This is as determined by ode_sol_simplicity().

    See the ode_1st_homogeneous_coeff_subs_indep_div_dep() and
    ode_1st_homogeneous_coeff_subs_dep_div_indep() docstrings for more
    information on these hints.  Note that there is no
    '1st_homogeneous_coeff_best_Integral' hint.

    **Example**
    ::
        >>> from sympy import Function, dsolve, pprint
        >>> from sympy.abc import x
        >>> f = Function('f')
        >>> pprint(dsolve(2*x*f(x) + (x**2 + f(x)**2)*f(x).diff(x), f(x),
        ... hint='1st_homogeneous_coeff_best'))
              ___________
             /     2
            /   3*x
           /   ----- + 1 *f(x) = C1
        3 /     2
        \/     f (x)

    **References**
        - http://en.wikipedia.org/wiki/Homogeneous_differential_equation
        - M. Tenenbaum & H. Pollard, "Ordinary Differential Equations",
          Dover 1963, pp. 59

        # indirect doctest

    """
    # There are two substitutions that solve the equation, u1=y/x and u2=x/y
    # They produce different integrals, so try them both and see which
    # one is easier.
    sol1 = ode_1st_homogeneous_coeff_subs_indep_div_dep(eq,
    func, order, match)
    sol2 = ode_1st_homogeneous_coeff_subs_dep_div_indep(eq,
    func, order, match)
    simplify = match.get('simplify', True)
    if simplify:
        sol1 = odesimp(sol1, func, order, "1st_homogeneous_coeff_subs_indep_div_dep")
        sol2 = odesimp(sol2, func, order, "1st_homogeneous_coeff_subs_dep_div_indep")
    return min([sol1, sol2], key=lambda x: ode_sol_simplicity(x, func,
        trysolving=not simplify))

def ode_1st_homogeneous_coeff_subs_dep_div_indep(eq, func, order, match):
    r"""
    Solves a 1st order differential equation with homogeneous coefficients
    using the substitution
    u1 = <dependent variable>/<independent variable>.

    This is a differential equation P(x, y) + Q(x, y)dy/dx = 0, that P
    and Q are homogeneous of the same order.  A function F(x, y) is
    homogeneous of order n if F(xt, yt) = t**n*F(x, y).  Equivalently,
    F(x, y) can be rewritten as G(y/x) or H(x/y).  See also the
    docstring of homogeneous_order().

    If the coefficients P and Q in the  differential equation above are
    homogeneous functions of the same order, then it can be shown that
    the substitution y = u1*x (u1 = y/x) will turn the differential
    equation into an equation separable in the variables x and u.  If
    h(u1) is the function that results from making the substitution
    u1 = f(x)/x on P(x, f(x)) and g(u2) is the function that results
    from the substitution on Q(x, f(x)) in the differential equation
    P(x, f(x)) + Q(x, f(x))*diff(f(x), x) = 0, then the general solution
    is::

        >>> from sympy import Function, dsolve, pprint
        >>> from sympy.abc import x
        >>> f, g, h = map(Function, ['f', 'g', 'h'])
        >>> genform = g(f(x)/x) + h(f(x)/x)*f(x).diff(x)
        >>> pprint(genform)
         /f(x)\    /f(x)\ d
        g|----| + h|----|*--(f(x))
         \ x  /    \ x  / dx
        >>> pprint(dsolve(genform, f(x),
        ... hint='1st_homogeneous_coeff_subs_dep_div_indep_Integral'))
                     f(x)
                     ----
                      x
                       /
                      |
                      |       -h(u1)
        log(C1*x) -   |  ---------------- d(u1) = 0
                      |  u1*h(u1) + g(u1)
                      |
                     /

    Where u1*h(u1) + g(u1) != 0 and x != 0.

    See also the docstrings of ode_1st_homogeneous_coeff_best() and
    ode_1st_homogeneous_coeff_subs_indep_div_dep().

    **Example**
    ::
        >>> from sympy import Function, dsolve
        >>> from sympy.abc import x
        >>> f = Function('f')
        >>> pprint(dsolve(2*x*f(x) + (x**2 + f(x)**2)*f(x).diff(x), f(x),
        ... hint='1st_homogeneous_coeff_subs_dep_div_indep'))
                ________________
               /           3
              /  3*f(x)   f (x)
        x*   /   ------ + -----  = C1
          3 /      x         3
          \/                x

    **References**
        - http://en.wikipedia.org/wiki/Homogeneous_differential_equation
        - M. Tenenbaum & H. Pollard, "Ordinary Differential Equations",
          Dover 1963, pp. 59

        # indirect doctest

    """
    x = func.args[0]
    f = func.func
    u1 = Dummy('u1') # u1 == f(x)/x
    r = match # d+e*diff(f(x),x)
    C1 = Symbol('C1')
    int = C.Integral((-r[r['e']]/(r[r['d']]+u1*r[r['e']])).subs({x:1, r['y']:u1}),
        (u1, None, f(x)/x))
    sol = logcombine(Eq(log(x), int + log(C1)), force=True)
    return sol

def ode_1st_homogeneous_coeff_subs_indep_div_dep(eq, func, order, match):
    r"""
    Solves a 1st order differential equation with homogeneous coefficients
    using the substitution
    u2 = <independent variable>/<dependent variable>.

    This is a differential equation P(x, y) + Q(x, y)dy/dx = 0, that P
    and Q are homogeneous of the same order.  A function F(x, y) is
    homogeneous of order n if F(xt, yt) = t**n*F(x, y).  Equivalently,
    F(x, y) can be rewritten as G(y/x) or H(x/y).  See also the
    docstring of homogeneous_order().

    If the coefficients P and Q in the  differential equation above are
    homogeneous functions of the same order, then it can be shown that
    the substitution x = u2*y (u2 = x/y) will turn the differential
    equation into an equation separable in the variables y and u2.  If
    h(u2) is the function that results from making the substitution
    u2 = x/f(x) on P(x, f(x)) and g(u2) is the function that results
    from the substitution on Q(x, f(x)) in the differential equation
    P(x, f(x)) + Q(x, f(x))*diff(f(x), x) = 0, then the general solution
    is:

    >>> from sympy import Function, dsolve, pprint
    >>> from sympy.abc import x
    >>> f, g, h = map(Function, ['f', 'g', 'h'])
    >>> genform = g(x/f(x)) + h(x/f(x))*f(x).diff(x)
    >>> pprint(genform)
     / x  \    / x  \ d
    g|----| + h|----|*--(f(x))
     \f(x)/    \f(x)/ dx
    >>> pprint(dsolve(genform, f(x),
    ... hint='1st_homogeneous_coeff_subs_indep_div_dep_Integral'))
                     x
                    ----
                    f(x)
                      /
                     |
                     |        g(u2)
    log(C1*f(x)) -   |  ----------------- d(u2) = 0
                     |  -u2*g(u2) - h(u2)
                     |
                    /

    Where u2*g(u2) + h(u2) != 0 and f(x) != 0.

    See also the docstrings of ode_1st_homogeneous_coeff_best() and
    ode_1st_homogeneous_coeff_subs_dep_div_indep().

    **Example**
        >>> from sympy import Function, pprint
        >>> from sympy.abc import x
        >>> f = Function('f')
        >>> pprint(dsolve(2*x*f(x) + (x**2 + f(x)**2)*f(x).diff(x), f(x),
        ... hint='1st_homogeneous_coeff_subs_indep_div_dep'))
              ___________
             /     2
            /   3*x
           /   ----- + 1 *f(x) = C1
        3 /     2
        \/     f (x)

    **References**
        - http://en.wikipedia.org/wiki/Homogeneous_differential_equation
        - M. Tenenbaum & H. Pollard, "Ordinary Differential Equations",
          Dover 1963, pp. 59

        # indirect doctest

    """
    x = func.args[0]
    f = func.func
    u2 = Dummy('u2') # u2 == x/f(x)
    r = match # d+e*diff(f(x),x)
    C1 = Symbol('C1')
    int = C.Integral(simplify((-r[r['d']]/(r[r['e']]+u2*r[r['d']])).subs({x:u2, r['y']:1})),
        (u2, None, x/f(x)))
    sol = logcombine(Eq(log(f(x)), int + log(C1)), force=True)
    return sol

# XXX: Should this function maybe go somewhere else?
def homogeneous_order(eq, *symbols):
    """
    Returns the order n if g is homogeneous and None if it is not
    homogeneous.

    Determines if a function is homogeneous and if so of what order.
    A function f(x,y,...) is homogeneous of order n if
    f(t*x,t*y,t*...) == t**n*f(x,y,...).

    If the function is of two variables, F(x, y), then f being
    homogeneous of any order is equivalent to being able to rewrite
    F(x, y) as G(x/y) or H(y/x).  This fact is used to solve 1st order
    ordinary differential equations whose coefficients are homogeneous
    of the same order (see the docstrings of
    ode.ode_1st_homogeneous_coeff_subs_indep_div_dep() and
    ode.ode_1st_homogeneous_coeff_subs_indep_div_dep()

    Symbols can be functions, but every argument of the function must be
    a symbol, and the arguments of the function that appear in the
    expression must match those given in the list of symbols.  If a
    declared function appears with different arguments than given in the
    list of symbols, None is returned.

    **Examples**
        >>> from sympy import Function, homogeneous_order, sqrt
        >>> from sympy.abc import x, y
        >>> f = Function('f')
        >>> homogeneous_order(f(x), f(x)) == None
        True
        >>> homogeneous_order(f(x,y), f(y, x), x, y) == None
        True
        >>> homogeneous_order(f(x), f(x), x)
        1
        >>> homogeneous_order(x**2*f(x)/sqrt(x**2+f(x)**2), x, f(x))
        2
        >>> homogeneous_order(x**2+f(x), x, f(x)) == None
        True

    """
    if eq.has(log):
        eq = logcombine(eq, force=True)
    return _homogeneous_order(eq, *symbols)

def _homogeneous_order(eq, *symbols):
    """
    The real work for homogeneous_order.

    This runs as a separate function call so that logcombine doesn't
    endlessly put back together what homogeneous_order is trying to take
    apart.
    """
    if not symbols:
        raise ValueError("homogeneous_order: no symbols were given.")

    n = set()

    # Replace all functions with dummy variables

    for i in symbols:
        if i.is_Function:
            if not all([j in symbols for j in i.args]):
                return None
            else:
                dummyvar = numbered_symbols(prefix='d', cls=Dummy).next()
                eq = eq.subs(i, dummyvar)
                symbols = list(symbols)
                symbols.remove(i)
                symbols.append(dummyvar)
                symbols = tuple(symbols)

    # The following are not supported
    if eq.has(Order, Derivative):
        return None

    # These are all constants
    if type(eq) in (int, float) or eq.is_Number or eq.is_Integer or \
    eq.is_Rational or eq.is_NumberSymbol or eq.is_Float:
        return sympify(0)

    # Break the equation into additive parts
    if eq.is_Add:
        s = set()
        for i in eq.args:
            s.add(_homogeneous_order(i, *symbols))
        if len(s) != 1:
            return None
        else:
            n = s

    if eq.is_Pow:
        if not eq.exp.is_number:
            return None
        o = _homogeneous_order(eq.base, *symbols)
        if o == None:
            return None
        else:
            n.add(sympify(o*eq.exp))

    t = Dummy('t', positive=True) # It is sufficient that t > 0
    r = Wild('r', exclude=[t])
    a = Wild('a', exclude=[t])
    eqs = eq.subs(dict(zip(symbols,(t*i for i in symbols))))

    if eqs.is_Mul:
        if t not in eqs:
            n.add(sympify(0))
        else:
            m = eqs.match(r*t**a)
            if m:
                n.add(sympify(m[a]))
            else:
                s = 0
                for i in eq.args:
                    o = _homogeneous_order(i, *symbols)
                    if o == None:
                        return None
                    else:
                        s += o
                n.add(sympify(s))

    if eq.is_Function:
        if eq.func is log:
            # The only possibility to pull a t out of a function is a power in
            # a logarithm.  This is very likely due to calling of logcombine().
            args = Mul.make_args(eq.args[0])
            if all(i.is_Pow for i in args):
                base = 1
                expos = set()
                for pow in args:
                    if sign(pow.exp).is_negative:
                        s = -1
                    else:
                        s = 1
                    expos.add(s*pow.exp)
                    base *= pow.base**s
                if len(expos) != 1:
                    return None
                else:
                    return _homogeneous_order(expos.pop()*log(base), *symbols)
            else:
                if _homogeneous_order(eq.args[0], *symbols) == 0:
                    return sympify(0)
                else:
                    return None
        else:
            if _homogeneous_order(eq.args[0], *symbols) == 0:
                return sympify(0)
            else:
                return None

    if len(n) != 1 or n == None:
        return None
    else:
        return n.pop()

    return None



def ode_1st_linear(eq, func, order, match):
    r"""
    Solves 1st order linear differential equations.

    These are differential equations of the form dy/dx _ P(x)*y = Q(x).
    These kinds of differential equations can be solved in a general
    way.  The integrating factor exp(Integral(P(x), x)) will turn the
    equation into a separable equation.  The general solution is::

        >>> from sympy import Function, dsolve, Eq, pprint, diff, sin
        >>> from sympy.abc import x
        >>> f, P, Q = map(Function, ['f', 'P', 'Q'])
        >>> genform = Eq(f(x).diff(x) + P(x)*f(x), Q(x))
        >>> pprint(genform)
                    d
        P(x)*f(x) + --(f(x)) = Q(x)
                    dx
        >>> pprint(dsolve(genform, f(x), hint='1st_linear_Integral'))
               /       /                   \
               |      |                    |
               |      |         /          |     /
               |      |        |           |    |
               |      |        | P(x) dx   |  - | P(x) dx
               |      |        |           |    |
               |      |       /            |   /
        f(x) = |C1 +  | Q(x)*e           dx|*e
               |      |                    |
               \     /                     /


    **Example**
        >>> f = Function('f')
        >>> pprint(dsolve(Eq(x*diff(f(x), x) - f(x), x**2*sin(x)),
        ... f(x), '1st_linear'))
        f(x) = x*(C1 - cos(x))

    **References**
        - http://en.wikipedia.org/wiki/Linear_differential_equation#First_order_equation
        - M. Tenenbaum & H. Pollard, "Ordinary Differential Equations",
          Dover 1963, pp. 92

        # indirect doctest

    """
    x = func.args[0]
    f = func.func
    r = match # a*diff(f(x),x) + b*f(x) + c
    C1 = Symbol('C1')
    t = exp(C.Integral(r[r['b']]/r[r['a']], x))
    tt = C.Integral(t*(-r[r['c']]/r[r['a']]), x)
    return Eq(f(x),(tt + C1)/t)

def ode_Bernoulli(eq, func, order, match):
    r"""
    Solves Bernoulli differential equations.

    These are equations of the form dy/dx + P(x)*y = Q(x)*y**n, n != 1.
    The substitution w = 1/y**(1-n) will transform an equation of this
    form into one that is linear (see the docstring of
    ode_1st_linear()).  The general solution is::

        >>> from sympy import Function, dsolve, Eq, pprint
        >>> from sympy.abc import x, n
        >>> f, P, Q = map(Function, ['f', 'P', 'Q'])
        >>> genform = Eq(f(x).diff(x) + P(x)*f(x), Q(x)*f(x)**n)
        >>> pprint(genform)
                    d                n
        P(x)*f(x) + --(f(x)) = Q(x)*f (x)
                    dx
        >>> pprint(dsolve(genform, f(x), hint='Bernoulli_Integral')) #doctest: +SKIP
                                                                                       1
                                                                                      ----
                                                                                     1 - n
               //                /                            \                     \
               ||               |                             |                     |
               ||               |                  /          |             /       |
               ||               |                 |           |            |        |
               ||               |        (1 - n)* | P(x) dx   |  (-1 + n)* | P(x) dx|
               ||               |                 |           |            |        |
               ||               |                /            |           /         |
        f(x) = ||C1 + (-1 + n)* | -Q(x)*e                   dx|*e                   |
               ||               |                             |                     |
               \\               /                            /                     /


    Note that when n = 1, then the equation is separable (see the
    docstring of ode_separable()).

    >>> pprint(dsolve(Eq(f(x).diff(x) + P(x)*f(x), Q(x)*f(x)), f(x),
    ... hint='separable_Integral'))
     f(x)
       /
      |                /
      |  1            |
      |  - dy = C1 +  | (-P(x) + Q(x)) dx
      |  y            |
      |              /
     /


    **Example**
        >>> from sympy import Function, dsolve, Eq, pprint, log
        >>> from sympy.abc import x
        >>> f = Function('f')

        >>> pprint(dsolve(Eq(x*f(x).diff(x) + f(x), log(x)*f(x)**2),
        ... f(x), hint='Bernoulli'))
                        1
        f(x) = -------------------
                 /     log(x)   1\
               x*|C1 + ------ + -|
                 \       x      x/

    **References**
        - http://en.wikipedia.org/wiki/Bernoulli_differential_equation
        - M. Tenenbaum & H. Pollard, "Ordinary Differential Equations",
          Dover 1963, pp. 95

        # indirect doctest

    """
    x = func.args[0]
    f = func.func
    r = match # a*diff(f(x),x) + b*f(x) + c*f(x)**n, n != 1
    C1 = Symbol('C1')
    t = exp((1-r[r['n']])*C.Integral(r[r['b']]/r[r['a']],x))
    tt = (r[r['n']]-1)*C.Integral(t*r[r['c']]/r[r['a']],x)
    return Eq(f(x),((tt + C1)/t)**(1/(1-r[r['n']])))

def ode_Riccati_special_minus2(eq, func, order, match):
    r"""
    The general Riccati equation has the form dy/dx = f(x)*y**2 + g(x)*y + h(x).
    While it does not have a general solution [1], the "special" form,
    dy/dx = a*y**2 - b*x**c, does have solutions in many cases [2]. This routine
    returns a solution for a*dy/dx = b*y**2 + c*y/x + d/x**2 that is obtained by
    using a suitable change of variables to reduce it to the special form and is
    valid when neither a nor b are zero and either c or d is zero.

    >>> from sympy.abc import x, y, a, b, c, d
    >>> from sympy.solvers.ode import dsolve, checkodesol
    >>> from sympy import pprint, Function
    >>> f = Function('f')
    >>> y = f(x)
    >>> genform = a*y.diff(x) - (b*y**2 + c*y/x + d/x**2)
    >>> sol = dsolve(genform, y)
    >>> pprint(sol)
             /                                 /        __________________      \\
            |           __________________    |       /                2        ||
            |          /                2     |     \/  4*b*d - (a + c)  *log(x)||
           -|a + c - \/  4*b*d - (a + c)  *tan|C1 + ----------------------------||
            \                                 \                 2*a             //
    f(x) = -----------------------------------------------------------------------
                                            2*b*x

    >>> checkodesol(genform, y, sol, order=1)[0]
    True

    References:
    [1] http://www.maplesoft.com/support/help/Maple/view.aspx?path=odeadvisor/Riccati
    [2] http://eqworld.ipmnet.ru/en/solutions/ode/ode0106.pdf -
        http://eqworld.ipmnet.ru/en/solutions/ode/ode0123.pdf
    """

    x = func.args[0]
    f = func.func
    r = match # a2*diff(f(x),x) + b2*f(x) + c2*f(x)/x + d2/x**2
    a2, b2, c2, d2 = [r[r[s]] for s in 'a2 b2 c2 d2'.split()]
    C1 = Symbol('C1')
    mu = sqrt(4*d2*b2 - (a2 - c2)**2)
    return Eq(f(x), (a2 - c2 - mu*tan(mu/(2*a2)*log(x)+C1))/(2*b2*x))

def ode_Liouville(eq, func, order, match):
    r"""
    Solves 2nd order Liouville differential equations.

    The general form of a Liouville ODE is
    d^2y/dx^2 + g(y)*(dy/dx)**2 + h(x)*dy/dx.  The general solution is::
        >>> from sympy import Function, dsolve, Eq, pprint, diff
        >>> from sympy.abc import x
        >>> f, g, h = map(Function, ['f', 'g', 'h'])
        >>> genform = Eq(diff(f(x),x,x) + g(f(x))*diff(f(x),x)**2 +
        ... h(x)*diff(f(x),x), 0)
        >>> pprint(genform)
                        2                    2
                d                d          d
        g(f(x))*--(f(x))  + h(x)*--(f(x)) + ---(f(x)) = 0
                dx               dx           2
                                            dx
        >>> pprint(dsolve(genform, f(x), hint='Liouville_Integral'))
                                          f(x)
                  /                     /
                 |                     |
                 |     /               |     /
                 |    |                |    |
                 |  - | h(x) dx        |    | g(y) dy
                 |    |                |    |
                 |   /                 |   /
        C1 + C2* | e            dx +   |  e           dy = 0
                 |                     |
                /                     /

    **Example**
    ::
        >>> from sympy import Function, dsolve, Eq, pprint
        >>> from sympy.abc import x
        >>> f = Function('f')
        >>> pprint(dsolve(diff(f(x), x, x) + diff(f(x), x)**2/f(x) +
        ... diff(f(x), x)/x, f(x), hint='Liouville'))
                   ___   ________________           ___   ________________
        [f(x) = -\/ 2 *\/ C1 + C2*log(x) , f(x) = \/ 2 *\/ C1 + C2*log(x) ]


    **References**
        - Goldstein and Braun, "Advanced Methods for the Solution of
          Differential Equations", pp. 98
        - http://www.maplesoft.com/support/help/Maple/view.aspx?path=odeadvisor/Liouville

        # indirect doctest

    """
    # Liouville ODE f(x).diff(x, 2) + g(f(x))*(f(x).diff(x, 2))**2 + h(x)*f(x).diff(x)
    # See Goldstein and Braun, "Advanced Methods for the Solution of
    # Differential Equations", pg. 98, as well as
    # http://www.maplesoft.com/support/help/view.aspx?path=odeadvisor/Liouville
    x = func.args[0]
    f = func.func
    r = match # f(x).diff(x, 2) + g*f(x).diff(x)**2 + h*f(x).diff(x)
    y = r['y']
    C1 = Symbol('C1')
    C2 = Symbol('C2')
    int = C.Integral(exp(C.Integral(r['g'], y)), (y, None, f(x)))
    sol = Eq(int + C1*C.Integral(exp(-C.Integral(r['h'], x)), x) + C2, 0)
    return sol


def _nth_linear_match(eq, func, order):
    """
    Matches a differential equation to the linear form:

    a_n(x)y^(n) + ... + a_1(x)y' + a_0(x)y + B(x) = 0

    Returns a dict of order:coeff terms, where order is the order of the
    derivative on each term, and coeff is the coefficient of that
    derivative.  The key -1 holds the function B(x). Returns None if
    the ode is not linear.  This function assumes that func has already
    been checked to be good.

    **Examples**
        >>> from sympy import Function, cos, sin
        >>> from sympy.abc import x
        >>> from sympy.solvers.ode import _nth_linear_match
        >>> f = Function('f')
        >>> _nth_linear_match(f(x).diff(x, 3) + 2*f(x).diff(x) +
        ... x*f(x).diff(x, 2) + cos(x)*f(x).diff(x) + x - f(x) -
        ... sin(x), f(x), 3)
        {-1: x - sin(x), 0: -1, 1: cos(x) + 2, 2: x, 3: 1}
        >>> _nth_linear_match(f(x).diff(x, 3) + 2*f(x).diff(x) +
        ... x*f(x).diff(x, 2) + cos(x)*f(x).diff(x) + x - f(x) -
        ... sin(f(x)), f(x), 3) == None
        True

    """
    x = func.args[0]
    one_x = set([x])
    terms = dict([(i, S.Zero) for i in range(-1, order+1)])
    for i in Add.make_args(eq):
        if not i.has(func):
            terms[-1] += i
        else:
            c, f = i.as_independent(func)
            if not ((isinstance(f, Derivative) and set(f.variables) == one_x) or\
                    f == func):
                return None
            else:
                terms[len(f.args[1:])] += c
    return terms

def ode_nth_linear_constant_coeff_homogeneous(eq, func, order, match, returns='sol'):
    """
    Solves an nth order linear homogeneous differential equation with
    constant coefficients.

    This is an equation of the form a_n*f(x)^(n) + a_(n-1)*f(x)^(n-1) +
    ... + a1*f'(x) + a0*f(x) = 0

    These equations can be solved in a general manner, by taking the
    roots of the characteristic equation a_n*m**n + a_(n-1)*m**(n-1) +
    ... + a1*m + a0 = 0.  The solution will then be the sum of
    Cn*x**i*exp(r*x) terms, for each  where Cn is an arbitrary constant,
    r is a root of the characteristic equation and i is is one of each
    from 0 to the multiplicity of the root - 1 (for example, a root 3 of
    multiplicity 2 would create the terms C1*exp(3*x) + C2*x*exp(3*x)).
    The exponential is usually expanded for complex roots using Euler's
    equation exp(I*x) = cos(x) + I*sin(x).  Complex roots always come in
    conjugate pars in polynomials with real coefficients, so the two
    roots will be represented (after simplifying the constants) as
    exp(a*x)*(C1*cos(b*x) + C2*sin(b*x)).

    If SymPy cannot find exact roots to the characteristic equation, a
    RootOf instance will be return in its stead.

    >>> from sympy import Function, dsolve, Eq
    >>> from sympy.abc import x
    >>> f = Function('f')
    >>> dsolve(f(x).diff(x, 5) + 10*f(x).diff(x) - 2*f(x), f(x),
    ... hint='nth_linear_constant_coeff_homogeneous')
    ... # doctest: +NORMALIZE_WHITESPACE
    f(x) == C1*exp(x*RootOf(_x**5 + 10*_x - 2, 0)) + \
    C2*exp(x*RootOf(_x**5 + 10*_x - 2, 1)) + \
    C3*exp(x*RootOf(_x**5 + 10*_x - 2, 2)) + \
    C4*exp(x*RootOf(_x**5 + 10*_x - 2, 3)) + \
    C5*exp(x*RootOf(_x**5 + 10*_x - 2, 4))

    Note that because this method does not involve integration, there is
    no 'nth_linear_constant_coeff_homogeneous_Integral' hint.

    The following is for internal use:

    - returns = 'sol' returns the solution to the ODE.
    - returns = 'list' returns a list of linearly independent
      solutions, for use with non homogeneous solution methods like
      variation of parameters and undetermined coefficients.  Note that,
      though the solutions should be linearly independent, this function
      does not explicitly check that.  You can do "assert
      simplify(wronskian(sollist)) != 0" to check for linear independence.
      Also, "assert len(sollist) == order" will need to pass.
    - returns = 'both', return a dictionary {'sol':solution to ODE,
      'list': list of linearly independent solutions}.

    **Example**
        >>> from sympy import Function, dsolve, pprint
        >>> from sympy.abc import x
        >>> f = Function('f')
        >>> pprint(dsolve(f(x).diff(x, 4) + 2*f(x).diff(x, 3) -
        ... 2*f(x).diff(x, 2) - 6*f(x).diff(x) + 5*f(x), f(x),
        ... hint='nth_linear_constant_coeff_homogeneous'))
                            x                            -2*x
        f(x) = (C1 + C2*x)*e  + (C3*cos(x) + C4*sin(x))*e


    **References**
        - http://en.wikipedia.org/wiki/Linear_differential_equation
            section: Nonhomogeneous_equation_with_constant_coefficients
        - M. Tenenbaum & H. Pollard, "Ordinary Differential Equations",
          Dover 1963, pp. 211

        # indirect doctest

    """
    x = func.args[0]
    f = func.func
    r = match

    # A generator of constants
    constants = numbered_symbols(prefix='C', cls=Symbol, start=1)

    # First, set up characteristic equation.
    chareq, symbol = S.Zero, Dummy('x')

    for i in r.keys():
        if type(i) == str or i < 0:
            pass
        else:
            chareq += r[i]*symbol**i

    chareq = Poly(chareq, symbol)
    chareqroots = [ RootOf(chareq, k) for k in xrange(chareq.degree()) ]

    # Create a dict root: multiplicity or charroots
    charroots = {}
    for root in chareqroots:
        if root in charroots:
            charroots[root] += 1
        else:
            charroots[root] = 1
    gsol = S(0)
    # We need keep track of terms so we can run collect() at the end.
    # This is necessary for constantsimp to work properly.
    global collectterms
    collectterms = []
    for root, multiplicity in charroots.items():
        for i in range(multiplicity):
            if isinstance(root, RootOf):
                gsol += exp(root*x)*constants.next()
                assert multiplicity == 1
                collectterms = [(0, root, 0)] + collectterms
            else:
                reroot = re(root)
                imroot = im(root)
                gsol += x**i*exp(reroot*x)*(constants.next()*sin(abs(imroot)*x) \
                + constants.next()*cos(imroot*x))
                # This ordering is important
                collectterms = [(i, reroot, imroot)] + collectterms
    if returns == 'sol':
        return Eq(f(x), gsol)
    elif returns in ('list' 'both'):
        # Create a list of (hopefully) linearly independent solutions
        gensols = []
        # Keep track of when to use sin or cos for nonzero imroot
        for i, reroot, imroot in collectterms:
            if imroot == 0:
                gensols.append(x**i*exp(reroot*x))
            else:
                if x**i*exp(reroot*x)*sin(abs(imroot)*x) in gensols:
                    gensols.append(x**i*exp(reroot*x)*cos(imroot*x))
                else:
                    gensols.append(x**i*exp(reroot*x)*sin(abs(imroot)*x))
        if returns == 'list':
            return gensols
        else:
            return {'sol':Eq(f(x), gsol), 'list':gensols}
    else:
        raise ValueError('Unknown value for key "returns".')

def ode_nth_linear_constant_coeff_undetermined_coefficients(eq, func, order, match):
    r"""
    Solves an nth order linear differential equation with constant
    coefficients using the method of undetermined coefficients.

    This method works on differential equations of the form a_n*f(x)^(n)
    + a_(n-1)*f(x)^(n-1) + ... + a1*f'(x) + a0*f(x) = P(x), where P(x)
    is a function that has a finite number of linearly independent
    derivatives.

    Functions that fit this requirement are finite sums functions of the
    form a*x**i*exp(b*x)*sin(c*x + d) or a*x**i*exp(b*x)*cos(c*x + d),
    where i is a non-negative integer and a, b, c, and d are constants.
    For example any polynomial in x, functions like x**2*exp(2*x),
    x*sin(x), and exp(x)*cos(x) can all be used.  Products of sin's and
    cos's have a finite number of derivatives, because they can be
    expanded into sin(a*x) and cos(b*x) terms.  However, SymPy currently
    cannot do that expansion, so you will need to manually rewrite the
    expression in terms of the above to use this method.  So, for example,
    you will need to manually convert sin(x)**2 into (1 + cos(2*x))/2 to
    properly apply the method of undetermined coefficients on it.

    This method works by creating a trial function from the expression
    and all of its linear independent derivatives and substituting them
    into the original ODE.  The coefficients for each term will be a
    system of linear equations, which are be solved for and substituted,
    giving the solution.  If any of the trial functions are linearly
    dependent on the solution to the homogeneous equation, they are
    multiplied by sufficient x to make them linearly independent.

    **Example**
        >>> from sympy import Function, dsolve, pprint, exp, cos
        >>> from sympy.abc import x
        >>> f = Function('f')
        >>> pprint(dsolve(f(x).diff(x, 2) + 2*f(x).diff(x) + f(x) -
        ... 4*exp(-x)*x**2 + cos(2*x), f(x),
        ... hint='nth_linear_constant_coeff_undetermined_coefficients'))
               /             4\
               |            x |  -x   4*sin(2*x)   3*cos(2*x)
        f(x) = |C1 + C2*x + --|*e   - ---------- + ----------
               \            3 /           25           25

    **References**
        - http://en.wikipedia.org/wiki/Method_of_undetermined_coefficients
        - M. Tenenbaum & H. Pollard, "Ordinary Differential Equations",
          Dover 1963, pp. 221

        # indirect doctest

    """
    gensol = ode_nth_linear_constant_coeff_homogeneous(eq, func, order, match,
        returns='both')
    match.update(gensol)
    return _solve_undetermined_coefficients(eq, func, order, match)

def _solve_undetermined_coefficients(eq, func, order, match):
    """
    Helper function for the method of undetermined coefficients.

    See the ode_nth_linear_constant_coeff_undetermined_coefficients()
    docstring for more information on this method.

    match should be a dictionary that has the following keys:
    'list' - A list of solutions to the homogeneous equation, such as
         the list returned by
         ode_nth_linear_constant_coeff_homogeneous(returns='list')
    'sol' - The general solution, such as the solution returned by
        ode_nth_linear_constant_coeff_homogeneous(returns='sol')
    'trialset' - The set of trial functions as returned by
        _undetermined_coefficients_match()['trialset']

    """
    x = func.args[0]
    f = func.func
    r = match
    coeffs = numbered_symbols('a', cls=Dummy)
    coefflist = []
    gensols = r['list']
    gsol = r['sol']
    trialset = r['trialset']
    notneedset = set([])
    newtrialset = set([])
    global collectterms
    if len(gensols) != order:
        raise NotImplementedError("Cannot find " + str(order) + \
        " solutions to the homogeneous equation nessesary to apply " + \
        "undetermined coefficients to " + str(eq) + " (number of terms != order)")
    usedsin = set([])
    mult = 0 # The multiplicity of the root
    getmult = True
    for i, reroot, imroot in collectterms:
        if getmult:
            mult = i + 1
            getmult = False
        if i == 0:
            getmult = True
        if imroot:
            # Alternate between sin and cos
            if (i, reroot) in usedsin:
                check = x**i*exp(reroot*x)*cos(imroot*x)
            else:
                check = x**i*exp(reroot*x)*sin(abs(imroot)*x)
                usedsin.add((i, reroot))
        else:
            check = x**i*exp(reroot*x)

        if check in trialset:
            # If an element of the trial function is already part of the homogeneous
            # solution, we need to multiply by sufficient x to make it linearly
            # independent.  We also don't need to bother checking for the coefficients
            # on those elements, since we already know it will be 0.
            while True:
                if check*x**mult in trialset:
                    mult += 1
                else:
                    break
            trialset.add(check*x**mult)
            notneedset.add(check)

    newtrialset = trialset - notneedset

    trialfunc = 0
    for i in newtrialset:
        c = coeffs.next()
        coefflist.append(c)
        trialfunc += c*i

    eqs = sub_func_doit(eq, f(x), trialfunc)

    coeffsdict = dict(zip(trialset, [0]*(len(trialset) + 1)))

    eqs = expand_mul(eqs)

    for i in Add.make_args(eqs):
        s = separatevars(i, dict=True, symbols=[x])
        coeffsdict[s[x]] += s['coeff']

    coeffvals = solve(coeffsdict.values(), coefflist)

    if not coeffvals:
        raise NotImplementedError("Could not solve " + str(eq) + " using the " + \
            " method of undetermined coefficients (unable to solve for coefficients).")

    psol = trialfunc.subs(coeffvals)

    return Eq(f(x), gsol.rhs + psol)

def _undetermined_coefficients_match(expr, x):
    """
    Returns a trial function match if undetermined coefficients can be
    applied to expr, and None otherwise.

    A trial expression can be found for an expression for use with the
    method of undetermined coefficients if the expression is an
    additive/multiplicative combination of constants, polynomials in x
    (the independent variable of expr), sin(a*x + b), cos(a*x + b), and
    exp(a*x) terms (in other words, it has a finite number of linearly
    independent derivatives).

    Note that you may still need to multiply each term returned here by
    sufficient x to make it linearly independent with the solutions to
    the homogeneous equation.

    This is intended for internal use by undetermined_coefficients
    hints.

    SymPy currently has no way to convert sin(x)**n*cos(y)**m into a sum
    of only sin(a*x) and cos(b*x) terms, so these are not implemented.
    So, for example, you will need to manually convert sin(x)**2 into
    (1 + cos(2*x))/2 to properly apply the method of undetermined
    coefficients on it.

    **Example**
        >>> from sympy import log, exp
        >>> from sympy.solvers.ode import _undetermined_coefficients_match
        >>> from sympy.abc import x
        >>> _undetermined_coefficients_match(9*x*exp(x) + exp(-x), x)
        {'test': True, 'trialset': set([x*exp(x), exp(-x), exp(x)])}
        >>> _undetermined_coefficients_match(log(x), x)
        {'test': False}

    """
    from sympy import S
    a = Wild('a', exclude=[x])
    b = Wild('b', exclude=[x])
    expr = powsimp(expr, combine='exp') # exp(x)*exp(2*x + 1) => exp(3*x + 1)
    retdict = {}
    def _test_term(expr, x):
        """
        Test if expr fits the proper form for undetermined coefficients.
        """
        if expr.is_Add:
            return all([_test_term(i, x) for i in expr.args])
        elif expr.is_Mul:
            if expr.has(sin, cos):
                foundtrig = False
                # Make sure that there is only one trig function in the args.
                # See the docstring.
                for i in expr.args:
                    if i.has(sin, cos):
                        if foundtrig:
                            return False
                        else:
                            foundtrig = True
            return all([_test_term(i, x) for i in expr.args])
        elif expr.is_Function:
            if expr.func in (sin, cos, exp):
                if expr.args[0].match(a*x + b):
                    return True
                else:
                    return False
            else:
                return False
        elif expr.is_Pow and expr.base.is_Symbol and expr.exp.is_Integer and \
            expr.exp >= 0:
            return True
        elif expr.is_Pow and expr.base.is_number:
            if expr.exp.match(a*x + b):
                return True
            else:
                return False
        elif expr.is_Symbol or expr.is_Number:
            return True
        else:
            return False

    def _get_trial_set(expr, x, exprs=set([])):
        """
        Returns a set of trial terms for undetermined coefficients.

        The idea behind undetermined coefficients is that the terms
        expression repeat themselves after a finite number of
        derivatives, except for the coefficients (they are linearly
        dependent).  So if we collect these, we should have the terms of
        our trial function.
        """
        def _remove_coefficient(expr, x):
            """
            Returns the expression without a coefficient.

            Similar to expr.as_independent(x)[1], except it only works
            multiplicatively.
            """
            # I was using the below match, but it doesn't always put all of the
            # coefficient in c.  c.f. 2**x*6*exp(x)*log(2)
            # The below code is probably cleaner anyway.
#            c = Wild('c', exclude=[x])
#            t = Wild('t')
#            r = expr.match(c*t)
            term = S.One
            if expr.is_Mul:
                for i in expr.args:
                    if i.has(x):
                        term *= i
            elif expr.has(x):
                term = expr
            return term

        expr = expand_mul(expr)
        if expr.is_Add:
            for term in expr.args:
                if _remove_coefficient(term, x) in exprs:
                    pass
                else:
                    exprs.add(_remove_coefficient(term, x))
                    exprs = exprs.union(_get_trial_set(term, x, exprs))
        else:
            term = _remove_coefficient(expr, x)
            tmpset = exprs.union(set([term]))
            oldset = set([])
            while tmpset != oldset:
                # If you get stuck in this loop, then _test_term is probably broken
                oldset = tmpset.copy()
                expr = expr.diff(x)
                term = _remove_coefficient(expr, x)
                if term.is_Add:
                    tmpset = tmpset.union(_get_trial_set(term, x, tmpset))
                else:
                    tmpset.add(term)
            exprs = tmpset
        return exprs



    retdict['test'] = _test_term(expr, x)
    if retdict['test']:
        # Try to generate a list of trial solutions that will have the undetermined
        # coefficients.  Note that if any of these are not linearly independent
        # with any of the solutions to the homogeneous equation, then they will
        # need to be multiplied by sufficient x to make them so.  This function
        # DOES NOT do that (it doesn't even look at the homogeneous equation).
        retdict['trialset'] = _get_trial_set(expr, x)

    return retdict

def ode_nth_linear_constant_coeff_variation_of_parameters(eq, func, order, match):
    r"""
    Solves an nth order linear differential equation with constant
    coefficients using the method of undetermined coefficients.

    This method works on any differential equations of the form
    f(x)^(n) + a_(n-1)*f(x)^(n-1) + ... + a1*f'(x) + a0*f(x) = P(x).

    This method works by assuming that the particular solution takes the
    form Sum(c_i(x)*y_i(x), (x, 1, n)), where y_i is the ith solution to
    the homogeneous equation.  The solution is then solved using
    Wronskian's and Cramer's Rule.  The particular solution is given by
    Sum(Integral(W_i(x)/W(x), x)*y_i(x), (x, 1, n)), where W(x) is the
    Wronskian of the fundamental system (the system of n linearly
    independent solutions to the homogeneous equation), and W_i(x) is
    the Wronskian of the fundamental system with the ith column replaced
    with [0, 0, ..., 0, P(x)].

    This method is general enough to solve any nth order inhomogeneous
    linear differential equation with constant coefficients, but
    sometimes SymPy cannot simplify the Wronskian well enough to
    integrate it.  If this method hangs, try using the
    'nth_linear_constant_coeff_variation_of_parameters_Integral' hint
    and simplifying the integrals manually.  Also, prefer using
    'nth_linear_constant_coeff_undetermined_coefficients' when it
    applies, because it doesn't use integration, making it faster and
    more reliable.

    Warning, using simplify=False with
    'nth_linear_constant_coeff_variation_of_parameters' in dsolve()
    may cause it to hang, because it will not attempt to simplify
    the Wronskian before integrating.  It is recommended that you only
    use simplify=False with
    'nth_linear_constant_coeff_variation_of_parameters_Integral' for
    this method, especially if the solution to the homogeneous
    equation has trigonometric functions in it.

    **Example**
        >>> from sympy import Function, dsolve, pprint, exp, log
        >>> from sympy.abc import x
        >>> f = Function('f')
        >>> pprint(dsolve(f(x).diff(x, 3) - 3*f(x).diff(x, 2) +
        ... 3*f(x).diff(x) - f(x) - exp(x)*log(x), f(x),
        ... hint='nth_linear_constant_coeff_variation_of_parameters'))
               /                2    3 /log(x)   11\\  x
        f(x) = |C1 + C2*x + C3*x  + x *|------ - --||*e
               \                       \  6      36//

    **References**
        - http://en.wikipedia.org/wiki/Variation_of_parameters
        - http://planetmath.org/encyclopedia/VariationOfParameters.html
        - M. Tenenbaum & H. Pollard, "Ordinary Differential Equations",
          Dover 1963, pp. 233

        # indirect doctest

    """
    gensol = ode_nth_linear_constant_coeff_homogeneous(eq, func, order, match,
        returns='both')
    match.update(gensol)
    return _solve_variation_of_parameters(eq, func, order, match)

def _solve_variation_of_parameters(eq, func, order, match):
    """
    Helper function for the method of variation of parameters.

    See the ode_nth_linear_constant_coeff_variation_of_parameters()
    docstring for more information on this method.

    match should be a dictionary that has the following keys:
    'list' - A list of solutions to the homogeneous equation, such as
         the list returned by
         ode_nth_linear_constant_coeff_homogeneous(returns='list')
    'sol' - The general solution, such as the solution returned by
        ode_nth_linear_constant_coeff_homogeneous(returns='sol')


    """
    x = func.args[0]
    f = func.func
    r = match
    psol = 0
    gensols = r['list']
    gsol = r['sol']
    wr = wronskian(gensols, x)

    if r.get('simplify', True):
        wr = simplify(wr) # We need much better simplification for some ODEs.
                          # See issue 1563, for example.

        # To reduce commonly occuring sin(x)**2 + cos(x)**2 to 1
        wr = trigsimp(wr, deep=True, recursive=True)
    if not wr:
        # The wronskian will be 0 iff the solutions are not linearly independent.
        raise NotImplementedError("Cannot find " + str(order) + \
        " solutions to the homogeneous equation nessesary to apply " + \
        "variation of parameters to " + str(eq) + " (Wronskian == 0)")
    if len(gensols) != order:
        raise NotImplementedError("Cannot find " + str(order) + \
        " solutions to the homogeneous equation nessesary to apply " + \
        "variation of parameters to " + str(eq) + " (number of terms != order)")
    negoneterm = (-1)**(order)
    for i in gensols:
        psol += negoneterm*C.Integral(wronskian(filter(lambda x: x != i, \
        gensols), x)*r[-1]/wr, x)*i/r[order]
        negoneterm *= -1

    if r.get('simplify', True):
        psol = simplify(psol)
        psol = trigsimp(psol, deep=True)
    return Eq(f(x), gsol.rhs + psol)

def ode_separable(eq, func, order, match):
    r"""
    Solves separable 1st order differential equations.

    This is any differential equation that can be written as
    P(y)*dy/dx = Q(x). The solution can then just be found by
    rearranging terms and integrating:
    Integral(P(y), y) = Integral(Q(x), x). This hint uses separatevars()
    as its back end, so if a separable equation is not caught by this
    solver, it is most likely the fault of that function. separatevars()
    is smart enough to do most expansion and factoring necessary to
    convert a separable equation F(x, y) into the proper form P(x)*Q(y).
    The general solution is::

        >>> from sympy import Function, dsolve, Eq, pprint
        >>> from sympy.abc import x
        >>> a, b, c, d, f = map(Function, ['a', 'b', 'c', 'd', 'f'])
        >>> genform = Eq(a(x)*b(f(x))*f(x).diff(x), c(x)*d(f(x)))
        >>> pprint(genform)
                     d
        a(x)*b(f(x))*--(f(x)) = c(x)*d(f(x))
                     dx
        >>> pprint(dsolve(genform, f(x), hint='separable_Integral'))
             f(x)
           /                  /
          |                  |
          |  b(y)            | c(x)
          |  ---- dy = C1 +  | ---- dx
          |  d(y)            | a(x)
          |                  |
         /                  /

    **Example**
    ::
        >>> from sympy import Function, dsolve, Eq
        >>> from sympy.abc import x
        >>> f = Function('f')
        >>> pprint(dsolve(Eq(f(x)*f(x).diff(x) + x, 3*x*f(x)**2), f(x),
        ... hint='separable'))
           /   2       \         2
        log\3*f (x) - 1/        x
        ---------------- = C1 + --
               6                2


    **Reference**
        - M. Tenenbaum & H. Pollard, "Ordinary Differential Equations",
          Dover 1963, pp. 52

        # indirect doctest

    """
    x = func.args[0]
    f = func.func
    C1 = Symbol('C1')
    r = match # {'m1':m1, 'm2':m2, 'y':y}
    return Eq(C.Integral(r['m2']['coeff']*r['m2'][r['y']]/r['m1'][r['y']],
        (r['y'], None, f(x))), C.Integral(-r['m1']['coeff']*r['m1'][x]/
        r['m2'][x], x)+C1)

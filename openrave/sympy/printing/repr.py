"""
A Printer for generating executable code.

The most important function here is srepr that returns a string so that the
relation eval(srepr(expr))=expr holds in an appropriate environment.
"""

from printer import Printer
from sympy.core import Basic
import sympy.mpmath.libmp as mlib
from sympy.mpmath.libmp import prec_to_dps, repr_dps

class ReprPrinter(Printer):
    printmethod = "_sympyrepr"

    _default_settings = {
        "order": None
    }

    def reprify(self, args, sep):
        return sep.join([self.doprint(item) for item in args])

    def emptyPrinter(self, expr):
        if isinstance(expr, str):
            return expr
        elif hasattr(expr, "__srepr__"):
            return expr.__srepr__()
        elif hasattr(expr, "args") and hasattr(expr.args, "__iter__"):
            l = []
            for o in expr.args:
                l.append(self._print(o))
            return expr.__class__.__name__ + '(%s)' % ', '.join(l)
        elif hasattr(expr, "__module__") and hasattr(expr, "__name__"):
            return "<'%s.%s'>" % (expr.__module__, expr.__name__)
        else:
            return str(expr)

    def _print_Add(self, expr, order=None):
        args = self._as_ordered_terms(expr, order=order)
        args = map(self._print, args)
        return "Add(%s)" % ", ".join(args)

    def _print_Function(self, expr):
        r = self._print(expr.func)
        r+= '(%s)' % ', '.join([self._print(a) for a in expr.args])
        return r

    def _print_FunctionClass(self, expr):
        return 'Function(%r)'%(expr.__name__)

    def _print_GeometryEntity(self, expr):
        # GeometryEntity is special -- its base is tuple
        return type(expr).__name__ + srepr(tuple(expr))

    def _print_Half(self, expr):
        return 'Rational(1, 2)'

    def _print_RationalConstant(self, expr):
        return str(expr)

    def _print_AtomicExpr(self, expr):
        return str(expr)

    def _print_NumberSymbol(self, expr):
        return str(expr)

    def _print_Integer(self, expr):
        return 'Integer(%i)' % expr.p

    def _print_list(self, expr):
        return "[%s]" % self.reprify(expr, ", ")

    def _print_Matrix(self, expr):
        l = []
        for i in range(expr.rows):
            l.append([])
            for j in range(expr.cols):
                l[-1].append(expr[i,j])
        return '%s(%s)' % (expr.__class__.__name__, self._print(l))

    def _print_NaN(self, expr):
        return "nan"

    def _print_Rational(self, expr):
        return 'Rational(%s, %s)' % (self._print(expr.p), self._print(expr.q))

    def _print_Mul(self, expr, order=None):
        terms = expr.args
        if self.order != 'old':
            args = expr._new_rawargs(*terms).as_ordered_factors()
        else:
            args = terms

        args = map(self._print, args)
        return "Mul(%s)" % ", ".join(args)

    def _print_Fraction(self, expr):
        return 'Fraction(%s, %s)' % (self._print(expr.numerator), self._print(expr.denominator))

    def _print_Float(self, expr):
        dps = prec_to_dps(expr._prec)
        r = mlib.to_str(expr._mpf_, repr_dps(expr._prec))
        return "%s('%s', prec=%i)" % (expr.__class__.__name__, r, dps)

    def _print_Sum2(self, expr):
        return "Sum2(%s, (%s, %s, %s))" % (self._print(expr.f), self._print(expr.i),
                                           self._print(expr.a), self._print(expr.b))

    def _print_Symbol(self, expr):
        return "%s(%s)" % (expr.__class__.__name__, self._print(expr.name))

    def _print_Predicate(self, expr):
        return "%s(%s)" % (expr.__class__.__name__, self._print(expr.name))

    def _print_AppliedPredicate(self, expr):
        return "%s(%s, %s)" % (expr.__class__.__name__, expr.func, expr.arg)

    def _print_str(self, expr):
        return repr(expr)

    def _print_tuple(self, expr):
        if len(expr)==1:
            return "(%s,)" % self._print(expr[0])
        else:
            return "(%s)" % self.reprify(expr, ", ")

    def _print_WildFunction(self, expr):
        return "%s('%s')" % (expr.__class__.__name__, expr.name)

    def _print_AlgebraicNumber(self, expr):
        return "%s(%s, %s)" % (self.__class__.__name__,
            self._print(self.coeffs()), self._print(expr.root))

def srepr(expr, **settings):
    """return expr in repr form"""
    return ReprPrinter(settings).doprint(expr)

from sympy.core import S, C, Add
from sympy.printing.str import StrPrinter
from sympy.tensor import get_indices, get_contraction_structure

class AssignmentError(Exception):
    pass

class CodePrinter(StrPrinter):

    def _doprint_a_piece(self, expr, assign_to=None):
        # Here we print an expression that may contain Indexed objects, they
        # correspond to arrays in the generated code.  The low-level implementation
        # involves looping over array elements and possibly storing results in temporary
        # variables or accumulate it in the assign_to object.

        lhs_printed = self._print(assign_to)
        lines = []

        # Setup loops over non-dummy indices  --  all terms need these
        indices = self.get_expression_indices(expr, assign_to)
        openloop, closeloop = self._get_loop_opening_ending(indices)

        # Setup loops over dummy indices  --  each term needs separate treatment
        d = get_contraction_structure(expr)

        # terms with no summations first
        if None in d:
            text = CodePrinter.doprint(self, Add(*d[None]))
        else:
            # If all terms have summations we must initialize array to Zero
            text = CodePrinter.doprint(self, 0)
        # skip redundant assignments
        if text != lhs_printed:
            lines.extend(openloop)
            if assign_to is not None:
                text = self._get_statement("%s = %s" % (lhs_printed, text))
            lines.append(text)
            lines.extend(closeloop)

        for dummies in d:
            # then terms with summations
            if isinstance(dummies, tuple):
                indices = self._sort_optimized(dummies, expr)
                openloop_d, closeloop_d = self._get_loop_opening_ending(indices)

                for term in d[dummies]:
                    if term in d and not ([f.keys() for f in d[term]]
                            == [[None] for f in d[term]]):
                        # If one factor in the term has it's own internal
                        # contractions, those must be computed first.
                        # (temporary variables?)
                        raise NotImplementedError(
                                "FIXME: no support for contractions in factor yet")
                    else:

                        # We need the lhs expression as an accumulator for
                        # the loops, i.e
                        #
                        # for (int d=0; d < dim; d++){
                        #    lhs[] = lhs[] + term[][d]
                        # }           ^.................. the accumulator
                        #
                        # We check if the expression already contains the
                        # lhs, and raise an exception if it does, as that
                        # syntax is currently undefined.  FIXME: What would be
                        # a good interpretation?
                        if assign_to is None:
                            raise AssignmentError("need assignment variable for loops")
                        if term.has(assign_to):
                            raise(ValueError("FIXME: lhs present in rhs,\
                                this is undefined in CCodePrinter"))

                        lines.extend(openloop)
                        lines.extend(openloop_d)
                        text = "%s = %s" % (lhs_printed, CodePrinter.doprint(self, assign_to + term))
                        lines.append(self._get_statement(text))
                        lines.extend(closeloop_d)
                        lines.extend(closeloop)

        return lines

    def get_expression_indices(self, expr, assign_to):
        rinds, junk = get_indices(expr)
        linds, junk = get_indices(assign_to)

        # support broadcast of scalar
        if linds and not rinds:
            rinds = linds
        if rinds != linds:
            raise ValueError("lhs indices must match non-dummy"
                    " rhs indices in %s" % expr)

        return self._sort_optimized(rinds, assign_to)

    def _sort_optimized(self, indices, expr):

        if not indices:
            return []

        # determine optimized loop order by giving a score to each index
        # the index with the highest score are put in the innermost loop.
        score_table = {}
        for i in indices:
            score_table[i] = 0

        arrays = expr.atoms(C.Indexed)
        for arr in arrays:
            for p, ind in enumerate(arr.indices):
                try:
                    score_table[ind] += self._rate_index_position(p)
                except KeyError:
                    pass

        return sorted(indices, key=lambda x: score_table[x])

    def _print_NumberSymbol(self, expr):
        # A Number symbol that is not implemented here or with _printmethod
        # is registered and evaluated
        self._number_symbols.add((expr,
            self._print(expr.evalf(self._settings["precision"]))))
        return str(expr)

    def _print_Dummy(self, expr):
        # dummies must be printed as unique symbols
        return "%s_%i" %(expr.name, expr.dummy_index)  # Dummy

    _print_Catalan = _print_NumberSymbol
    _print_EulerGamma = _print_NumberSymbol
    _print_GoldenRatio = _print_NumberSymbol

    def _print_not_supported(self, expr):
        self._not_supported.add(expr)
        return self.emptyPrinter(expr)


    # The following can not be simply translated into C or Fortran
    _print_Basic = _print_not_supported
    _print_ComplexInfinity = _print_not_supported
    _print_Derivative = _print_not_supported
    _print_dict = _print_not_supported
    _print_ExprCondPair = _print_not_supported
    _print_GeometryEntity = _print_not_supported
    _print_Infinity = _print_not_supported
    _print_Integral = _print_not_supported
    _print_Interval = _print_not_supported
    _print_Limit = _print_not_supported
    _print_list = _print_not_supported
    _print_Matrix = _print_not_supported
    _print_DeferredVector = _print_not_supported
    _print_NaN = _print_not_supported
    _print_NegativeInfinity = _print_not_supported
    _print_Normal = _print_not_supported
    _print_Order = _print_not_supported
    _print_PDF = _print_not_supported
    _print_RootOf = _print_not_supported
    _print_RootsOf = _print_not_supported
    _print_RootSum = _print_not_supported
    _print_Sample = _print_not_supported
    _print_SparseMatrix = _print_not_supported
    _print_tuple = _print_not_supported
    _print_Uniform = _print_not_supported
    _print_Unit = _print_not_supported
    _print_Wild = _print_not_supported
    _print_WildFunction = _print_not_supported

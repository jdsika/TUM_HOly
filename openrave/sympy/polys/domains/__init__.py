"""Implementation of mathematical domains. """

from domain import Domain
from ring import Ring
from field import Field

from simpledomain import SimpleDomain
from compositedomain import CompositeDomain
from characteristiczero import CharacteristicZero

from finitefield import FiniteField
from integerring import IntegerRing
from rationalfield import RationalField
from realdomain import RealDomain

from pythonfinitefield import PythonFiniteField
from sympyfinitefield import SymPyFiniteField
from gmpyfinitefield import GMPYFiniteField

from pythonintegerring import PythonIntegerRing
from sympyintegerring import SymPyIntegerRing
from gmpyintegerring import GMPYIntegerRing

from pythonrationalfield import PythonRationalField
from sympyrationalfield import SymPyRationalField
from gmpyrationalfield import GMPYRationalField

from sympyrealdomain import SymPyRealDomain
from pythonrealdomain import PythonRealDomain
from mpmathrealdomain import MPmathRealDomain

from pythoncomplexdomain import PythonComplexDomain
from mpmathcomplexdomain import MPmathComplexDomain

from algebraicfield import AlgebraicField

from polynomialring import PolynomialRing
from fractionfield import FractionField

from expressiondomain import ExpressionDomain

FF_python = PythonFiniteField
FF_sympy = SymPyFiniteField
FF_gmpy = GMPYFiniteField

ZZ_python = PythonIntegerRing
ZZ_sympy = SymPyIntegerRing
ZZ_gmpy = GMPYIntegerRing

QQ_python = PythonRationalField
QQ_sympy = SymPyRationalField
QQ_gmpy = GMPYRationalField

RR_sympy = SymPyRealDomain
RR_python = PythonRealDomain
RR_mpmath = MPmathRealDomain

CC_python = PythonComplexDomain
CC_mpmath = MPmathComplexDomain

from pythonrationaltype import PythonRationalType

from groundtypes import HAS_GMPY

def _getenv(key, default=None):
    from os import getenv
    return getenv(key, default)

GROUND_TYPES = _getenv('SYMPY_GROUND_TYPES', 'auto').lower()

if GROUND_TYPES == 'auto':
    if HAS_GMPY:
        GROUND_TYPES = 'gmpy'
    else:
        GROUND_TYPES = 'python'

if GROUND_TYPES == 'gmpy' and not HAS_GMPY:
    from warnings import warn
    warn("gmpy library is not installed, switching to 'python' ground types")
    GROUND_TYPES = 'python'

_GROUND_TYPES_MAP = {
    'gmpy': (FF_gmpy, ZZ_gmpy(), QQ_gmpy()),
    'sympy': (FF_sympy, ZZ_sympy(), QQ_sympy()),
    'python': (FF_python, ZZ_python(), QQ_python()),
}

try:
    FF, ZZ, QQ = _GROUND_TYPES_MAP[GROUND_TYPES]
except KeyError:
    raise ValueError("invalid ground types: %s" % GROUND_TYPES)

GF = FF

RR = RR_mpmath()
CC = CC_mpmath()

EX = ExpressionDomain()


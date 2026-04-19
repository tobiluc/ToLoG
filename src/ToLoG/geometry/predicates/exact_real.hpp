#pragma once

namespace ToLoG
{

#if defined(TOLOG_EXACT_SINGLE_PRECISION)
typedef float exact_real;
#else
typedef double exact_real;
#endif

}

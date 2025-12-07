#include <ToLoG/predicates/ExactPredicates.h>

namespace ToLoG
{

PredicatesInitalizer PredicatesInitalizer::instance;
PredicatesInitalizer::PredicatesInitalizer() {
    exactinit(); // This must be called before using the predicates, otherwise - BOOOM!!!
}

}

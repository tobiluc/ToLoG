#include <ToLoG/predicates/ExactPredicates.hpp>

namespace ToLoG
{

PredicatesInitalizer PredicatesInitalizer::instance;
PredicatesInitalizer::PredicatesInitalizer() {
    exactinit(); // This must be called before using the predicates, otherwise - BOOOM!!!
}

}

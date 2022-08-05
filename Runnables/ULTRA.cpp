#include "Commands/CH.h"
#include "Commands/QueryBenchmark.h"
#include "Commands/ULTRAPreprocessing.h"

#include "../Helpers/Console/CommandLineParser.h"

#include "../Shell/Shell.h"
using namespace Shell;

int main(int argc, char** argv) {
    CommandLineParser clp(argc, argv);
    pinThreadToCoreId(clp.value<int>("core", 1));
    checkAsserts();
    ::Shell::Shell shell;
    new BuildCH(shell);
    new BuildCoreCH(shell);

    new ComputeStopToStopShortcuts(shell);
    new ComputeMcStopToStopShortcuts(shell);
    new ComputeMultimodalMcStopToStopShortcuts(shell);
    new RAPTORToTripBased(shell);
    new ComputeEventToEventShortcuts(shell);
    new ComputeMcEventToEventShortcuts(shell);
    new ComputeMultimodalMcEventToEventShortcuts(shell);
    new AugmentTripBasedShortcuts(shell);
    new ValidateStopToStopShortcuts(shell);
    new ValidateEventToEventShortcuts(shell);

    new RunTransitiveCSAQueries(shell);
    new RunDijkstraCSAQueries(shell);
    new RunHLCSAQueries(shell);
    new RunULTRACSAQueries(shell);

    new RunTransitiveRAPTORQueries(shell);
    new RunDijkstraRAPTORQueries(shell);
    new RunHLRAPTORQueries(shell);
    new RunULTRARAPTORQueries(shell);

    new RunTransitiveMcRAPTORQueries(shell);
    new RunMCRQueries(shell);
    new RunULTRAMcRAPTORQueries(shell);
    new RunTransitiveBoundedMcRAPTORQueries(shell);
    new RunUBMRAPTORQueries(shell);

    new RunTransitiveTripBasedQueries(shell);
    new RunULTRATripBasedQueries(shell);

    new RunULTRAMcTripBasedQueries(shell);
    new RunBoundedULTRAMcTripBasedQueries(shell);

    new RunMultimodalMCRQueries(shell);
    new RunMultimodalULTRAMcRAPTORQueries(shell);
    new RunUBMHydRAQueries(shell);
    new RunMultimodalUBMRAPTORQueries(shell);
    new RunMultimodalUBMHydRAQueries(shell);

    new ComputeTransferTimeSavings(shell);
    shell.run();
    return 0;
}

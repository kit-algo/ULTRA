#include <sched.h>

#include "../Helpers/Console/CommandLineParser.h"
#include "../Shell/Shell.h"
#include "Commands/DelayExperiments.h"

using namespace Shell;

int main(int argc, char** argv) {
    CommandLineParser clp(argc, argv);
    checkAsserts();
    Shell::Shell shell;
    new AnalyzeHeadwayDistribution(shell);
    new AnalyzeTransferSlacks(shell);
    new BuildFakeDelayData(shell);
    new GenerateDelayScenario(shell);
    new ValidateDelayULTRATripBased(shell);
    new RunDelayUpdatesWithReplacement(shell);
    new RunDelayUpdatesWithoutReplacement(shell);
    new GenerateDelayQueries(shell);
    new MeasureDelayULTRAQueryCoverage(shell);
    new MeasureHypotheticalDelayULTRAQueryCoverage(shell);
    new MeasureDelayULTRAQueryPerformance(shell);
    shell.run();
    return 0;
}

#include <sched.h>

#include "../Helpers/Console/CommandLineParser.h"
#include "../Shell/Shell.h"
#include "Commands/IsochroneExperiments.h"

using namespace Shell;

int main(int argc, char** argv) {
    CommandLineParser clp(argc, argv);
    checkAsserts();
    Shell::Shell shell;
    new ComputePartition(shell);
    new RunIsoPHASTPreprocessing(shell);
    new ValidateIsoPHASTQuery(shell);
    new ValidateIsoPHAST(shell);
    new ValidateIsoDijkstraCSA(shell);
    new ValidateIsoPHASTCSA(shell);
    new VisualizeWalkingIsochrone(shell);
    new VisualizeMultimodalIsochrone(shell);
    shell.run();
    return 0;
}

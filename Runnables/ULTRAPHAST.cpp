#include "Commands/BenchmarkULTRAPHAST.h"

#include "../Helpers/Console/CommandLineParser.h"
#include "../Helpers/MultiThreading.h"

#include "../Shell/Shell.h"
using namespace Shell;

int main(int argc, char** argv) {
    CommandLineParser clp(argc, argv);
    pinThreadToCoreId(clp.value<int>("core", 1));
    checkAsserts();
    ::Shell::Shell shell;

    new RunOneToAllDijkstraCSAQueriesToVertices(shell);
    new RunOneToManyDijkstraCSAQueriesToStops(shell);
    new RunUPCSAQueries(shell);
    new RunOneToAllDijkstraRAPTORQueriesToVertices(shell);
    new RunOneToManyDijkstraRAPTORQueriesToStops(shell);
    new RunUPRAPTORQueries(shell);
    new RunUPTBQueries(shell);
    new CreateBallTargetSets(shell);
    new BuildCoreCHForTargetSets(shell);
    new BuildUPCHForTargetSets(shell);
    new RunOneToManyDijkstraCSAQueriesToBall(shell);
    new RunUPCSAQueriesToBall(shell);
    new RunOneToManyDijkstraRAPTORQueriesToBall(shell);
    new RunUPRAPTORQueriesToBall(shell);

    shell.run();
    return 0;
}

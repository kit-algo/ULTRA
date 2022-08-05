#include "Commands/NetworkIO.h"
#include "Commands/NetworkTools.h"

#include "../Helpers/Console/CommandLineParser.h"
#include "../Helpers/MultiThreading.h"

#include "../Shell/Shell.h"
using namespace Shell;

int main(int argc, char** argv) {
    CommandLineParser clp(argc, argv);
    pinThreadToCoreId(clp.value<int>("core", 1));
    checkAsserts();
    ::Shell::Shell shell;
    new ParseGTFS(shell);
    new GTFSToIntermediate(shell);
    new IntermediateToCSA(shell);
    new IntermediateToRAPTOR(shell);
    new BuildMultimodalRAPTORData(shell);
    new AddModeToMultimodalRAPTORData(shell);
    new BuildMultimodalTripBasedData(shell);
    new AddModeToMultimodalTripBasedData(shell);
    new LoadDimacsGraph(shell);
    new DuplicateTrips(shell);
    new AddGraph(shell);
    new ReplaceGraph(shell);
    new ReduceGraph(shell);
    new ReduceToMaximumConnectedComponent(shell);
    new ReduceToMaximumConnectedComponentWithTransitive(shell);
    new ApplyBoundingBox(shell);
    new ApplyCustomBoundingBox(shell);
    new MakeOneHopTransfers(shell);
    new ApplyMaxTransferSpeed(shell);
    new ApplyConstantTransferSpeed(shell);
    shell.run();
    return 0;
}

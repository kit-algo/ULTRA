#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../CH/Query/BucketQuery.h"
#include "../CH/Query/CHQuery.h"

#include "../../Helpers/Vector/Vector.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/Container/ExternalKHeap.h"

namespace RAPTOR {

using DijkstraInitialTransfers = CH::Query<TransferGraph, false, false, true>;
using CoreCHInitialTransfers = CH::Query<CHGraph, true, false, true>;
using BucketCHInitialTransfers = CH::BucketQuery<CHGraph, true, false>;

}

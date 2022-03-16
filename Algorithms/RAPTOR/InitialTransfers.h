#pragma once

#include "../CH/Query/BucketQuery.h"
#include "../CH/Query/CHQuery.h"

namespace RAPTOR {

using DijkstraInitialTransfers = CH::Query<TransferGraph, false, false, true>;
using CoreCHInitialTransfers = CH::Query<CHGraph, true, false, true>;
using BucketCHInitialTransfers = CH::BucketQuery<CHGraph, true, false>;

}

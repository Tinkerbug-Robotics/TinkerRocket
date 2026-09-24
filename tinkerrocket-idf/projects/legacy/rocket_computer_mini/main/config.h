#pragma once

#include <stdint.h>
#include "driver/spi_common.h"

// Board revision comes ONLY from -DTR_MINI_BOARD=<n>; the build-dir name is
// cosmetic (same rule as the base station's TR_BS_BOARD). Default is the
// first and so far only board.
#ifndef TR_MINI_BOARD
#define TR_MINI_BOARD 1
#endif

// #413: the string form of the same number, for the flight log's one-byte
// revision code (board_identity::encodeRevCode). 'B' is the mini family — it
// is what TR_BOARD_SUFFIX already spells as "-b<n>".
#define TR_STRINGIFY_(x) #x
#define TR_STRINGIFY(x) TR_STRINGIFY_(x)
#define TR_BOARD_REV_STR "B" TR_STRINGIFY(TR_MINI_BOARD)

#if TR_MINI_BOARD == 1
#include "board/board_v1.h"
#else
#error "TR_MINI_BOARD must be 1"
#endif

// Policy lives here, split by ancestry — this project is a merge of two
// codebases, and keeping their constants in separate fragments preserves the
// line of sight back to the files they were ported from:
//   config_flight.inc — flight_computer/main/config.h descendants
//   config_comms.inc  — out_computer/main/config.h descendants
// Both are struct-body fragments (bare `static constexpr` members), textually
// included so that ported code's `config::NAME` references compile unchanged.
// ONLY pins and part-presence flags live in board_v1.h; everything
// rate/threshold/protocol-shaped belongs in one of these two fragments.
struct config : board_pins
{
#include "config_flight.inc"
#include "config_comms.inc"
};

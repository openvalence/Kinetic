// kinetic2/trace.hpp -- the planner's trace hooks: each expands to nothing
// unless the including build defines it first
// Constraints:
// - Undefined, a hook is ((void)0) and its arguments are never evaluated:
//   firmware compiles to the same code with or without these lines.
// - A build that defines one defines all seventeen, before any kinetic2 header
//   (wasm/kinetic2_trace.cpp, the playground's planner steps). A hook only
//   reads: it never writes planner state, never moves a decision.
// - No thread_local, no heap, no static locals behind a hook here; a tracing
//   build keeps its own buffer.
// See: playground/README.md "Planner steps"
#pragma once

#ifndef K2T_SOLVE
// handles.hpp
#define K2T_PCHIP(i, m, h1, h2, w1, w2, r) ((void)0)           // pchipAngle: its spans, weights and result
#define K2T_SWEEP(s, i, AL, BL, AR, BR, lo, hi, v) ((void)0)   // solveStyle: one G2 sweep at a through point, v the new angle
#define K2T_STYLE(k, n, smooth, phase) ((void)0)              // solveStyle: 0 the start angles, 1 after the G2 sweeps
#define K2T_ROUND(k, n, tag, round) ((void)0)                 // render: 0/2 after a solve, 1/3 after a nudge
#define K2T_PIECE(i, hit, L, R, pL) ((void)0)                 // nudge: piece i, memo hit or fit
#define K2T_PHASE(x) ((void)0)                                // nudge: 0 at the knot, 1 the full move, 10+s halving s, 30+q quarter q, 40 flat
#define K2T_RETRY() ((void)0)                                 // nudge: searched again without the end acceleration ask
#define K2T_JUDGE(f, q, pL, cs, ro, o) ((void)0)              // fitPiece: one length factor judged
#define K2T_FIT(pR, f) ((void)0)                              // nudge: one fit at a tested position
#define K2T_PIECE_END(i, L, R, f, dp, hold, canTrim) ((void)0) // nudge: what piece i keeps
// solver.hpp
#define K2T_SOLVE(origin, origin_us, n, c) ((void)0)          // solveWindow: a window from its origin
#define K2T_RUN(k, te, m, me, base) ((void)0)                 // renderRun: the knots of one run
#define K2T_PASS(pass, rescue) ((void)0)                      // renderRun: one render pass
#define K2T_PASS_END(k, built, me, again, over) ((void)0)     // renderRun: each built piece's worst ratio
#define K2T_RUN_END(out, me) ((void)0)                        // renderRun: the solved knots
#define K2T_CHASE(i, j, out) ((void)0)                        // solveWindow: samples i..j as one move
#define K2T_HARD(i, o) ((void)0)                              // solveWindow: a HARD knot
#endif

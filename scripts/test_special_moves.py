#!/usr/bin/env python3.8

from stockfish import Stockfish
import pprint

stockfish = Stockfish(parameters={"Threads": 2, "Minimum Thinking Time": 30})

# PROMOTION
# https://lichess.org/editor/1k6/6PR/8/8/8/8/8/3K4_w_-_-_0_1
stockfish.set_fen_position("1k6/6PR/8/8/8/8/8/3K4 w - - 0 1")

print(stockfish.get_best_move())

print(stockfish.get_evaluation())

pprint.pprint(stockfish.get_board_visual())

# EN PASSANT
# https://lichess.org/editor/r1b1k3/pppp2pp/8/5p2/4pP2/8/PPPP3P/R1B1K3_b_Qq_f3_0_1
stockfish.set_fen_position("r1b1k3/pppp2pp/8/5p2/4pP2/8/PPPP3P/R1B1K3 b Qq f3 0 1")

print(stockfish.get_best_move())

print(stockfish.get_evaluation())

pprint.pprint(stockfish.get_board_visual())

# CASTLING 1
# https://lichess.org/editor/5k2/ppppp1pp/8/8/8/8/PPPPP1PP/2BQK2R_w_K_-_0_1
stockfish.set_fen_position("5k2/ppppp1pp/8/8/8/8/PPPPP1PP/2BQK2R w K - 0 1")

print(stockfish.get_best_move())

print(stockfish.get_evaluation())

pprint.pprint(stockfish.get_board_visual())

# CASTLING 2
# https://lichess.org/editor/6k1/pppppp1p/8/8/8/8/PPPPPP1P/R3K3_w_Q_-_0_1
stockfish.set_fen_position("6k1/pppppp1p/8/8/8/8/PPPPPP1P/R3K3 w Q - 0 1")

print(stockfish.get_best_move())

print(stockfish.get_evaluation())

pprint.pprint(stockfish.get_board_visual())
#!/usr/bin/env python3.8

from stockfish import Stockfish
from Chessnut import Game
import fen_parser
import pprint

stockfish = Stockfish(parameters={"Threads": 2, "Minimum Thinking Time": 30})
chessgame = Game()

# PROMOTION
# https://lichess.org/editor/1k6/6PR/8/8/8/8/8/3K4_w_-_-_0_1
print(20*"=" + " PROMOTION " + 20*"=")
chessgame.set_fen("1k6/6PR/8/8/8/8/8/3K4 w - - 0 1")
stockfish.set_fen_position(chessgame.get_fen())
print(chessgame.get_fen())
print(stockfish.get_board_visual())

move = stockfish.get_best_move()
print(move)

#print(stockfish.get_evaluation())
#print(chessgame.get_fen())
chessgame.apply_move(move)
print(chessgame.get_fen())

stockfish.set_fen_position(chessgame.get_fen())
print(stockfish.get_board_visual())

# EN PASSANT
# https://lichess.org/editor/r1b1k3/pppp2pp/8/5p2/4pP2/8/PPPP3P/R1B1K3_b_Qq_f3_0_1
print(20*"=" + " EN PASSANT " + 20*"=")
chessgame.set_fen("r1b1k3/pppp2pp/8/5p2/4pP2/8/PPPP3P/R1B1K3 b Qq f3 0 1")
stockfish.set_fen_position(chessgame.get_fen())
print(chessgame.get_fen())
print(stockfish.get_board_visual())

move = stockfish.get_best_move()
print(move)

#print(stockfish.get_evaluation())
#print(chessgame.get_fen())
chessgame.apply_move(move)
print(chessgame.get_fen())

stockfish.set_fen_position(chessgame.get_fen())
print(stockfish.get_board_visual())

# CASTLING 1
# https://lichess.org/editor/5k2/ppppp1pp/8/8/8/8/PPPPP1PP/2BQK2R_w_K_-_0_1
print(20*"=" + " CASTLING K " + 20*"=")
chessgame.set_fen("5k2/ppppp1pp/8/8/8/8/PPPPP1PP/2BQK2R w K - 0 1")
stockfish.set_fen_position(chessgame.get_fen())
print(chessgame.get_fen())
print(stockfish.get_board_visual())

move = stockfish.get_best_move()
print(move)

#print(stockfish.get_evaluation())
#print(chessgame.get_fen())
chessgame.apply_move(move)
print(chessgame.get_fen())

stockfish.set_fen_position(chessgame.get_fen())
print(stockfish.get_board_visual())

# CASTLING 2
# https://lichess.org/editor/6k1/pppppp1p/8/8/8/8/PPPPPP1P/R3K3_w_Q_-_0_1
print(20*"=" + " CASTLING Q " + 20*"=")
chessgame.set_fen("6k1/pppppp1p/8/8/8/8/PPPPPP1P/R3K3 w Q - 0 1")
stockfish.set_fen_position(chessgame.get_fen())
print(chessgame.get_fen())
print(stockfish.get_board_visual())

move = stockfish.get_best_move()
print(move)

#print(stockfish.get_evaluation())
#print(chessgame.get_fen())
chessgame.apply_move(move)
print(chessgame.get_fen())

stockfish.set_fen_position(chessgame.get_fen())
print(stockfish.get_board_visual())

# Special tests
field = "g8"
piece = fen_parser.get_piece(chessgame.get_fen(), field)
print("Piece on %s is %s" % (field, piece))

diff = fen_parser.fen_diff("6k1/pppppp1p/8/8/8/8/PPPPPP1P/R3K3 w Q - 0 1", "6k1/pppppp1p/8/8/8/8/PPPPP1PP/2KR4 b - - 1 1")
print(diff)
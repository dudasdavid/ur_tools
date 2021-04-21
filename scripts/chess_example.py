#!/usr/bin/env python3.8

from stockfish import Stockfish
import pprint

import rospy
from std_msgs.msg import String

# Stockfish documentation: https://pypi.org/project/stockfish/

def update_occupied(step):
    start = step[:2]
    end   = step[2:]

    is_hit = False

    print(occupied)
    print(len(occupied))
    if start not in occupied:
        print("Something wrong happened, %s is not in occupied list." % start)
    else:
        print("Start: %s was removed from occupied list" % start)
        index = occupied.index(start)
        occupied.pop(index)

    if end in occupied:
        print("End: %s is already in occupied list, it's a hit!" % end)
        is_hit = True
    else:
        print("End: %s isn't in the occupied list, adding it" % end)
        occupied.append(end)

    print(occupied)
    print(len(occupied))
    return is_hit
    
# Set up ROS stuff
pub = rospy.Publisher('chess_steps', String, queue_size=1)
rospy.init_node('chess_ai')


stockfish = Stockfish(parameters={"Threads": 2, "Minimum Thinking Time": 30})

steps = []
occupied = ["a1", "b1", "c1", "d1", "e1", "f1", "g1", "h1", 
            "a2", "b2", "c2", "d2", "e2", "f2", "g2", "h2",
            "a7", "b7", "c7", "d7", "e7", "f7", "g7", "h7",
            "a8", "b8", "c8", "d8", "e8", "f8", "g8", "h8"]

pprint.pp(stockfish.get_board_visual())

while 1:
    while 1:
        ret = input("Your move (e.g. e2e4): ")
        if stockfish.is_move_correct(ret):
            print("Valid movement: %s" % ret)
            break
        else:
            print("Invalid movement: %s!" % ret)

    
    steps.append(ret)
    is_hit = update_occupied(ret)
    print("is_hit: %s" % is_hit)
    pub_str = "%s;%s" % (ret, is_hit)
    pub.publish(pub_str)

    #stockfish.set_position(["e2e4", "e7e6"])
    stockfish.set_position(steps)

    input("Press enter to step with the AI!")

    ai_step = stockfish.get_best_move()
    print("AI step: %s" % ai_step)

    steps.append(ai_step)
    is_hit = update_occupied(ai_step)
    print("is_hit: %s" % is_hit)
    pub_str = "%s;%s" % (ai_step, is_hit)
    pub.publish(pub_str)

    stockfish.set_position(steps)

    print("Evaluation:")
    print(stockfish.get_evaluation())

    pprint.pp(stockfish.get_board_visual())


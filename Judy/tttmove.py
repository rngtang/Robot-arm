from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot import PI_PORT, PI_BAUD
from pymycobot.mypalletizer import MyPalletizer
from pymycobot.genre import Coord
from ttt import *
import time
import random

mc = MyCobot("/dev/ttyAMA0", 1000000)

# mc.send_angles([0, 0, 0, 0, 0, 0], 50)
# time.sleep(3)
# mc.set_color(0,0,255) #blue light on


board = [
	[ '_', '_', '_' ],
	[ '_', '_', '_' ],
	[ '_', '_', '_' ]
]
# bestMove = findBestMove(board)
# updateBoard(board, bestMove)

# then make adjustments to move the arm to where it needs to go based on bestMove

# Raul: this simple program plays tic tac toe using onle the terminal
print(board[0])
print(board[1])
print(board[2])

while(isMovesLeft(board)):
    # Takes user input.
    # I guess that in the future this input will be replace with whatever the user does in the screen 
    # Also right now we are trusting that the user's input will have this format n,m 
    print("What is your move input?")
    n = input()

    # Validates the input
    while(not isValid(board, (int(n[0])-1, int(n[2])-1))):
        print("What is your move input?")
        n = input()
    # Updates the board with the user's input
    board[int(n[0])-1][int(n[2])-1] = 'o'
    if(evaluate(board) == -10):
        print("You won!")
        exit()

    print("Now is the robot's turn")
    updateBoard(board)

    if(evaluate(board) == 10):
        print("You lost :(")
        exit()

    # Prints updated board
    print(board[0])
    print(board[1])
    print(board[2])
    
print("Nobody won")
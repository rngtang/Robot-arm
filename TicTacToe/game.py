from ttt import updateBoard
import tkinter as tk
import tkinter.messagebox
from tkinter.tix import COLUMN
import requests
import time

# Creates the main window for the game
window=tk.Tk()
window.title('Tic Tac Toe')
frame=tk.Frame(master=window)
frame.pack(pady=10)
window.resizable(True, True)

# Get the current screen width and height
screen_width = window.winfo_screenwidth()
screen_height = window.winfo_screenheight()

# Print the screen size
print("Screen width:", screen_width)
print("Screen height:", screen_height)

frame1=tk.Frame(master=window,borderwidth=2,relief=tk.SUNKEN,bg='#00539B')
frame1.pack(padx=0,pady=0)

# Creates the buttons grid
button1=tk.Button(master=frame1,text='',width=15,height=3,bg='white',activebackground='white',command=lambda : buttonclick(1))
button1.grid(row=0,column=0,padx=5,pady=5)

button2=tk.Button(master=frame1,text='',width=15,height=3,bg='white',activebackground='white',command=lambda : buttonclick(2))
button2.grid(row=0,column=1,padx=5,pady=5)

button3=tk.Button(master=frame1,text='',width=15,height=3,bg='white',activebackground='white',command=lambda : buttonclick(3))
button3.grid(row=0,column=2,padx=5,pady=5)

button4=tk.Button(master=frame1,text='',width=15,height=3,bg='white',activebackground='white',command=lambda : buttonclick(4))
button4.grid(row=1,column=0,padx=5,pady=5)

button5=tk.Button(master=frame1,text='',width=15,height=3,bg='white',activebackground='white',command=lambda : buttonclick(5))
button5.grid(row=1,column=1,padx=5,pady=5)

button6=tk.Button(master=frame1,text='',width=15,height=3,bg='white',activebackground='white',command=lambda : buttonclick(6))
button6.grid(row=1,column=2,padx=5,pady=5)

button7=tk.Button(master=frame1,text='',width=15,height=3,bg='white',activebackground='white',command=lambda : buttonclick(7))
button7.grid(row=2,column=0,padx=5,pady=5)

button8=tk.Button(master=frame1,text='',width=15,height=3,bg='white',activebackground='white',command=lambda : buttonclick(8))
button8.grid(row=2,column=1,padx=5,pady=5)

button9=tk.Button(master=frame1,text='',width=15,height=3,bg='white',activebackground='white',command=lambda : buttonclick(9))
button9.grid(row=2,column=2,padx=5,pady=5)



framePlayer1=tk.Frame(master=window,border=2,relief=tk.SUNKEN, bg='#00539B')
framePlayer1.pack(padx=20, pady=20)

# Create an inner frame within frame2 and use grid for it
inner_frame1 = tk.Frame(framePlayer1, bg='#00539B')
inner_frame1.grid(row=0, column=0)
labelPlayer1 = tk.Label(inner_frame1, text="You Won!", font=("Arial", 17), bg='#00539B', foreground="white")
labelPlayer1.grid(row=0, column=0, padx=10, pady=(60, 3))
button_restartPlayer1=tk.Button(master=framePlayer1,text="Restart", width=14,height=3,relief=tk.GROOVE,command=lambda: restartbutton())
button_restartPlayer1.grid(row=1,column=0,padx=166, pady=(3, 61))
# button_restartPlayer1.grid_remove()
framePlayer1.pack_forget()


framePlayer2=tk.Frame(master=window,border=2,relief=tk.SUNKEN, bg='#00539B')
framePlayer2.pack(padx=20, pady=20)
# Create an inner frame within frame3 and use grid for it
inner_frame2 = tk.Frame(framePlayer2, bg='#00539B')
inner_frame2.grid(row=0, column=0)
labelPlayer2 = tk.Label(inner_frame2, text="Robot Won!", font=("Arial", 17), bg='#00539B',  foreground="white")
labelPlayer2.grid(row=0, column=0,padx=10,pady=(60, 3))
button_restartPlayer2=tk.Button(master=framePlayer2,text="Restart",width=14,height=3,relief=tk.GROOVE,command=lambda: restartbutton())
button_restartPlayer2.grid(row=1,column=0,padx=166,pady=(3, 61))
# button_restartPlayer1.grid_remove()
framePlayer2.pack_forget()

frameDraw=tk.Frame(master=window,border=2,relief=tk.SUNKEN, bg='#00539B')
frameDraw.pack(padx=20, pady=20)
# Create an inner frame within frame3 and use grid for it
inner_frame3 = tk.Frame(frameDraw, bg='#00539B')
inner_frame3.grid(row=0, column=0)
labelPlayer3 = tk.Label(inner_frame3, text="A Draw!", font=("Arial", 17), bg='#00539B',  foreground="white")
labelPlayer3.grid(row=0, column=0,padx=10,pady=(60, 3))
button_restartPlayer3=tk.Button(master=frameDraw,text="Restart",width=14,height=3,relief=tk.GROOVE,command=lambda: restartbutton())
button_restartPlayer3.grid(row=1,column=0,padx=166,pady=(3, 61))
# button_restartPlayer1.grid_remove()
frameDraw.pack_forget()


# label2=tk.Label(master=frame2,text='Player-1 Turn',bg="skyblue",width=10,height=3,relief=tk.SUNKEN)
label2=tk.Label(master=framePlayer1,text='Player-1 Turn',bg="skyblue",width=0,height=0,relief=tk.SUNKEN)
# label2.grid(row=0,column=2,padx=5)
label2.grid(row=0,column=0,padx=0)
label2.grid_remove()

a=1
b=0
c=0

# Current board and variables that decide the winner are declared
currGame = [["_", "_", "_"], ["_", "_", "_"], ["_", "_", "_"]]
player1 = False
player2 = False
draw = False

# Disables the buttons after the game is over
def disablebutton():
    button1['state']=tk.DISABLED
    button2['state']=tk.DISABLED
    button3['state']=tk.DISABLED
    button4['state']=tk.DISABLED
    button5['state']=tk.DISABLED
    button6['state']=tk.DISABLED
    button7['state']=tk.DISABLED
    button8['state']=tk.DISABLED
    button9['state']=tk.DISABLED

# Resets everything to start a new game
def restartbutton():
    global a,b,c
    global player1, player2, draw
    global currGame 
    currGame = [["_", "_", "_"], ["_", "_", "_"], ["_", "_", "_"]]
    a=1
    b=0
    c=0
    label2['bg']="skyblue"
    label2['text']='Player-1 Turn'
    button1['text']=''
    button1['bg']='white'
    button1['activebackground']='white'
    button2['text']=''
    button2['bg']='white'
    button2['activebackground']='white'
    button3['text']=''
    button3['bg']='white'
    button3['activebackground']='white'
    button4['text']=''
    button4['bg']='white'
    button4['activebackground']='white'
    button5['text']=''
    button5['bg']='white'
    button5['activebackground']='white'
    button6['text']=''
    button6['bg']='white'
    button6['activebackground']='white'
    button7['text']=''
    button7['bg']='white'
    button7['activebackground']='white'
    button8['text']=''
    button8['bg']='white'
    button8['activebackground']='white'
    button9['text']=''
    button9['bg']='white'
    button9['activebackground']='white'
    button1['state']=tk.NORMAL
    button2['state']=tk.NORMAL
    button3['state']=tk.NORMAL
    button4['state']=tk.NORMAL
    button5['state']=tk.NORMAL
    button6['state']=tk.NORMAL
    button7['state']=tk.NORMAL
    button8['state']=tk.NORMAL
    button9['state']=tk.NORMAL
    if player1:
        framePlayer1.pack_forget()
        frame1.pack()
        player1 = not player1
    elif player2:
        framePlayer2.pack_forget()
        frame1.pack()
        player2 = not player2
    elif draw:
        frameDraw.pack_forget()
        frame1.pack()
        draw = not draw

# When one of the buttons is clicked
def buttonclick(x):
    global a,b,c
    global player1, player2, draw
    runScript = a

    # When is the Player's turn
    if(x==1 and a==1 and button1['text']==''):
        currGame[0][0] = "x"
        button1['text']="X"
        button1['bg']="skyblue"
        button1['activebackground']="skyblue"
        label2['bg']="#e8956f"
        label2['text']='Player-2 Turn'
        a = 0
    if(x==2 and a==1 and button2['text']==''):
        currGame[0][1] = "x"
        button2['text']="X"
        button2['bg']="skyblue"
        button2['activebackground']="skyblue"
        label2['bg']="#e8956f"
        label2['text']='Player-2 Turn'
        a = 0
    if(x==3 and a==1 and button3['text']==''):
        currGame[0][2] = "x"
        button3['text']="X"
        button3['bg']="skyblue"
        button3['activebackground']="skyblue"
        label2['bg']="#e8956f"
        label2['text']='Player-2 Turn'
        a = 0
    if(x==4 and a==1 and button4['text']==''):
        currGame[1][0] = "x"
        button4['text']="X"
        button4['bg']="skyblue"
        button4['activebackground']="skyblue"
        label2['bg']="#e8956f"
        label2['text']='Player-2 Turn'
        a = 0
    if(x==5 and a==1 and button5['text']==''):
        currGame[1][1] = "x"
        button5['text']="X"
        button5['bg']="skyblue"
        button5['activebackground']="skyblue"
        label2['bg']="#e8956f"
        label2['text']='Player-2 Turn'
        a = 0
    if(x==6 and a==1 and button6['text']==''):
        currGame[1][2] = "x"
        button6['text']="X"
        button6['bg']="skyblue"
        button6['activebackground']="skyblue"
        label2['bg']="#e8956f"
        label2['text']='Player-2 Turn'
        a = 0
    if(x==7 and a==1 and button7['text']==''):
        currGame[2][0] = "x"
        button7['text']="X"
        button7['bg']="skyblue"
        button7['activebackground']="skyblue"
        label2['bg']="#e8956f"
        label2['text']='Player-2 Turn'
        a = 0
    if(x==8 and a==1 and button8['text']==''):
        currGame[2][1] = "x"
        button8['text']="X"
        button8['bg']="skyblue"
        button8['activebackground']="skyblue"
        label2['bg']="#e8956f"
        label2['text']='Player-2 Turn'
        a = 0
    if(x==9 and a==1 and button9['text']==''):
        currGame[2][2] = "x"
        button9['text']="X"
        button9['bg']="skyblue"
        button9['activebackground']="skyblue"
        label2['bg']="#e8956f"
        label2['text']='Player-2 Turn'
        a = 0

    # When is the robot's turn
    if(x==1 and a==0 and button1['text']==''):
        currGame[0][0] = "o"
        button1['text']='O'
        button1['bg']="#e8956f"
        button1['activebackground']="#e8956f"
        label2['bg']="skyblue"
        label2['text']='Player-1 Turn'
        a=1
    if(x==2 and a==0 and button2['text']==''):
        currGame[0][1] = "o"
        button2['text']='O'
        button2['bg']="#e8956f"
        button2['activebackground']="#e8956f"
        label2['bg']="skyblue"
        label2['text']='Player-1 Turn'
        a=1
    if(x==3 and a==0 and button3['text']==''):
        currGame[0][2] = "o"
        button3['text']='O'
        button3['bg']="#e8956f"
        button3['activebackground']="#e8956f"
        label2['bg']="skyblue"
        label2['text']='Player-1 Turn'
        a=1
    if(x==4 and a==0 and button4['text']==''):
        currGame[1][0] = "o"
        button4['text']='O'
        button4['bg']="#e8956f"
        button4['activebackground']="#e8956f"
        label2['bg']="skyblue"
        label2['text']='Player-1 Turn'
        a=1
    if(x==5 and a==0 and button5['text']==''):
        currGame[1][1] = "o"
        button5['text']='O'
        button5['bg']="#e8956f"
        button5['activebackground']="#e8956f"
        label2['bg']="skyblue"
        label2['text']='Player-1 Turn'
        a=1
    if(x==6 and a==0 and button6['text']==''):
        currGame[1][2] = "o"
        button6['text']='O'
        button6['bg']="#e8956f"
        button6['activebackground']="#e8956f"
        label2['bg']="skyblue"
        label2['text']='Player-1 Turn'
        a=1
    if(x==7 and a==0 and button7['text']==''):
        currGame[2][0] = "o"
        button7['text']='O'
        button7['bg']="#e8956f"
        button7['activebackground']="#e8956f"
        label2['bg']="skyblue"
        label2['text']='Player-1 Turn'
        a=1
    if(x==8 and a==0 and button8['text']==''):
        currGame[2][1] = "o"
        button8['text']='O'
        button8['bg']="#e8956f"
        button8['activebackground']="#e8956f"
        label2['bg']="skyblue"
        label2['text']='Player-1 Turn'
        a=1
    if(x==9 and a==0 and button9['text']==''):
        currGame[2][2] = "o"
        button9['text']='O'
        button9['bg']="#e8956f"
        button9['activebackground']="#e8956f"
        label2['bg']="skyblue"
        label2['text']='Player-1 Turn'
        a=1

    window.update()
    b+=1

    #checking winner
    if(button1['text']=='X' and button2['text']=='X' and button3['text']=='X' or
        button4['text']=='X' and button5['text']=='X' and button6['text']=='X' or
        button7['text']=='X' and button8['text']=='X' and button9['text']=='X' or
        button1['text']=='X' and button4['text']=='X' and button7['text']=='X' or
        button2['text']=='X' and button5['text']=='X' and button8['text']=='X' or
        button3['text']=='X' and button6['text']=='X' and button9['text']=='X' or
        button1['text']=='X' and button5['text']=='X' and button9['text']=='X' or
        button3['text']=='X' and button5['text']=='X' and button7['text']=='X'):
            disablebutton()
            framePlayer1.pack()
            player1 = not player1
            frame1.pack_forget()
            c=1
            # tkinter.messagebox.showinfo("Tic Tac Toe","Winner is player 1")
    elif( button1['text']=='O' and button2['text']=='O' and button3['text']=='O' or
        button4['text']=='O' and button5['text']=='O' and button6['text']=='O' or
        button7['text']=='O' and button8['text']=='O' and button9['text']=='O' or
        button1['text']=='O' and button4['text']=='O' and button7['text']=='O' or
        button2['text']=='O' and button5['text']=='O' and button8['text']=='O' or
        button3['text']=='O' and button6['text']=='O' and button9['text']=='O' or
        button1['text']=='O' and button5['text']=='O' and button9['text']=='O' or
        button3['text']=='O' and button5['text']=='O' and button7['text']=='O'):
            disablebutton()
            framePlayer2.pack()
            player2 = not player2
            frame1.pack_forget()
            c=1
            # tkinter.messagebox.showinfo("Tic Tac Toe","Winner is player 2")
    elif(b==9):
        disablebutton()
        frameDraw.pack()
        draw = not draw
        frame1.pack_forget()
        c=1
        # tkinter.messagebox.showinfo("Tic Tac Toe","Match is Draw.")

    elif runScript == 1:  # run the script to call robot move if the player made their move and game is not over
        try: 
            updateBoard(currGame)
        except: 
            print("FROM GAME: can't call on ttt.py ??")

        time.sleep(1)
    
window.mainloop()

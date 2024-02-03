class StateMachine:
    LOCKED = 0
    UNLOCKED = 1
    def __init__(self):
        self.state = self.LOCKED
        self.coin = False
        self.turn = False
    def process (self):
        if(self.state == self.LOCKED):
            if(self.coin == True):
                self.state = self.UNLOCKED
                self.turn = False
        elif(self.state == self.UNLOCKED):
            if(self.turn == True):
                self.state = self.LOCKED
                self.coin = False
subway = StateMachine()
yes = "y"
i = [1, 2, 3, 4]
for x in i:

    print(subway.state)
    coin = input("add coin? y or n: ")
    turn = input("turn? y or n: ")
    if(turn.__eq__(yes)):
        subway.turn = True
    if(coin.__eq__(yes)):
        subway.coin = True
    subway.process()
    print(subway.state)
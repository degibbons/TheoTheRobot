import time

class Servo:
    def __init__(self,IDnum):
        self.ID = IDnum

        if (IDnum == 1 or IDnum == 2 or IDnum == 3 or IDnum == 4):
            self.Limbnum = 1
        elif (IDnum == 5 or IDnum == 6 or IDnum == 7 or IDnum == 8):
            self.Limbnum = 2
        elif (IDnum == 9 or IDnum == 10 or IDnum == 11 or IDnum == 12):
            self.Limbnum = 3
        elif (IDnum == 13 or IDnum == 14 or IDnum == 15 or IDnum == 16):
            self.Limbnum = 4
        elif (IDnum == 17 or IDnum == 18):
            self.Limbnum = 5
        elif (IDnum == 19 or IDnum == 20 or IDnum == 21 or IDnum == 22):
            self.Limbnum = 6
        elif (IDnum == 23 or IDnum == 24):
            self.Limbnum = 7
        else:
            self.Limbnum = 0
        
        self.Positions = []

        self.Speeds = []

        self.Activated = 1 # Change this to zero if you don't want it to move

        self.HomePosition = []

        self.FirstMovePosition = []

        self.OffsetPercent = []

        self.Phase = [] # 2 for Stance, 1 for transition to stance, -2 for swing, -1 for transition to swing, 0 for other

        self.PresentPosition = []

        self.PresentSpeed = [] # Probably is not needed, GivenSpeed should suffice

        self.GivenPosition = []

        self.GivenSpeed = []

        self.MovementTime = []

        self.PhaseIndex = [] 

        self.StrideIndex = []

        self.PhaseTime = []

        self.StrideTime = []

        self.TotalTime = []


    def InitialSetup(self): # Use SetServoTraits to fill this
        pass

    def ToggleTorque(self):
        pass

    def MoveServo(self):
        pass

    def MoveHome(self):
        pass

    def StartTimer(self):
        pass

    def EndTimer(self):
        pass

    def DisplayTraitValues(self):
        pass

    def RebootServo(self):
        pass

    def ResetServo(self):
        pass

    def CleanUp(self):
        pass

    def __del__(self):
        pass



class Limb:
    def __init__(self,limbnum,servolist):
        self.LimbNumber = limbnum
        self.ServoList = servolist

        self.StrideCount = 0

        self.IsHome = [] # Check all Servo Members to set this value

        self.PhaseTime = [] # Maybe Not Needed? Each Servo also has this trait

        self.StrideTime = []

        self.TotalTime = []

    def MoveLimb(self):
        pass

    def MoveHome(self):
        pass

class Leg(Limb):
    def __init__(self,limbnum,servolist):
        super().__init__(limbnum,servolist)

        self.Phase = [] # Check all Servo Members to set this value


class Neck(Limb):
    pass

class Spine(Limb):
    pass

class Tail(Limb):
    pass



class Body:
    def __init__(self,name):
        self.Name = name

    def MoveBody(self):
        pass

    def MoveHome(self):
        pass


class DataDocument:
    def __init__(self,directory,name,extension):
        self.Directory = directory
        self.Name = name
        self.FileExtension = extension

    def CreateDoc(self):
        pass

    def CheckForDoc(self):
        pass

    def WriteToDoc(self):
        pass

    def CloseDoc(self):
        pass

    def __del__(self):
        pass   


import wpilib
import math

class wheelDrive():
    def __init__(self, motorGroup)
        '''
        pass a tuple with the order being angle, drive, encoder
        '''
        self.angleMotor = wpilib.Talon(motorGroup[0])
        self.speedMotor = wpilib.Spark(motorGroup[1])
        self.pidc = wpilib.controller.PIDController(1, 0, 0, wpilib.AnalogInput(motorGroup[2]), self.angleMotor)

        self.pidc.setIntegratorRange(-1, 1)
        self.pidc.enableContinuousInput()

    def drive(self, speed, angle):
        maxVolts = 4.95
        speedMotor.set(speed)

        setPoint = angle * (maxVolts * 0.5) + (maxVolts + 0.5)
        if setPoint < 0:
            setPoint += maxVolts
        if setPoint > maxVolts:
            setPoint -= maxVolts
        
        self.pidc.setSetpoint(setPoint)

class swerveDrive():
    def __init__(self, lbmg, lfmg, rbmg, rfmg):
        '''
        Expects each group to be an instance of the wheel drive class
        '''
        self.lf = lfmg
        self.lb = lbmg
        self.rb = rbmg
        self.rf = rfmg

        def drive(self, x1, y1, x2):
            l = 23
            w = 20

            r = math.sqrt((l*l) + (w*w))
            y1 *= -1

            a = x1 - x2 * (l / r)
            b = x1 + x2 * (l / r)
            c = y1 -x2 * (w / r)
            d = y1 + x2 * (w / r)

            rbs = math.sqrt((a * a) + (d * d))
            lbs = math.sqrt((a * a) + (c * c))
            rfs = math.sqrt((b * b) + (d * d))
            lfs = math.sqrt((b * b) + (c * c))

            rba = math.atan2(a, d) / Math.p
            lba = math.atan2(a, c) / Math.pi
            rfa = math.atan2(b, d) / Math.pi
            lfa = math.atan2(b, c) / Math.pi
            
            self.lf.drive(lfs, lfa)
            self.lb.drive(lbs, lba)
            self.rf.drive(rfs, rfa)
            self.rb.drive(rbs, rba)


class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        self.lbdm = wpilib.Spark(0)
        self.lbtm = wpilib.Talon(1)
        self.lbe = wpilib.Encoder(0)
        self.lbmg = (self.lbtm, self.lbdm, self.lbe)

        self.lfdm = wpilib.Spark(2)
        self.lftm = wpilib.Talon(3)
        self.lfe = wpilib.Encoder(1)
        self.lfmg = (self.lftm, self.lfdm, self.lfe)

        self.rbdm = wpilib.Spark(4)
        self.rbtm = wpilib.Talon(5)
        self.rbe = wpilib.Encoder(2)
        self.rbmg = (self.rbtm, self.rbdm, self.rbe)

        self.rfdm = wpilib.Spark(6)
        self.rftm = wpilib.Talon(7)
        self.rfe = wpilib.Encoder(3)
        self.rfmg = (self.rftm, self.rfdm, self.rfe)

        mChooser = wpilib.SendableChooser

        lb = wheelDrive(self.lbmg)
        lf = wheelDrive(self.lfmg)
        rb = wheelDrive(self.rbmg)
        rf = wheelDrive(self.rfmg)

        swerveDrive(lb, lf, rb, rf)
        lj = wpilib.Joystick(1)
        swerveDrive.drive(lj.getRawAxis(1), lj.getRawAxis(0), lj.getRawAxis(4))

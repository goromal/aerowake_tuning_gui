import rospy
from rosflight_msgs.msg import Status, Command
from math import pi

class CommandPubSub():
    armed = False
    # status_sub = None
    rf_com_pub = None
    rc_com_pub = None
    timer = None
    mode = 'ROSflight'
    F = 0.0
    x = 0.0
    y = 0.0
    z = 0.0

    @staticmethod
    def initialize():
        # CommandPubSub.status_sub = rospy.Subscriber('status', Status, CommandPubSub.status_callback)
        CommandPubSub.rf_com_pub = rospy.Publisher('command', Command, queue_size=1)
        CommandPubSub.rc_com_pub = rospy.Publisher('high_level_command', Command, queue_size=1)
        CommandPubSub.timer = rospy.Timer(rospy.Duration(1.0/25.0), CommandPubSub.update)

    @staticmethod
    def setArmed(armed):
        CommandPubSub.armed = armed

    @staticmethod
    def getArmed():
        return CommandPubSub.armed

    @staticmethod
    def setMode(mode):
        CommandPubSub.mode = mode

    @staticmethod
    def setF(F):
        CommandPubSub.F = F

    @staticmethod
    def setx(x):
        CommandPubSub.x = x

    @staticmethod
    def sety(y):
        CommandPubSub.y = y

    @staticmethod
    def setz(z):
        CommandPubSub.z = z

    # @staticmethod
    # def status_callback(msg):
    #     CommandPubSub.armed = msg.armed

    @staticmethod
    def update(event):
        command = Command()
        if CommandPubSub.armed:
            command.ignore = 0
        else:
            command.ignore = 8
        if CommandPubSub.mode == 'ROSflight':
            command.mode = Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE
            command.F = CommandPubSub.F
            command.x = pi * CommandPubSub.x / 180.0
            command.y = pi * CommandPubSub.y / 180.0
            command.z = pi * CommandPubSub.z / 180.0
            CommandPubSub.rf_com_pub.publish(command)
        elif CommandPubSub.mode == 'Aerowake':
            command.mode = Command.MODE_PITCH_YVEL_YAW_ALTITUDE
            command.F = CommandPubSub.F
            command.x = pi * CommandPubSub.x / 180.0
            command.y = CommandPubSub.y
            command.z = pi * CommandPubSub.z / 180.0
            CommandPubSub.rc_com_pub.publish(command)
        elif CommandPubSub.mode == 'ROScopterPOS':
            command.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
            command.F = CommandPubSub.F
            command.x = CommandPubSub.x
            command.y = CommandPubSub.y
            command.z = pi * CommandPubSub.z / 180.0
            CommandPubSub.rc_com_pub.publish(command)
        elif CommandPubSub.mode == 'ROScopterVEL':
            command.mode = Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE
            command.F = CommandPubSub.F
            command.x = CommandPubSub.x
            command.y = CommandPubSub.y
            command.z = pi * CommandPubSub.z / 180.0
            CommandPubSub.rc_com_pub.publish(command)
        # if CommandPubSub.armed:
        #     command = Command()
        #     command.ignore = 0
        #     if CommandPubSub.mode == 'ROSflight':
        #         command.mode = Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE
        #         command.F = CommandPubSub.F
        #         command.x = pi * CommandPubSub.x / 180.0
        #         command.y = pi * CommandPubSub.y / 180.0
        #         command.z = pi * CommandPubSub.z / 180.0
        #         CommandPubSub.rf_com_pub.publish(command)
        #     elif CommandPubSub.mode == 'Aerowake':
        #         command.mode = Command.MODE_PITCH_YVEL_YAW_ALTITUDE
        #         command.F = CommandPubSub.F
        #         command.x = pi * CommandPubSub.x / 180.0
        #         command.y = CommandPubSub.y
        #         command.z = pi * CommandPubSub.z / 180.0
        #         CommandPubSub.rc_com_pub.publish(command)
        #     elif CommandPubSub.mode == 'ROScopterPOS':
        #         command.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
        #         command.F = CommandPubSub.F
        #         command.x = CommandPubSub.x
        #         command.y = CommandPubSub.y
        #         command.z = pi * CommandPubSub.z / 180.0
        #         CommandPubSub.rc_com_pub.publish(command)
        #     elif CommandPubSub.mode == 'ROScopterVEL':
        #         command.mode = Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE
        #         command.F = CommandPubSub.F
        #         command.x = CommandPubSub.x
        #         command.y = CommandPubSub.y
        #         command.z = pi * CommandPubSub.z / 180.0
        #         CommandPubSub.rc_com_pub.publish(command)

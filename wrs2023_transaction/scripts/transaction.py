#This code refers to smartcard example
# Author : Muhammad Labiyb Afakh

from smartcard.CardConnectionObserver import ConsoleCardConnectionObserver
from smartcard.CardMonitoring import CardMonitor, CardObserver
from smartcard.util import toHexString
from evdev import UInput, ecodes
import uinput
import rospy
from std_msgs.msg import Int8



# define the class that will handle card events
class MyCardObserver(CardObserver):
    """A card observer that is called when cards are inserted/removed"""

    def __init__(self):
        self.flag_payment = 0

    def update(self, observable, actions):
        (addedcards, removedcards) = actions

        for card in addedcards:
            # print("+Inserted: ", toHexString(card.atr))
            # card.connection = card.createConnection()
            # card.connection.connect()
            self.flag_payment = 1
            # You may add code here to handle the card when it's inserted

        for card in removedcards:
            print("-Removed: ", toHexString(card.atr))
            # You may add code here to handle the card when it's removed
            if(self.flag_payment == 1):
                self.flag_payment = 100
            else:
                self.flag_payment = 0

if __name__ == '__main__' :
    rospy.init_node('transaction', anonymous=True)
    pub =  rospy.Publisher('nfc_checker', Int8, queue_size=10)
    rate = rospy.Rate(100)
    # Create an instance of the card monitor and observer
    monitor = CardMonitor()
    observer = MyCardObserver()

    # Register the observer with the monitor
    monitor.addObserver(observer)

    try:
        print("Monitoring smartcard, press Ctrl-C to quit.")
        while True:
            print("flag_payment: ", observer.flag_payment)
            pub.publish(observer.flag_payment)
            rate.sleep()
            # observer.flag_payment = 0
    except KeyboardInterrupt:
        # Unregister the observer and stop monitoring when the script is interrupted
        monitor.deleteObserver(observer)

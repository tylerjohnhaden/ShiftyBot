"""A Basic Rospy Node

Main control loop runs until shutdown or `stop()` is called.
"""

import rospy

ROSBOT_IMAGE = """
                                                               
                       &&&&&&&&..........                        
                     &@@@@&&%@&@@@@&@@&&&&&&&%%%###             
                      &&&&&&&&&@&@@&&&&%&&&&&, #%%%      &%      
                             &&@@@@@@&&@@&@@#/%          &%      
                              &&&&@@@&&&&&/,, ,          &       
                                /                       %&       
                                .        .              @%       
                      %%&&%%%##%%%%%(                  @%       
                      &%%%%#%%#(#%%%%  &&             %@        
                   ,.,&@&&&&@@@&&%%&%&...,,      .   %%         
      /(//////*,,,,,,,&@@@@@@@@&&&&%(.(...*,,,,,,,,,,*/*%%#     
    ((///////////////////////(((@////,,,    ,.  ..,,.....       
    &((((((((((((/////////////////,...         ..*&%@&&%%%&#    
   &@@@ .((((((((((((((((((((////.,&@&&%&#&( ../&&&@&%%& @. &%  
   &@@@  &@&@@   .,,(((((((((//,(@@@&&%#%%  &&%&&&@%%&   .,& &  
   &@@@&...           &@@@&&  *%@&&&%%&   *%@ &%@&@@@, %./#& %# 
    @@&@@@&%............     .(@&&@@%@,   ..%* &@@@@&.   %%& &  
     *%@@@@@@@&%%%%%%%%,......*&&&&@@&  *  &&,@%@@@@@&  && .&   
               .,*/(##%%%%%%%%%%&&@@&&       &&((#@@@@@&&&%*    
                           .,/(##%&&&&&&   %&,.                 
                                  .*#@@@&(.

"""


class Bot(object):
    def __init__(self):
        rospy.init_node('shifty', anonymous=True, log_level=rospy.INFO)

        self.dt = .01  # todo: see if calibration changes behavior
        self.hz = int(1 / self.dt)
        self.rate = rospy.Rate(self.hz)

        self.running = True

    def step(self):
        """A single (relatively atomic) method to set the velocity
        based on all current environment and control inputs.
        """
        pass

    def stop(self):
        self.running = False

    def run(self):
        rospy.loginfo(ROSBOT_IMAGE)
        rospy.loginfo('\n\n\n\nRunning Control Loop ...\n\n\n\n')

        while not rospy.is_shutdown() and self.running:
            self.step()
            self.rate.sleep()

        rospy.loginfo('\n\n\n\n... Control Loop Closed\n\n\n\n')

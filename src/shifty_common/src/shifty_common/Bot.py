"""A Basic Rospy Node

Main control loop runs until shutdown or `exit()` is called.
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

                     [[[[    SHIFTY    ]]]]
"""


class Bot(object):
    def __init__(self, name='bot'):
        rospy.init_node(name, anonymous=True, log_level=rospy.INFO)

        self.dt = 0.1  # todo: see if calibration changes behavior
        self.hz = int(1 / self.dt)
        self.rate = rospy.Rate(self.hz)

        self.running = True

    def run(self):
        rospy.loginfo(ROSBOT_IMAGE)
        rospy.loginfo('\nRunning Control Loop ...\n\n\n\n')

        while not rospy.is_shutdown() and self.running:
            self.step()
            self.post_step()
            self.rate.sleep()

        rospy.loginfo('\n\n\n\n... Control Loop Closed\n\n\n\n')

    def step(self):
        """A single (relatively atomic) method to set the velocity
        based on all current environment and control inputs.
        """
        pass

    def post_step(self):
        """Call needed at the end of every step and should be
        unrelated to setting velocity.
        """
        pass

    def exit(self):
        self.running = False

    def set_dt(self, dt):
        self.dt = dt
        self.hz = int(1 / self.dt)
        self.rate = rospy.Rate(self.hz)

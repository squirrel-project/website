#!/usr/bin/env python
import roslib
roslib.load_manifest( 'squirrel_website' )
import rospy
import debug_server

debug_server.logwarn = rospy.logwarn
debug_server.loginfo = rospy.loginfo
debug_server.logerr  = rospy.logerr

if __name__ == '__main__':
    rospy.init_node( 'debug_server' )
    port = int( rospy.get_param( '~port' ))
    debug_server.start( port )

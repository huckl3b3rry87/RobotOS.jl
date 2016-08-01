using PyCall

const rospy = pyimport("rospy")
const gmsgs = pyimport("geometry_msgs.msg")

rospy[:init_node]("jltest", anonymous=true)
jl_cb(msg::PyObject) = println("message: ", msg)

const ros_sub = rospy[:Subscriber]("points", gmsgs["Point"], jl_cb)
rospy[:sleep](10.0)

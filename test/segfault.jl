using Base.Test
using RobotOS
using Compat
RobotOS.debug(true)

init_node("jltest", anonymous=true)

@rosimport geometry_msgs.msg: PoseStamped, Vector3
rostypegen()

using geometry_msgs.msg

function pose_cb(msg)
    println("message: ", msg)
end

const ros_pub = Publisher("vectors", Vector3; queue_size = 10)
const ros_sub = Subscriber("poses", PoseStamped, pose_cb; queue_size = 10)

rossleep(Duration(1.0))
publish(ros_pub, Vector3(10,0,0))
rossleep(Duration(1.0))
